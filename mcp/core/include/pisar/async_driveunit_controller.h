#pragma once

#include "pisar/driveunit_transport.h"
#include "pisar/driveunit_controller.h"
#include "pisar/concurrent_queue.h"

#include <unordered_map>
#include <thread>
#include <atomic>
#include <future>
#include <chrono>

namespace pisar::mcp {

/**
 * @brief Handles asynchronous Driveunit communication while allowing synchronous calls.
 */
class AsyncDriveunitController {
private:
    struct RequestEntry
    {
        uint32_t request_id;
        driveunit_interface::Request data;
    };

    std::reference_wrapper<DriveunitTransport> m_transport;

    std::atomic<uint32_t> m_next_request_id;
    ConcurrentQueue<RequestEntry> m_request_queue;

    using ResponseResult = rd::expected<driveunit_interface::Response, std::error_code>;
    std::unordered_map<uint32_t, ResponseResult> m_response_map;
    std::mutex m_response_mutex;

    std::thread m_worker_thread;
    std::atomic<bool> m_running;

public:
    /**
     * @brief Constructs the asynchronous Driveunit controller.
     * @param transport The driveunit transport for communication.
     */
    explicit AsyncDriveunitController(DriveunitTransport& transport)
        : m_transport(transport), m_next_request_id(1), m_running(true) {}

    /**
     * @brief Destructor to ensure cleanup.
     */
    ~AsyncDriveunitController()
    {
        stop();
    }

    /**
     * @brief Starts the worker thread.
     * @return True if successfully started, false otherwise.
     */
    inline bool start()
    {
        close();
        m_running.store(true, std::memory_order_release);
        m_worker_thread = std::thread(&AsyncDriveunitController::processRequests, this);
        return true;
    }

    /**
     * @brief Stops the worker thread.
     */
    inline void stop()
    {
        m_running.store(false, std::memory_order_release);

        if (m_worker_thread.joinable())
        {
            m_worker_thread.join();
        }

        m_next_request_id = 1;
        m_request_queue.clear();
        m_response_map.clear();
    }

    /**
     * @brief Sends a request asynchronously.
     * @param request The request data.
     * @return uint32_t The assigned request ID.
     */
    uint32_t sendRequest(const driveunit_interface::Request& request);

    /**
     * @brief Retrieves a response for a given request ID.
     * @param request_id The request ID to query.
     * @return `std::optional<rd::expected<TResponse, std::error_code>>` containing the response or error.
     * @return std::nullopt if request has not been submitted or not processed yet.
     */
    std::optional<rd::expected<driveunit_interface::Response, std::error_code>> getResponse(uint32_t request_id);

    /**
     * @brief Sends a request asynchronously.
     * @tparam TRequest Type of request.
     * @param request The request data.
     * @return uint32_t The assigned request ID.
     */
    template <typename TRequest>
    inline uint32_t sendRequest(const TRequest& request)
    {
        return sendRequest(driveunit_interface::Request(request));
    }

    /**
     * @brief Retrieves a response for a given request ID.
     * @tparam TResponse Expected response type.
     * @param request_id The request ID to query.
     * @return `std::optional<rd::expected<TResponse, std::error_code>>` containing the response or error.
     * @return std::nullopt if request has not been submitted or not processed yet.
     */
    template <typename TResponse>
    inline std::optional<rd::expected<TResponse, std::error_code>> getResponse(uint32_t request_id)
    {
        // Use "this->" to explicitly call non-template overload.
        auto result = this->getResponse(request_id);
        if (!result)
        {
            return std::nullopt;
        }

        auto response = std::move(result.value());

        if (response.has_value() == false)
        {
            return rd::unexpected(response.error());
        }

        if (auto valid_response = std::get_if<TResponse>(&response.value()))
        {
            return rd::expected(*valid_response);
        }

        return rd::unexpected(make_error_code(DriveunitTransport::TransportError::kInvalidResponse));
    }

    /**
     * @brief Sends a command synchronously (blocking).
     * @param request The request data.
     * @return The request response or error.
     */
    [[nodiscard]] inline rd::expected<driveunit_interface::Response, std::error_code>
    sendRequest(const driveunit_interface::Request& request)
    {
        return m_transport.get().sendRequest(request);
    }

    /**
     * @brief Sends a command synchronously (blocking).
     * @tparam TRequest Type of request.
     * @tparam TResponse Expected response type.
     * @param request The request data.
     * @return The request response or error.
     */
    template <typename TRequest, typename TResponse>
    [[nodiscard]] inline rd::expected<TResponse, std::error_code>
    sendRequestSync(const TRequest& request)
    {
        return m_transport.get().template sendRequest<TRequest, TResponse>(request);
    }

private:
    /**
     * @brief Worker thread that processes requests and stores responses.
     */
    void processRequests()
    {
        while (m_running.load(std::memory_order_acquire))
        {
            // Timeout so we can check the running flag periodically
            auto request_entry = m_request_queue.pop(std::chrono::milliseconds(10));
            if (!request_entry.has_value())
            {
                continue;
            }

            const uint32_t request_id = request_entry->request_id;
            const driveunit_interface::Request& request_data = request_entry->request;

            // Send request and wait for response
            const auto response = m_transport.get().sendRequest(request_data);

            // Store the response in the map
            std::scoped_lock lock(m_response_mutex);
            m_response_map[request_id] = std::move(response);
        }
    }
};

} // namespace pisar::mcp
