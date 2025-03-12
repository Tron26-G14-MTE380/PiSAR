#pragma once

#include "FreeRTOS.h"
#include <stream_buffer.h>

#include <optional>
#include <span>
#include <cstdint>

namespace pisar::driveunit {

/// @brief Thread-safe wrapper for FreeRTOS xStreamBuffer.
template<size_t tkBufferSize>
class StaticStreamBuffer {
private:
    std::array<std::byte, tkBufferSize> m_mem;
    StaticStreamBuffer_t m_buffer;
    StreamBufferHandle_t m_handle;

public:
    /**
     * @brief Constructs a stream buffer with the given size.
     * @param trigger_level Minimum bytes required to trigger a read operation.
     */
    explicit inline StaticStreamBuffer(size_t trigger_level = 1)
        : m_handle(
            xStreamBufferCreateStatic(tkBufferSize, trigger_level, reinterpret_cast<uint8_t*>(m_mem.data()), &m_buffer)
        ) {}

    /// @brief Deleted copy constructor to prevent accidental duplication.
    StaticStreamBuffer(const StaticStreamBuffer&) = delete;

    /// @brief Deleted copy assignment operator.
    StaticStreamBuffer& operator=(const StaticStreamBuffer&) = delete;

    /// @brief Move constructor.
   inline StaticStreamBuffer(StaticStreamBuffer&& other) noexcept
        : m_handle(other.m_handle) {
        other.m_handle = nullptr;
    }

    /// @brief Move assignment operator.
    inline StaticStreamBuffer& operator=(StaticStreamBuffer&& other) noexcept {
        if (this != &other) {
            destroy();
            m_handle = other.m_handle;
            other.m_handle = nullptr;
        }
        return *this;
    }

    /// @brief Destructor.
    inline ~StaticStreamBuffer()
    {
        destroy();
    }

    /**
     * @brief Sends data to the stream buffer.
     * @param data Data to send.
     * @param timeout_ticks Maximum ticks to wait for space.
     * @return Number of bytes successfully written.
     */
    [[nodiscard]] inline size_t send(const std::span<const std::byte> data, TickType_t timeout_ticks = portMAX_DELAY)
    {
        if (!m_handle) return 0;
        return xStreamBufferSend(m_handle, data.data(), data.size(), timeout_ticks);
    }

    /**
     * @brief Sends data to the stream buffer.
     * @param data Data to send.
     * @param p_higher_priority_task_woken See pxHigherPriorityTaskWoken in freertos docs.
     * @return Number of bytes successfully written.
     */
    [[nodiscard]] inline size_t sendIsr(const std::span<const std::byte> data, BaseType_t* p_higher_priority_task_woken)
    {
        if (!m_handle) return 0;
        return xStreamBufferSendFromISR(m_handle, data.data(), data.size(), p_higher_priority_task_woken);
    }

    /**
     * @brief Receives data from the stream buffer.
     * @param buffer Buffer to store received data.
     * @param timeout_ticks Maximum ticks to wait for data.
     * @return Number of bytes read.
     */
    [[nodiscard]] inline size_t receive(const std::span<std::byte> buffer, TickType_t timeout_ticks = portMAX_DELAY)
    {
        if (!m_handle) return 0;
        return xStreamBufferReceive(m_handle, buffer.data(), buffer.size(), timeout_ticks);
    }

    /**
     * @brief Receives data from the stream buffer.
     * @param buffer Buffer to store received data.
     * @param p_higher_priority_task_woken See pxHigherPriorityTaskWoken in freertos docs.
     * @return Number of bytes read.
     */
    [[nodiscard]] inline size_t receiveIsr(const std::span<std::byte> buffer, BaseType_t* p_higher_priority_task_woken)
    {
        if (!m_handle) return 0;
        return xStreamBufferReceiveFromISR(m_handle, buffer.data(), buffer.size(), p_higher_priority_task_woken);
    }

    /// @brief Resets the stream buffer.
    /// @return True if successfully reset.
    [[nodiscard]] inline bool reset()
    {
        if (!m_handle) return false;
        return xStreamBufferReset(m_handle) == pdPASS;
    }

    /// @brief Gets the number of bytes available for reading.
    [[nodiscard]] inline size_t bytesAvailable() const
    {
        return m_handle ? xStreamBufferBytesAvailable(m_handle) : 0;
    }

    /// @brief Gets the number of bytes that can be written to the buffer.
    [[nodiscard]] inline size_t spaceAvailable() const
    {
        return m_handle ? xStreamBufferSpacesAvailable(m_handle) : 0;
    }

    /// @brief Checks if the stream buffer is valid.
    [[nodiscard]] inline bool isValid() const
    {
        return m_handle != nullptr;
    }

private:
    /// @brief Destroys the stream buffer if it exists.
    inline void destroy()
    {
        if (m_handle)
        {
            vStreamBufferDelete(m_handle);
            m_handle = nullptr;
        }
    }
};

}