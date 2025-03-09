#pragma once

#include "pisar/driveunit_interface/codec.h"
#include "pisar/driveunit_interface/interface.h"

#include "pisar/driveunit/async_serial_uart.h"

#include "pisar/driveunit/rtos/stream_buffer.h"
#include "pisar/driveunit/logging.h"

#include "pisar/utilities/fixed_vector.h"

#include <Arduino.h>
#include <SPI.h>
#include <SPISlave.h>

#include <hardware/spi.h>
#include <hardware/gpio.h>
#include <hardware/structs/iobank0.h>
#include <hardware/irq.h>

#include <FreeRTOS.h>
#include <semphr.h>

namespace pisar::driveunit {

/**
 * @brief UART-based communication interface for synchronized request-response handling.
 * @tparam THandler The message handler type.
 */
template <typename THandler>
class TransportInterface {
private:
    AsyncSerialUart& m_uart;                                ///< UART instance.
    THandler& m_handler;                                    ///< Message handler.
    TaskHandle_t m_task_handle;                             ///< FreeRTOS task handle for processing.

    /// @brief Packet decoder.
    driveunit_interface::RequestDecoder<5> m_decoder;

    /// @brief Input memory buffer for incoming requests. Store max 2 at a time before overflow.
    StaticStreamBuffer<2 * driveunit_interface::RequestEncoder::kMaxEncodedPacketSize> m_recv_buffer;

     /// @brief Buffer for outgoing responses. +1 for size byte
    FixedVector<std::byte, driveunit_interface::ResponseEncoder::kMaxEncodedPacketSize> m_send_buffer;

public:
    /**
     * @brief Constructs a UART-based interface.
     * @param uart Reference to the AsyncSerialUart instance.
     * @param handler Reference to the message handler.
     */
    explicit TransportInterface(AsyncSerialUart& uart, THandler& handler)
        : m_uart(uart), m_handler(handler), m_task_handle(nullptr)
    {
        // Set up UART callback for incoming data
        m_uart.setIRQCallback([this](const std::span<const std::byte> buffer) { onDataReceived(buffer); });
    }

    /**
     * @brief Initializes UART and starts the processing task.
     * @param task_priority Priority of the processing task.
     */
    void initialize(UBaseType_t task_priority)
    {
        if (task_priority < 0 || task_priority > configMAX_PRIORITIES)
        {
            PISAR_LOG_ERROR("Task priority %u is out of range", task_priority);
            return; // TODO ERROR CODE
        }

        // Spawn the processing task
        if (xTaskCreate(taskEntry, "transport_interface_task", 2048, this, task_priority, &m_task_handle) != pdPASS)
        {
            PISAR_LOG_ERROR("Failed to create SPI processing task");
            return;
        }

        // Initialize UART
        m_uart.begin(driveunit_interface::kUartSpeed);
    }

private:
    /**
     * @brief FreeRTOS task entry function.
     * @param param Pointer to the TransportInterface instance.
     */
    static void taskEntry(void* param)
    {
        reinterpret_cast<TransportInterface*>(param)->taskLoop();
    }

    /**
     * @brief Main loop for the processing task.
     */
    void taskLoop()
    {
        while (true)
        {
            while (!m_decoder.packetsAvailable())
            {
                std::array<std::byte, driveunit_interface::RequestEncoder::kMaxEncodedPacketSize> buffer;

                // Wait until data reception
                const size_t num_bytes = readData(std::span(buffer));
                if (num_bytes == 0)
                {
                    continue;
                }

                // Decode the received message
                m_decoder.submit(std::span(buffer.data(), num_bytes));

                if (m_decoder.errorCount())
                {
                    PISAR_LOG_ERROR("%d errors while decoding", m_decoder.errorCount());
                    m_decoder.clearErrors();
                }
            }

            auto packet = m_decoder.query();
            if (!packet)
            {
                PISAR_LOG_ERROR("Failed to query message");
                continue;
            }

            processMessage(packet.value());
        }
    }

    /**
     * @brief Processes a received message.
     */
    void processMessage(const driveunit_interface::Request& message)
    {
        // Handle the message
        const auto response = m_handler.handleMessage(message);

        if (!response)
        {
            PISAR_LOG_WARN("No response");
            return;
        }

        const auto bytes_sent = sendResponse(response.value());
        // Encode the response and send it
        if (!bytes_sent)
        {
            PISAR_LOG_ERROR("Failed to send response!");
        }

        PISAR_LOG_DEBUG("Sent %u bytes", bytes_sent.value());
    }

    /**
     * @brief Reads data from the UART buffer.
     * @param buffer Buffer to store received data.
     * @return Number of bytes read.
     */
    inline size_t readData(const std::span<std::byte> buffer)
    {
        return m_recv_buffer.receive(buffer);
    }

    /**
     * @brief Encode and setup the response for transmitting.
     *
     * @param response_msg The response message to send.
     * @return true if successful otherwise false.
     */
    inline std::optional<size_t> sendResponse(driveunit_interface::Response response_msg)
    {
        m_send_buffer.resize(m_send_buffer.capacity());

        driveunit_interface::ResponseEncoder encoder;
        const auto encoded_buffer = encoder.encode(response_msg, std::span(m_send_buffer.data(), m_send_buffer.size()));
        if (!encoded_buffer)
        {
            PISAR_LOG_ERROR("Failed to encode response");
            return std::nullopt;
        }

        m_uart.write(reinterpret_cast<const uint8_t*>(encoded_buffer.value().data()), encoded_buffer.value().size());

        return encoded_buffer.value().size();
    }

    /**
     * @brief Handles received UART data (called from IRQ).
     * @param byte The received byte.
     */
    void onDataReceived(const std::span<const std::byte> buffer)
    {
        BaseType_t higher_priority_task_woken = pdFALSE;
        m_recv_buffer.sendIsr(buffer, &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
};

} // namespace pisar::driveunit
