#pragma once

#include "pisar/driveunit_interface/codec.h"
#include "pisar/driveunit_interface/interface.h"

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
 * @brief SPI Slave communication interface with FreeRTOS task for processing.
 * @tparam tkRecvBufferSize size of the SPI receive buffer.
 * @tparam tkSendBufferSize size of the SPI send buffer.
 * @tparam THandler The message handler type.
 */
template <typename THandler>
class TransportInterface {
private:
    SPISlaveClass& m_spi;                                   ///< SPI slave instance.
    THandler& m_handler;                                    ///< Message handler.
    TaskHandle_t m_task_handle;                             ///< FreeRTOS task handle for processing.

    /// @brief Packet decoder.
    driveunit_interface::RequestDecoder<5> m_decoder;

    /// @brief Input memory buffer for incoming requests. Store max 2 at a time before overflow.
    StaticStreamBuffer<2 * driveunit_interface::RequestEncoder::kMaxEncodedPacketSize> m_recv_buffer;

     /// @brief Buffer for outgoing responses. +1 for size byte
    FixedVector<std::byte, driveunit_interface::ResponseEncoder::kMaxEncodedPacketSize + 1> m_send_buffer;
    uint8_t m_default_send_byte;

    size_t m_response_bytes_pending;

    enum class SendState : uint8_t {
        Initiated = 0,
        Pending,
        Completed
    };

    volatile SendState m_send_state;

public:
    /**
     * @brief Constructs an SPI slave interface.
     * @param spi Reference to the SPI slave instance.
     * @param handler Reference to the message handler.
     */
    explicit TransportInterface(SPISlaveClass& spi, THandler& handler)
        : m_spi(spi), m_handler(handler), m_task_handle(nullptr), m_default_send_byte(0), m_response_bytes_pending(0), m_send_state(SendState::Completed)
    {
    }

    /**
     * @brief Initializes SPI as a slave device.
     *
     * @param task_priority Priority of the processing task
     */
    void initialize(UBaseType_t task_priority)
    {
        if (task_priority < 0 || task_priority > configMAX_PRIORITIES)
        {
            PISAR_LOG_ERROR("Task priority %u is out of range", task_priority);
            return; // TODO ERROR CODE
        }

        // Spawn the processing task
        if (xTaskCreate(taskEntry, "SPI_Processing", 2048, this, task_priority, &m_task_handle) != pdPASS)
        {
            PISAR_LOG_ERROR("Failed to create SPI processing task");
            return;
        }

        // Set default response.
        onDataSent();

        // Capture `this` in lambda to avoid needing a singleton
        m_spi.onDataRecv([this](uint8_t* data, size_t len) { this->onDataReceived(data, len); });
        m_spi.onDataSent([this]() { this->onDataSent(); });

        // Start SPI slave
        m_spi.begin(SPISettings(driveunit_interface::kSpiSpeed, MSBFIRST, SPI_MODE1));
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

                // Wait until SPI ISR signals data reception
                const size_t num_bytes = readData(std::span(buffer));

                for (int i = 0; i < num_bytes; ++i)
                {
                    Serial.printf("%x ", buffer[i]);
                }
                Serial.println();

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
        m_response_bytes_pending = bytes_sent.value() + 9;
    }

    /**
     * @brief Called when SPI data is received.
     * @param data Pointer to received data.
     * @param len Length of received data.
     */
    void onDataReceived(uint8_t* data, size_t len)
    {
        if (m_recv_buffer.spaceAvailable() < len)
        {
            // Not enouph space
            return;
        }

        BaseType_t higher_priority_task_woken = pdFALSE;
        m_recv_buffer.sendIsr(std::span(reinterpret_cast<std::byte*>(data), len), &higher_priority_task_woken);

        //portYIELD_FROM_ISR(higher_priority_task_woken);
    }

    /**
     * @brief Called when the master has finished reading from SPI.
     */
    void onDataSent()
    {
        if (m_send_state == SendState::Initiated)
        {
            Serial.printf("Initiated data transfer for %u bytes: %u\n", m_send_buffer.size(), m_default_send_byte);
            m_spi.setData(reinterpret_cast<uint8_t*>(m_send_buffer.data()), m_send_buffer.size());
            m_send_state = SendState::Pending;
            return;
        }

        if (m_send_state == SendState::Pending)
        {
            Serial.printf("Completed data transfer\n");
            m_send_state = SendState::Completed;
        }

        // Only send default byte if nothing is pending
        m_default_send_byte++;
        m_spi.setData(&m_default_send_byte, 1);
        Serial.printf("Sent %u\n", m_default_send_byte);
    }

    inline size_t readData(const std::span<std::byte> buffer)
    {
        if (m_response_bytes_pending)
        {
            size_t num_bytes = m_recv_buffer.receive(std::span(buffer.data(), m_response_bytes_pending));
            m_response_bytes_pending = 0;
        }

        return m_recv_buffer.receive(std::span(buffer));
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
        const auto encoded_buffer = encoder.encode(response_msg, std::span(m_send_buffer.data(), m_send_buffer.size()).subspan(1));
        if (!encoded_buffer)
        {
            PISAR_LOG_ERROR("Failed to encode response");
            return std::nullopt;
        }

        // First byte will contain the size of the packet
        m_send_buffer[0] = std::byte(static_cast<uint8_t>(encoded_buffer.value().size()));
        m_send_buffer.resize(encoded_buffer.value().size() + 1);

        portENTER_CRITICAL();
        m_send_state = SendState::Initiated;
        portEXIT_CRITICAL();

        return m_send_buffer.size();
    }
};

} // namespace pisar::driveunit
