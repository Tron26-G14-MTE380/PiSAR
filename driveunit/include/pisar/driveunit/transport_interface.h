#pragma once

#include "pisar/driveunit/logging.h"

#include "pisar/driveunit_interface/codec.h"
#include "pisar/driveunit_interface/interface.h"

#include <Arduino.h>
#include <SPI.h>
#include <SPISlave.h>

#include "FreeRTOS.h"
#include "semphr.h"

namespace pisar::driveunit {

/**
 * @brief SPI Slave communication interface with FreeRTOS task for processing.
 * @tparam tkRecvBufferSize size of the SPI receive buffer.
 * @tparam tkSendBufferSize size of the SPI send buffer.
 * @tparam THandler The message handler type.
 */
template <size_t tkRecvBufferSize, size_t tkSendBufferSize, typename THandler>
class TransportInterface {
private:
    SPISlaveClass& m_spi;                                   ///< SPI slave instance.
    THandler& m_handler;                                    ///< Message handler.
    UBaseType_t m_task_priority;                            ///< Internal task priority.
    TaskHandle_t m_task_handle;                             ///< FreeRTOS task handle for processing.
    BinarySemaphore m_data_ready_semaphore;                 ///< Semaphore for signaling data reception.
    BinarySemaphore m_response_pending;                     ///< Semaphore to signal when a response message is pending.

    std::array<std::byte, tkRecvBufferSize> m_recv_buffer;  ///< Buffer for incoming messages.
    std::array<std::byte, tkSendBufferSize> m_send_buffer;  ///< Buffer for outgoing responses.

public:
    /**
     * @brief Constructs an SPI slave interface.
     * @param spi Reference to the SPI slave instance.
     * @param handler Reference to the message handler.
     * @param task_priority Priority of the processing task.
     */
    explicit TransportInterface(SPISlaveClass& spi, THandler& handler, UBaseType_t task_priority)
        : m_spi(spi), m_handler(handler), m_task_priority(task_priority), m_task_handle(nullptr)
    {
        m_response_pending.unlock();
    }

    /**
     * @brief Initializes SPI as a slave device.
     */
    void initialize()
    {
        // Spawn the processing task
        if (xTaskCreate(taskEntry, "SPI_Processing", 2048, this, m_task_priority, &m_task_handle) != pdPASS)
        {
            PISAR_LOG_ERROR("Failed to create SPI processing task");
            return;
        }

        // Capture `this` in lambda to avoid needing a singleton
        m_spi.onDataRecv([this](uint8_t* data, size_t len) { this->onDataReceived(data, len); });
        m_spi.onDataSent([this]() { this->onDataSent(); });

        // Set default response.
        transmit(driveunit_interface::DefaultResponse(), false);

        // Start SPI slave
        m_spi.begin(SPISettings(driveunit_interface::kSpiSpeed, MSBFIRST, SPI_MODE0));

        PISAR_LOG_INFO("SPI Slave Initialized");
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
            // Wait until SPI ISR signals data reception
            if (m_data_ready_semaphore.lock())
            {
                processMessage();
            }
        }
    }

    /**
     * @brief Processes a received message.
     */
    void processMessage()
    {
        // Decode the received message
        driveunit_interface::PacketDecoder<driveunit_interface::Request> decoder;
        auto packet = decoder.decode(std::span<const std::byte>(m_recv_buffer));

        if (!packet.has_value())
        {
            PISAR_LOG_ERROR("Failed to decode SPI message.");
            return;
        }

        // Handle the message
        const auto response  = m_handler.handleMessage(packet.value());

        // Send the response
        if (response)
        {
            transmit(response.value(), true);
        }
    }

    /**
     * @brief Called when SPI data is received.
     * @param data Pointer to received data.
     * @param len Length of received data.
     */
    void onDataReceived(uint8_t* data, size_t len)
    {
        if (len > tkRecvBufferSize)
        {
            return;
        }

        // If we are waiting for response to get clocked out then we just ignore the transaction
        if (m_response_pending.countIsr() == 0) // 0 means not available
        {
            return;  // Ignore incoming data if the previous response is still being read
        }


        memcpy(m_recv_buffer.data(), data, len);
        m_data_ready_semaphore.unlockIsr();
    }

    /**
     * @brief Called when the master has finished reading from SPI.
     */
    void onDataSent()
    {
        // We automatically clear the response pending flag when data is sent.
        m_response_pending.unlockIsr();

        // Set default response.
        transmitIsr(driveunit_interface::DefaultResponse(), false);
    }

    inline void transmit(driveunit_interface::Response response_msg, bool ignore_messages_until_sent)
    {
        driveunit_interface::PacketEncoder<driveunit_interface::Response> encoder;

        // Encode directly into m_send_buffer at offset 1 (leaving space for size byte)
        auto encoded = encoder.encode(response_msg, std::span(m_send_buffer).subspan(1));
        if (encoded)
        {
            if (ignore_messages_until_sent)
            {
                if (m_response_pending.lock() == false)
                {
                    return; // Ignore transmission if the last response hasn't been fully read
                }
            }
            else
            {
                if (m_response_pending.count() == 0)
                {
                    return; // Ignore transmission if the last response hasn't been fully read
                }
            }

            m_send_buffer[0] = static_cast<std::byte>(encoded->size()); // Store size in first byte
            m_spi.setData(reinterpret_cast<uint8_t*>(m_send_buffer.data()), encoded.value().size() + 1);
        }
        else
        {
            PISAR_LOG_ERROR("Failed to encode response");
        }
    }

    inline void transmitIsr(driveunit_interface::Response response_msg, bool ignore_messages_until_sent)
    {
        driveunit_interface::PacketEncoder<driveunit_interface::Response> encoder;

        // Encode directly into m_send_buffer at offset 1 (leaving space for size byte)
        auto encoded = encoder.encode(response_msg, std::span(m_send_buffer).subspan(1));
        if (encoded)
        {
            if (ignore_messages_until_sent)
            {
                if (m_response_pending.lockIsr() == false)
                {
                    return; // Ignore transmission if the last response hasn't been fully read
                }
            }
            else
            {
                if (m_response_pending.countIsr() == 0)
                {
                    return; // Ignore transmission if the last response hasn't been fully read
                }
            }

            m_send_buffer[0] = static_cast<std::byte>(encoded->size()); // Store size in first byte
            m_spi.setData(reinterpret_cast<uint8_t*>(m_send_buffer.data()), encoded.value().size() + 1);
        }
        else
        {
            PISAR_LOG_ERROR("Failed to encode response");
        }
    }
};

} // namespace pisar::driveunit
