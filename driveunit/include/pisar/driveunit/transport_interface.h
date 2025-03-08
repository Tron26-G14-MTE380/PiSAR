#pragma once

#include "pisar/driveunit_interface/codec.h"
#include "pisar/driveunit_interface/interface.h"

#include "pisar/driveunit/rtos/stream_buffer.h"
#include "pisar/driveunit/logging.h"

#include <Arduino.h>
#include <SPI.h>
#include <SPISlave.h>

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
    BinarySemaphore m_response_pending;                     ///< Semaphore to signal when a response message is pending.

    /// @brief Packet decoder.
    driveunit_interface::RequestDecoder<5> m_decoder;

    /// @brief Input memory buffer for incoming requests. Store max 2 at a time before overflow.
    StaticStreamBuffer<2 * driveunit_interface::RequestEncoder::kMaxEncodedPacketSize> m_recv_buffer;

     /// @brief Buffer for outgoing responses.
    std::array<std::byte, driveunit_interface::ResponseEncoder::kMaxEncodedPacketSize> m_send_buffer;

public:
    /**
     * @brief Constructs an SPI slave interface.
     * @param spi Reference to the SPI slave instance.
     * @param handler Reference to the message handler.
     */
    explicit TransportInterface(SPISlaveClass& spi, THandler& handler)
        : m_spi(spi), m_handler(handler), m_task_handle(nullptr), m_recv_buffer()
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
            PISAR_LOG_ERROR("Task priority %u is out of range");
            return; // TODO ERROR CODE
        }

        m_response_pending.unlock();

        // Spawn the processing task
        if (xTaskCreate(taskEntry, "SPI_Processing", 2048, this, task_priority, &m_task_handle) != pdPASS)
        {
            PISAR_LOG_ERROR("Failed to create SPI processing task");
            return;
        }

        // Capture `this` in lambda to avoid needing a singleton
        m_spi.onDataRecv([this](uint8_t* data, size_t len) { this->onDataReceived(data, len); });
        m_spi.onDataSent([this]() { this->onDataSent(); });

        // Set default response.
        sendResponse(driveunit_interface::DefaultResponse());

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
                const size_t num_bytes = m_recv_buffer.receive(std::span(buffer));

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
            return;
        }

        if (m_response_pending.lock() == false)
        {
            return; // Ignore transmission if the last response hasn't been fully read
        }

        // Encode the response and send it
        if (sendResponse(driveunit_interface::DefaultResponse()) == false)
        {
            // Failed to encode or send, we have to unlock the pending flag
            m_response_pending.unlock();
        }
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

        // If we are waiting for response to get clocked out then we just ignore the transaction
        if (m_response_pending.countIsr() == 0) // 0 means not available
        {
            return;  // Ignore incoming data if the previous response is still being read
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
        BaseType_t higher_priority_task_woken = pdFALSE;
        // We automatically clear the response pending flag when data is sent.
        m_response_pending.unlockIsr(&higher_priority_task_woken);

        if (m_response_pending.countIsr() == 0)
        {
            return; // Ignore transmission if the last response hasn't been fully read
        }

        // Set default response.
        sendResponse(driveunit_interface::DefaultResponse());

        //portYIELD_FROM_ISR(higher_priority_task_woken);
    }

    /**
     * @brief Encode and setup the response for transmitting.
     *
     * @param response_msg The response message to send.
     * @return true if successful otherwise false.
     */
    inline bool sendResponse(driveunit_interface::Response response_msg)
    {
        driveunit_interface::ResponseEncoder encoder;
        const auto encoded_buffer = encoder.encode(response_msg, std::span(m_send_buffer).subspan(1));
        if (!encoded_buffer)
        {
            PISAR_LOG_ERROR("Failed to encode response");
            return false;
        }

        // First byte will contain the size of the packet
        m_send_buffer[0] = encoded_buffer.value().size();
        m_spi.setData(reinterpret_cast<uint8_t*>(m_send_buffer.data()), encoded_buffer.value().size() + 1);

        return true;
    }
};

} // namespace pisar::driveunit
