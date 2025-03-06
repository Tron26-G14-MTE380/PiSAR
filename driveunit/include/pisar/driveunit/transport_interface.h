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
    SemaphoreHandle_t m_data_ready_semaphore;               ///< Semaphore for signaling data reception.

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
        m_data_ready_semaphore = xSemaphoreCreateBinary();
        configASSERT(m_data_ready_semaphore != nullptr);
    }

    /**
     * @brief Initializes SPI as a slave device.
     */
    void initialize()
    {
        // Spawn the processing task
        xTaskCreate(
            taskEntry, "SPI_Processing", 2048, this, m_task_priority, &m_task_handle
        );

        // Capture `this` in lambda to avoid needing a singleton
        m_spi.onDataRecv([this](uint8_t* data, size_t len) { this->onDataReceived(data, len); });
        m_spi.onDataSent([this]() { this->onDataSent(); });

        // Set default response
        prepareDefaultResponse();

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
            if (xSemaphoreTake(m_data_ready_semaphore, portMAX_DELAY) == pdTRUE)
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
        const size_t response_size = m_handler.handleMessage(packet.value(), std::span<std::byte>(m_send_buffer));

        // Set the response in SPI buffer
        if (response_size > 0)
        {
            m_spi.setData(reinterpret_cast<uint8_t*>(m_send_buffer.data()), response_size);
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
        memcpy(m_recv_buffer.data(), data, len);
        xSemaphoreGiveFromISR(m_data_ready_semaphore, nullptr);  // Signal processing task
    }

    /**
     * @brief Called when the master has finished reading from SPI.
     */
    void onDataSent()
    {
        prepareDefaultResponse();
    }

    /**
     * @brief Sets a default response in the send buffer.
     */
    inline void prepareDefaultResponse()
    {
        strcpy(m_send_buffer.data(), "ACK");
        m_spi.setData(reinterpret_cast<uint8_t*>(m_send_buffer.data()), strlen(m_send_buffer.data()));
    }
};

} // namespace pisar::driveunit
