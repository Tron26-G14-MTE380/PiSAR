#pragma once

#include "pisar/driveunit/sync.h"

#include <Arduino.h>
#include "api/HardwareSerial.h"

#include <stdarg.h>
#include <functional>

namespace pisar::driveunit {


/**
 * @brief Specialized AsyncSerialUart class with template-based pin assignment and IRQ hook support.
 */
class AsyncSerialUart {
public:
    using CallbackFunc = std::function<void(const std::span<const std::byte>)>;

private:
    uart_inst_t* m_p_uart;

    uint8_t m_tx_pin;
    uint8_t m_rx_pin;
    int8_t m_rts_pin;
    int8_t m_cts_pin;

    bool m_running;
    CallbackFunc m_irq_callback;                 ///< Custom IRQ callback function
    Mutex m_mutex;

    gpio_function_t m_original_tx_fcn;
    gpio_function_t m_original_rx_fcn;
    gpio_function_t m_original_rts_fcn;
    gpio_function_t m_original_cts_fcn;

public:
    inline AsyncSerialUart(uart_inst_t* p_uart, uint8_t tx_pin, uint8_t rx_pin, int8_t rts_pin = -1, int8_t cts_pin = -1) :
        m_p_uart(p_uart), m_tx_pin(tx_pin), m_rx_pin(rx_pin), m_rts_pin(rts_pin), m_cts_pin(cts_pin),
        m_running(false), m_irq_callback(nullptr) {}

    bool setRX(pin_size_t pin);
    bool setTX(pin_size_t pin);

    bool setRTS(pin_size_t pin);

    bool setCTS(pin_size_t pin);

    /**
     * @brief Initializes the UART.
     * @param baud Baud rate.
     * @param config UART configuration (e.g., stop bits, parity).
     */
    void begin(unsigned long baud, uint16_t config);

    /**
     * @brief Initializes the UART.
     * @param baud Baud rate.
     * @param config UART configuration (e.g., stop bits, parity).
     */
    inline void begin(unsigned long baud = 115200)
    {
        begin(baud, SERIAL_8N1);
    };

    /**
     * @brief Stops UART and disables IRQs.
     */
    void end();


    /**
     * @brief Sets a custom IRQ callback function.
     * @param callback Function to handle incoming UART data.
     */
    inline void setIRQCallback(CallbackFunc callback)
    {
        Lock<Mutex> lock(m_mutex);
        m_irq_callback = callback;
    }

    /**
     * @brief Writes a byte to the UART.
     * @param c The byte to send.
     * @return Number of bytes written.
     */
    size_t write(uint8_t c);

    /**
     * @brief Writes a buffer to the UART.
     * @param data The data buffer.
     * @param len Length of data to send.
     * @return Number of bytes written.
     */
    size_t write(const uint8_t* data, size_t len);

    /**
     * @brief Flushes the UART m_tx_pin buffer.
     */
    inline void flush()
    {
        Lock<Mutex> lock(m_mutex);

        uart_tx_wait_blocking(m_p_uart);
    }

    /**
     * @brief IRQ Handler function.
     */
    void handleIRQ();

private:
    /**
     * @brief Enables UART IRQs.
     */
    void enableIRQ();

    /**
     * @brief Disables UART IRQs.
     */
    void disableIRQ();
};

extern AsyncSerialUart AsyncSerial1; // HW UART 0
extern AsyncSerialUart AsyncSerial2; // HW UART 1


} // namespace pisar::driveunit
