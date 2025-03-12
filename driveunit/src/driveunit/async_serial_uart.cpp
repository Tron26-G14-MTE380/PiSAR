#include "pisar/driveunit/async_serial_uart.h"

#include "pisar/driveunit/logging.h"
#include "pisar/utilities/fixed_vector.h"

#include <hardware/uart.h>
#include <hardware/gpio.h>
#include <hardware/irq.h>
#include <FreeRTOS.h>
#include <semphr.h>
#include <cstdint>
#include <functional>

namespace pisar::driveunit {

static void _uart0IRQ();
static void _uart1IRQ();

bool AsyncSerialUart::setRX(pin_size_t pin)
{
#ifdef PICO_RP2350B
    constexpr uint64_t valid[2] = { __bitset({1, 13, 17, 29, 33, 45}) /* UART0 */,
                                    __bitset({5, 9, 21, 25, 37, 41})  /* UART1 */
                                    };
#else
    constexpr uint64_t valid[2] = { __bitset({1, 13, 17, 29}) /* UART0 */,
                                    __bitset({5, 9, 21, 25})  /* UART1 */
                                    };
#endif
    if ((!m_running) && ((1LL << pin) & valid[uart_get_index(m_p_uart)]))
    {
        m_rx_pin = pin;
        return true;
    }

    if (m_rx_pin == pin)
    {
        return true;
    }

    if (m_running)
    {
        panic("FATAL: Attempting to set Serial%d.RX while running", uart_get_index(m_p_uart) + 1);
    }
    else
    {
        panic("FATAL: Attempting to set Serial%d.RX to illegal pin %d", uart_get_index(m_p_uart) + 1, pin);
    }
    return false;
}

bool AsyncSerialUart::setTX(pin_size_t pin)
{
#ifdef PICO_RP2350B
    constexpr uint64_t valid[2] = { __bitset({0, 12, 16, 28, 32, 44}) /* UART0 */,
                                    __bitset({4, 8, 20, 24, 36, 40})  /* UART1 */
                                    };
#else
    constexpr uint64_t valid[2] = { __bitset({0, 12, 16, 28}) /* UART0 */,
                                    __bitset({4, 8, 20, 24})  /* UART1 */
                                    };
#endif
    if ((!m_running) && ((1LL << pin) & valid[uart_get_index(m_p_uart)]))
    {
        m_tx_pin = pin;
        return true;
    }

    if (m_tx_pin == pin)
    {
        return true;
    }

    if (m_running)
    {
        panic("FATAL: Attempting to set Serial%d.TX while running", uart_get_index(m_p_uart) + 1);
    }
    else
    {
        panic("FATAL: Attempting to set Serial%d.TX to illegal pin %d", uart_get_index(m_p_uart) + 1, pin);
    }
    return false;
}

bool AsyncSerialUart::setRTS(pin_size_t pin)
{
#ifdef PICO_RP2350B
    constexpr uint64_t valid[2] = { __bitset({3, 15, 19, 31, 35, 47}) /* UART0 */,
                                    __bitset({7, 11, 23, 27, 39, 43})  /* UART1 */
                                    };
#else
    constexpr uint64_t valid[2] = { __bitset({3, 15, 19}) /* UART0 */,
                                    __bitset({7, 11, 23, 27})  /* UART1 */
                                    };
#endif
    if ((!m_running) && ((pin == UART_PIN_NOT_DEFINED) || ((1LL << pin) & valid[uart_get_index(m_p_uart)])))
    {
        m_rts_pin = pin;
        return true;
    }

    if (m_rts_pin == pin)
    {
        return true;
    }

    if (m_running)
    {
        panic("FATAL: Attempting to set Serial%d.RTS while running", uart_get_index(m_p_uart) + 1);
    }
    else
    {
        panic("FATAL: Attempting to set Serial%d.RTS to illegal pin %d", uart_get_index(m_p_uart) + 1, pin);
    }
    return false;
}

bool AsyncSerialUart::setCTS(pin_size_t pin)
{
#ifdef PICO_RP2350B
    constexpr uint64_t valid[2] = { __bitset({2, 14, 18, 30, 34, 46}) /* UART0 */,
                                    __bitset({6, 10, 22, 26, 38, 42})  /* UART1 */
                                    };
#else
    constexpr uint64_t valid[2] = { __bitset({2, 14, 18}) /* UART0 */,
                                    __bitset({6, 10, 22, 26})  /* UART1 */
                                    };
#endif
    if ((!m_running) && ((pin == UART_PIN_NOT_DEFINED) || ((1LL << pin) & valid[uart_get_index(m_p_uart)])))
    {
        m_cts_pin = pin;
        return true;
    }

    if (m_cts_pin == pin)
    {
        return true;
    }

    if (m_running)
    {
        panic("FATAL: Attempting to set Serial%d.CTS while running", uart_get_index(m_p_uart) + 1);
    }
    else
    {
        panic("FATAL: Attempting to set Serial%d.CTS to illegal pin %d", uart_get_index(m_p_uart) + 1, pin);
    }
    return false;
}


void AsyncSerialUart::begin(unsigned long baud, uint16_t config)
{
    if (m_running)
    {
        end();
    }

    Lock<Mutex> lock(m_mutex);

    m_original_tx_fcn = gpio_get_function(m_tx_pin);
    gpio_set_function(m_tx_pin, GPIO_FUNC_UART);
    //gpio_set_outover(m_tx_pin, _invertTX ? 1 : 0);

    m_original_rx_fcn = gpio_get_function(m_rx_pin);
    gpio_set_function(m_rx_pin, GPIO_FUNC_UART);
    //gpio_set_inover(m_rx_pin, _invertRX ? 1 : 0);

    if (m_rts_pin != UART_PIN_NOT_DEFINED)
    {
        m_original_rts_fcn = gpio_get_function(m_rts_pin);
        gpio_set_function(m_rts_pin, GPIO_FUNC_UART);
        //gpio_set_outover(m_rts_pin, _invertControl ? 1 : 0);
    }

    if (m_cts_pin != UART_PIN_NOT_DEFINED)
    {
        m_original_cts_fcn = gpio_get_function(m_cts_pin);
        gpio_set_function(m_cts_pin, GPIO_FUNC_UART);
        //gpio_set_inover(m_cts_pin, _invertControl ? 1 : 0);
    }

    uart_init(m_p_uart, baud);
    int bits, stop;
    uart_parity_t parity;
    switch (config & SERIAL_PARITY_MASK)
    {
    case SERIAL_PARITY_EVEN:
        parity = UART_PARITY_EVEN;
        break;
    case SERIAL_PARITY_ODD:
        parity = UART_PARITY_ODD;
        break;
    default:
        parity = UART_PARITY_NONE;
        break;
    }
    switch (config & SERIAL_STOP_BIT_MASK)
    {
    case SERIAL_STOP_BIT_1:
        stop = 1;
        break;
    default:
        stop = 2;
        break;
    }
    switch (config & SERIAL_DATA_MASK)
    {
    case SERIAL_DATA_5:
        bits = 5;
        break;
    case SERIAL_DATA_6:
        bits = 6;
        break;
    case SERIAL_DATA_7:
        bits = 7;
        break;
    default:
        bits = 8;
        break;
    }

    uart_set_format(m_p_uart, bits, stop, parity);
    uart_set_hw_flow(m_p_uart, m_cts_pin != UART_PIN_NOT_DEFINED, m_rts_pin != UART_PIN_NOT_DEFINED);

    // Enable UART IRQ
    enableIRQ();

    m_running = true;
}

void AsyncSerialUart::end()
{
    if (!m_running)
    {
        return;
    }

    Lock<Mutex> lock(mutex);

    m_running = false;
    disableIRQ();

    uart_deinit(m_p_uart);

    // Restore pin functions
    gpio_set_function(m_tx_pin, m_original_tx_fcn);
    gpio_set_outover(m_tx_pin, 0);

    gpio_set_function(m_rx_pin, m_original_rx_fcn);
    gpio_set_inover(m_rx_pin, 0);

    if (m_rts_pin != UART_PIN_NOT_DEFINED)
    {
        gpio_set_function(m_rts_pin, m_original_rts_fcn);
        gpio_set_outover(m_rts_pin, 0);
    }

    if (m_cts_pin != UART_PIN_NOT_DEFINED)
    {
        gpio_set_function(m_cts_pin, m_original_cts_fcn);
        gpio_set_inover(m_cts_pin, 0);
    }
}

size_t AsyncSerialUart::write(uint8_t c)
{
    Lock<Mutex> lock(m_mutex);

    if (!m_running) return 0;
    uart_putc_raw(m_p_uart, c);
    return 1;
}

size_t AsyncSerialUart::write(const uint8_t* data, size_t len)
{
    Lock<Mutex> lock(m_mutex);

    if (!m_running) return 0;
    uart_write_blocking(m_p_uart, data, len);
    return len;
}

void AsyncSerialUart::handleIRQ()
{
    // Check for errors in RSR (Raw Status Register)
    uint32_t rsr = uart_get_hw(m_p_uart)->rsr;

    if (rsr & UART_UARTRSR_OE_BITS)
    {
        PISAR_LOG_WARN("Overrun Error!");
    }

    if (rsr & UART_UARTRSR_FE_BITS)
    {
        PISAR_LOG_WARN("Framing Error!");
    }

    if (rsr & UART_UARTRSR_PE_BITS)
    {
        PISAR_LOG_WARN("Parity Error!");
    }

    // Clear UART interrupt flags
    uart_get_hw(m_p_uart)->icr = UART_UARTICR_RTIC_BITS | UART_UARTICR_RXIC_BITS;

    if (!m_irq_callback)
    {
        return;
    }

    FixedVector<std::byte, 256> buffer;

    while (uart_is_readable(m_p_uart) && buffer.size() < buffer.capacity())
    {
        uint32_t raw = uart_get_hw(m_p_uart)->dr;
        if (raw & 0x400)
        {
            // break!
            continue;
        }
        else if (raw & 0x300)
        {
            // Framing, Parity Error.  Ignore this bad char
            continue;
        }

        buffer.push_back(std::byte(raw & 0xff));
    }

    // If a callback is set, call it with the received data
    m_irq_callback(std::span(buffer.data(), buffer.size()));
}

void AsyncSerialUart::enableIRQ()
{
    constexpr uint32_t kUartTimscFeimBits = (1 << 7);    // Framing Error
    constexpr uint32_t kUartTimscPeimBits = (1 << 8);    // Parity Error
    constexpr uint32_t kUartTimscBeimBits = (1 << 9);    // Break Condition
    constexpr uint32_t kUartTimscOeimBits = (1 << 10);   // Overrun Error

    // Enable UART error interrupts
    uart_get_hw(m_p_uart)->imsc |= kUartTimscFeimBits | kUartTimscPeimBits | kUartTimscBeimBits | kUartTimscOeimBits;

    const int irq_no = (m_p_uart == uart0) ? UART0_IRQ : UART1_IRQ;
    irq_set_exclusive_handler(irq_no, irq_no == UART0_IRQ ? _uart0IRQ : _uart1IRQ);
    irq_set_enabled(irq_no, true);

    uart_set_irq_enables(m_p_uart, true, false);
}

void AsyncSerialUart::disableIRQ()
{
    const int irq_no = (m_p_uart == uart0) ? UART0_IRQ : UART1_IRQ;
    irq_set_enabled(irq_no, false);
}

#if defined(PIN_SERIAL1_RTS)
AsyncSerialUart AsyncSerial1(uart0, PIN_SERIAL1_TX, PIN_SERIAL1_RX, PIN_SERIAL1_RTS, PIN_SERIAL1_CTS);
#else
AsyncSerialUart AsyncSerial1(uart0, PIN_SERIAL1_TX, PIN_SERIAL1_RX);
#endif

#if defined(PIN_SERIAL2_RTS)
AsyncSerialUart AsyncSerial2(uart1, PIN_SERIAL2_TX, PIN_SERIAL2_RX, PIN_SERIAL2_RTS, PIN_SERIAL2_CTS);
#else
AsyncSerialUart AsyncSerial2(uart1, PIN_SERIAL2_TX, PIN_SERIAL2_RX);
#endif

static void __not_in_flash_func(_uart0IRQ)()
{
    AsyncSerial1.handleIRQ();
}

static void __not_in_flash_func(_uart1IRQ)()
{
    AsyncSerial2.handleIRQ();
}


} // namespace pisar::driveunit
