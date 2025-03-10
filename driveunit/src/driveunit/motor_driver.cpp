#include "pisar/driveunit/motor_driver.h"
#include "pisar/driveunit/logging.h"

#include <Arduino.h>

#include "hardware/clocks.h"
#include "hardware/divider.h"

#include <algorithm>

namespace pisar::driveunit
{

static uint32_t div_u32u32(uint32_t a, uint32_t b) {
    return hw_divider_u32_quotient(a, b);
}


MotorDriver::MotorDriver(uint8_t pin_a, uint8_t pin_b, uint32_t pwm_freq)
    : m_pin_a(pin_a), m_pin_b(pin_b), m_pwm_freq(pwm_freq), m_pwm_resolution(0)
{
    // Store PWM slice and channel once
    m_pwm_slice_a = pwm_gpio_to_slice_num(pin_a);
    m_pwm_channel_a = pwm_gpio_to_channel(pin_a);

    m_pwm_slice_b = pwm_gpio_to_slice_num(pin_b);
    m_pwm_channel_b = pwm_gpio_to_channel(pin_b);
}

void MotorDriver::initialize()
{
    gpio_set_function(m_pin_a, GPIO_FUNC_NULL);
    gpio_set_function(m_pin_b, GPIO_FUNC_NULL);
    delayMicroseconds(10);
    gpio_set_function(m_pin_a, GPIO_FUNC_PWM);
    gpio_set_function(m_pin_b, GPIO_FUNC_PWM);

    // Compute clock divider for desired PWM frequency
    uint8_t clk_divider = 0;
    uint32_t wrap = 0;
    uint32_t clock_div = 0;
    uint32_t clock = clock_get_hz(clk_sys);

    for(clk_divider = 1; clk_divider < UINT8_MAX; clk_divider++)
    {
        /* Find clock_division to fit current frequency */
        clock_div = div_u32u32(clock, clk_divider);
        wrap = div_u32u32(clock_div, m_pwm_freq);
        if (div_u32u32 (clock_div, UINT16_MAX) <= m_pwm_freq && wrap <= UINT16_MAX)
        {
            break;
        }
    }

    m_pwm_resolution = wrap;

    PISAR_LOG_DEBUG("PWM Slice %u and %u: Clock = %u, div = %u, wrap = %u", m_pwm_slice_a, m_pwm_slice_b, clock, clk_divider, m_pwm_resolution);

    pwm_config c = pwm_get_default_config();
    pwm_config_set_clkdiv_int_frac(&c, clk_divider, 0);
    pwm_config_set_wrap(&c, m_pwm_resolution);

    pwm_init(m_pwm_slice_a, &c, true);
    pwm_init(m_pwm_slice_b, &c, true);
}

void MotorDriver::setSpeed(float speed)
{
    if (!m_enabled)
    {
        return;
    }

    // Clamp speed to range -1.0 to 1.0
    speed = std::clamp(speed, -1.0f, 1.0f);

    // Convert absolute speed to PWM duty cycle (0 to resolution)
    uint16_t duty_cycle = speed == 1.0f ? (m_pwm_resolution + 1) : static_cast<uint16_t>(std::abs(speed) * m_pwm_resolution);

    if (speed > 0)
    {
        // Forward: A = PWM, B = LOW
        pwm_set_chan_level(m_pwm_slice_a, m_pwm_channel_a, duty_cycle);
        pwm_set_chan_level(m_pwm_slice_b, m_pwm_channel_b, 0);
    }
    else if (speed < 0)
    {
        // Reverse: A = LOW, B = PWM
        pwm_set_chan_level(m_pwm_slice_a, m_pwm_channel_a, 0);
        pwm_set_chan_level(m_pwm_slice_b, m_pwm_channel_b, duty_cycle);
    }
    else
    {
        // Stop: A = LOW, B = LOW (brake)
        pwm_set_chan_level(m_pwm_slice_a, m_pwm_channel_a, 0);
        pwm_set_chan_level(m_pwm_slice_b, m_pwm_channel_b, 0);
    }
}

} // namespace pisar::driveunit
