#include "pisar/driveunit/motor_driver.h"

#include "hardware/clocks.h"

#include <algorithm>

namespace pisar::driveunit
{

MotorDriver::MotorDriver(uint8_t pin_a, uint8_t pin_b, float pwm_freq, uint16_t pwm_resolution)
    : m_pin_a(pin_a), m_pin_b(pin_b), m_pwm_freq(pwm_freq), m_pwm_resolution(pwm_resolution)
{
    // Store PWM slice and channel once
    m_pwm_slice = pwm_gpio_to_slice_num(pin_a);
    m_pwm_channel_a = pwm_gpio_to_channel(pin_a);
    m_pwm_channel_b = pwm_gpio_to_channel(pin_b);
}

void MotorDriver::initialize()
{
    gpio_set_function(m_pin_a, GPIO_FUNC_PWM);
    gpio_set_function(m_pin_b, GPIO_FUNC_PWM);

    // Compute clock divider for desired PWM frequency
    float system_clock = clock_get_hz(clk_sys); // Get system clock (default 125MHz)
    float clkdiv = system_clock / (m_pwm_resolution * m_pwm_freq);

    pwm_set_clkdiv(m_pwm_slice, clkdiv);
    pwm_set_wrap(m_pwm_slice, m_pwm_resolution); // Set PWM resolution (wrap value)

    pwm_set_enabled(m_pwm_slice, false);
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
    uint16_t duty_cycle = static_cast<uint16_t>(std::abs(speed) * m_pwm_resolution);

    if (speed > 0)
    {
        // Forward: A = PWM, B = LOW
        pwm_set_chan_level(m_pwm_slice, m_pwm_channel_a, duty_cycle);
        pwm_set_chan_level(m_pwm_slice, m_pwm_channel_b, 0);
        pwm_set_enabled(m_pwm_slice, true);
    }
    else if (speed < 0)
    {
        // Reverse: A = LOW, B = PWM
        pwm_set_chan_level(m_pwm_slice, m_pwm_channel_a, 0);
        pwm_set_chan_level(m_pwm_slice, m_pwm_channel_b, duty_cycle);
        pwm_set_enabled(m_pwm_slice, true);
    }
    else
    {
        // Stop: A = LOW, B = LOW (brake)
        pwm_set_chan_level(m_pwm_slice, m_pwm_channel_a, 0);
        pwm_set_chan_level(m_pwm_slice, m_pwm_channel_b, 0);
        pwm_set_enabled(m_pwm_slice, false);
    }
}

} // namespace pisar::driveunit
