#include "pisar/driveunit/servo_driver.h"
#include "pisar/driveunit/logging.h"

#include <Arduino.h>

#include "hardware/clocks.h"
#include "hardware/divider.h"

#include <algorithm>

namespace pisar::driveunit
{

ServoDriver::ServoDriver(uint8_t gpio) : 
    m_gpio_pin(gpio)
{
    // Store PWM slice and channel once
    m_pwm_slice = pwm_gpio_to_slice_num(gpio);
    m_pwm_channel= pwm_gpio_to_channel(gpio);

}

bool ServoDriver::initialize()
{
    gpio_set_function(m_gpio_pin, GPIO_FUNC_NULL);
    delayMicroseconds(10);
    gpio_set_function(m_gpio_pin, GPIO_FUNC_PWM);
    
    pwm_config c = pwm_get_default_config();

    pwm_config_set_clkdiv(&c, 125.0f); // 125 MHz / 125 = 1 MHz → 1 tick = 1 µs
    pwm_set_wrap(m_pwm_slice, 20000); // 20 ms period → 50 Hz

    pwm_init(m_pwm_slice, &c, true);

    return true;
}

void ServoDriver::setAngle(float degrees)
{
    degrees = std::clamp(degrees, 0.0f, 180.0f);

    float pulse_width = degreesToPulseWidth(degrees);
    uint16_t duty = pulseWidthToDuty(pulse_width);
    pwm_set_gpio_level(m_gpio_pin, duty);
    m_current_angle = degrees;
}

void ServoDriver::setMinServoAngle()
{
    float pulse_width = degreesToPulseWidth(0.0f);
    uint16_t duty = pulseWidthToDuty(pulse_width);
    pwm_set_gpio_level(m_gpio_pin, duty);
    m_current_angle = 0.0f;
}

void ServoDriver::setMaxServoAngle()
{
    float pulse_width = degreesToPulseWidth(180.0f);
    uint16_t duty = pulseWidthToDuty(pulse_width);
    pwm_set_gpio_level(m_gpio_pin, duty);
    m_current_angle = 180.0f;
}

float ServoDriver::getAngle() const {
    return m_current_angle;
}

float ServoDriver::getPWM() const {
    return m_pwm;
}

constexpr float ServoDriver::degreesToPulseWidth(float degrees) {
    constexpr float kMinPulse = 1000.0f; // microseconds
    constexpr float kMaxPulse = 2000.0f;
    return kMinPulse + ((degrees / 180.0f) * (kMaxPulse - kMinPulse));
}

uint16_t ServoDriver::pulseWidthToDuty(float pulse_width_us) {
    constexpr float kPwmPeriodUs = 20000.0f; // 50Hz = 20ms (datasheet)
    constexpr uint16_t kMaxDuty = 65535;
    return static_cast<uint16_t>((pulse_width_us / kPwmPeriodUs) * kMaxDuty);
}

} // namespace pisar::driveunit
