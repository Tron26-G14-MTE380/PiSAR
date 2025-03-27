#pragma once
#include <cstdint>
#include "hardware/pwm.h"
#include "hardware/gpio.h"

namespace pisar::driveunit
{

/**
 * @brief Controls a single SG90 servo motor.
 */
class ServoDriver
{
private:
    uint8_t m_gpio_pin;          ///< GPIO pin
    float m_current_angle;         ///< Angle of servo motor
    float m_pwm;    ///< PWM signal (1ms = 0 deg, 2 ms = 180 deg)
    uint8_t m_pwm_slice;    ///< PWM slice for GPIO pin
    uint8_t m_pwm_channel;    ///< PWM channel for GPIO pin

    /**
     * @brief Converts degrees to PWM pulse width in microseconds.
     * @param degrees Angle in degrees.
     * @return Pulse width in microseconds.
     */
    static constexpr float degreesToPulseWidth(float degrees);

    /**
     * @brief Converts microseconds to duty cycle (0-65535).
     * @param pulse_width_us Pulse width in microseconds.
     * @return PWM duty cycle.
     */
    static uint16_t pulseWidthToDuty(float pulse_width_us);

public:
    /**
     * @brief Constructs a ServoDriver for an SG90 servo motor.
     * @param gpio The GPIO pin used for PWM control.
     */
    ServoDriver(uint8_t gpio);

    /// @brief Initializes the servo driver.
    [[nodiscard]] bool initialize();

    /**
     * @brief Sets the servo angle.
     * @param degrees Angle in degrees (0 to 180).
     */
    void setAngle(float degrees);

    /**
     * @brief Sets the min servo angle (0).
     */
    void setMinServoAngle();

    /**
     * @brief Sets the max servo angle (180).
     */
    void setMaxServoAngle();

    /**
     * @brief Gets the current angle.
     *
     * @return Current angle in degrees.
     */
    [[nodiscard]] float getAngle() const;

    /**
     * @brief Gets the current PWM.
     *
     * @return Current PWM.
     */
    [[nodiscard]] float getPWM() const;

};
} // namespace pisar::driveunit