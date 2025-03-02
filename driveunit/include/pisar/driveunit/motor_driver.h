#pragma once

#include <Arduino.h>

#include "hardware/pwm.h"
#include "hardware/gpio.h"

#include <cstdint>
#include <limits>

namespace pisar::driveunit
{

/**
 * @brief Controls a single DC motor using the Cytron MDD3A H-Bridge.
 */
class MotorDriver
{
private:
    uint8_t m_pin_a;          ///< GPIO pin A (PWM capable)
    uint8_t m_pin_b;          ///< GPIO pin B (PWM capable)
    uint8_t m_pwm_slice;      ///< PWM slice controlling both pins
    uint8_t m_pwm_channel_a;  ///< PWM channel for pin A
    uint8_t m_pwm_channel_b;  ///< PWM channel for pin B
    bool m_enabled = false;

    float m_pwm_freq;         ///< Desired PWM frequency in Hz
    uint16_t m_pwm_resolution; ///< PWM resolution (max counter value)

public:
    /**
     * @brief Constructs a MotorDriver object for Cytron MDD3A.
     * @param pin_a GPIO pin A.
     * @param pin_b GPIO pin B.
     * @param pwm_freq (Optional) PWM frequency in Hz (default: 16 kHz).
     * @param pwm_resolution (Optional) PWM resolution (default: 65535 for 16-bit).
     */
    MotorDriver(uint8_t pin_a, uint8_t pin_b, float pwm_freq = 16000.0f, uint16_t pwm_resolution = std::numeric_limits<std::uint16_t>::max());

    /// @brief Initializes the motor driver.
    void initialize();

    /**
     * @brief Sets the motor speed and direction.
     * @param speed Value between -1.0 (full reverse) and 1.0 (full forward).
     */
    void setSpeed(float speed);

    /// @brief Enables the motor driver (allows setting speed).
    inline void enable()
    {
        m_enabled = true;
    }

    /// @brief Disables the motor driver (stops motor and prevents new commands).
    inline void disable()
    {
        stop();
        m_enabled = false;
    }

    /// @brief Stops the motor (braking mode).
    inline void stop()
    {
        setSpeed(0.0f);
    }

    /**
     * @brief Checks if the motor driver is enabled.
     * @return True if enabled, false otherwise.
     */
    [[nodiscard]] inline bool isEnabled() const
    {
        return m_enabled;
    }
};

} // namespace pisar::driveunit
