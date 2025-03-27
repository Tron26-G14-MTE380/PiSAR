#pragma once

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
    uint8_t m_pwm_slice_a;    ///< PWM slice for pin A
    uint8_t m_pwm_slice_b;    ///< PWM slice for pin B
    uint8_t m_pwm_channel_a;  ///< PWM channel for pin A
    uint8_t m_pwm_channel_b;  ///< PWM channel for pin B
    uint32_t m_pwm_freq;         ///< Desired PWM frequency in Hz
    uint32_t m_pwm_resolution; ///< PWM resolution (max counter value)
    float m_min_speed;
    float m_max_speed;
    bool m_enabled = false;

public:
    /**
     * @brief Constructs a MotorDriver object for Cytron MDD3A.
     * @param pin_a GPIO pin A.
     * @param pin_b GPIO pin B.
     * @param min_speed The minimum motor speed (0.0f - 1.0f).
     * @param max_speed The maximum motor speed (0.0f - 1.0f).
     * @param pwm_freq (Optional) PWM frequency in Hz (default: 20 kHz).
     */
    MotorDriver(uint8_t pin_a, uint8_t pin_b, float min_speed = 0.0f, float max_speed = 1.0f, uint32_t pwm_freq = 20000);

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

    [[nodiscard]] inline float getMinSpeed() const { return m_min_speed; }
    [[nodiscard]] inline float getMaxSpeed() const { return m_max_speed; } 
    [[nodiscard]] inline float getSpeedRange() const { return m_max_speed - m_min_speed; } 
};

} // namespace pisar::driveunit
