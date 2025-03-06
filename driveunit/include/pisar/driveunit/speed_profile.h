#pragma once

#include <cstdint>
#include <cmath>

namespace pisar::driveunit
{

/**
 * @brief Generates speed profiles for smooth motor control.
 */
class SpeedProfile
{
private:
    float m_current_speed;      ///< Current speed value
    float m_target_speed;       ///< Desired final speed
    float m_forward_accel;      ///< Acceleration rate for forward movement (speed/sec)
    float m_forward_decel;      ///< Deceleration rate for stopping forward movement (speed/sec)
    float m_reverse_accel;      ///< Acceleration rate for reverse movement (speed/sec)
    float m_reverse_decel;      ///< Deceleration rate for stopping reverse movement (speed/sec)

    uint32_t m_last_update_time_us;

public:
    /**
     * @brief Constructs a SpeedProfile object.
     * @param acceleration Acceleration rate (speed increase per second).
     * @param deceleration Deceleration rate (speed decrease per second).
     */
    inline constexpr SpeedProfile(float forward_accel, float forward_decel, float reverse_accel, float reverse_decel)
    : m_current_speed(0), m_target_speed(0),
      m_forward_accel(forward_accel), m_forward_decel(forward_decel),
      m_reverse_accel(reverse_accel), m_reverse_decel(reverse_decel),
      m_last_update_time_us(0)
    {
    }

    /**
     * @brief Sets a new target speed.
     * @param target_speed Desired speed.
     */
    inline constexpr void setTargetSpeed(float target_speed)
    {
        m_target_speed = target_speed;
    }

    /**
     * @brief Updates the speed based on time elapsed.
     * @param current_time_us Current time stamp in microseconds.
     * @return The updated speed value.
     */
    inline constexpr float update(uint32_t current_time_us)
    {
        const float delta_time = (current_time_us - m_last_update_time_us) / 1'000'000.0f;

        const float delta_speed = (m_target_speed - m_current_speed);
        if (delta_speed == 0)
        {
            return 0;
        }

        const int operating_direction = (m_current_speed > 0.0f) - (m_current_speed < 0.0f);
        const int target_operating_direction = (m_target_speed > 0.0f) - (m_target_speed < 0.0f);

        const float accel_abs = operating_direction > 0 ? m_forward_accel : m_reverse_accel;
        const float decel_abs = operating_direction > 0 ? m_forward_decel : m_reverse_decel;

        const float accel = accel_abs * operating_direction;
        const float decel = -1 * decel_abs * operating_direction;

        // **Special Case: Switching Directions → Decelerate to 0 First**
        if (operating_direction != target_operating_direction && operating_direction != 0)
        {
            m_current_speed += decel * delta_time;

            const int new_operating_direction = (m_current_speed > 0.0f) - (m_current_speed < 0.0f);

            // If we cross zero, snap to zero to avoid floating-point drift
            if (operating_direction != new_operating_direction)
            {
                m_current_speed = 0.0f;
            }
        }
        else
        {
            // If going forward and abs(m_target_speed) > abs(m_current_speed), then abs_delta_speed > 0
            // If going forward and abs(m_target_speed) < abs(m_current_speed), then abs_delta_speed < 0
            // If going backward and abs(m_target_speed) > abs(m_current_speed), then abs_delta_speed > 0
            // If going backward and abs(m_target_speed) < abs(m_current_speed), then abs_delta_speed < 0
            const float abs_delta_speed = delta_speed * operating_direction;
            m_current_speed += (abs_delta_speed > 0 ? accel : decel) * delta_time;

            // **Prevent Overshooting**
            if ((delta_speed > 0.0f && m_current_speed > m_target_speed) || (delta_speed < 0.0f && m_current_speed < m_target_speed))
            {
                m_current_speed = m_target_speed;
            }
        }

        return m_current_speed;
    }

    /// @brief Resets the speed to zero.
    inline constexpr void reset()
    {
        m_current_speed = 0.0f;
        m_target_speed = 0.0f;
    }

    /// @brief Gets the current speed.
    [[nodiscard]] inline constexpr float getCurrentSpeed() const
    {
        return m_current_speed;
    }
};

} // namespace pisar::driveunit
