#pragma once

#include <algorithm>
#include <chrono>

namespace pisar::driveunit {

/**
 * @brief Generic PID controller class.
 */
class PidController {
private:
    float m_kp;
    float m_ki;
    float m_kd;

    float m_output_min;
    float m_output_max;

    float m_integral;
    float m_prev_error;
    bool m_first_update;

public:
    /**
     * @brief Construct a new PidController.
     * 
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     * @param output_min Minimum allowed output value
     * @param output_max Maximum allowed output value
     */
    inline PidController(float kp, float ki, float kd, float output_min = -1e6f, float output_max = 1e6f)
        : m_kp(kp), m_ki(ki), m_kd(kd),
         m_output_min(output_min), m_output_max(output_max),
         m_integral(0.0f), m_prev_error(0.0f), m_first_update(true) {}

    /**
     * @brief Resets the PID controller internal state.
     */
    inline void reset()
    {
        m_integral = 0.0f;
        m_prev_error = 0.0f;
        m_first_update = true;
    }

    /**
     * @brief Updates the PID controller with a new measurement.
     * 
     * @param setpoint The desired target value
     * @param measurement The current value
     * @param dt Time delta in seconds
     * @return The control output
     */
    [[nodiscard]] inline float update(const float setpoint, const float measurement, const std::chrono::duration<float> dt)
    {
        const float error = setpoint - measurement;
        return update(error, dt);
    }

    /**
     * @brief Updates the PID controller with a new measurement.
     * 
     * @param error The error value.
     * @param dt Time delta in seconds
     * @return The control output
     */
    [[nodiscard]] float update(const float error, const std::chrono::duration<float> dt)
    {
        m_integral += error * dt.count();

        // Anti-windup via clamping
        // ELEPHANT
        // m_integral = std::clamp(m_integral, m_output_min, m_output_max);

        float derivative = 0.0f;
        if (!m_first_update && dt.count() > 0.0f) 
        {
            derivative = (error - m_prev_error) / dt.count();
        }

        m_prev_error = error;
        m_first_update = false;

        float output = m_kp * error + m_ki * m_integral + m_kd * derivative;
        return std::clamp(output, m_output_min, m_output_max);
    }

    /**
     * @brief Sets PID gains.
     */
    void inline setGains(float kp, float ki, float kd)
    {
        m_kp = kp;
        m_ki = ki;
        m_kd = kd;
    }

    /**
     * @brief Sets output saturation limits.
     */
    void inline setOutputLimits(float min_val, float max_val)
    {
        m_output_min = min_val;
        m_output_max = max_val;
    }
};

}