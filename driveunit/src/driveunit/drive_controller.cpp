#include "pisar/driveunit/drive_controller.h"

#include <Arduino.h>

#include <cmath>
#include <algorithm>

static constexpr float kCurvatureRadiusSpinThreshold = 1e-5;

namespace pisar::driveunit
{

DifferentialDriveController::DifferentialDriveController(MotorDriver& left_motor, MotorDriver& right_motor,
                                 float wheel_base, float accel, float decel)
    : m_left_motor(left_motor), m_right_motor(right_motor),
      m_left_profile(accel, decel, accel, decel),
      m_right_profile(accel, decel, accel, decel),
      m_wheel_base(wheel_base)
{
}

void DifferentialDriveController::tankDrive(const float left_speed, const float right_speed)
{
    m_left_profile.setTargetSpeed(left_speed);
    m_right_profile.setTargetSpeed(right_speed);
}

void DifferentialDriveController::arcadeDrive(const float forward, const float rotation)
{
    float left_speed = forward + rotation;
    float right_speed = forward - rotation;

    left_speed = std::clamp(left_speed, -1.0f, 1.0f);
    right_speed = std::clamp(right_speed, -1.0f, 1.0f);

    tankDrive(left_speed, right_speed);
}

void DifferentialDriveController::curvatureDrive(const float speed, const float curvature, const bool allow_reverse)
{
    const float radius = 1.0f / curvature; // Convert curvature to radius

    if (std::abs(radius) < kCurvatureRadiusSpinThreshold)
    {
        tankDrive(speed, speed);
        return;
    }

    float left_speed = speed * ((radius - m_wheel_base / 2) / radius);
    float right_speed = speed * ((radius + m_wheel_base / 2) / radius);

    if (allow_reverse && speed < 0.0f)
    {
        std::swap(left_speed, right_speed);
    }

    tankDrive(left_speed, right_speed);
}

void DifferentialDriveController::rotate(const float angular_velocity)
{
    float left_speed = angular_velocity;
    float right_speed = -angular_velocity;

    tankDrive(left_speed, right_speed);
}

void DifferentialDriveController::driveArc(const float radius, const float speed)
{
    if (std::abs(radius) < kCurvatureRadiusSpinThreshold)
    {
        rotate(speed); // If radius is too small, just rotate in place
        return;
    }

    // Calculate speed ratios based on radius and wheelbase
    float left_speed = speed * (1.0f - (m_wheel_base / (2.0f * std::abs(radius))));
    float right_speed = speed * (1.0f + (m_wheel_base / (2.0f * std::abs(radius))));

    // Normalize the speeds so the faster wheel never exceeds 1.0
    float max_speed = std::max(std::abs(left_speed), std::abs(right_speed));
    if (max_speed > 1.0f)
    {
        left_speed /= max_speed;
        right_speed /= max_speed;
    }

    // Apply signs to maintain direction
    if (radius < 0) // Turning left
    {
        left_speed *= -1.0f;
        right_speed *= 1.0f;
    }

    tankDrive(left_speed, right_speed);
}

void DifferentialDriveController::update()
{
    const std::chrono::microseconds current_time = static_cast<std::chrono::microseconds>(micros());
    const float left_speed = m_left_profile.update(current_time);
    const float right_speed = m_right_profile.update(current_time);

    m_left_motor.setSpeed(left_speed);
    m_right_motor.setSpeed(right_speed);
}

} // namespace pisar::driveunit
