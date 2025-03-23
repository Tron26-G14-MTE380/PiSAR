#pragma once

#include "pisar/driveunit/motor_driver.h"
#include "pisar/driveunit/speed_profile.h"

namespace pisar::driveunit
{

/**
 * @brief Controls two motors using a differential drive system.
 */
class DifferentialDriveController
{
private:
    MotorDriver& m_left_motor;      ///< Left motor instance
    MotorDriver& m_right_motor;     ///< Right motor instance
    SpeedProfile m_left_profile;    ///< Speed profile for left motor
    SpeedProfile m_right_profile;   ///< Speed profile for right motor

    float m_wheel_base;             ///< Distance between wheels in meters

public:
    /**
     * @brief Constructs a DifferentialDriveController with two motors.
     * @param left_motor Reference to the left motor.
     * @param right_motor Reference to the right motor.
     * @param wheel_base Distance between left and right wheels (meters).
     * @param accel Acceleration rate.
     * @param decel Deceleration rate.
     */
    DifferentialDriveController(MotorDriver& left_motor, MotorDriver& right_motor,
                    const float wheel_base, const float accel = 0.5f, const float decel = 2.0f);

    /**
     * @brief Initializes the drive controller
     *
     */
    inline void initialize()
    {
        m_left_motor.initialize();
        m_right_motor.initialize();
        m_left_profile.reset();
        m_right_profile.reset();
    }

    const MotorDriver& getLeftMotor() const { return m_left_motor; }
    const MotorDriver& getRightMotor() const { return m_right_motor; }

    [[nodiscard]] inline float getMinSpeed() const { return m_left_motor.getMinSpeed(); }
    [[nodiscard]] inline float getMaxSpeed() const { return m_left_motor.getMaxSpeed(); } 
    [[nodiscard]] inline float getSpeedRange() const { return m_left_motor.getSpeedRange(); } 

    /**
     * @brief Drives the robot using tank-style controls.
     *
     * @param left_speed The left wheel speed (-1.0 to 1.0).
     * @param right_speed The right wheel speed (-1.0 to 1.0).
     */
    void tankDrive(const float left_speed, const float right_speed);

    /**
     * @brief Drives the robot using arcade-style controls.
     * @param forward The forward speed (-1.0 to 1.0).
     * @param rotation The rotation speed (-1.0 to 1.0).
     */
    void arcadeDrive(const float forward, const float rotation);

    /**
     * @brief Drives using curvature (smooth turning).
     * @param speed Forward speed (-1.0 to 1.0).
     * @param curvature Turning curvature (-1.0 for left, 1.0 for right, 0 for straight).
     * @param allow_reverse If true, allows reversing at negative curvature.
     */
    void curvatureDrive(const float speed, const float curvature, const bool allow_reverse = false);

    /**
     * @brief Rotates in place.
     * @param angular_velocity Rotation speed (-1.0 to 1.0).
     */
    void rotate(const float angular_velocity);

    /**
     * @brief Moves in an arc with a specified radius.
     * @param radius Radius of the arc (meters).
     * @param speed Forward speed (-1.0 to 1.0).
     */
    void driveArc(const float radius, const float speed);

    /// @brief Stops both motors smoothly.
    void stop()
    {
        m_left_profile.setTargetSpeed(0.0f);
        m_right_profile.setTargetSpeed(0.0f);
    }

    void hardStop()
    {
        m_left_motor.stop();
        m_right_motor.stop();

        m_left_profile.reset();
        m_right_profile.reset();
    }

    /// @brief Enables the motor drivers.
    void enable()
    {
        m_left_motor.enable();
        m_right_motor.enable();
    }

    /// @brief Disables the motor drivers.
    void disable()
    {
        m_left_motor.disable();
        m_right_motor.disable();

        m_left_profile.reset();
        m_right_profile.reset();
    }

    /// @brief Updates motor speeds, applying acceleration/deceleration profiles.
    void update();
};

} // namespace pisar::driveunit
