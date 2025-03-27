#pragma once

#include "pisar/driveunit/sync.h"

#include <Servo.h>

namespace pisar::driveunit {

/**
 * @brief Controls the robot gripper.
 */
class GripperController
{
private:
    const int kMinPulseWidth = 800;
    const int kMaxPulseWidth = 2500; 
    uint8_t m_servo_pin;        ///< GPIO pin for the servo.
    uint8_t m_open_angle;       ///< Angle to open the gripper.
    uint8_t m_close_angle;      ///< Angle to close the gripper.
    Servo m_servo;              ///< Servo object for controlling the gripper.

    mutable Mutex m_mutex;

public:

    /**
     * @brief Constructs a GripperController object.
     * @param servo_pin The GPIO pin for the servo.
     * @param open_angle The angle to open the gripper (default: 0).
     * @param close_angle The angle to close the gripper (default: 180).
     */
    GripperController(uint8_t servo_pin, uint8_t open_angle = 0, uint8_t close_angle = 180) 
        : m_servo_pin(servo_pin), m_open_angle(open_angle), m_close_angle(close_angle)
    {
    }

    /// @brief Destructor.
    ~GripperController()
    {
        m_servo.detach();
    }

    /**
     * @brief Initializes the gripper controller.
     * @return True if initialization was successful, false otherwise.
     */
    [[nodiscard]] bool initialize()
    {
        return m_servo.attach(m_servo_pin, kMinPulseWidth, kMaxPulseWidth) != 0;
    }

    /**
     * @brief Opens the gripper.
     */
    void open()
    {
        Lock<Mutex> lock(m_mutex);
        m_servo.write(m_open_angle);
    }

    /**
     * @brief Closes the gripper.
     */
    void close()
    {
        Lock<Mutex> lock(m_mutex);
        m_servo.write(m_close_angle);
    }
};

} // namespace pisar::driveunit