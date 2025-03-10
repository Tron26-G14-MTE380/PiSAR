#include <iostream>
#include <chrono>
#include <vector>
#include <thread>
#include <Eigen/Dense>

#include "pisar/driveunit_interface/interface.h"
#include "pisar/async_driveunit_controller.h"

#include <wiringPi.h>

using namespace pisar::driveunit_interface;
using namespace pisar::mcp;

int main()
{
    wiringPiSetup(); // Initializes wiringPi using wiringPi's simlified number system.

    DriveunitTransport transport;
    DriveunitController controller(transport);

    if (transport.open() == false)
    {
        std::cerr << "Error opening UART device: " << transport.device() << std::endl;
        return 0;
    }

    std::cout << "Starting Communication Test..." << std::endl;

    // 1️⃣ Test Heartbeat
    auto heartbeat = controller.sendHeartbeat();
    if (heartbeat)
    {
        std::cout << "✅ Heartbeat received. Time alive: " << heartbeat->time_alive.count() << " ms" << std::endl;
    }
    else
    {
        std::cerr << "❌ Failed to receive heartbeat!" << std::endl;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // 2️⃣ Test Idle Command
    if (controller.sendIdleCommand())
    {
        std::cout << "✅ Idle command acknowledged." << std::endl;
    }
    else
    {
        std::cerr << "❌ Idle command failed!" << std::endl;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // 3️⃣ Test Rotation Command
    float rotation_degrees = 90.0f; // Rotate 90 degrees CCW
    if (controller.sendRotateCommand(rotation_degrees))
    {
        std::cout << "✅ Rotate command acknowledged." << std::endl;
    }
    else
    {
        std::cerr << "❌ Rotate command failed!" << std::endl;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));


    // 4️⃣ Test Trajectory Command
    std::vector<Eigen::Vector2f> trajectory = {
        {0.0f, 0.0f}, {1.0f, 1.0f}, {2.0f, 0.0f}, {3.0f, -1.0f},
        {0.0f, 0.0f}, {1.0f, 1.0f}, {2.0f, 0.0f}, {3.0f, -1.0f},
        {0.0f, 0.0f}, {1.0f, 1.0f}, {2.0f, 0.0f}, {3.0f, -1.0f},
        {0.0f, 0.0f}, {1.0f, 1.0f}, {2.0f, 0.0f}, {3.0f, -1.0f},
    };
    auto reference_time = std::chrono::duration<float>(2.5f); // Execute over 2.5 seconds

    if (controller.sendTrajectoryCommand(reference_time, trajectory))
    {
        std::cout << "✅ Trajectory command acknowledged." << std::endl;
    }
    else
    {
        std::cerr << "❌ Trajectory command failed!" << std::endl;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    std::cout << "Comm Test Complete!" << std::endl;
    return 0;
}
