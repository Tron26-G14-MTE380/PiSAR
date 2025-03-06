#include <iostream>
#include <chrono>
#include <vector>
#include <thread>
#include <Eigen/Dense>

#include "pisar/driveunit_interface/interface.h"
#include "pisar/driveunit_controller.h"  // Your SPI master class

using namespace pisar::driveunit_interface;
using namespace pisar::mcp;

int main()
{
    // Initialize SPI on channel 0
    DriveUnitSPI spi_master(0);

    std::cout << "Starting SPI Communication Test..." << std::endl;

    // 1️⃣ Test Heartbeat
    auto heartbeat = spi_master.sendHeartbeat();
    if (heartbeat)
    {
        std::cout << "✅ Heartbeat received. Time alive: " << heartbeat->time_alive.count() << " ms" << std::endl;
    }
    else
    {
        std::cerr << "❌ Failed to receive heartbeat!" << std::endl;
    }

    // 2️⃣ Test Idle Command
    if (spi_master.sendIdleCommand())
    {
        std::cout << "✅ Idle command acknowledged." << std::endl;
    }
    else
    {
        std::cerr << "❌ Idle command failed!" << std::endl;
    }

    // 3️⃣ Test Rotation Command
    float rotation_degrees = 90.0f; // Rotate 90 degrees CCW
    if (spi_master.sendRotateCommand(rotation_degrees))
    {
        std::cout << "✅ Rotate command acknowledged." << std::endl;
    }
    else
    {
        std::cerr << "❌ Rotate command failed!" << std::endl;
    }

    // 4️⃣ Test Trajectory Command
    std::vector<Eigen::Vector2f> trajectory = {
        {0.0f, 0.0f}, {1.0f, 1.0f}, {2.0f, 0.0f}, {3.0f, -1.0f}
    };
    auto reference_time = std::chrono::duration<float>(2.5f); // Execute over 2.5 seconds

    if (spi_master.sendTrajectoryCommand(reference_time, trajectory))
    {
        std::cout << "✅ Trajectory command acknowledged." << std::endl;
    }
    else
    {
        std::cerr << "❌ Trajectory command failed!" << std::endl;
    }

    std::cout << "SPI Test Complete!" << std::endl;
    return 0;
}
