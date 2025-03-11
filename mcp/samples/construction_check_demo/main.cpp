#include "pisar/math.h"
#include "pisar/vision/video_source.h"
#include "pisar/vision/line_tracker.h"
#include "pisar/vision/homography.h"
#include "pisar/vision/debug_visualization.h"
#include "pisar/driveunit_controller.h"

#include <easy/profiler.h>

#include <wiringPi.h>

#include <numbers>
#include <iostream>
#include <ranges>
#include <algorithm>
#include <span>
#include <chrono>


using namespace pisar::mcp;

constexpr bool kDebug = true;

const std::array<std::pair<cv::Scalar, cv::Scalar>, 2> kTapeHsvThresholds = {
    std::make_pair(cv::Scalar(0, 120, 70), cv::Scalar(10, 255, 255)),
    std::make_pair(cv::Scalar(170, 120, 70), cv::Scalar(180, 255, 255))
};

const CameraTransform kCameraTransform {
    .position = {0, 5, 12},
    .tilt = Eigen::AngleAxisd(pisar::mcp::deg_to_rad<double>(-45), Eigen::Vector3d::UnitX())
};

const Eigen::Matrix3d kAxisMapping {
    {1, 0, 0},
    {0, 0, 1},
    {0, 1, 0}
};

const CameraCalibrationData kCameraCalibration {
    .calibration_img_size_px = {640, 480},
    .focal_length_px = {450, 450},
    .principle_axis_offset_px = {320, 240},
    .skew = 0
};

const HomographyProjection kProjection = HomographyProjection(kCameraTransform, kAxisMapping, kCameraCalibration);

std::vector<Eigen::Vector2f> convertVector(const std::vector<Eigen::Vector2d>& input)
{
    std::vector<Eigen::Vector2f> output;
    output.reserve(input.size()); // Reserve memory to avoid reallocation

    std::transform(input.begin(), input.end(), std::back_inserter(output),
                   [](const Eigen::Vector2d& v) { return v.cast<float>(); });

    return output;
}

void runRobot(DriveunitController& driveunit_controller, std::vector<Eigen::Vector2d> trajectory)
{
    if (trajectory.empty())
    {
        return;
    }
    
    if (trajectory.size() > 1)
    {
        const float firstPointLength = trajectory[0].norm();
        const float lastPointLength = trajectory[1].norm();

        // Take closest end of the trajectory as start
        if (lastPointLength < firstPointLength)
        {
            std::reverse(trajectory.begin(), trajectory.end()); // Reverses the vector in place
        }
    }
    
    if (trajectory.size() > 5)
    {
        trajectory.resize(5);
    }

    const auto du_response = driveunit_controller.sendTrajectoryCommand(std::chrono::duration<float>(0), convertVector(trajectory));
    if (!du_response.has_value())
    {
        std::cerr << "Error sending trajectory: " << du_response.error().message() << std::endl;
    }
}

int main()
{
    //RepeatedImageFileSource video_source("../../sample_images/red_tape2.jpg");
    VideoCameraSource video_source(1);
    
    DriveunitTransport driveunit_transport;
    DriveunitController driveunit_controller(driveunit_transport);

    auto line_tracker = LineTracker<kDebug>(std::span(kTapeHsvThresholds));
    const auto sized_projection = kProjection.for_image({640, 480});

    driveunit_transport.open();

    video_source.start({640, 480});

    const auto start = std::chrono::high_resolution_clock::now(); // Start time

    std::optional<cv::Mat> captured_frame;
    while ((captured_frame = video_source.getFrame()).has_value())
    {
        const auto loop_start = std::chrono::high_resolution_clock::now(); // Start time

        const auto image_trajectory = line_tracker.extractTrajectory(captured_frame.value());
        std::vector<Eigen::Vector2d> world_trajectory(image_trajectory.size());
        sized_projection.project(std::span(image_trajectory), std::span(world_trajectory));

        const auto loop_end = std::chrono::high_resolution_clock::now(); // end time
        const double elapsed = std::chrono::duration_cast<std::chrono::microseconds>(loop_end - loop_start).count(); // Convert to milliseconds
        const double fps = 1000000.0 / elapsed;
        std::cout << "FPS: " << fps << std::endl;

        if constexpr (kDebug)
        {
            auto debug_data = line_tracker.debugData();
            std::vector<std::pair<std::string, cv::Mat>> debug_image_map = {
                std::make_pair("Original", captured_frame.value_or(cv::Mat()))
            };

            displayDebug(createDebugCanvas(debug_image_map));
        }

        runRobot(driveunit_controller, world_trajectory);
    }

    video_source.stop();

}
