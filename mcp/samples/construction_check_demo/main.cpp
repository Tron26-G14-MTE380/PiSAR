#include "pisar/math.h"
#include "pisar/vision/video_source.h"
#include "pisar/vision/line_tracker.h"
#include "pisar/vision/homography.h"
#include "pisar/vision/debug_visualization.h"

#include <easy/profiler.h>

#include <wiringPi.h>

#include <numbers>
#include <iostream>
#include <ranges>
#include <algorithm>
#include <span>
#include <chrono>


using namespace pisar::mcp;

constexpr bool kDebug = false;
constexpr uint64_t kProfileTimeMs = 10000;

constexpr int kLeftMotorControlPin = 10;
constexpr int kRightMotorControlPin = 9;

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

void runRobot(std::vector<Eigen::Vector2d> trajectory)
{
    const float firstPointLength = trajectory[0].norm();
    const float lastPointLength = trajectory[1].norm();

    // Take closest end of the trajectory as start
    if (lastPointLength < firstPointLength)
    {
        std::reverse(trajectory.begin(), trajectory.end()); // Reverses the vector in place
    }

    double x = trajectory[0].x();
    double y = trajectory[0].y();

    // Threshold to ignore tiny x deviations
    constexpr double kThreshold = 0.1;

    // Determine motor control
    bool left_motor_on = false;
    bool right_motor_on = false;

    if (x > kThreshold) {
        // Turn right: Stop left motor, Run right motor
        left_motor_on = false;
        right_motor_on = true;
    } else if (x < -kThreshold) {
        // Turn left: Run left motor, Stop right motor
        left_motor_on = true;
        right_motor_on = false;
    } else {
        // Move forward: Run both motors
        left_motor_on = true;
        right_motor_on = true;
    }

    const char* left_status = (left_motor_on ? "on" : "off");
    const char* right_status = (right_motor_on ? "on" : "off");
    std::cout << "Left: " << left_status << ", Right: " << right_status << std::endl;

    // Apply to GPIO
    digitalWrite(kLeftMotorControlPin, left_motor_on ? HIGH : LOW);
    digitalWrite(kRightMotorControlPin, right_motor_on ? HIGH : LOW);
}


int main()
{
    RepeatedImageFileSource video_source("../../sample_images/red_tape2.jpg");
    auto line_tracker = LineTracker<kDebug>(std::span(kTapeHsvThresholds));
    const auto sized_projection = kProjection.for_image({640, 480});

    wiringPiSetup();  // Initialize WiringPi
    wiringPiSetupGpio();
    pinMode(kLeftMotorControlPin, OUTPUT);
    pinMode(kRightMotorControlPin, OUTPUT);

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
                std::make_pair("Original", captured_frame.value_or(cv::Mat())),
                std::make_pair("Preprocessed", debug_data.preprocessed),
                std::make_pair("HSV Filtered", debug_data.hsvFiltered),
                std::make_pair("Skeleton", debug_data.skeleton),
                std::make_pair("Filtered Skeleton", debug_data.filtered_skeleton),
                std::make_pair("Trajectory", debug_data.trajectory),
                std::make_pair("Simplified Trajectory", debug_data.simplified_trajectory),
                std::make_pair("Trajectory Homography Projection", createHomographyProjectionVisualization(640, 480, sized_projection, world_trajectory))
            };

            displayDebug(createDebugCanvas(debug_image_map));
        }

        runRobot(world_trajectory);
    }

    video_source.stop();

}
