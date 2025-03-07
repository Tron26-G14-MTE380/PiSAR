#include "pisar/math.h"
#include "pisar/vision/video_source.h"
#include "pisar/vision/line_tracker.h"
#include "pisar/vision/homography.h"

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

cv::Mat createHomographyProjectionVisualization(int width, int height, const HomographySizedProjection& projection, const std::vector<Eigen::Vector2d>& trajectory)
{
    // Create a blank image
    cv::Mat visualization = cv::Mat::zeros(height, width, CV_8UC3);

    // Step 1: Generate a grid of points for visualization
    std::vector<Eigen::Vector2i> camera_view_grid_image_points;
    for (int i = 0; i < 10; ++i)
    {
        for (int j = 0; j < 10; ++j)
        {
            double u = i * (width / 9.0);
            double v = j * (height / 9.0);
            camera_view_grid_image_points.emplace_back(static_cast<int>(u), static_cast<int>(v));
        }
    }

    // Project the grid into world space
    std::vector<Eigen::Vector2d> camera_view_grid_world_points = projection.project(std::span(camera_view_grid_image_points));

    // Combine trajectory and projected grid points for world bounds computation
    std::vector<Eigen::Vector2d> all_world_points = trajectory;
    all_world_points.insert(all_world_points.end(), camera_view_grid_world_points.begin(), camera_view_grid_world_points.end());

    // Find world bounds
    double X_min = std::numeric_limits<double>::max(), X_max = std::numeric_limits<double>::lowest();
    double Y_min = std::numeric_limits<double>::max(), Y_max = std::numeric_limits<double>::lowest();

    for (const auto& pt : all_world_points) {
        X_min = std::min(X_min, pt.x());
        X_max = std::max(X_max, pt.x());
        Y_min = std::min(Y_min, pt.y());
        Y_max = std::max(Y_max, pt.y());
    }

    // Compute scale factors
    double world_x_range = X_max - X_min;
    double world_y_range = Y_max - Y_min;
    double world_to_image_scale_factor_x = width / world_x_range;
    double world_to_image_scale_factor_y = height / world_y_range;
    double world_to_image_scale_factor = std::min(world_to_image_scale_factor_x, world_to_image_scale_factor_y);

    // Function to convert world points to image space
    auto worldToImageFixed = [&](const std::vector<Eigen::Vector2d>& world_points) {
        std::vector<cv::Point2f> image_points;
        for (const auto& pt : world_points) {
            float x_scaled = static_cast<float>(pt.x() * world_to_image_scale_factor + width / 2);
            float y_scaled = static_cast<float>(height - (pt.y() * world_to_image_scale_factor));
            image_points.emplace_back(x_scaled, y_scaled);
        }
        return image_points;
    };

    // Convert points to image space
    std::vector<cv::Point2f> image_grid_points = worldToImageFixed(camera_view_grid_world_points);
    std::vector<cv::Point2f> image_trajectory = worldToImageFixed(trajectory);

    // Draw grid points
    for (const auto& pt : image_grid_points) {
        cv::circle(visualization, pt, 1, cv::Scalar(255, 255, 255), -1);
    }

    // Draw trajectory
    for (size_t i = 0; i < image_trajectory.size(); ++i) {
        cv::circle(visualization, image_trajectory[i], 3, cv::Scalar(0, 0, 255), -1); // Red points
        if (i > 0) {
            cv::line(visualization, image_trajectory[i - 1], image_trajectory[i], cv::Scalar(0, 0, 255), 2);
        }
    }

    return visualization;
}

cv::Mat createDebugCanvas(const std::vector<std::pair<std::string, cv::Mat>>& debug_images, int max_size = 640, int grid_padding = 10)
{
    if (debug_images.empty()) {
        return cv::Mat::ones(100, 100, CV_8UC3) * 255; // Return a blank white canvas if empty
    }

    // Determine minimum width and height
    int min_width = std::numeric_limits<int>::max();
    int min_height = std::numeric_limits<int>::max();

    for (const auto& [key, img] : debug_images)
    {
        min_width = std::min(min_width, img.size().width);
        min_height = std::min(min_height, img.size().height);
    }

    // Ensures no image exceeds 320px in width/height
    double scale_factor = (std::max(min_width, min_height) > max_size) ?
                          static_cast<double>(max_size) / static_cast<double>(std::max(min_width, min_height)) : 1.0;

    // Resize images while maintaining aspect ratio
    std::vector<cv::Mat> resized_images(debug_images.size());
    std::transform(debug_images.begin(), debug_images.end(), resized_images.begin(),
                   [scale_factor](const auto& pair) {
                       cv::Mat resized;
                       cv::resize(pair.second, resized, cv::Size(), scale_factor, scale_factor, cv::INTER_LINEAR);
                       return resized;
                   });

    // Convert grayscale to BGR for consistency
    for (auto& img : resized_images)
    {
        if (img.channels() == 1) {
            cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
        }
    }

    // Determine grid layout
    const int num_images = resized_images.size();
    const int grid_cols = 4;  // Number of columns in the grid
    const int grid_rows = std::ceil(static_cast<double>(num_images) / grid_cols); // Correct floating-point division

    const cv::Size img_size = resized_images[0].size();

    // Create an empty canvas for the grid with padding
    const cv::Size grid_size = {
        (img_size.width + grid_padding) * grid_cols - grid_padding,
        (img_size.height + grid_padding) * grid_rows - grid_padding
    };

    cv::Mat grid_canvas = cv::Mat::ones(grid_size, CV_8UC3) * 255;

    // Place images in the grid with padding
    for (int i = 0; i < resized_images.size(); ++i)
    {
        const int row = i / grid_cols;
        const int col = i % grid_cols;

        cv::Point start_point = {
            col * (img_size.width + grid_padding),
            row * (img_size.height + grid_padding),
        };

        resized_images[i].copyTo(
            grid_canvas(cv::Rect(start_point.x, start_point.y, resized_images[i].cols, resized_images[i].rows))
        );

        // Add text labels
        cv::putText(
            grid_canvas, debug_images[i].first, {start_point.x + 10, start_point.y + 20},
            cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 255, 0}, 1, cv::LINE_AA
        );
    }

    return grid_canvas;
}

void displayDebug(const cv::Mat debug_canvas)
{
    // Show the composite debug display
    cv::imshow("Debug Data", debug_canvas);
    const int key = cv::waitKey(1);  // Small delay to allow window to refresh

    if (key == 27 || cv::getWindowProperty("Debug Data", cv::WND_PROP_VISIBLE) <= 0)  // ESC key or window closed
    {
        std::cout << "Debug window closed. Exiting program." << std::endl;
        cv::destroyAllWindows();
        std::terminate();
    }
}

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
