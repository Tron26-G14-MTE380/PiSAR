#include "pisar/math.h"
#include "pisar/vision/video_source.h"
#include "pisar/vision/line_tracker.h"
#include "pisar/vision/homography.h"
#include "pisar/vision/roi_mat.h"
#include "pisar/vision/debug_visualization.h"
#include "pisar/vision/trajectory_filter.h"

#include <easy/profiler.h>

#include <numbers>
#include <iostream>
#include <ranges>
#include <algorithm>
#include <span>
#include <chrono>

using namespace pisar::mcp;

constexpr bool kDebug = true;
constexpr bool kProfile = false;
constexpr uint64_t kProfileTimeMs = 10000;

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



int main()
{
    if constexpr(kProfile)
    {
        EASY_PROFILER_ENABLE;
    }

    HomographyProjection projection = HomographyProjection(kCameraTransform, kAxisMapping, kCameraCalibration);
    TrajectoryFilter<double> filter;

    auto line_tracker = LineTracker<kDebug>({640, 480}, std::span(kTapeHsvThresholds), projection, filter);

    //RepeatedImageFileSource video_source("../../sample_images/red_tape2.jpg");
    VideoFileSource video_source("../../sample_images/track_video.mp4");
    video_source.start({640, 480});

    const auto start = std::chrono::high_resolution_clock::now(); // Start time

    std::optional<cv::Mat> captured_frame;
    while ((captured_frame = video_source.getFrame()).has_value())
    {
        const auto loop_start = std::chrono::high_resolution_clock::now(); // Start time
        const std::chrono::duration<float> frame_capture_time = loop_start.time_since_epoch();

        const std::vector<Eigen::Vector2d> world_trajectory = line_tracker.extractTrajectory(captured_frame.value(), frame_capture_time);

        const auto loop_end = std::chrono::high_resolution_clock::now(); // end time
        const double elapsed = std::chrono::duration_cast<std::chrono::microseconds>(loop_end - loop_start).count(); // Convert to milliseconds
        const double fps = 1000000.0 / elapsed;
        std::cout << "FPS: " << fps << std::endl;

        if constexpr (kDebug)
        {
            auto debug_data = line_tracker.debugData();
            std::vector<std::pair<std::string, RoiMat>> debug_image_map = {
                std::make_pair("1. Original", RoiMat(captured_frame.value_or(cv::Mat()))),
                std::make_pair("2. Preprocessed", debug_data.preprocessed),
                std::make_pair("3. HSV Filtered", debug_data.hsvFiltered),
                std::make_pair("4. Skeleton", debug_data.skeleton),
                std::make_pair("5. Filtered Skeleton", debug_data.filtered_skeleton),
                std::make_pair("6. Trajectory", debug_data.trajectory),
                std::make_pair("7. Simplified Trajectory", debug_data.simplified_trajectory),
                std::make_pair("8. Projected Trajectory", debug_data.projected_trajectory),
                std::make_pair("9. Filtered Trajectory", debug_data.filtered_trajectory),
                std::make_pair("10. Simplified Trajectory", debug_data.simplified_filtered_trajectory)
            };


            displayDebug(createDebugCanvas(debug_image_map, 480));
        }

        if constexpr (kProfile)
        {
            const auto end = std::chrono::high_resolution_clock::now(); // end time
            const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count(); // Convert to milliseconds

            if (elapsed > kProfileTimeMs)
            {
                profiler::dumpBlocksToFile("profile.prof");
                break;
            }
        }
    }

    video_source.stop();

}