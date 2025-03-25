
#include "pisar/mcp/config.h"
#include "pisar/mcp/vision/video_source.h"
#include "pisar/mcp/vision/line_tracker.h"
#include "pisar/mcp/vision/homography.h"
#include "pisar/mcp/vision/roi_mat.h"
#include "pisar/mcp/vision/debug_visualization.h"
#include "pisar/mcp/vision/trajectory_filter.h"

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
constexpr double kFrameRateLimit = 1.0;
constexpr auto kFrameDurationLimit = std::chrono::duration<double>(1.0 / kFrameRateLimit);

int main()
{
    if constexpr(kProfile)
    {
        EASY_PROFILER_ENABLE;
    }

    HomographyProjection projection = HomographyProjection(
        gkCameraTransform,
        gkCameraAxisMapping,
        gkCameraCalibration
    );

    TrajectoryFilter<double> filter;


    #if __linux__
        LibcameraVideoSource video_source("/base/axi/pcie@120000/rp1/i2c@80000/imx219@10");
        std::optional<cv::Rect> frame_crop = computeCenterCrop(gkFullFrameSize, gkCaptureFrameSize);
    #else
        //CvCameraVideoSource video_source(0);
        //RepeatedImageFileSource video_source("../../sample_images/red_tape2.jpg");
        VideoFileSource video_source("../../sample_images/track_video.mp4");
        std::optional<cv::Rect> frame_crop = std::nullopt;
    #endif

    video_source.start(gkCaptureFrameSize);

    auto line_tracker = LineTracker<kDebug>(
        gkFrameSize,
        gkRedTapeHsvThresholds,
        projection.for_image(gkFrameSize, frame_crop),
        filter
    );

    const auto start = std::chrono::high_resolution_clock::now(); // Start time

    std::optional<cv::Mat> captured_frame;
    while ((captured_frame = video_source.getFrame()).has_value())
    {
        const auto loop_start = std::chrono::high_resolution_clock::now(); // Start time
        const std::chrono::duration<float> frame_capture_time = loop_start - start;

        // TODO: account for any cropping that happens here
        cv::Mat input_frame = downscaleCrop(captured_frame.value(), gkFrameSize);
        const std::vector<Eigen::Vector2d> world_trajectory = line_tracker.extractTrajectory(input_frame, frame_capture_time);

        const double elapsed = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - loop_start).count(); // Convert to milliseconds
        const double fps = 1000000.0 / elapsed;
        std::cout << "FPS: " << fps << std::endl;

        if constexpr (kDebug)
        {
            auto debug_data = line_tracker.debugData();
            std::vector<std::pair<std::string, RoiMat>> debug_image_map = {
                std::make_pair("1. Original", RoiMat(input_frame)),
                std::make_pair("2. Preprocessed", debug_data.preprocessed),
                std::make_pair("3. HSV Filtered", debug_data.hsvFiltered),
                std::make_pair("4. Skeleton", debug_data.skeleton),
                std::make_pair("5. Filtered Skeleton", debug_data.filtered_skeleton),
                std::make_pair("6. Trajectory", debug_data.trajectory),
                std::make_pair("7. Simplified Trajectory", debug_data.simplified_trajectory),
                std::make_pair("8. Ordered Trajectory", debug_data.ordered_trajectory),
                std::make_pair("9. Projected Trajectory", debug_data.projected_trajectory),
                std::make_pair("10. Filtered Trajectory", debug_data.filtered_trajectory),
                std::make_pair("11. Simplified Trajectory", debug_data.simplified_filtered_trajectory)
            };


            displayDebug(createDebugCanvas(debug_image_map, 320));
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

        auto loop_end = std::chrono::high_resolution_clock::now();
        auto loop_duration = std::chrono::duration_cast<std::chrono::duration<double>>(loop_end - loop_start);
        while (loop_duration < kFrameDurationLimit)
        {
            loop_end = std::chrono::high_resolution_clock::now();
            loop_duration = std::chrono::duration_cast<std::chrono::duration<double>>(loop_end - loop_start);
        }
    }

    video_source.stop();

}
