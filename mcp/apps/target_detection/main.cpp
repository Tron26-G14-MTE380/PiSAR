#include "pisar/mcp/config.h"
#include "pisar/mcp/vision/camera_source.h"
#include "pisar/mcp/vision/target_tracker.h"
#include "pisar/mcp/vision/target_detector.h"

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
constexpr double kFrameRateLimit = 10.0;
constexpr auto kFrameDurationLimit = std::chrono::duration<double>(1.0 / kFrameRateLimit);

int main()
{
    if constexpr(kProfile)
    {
        EASY_PROFILER_ENABLE;
    }

    const HomographyProjection projection = HomographyProjection(
        gkCameraTransform,
        gkCameraAxisMapping,
        gkCameraCalibration
    );

    #if __linux__
        LibcameraCameraSource video_source(gkCamCaptureConfig, "/base/axi/pcie@120000/rp1/i2c@80000/imx219@10");
    #else
        //CvCameraSource video_source(gkCamCaptureConfig, 0);
        RepeatedImageFileCameraSource video_source(gkCamCaptureConfig, "../../sample_images/lego_on_bullseye.jpg");
        //VideoFileCameraSource video_source(gkCamCaptureConfig, "../../sample_images/track_video.mp4");
    #endif

    const auto sized_projection = projection.forCapture(video_source.captureConfig());

    //HsvColorExtractor color_extractor(gkBullseyeWhiteHsvMask)
    const YuvColorExtractor color_extractor(std::span(&gkBullseyeWhiteYuvMask, 1));
    using ColorExtractorT = decltype(color_extractor);

    auto target_tracker = TargetTracker<CapturedFrame::TimestampT, ColorExtractorT, kDebug>(
        color_extractor,
        sized_projection
    );

    video_source.start();
    const auto start = std::chrono::high_resolution_clock::now(); // Start time

    std::optional<CapturedFrame> captured_frame;
    while ((captured_frame = video_source.getFrame()).has_value())
    {
        const auto loop_start = std::chrono::high_resolution_clock::now(); // Start time

        const std::optional<Eigen::Vector2d> target_pos = target_tracker.track(captured_frame.value().frame, captured_frame.value().timestamp);

        if (target_pos)
        {
            std::cout << "World point at " << target_pos.value() << std::endl;
        }
        else
        {
            std::cout << "No target detected!" << std::endl;
        }

        const double elapsed = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - loop_start).count(); // Convert to milliseconds
        const double fps = 1000000.0 / elapsed;
        std::cout << "targetTrack FPS: " << fps << std::endl;

        if constexpr (kDebug)
        {
            auto debug_data = target_tracker.debugData();
            std::vector<std::pair<std::string, cv::Mat>> debug_image_map = {
                std::make_pair("1. Original", captured_frame.value().frame),
                std::make_pair("2. Color Extracted", debug_data.color_extracted),
                std::make_pair("3. Target Point", debug_data.target_point),
                std::make_pair("4. Projected Point", debug_data.projected_point)
            };

            displayDebug(createDebugCanvas(debug_image_map, 410));
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
