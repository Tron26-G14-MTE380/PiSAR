#include "pisar/mcp/config.h"
#include "pisar/mcp/vision/camera_source.h"
#include "pisar/mcp/vision/line_tracker.h"
#include "pisar/mcp/driveunit/controller.h"

#include <numbers>
#include <iostream>
#include <ranges>
#include <algorithm>
#include <span>
#include <chrono>


using namespace pisar::mcp;

constexpr bool kDebug = false;
constexpr double kFrameRateLimit = 120.0;
constexpr auto kFrameDurationLimit = std::chrono::duration<double>(1.0 / kFrameRateLimit);

std::vector<Eigen::Vector2f> convertVector(const std::vector<Eigen::Vector2d>& input)
{
    std::vector<Eigen::Vector2f> output;
    output.reserve(input.size()); // Reserve memory to avoid reallocation

    std::transform(input.begin(), input.end(), std::back_inserter(output),
                   [](const Eigen::Vector2d& v) { return v.cast<float>(); });

    return output;
}

void runRobot(DriveunitController& driveunit_controller, std::vector<Eigen::Vector2d> trajectory, CapturedFrame::TimestampT frame_time)
{
    if (trajectory.empty())
    {
        return;
    }

    if (trajectory.size() > gkMaxTrajectoryPoints)
    {
        trajectory.resize(gkMaxTrajectoryPoints);
    }

    const auto reference_time = std::chrono::duration_cast<pisar::driveunit_interface::CommandFollowTrajectory::ReferenceTimeT>(CapturedFrame::ClockT::now() - frame_time);
    const auto du_response = driveunit_controller.sendTrajectoryCommand(reference_time, convertVector(trajectory));
    if (!du_response.has_value())
    {
        std::cerr << "Error sending trajectory: " << du_response.error().message() << std::endl;
    }
}

int main()
{
    HomographyProjection projection = HomographyProjection(
        gkCameraTransform,
        gkCameraAxisMapping,
        gkCameraCalibration
    );

    TrajectoryFilter<double, CapturedFrame::TimestampT> filter;

    #if __linux__
        LibcameraCameraSource video_source(gkCamCaptureConfig, "/base/axi/pcie@120000/rp1/i2c@80000/imx219@10");
    #else
        //CvCameraSource video_source(gkCamCaptureConfig, 0);
        //RepeatedImageFileCameraSource video_source(gkCamCaptureConfig, "../../sample_images/red_tape2.jpg");
        VideoFileCameraSource video_source(gkCamCaptureConfig, "../../sample_images/track_video.mp4");
    #endif

    const auto sized_projection = projection.forCapture(video_source.captureConfig());

    //HsvColorExtractor color_extractor(gkRedTapeHsvThresholds)
    YuvColorExtractor color_extractor(gkRedTapeYuvThresholds);
    using ColorExtractorT = decltype(color_extractor);

    auto line_tracker = LineTracker<CapturedFrame::TimestampT, ColorExtractorT, kDebug>(
        gkCamCaptureConfig.downscaledSize(),
        color_extractor,
        sized_projection,
        filter
    );

    DriveunitTransport driveunit_transport;
    DriveunitController driveunit_controller(driveunit_transport);

    driveunit_transport.open();
    video_source.start();

    const auto start = std::chrono::high_resolution_clock::now(); // Start time

    std::optional<CapturedFrame> captured_frame;
    while ((captured_frame = video_source.getFrame()).has_value())
    {
        const auto loop_start = std::chrono::high_resolution_clock::now(); // Start time

        const std::vector<Eigen::Vector2d> world_trajectory = line_tracker.extractTrajectory(captured_frame.value().frame, captured_frame.value().timestamp);

        const double elapsed = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - loop_start).count(); // Convert to milliseconds
        const double fps = 1000000.0 / elapsed;
        std::cout << "extractTrajectory FPS: " << fps << std::endl;

        runRobot(driveunit_controller, world_trajectory, captured_frame.value().timestamp);

        if constexpr (kDebug)
        {
            auto debug_data = line_tracker.debugData();
            std::vector<std::pair<std::string, RoiMat>> debug_image_map = {
                std::make_pair("1. Original", RoiMat(captured_frame.value().frame)),
                std::make_pair("2. Preprocessed", debug_data.preprocessed),
                std::make_pair("3. Color Extracted", debug_data.extractedColor),
                std::make_pair("4. Skeleton", debug_data.skeleton),
                std::make_pair("5. Filtered Skeleton", debug_data.filtered_skeleton),
                std::make_pair("6. Trajectory", debug_data.trajectory),
                std::make_pair("7. Simplified Trajectory", debug_data.simplified_trajectory),
                std::make_pair("8. Ordered Trajectory", debug_data.ordered_trajectory),
                std::make_pair("9. Projected Trajectory", debug_data.projected_trajectory),
                std::make_pair("10. Filtered Trajectory", debug_data.filtered_trajectory),
                std::make_pair("11. Simplified Trajectory", debug_data.simplified_filtered_trajectory)
            };


            displayDebug(createDebugCanvas(debug_image_map, gkCamCaptureConfig.downscaledSize().width));
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
