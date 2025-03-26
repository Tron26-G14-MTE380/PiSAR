#include "pisar/mcp/config.h"
#include "pisar/mcp/vision/video_source.h"
#include "pisar/mcp/vision/line_tracker.h"
#include "pisar/mcp/driveunit/controller.h"

#include <numbers>
#include <iostream>
#include <ranges>
#include <algorithm>
#include <span>
#include <chrono>


using namespace pisar::mcp;

constexpr bool kDebug = true;
constexpr double kFrameRateLimit = 10.0;
constexpr auto kFrameDurationLimit = std::chrono::duration<double>(1.0 / kFrameRateLimit);

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

    if (trajectory.size() > gkMaxTrajectoryPoints)
    {
        trajectory.resize(gkMaxTrajectoryPoints);
    }

    const auto du_response = driveunit_controller.sendTrajectoryCommand(std::chrono::duration<float>(0), convertVector(trajectory));
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
        LibcameraVideoSource video_source("/base/axi/pcie@120000/rp1/i2c@80000/imx219@10");
    #else
        //CvCameraVideoSource video_source(0);
        //RepeatedImageFileSource video_source("../../sample_images/red_tape2.jpg");
        VideoFileSource video_source("../../sample_images/track_video.mp4");
    #endif

    std::optional<cv::Rect> frame_crop = computeCenterCrop(gkFullFrameSize, gkCaptureFrameSize);

    //HsvColorExtractor color_extractor(gkRedTapeHsvThresholds)
    YuvColorExtractor color_extractor(gkRedTapeYuvThresholds);
    using ColorExtractorT = decltype(color_extractor);

    auto line_tracker = LineTracker<CapturedFrame::TimestampT, ColorExtractorT, kDebug>(
        gkFrameSize,
        color_extractor,
        projection.for_image(gkFrameSize, frame_crop),
        filter
    );

    DriveunitTransport driveunit_transport;
    DriveunitController driveunit_controller(driveunit_transport);

    driveunit_transport.open();
    video_source.start(gkCaptureFrameSize);

    const auto start = std::chrono::high_resolution_clock::now(); // Start time

    std::optional<CapturedFrame> captured_frame;
    while ((captured_frame = video_source.getFrame()).has_value())
    {
        const auto loop_start = std::chrono::high_resolution_clock::now(); // Start time

        // TODO: account for any cropping that happens here
        cv::Mat input_frame = downscaleCrop(captured_frame.value().frame, gkFrameSize);
        const std::vector<Eigen::Vector2d> world_trajectory = line_tracker.extractTrajectory(input_frame, captured_frame.value().timestamp);

        const double elapsed = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - loop_start).count(); // Convert to milliseconds
        const double fps = 1000000.0 / elapsed;
        std::cout << "extractTrajectory FPS: " << fps << std::endl;

        if constexpr (kDebug)
        {
            auto debug_data = line_tracker.debugData();
            std::vector<std::pair<std::string, RoiMat>> debug_image_map = {
                std::make_pair("1. Original", RoiMat(input_frame)),
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


            displayDebug(createDebugCanvas(debug_image_map, 320));
        }

        runRobot(driveunit_controller, world_trajectory);

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
