#include "pisar/mcp/config.h"
#include "pisar/mcp/vision/camera_source.h"
#include "pisar/mcp/vision/line_tracker.h"
#include "pisar/mcp/driveunit/controller.h"

#include "pisar/mcp/statemachine/statemachine.h"
#include "pisar/mcp/statemachine/statemachine_controller.h"

#include <numbers>
#include <iostream>
#include <ranges>
#include <algorithm>
#include <span>
#include <chrono>


using namespace pisar::mcp;

constexpr bool kDebug = false;

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

    //HsvColorExtractor line_color_extractor(gkRedTapeHsvThresholds)
    YuvColorExtractor line_color_extractor(gkRedTapeYuvThresholds);
    using LineColorExtractorT = decltype(line_color_extractor);

    auto line_tracker = LineTracker<CapturedFrame::TimestampT, LineColorExtractorT, kDebug>(
        gkCamCaptureConfig.downscaledSize(),
        line_color_extractor,
        sized_projection,
        filter
    );

    const YuvColorExtractor target_color_extractor(std::span(&gkBullseyeWhiteYuvMask, 1));
    using TargetColorExtractorT = decltype(target_color_extractor);

    auto target_tracker = TargetTracker<CapturedFrame::TimestampT, TargetColorExtractorT, kDebug>(
        target_color_extractor,
        sized_projection,
        gkTargetGrabDistanceOffset
    );

    auto target_detector = TargetDetector(target_tracker);

    DriveunitTransport driveunit_transport;
    DriveunitController driveunit_controller(driveunit_transport);

    RobotContext sm_context(video_source, line_tracker, target_detector, target_tracker, driveunit_controller);
    StateMachineController sm_controller(sm_context);

    driveunit_transport.open();
    video_source.start();

    while (sm_controller.update() == false) {}
}
