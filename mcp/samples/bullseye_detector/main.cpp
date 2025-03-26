#include "pisar/vision/bullseye_detector.h"
#include "pisar/vision/video_source.h"
#include "pisar/vision/homography.h"
#include "pisar/vision/camera.h"

#include <opencv2/opencv.hpp>
#include <iostream>

using namespace pisar::mcp;

int main() {
    const CameraTransform cam_transform {
        .position = {0, 5, 45},
        .tilt = Eigen::AngleAxisd(pisar::mcp::deg_to_rad<double>(-40), Eigen::Vector3d::UnitX())
    };

    const Eigen::Matrix3d axis_mapping {
        {1, 0, 0},
        {0, 0, 1},
        {0, 1, 0}
    };

    const CameraCalibrationData cam_calib {
        .calibration_img_size_px = {640, 480},
        .focal_length_px = {450, 450},
        .principle_axis_offset_px = {320, 240},
        .skew = 0
    };

    HomographyProjection projection(cam_transform, axis_mapping, cam_calib);
    auto sized_projection = projection.for_image({640, 480});
    BullseyeDetector detector(sized_projection);

    VideoCameraSource video_source(1);
    video_source.start({640, 480});

    while (true) {
        auto frame_opt = video_source.getFrame();
        if (!frame_opt.has_value()) break;

        auto frame = frame_opt.value();
        int crop_start = frame.rows / 3;
        cv::Mat cropped = frame(cv::Range(crop_start, frame.rows), cv::Range::all());

        auto center_opt = detector.findBullseye(cropped);
        if (center_opt.has_value()) {
            auto center = center_opt.value();
            cv::circle(cropped, center, 20, {0, 255, 0}, 3);
            cv::putText(cropped, "Bullseye", center - cv::Point(40, 20),
                        cv::FONT_HERSHEY_SIMPLEX, 0.8, {0, 255, 0}, 2);
        }

        cv::imshow("Bullseye Detection", cropped);
        if (cv::waitKey(30) == 'q') break;
    }

    video_source.stop();
    return 0;
}