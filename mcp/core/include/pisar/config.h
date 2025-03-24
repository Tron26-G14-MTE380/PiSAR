#pragma once

#include "pisar/vision/camera.h"
#include "pisar/math.h"

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

namespace pisar::mcp {


static const std::array<std::pair<cv::Scalar, cv::Scalar>, 2> gkRedTapeHsvThresholds = {
    std::make_pair(cv::Scalar(0, 120, 70), cv::Scalar(10, 255, 255)),
    std::make_pair(cv::Scalar(170, 120, 70), cv::Scalar(180, 255, 255))
};

static const CameraTransform gkCameraTransform = {
    .position = {0, 9.5, 12},
    .tilt = Eigen::AngleAxisd(pisar::mcp::deg_to_rad<double>(-48), Eigen::Vector3d::UnitX())
};

static const auto gkCameraAxisMapping = Eigen::Matrix3d{
    {1, 0, 0},
    {0, 0, 1},
    {0, 1, 0}
};

static const CameraCalibrationData gkCameraCalibration = {
    .calibration_img_size_px = {3280, 2464},
    .focal_length_px = {2630.73, 2621.95},
    .principle_axis_offset_px = {1589.29, 1085.99},
    .skew = 0,
    .distortion_coeffs = {
        .k1 = 0.13149815,
        .k2 = 0.40868211,
        .p1 = -0.00805099,
        .p2 = -0.00454671,
        .k3 = -2.73795495
    }
};

static const cv::Size gkCaptureFrameSize = {1280, 960};
static const cv::Size gkFrameSize = {320, 240};
}