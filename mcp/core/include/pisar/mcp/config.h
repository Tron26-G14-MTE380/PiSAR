#pragma once

#include "pisar/mcp/vision/camera.h"
#include "pisar/mcp/utils/math.h"

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

namespace pisar::mcp {


static const std::array<std::pair<cv::Scalar, cv::Scalar>, 2> gkRedTapeHsvThresholds = {
    std::make_pair(cv::Scalar(0, 120, 70), cv::Scalar(10, 255, 255)),
    std::make_pair(cv::Scalar(170, 120, 70), cv::Scalar(180, 255, 255))
};

// static const std::array<std::pair<cv::Scalar, cv::Scalar>, 2> gkRedTapeYuvThresholds = {
//     std::make_pair(cv::Scalar(0, 110, 180), cv::Scalar(255, 140, 255))
// };

static const std::array<std::pair<cv::Scalar, cv::Scalar>, 2> gkRedTapeYuvThresholds = {
    std::make_pair(cv::Scalar(0, 0, 145), cv::Scalar(255, 255, 255))
};

static const CameraTransform gkCameraTransform = {
    .position = {0, 10.0 / 100.0, 15.8 / 100.0},
    .tilt = Eigen::AngleAxisd(pisar::mcp::degToRad<double>(-70), Eigen::Vector3d::UnitX())
};

static const auto gkCameraAxisMapping = Eigen::Matrix3d{
    {1, 0, 0},
    {0, 0, 1},
    {0, 1, 0}
};

// RPI Camera v2 calibration done at full-res
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

// RPI Camera v2 capture config for mode 4
static const auto gkRpiCamV2Mode4CaptureConfig = CameraCaptureConfig(
    cv::Size(3280, 2464),       // Camera Full resolution
    2,                          // binning mode
    std::nullopt,               // Capture crop offset
    cv::Size(1640, 1232),       // Capture crop size
    4,                          // Downscale size
    40                          // Framerate
);

// RPI Camera v2 capture config for mode 6
static const auto gkRpiCamV2Mode6CaptureConfig = CameraCaptureConfig(
    cv::Size(3280, 2464),       // Camera Full resolution
    2,                          // binning mode
    std::nullopt,               // Capture crop offset
    cv::Size(1280, 720),        // Capture crop size
    4,                          // Downscale size
    90                          // Framerate
);

static const auto gkCamCaptureConfig = gkRpiCamV2Mode4CaptureConfig;

// Bullseye/Target detection
static const std::pair<cv::Scalar, cv::Scalar> gkBullseyeWhiteHsvMask =
    std::make_pair(cv::Scalar(0, 0, 220), cv::Scalar(180, 40, 255));

static const std::pair<cv::Scalar, cv::Scalar> gkBullseyeWhiteYuvMask =
    std::make_pair(cv::Scalar(200, 0, 0), cv::Scalar(255, 255, 255));

// Interfacing settings
static const size_t gkMaxTrajectoryPoints = 5;
}
