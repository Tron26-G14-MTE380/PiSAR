#pragma once

#include "pisar/vision/homography.h"
#include "pisar/vision/roi_mat.h"

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

namespace pisar::mcp {

void createTrajectoryVisualization(cv::InputOutputArray output, const std::vector<Eigen::Vector2i>& points, const cv::Scalar& color);

cv::Mat createHomographyProjectionVisualization(
    cv::Size image_size,
    const HomographySizedProjection& projection,
    const std::vector<Eigen::Vector2d>& trajectory
);

std::vector<std::pair<std::string, cv::Mat>> createDebugImageMap();

cv::Mat createDebugCanvas(
    const std::vector<std::pair<std::string, cv::Mat>>& debug_images,
    int max_size = 640, int grid_padding = 10
);

cv::Mat createDebugCanvas(
    const std::vector<std::pair<std::string, RoiMat>>& debug_images,
    int max_size = 640, int grid_padding = 10
);

void displayDebug(const cv::Mat debug_canvas);

}