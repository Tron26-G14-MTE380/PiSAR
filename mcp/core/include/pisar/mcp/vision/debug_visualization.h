#pragma once

#include "pisar/mcp/vision/homography.h"
#include "pisar/mcp/vision/roi_mat.h"

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

namespace pisar::mcp {

cv::Mat createTrajectoryVisualization(
    const cv::Size& image_size,
    const std::vector<Eigen::Vector2i>& points,
    const cv::Scalar& color,
    const std::optional<cv::Scalar>& first_point_color,
    const std::optional<cv::Scalar>& last_point_color
);

cv::Mat createHomographyProjectionVisualization(
    cv::Size image_size,
    const HomographySizedProjection& projection,
    const std::span<const Eigen::Vector2d>& trajectory
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

bool windowClosed(const std::string_view window_name);
bool windowClosedOrEsc(const std::string_view window_name, const int key);
void terminateOnWindowCloseOrEsc(const std::string_view window_name, const int key);

}