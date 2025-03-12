#pragma once

#include "pisar/vision/homography.h"
#include "pisar/vision/line_tracker.h"

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

namespace pisar::mcp {

cv::Mat createHomographyProjectionVisualization(
    cv::Size2i image_size,
    const HomographySizedProjection& projection,
    const std::vector<Eigen::Vector2d>& trajectory
);

std::vector<std::pair<std::string, cv::Mat>> createDebugImageMap();

cv::Mat createDebugCanvas(
    const std::vector<std::pair<std::string, cv::Mat>>& debug_images,
    int max_size = 640, int grid_padding = 10
);

void displayDebug(const cv::Mat debug_canvas);

}