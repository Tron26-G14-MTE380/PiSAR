#pragma once

#include <opencv2/opencv.hpp>

namespace pisar::mcp {

    /// @brief Computes the bounding box of nonzero pixels in a binary image.
    /// @param img Input binary image (8-bit, single-channel).
    /// @return Bounding box as cv::Rect.
    cv::Rect computeBoundingBox(const cv::Mat& img);

}