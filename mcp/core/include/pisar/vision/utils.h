#pragma once

#include <opencv2/opencv.hpp>

namespace pisar::mcp {

    /// @brief Computes the bounding box of nonzero pixels in a binary image.
    /// @param img Input binary image (8-bit, single-channel).
    /// @return Bounding box as cv::Rect.
    cv::Rect computeBoundingBox(const cv::Mat& img);

    /**
     * @brief Resizes an image while maintaining aspect ratio and adding padding.
     * @param input The input image frame.
     * @param target_size The desired output size (width, height).
     * @return The resized and padded image.
     */
    cv::Mat resizeWithPadding(const cv::Mat& input, const cv::Size& target_size);


}