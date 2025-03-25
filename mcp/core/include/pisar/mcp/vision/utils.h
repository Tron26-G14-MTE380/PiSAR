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

    /**
     * @brief Downscale an image to fit the target size using a single scale factor, cropping excess areas.
     * @param input The input high-resolution image.
     * @param target_size The desired target size as cv::Size.
     * @return The downscaled and cropped image.
     */
    cv::Mat downscaleCrop(const cv::Mat &input, const cv::Size &target_size);

    /**
     * @brief Computes the crop rect of an image from the full size image in the center.
     * @param full_size The size of the full res image the image from cropped from.
     * @param cropped_size The size of the cropped region.
     * @return Rect describing the crop region.
     */
    [[nodiscard]] cv::Rect computeCenterCrop(const cv::Size& full_size, const cv::Size& cropped_size);

}
