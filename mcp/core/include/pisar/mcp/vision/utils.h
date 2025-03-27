#pragma once

#include <opencv2/opencv.hpp>

#include <span>

namespace pisar::mcp {

    /// @brief Computes the bounding box of nonzero pixels in a binary image.
    /// @param img Input binary image (8-bit, single-channel).
    /// @return Bounding box as cv::Rect.
    [[nodiscard]] cv::Rect computeBoundingBox(const cv::Mat& img);

    /**
     * @brief Resizes an image while maintaining aspect ratio and adding padding.
     * @param input The input image frame.
     * @param target_size The desired output size (width, height).
     * @return The resized and padded image.
     */
    [[nodiscard]] cv::Mat resizeWithPadding(const cv::Mat& input, const cv::Size& target_size);

    /**
     * @brief Resize an image to fit the target size using a single scale factor, and cropping.
     * @param input The input image.
     * @param target_size The desired target size as cv::Size.
     * @return The resized and cropped image.
     */
    [[nodiscard]] cv::Mat resizeCropMaintainRatio(const cv::Mat &input, const cv::Size &target_size);

    /**
     * @brief Computes the crop rect of an image from the full size image in the center.
     * @param full_size The size of the full res image the image from cropped from.
     * @param cropped_size The size of the cropped region.
     * @return Rect describing the crop region.
     */
    [[nodiscard]] cv::Rect computeCenterCrop(const cv::Size& full_size, const cv::Size& cropped_size);

    /**
     * @brief Creates a binary mask by applying mask thresholds on the image. The thresholds are combined using OR.
     *
     * @param input The input image to mask.
     * @param mask_thresholds The mask thresholds to apply to the input.
     */
    [[nodiscard]] cv::Mat createBinaryMaskOr(
        const cv::Mat& input,
        const std::span<const std::pair<cv::Scalar, cv::Scalar>>& mask_thresholds
    );

}
