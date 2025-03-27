#pragma once

#include "pisar/mcp/vision/utils.h"

#include <opencv2/opencv.hpp>

namespace pisar::mcp {

/**
 * @brief Color extractor used to extract colors from images.
 *
 */
template<typename TDerived>
class ColorExtractor {
public:
    /**
     * @brief Extracts color from the image into a binary mask.
     *
     * @param input The input image to extract color from.
     * @return cv::Mat Binary mask of the extracted color.
     */
    [[nodiscard]] inline cv::Mat extract(const cv::Mat& input) const
    {
        return static_cast<const TDerived*>(this)->extractImpl(input);
    }
};

/**
 * @brief Color extractor used to extract colors from images using HSV filtering.
 *
 */
class HsvColorExtractor : public ColorExtractor<HsvColorExtractor> {
private:
    std::vector<std::pair<cv::Scalar, cv::Scalar>> m_hsv_masks; ///< HSV threshold masks.

public:

    inline HsvColorExtractor(const std::span<const std::pair<cv::Scalar, cv::Scalar>>& hsv_masks) :
        m_hsv_masks(hsv_masks.begin(), hsv_masks.end()) {}

    /**
     * @brief Extracts color from the image into a binary mask using YUV filtering.
     *
     * @param input The input image to extract color from.
     * @return cv::Mat Binary mask of the extracted color.
     */
    [[nodiscard]] inline cv::Mat extractImpl(const cv::Mat& input) const
    {
        cv::Mat hsv;
        cv::cvtColor(input, hsv, cv::COLOR_BGR2HSV);
        return createBinaryMaskOr(hsv, std::span(m_hsv_masks));
    }
};

/**
 * @brief Color extractor used to extract colors from images using YUV filtering.
 *
 */
class YuvColorExtractor : public ColorExtractor<YuvColorExtractor> {
private:
    std::vector<std::pair<cv::Scalar, cv::Scalar>> m_yuv_masks; ///< YUV threshold masks.

public:

    inline YuvColorExtractor(const std::span<const std::pair<cv::Scalar, cv::Scalar>>& yuv_masks) :
        m_yuv_masks(yuv_masks.begin(), yuv_masks.end()) {}

    /**
     * @brief Extracts color from the image into a binary mask using YUV filtering.
     *
     * @param input The input image to extract color from.
     * @return cv::Mat Binary mask of the extracted color.
     */
    [[nodiscard]] inline cv::Mat extractImpl(const cv::Mat& input) const
    {
        cv::Mat yuv;
        cv::cvtColor(input, yuv, cv::COLOR_BGR2YUV);
        return createBinaryMaskOr(yuv, std::span(m_yuv_masks));
    }
};

}