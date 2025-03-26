#pragma once

#include <opencv2/opencv.hpp>
#include <optional>
#include <pisar/vision/homography.h>

namespace pisar::mcp {

class BullseyeDetector {
public:
    explicit BullseyeDetector(const HomographySizedProjection& projection);

    [[nodiscard]] bool isBullseyeInScene(const cv::Mat& frame) const;
    [[nodiscard]] std::optional<cv::Point> findBullseye(const cv::Mat& frame) const;

private:
    HomographySizedProjection m_projection;

    [[nodiscard]] cv::Mat hsvFilter(const cv::Mat& frame) const;
};

}