#pragma once

#include <opencv2/opencv.hpp>
#include <optional>
#include <pisar/vision/homography.h>

namespace pisar::mcp {

class SafeZoneDetector {
public:
    explicit SafeZoneDetector(const HomographySizedProjection& projection);

    [[nodiscard]] bool isSafeZoneInScene(const cv::Mat& frame) const;
    [[nodiscard]] std::optional<cv::Point> findSafeZone(const cv::Mat& frame) const;

private:
    HomographySizedProjection m_projection;
};

}