#include "pisar/vision/bullseye_detector.h"

namespace pisar::mcp {

BullseyeDetector::BullseyeDetector(const HomographySizedProjection& projection)
    : m_projection(projection) {}

bool BullseyeDetector::isBullseyeInScene(const cv::Mat& frame) const {
    return findBullseye(frame).has_value();
}

std::optional<cv::Point> BullseyeDetector::findBullseye(const cv::Mat& frame) const {
    const cv::Mat mask = hsvFilter(frame);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) return std::nullopt;

    const auto& largest = *std::max_element(contours.begin(), contours.end(),
                                            [](const auto& a, const auto& b) {
                                                return cv::contourArea(a) < cv::contourArea(b);
                                            });

    if (cv::contourArea(largest) < 2000) return std::nullopt;

    std::vector<cv::Point> hull;
    cv::convexHull(largest, hull);

    cv::Point2f center;
    float radius;
    cv::minEnclosingCircle(hull, center, radius);

    if (radius < 20 || radius > 200) return std::nullopt;

    const Eigen::Vector2i pixel(static_cast<int>(center.x), static_cast<int>(center.y));
    const auto world = m_projection.project(pixel);
    std::cout << "Bullseye Center (World): " << world.transpose() << std::endl;

    return cv::Point(center);
}

cv::Mat BullseyeDetector::hsvFilter(const cv::Mat& frame) const {
    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    cv::Mat white_mask;
    cv::inRange(hsv, cv::Scalar(0, 0, 220), cv::Scalar(180, 40, 255));

    const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, {3, 3});
    cv::morphologyEx(white_mask, white_mask, cv::MORPH_OPEN, kernel);

    return white_mask;
}

}