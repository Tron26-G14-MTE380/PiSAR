#include "pisar/vision/safe_zone_detector.h"

namespace pisar::mcp {

SafeZoneDetector::SafeZoneDetector(const HomographySizedProjection& projection)
    : m_projection(projection) {}

bool SafeZoneDetector::isSafeZoneInScene(const cv::Mat& frame) const {
    return findSafeZone(frame).has_value();
}

std::optional<cv::Point> SafeZoneDetector::findSafeZone(const cv::Mat& frame) const {
    if (frame.empty()) return std::nullopt;

    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    cv::Mat mask;
    cv::inRange(hsv, cv::Scalar(40, 50, 50), cv::Scalar(80, 255, 255));

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) return std::nullopt;

    std::vector<cv::Point> all_points;
    for (const auto& c : contours)
        all_points.insert(all_points.end(), c.begin(), c.end());

    std::vector<cv::Point> hull;
    cv::convexHull(all_points, hull);

    const double epsilon = 0.02 * cv::arcLength(hull, true);
    std::vector<cv::Point> approx;
    cv::approxPolyDP(hull, approx, epsilon, true);

    if (approx.size() < 4 || approx.size() > 8) return std::nullopt;

    const cv::Rect bbox = cv::boundingRect(hull);
    const double hull_area = cv::contourArea(hull);
    const double bbox_area = bbox.area();

    if (hull_area < 300 || bbox_area < 1000) return std::nullopt;

    const cv::Point center(bbox.x + bbox.width / 2, bbox.y + bbox.height / 2);
    const Eigen::Vector2i pixel(center.x, center.y);
    const auto world = m_projection.project(pixel);
    std::cout << "Safe Zone Center (World): " << world.transpose() << std::endl;

    return center;
}

}