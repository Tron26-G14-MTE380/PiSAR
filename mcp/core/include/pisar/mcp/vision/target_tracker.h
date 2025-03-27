#pragma once

#include "pisar/mcp/vision/color_extractor.h"
#include "pisar/mcp/vision/homography.h"
#include "pisar/mcp/vision/debug_visualization.h"

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

namespace pisar::mcp {

/**
 * @brief Tracks the target (bullseye) in frame, returning a smoothed position.
 *
 * @tparam TTimestamp The timestamp type used with the captured frame.
 * @tparam TColorExtractor The color extractor type used for extracting color-based masks from frames.
 */
template<class TTimestamp, class TColorExtractor, bool tkDebug>
class TargetTracker {
public:
    struct DebugData {
        cv::Mat color_extracted;
        cv::Mat target_point;
        cv::Mat projected_point;
    };

private:
    using DebugDataT = typename std::conditional_t<tkDebug, DebugData, std::monostate>;

    static constexpr float kMinContourArea = 2000.0f;
    static constexpr float kMinRadius = 20.0f;
    static constexpr float kMaxRadius = 200.0f;

    std::reference_wrapper<const TColorExtractor> m_color_extractor;
    std::reference_wrapper<const HomographySizedProjection> m_projection;

    DebugDataT m_debug;

public:

    /**
     * @brief Construct a new Target Tracker object
     *
     * @param color_extractor
     * @param projection
     */
    TargetTracker(const TColorExtractor& color_extractor, const HomographySizedProjection& projection) :
        m_color_extractor(color_extractor), m_projection(projection) {}

    /**
     * @brief Scans the input @p frame and returns the position of the target.
     *
     * @param frame The input frame to scan for target position.
     * @param timestamp The associated frame timestamp.
     * @return std::optional<Eigen::Vector2d> The target position if found otherwise std::nullopt.
     */
    [[nodiscard]] std::optional<Eigen::Vector2d> track(const cv::Mat& frame, TTimestamp timestamp)
    {
        if (tkDebug)
        {
            const auto empty_frame = cv::Mat::zeros(frame.size(), CV_8UC3);
            m_debug = {
                .color_extracted = empty_frame,
                .target_point = empty_frame,
                .projected_point = empty_frame
            };
        }

        const cv::Mat mask = m_color_extractor.get().extract(frame);

        if (tkDebug)
        {
            m_debug.color_extracted = mask.clone();
        }

        std::vector<std::vector<cv::Point>> contours;

        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if (contours.empty())
        {
            return std::nullopt;
        }

        const auto& largest = *std::max_element(contours.begin(), contours.end(), [](const auto& a, const auto& b)
        {
            return cv::contourArea(a) < cv::contourArea(b);
        });

        // Target too small, ignore
        if (cv::contourArea(largest) < kMinContourArea)
        {
            return std::nullopt;
        }

        std::vector<cv::Point> hull;
        cv::convexHull(largest, hull);

        cv::Point2f center;
        float radius;
        cv::minEnclosingCircle(hull, center, radius);

        if (radius < kMinRadius || radius > kMaxRadius)
        {
            return std::nullopt;
        }

        const Eigen::Vector2i pixel(static_cast<int>(center.x), static_cast<int>(center.y));
        const auto world = m_projection.get().project(pixel);

        if (tkDebug)
        {
            m_debug.target_point = frame.clone();
            cv::circle(m_debug.target_point, center, 5, cv::Scalar(0, 255, 0), 2);
            m_debug.projected_point = createHomographyProjectionVisualization(frame.size(), m_projection, std::span(&world, 1));
        }

        return world;
    }

    /**
     * @brief Resets the tracker state.
     *
     */
    void reset()
    {

    }

    /// @brief Retrieve debug data from last frame submitted.
    [[nodiscard]] inline const DebugData& debugData() const { return m_debug; }
};

}