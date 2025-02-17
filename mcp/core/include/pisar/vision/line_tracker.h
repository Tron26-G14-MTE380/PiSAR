#pragma once

#include "pisar/vision/thinning.h"

#include <opencv2/opencv.hpp>
#include <eigen/Dense>

#include <span>
#include <vector>
#include <numeric>
#include <optional>
#include <type_traits>
#include <variant>

namespace pisar::mcp {

/**
 * @brief Extracts a fitted trajectory from a skeletonized path.
 */
template<bool tkDebug>
class LineTracker {
public:
    struct DebugData {
        cv::Mat preprocessed;
        cv::Mat hsvFiltered;
        cv::Mat skeleton;
        cv::Mat filtered_skeleton;
        cv::Mat trajectory;
    };

private:
    using DebugDataT = typename std::conditional_t<tkDebug, DebugData, std::monostate>;

    std::vector<std::pair<cv::Scalar, cv::Scalar>> m_hsvMasks;
    DebugDataT m_debug;

public:

    /**
     * @brief Constructs a LineTracker with multiple HSV masks.
     * @param hsv_masks List of HSV threshold pairs (lower and upper bounds).
     */
    explicit LineTracker(const std::span<const std::pair<cv::Scalar, cv::Scalar>>& hsv_masks)
        : m_hsvMasks(hsv_masks.begin(), hsv_masks.end()) {}

    /**
     * @brief Processes the input frame to extract a fitted trajectory.
     * @param frame The input frame (BGR format).
     * @return Ordered list of key points representing the fitted trajectory.
     */
    [[nodiscard]] std::vector<Eigen::Vector2i> extractTrajectory(const cv::Mat& frame)
    {
        cv::Mat preprocessed = preprocess(frame);
        cv::Mat binary_mask = applyHSVThreshold(preprocessed);
        cv::Mat skeleton = skeletonize(binary_mask);
        return fitSkeleton(skeleton);
    }

    [[nodiscard]] inline const DebugData& debugData() const { return m_debug; }

private:

    /**
     * @brief Applies Gaussian blur to reduce noise.
     * @param input Input image.
     * @return Blurred image.
     */
    [[nodiscard]] cv::Mat preprocess(const cv::Mat& input)
    {
        cv::Mat blurred;
        cv::GaussianBlur(input, blurred, cv::Size(5, 5), 0);
        if constexpr (tkDebug)
        {
            m_debug.preprocessed = blurred;
        }
        return blurred;
    }

    /**
     * @brief Converts an image to HSV and applies multiple threshold masks.
     * @param input Input image (BGR format).
     * @return Binary mask combining all detected ranges.
     */
    [[nodiscard]] cv::Mat applyHSVThreshold(const cv::Mat& input)
    {
        cv::Mat hsv, mask;
        cv::cvtColor(input, hsv, cv::COLOR_BGR2HSV);
        mask = cv::Mat::zeros(input.size(), CV_8U);
    
        for (const auto& [lower, upper] : m_hsvMasks) {
            cv::Mat temp_mask;
            cv::inRange(hsv, lower, upper, temp_mask);
            cv::bitwise_or(mask, temp_mask, mask);
        }

        if constexpr (tkDebug)
        {
            m_debug.hsvFiltered = mask;
        }
    
        return mask;
    }

    /**
     * @brief Applies skeletonization with component filtering.
     * @param input Binary image.
     * @return Skeletonized image with noise removed.
     */
    [[nodiscard]] cv::Mat skeletonize(const cv::Mat& input)
    {
        cv::Mat binary;
        cv::threshold(input, binary, 127, 255, cv::THRESH_BINARY);
    
        // Apply Zhang-Suen thinning
        cv::Mat skeleton;
        thinningZhangSuen(binary, skeleton);
    
        // Remove small connected components
        cv::Mat filtered_skeleton = filterComponents(skeleton);

        if constexpr (tkDebug)
        {
            m_debug.skeleton = skeleton;
            m_debug.filtered_skeleton = skeleton;
        }
    
        return filtered_skeleton;
    }

    /**
     * @brief Filters small connected components from a skeletonized image.
     * @param skeleton Skeletonized image.
     * @return Filtered skeleton with only the largest component retained.
     */
    [[nodiscard]] cv::Mat filterComponents(const cv::Mat& skeleton) const
    {
        cv::Mat labels, stats, centroids;
        int num_labels = cv::connectedComponentsWithStats(skeleton, labels, stats, centroids, 8);
    
        if (num_labels <= 1) {
            return skeleton.clone();
        }
    
        // Find the largest component (ignoring the background at index 0)
        int largest_idx = 1;
        int max_area = stats.at<int>(1, cv::CC_STAT_AREA);
        for (int i = 2; i < num_labels; ++i) {
            int area = stats.at<int>(i, cv::CC_STAT_AREA);
            if (area > max_area) {
                max_area = area;
                largest_idx = i;
            }
        }
    
        cv::Mat largest_skeleton = cv::Mat::zeros(skeleton.size(), CV_8U);
        largest_skeleton.setTo(255, labels == largest_idx);
    
        return largest_skeleton;
    }

    /**
     * @brief Fits a trajectory to the skeleton points.
     * @param skeleton Skeletonized image.
     * @return Ordered list of key points representing the fitted trajectory.
     */
    [[nodiscard]] std::vector<Eigen::Vector2i> fitSkeleton(const cv::Mat& skeleton)
    {
        std::vector<Eigen::Vector2i> points;
        for (int y = 0; y < skeleton.rows; ++y) {
            for (int x = 0; x < skeleton.cols; ++x) {
                if (skeleton.at<uchar>(y, x) > 0) {
                    points.emplace_back(x, y);
                }
            }
        }

        const std::vector<Eigen::Vector2i> ordered_points = orderPoints(points);

        if constexpr (tkDebug)
        {
            // Create an empty image (same size as input_img, single-channel black)
            m_debug.trajectory = cv::Mat::zeros(skeleton.size(), skeleton.type());

            // Draw points
            for (const auto& pt : ordered_points) 
            {
                cv::circle(m_debug.trajectory, cv::Point(pt.x(), pt.y()), 1, 255, cv::FILLED);
            }

            // Connect points with lines
            for (size_t i = 1; i < ordered_points.size(); ++i) 
            {
                const auto prev_pt = cv::Point(ordered_points[i - 1].x(), ordered_points[i - 1].y());
                const auto current_pt = cv::Point(ordered_points[i].x(), ordered_points[i].y());
                cv::line(m_debug.trajectory, prev_pt, current_pt, 255, 1);
            }
        }
    
        return ordered_points;
    }    

    /**
     * @brief Orders skeleton points into a continuous trajectory using nearest-neighbor traversal.
     * @param points List of skeleton keypoints.
     * @return Ordered list of trajectory points.
     */
    [[nodiscard]] std::vector<Eigen::Vector2i> orderPoints(const std::vector<Eigen::Vector2i>& points)
    {
        if (points.empty()) return {};
    
        std::vector<Eigen::Vector2i> ordered_path;
        std::vector<bool> visited(points.size(), false);
    
        // Find the starting point (left-most point)
        auto min_it = std::min_element(points.begin(), points.end(), [](const Eigen::Vector2i& a, const Eigen::Vector2i& b) {
            return a.x() < b.x();
        });
    
        int start_idx = std::distance(points.begin(), min_it);
        ordered_path.push_back(points[start_idx]);
        visited[start_idx] = true;
    
        // Nearest-neighbor traversal to order points
        while (ordered_path.size() < points.size()) {
            int last_idx = static_cast<int>(ordered_path.size()) - 1;
            double min_dist = std::numeric_limits<double>::max();
            int next_idx = -1;
    
            for (size_t i = 0; i < points.size(); ++i) {
                if (visited[i]) continue;
    
                double dist = (ordered_path[last_idx] - points[i]).norm();
                if (dist < min_dist) {
                    min_dist = dist;
                    next_idx = static_cast<int>(i);
                }
            }
    
            if (next_idx != -1) {
                ordered_path.push_back(points[next_idx]);
                visited[next_idx] = true;
            } else {
                break;
            }
        }
    
        return ordered_path;
    }
};

}