#pragma once

#include "pisar/vision/thinning.h"
#include "pisar/vision/path_extraction.h"
#include "pisar/vision/path_simplification.h"

#include <opencv2/opencv.hpp>
#include <eigen/Dense>
#include <easy/profiler.h>

#include <span>
#include <vector>
#include <numeric>
#include <optional>
#include <type_traits>
#include <variant>

namespace pisar::mcp {


/// @brief Computes the bounding box of nonzero pixels in a binary image.
/// @param img Input binary image (8-bit, single-channel).
/// @return Bounding box as cv::Rect.
cv::Rect computeBoundingBox(const cv::Mat& img) {
    CV_Assert(img.type() == CV_8UC1);

    int min_x = 0, max_x = img.cols - 1, min_y = 0, max_y = img.rows - 1;

    // Find first and last rows that contain nonzero pixels
    while (min_y <= max_y && cv::countNonZero(img.row(min_y)) == 0) ++min_y;
    while (max_y >= min_y && cv::countNonZero(img.row(max_y)) == 0) --max_y;

    if (min_y > max_y) return cv::Rect(); // No nonzero pixels found

    //return cv::Rect(min_x, min_y, max_x - min_x + 1, max_y - min_y + 1); 

    // Find first and last columns that contain nonzero pixels
    cv::Mat roi = img.rowRange(min_y, max_y + 1);
    while (min_x < img.cols && cv::countNonZero(roi.col(min_x)) == 0) ++min_x;
    while (max_x < img.cols && cv::countNonZero(roi.col(max_x)) == 0) --max_x;

    return cv::Rect(min_x, min_y, max_x - min_x + 1, max_y - min_y + 1);
}

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
        cv::Mat simplified_trajectory;
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
        EASY_FUNCTION();

        cv::Mat preprocessed = preprocess(frame);
        cv::Mat binary_mask = applyHSVThreshold(preprocessed);
        cv::Mat skeleton = skeletonizeCropped(binary_mask);
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
        EASY_FUNCTION();

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
        EASY_FUNCTION();

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
     * @param binary_input Binary image.
     * @return Skeletonized image with noise removed.
     */
    [[nodiscard]] cv::Mat skeletonize(const cv::Mat& binary_input)
    {
        EASY_FUNCTION();

        const int padding = 3;

        // Pad to simulate an "extended line" --> thinning doesn't truncate or flatten tips
        cv::Mat padded_binary;
        cv::copyMakeBorder(binary_input, padded_binary, padding, padding, padding, padding, cv::BORDER_REPLICATE, cv::Scalar(0));
    
        // Apply Zhang-Suen thinning
        cv::Mat skeleton;
        thinningMaRenParallel(padded_binary, skeleton);

        // Remove padding
        skeleton = skeleton(cv::Rect(padding, padding, skeleton.cols - 2 * padding, skeleton.rows - 2 * padding));
    
        // Remove small connected components
        cv::Mat filtered_skeleton = filterComponents(skeleton);

        if constexpr (tkDebug)
        {
            m_debug.skeleton = skeleton;
            m_debug.filtered_skeleton = skeleton;
        }
    
        return filtered_skeleton;
    }

    [[nodiscard]] cv::Mat skeletonizeCropped(const cv::Mat& binary_mask)
    {
        EASY_FUNCTION();

        const cv::Rect bbox = computeBoundingBox(binary_mask);
        if (bbox.width == 0 || bbox.height == 0) return {};

        // Crop the region of interest (ROI)
        cv::Mat cropped_binary_mask = binary_mask(bbox).clone(); // Clone ensures we work on a separate copy

        cv::Mat cropped_skeleton = skeletonize(cropped_binary_mask);

        return cropped_skeleton;
    }

    /**
     * @brief Filters small connected components from a skeletonized image.
     * @param skeleton Skeletonized image.
     * @return Filtered skeleton with only the largest component retained.
     */
    [[nodiscard]] cv::Mat filterComponents(const cv::Mat& skeleton) const
    {
        EASY_FUNCTION();

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
        EASY_FUNCTION();

        std::vector<Eigen::Vector2i> points;
        for (int y = 0; y < skeleton.rows; ++y) {
            for (int x = 0; x < skeleton.cols; ++x) {
                if (skeleton.at<uchar>(y, x) > 0) {
                    points.emplace_back(x, y);
                }
            }
        }

        // const std::vector<Eigen::Vector2i> ordered_points = orderPoints(points);
        const std::vector<Eigen::Vector2i> ordered_points = extractLongestPath(points);

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
        
        const std::vector<Eigen::Vector2i> simplified_trajectory = simplifyPath(std::span(ordered_points), 3);

        if constexpr (tkDebug)
        {
            // Create an empty image (same size as input_img, single-channel black)
            m_debug.simplified_trajectory = cv::Mat::zeros(skeleton.size(), skeleton.type());

            // Draw points
            for (const auto& pt : simplified_trajectory) 
            {
                cv::circle(m_debug.simplified_trajectory, cv::Point(pt.x(), pt.y()), 3, 255, cv::FILLED);
            }

            // Connect points with lines
            for (size_t i = 1; i < simplified_trajectory.size(); ++i) 
            {
                const auto prev_pt = cv::Point(simplified_trajectory[i - 1].x(), simplified_trajectory[i - 1].y());
                const auto current_pt = cv::Point(simplified_trajectory[i].x(), simplified_trajectory[i].y());
                cv::line(m_debug.simplified_trajectory, prev_pt, current_pt, 255, 1);
            }
        }
    
        return simplified_trajectory;
    }
};

}