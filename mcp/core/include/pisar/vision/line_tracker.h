#pragma once

#include "pisar/vision/thinning.h"
#include "pisar/vision/path_extraction.h"
#include "pisar/vision/path_simplification.h"
#include "pisar/vision/roi_mat.h"
#include "pisar/vision/utils.h"

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <easy/profiler.h>

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
        RoiMat preprocessed;
        RoiMat hsvFiltered;
        RoiMat skeleton;
        RoiMat filtered_skeleton;
        RoiMat trajectory;
        RoiMat simplified_trajectory;
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

        auto preprocessed = preprocess(RoiMat(frame));
        auto binary_mask = applyHSVThreshold(preprocessed);
        const auto skeleton = skeletonizeCropped(binary_mask);
        if (!skeleton)
        {
            return {};
        }

        return fitSkeleton(skeleton.value());
    }

    [[nodiscard]] inline const DebugData& debugData() const { return m_debug; }

private:

    /**
     * @brief Applies Gaussian blur to reduce noise.
     * @param input Input image.
     * @return Blurred image.
     */
    [[nodiscard]] RoiMat preprocess(const RoiMat& input)
    {
        EASY_FUNCTION();

        cv::Mat blurred;
        cv::GaussianBlur(input.getMat(), blurred, cv::Size(5, 5), 0);

        RoiMat output(blurred, input);

        if constexpr (tkDebug)
        {
            m_debug.preprocessed = output;
        }

        return output;
    }

    /**
     * @brief Converts an image to HSV and applies multiple threshold masks.
     * @param input Input image (BGR format).
     * @return Binary mask combining all detected ranges.
     */
    [[nodiscard]] RoiMat applyHSVThreshold(const RoiMat& input)
    {
        EASY_FUNCTION();

        cv::Mat hsv, mask;
        cv::cvtColor(input.getMat(), hsv, cv::COLOR_BGR2HSV);
        mask = cv::Mat::zeros(input.getMat().size(), CV_8U);

        for (const auto& [lower, upper] : m_hsvMasks) {
            cv::Mat temp_mask;
            cv::inRange(hsv, lower, upper, temp_mask);
            cv::bitwise_or(mask, temp_mask, mask);
        }

        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(11, 11)));

        // Apply Erosion to remove small artifacts
        cv::erode(mask, mask, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(11, 11)));

        RoiMat output(mask, input);

        if constexpr (tkDebug)
        {
            m_debug.hsvFiltered = output;
        }

        return output;
    }

    /**
     * @brief Applies skeletonization with component filtering.
     * @param binary_input Binary image.
     * @return Skeletonized image with noise removed.
     */
    [[nodiscard]] RoiMat skeletonize(const RoiMat& binary_input)
    {
        EASY_FUNCTION();

        const int padding = 3;

        // Pad to simulate an "extended line" --> thinning doesn't truncate or flatten tips
        cv::Mat padded_binary;
        cv::copyMakeBorder(binary_input.getMat(), padded_binary, padding, padding, padding, padding, cv::BORDER_REPLICATE, cv::Scalar(0));

        // Apply Zhang-Suen thinning
        cv::Mat skeleton;
        thinningMaRenParallel(padded_binary, skeleton);

        // Remove padding
        skeleton = skeleton(cv::Rect(padding, padding, skeleton.cols - 2 * padding, skeleton.rows - 2 * padding));

        // Remove small connected components
        cv::Mat filtered_skeleton = filterComponents(skeleton);

        RoiMat output(filtered_skeleton, binary_input);

        if constexpr (tkDebug)
        {
            m_debug.skeleton = RoiMat(skeleton, binary_input);
            m_debug.filtered_skeleton = output;
        }

        return output;
    }

    [[nodiscard]] std::optional<RoiMat> skeletonizeCropped(const RoiMat& binary_mask)
    {
        EASY_FUNCTION();

        const cv::Rect bbox = computeBoundingBox(binary_mask.getMat());
        if (bbox.width == 0 || bbox.height == 0) return std::nullopt;

        // Crop the region of interest (ROI)
        RoiMat cropped_binary_mask = binary_mask.crop(bbox);
        RoiMat cropped_skeleton = skeletonize(cropped_binary_mask);

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

        cv::Mat closed_skeleton;
        cv::morphologyEx(skeleton, closed_skeleton, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));

        cv::Mat labels, stats, centroids;
        int num_labels = cv::connectedComponentsWithStats(closed_skeleton, labels, stats, centroids, 8);

        if (num_labels <= 1) {
            return closed_skeleton.clone();
        }

        // Find the largest connected component (excluding background)
        int largest_idx = 1;
        int max_area = stats.at<int>(1, cv::CC_STAT_AREA);
        for (int i = 2; i < num_labels; ++i) {
            int area = stats.at<int>(i, cv::CC_STAT_AREA);
            if (area > max_area) {
                max_area = area;
                largest_idx = i;
            }
        }

        // Create a blank mask and explicitly extract the largest component
        cv::Mat largest_skeleton = cv::Mat::zeros(closed_skeleton.size(), CV_8U);

        for (int r = 0; r < labels.rows; ++r) {
            for (int c = 0; c < labels.cols; ++c) {
                if (labels.at<int>(r, c) == largest_idx) {
                    largest_skeleton.at<uint8_t>(r, c) = 255;  // Set pixel to white
                }
            }
        }

        return largest_skeleton;
    }

    /**
     * @brief Fits a trajectory to the skeleton points.
     * @param skeleton Skeletonized image.
     * @return Ordered list of key points representing the fitted trajectory.
     */
    [[nodiscard]] std::vector<Eigen::Vector2i> fitSkeleton(const RoiMat& skeleton)
    {
        EASY_FUNCTION();

        std::vector<Eigen::Vector2i> points;
        for (int y = 0; y < skeleton.getMat().rows; ++y) {
            for (int x = 0; x < skeleton.getMat().cols; ++x) {
                if (skeleton.getMat().at<uchar>(y, x) > 0) {
                    const cv::Point coord = skeleton.toOriginalCoords({x, y});
                    points.emplace_back(coord.x, coord.y);
                }
            }
        }

        // const std::vector<Eigen::Vector2i> ordered_points = orderPoints(points);
        const std::vector<Eigen::Vector2i> ordered_points = extractLongestPath(points);

        if constexpr (tkDebug)
        {
            // Create an empty image (same size as input_img, single-channel black)
            m_debug.trajectory = RoiMat(cv::Mat::zeros(skeleton.getOriginalSize(), skeleton.getMat().type()));

            // Draw points
            for (const auto& pt : ordered_points)
            {
                cv::circle(m_debug.trajectory.getMat(), cv::Point(pt.x(), pt.y()), 1, 255, cv::FILLED);
            }

            // Connect points with lines
            for (size_t i = 1; i < ordered_points.size(); ++i)
            {
                const auto prev_pt = cv::Point(ordered_points[i - 1].x(), ordered_points[i - 1].y());
                const auto current_pt = cv::Point(ordered_points[i].x(), ordered_points[i].y());
                cv::line(m_debug.trajectory.getMat(), prev_pt, current_pt, 255, 1);
            }
        }

        const std::vector<Eigen::Vector2i> simplified_trajectory = simplifyPath(std::span(ordered_points), 3);

        if constexpr (tkDebug)
        {
            // Create an empty image (same size as input_img, single-channel black)
            m_debug.simplified_trajectory = RoiMat(cv::Mat::zeros(skeleton.getOriginalSize(), skeleton.getMat().type()));

            // Draw points
            for (const auto& pt : simplified_trajectory)
            {
                cv::circle(m_debug.simplified_trajectory.getMat(), cv::Point(pt.x(), pt.y()), 3, 255, cv::FILLED);
            }

            // Connect points with lines
            for (size_t i = 1; i < simplified_trajectory.size(); ++i)
            {
                const auto prev_pt = cv::Point(simplified_trajectory[i - 1].x(), simplified_trajectory[i - 1].y());
                const auto current_pt = cv::Point(simplified_trajectory[i].x(), simplified_trajectory[i].y());
                cv::line(m_debug.simplified_trajectory.getMat(), prev_pt, current_pt, 255, 1);
            }
        }

        return simplified_trajectory;
    }
};

}
