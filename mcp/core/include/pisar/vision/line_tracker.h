#pragma once

#include "pisar/vision/thinning.h"
#include "pisar/vision/path_extraction.h"
#include "pisar/vision/path_simplification.h"
#include "pisar/vision/roi_mat.h"
#include "pisar/vision/utils.h"
#include "pisar/vision/debug_visualization.h"
#include "pisar/vision/trajectory_filter.h"
#include "pisar/vision/homography.h"

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <easy/profiler.h>

#include <span>
#include <vector>
#include <numeric>
#include <optional>
#include <type_traits>
#include <variant>
#include <ranges>
#include <iostream>
#include <chrono>

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
        RoiMat projected_trajectory;
        RoiMat filtered_trajectory;
        RoiMat simplified_filtered_trajectory;
    };

private:
    using DebugDataT = typename std::conditional_t<tkDebug, DebugData, std::monostate>;

    cv::Size m_frame_size;
    std::vector<std::pair<cv::Scalar, cv::Scalar>> m_hsv_masks;

    std::reference_wrapper<HomographyProjection> m_homography_projection;
    HomographySizedProjection m_homography_sized_projection;
    std::reference_wrapper<TrajectoryFilter<double>> m_trajectory_filter;

    DebugDataT m_debug;

public:

    /**
     * @brief Constructs a LineTracker with multiple HSV masks and validation parameters.
     * @param hsv_masks List of HSV threshold pairs (lower and upper bounds).
     * @param homography_projection Reference to homography projection to use.
     * @param trajectory_filter Reference to trajectory filter.
     */
    explicit LineTracker(
        const cv::Size& frame_size,
        const std::span<const std::pair<cv::Scalar, cv::Scalar>>& hsv_masks,
        HomographyProjection& homography_projection,
        TrajectoryFilter<double>& trajectory_filter
    )
        : m_frame_size(frame_size),
          m_hsv_masks(hsv_masks.begin(), hsv_masks.end()),
          m_homography_projection(homography_projection),
          m_homography_sized_projection(homography_projection.for_image({frame_size.width, frame_size.height})),
          m_trajectory_filter(trajectory_filter) {}

    /**
     * @brief Processes the input frame to extract a fitted trajectory.
     * @param frame The input frame (BGR format).
     * @return Ordered list of key points representing the fitted trajectory.
     */
    [[nodiscard]] std::vector<Eigen::Vector2d> extractTrajectory(const cv::Mat& frame, std::chrono::duration<float> timestamp)
    {
        EASY_FUNCTION();

        if (tkDebug)
        {
            const RoiMat empty_img = RoiMat(cv::Mat::zeros(m_frame_size, CV_8UC3));
            m_debug = {
                .preprocessed = empty_img,
                .hsvFiltered = empty_img,
                .skeleton = empty_img,
                .filtered_skeleton = empty_img,
                .trajectory = empty_img,
                .simplified_trajectory = empty_img,
                .projected_trajectory = empty_img,
                .filtered_trajectory = empty_img,
                .simplified_filtered_trajectory = empty_img
            };
        }

        if (frame.size() != m_frame_size)
        {
            std::cerr << "Input frame size (" << frame.size() << ") doesn't match expected frame size (" << m_frame_size << ")" << std::endl;
        }

        // Smooth out the frame with some preprocessing
        auto preprocessed = preprocess(RoiMat(frame));

        // Apply hsv filtering to detect colors
        auto binary_mask = applyHSVThreshold(preprocessed);

        // Skeletonize the line
        auto skeleton = skeletonizeCropped(binary_mask);

        // Get trajectory from skeleton
        auto trajectory = skeleton.has_value() ? fitSkeleton(skeleton.value()) : std::vector<Eigen::Vector2i>();

        // Project trajectory to real-world coordinates
        auto projected_trajectory = projectTrajectory(std::span(trajectory));

        // Filter the trajectory.
        return filterTrajectory(std::span(projected_trajectory), timestamp);
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

        for (const auto& [lower, upper] : m_hsv_masks) {
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

        // Order the points in trajectory
        const std::vector<Eigen::Vector2i> ordered_points = extractLongestPath(points);


        // Simplify the trajectory
        const std::vector<Eigen::Vector2i> simplified_trajectory = simplifyPath(std::span(ordered_points), 3);

        // Orient the trajectory
        // TODO

        if constexpr (tkDebug)
        {
            m_debug.trajectory = RoiMat(cv::Mat::zeros(skeleton.getOriginalSize(), CV_8UC1));
            createTrajectoryVisualization(m_debug.trajectory.getMat(), ordered_points, 255);

            m_debug.simplified_trajectory = RoiMat(cv::Mat::zeros(skeleton.getOriginalSize(), CV_8UC1));
            createTrajectoryVisualization(m_debug.simplified_trajectory.getMat(), simplified_trajectory, 255);
        }

        return simplified_trajectory;
    }

    [[nodiscard]] std::vector<Eigen::Vector2d> projectTrajectory(const std::span<const Eigen::Vector2i> trajectory)
    {
        const auto projected_trajectory = m_homography_sized_projection.project(trajectory);

        if constexpr (tkDebug)
        {
            m_debug.projected_trajectory = RoiMat(createHomographyProjectionVisualization(m_frame_size, m_homography_sized_projection, projected_trajectory));
        }

        return projected_trajectory;
    }

    [[nodiscard]] std::vector<Eigen::Vector2d> filterTrajectory(const std::span<const Eigen::Vector2d> trajectory, std::chrono::duration<float> timestamp)
    {
        const auto filtered_trajectory = m_trajectory_filter.get().filter(trajectory, timestamp);
        const auto simplified_filtered_trajectory = simplifyPath<double>(filtered_trajectory, 0.5f);

        if constexpr (tkDebug)
        {
            m_debug.filtered_trajectory = RoiMat(createHomographyProjectionVisualization(m_frame_size, m_homography_sized_projection, filtered_trajectory));
            m_debug.simplified_filtered_trajectory = RoiMat(createHomographyProjectionVisualization(m_frame_size, m_homography_sized_projection, simplified_filtered_trajectory));
        }

        return simplified_filtered_trajectory;
    }
};

}
