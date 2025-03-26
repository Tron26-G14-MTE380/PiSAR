#pragma once

#include "pisar/mcp/vision/roi_mat.h"
#include "pisar/mcp/vision/roi_tracker.h"
#include "pisar/mcp/vision/color_extractor.h"
#include "pisar/mcp/vision/thinning.h"
#include "pisar/mcp/vision/path_extraction.h"
#include "pisar/mcp/vision/path_simplification.h"
#include "pisar/mcp/vision/homography.h"
#include "pisar/mcp/vision/trajectory_filter.h"
#include "pisar/mcp/vision/utils.h"
#include "pisar/mcp/vision/debug_visualization.h"

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
template<class TTimestamp, class TColorExtractor, bool tkDebug>
class LineTracker {
public:
    struct DebugData {
        RoiMat preprocessed;
        RoiMat extractedColor;
        RoiMat skeleton;
        RoiMat filtered_skeleton;
        RoiMat trajectory;
        RoiMat simplified_trajectory;
        RoiMat ordered_trajectory;
        RoiMat projected_trajectory;
        RoiMat filtered_trajectory;
        RoiMat simplified_filtered_trajectory;
    };

private:
    using DebugDataT = typename std::conditional_t<tkDebug, DebugData, std::monostate>;

    cv::Size m_frame_size;
    std::vector<std::pair<cv::Scalar, cv::Scalar>> m_yuv_masks;

    RoiTracker m_roi_tracker;
    std::reference_wrapper<const TColorExtractor> m_color_extractor;
    HomographySizedProjection m_homography_sized_projection;
    std::reference_wrapper<TrajectoryFilter<double, TTimestamp>> m_trajectory_filter;

    DebugDataT m_debug;

public:

    /**
     * @brief Constructs a LineTracker.
     * @param frame_size The input frame size.
     * @param yuv_masks List of YUV threshold pairs (lower and upper bounds).
     * @param homography_projection Reference to homography projection to use.
     * @param trajectory_filter Reference to trajectory filter.
     */
    explicit LineTracker(
        const cv::Size& frame_size,
        const TColorExtractor& color_extractor,
        const HomographySizedProjection& homography_projection,
        TrajectoryFilter<double, TTimestamp>& trajectory_filter
    )
        : m_frame_size(frame_size),
          m_color_extractor(color_extractor),
          m_homography_sized_projection(homography_projection),
          m_trajectory_filter(trajectory_filter)
          {}

    /**
     * @brief Processes the input frame to extract a fitted trajectory.
     * @param frame The input frame (BGR format).
     * @return Ordered list of key points representing the fitted trajectory.
     */
    [[nodiscard]] std::vector<Eigen::Vector2d> extractTrajectory(const cv::Mat& frame, TTimestamp timestamp)
    {
        EASY_FUNCTION();

        if (tkDebug)
        {
            const RoiMat empty_img = RoiMat(cv::Mat::zeros(m_frame_size, CV_8UC3));
            m_debug = {
                .preprocessed = empty_img,
                .extractedColor = empty_img,
                .skeleton = empty_img,
                .filtered_skeleton = empty_img,
                .trajectory = empty_img,
                .simplified_trajectory = empty_img,
                .ordered_trajectory = empty_img,
                .projected_trajectory = empty_img,
                .filtered_trajectory = empty_img,
                .simplified_filtered_trajectory = empty_img
            };
        }

        if (frame.size() != m_frame_size)
        {
            std::cerr << "Input frame size (" << frame.size() << ") doesn't match expected frame size (" << m_frame_size << ")" << std::endl;
        }

        auto input_frame = RoiMat(frame);

        auto precropped_frame = preCrop(input_frame);

        // Smooth out the frame with some preprocessing
        auto preprocessed = preprocess(precropped_frame);

        // Apply filtering to detect colors
        auto binary_mask = extractColor(preprocessed);

        // Crop the ROI
        auto cropped_binary_mask = cropRoi(binary_mask);

        std::vector<Eigen::Vector2i> trajectory;
        if (cropped_binary_mask)
        {
            m_roi_tracker.submit(cropped_binary_mask.value().getCrop());

            // Skeletonize the line
            auto skeleton = skeletonize(cropped_binary_mask.value());

            // Get trajectory from skeleton
            trajectory = fitSkeleton(skeleton);
        }
        else
        {
            m_roi_tracker.reset();
        }

        // Project trajectory to real-world coordinates
        auto projected_trajectory = projectTrajectory(std::span(trajectory));

        // Filter the trajectory.
        return filterTrajectory(std::span(projected_trajectory), timestamp);
    }

    /// @brief Resets the line tracker state.
    [[nodiscard]] inline void reset()
    {
        m_roi_tracker.reset();
        m_trajectory_filter.get().reset();
    };

    /// @brief Retrieve debug data from last frame submitted.
    [[nodiscard]] inline const DebugData& debugData() const { return m_debug; }

private:

    [[nodiscard]] RoiMat preCrop(const RoiMat& input)
    {
        const auto roi_crop = m_roi_tracker.getEstimatedRoi(input.getOriginalSize());

        if (!roi_crop)
        {
            return input;
        }

        return input.crop(roi_crop.value());
    }

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
     * @brief Creates a binary mask with extracted color from input image.
     * @param input Input image (BGR format).
     * @return Binary mask with extracted color.
     */
    [[nodiscard]] RoiMat extractColor(const RoiMat& input)
    {
        EASY_FUNCTION();

        cv::Mat mask = m_color_extractor.get().extract(input.getMat());

        // Morphological cleanup
        const auto kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(11, 11));
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
        cv::erode(mask, mask, kernel);

        RoiMat output(mask, input);

        if constexpr (tkDebug)
        {
            m_debug.extractedColor = output;
        }

        return output;
    }

    [[nodiscard]] std::optional<RoiMat> cropRoi(const RoiMat& binary_mask)
    {
        EASY_FUNCTION();

        const cv::Rect bbox = computeBoundingBox(binary_mask.getMat());
        if (bbox.width == 0 || bbox.height == 0) return std::nullopt;

        return binary_mask.crop(bbox);
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
        const std::vector<Eigen::Vector2i> trajectory = extractLongestPath(points);


        // Simplify the trajectory
        const std::vector<Eigen::Vector2i> simplified_trajectory = simplifyPath(std::span(trajectory), 3);

        // Orient the trajectory

        std::vector<Eigen::Vector2i> ordered_trajectory;
        if (!simplified_trajectory.empty())
        {
            const Eigen::Vector2i ref_point(skeleton.getOriginalSize().width/2, skeleton.getOriginalSize().height);
            const double first_point_distance = (simplified_trajectory.front() - ref_point).norm();
            const double last_point_distance = (simplified_trajectory.back() - ref_point).norm();

            if (first_point_distance > last_point_distance)
            {
                ordered_trajectory = std::vector<Eigen::Vector2i>(simplified_trajectory.rbegin(), simplified_trajectory.rend());
            }
            else
            {
                ordered_trajectory = simplified_trajectory;
            }
        }

        if constexpr (tkDebug)
        {
            m_debug.trajectory = RoiMat(
                createTrajectoryVisualization(
                    skeleton.getOriginalSize(), trajectory,
                    cv::Scalar(255, 255, 255), std::nullopt, std::nullopt
                )
            );

            m_debug.simplified_trajectory = RoiMat(
                createTrajectoryVisualization(
                    skeleton.getOriginalSize(), simplified_trajectory,
                    cv::Scalar(255, 255, 255), std::nullopt, std::nullopt
                )
            );

            m_debug.ordered_trajectory = RoiMat(
                createTrajectoryVisualization(
                    skeleton.getOriginalSize(), ordered_trajectory,
                    cv::Scalar(255, 255, 255), std::nullopt, std::nullopt
                )
            );
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

    [[nodiscard]] std::vector<Eigen::Vector2d> filterTrajectory(const std::span<const Eigen::Vector2d> trajectory, TTimestamp timestamp)
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
