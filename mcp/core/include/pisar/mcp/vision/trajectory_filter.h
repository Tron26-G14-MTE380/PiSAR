#pragma once

#include "pisar/mcp/vision/extended_kalman_filter.h"

#include <Eigen/Dense>

#include <vector>
#include <deque>
#include <ranges>
#include <chrono>

namespace pisar::mcp {

/**
 * @brief Smoothes and filters a time-series trajectory.
 *
 * @tparam T The type of output trajectory data value.
 * @tparam TTimestamp The timestamp type, must be of type std::chrono::time_point.
 */
template<typename T, class TTimestamp>
class TrajectoryFilter
{
public:
    using TimestampT = TTimestamp;
    using ClockT = TTimestamp::clock;
    using DurationT = TimestampT::duration;

private:
    DurationT m_reset_timeout;
    float m_max_jump_distance;
    int m_min_persistence_frames;
    int m_fixed_sample_size;

    std::vector<ExtendedKalmanFilter<2>> m_kalman_filters; ///< One Kalman filter per trajectory point
    std::deque<std::pair<TimestampT, std::vector<Eigen::Vector2d>>> m_past_trajectories; ///< Stores past trajectories
    std::vector<Eigen::Vector2d> m_smoothed_trajectory; ///< Stores the last valid trajectory
    std::vector<Eigen::Vector2<T>> m_output_trajectory; ///< Smoothed trajectory casted to output type
    std::optional<TimestampT> m_last_valid_trajectory_time;

public:

    /**
     * @brief Constructs a TrajectoryFilter.
     * @param reset_timeout The timeout for consecutive empty trajectories that causes filter to reset its state.
     * @param max_jump_distance Maximum allowed jump distance between consecutive frames.
     * @param min_persistence_frames Minimum number of frames a trajectory must be consistent.
     * @param fixed_sample_size Number of points used for arc-length parameterization.
     */
    explicit TrajectoryFilter(
        DurationT reset_timeout = std::chrono::milliseconds(500),
        float max_jump_distance = 30.0f,
        int min_persistence_frames = 3,
        int fixed_sample_size = 20)
        : m_reset_timeout(reset_timeout),
          m_max_jump_distance(max_jump_distance),
          m_min_persistence_frames(min_persistence_frames),
          m_fixed_sample_size(fixed_sample_size),
          m_last_valid_trajectory_time(std::nullopt)
          {
            m_kalman_filters.resize(m_fixed_sample_size);
          }

    /**
     * @brief Filters and smooths the extracted trajectory using hybrid validation.
     * @param trajectory Extracted raw trajectory.
     * @return Smoothed and filtered trajectory.
     */
    [[nodiscard]] std::vector<Eigen::Vector2<T>> filter(const std::span<const Eigen::Vector2<T>>& trajectory, TimestampT timestamp)
    {
        if (trajectory.size() < 2)
        {
            if (m_last_valid_trajectory_time.has_value() && (timestamp - m_last_valid_trajectory_time.value()) > m_reset_timeout)
            {
                // If the trajectory has been missing for too long, reset the filter.
                reset();
            }
        }
        else
        {
            // Valid trajectory detected → Update tracking
            m_last_valid_trajectory_time = timestamp;
        }

        auto trajectory_casted_view = trajectory | std::views::transform([](const Eigen::Vector2<T>& p) -> Eigen::Vector2d { return p.template cast<double>(); });
        std::vector<Eigen::Vector2d> trajectory_casted(trajectory_casted_view.begin(), trajectory_casted_view.end());

        if (m_past_trajectories.empty())
        {
            m_past_trajectories.push_back(std::make_pair(timestamp, trajectory_casted));
            return std::vector<Eigen::Vector2<T>>(trajectory.begin(), trajectory.end());
        }

        // **Pre-Filtering Checks**
        // if (checkLargeJump(trajectory_casted) || !checkStability(trajectory_casted))
        // {
        //     return m_output_trajectory.empty() ? trajectory : m_output_trajectory;
        // }

        // **Apply Arc-Length Parameterization**
        std::vector<Eigen::Vector2d> resampled = arcLengthResample(trajectory_casted);

        const DurationT delta_t = timestamp - m_past_trajectories.back().first;

        // Apply smoothing
        std::vector<Eigen::Vector2d> smoothed = applyFiltering(resampled, delta_t);

        // Update stored trajectories
        m_smoothed_trajectory = smoothed;
        m_past_trajectories.push_back(std::make_pair(timestamp, m_smoothed_trajectory));
        if (m_past_trajectories.size() > m_min_persistence_frames)
        {
            m_past_trajectories.pop_front();
        }

        m_output_trajectory.resize(m_smoothed_trajectory.size());
        std::transform(m_smoothed_trajectory.begin(), m_smoothed_trajectory.end(), m_output_trajectory.begin(), [](const Eigen::Vector2d& p)
        {
            return p.cast<T>();
        });

        return m_output_trajectory;
    }

    void reset()
    {
        // Clear stored trajectories
        m_past_trajectories.clear();
        m_smoothed_trajectory.clear();
        m_output_trajectory.clear();

        // Reset Kalman filters
        m_kalman_filters.clear();
        m_kalman_filters.resize(m_fixed_sample_size); // Reinitialize Kalman filters

        // Clear last valid trajectory timestamp
        m_last_valid_trajectory_time.reset();
    }

private:
    /**
     * @brief Converts a trajectory into a fixed number of points using arc-length parameterization.
     * @param trajectory The raw trajectory points.
     * @return Resampled trajectory with a fixed number of points.
     */
    [[nodiscard]] std::vector<Eigen::Vector2d> arcLengthResample(const std::vector<Eigen::Vector2d>& trajectory) const
    {
        if (trajectory.size() < 2)
        {
            return trajectory; // Not enough points to resample
        }

        // Compute arc-lengths
        std::vector<float> arc_lengths(trajectory.size(), 0.0f);
        for (size_t i = 1; i < trajectory.size(); ++i)
        {
            arc_lengths[i] = arc_lengths[i - 1] + (trajectory[i] - trajectory[i - 1]).norm();
        }
        const float total_length = arc_lengths.back();

        // Generate resampled points
        std::vector<Eigen::Vector2d> resampled;
        for (int i = 0; i < m_fixed_sample_size; ++i)
        {
            float target_length = total_length * (static_cast<float>(i) / (m_fixed_sample_size - 1));

            // Find the two closest points on the original trajectory
            for (size_t j = 1; j < trajectory.size(); ++j)
            {
                if (arc_lengths[j] >= target_length)
                {
                    float ratio = (target_length - arc_lengths[j - 1]) / (arc_lengths[j] - arc_lengths[j - 1]);
                    Eigen::Vector2d interpolated = trajectory[j - 1] + ratio * (trajectory[j] - trajectory[j - 1]);
                    resampled.push_back(interpolated);
                    break;
                }
            }
        }

        return resampled;
    }

    /**
     * @brief Rejects trajectories that jump too far from the last known good trajectory.
     */
    [[nodiscard]] bool checkLargeJump(const std::vector<Eigen::Vector2d>& trajectory) const
    {
        if (m_smoothed_trajectory.empty()) return false;
        return (trajectory.front() - m_smoothed_trajectory.back()).norm() > m_max_jump_distance;
    }

    /**
     * @brief Ensures the trajectory remains stable over multiple frames.
     */
    [[nodiscard]] bool checkStability(const std::vector<Eigen::Vector2d>& trajectory) const
    {
        int similar_count = 0;
        for (const auto& [ts, past] : m_past_trajectories)
        {
            float error = 0.0f;
            for (size_t i = 0; i < trajectory.size(); ++i)
            {
                error += (past[i] - trajectory[i]).norm();
            }

            if (error / trajectory.size() < m_max_jump_distance * 0.5f)
            {
                similar_count++;
            }
        }
        return similar_count >= m_min_persistence_frames;
    }

    /**
     * @brief Applies Kalman filtering.
     */
    [[nodiscard]] std::vector<Eigen::Vector2d> applyFiltering(const std::vector<Eigen::Vector2d>& trajectory, DurationT delta_t)
    {
        const bool empty_trajectory = trajectory.size() < 2;

        if (empty_trajectory == false && m_fixed_sample_size != trajectory.size())
        {
            std::cerr << "Trajectory expected to have " << m_fixed_sample_size << " points, had " << trajectory.size() << std::endl;
        }

        if (m_fixed_sample_size != m_kalman_filters.size())
        {
            std::cerr << "Expected to have " << m_fixed_sample_size << " kalman filters, had " << m_kalman_filters.size() << std::endl;
        }

        std::vector<Eigen::Vector2<T>> filtered_trajectory;
        for (size_t i = 0; i < m_fixed_sample_size; ++i)
        {
            m_kalman_filters[i].predict(delta_t);
            if (empty_trajectory == false)
            {
                m_kalman_filters[i].update(trajectory[i]);
            }

            filtered_trajectory.push_back(m_kalman_filters[i].getFilteredPosition().template cast<double>());
        }

        return filtered_trajectory;
    }
};

}