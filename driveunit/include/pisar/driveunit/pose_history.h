#pragma once

#include "pisar/circular_queue.h"

#include <Eigen/Dense>

namespace pisar::driveunit {

template<typename TData>
struct HistoryRecord
{
    uint64_t timestamp;
    TData value;
};

struct KinematicPose
{
    Eigen::Vector2f position;
    float orientation;
};

using KinematicPoseRecord = HistoryRecord<KinematicPose>;

template <std::size_t tkCapacity>
class PoseHistory {
private:
    CircularQueue<KinematicPoseRecord, tkCapacity, true> m_history; ///< Circular queue storing pose history.

public:
    /**
     * @brief Adds a new pose record to the history.
     * @param timestamp The timestamp of the pose.
     * @param pose The kinematic pose to store.
     */
    constexpr void addRecord(uint64_t timestamp, const KinematicPose& pose) noexcept
    {
        m_history.push({timestamp, pose});
    }

    /**
     * @brief Retrieves the full record at the nearest timestamp.
     * @param time The target timestamp.
     * @return The closest history record.
     */
    [[nodiscard]] constexpr KinematicPoseRecord recordAtNearest(uint64_t time) const noexcept
    {
        if (m_history.empty())
        {
            return {0, {Eigen::Vector2f::Zero(), 0.0f}};
        }

        // Early return if time is before/after the stored range
        if (time <= m_history.front().timestamp)
        {
            return m_history.front();
        }

        if (time >= m_history.back().timestamp)
        {
            return m_history.back();
        }

        const auto [before_idx, after_idx] = surroundingRecords(time);
        if (before_idx == after_idx)
        {
            return before; // No valid interpolation, return nearest
        }

        const auto& before = m_history[before_idx];
        const auto& after = m_history[after_idx];

        // Determine which timestamp is closer
        if ((time - before.timestamp) <= (after.timestamp - time))
        {
            return before;
        }

        return after;
    }

    /**
     * @brief Retrieves the position at the nearest timestamp.
     * @param time The target timestamp.
     * @return The position of the closest record in time.
     */
    [[nodiscard]] constexpr Eigen::Vector2f positionAtNearest(uint64_t time) const noexcept
    {
        return recordAtNearest(time).value.position;
    }

    /**
     * @brief Retrieves the orientation at the nearest timestamp.
     * @param time The target timestamp.
     * @return The orientation of the closest record in time.
     */
    [[nodiscard]] constexpr float orientationAtNearest(uint64_t time) const noexcept
    {
        return recordAtNearest(time).value.orientation;
    }

    /**
     * @brief Retrieves the interpolated pose at the given timestamp.
     * @param time The target timestamp.
     * @return The interpolated kinematic pose record.
     */
    [[nodiscard]] constexpr KinematicPose poseAtLinInterp(uint64_t time) const noexcept
    {
        if (m_history.empty())
        {
            return {0, {Eigen::Vector2f::Zero(), 0.0f}};
        }

        // Early return if time is before/after the stored range
        if (time <= m_history.front().timestamp)
        {
            return m_history.front();
        }

        if (time >= m_history.back().timestamp)
        {
            return m_history.back();
        }

        const auto [before_idx, after_idx] = surroundingRecords(time);
        if (before_idx == after_idx)
        {
            return before; // No valid interpolation, return nearest
        }

        const auto& before = m_history[before_idx];
        const auto& after = m_history[after_idx];

        return {
            interpPosition(before.value.position, after.value.position, alpha),
            interpOrienation(before.value.orientation, after.value.orientation, alpha)
        };
    }

    /**
     * @brief Retrieves the interpolated position at the given timestamp.
     * @param time The target timestamp.
     * @return The interpolated position.
     */
    [[nodiscard]] constexpr Eigen::Vector2f positionAtLinInterp(uint64_t time) const noexcept
    {
        if (m_history.empty())
        {
            return {0, {Eigen::Vector2f::Zero(), 0.0f}};
        }

        // Early return if time is before/after the stored range
        if (time <= m_history.front().timestamp)
        {
            return m_history.front();
        }

        if (time >= m_history.back().timestamp)
        {
            return m_history.back();
        }

        const auto [before_idx, after_idx] = surroundingRecords(time);
        if (before_idx == after_idx)
        {
            return before; // No valid interpolation, return nearest
        }

        const auto& before = m_history[before_idx];
        const auto& after = m_history[after_idx];

        return interpPosition(before.value.position, after.value.position, alpha);
    }

    /**
     * @brief Retrieves the interpolated orientation at the given timestamp.
     * @param time The target timestamp.
     * @return The interpolated orientation in radians.
     */
    [[nodiscard]] constexpr float orientationAtLinInterp(uint64_t time) const noexcept
    {
        if (m_history.empty())
        {
            return {0, {Eigen::Vector2f::Zero(), 0.0f}};
        }

        // Early return if time is before/after the stored range
        if (time <= m_history.front().timestamp)
        {
            return m_history.front();
        }

        if (time >= m_history.back().timestamp)
        {
            return m_history.back();
        }

        const auto [before_idx, after_idx] = surroundingRecords(time);
        if (before_idx == after_idx)
        {
            return before; // No valid interpolation, return nearest
        }

        const auto& before = m_history[before_idx];
        const auto& after = m_history[after_idx];

        return interpOrienation(before.value.orientation, after.value.orientation, alpha);
    }

    /**
     * @brief Set the reference pose to the input @p pose.
     *
     * @param ref The reference pose.
     */
    void setReference(const KinematicPose& ref)
    {
        for (auto& pose_record : m_history)
        {
            pose_record.value.position -= ref.position;
            post_record.value.orientation -= ref.orientation;
        }
    }

    /**
     * @brief Set the reference pose to the last item in the history.
     *
     */
    void setReference()
    {
        setReference(m_history.back());
    }

    /**
     * @brief Clears all stored history records.
     */
    constexpr void clearHistory() noexcept
    {
        m_history.clear();
    }

private:

    /**
     * @brief Finds the indices of the two records that bound a given timestamp.
     * @param time The target timestamp.
     * @return A pair of indices (before, after) that bound the given time.
     *         If time is out of range, both indices will be the nearest available.
     */
    [[nodiscard]] constexpr std::pair<std::size_t, std::size_t> surroundingRecords(uint64_t time) const noexcept
    {
        if (m_history.size() < 2)
        {
            return {0, 0}; // Not enough records to interpolate
        }

        if (time <= m_history.front().timestamp)
        {
            return {0, 0}; // Before first record (extrapolation)
        }

        if (time >= m_history.back().timestamp)
        {
            return {m_history.size() - 1, m_history.size() - 1}; // After last record (extrapolation)
        }

        for (std::size_t i = 0; i < m_history.size() - 1; ++i)
        {
            if (m_history[i].timestamp <= time && m_history[i + 1].timestamp >= time)
            {
                return {i, i + 1};
            }
        }

        return {0, 0}; // Should never reach here
    }

    [[nodiscard]] constexpr static inline Eigen::Vector2f interpPosition(const Eigen::Vector2f& pos1, const Eigen::Vector2f& pos2, const float alpha)
    {
        return pos1 + (pos2 - pos1) * alpha;
    }

    [[nodiscard]] constexpr static inline float interpOrientation(const float theta1, const float theta2, const float alpha)
    {
        // Handle wrap-around for angles in radians [-π, π]
        const float delta = std::fmod(theta2 - theta1 + M_PI, 2.0f * M_PI) - M_PI;
        return theta1 + alpha * delta;
    }
};

}