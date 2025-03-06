#pragma once

#include "pisar/circular_queue.h"

#include <Eigen/Dense>

#include <optional>

namespace pisar::driveunit {

template<typename TTimeStampRep, typename TData>
struct HistoryRecord
{
    TTimeStampRep timestamp;
    TData value;
};

struct  KinematicPose
{
    Eigen::Vector2f position;
    float orientation;

    /// @brief Default constructor (initializes to zero).
    KinematicPose() noexcept : position(Eigen::Vector2f::Zero()), orientation(0.0f) {}

    /// @brief Constructor with values.
    KinematicPose(const Eigen::Vector2f& pos, float orient) noexcept
        : position(pos), orientation(orient) {}

    /** @name Arithmetic Operators */
    ///@{

    /// @brief Adds another pose (element-wise).
    [[nodiscard]] KinematicPose operator+(const KinematicPose& other) const noexcept
    {
        return {position + other.position, orientation + other.orientation};
    }

    /// @brief Subtracts another pose (element-wise).
    [[nodiscard]] KinematicPose operator-(const KinematicPose& other) const noexcept
    {
        return {position - other.position, orientation - other.orientation};
    }

    /// @brief Adds another pose to this one (element-wise).
    KinematicPose& operator+=(const KinematicPose& other) noexcept
    {
        position += other.position;
        orientation += other.orientation;
        return *this;
    }

    /// @brief Subtracts another pose from this one (element-wise).
    KinematicPose& operator-=(const KinematicPose& other) noexcept
    {
        position -= other.position;
        orientation -= other.orientation;
        return *this;
    }

    ///@}

    /** @name Comparison Operators */
    ///@{

    /// @brief Checks equality with another KinematicPose.
    [[nodiscard]] bool operator==(const KinematicPose& other) const noexcept
    {
        return position.isApprox(other.position) && std::abs(orientation - other.orientation) < 1e-6f;
    }

    /// @brief Checks inequality with another KinematicPose.
    [[nodiscard]] bool operator!=(const KinematicPose& other) const noexcept
    {
        return !(*this == other);
    }
};

template<typename TTimeStampRep>
using KinematicPoseRecord = HistoryRecord<TTimeStampRep, KinematicPose>;

template <typename TTimeStampRep, std::size_t tkCapacity>
class PoseHistory {
public:
    using PoseRecordT = KinematicPoseRecord<TTimeStampRep>;

private:
    CircularQueue<PoseRecordT, tkCapacity, true> m_history; ///< Circular queue storing pose history.

public:
    /**
     * @brief Adds a new pose record to the history.
     * @param timestamp The timestamp of the pose.
     * @param pose The kinematic pose to store.
     */
    constexpr void addRecord(TTimeStampRep timestamp, const KinematicPose& pose) noexcept
    {
        m_history.push({timestamp, pose});
    }

    /**
     * @brief Retrieves the full record at the nearest timestamp.
     * @param time The target timestamp.
     * @return The closest history record.
     */
    [[nodiscard]] constexpr std::optional<PoseRecordT> recordAtNearest(TTimeStampRep time) const noexcept
    {
        if (m_history.empty())
        {
            return std::nullopt;
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
        const auto& before = m_history[before_idx];
        const auto& after = m_history[after_idx];

        if (before_idx == after_idx)
        {
            return before; // No valid interpolation, return nearest
        }

        // Determine which timestamp is closer
        if ((time - before.timestamp) <= (after.timestamp - time))
        {
            return before;
        }

        return after;
    }

    /**
     * @brief Retrieves the pose at the nearest timestamp.
     * @param time The target timestamp.
     * @return The pose of the closest record in time.
     */
    [[nodiscard]] constexpr std::optional<Eigen::Vector2f> poseAtNearest(TTimeStampRep time) const noexcept
    {
        const auto record = recordAtNearest(time);
        if (record)
        {
            return record.value.value;
        }
        else
        {
            return std::nullopt;
        }
    }

    /**
     * @brief Retrieves the position at the nearest timestamp.
     * @param time The target timestamp.
     * @return The position of the closest record in time.
     */
    [[nodiscard]] constexpr std::optional<Eigen::Vector2f> positionAtNearest(TTimeStampRep time) const noexcept
    {
        const auto record = recordAtNearest(time);
        if (record)
        {
            return record.value.value.position;
        }
        else
        {
            return std::nullopt;
        }
    }

    /**
     * @brief Retrieves the orientation at the nearest timestamp.
     * @param time The target timestamp.
     * @return The orientation of the closest record in time.
     */
    [[nodiscard]] constexpr std::optional<float> orientationAtNearest(TTimeStampRep time) const noexcept
    {
        const auto record = recordAtNearest(time);
        if (record)
        {
            return record.value.value.orientation;
        }
        else
        {
            return std::nullopt;
        }
    }

    /**
     * @brief Retrieves the interpolated pose at the given timestamp.
     * @param time The target timestamp.
     * @return The interpolated kinematic pose record.
     */
    [[nodiscard]] constexpr std::optional<KinematicPose> poseAtLinInterp(TTimeStampRep time) const noexcept
    {
        if (m_history.empty())
        {
            return std::nullopt;
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
        const auto& before = m_history[before_idx];
        const auto& after = m_history[after_idx];

        if (before_idx == after_idx)
        {
            return before; // No valid interpolation, return nearest
        }

        const float alpha = static_cast<float>(time - before.timestamp) / static_cast<float>(after.timestamp - before.timestamp);
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
    [[nodiscard]] constexpr std::optional<Eigen::Vector2f> positionAtLinInterp(TTimeStampRep time) const noexcept
    {
        if (m_history.empty())
        {
            return std::nullopt;
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
        const auto& before = m_history[before_idx];
        const auto& after = m_history[after_idx];

        if (before_idx == after_idx)
        {
            return before; // No valid interpolation, return nearest
        }

        const float alpha = static_cast<float>(time - before.timestamp) / static_cast<float>(after.timestamp - before.timestamp);
        return interpPosition(before.value.position, after.value.position, alpha);
    }

    /**
     * @brief Retrieves the interpolated orientation at the given timestamp.
     * @param time The target timestamp.
     * @return The interpolated orientation in radians.
     */
    [[nodiscard]] constexpr std::optional<float> orientationAtLinInterp(TTimeStampRep time) const noexcept
    {
        if (m_history.empty())
        {
            return std::nullopt;
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
        const auto& before = m_history[before_idx];
        const auto& after = m_history[after_idx];

        if (before_idx == after_idx)
        {
            return before; // No valid interpolation, return nearest
        }

        const float alpha = static_cast<float>(time - before.timestamp) / static_cast<float>(after.timestamp - before.timestamp);
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
            pose_record.value -= ref;
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
    [[nodiscard]] constexpr std::pair<std::size_t, std::size_t> surroundingRecords(TTimeStampRep time) const noexcept
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