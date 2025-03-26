#pragma once

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

namespace pisar::mcp {

/**
 * @brief Tracks the target (bullseye) in frame, returning a smoothed position.
 *
 * @tparam TTimestamp The timestamp type used with the captured frame.
 */
template<class TTimestamp>
class TargetTracker {
public:

    /**
     * @brief Scans the input @p frame and returns the position of the target.
     *
     * @param frame The input frame to scan for target position.
     * @param timestamp The associated frame timestamp.
     * @return std::optional<Eigen::Vector2d> The target position if found otherwise std::nullopt.
     */
    [[nodiscard]] std::optional<Eigen::Vector2d> track(const cv::Mat& frame, TTimestamp timestamp)
    {
        return std::nullopt;
    }

    /**
     * @brief Resets the tracker state.
     *
     */
    void reset()
    {

    }
};

}