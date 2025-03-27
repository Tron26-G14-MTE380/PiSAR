#pragma once

#include "pisar/mcp/vision/target_tracker.h"

namespace pisar::mcp {

/**
 * @brief Detects if the target is in frame through a series of tests.
 *
 */
template<class TTargetTracker>
class TargetDetector {
private:
    std::reference_wrapper<TTargetTracker> m_target_tracker;

public:
    /**
     * @brief Construct a new Target Tracker object
     *
     * @param target_tracker Target tracker used to detect presence of target.
     */
    TargetDetector(TTargetTracker& target_tracker) : m_target_tracker(target_tracker) {}

    /**
     * @brief Scan the current frame and returns if target is in frame.
     *
     * @note It may take multiple frames with the target detected to gain confidence to return true.
     *
     * @param input_frame The frame to scan
     * @param timestamp The associated frame timestamp.
     * @return true if the target was found otherwise false.
     */
    [[nodiscard]] bool scanFrame(const cv::Mat& frame, TTargetTracker::TimestampT timestamp)
    {
        return m_target_tracker.get().track(frame, timestamp).has_value();
    }

    /**
     * @brief Resets the detector state.
     *
     */
    void reset()
    {
        m_target_tracker.get().reset();
    }
};

}