#pragma once

#include "pisar/mcp/vision/target_tracker.h"

namespace pisar::mcp {

/**
 * @brief Detects if the target is in frame through a series of tests.
 *
 */
template<class TTargetTracker>
class TargetDetector {
public:
    using TimestampT = TTargetTracker::TimestampT;
    using DurationT = TimestampT::duration;
    using ClockT = TimestampT::clock;

private:
    std::reference_wrapper<TTargetTracker> m_target_tracker;
    DurationT m_threshold_duration;
    std::optional<TimestampT> m_detection_start_time;

public:
    /**
     * @brief Construct a new Target Tracker object
     *
     * @param target_tracker Target tracker used to detect presence of target.
     */
    TargetDetector(TTargetTracker& target_tracker, DurationT threshold_duration = std::chrono::milliseconds(500)) :
        m_target_tracker(target_tracker), m_threshold_duration(threshold_duration) {}

    /**
     * @brief Scan the current frame and returns if target is in frame.
     *
     * @note It may take multiple frames with the target detected to gain confidence to return true.
     *
     * @param input_frame The frame to scan
     * @param timestamp The associated frame timestamp.
     * @return true if the target was found otherwise false.
     */
    [[nodiscard]] bool scanFrame(const cv::Mat& frame, TimestampT timestamp)
    {
        const bool detected = m_target_tracker.get().track(frame, timestamp).has_value();

        if (detected)
        {
            auto now = ClockT::now();
            if (!m_detection_start_time.has_value())
            {
                m_detection_start_time = now;
            }

            auto duration = now - *m_detection_start_time;
            return duration >= m_threshold_duration;
        }
        else
        {
            m_detection_start_time.reset();
            return false;
        }
    }

    /**
     * @brief Resets the detector state.
     *
     */
    void reset()
    {
        m_target_tracker.get().reset();
        m_detection_start_time.reset();
    }
};

}