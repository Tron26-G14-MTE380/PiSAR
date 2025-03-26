#pragma once

#include "pisar/mcp/vision/line_tracker.h"
#include "pisar/mcp/vision/target_detector.h"
#include "pisar/mcp/vision/target_tracker.h"
#include "pisar/mcp/driveunit/controller.h"

namespace pisar::mcp {

template<class TVideoSource, class TLineTracker, class TTargetDetector, class TTargetTracker>
class RobotContext {
private:
    std::reference_wrapper<TVideoSource> m_video_source;

    // Line tracking
    std::reference_wrapper<TLineTracker> m_line_tracker;

    // Bullseye target detection and tracking
    std::reference_wrapper<TTargetDetector> m_target_detector;          ///< Target detector reference.
    std::reference_wrapper<TTargetTracker> m_target_tracker;            ///< Target tracker reference.

    // Driveunit control interface
    std::reference_wrapper<DriveunitController> m_drive_controller;     ///< driveunit controller interface ref.

public:
    /**
     * @brief Construct the robot context.
     *
     * @param video_source The video source used to query image frames.
     * @param line_tracker Line tracking vision module.
     * @param target_detector Target detector vision module.
     * @param target_tracker Target tracker vision module.
     * @param drive_controller The driveunit controller interface.
     */
    RobotContext(
        TVideoSource& video_source,
        TLineTracker& line_tracker,
        TTargetDetector& target_detector,
        TTargetTracker& target_tracker,
        DriveunitController& drive
    )
        : m_video_source(video_source),
          m_line_tracker(line_tracker),
          m_drive_controller(drive_controller)
    {}

    [[nodiscard]] inline TVideoSource& getVideoSource()
    {
        return m_video_source.get();
    }

    [[nodiscard]] inline TLineTracker& getLineTracker()
    {
        return m_line_tracker.get();
    }

    [[nodiscard]] inline TTargetDetector& getTargetDetector()
    {
        return m_line_tracker.get();
    }

    [[nodiscard]] inline TTargetTracker& getTargetTracker()
    {
        return m_line_tracker.get();
    }

    [[nodiscard]] inline DriveunitController& getDriveunitController()
    {
        return m_drive_controller.get();
    }
};

}