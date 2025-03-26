#pragma once

#include "pisar/mcp/driveunit/controller.h"
#include "pisar/mcp/vision/line_tracker.h"

namespace pisar::mcp {

template<class TVideoSource, class TLineTracker>
class RobotContext {
private:
    std::reference_wrapper<TVideoSource> m_video_source;

    /** Line tracker interface */
    std::reference_wrapper<TLineTracker> m_line_tracker;

    /** Motor control interface */
    std::reference_wrapper<DriveunitController> m_drive_controller;

public:
    /**
     * @brief Construct the robot context.
     *
     * @param video_source The video source used to query image frames.
     * @param line_tracker Line following vision module.
     * @param drive_controller The driveunit controller interface.
     */
    RobotContext(
        TVideoSource& video_source,
        TLineTracker& line_tracker,
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

    [[nodiscard]] inline DriveunitController& getDriveunitController()
    {
        return m_drive_controller.get();
    }
};

}