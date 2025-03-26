#pragma once

#include "pisar/mcp/statemachine/robot_context.h"
#include "pisar/mcp/statemachine/events.h"

#include <optional>

namespace pisar::mcp {

template<typename TDerived, class TRobotContext>
class State {
protected:
    std::reference_wrapper<TRobotContext> m_context;

public:
    State(TRobotContext& ctx) : m_context(ctx) {}
    ~State() = default;

    /** Called when entering the state */
    inline void enter() { return std::static_cast<TDerived*>(this)->enterImpl(); }

    /** Called repeatedly while in this state. Returns optional event */
    [[nodiscard]] inline std::optional<RobotEvent> update() { return std::static_cast<TDerived*>(this)->updateImpl(); }

    /** Called before transitioning out of the state */
    inline void exit() { return std::static_cast<TDerived*>(this)->exitImpl(); }
};

template<class TRobotContext>
class StateIdle : public State<StateIdle, TRobotContext> {
public:
    void enterImpl() {}
    std::optional<RobotEvent> updateImpl() { return std::nullopt; }
    void exitImpl() {}
};

template<class TRobotContext>
class StateTracklineToTarget : public State<StateTracklineToTarget, TRobotContext> {
public:
    using Clock = std::chrono::high_resolution_clock;

    void enterImpl()
    {
        auto& video_source = m_context.get().getVideoSource();
        auto& line_tracker = m_context.get().getLineTracker();
        auto& target_detector = m_context.get().getTargetDetector();

        if (!video_source.isRunning())
        {
            video_source.start(gkCaptureFrameSize);
        }

        line_tracker.reset();
        target_detector.reset();
    }

    std::optional<RobotEvent> updateImpl()
    {
        auto& video_source = m_context.get().getVideoSource();
        auto& line_tracker = m_context.get().getLineTracker();
        auto& target_detector = m_context.get().getTargetDetector();
        auto& driveunit_controller = m_context.get().getDriveunitController();

        std::optional<cv::Mat> frame = video_source.getFrame();
        if (!frame.has_value())
        {
            return std::nullopt;
        }

        const auto now = Clock::now();
        const std::chrono::duration<float> frame_timestamp = now.time_since_epoch();

        // See if target is found
        if (target_detector.scanFrame(frame.value()))
        {
            return EventTargetFound{};
        }

        // Extract trajectory data.
        const auto trajectory = line_tracker.extractTrajectory(*frame, frame_timestamp);

        if (trajectory.empty())
        {
            return std::nullopt;
        }

        // Convert trajectory to float
        std::vector<Eigen::Vector2f> float_trajectory(trajectory.size(), Eigen::Vector2f::Zero());
        std::transform(trajectory.begin(), trajectory.end(), trajectory.begin(), [](const auto& pt){
            return pt.cast<float>();
        });

        // Send trajectory to driveunit
        const std::chrono::duration<float> trajectory_reference_time = Clock::now() - frame_timestamp;
        const auto res = driveunit_controller.sendTrajectoryCommand(trajectory_reference_time, std::span(float_trajectory));
        if (res)
        {
            if (!res.value())
            {
                std::cerr << "Trajectory command not acknowledeged!" << std::endl;
            }
        }
        else
        {
            std::cerr << "Error sending trajectory command: " << res.error().message() << std::endl;
        }

        return std::nullopt;
    }

    void exitImpl() {}
};

template<class TRobotContext>
class StateGoToTarget : public State<StateGoToTarget, TRobotContext> {
public:
    void enterImpl()
    {
        auto& video_source = m_context.get().getVideoSource();
        auto& target_tracker = m_context.get().getLineTracker();

        if (!video_source.isRunning())
        {
            video_source.start(gkCaptureFrameSize);
        }

        target_tracker.reset();
    }

    std::optional<RobotEvent> updateImpl()
    {
        auto& video_source = m_context.get().getVideoSource();
        auto& target_tracker = m_context.get().getTargetTracker();
        auto& driveunit_controller = m_context.get().getDriveunitController();

        std::optional<cv::Mat> frame = video_source.getFrame();
        if (!frame.has_value())
        {
            return std::nullopt;
        }

        const auto now = Clock::now();
        const std::chrono::duration<float> frame_timestamp = now.time_since_epoch();

        // See if target is found
        if (target_detector.scanFrame(frame.value()))
        {
            return EventTargetFound{};
        }

        // Find location of target
        const auto target_position = target_tracker.track(*frame, frame_timestamp);

        // Lost target, no position
        if (!target_position)
        {
            std::cerr << "Lost sight of target" << std::endl;
            return EventTargetLost{};
        }

        // If we reached the target, kill the motors and switch states.
        if (target_position.norm() < gkOnTargetDistanceTolerance)
        {
            const auto res = driveunit_controller.sendHardStopCommand();
            if (res)
            {
                if (!res.value())
                {
                    std::cerr << "'Hard stop' command not acknowledeged!" << std::endl;
                }
            }
            else
            {
                std::cerr << "Error sending 'hard stop' command: " << res.error().message() << std::endl;
            }

            return EventReachedTarget{};
        }

        // Send trajectory to driveunit
        const std::chrono::duration<float> trajectory_reference_time = Clock::now() - frame_timestamp;
        const auto res = driveunit_controller.sendGoToTargetCommand(trajectory_reference_time, target_position);
        if (res)
        {
            if (!res.value())
            {
                std::cerr << "'Go to target' command not acknowledeged!" << std::endl;
            }
        }
        else
        {
            std::cerr << "Error sending 'go to target' command: " << res.error().message() << std::endl;
        }

        return std::nullopt;
    }

    void exitImpl() {}
};

template<class TRobotContext>
class StatePickupTarget : public State<StatePickupTarget, TRobotContext> {
private:
    bool m_grab_cmd_sent;
public:
    void enterImpl()
    {
        m_grab_cmd_sent = false;
    }

    std::optional<RobotEvent> updateImpl()
    {
        auto& driveunit_controller = m_context.get().getDriveunitController();
        if (!m_grab_cmd_sent)
        {
            const auto res = driveunit_controller.sendGrabCommand();
            if (res)
            {
                if (!res.value())
                {
                    std::cerr << "'Grab' command not acknowledeged!" << std::endl;
                }
                else
                {
                    m_grab_cmd_sent = true;
                }
            }
            else
            {
                std::cerr << "Error sending 'grab' command: " << res.error().message() << std::endl;
            }

        }
        else
        {
            const auto res = driveunit_controller.commandInProgress();
            if (res)
            {
                if (res.value() == false)
                {
                    return EventPickedUpTarget {};
                }
            }
            else
            {
                std::cerr << "Error checking whether a command is in progress: " << res.error().message() << std::endl;
            }
        }

        return std::nullopt;
    }

    void exitImpl() {}
};

template<class TRobotContext>
class StateFindLineAfterRetrieval : public State<StateFindLineAfterRetrieval, TRobotContext> {
private:
    bool m_rotate_cmd_sent;
public:
    void enterImpl()
    {
        m_rotate_cmd_sent = false;
    }

    std::optional<RobotEvent> updateImpl()
    {
        auto& driveunit_controller = m_context.get().getDriveunitController();
        if (!m_rotate_cmd_sent)
        {
            const auto res = driveunit_controller.sendGrabCommand();
            if (res)
            {
                if (!res.value())
                {
                    std::cerr << "'Rotate' command not acknowledeged!" << std::endl;
                }
                else
                {
                    m_rotate_cmd_sent = true;
                }
            }
            else
            {
                std::cerr << "Error sending 'Rotate' command: " << res.error().message() << std::endl;
            }

        }
        else
        {
            const auto res = driveunit_controller.commandInProgress();
            if (res)
            {
                if (res.value() == false)
                {
                    return EventFoundLineWithTarget {};
                }
            }
            else
            {
                std::cerr << "Error checking whether a command is in progress: " << res.error().message() << std::endl;
            }
        }

        return std::nullopt;
    }

    void exitImpl() {}
};

template<class TRobotContext>
class StateTrackLineToHome : public State<StateTrackLineToHome, TRobotContext> {
public:
    void enterImpl()
    {
        auto& video_source = m_context.get().getVideoSource();
        auto& line_tracker = m_context.get().getLineTracker();

        if (!video_source.isRunning())
        {
            video_source.start(gkCaptureFrameSize);
        }

        line_tracker.reset();
    }

    std::optional<RobotEvent> updateImpl()
    {
        auto& video_source = m_context.get().getVideoSource();
        auto& line_tracker = m_context.get().getLineTracker();
        auto& driveunit_controller = m_context.get().getDriveunitController();

        std::optional<cv::Mat> frame = video_source.getFrame();
        if (!frame.has_value())
        {
            return std::nullopt;
        }

        const auto now = Clock::now();
        const std::chrono::duration<float> frame_timestamp = now.time_since_epoch();

        // Extract trajectory data.
        const auto trajectory = line_tracker.extractTrajectory(*frame, frame_timestamp);

        if (trajectory.empty())
        {
            return std::nullopt;
        }

        // Convert trajectory to float
        std::vector<Eigen::Vector2f> float_trajectory(trajectory.size(), Eigen::Vector2f::Zero());
        std::transform(trajectory.begin(), trajectory.end(), trajectory.begin(), [](const auto& pt){
            return pt.cast<float>();
        });

        // Send trajectory to driveunit
        const std::chrono::duration<float> trajectory_reference_time = Clock::now() - frame_timestamp;
        const auto res = driveunit_controller.sendTrajectoryCommand(trajectory_reference_time, std::span(float_trajectory));
        if (res)
        {
            if (!res.value())
            {
                std::cerr << "Trajectory command not acknowledeged!" << std::endl;
            }
        }
        else
        {
            std::cerr << "Error sending trajectory command: " << res.error().message() << std::endl;
        }

        return std::nullopt;
    }

    void exitImpl() {}
};

template<class TRobotContext>
class StateFinish : public State<StateFinish, TRobotContext> {
public:
    void enterImpl() {}
    std::optional<RobotEvent> updateImpl() { return std::nullopt; }
    void exitImpl() {}
};

template<class TRobotContext>
using RobotState = std::variant<
    StateIdle<TRobotContext>,
    StateTracklineToTarget<TRobotContext>,
    StateGoToTarget<TRobotContext>,
    StatePickupTarget<TRobotContext>,
    StateFindLineAfterRetrieval<TRobotContext>,
    StateTrackLineToHome<TRobotContext>,
    StateFinish<TRobotContext>
>;

}