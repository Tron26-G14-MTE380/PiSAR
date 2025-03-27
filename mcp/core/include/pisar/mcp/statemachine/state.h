#pragma once

#include "pisar/mcp/statemachine/robot_context.h"
#include "pisar/mcp/statemachine/events.h"
#include "pisar/utilities/type_info.h"

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

template<class TCommand>
inline bool logCommandResponse(rd::expected<bool, std::error_code> res)
{
    if (res)
    {
        if (!res.value())
        {
            std::cerr << "Command '" << TypeName<TCommand>::value  << "' not acknowledeged!" << std::endl;
            return false;
        }
    }
    else
    {
        std::cerr << "Error command '" << TypeName<TCommand>::value <<"': " << res.error().message() << std::endl;
        return false;
    }

    return true;
}

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
            video_source.start();
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

        const std::optional<CapturedFrame> captured_frame = video_source.getFrame();
        if (!captured_frame.has_value())
        {
            return std::nullopt;
        }

        // See if target is found
        if (target_detector.scanFrame(captured_frame.value().frame, captured_frame.value().timestamp))
        {
            return EventTargetFound{};
        }

        // Extract trajectory data.
        const auto trajectory = line_tracker.extractTrajectory(captured_frame.value().frame, captured_frame.value().timestamp);

        if (trajectory.empty())
        {
            std::cerr << "Line lost" << std::endl;
            const auto res = driveunit_controller.sendHardStopCommand(100);
            logCommandResponse<driveunit_interface::CommandFollowTrajectory>(res);
            return std::nullopt;
        }

        // Convert trajectory to float
        std::vector<Eigen::Vector2f> float_trajectory(trajectory.size(), Eigen::Vector2f::Zero());
        std::transform(trajectory.begin(), trajectory.end(), trajectory.begin(), [](const auto& pt){
            return pt.cast<float>();
        });

        // Send trajectory to driveunit
        const std::chrono::duration<float> trajectory_reference_time = Clock::now() - frame_timestamp;
        const auto res = driveunit_controller.sendTrajectoryCommand(
            CapturedFrame::ClockT::now() - captured_frame.value().timestamp,
            std::span(float_trajectory)
        );

        logCommandResponse<driveunit_interface::CommandFollowTrajectory>(res);

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
        auto& target_tracker = m_context.get().getTargetTracker();

        if (!video_source.isRunning())
        {
            video_source.start();
        }

        target_tracker.reset();
    }

    std::optional<RobotEvent> updateImpl()
    {
        auto& video_source = m_context.get().getVideoSource();
        auto& target_tracker = m_context.get().getTargetTracker();
        auto& driveunit_controller = m_context.get().getDriveunitController();

        const std::optional<CapturedFrame> captured_frame = video_source.getFrame();
        if (!captured_frame.has_value())
        {
            return std::nullopt;
        }

        // Find location of target
        const auto target_position = target_tracker.track(captured_frame.value().frame, captured_frame.value().timestamp);

        // Lost target, no position
        if (!target_position)
        {
            std::cerr << "Lost sight of target" << std::endl;
            return EventTargetLost{};
        }

        // If we reached the target, kill the motors and switch states.
        if (target_position.norm() < gkOnTargetDistanceTolerance)
        {
            const auto res = driveunit_controller.sendHardStopCommand(100);
            logCommandResponse<driveunit_interface::CommandHardStop>(res);

            return EventReachedTarget{};
        }

        // Send trajectory to driveunit
        const auto res = driveunit_controller.sendGoToTargetCommand(
            CapturedFrame::ClockT::now() - captured_frame.value().timestamp,
            target_position
        );

        logCommandResponse<driveunit_interface::CommandGoToTarget>(res);

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
            const auto res = driveunit_controller.sendGripperCommand(true, 100);
            m_grab_cmd_sent = logCommandResponse<driveunit_interface::CommandSetGripper>(res);
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
            const auto res = driveunit_controller.sendRotateCommand(180.0f, 100);
            m_rotate_cmd_sent = logCommandResponse<driveunit_interface::CommandRotate>(res);
        }
        else
        {
            const auto res = driveunit_controller.sendCommandStatusRequest();
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
            video_source.start();
        }

        line_tracker.reset();
    }

    std::optional<RobotEvent> updateImpl()
    {
        auto& video_source = m_context.get().getVideoSource();
        auto& line_tracker = m_context.get().getLineTracker();
        auto& driveunit_controller = m_context.get().getDriveunitController();

        const std::optional<CapturedFrame> captured_frame = video_source.getFrame();
        if (!captured_frame.has_value())
        {
            return std::nullopt;
        }

        // Extract trajectory data.
        const auto trajectory = line_tracker.extractTrajectory(captured_frame.value().frame, captured_frame.value().timestamp);

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
        const auto res = driveunit_controller.sendTrajectoryCommand(
            CapturedFrame::ClockT::now() - captured_frame.value().timestamp,
            std::span(float_trajectory)
        );

        logCommandResponse<driveunit_interface::CommandFollowTrajectory>(res);

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