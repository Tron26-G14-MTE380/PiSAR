#pragma once

#include "pisar/mcp/statemachine/robot_context.h"
#include "pisar/mcp/statemachine/events.h"

#include <optional>

namespace pisar::mcp {

class IState {
protected:
    std::reference_wrapper<RobotContext> m_context;

public:
    virtual IState(RobotContext& ctx) : m_context(ctx) {}
    virtual ~IState() = default;

    /** Called when entering the state */
    virtual void enter() = 0;

    /** Called repeatedly while in this state. Returns optional event */
    virtual std::optional<RobotEvents> update() = 0;

    /** Called before transitioning out of the state */
    virtual void exit() = 0;
};

class StateTracklineToTarget : IState {
public:
    virtual void enter() override;
    virtual std::optional<RobotEvents> update() override;
    virtual void exit() override;
};

class StateGoToTarget : IState {
public:
    virtual void enter() override;
    virtual std::optional<RobotEvents> update() override;
    virtual void exit() override;
};

class StateFindLineAfterRetrieval : IState {
public:
    virtual void enter() override;
    virtual std::optional<RobotEvents> update() override;
    virtual void exit() override;
};

class StateTrackLineToHome : IState {
public:
    virtual void enter() override;
    virtual std::optional<RobotEvents> update() override;
    virtual void exit() override;
};

}