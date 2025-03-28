#pragma once

#include "pisar/mcp/statemachine/state.h"
#include "pisar/mcp/statemachine/events.h"

#include <boost/sml.hpp>

namespace sml = boost::sml;

namespace pisar::mcp {

template<class TRobotContext>
struct RobotStateMachine {
    auto operator()() const {
        using namespace sml;

        return make_transition_table(
            *state<StateIdle<TRobotContext>>                   + event<EventStartSearchAndRescue>  = state<StateTracklineToTarget<TRobotContext>>,
            state<StateTracklineToTarget<TRobotContext>>       + event<EventTargetFound>           = state<StateGoToTarget<TRobotContext>>,
            state<StateGoToTarget<TRobotContext>>              + event<EventTargetLost>            = state<StateTracklineToTarget<TRobotContext>>,
            state<StateGoToTarget<TRobotContext>>              + event<EventReachedTarget>         = state<StatePickupTarget<TRobotContext>>,
            state<StatePickupTarget<TRobotContext>>            + event<EventPickedUpTarget>        = state<StateFindLineAfterRetrieval<TRobotContext>>,
            state<StateFindLineAfterRetrieval<TRobotContext>>  + event<EventFoundLineWithTarget>   = state<StateTrackLineToHome<TRobotContext>>,
            state<StateTrackLineToHome<TRobotContext>>         + event<EventReachedHome>           = state<StateFinish<TRobotContext>>,
            state<StateFinish<TRobotContext>>                  + event<EventFinish>                = X
        );
    }
};

}