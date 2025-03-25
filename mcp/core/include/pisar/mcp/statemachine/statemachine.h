#pragma once

#include "pisar/mcp/statemachine/state.h"
#include "pisar/mcp/statemachine/events.h"

#include <boost/sml.hpp>

namespace pisar::mcp {

struct StateMachine {
  auto operator()() const {
    using namespace sml;

    return make_transition_table(
      *state<SearchForTargetState> + event<EventTargetFound> = state<NavigateToTargetState>,
      state<NavigateToTargetState> + event<EventArrivedAtSafeZone> = state<DropOffState>
      // etc.
    );
  }
};

}