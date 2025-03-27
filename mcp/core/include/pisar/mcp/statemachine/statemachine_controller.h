#pragma once

#include "pisar/mcp/statemachine/statemachine.h"

#include "pisar/utilities/type_info.h"

#include <iostream>

namespace pisar::mcp {

/**
 * @brief State machine controller that manages state transitions, lifecycle calls, and SML events.
 */
template<class TRobotContext>
class StateMachineController {
private:
    std::reference_wrapper<TRobotContext> m_context;
    boost::sml::sm<RobotStateMachine<TRobotContext>> m_sm;
    RobotState m_current_state;

public:
    /**
     * @brief Constructs the state machine controller and initializes the first state.
     *
     * @param ctx Shared robot context
     */
    inline explicit StateMachineController(TRobotContext& ctx)
        : m_context(ctx), m_sm(), m_current_state(StateIdle(ctx))
    {
        enterState();
    }

    /**
     * @brief Should be called regularly to update the current state and manage transitions.
     */
    inline void update()
    {
        auto event = updateState();

        if (event.has_value())
        {
            m_sm.process_event(event.value());

            m_sm.visit_current_states([this](auto state){
                using StateType = std::decay_t<decltype(state)>;
                switchState<StateType>();
            });
        }
    }

private:
    /**
     * @brief Switches to a new state by constructing the given type and updating the variant.
     *
     * @tparam TNextState The C++ type of the next state (injected by SML).
     */
    template<typename TNextState>
    inline void switchState()
    {
        exitState();
        m_current_state.emplace(TNextState(m_context));
        enterState();
    }

    /// @brief Enter the current state set.
    inline void enterState()
    {
        std::visit([this](auto& state) {
            std::cout << "Entering state: " << TypeName<std::decay<decltype(state)>>::value << std::endl;
            state.enter();
        }, m_current_state);
    }

    /// @brief Update the current state set.
    [[nodiscard]] inline std::optional<RobotEvent> updateState()
    {
        return std::visit([this](auto& state) -> std::optional<RobotEvent> {
            return state.update();
        }, m_current_state);
    }

    /// @brief exit the current state set.
    inline void exitState()
    {
        std::visit([this](auto& state) {
            std::cout << "Exiting state: " << TypeName<std::decay<decltype(state)>>::value << std::endl;
            state.exit();
        }, m_current_state);
    }
};

}