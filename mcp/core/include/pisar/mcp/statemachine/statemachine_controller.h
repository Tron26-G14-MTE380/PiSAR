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
    RobotState<TRobotContext> m_current_state;

public:
    /**
     * @brief Constructs the state machine controller and initializes the first state.
     *
     * @param ctx Shared robot context
     */
    inline explicit StateMachineController(TRobotContext& ctx)
        : m_context(ctx), m_sm(), m_current_state(StateIdle<TRobotContext>(ctx))
    {
        enterState();
    }

    /**
     * @brief Should be called regularly to update the current state and manage transitions.
     * @return True if finished, false if still going.
     */
    inline bool update()
    {
        auto event = updateState();

        if (event.has_value())
        {
            const bool handled = std::visit([this](const auto& e) -> bool
            {
                return m_sm.process_event(e);
            }, event.value());

            if (!handled)
            {
                throw std::runtime_error("Failed to handle transition");
            }

            if (m_sm.is(sml::X))
            {
                return true;
            }

            m_sm.visit_current_states(
                [this](auto state){
                using Tag = std::decay_t<decltype(state)>;

                // if it's a string<T>, then type = T
                using StateType = typename Tag::type;

                if constexpr (std::is_same_v<StateType, boost::ext::sml::back::terminate_state>)
                {
                }
                else
                {
                    switchState<StateType>();
                }
            });
        }

        return false;
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
        m_current_state.template emplace<TNextState>(TNextState(m_context.get()));
        enterState();
    }

    /// @brief Enter the current state set.
    inline void enterState()
    {
        std::visit([this](auto& state) {
            std::cout << "Entering state: " << SimpleTypeName<decltype(state)>::value << std::endl;
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
            std::cout << "Exiting state: " << SimpleTypeName<decltype(state)>::value << std::endl;
            state.exit();
        }, m_current_state);
    }
};

}