#pragma once

#include "pisar/utilities/type_info.h"

#include "Arduino.h"

#include "FreeRTOS.h"
#include "task.h"

#include <variant>
#include <functional>

namespace pisar::driveunit {

/**
 * @brief Manages and runs the active operating mode.
 * @tparam TOperatingModes List of operating modes.
 */
template<typename TDefaultMode, typename... TOperatingModes>
class OperatingModeManager {
private:
    using ModeT = std::variant<TDefaultMode, TOperatingModes...>;

    std::function<TDefaultMode()> m_default_mode_factory;               ///< Factory function for default mode
    ModeT m_current_mode;                                               ///< Active mode.
    QueueHandle_t m_mode_queue;                                         ///< Queue for pending mode changes
    TaskHandle_t m_task_handle;                                         ///< FreeRTOS task handle for mode execution.

public:
    /**
     * @brief Constructs an OperatingModeManager and starts its execution task.
     * @param task_priority The priority for the mode update task.
     */
    explicit OperatingModeManager(std::function<TDefaultMode()> default_mode_factory) :
        m_default_mode_factory(default_mode_factory), m_current_mode(default_mode_factory()), m_task_handle(nullptr)
    {}

    /// @brief Initialize the operating mode manager and run thread loop.
    void initialize(UBaseType_t task_priority)
    {
        if (task_priority < 0 || task_priority > configMAX_PRIORITIES)
        {
            PISAR_LOG_ERROR("Task priority %u is out of range");
            return; // TODO ERROR CODE
        }

        m_mode_queue = xQueueCreate(1, sizeof(ModeT));
        configASSERT(m_mode_queue != nullptr);

        if (xTaskCreate(taskEntry, "Mode_Manager", 512, this, task_priority, &m_task_handle) != pdPASS)
        {
            PISAR_LOG_ERROR("Failed to create operating mode manager task!");
            return; // TODO ERROR CODE
        }
    }

    /**
     * @brief Switches to a new operating mode with custom parameters. This is thread-safe.
     * @tparam TMode The new operating mode type.
     * @tparam TArgs Types of arguments for mode construction.
     * @param args Arguments to construct the mode.
     */
    template<typename TMode, typename... TArgs>
    void switchMode(TArgs&&... args)
    {
        // Store the new mode, to be applied in the next update cycle
        ModeT new_mode = TMode(std::forward<TArgs>(args)...);
        xQueueOverwrite(m_mode_queue, &new_mode); // Always overwrite the latest mode
    }

    /**
     * @brief Switches to a new operating mode with custom parameters from an ISR. This is thread-safe.
     *
     * This function is designed to be called within an ISR context. It posts a new mode
     * to the FreeRTOS queue without blocking and wakes the mode manager task if needed.
     *
     * @tparam TMode The new operating mode type.
     * @tparam TArgs Types of arguments for mode construction.
     * @param args Arguments to construct the mode.
     */
    template<typename TMode, typename... TArgs>
    void switchModeFromISR(TArgs&&... args)
    {
        ModeT new_mode = TMode(std::forward<TArgs>(args)...);
        BaseType_t taskWoken = pdFALSE;
        xQueueOverwriteFromISR(m_mode_queue, &new_mode, &taskWoken);
        portYIELD_FROM_ISR(taskWoken); // Ensure the mode manager task runs immediately if needed
    }

    /// @brief Switch operating mode to default mode. This is thread-safe.
    void resetToDefaultMode()
    {
        // Store the new mode, to be applied in the next update cycle
        ModeT new_mode = m_default_mode_factory();
        xQueueOverwrite(m_mode_queue, &new_mode); // Always overwrite the latest mode
    }

    /**
     * @brief Switch operating mode to the default mode from an ISR. This is thread-safe.
     *
     * This function is designed to be called within an ISR context. It resets the operating
     * mode back to the default and wakes the mode manager task if needed.
     */
    void resetToDefaultModeFromISR()
    {
        ModeT new_mode = m_default_mode_factory();
        BaseType_t taskWoken = pdFALSE;
        xQueueOverwriteFromISR(m_mode_queue, &new_mode, &taskWoken);
        portYIELD_FROM_ISR(taskWoken); // Ensure the mode manager task runs immediately if needed
    }

private:
    /**
     * @brief FreeRTOS task entry function.
     * @param param Pointer to the OperatingModeManager instance.
     */
    static void taskEntry(void* param)
    {
        reinterpret_cast<OperatingModeManager*>(param)->taskLoop();
    }

    /**
     * @brief Main loop for continuously updating the active mode.
     */
    void taskLoop()
    {
        while (true)
        {
            // If new mode is available and pending, switch to it
            ModeT next_mode = m_default_mode_factory();
            if (xQueueReceive(m_mode_queue, &next_mode, 0) == pdTRUE) // Non-blocking receive
            {
                switchModeImpl(std::move(next_mode));
            }

            // Run current mode
            bool finished = std::visit([](auto& mode) { return mode.update(); }, m_current_mode);

            if (finished)
            {
                switchModeImpl(m_default_mode_factory());
            }

            vTaskDelay(pdMS_TO_TICKS(10)); // Keep updating even if no new mode is received
        }
    }

    /**
     * @brief Switches to a new operating mode by constructing it in place.
     *
     * This function exits the current mode, constructs a new mode of type `TMode`
     * with the provided arguments, and then enters the new mode.
     *
     * @tparam TMode The type of the new operating mode.
     * @tparam TArgs The argument types used for constructing the new mode.
     * @param args The arguments to forward to the new mode's constructor.
     */
    template<typename TMode, typename... TArgs>
    void switchModeImpl(TArgs&&... args)
    {
        exitMode();
        m_current_mode.template emplace<TMode>(std::forward<TArgs>(args)...);
        enterMode();
    }

    /**
     * @brief Switches to a new operating mode by moving an existing mode instance.
     *
     * This function exits the current mode, assigns the given mode instance as
     * the new mode, and then enters the new mode.
     *
     * @param next_mode The new mode instance to move into `m_current_mode`.
     */
    void switchModeImpl(ModeT&& next_mode)
    {
        exitMode();
        m_current_mode = std::move(next_mode);
        enterMode();
    }

    /// @brief Enter the current mode.
    inline void enterMode()
    {
        PISAR_LOG_DEBUG("Entering mode %s", getModeName().data());
        std::visit([](auto& mode) { mode.onEnter(); }, m_current_mode);
    }

    /// @brief Exit the current mode.
    inline void exitMode()
    {
        PISAR_LOG_DEBUG("Exiting mode %s", getModeName().data());
        std::visit([](auto& mode) { mode.onExit(); }, m_current_mode);
    }

    /// @brief Get the current mode's name
    inline std::string_view getModeName()
    {
        return std::visit([](const auto& value) -> std::string_view {
            return TypeName<std::decay_t<decltype(value)>>::value;
        }, m_current_mode);
    }
};

} // namespace pisar::driveunit
