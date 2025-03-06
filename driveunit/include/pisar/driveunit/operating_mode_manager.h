#pragma once

#include "pisar/driveunit/facility.h"
#include "pisar/driveunit/operating_mode.h"

#include "FreeRTOS.h"
#include "task.h"

namespace pisar::driveunit {

/**
 * @brief Manages and runs the active operating mode.
 * @tparam TOperatingModes List of operating modes.
 */
template<typename... TOperatingModes>
class OperatingModeManager {
private:
    using ModeT = std::variant<OperatingModeIdle, TOperatingModes...>;
    UBaseType_t m_task_priority;
    ModeT m_current_mode;                                               ///< Active mode.
    QueueHandle_t m_mode_queue;                                         ///< Queue for pending mode changes
    TaskHandle_t m_task_handle;                                         ///< FreeRTOS task handle for mode execution.

public:
    /**
     * @brief Constructs an OperatingModeManager and starts its execution task.
     * @param task_priority The priority for the mode update task.
     */
    explicit OperatingModeManager(UBaseType_t task_priority)
        : m_task_priority(task_priority), m_current_mode(OperatingModeIdle{}), m_task_handle(nullptr)
    {}

    /// @brief Initialize the operating mode manager and run thread loop.
    void initialize()
    {
        m_mode_queue = xQueueCreate(1, sizeof(std::variant<OperatingModeIdle, TOperatingModes...>));
        configASSERT(m_mode_queue != nullptr);

        xTaskCreate(
            taskEntry, "Mode_Manager", 2048, this, m_task_priority, &m_task_handle
        );
    }

    /**
     * @brief Switches to a new operating mode with custom parameters.
     * @tparam TMode The new operating mode type.
     * @tparam TArgs Types of arguments for mode construction.
     * @param args Arguments to construct the mode.
     */
    template<typename TMode, typename... TArgs>
    void switchMode(TArgs&&... args)
    {
        // Store the new mode, to be applied in the next update cycle
        std::variant<OperatingModeIdle, TOperatingModes...> new_mode = TMode(std::forward<TArgs>(args)...);
        xQueueOverwrite(m_mode_queue, &new_mode); // Always overwrite the latest mode
    }

private:
    /**
     * @brief FreeRTOS task entry function.
     * @param param Pointer to the OperatingModeManager instance.
     */
    static void taskEntry(void* param)
    {
        static_cast<OperatingModeManager*>(param)->taskLoop();
    }

    /**
     * @brief Main loop for continuously updating the active mode.
     */
    void taskLoop()
    {
        while (true)
        {
            // If new mode is available and pending, switch to it
            std::variant<OperatingModeIdle, TOperatingModes...> next_mode;
            if (xQueueReceive(m_mode_queue, &next_mode, 0) == pdTRUE) // Non-blocking receive
            {
                std::visit([](auto& mode) { mode.onExit(); }, m_current_mode);
                m_current_mode = std::move(next_mode);
                std::visit([](auto& mode) { mode.onEnter(); }, m_current_mode);
            }

            // Run current mode
            bool finished = std::visit([](auto& mode) { return mode.update(); }, m_current_mode);

            if (finished)
            {
                switchMode<OperatingModeIdle>();
            }

            vTaskDelay(pdMS_TO_TICKS(10)); // Keep updating even if no new mode is received
        }
    }
};

} // namespace pisar::driveunit
