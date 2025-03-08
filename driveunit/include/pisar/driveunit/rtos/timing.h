#pragma once

#include "FreeRTOS.h"
#include <chrono>

namespace pisar::driveunit {
    constexpr auto kMaxDelay = std::chrono::milliseconds::max();

    /**
     * @brief Converts any std::chrono::duration to FreeRTOS ticks.
     * @tparam TRep Type of the representation (integer or floating-point).
     * @tparam TPeriod Type of the ratio (e.g., micro, milli, seconds).
     * @param duration The duration to convert.
     * @return Equivalent FreeRTOS tick count.
     */
    template <typename TRep, typename TPeriod>
    [[nodiscard]] inline constexpr TickType_t toTicks(std::chrono::duration<TRep, TPeriod> duration)
    {
        using namespace std::chrono;

        // Special case: If max duration is provided, return portMAX_DELAY
        if (duration == duration.max())
        {
            return portMAX_DELAY;
        }

        // Convert duration to milliseconds first
        constexpr auto tick_period = duration_cast<nanoseconds>(seconds(1)) / configTICK_RATE_HZ;
        return static_cast<TickType_t>(duration_cast<nanoseconds>(duration).count() / tick_period.count());
    }
};