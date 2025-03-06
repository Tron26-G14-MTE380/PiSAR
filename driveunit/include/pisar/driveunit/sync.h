#pragma once

#include "FreeRTOS.h"
#include "semphr.h"
#include <chrono>

namespace pisar::driveunit {

/**
 * @brief RAII-based lock guard for Mutex.
 */
template<class TLockPrimitive>
class Lock {
private:
    TLockPrimitive& m_primitive;
public:
    /// @brief Acquires the mutex on construction.
    explicit inline Lock(TLockPrimitive& primitive) noexcept : m_primitive(primitive)
    {
        m_primitive.lock();
    }

    /// @brief Deleted copy constructor.
    Lock(const Lock&) = delete;

    /// @brief Deleted copy assignment operator.
    Lock& operator=(const Lock&) = delete;

    /// @brief Releases the mutex on destruction.
    inline ~Lock()
    {
        m_primitive.unlock();
    }
};

/**
 * @brief Wrapper around a FreeRTOS mutex for thread-safe synchronization.
 */
class Mutex {
private:
    SemaphoreHandle_t m_mutex;

public:
    /// @brief Constructs a new Mutex.
    inline Mutex() noexcept
    {
        m_mutex = xSemaphoreCreateMutex();
    }

    /// @brief Destroys the mutex.
    inline ~Mutex()
    {
        if (m_mutex)
        {
            vSemaphoreDelete(m_mutex);
        }
    }

    /// @brief Deleted copy constructor.
    Mutex(const Mutex&) = delete;

    /// @brief Deleted copy assignment operator.
    Mutex& operator=(const Mutex&) = delete;

    /// @brief Attempts to lock the mutex, blocking indefinitely.
    /// @return `true` if the mutex was acquired, `false` otherwise.
    [[nodiscard]] inline bool lock() noexcept
    {
        return xSemaphoreTake(m_mutex, portMAX_DELAY) == pdTRUE;
    }

    /**
     * @brief
     * @param timeout Duration to wait before failing.
     * @return `true` if the mutex was acquired, `false` otherwise.
     */
    template <typename TRep, typename TPeriod>
    [[nodiscard]] inline bool tryLock(const std::chrono::duration<TRep, TPeriod>& timeout) noexcept
    {
        auto ticks = pdMS_TO_TICKS(std::chrono::duration_cast<std::chrono::milliseconds>(timeout).count());
        return xSemaphoreTake(m_mutex, ticks) == pdTRUE;
    }

    /// @brief Unlocks the mutex.
    inline void unlock() noexcept
    {
        xSemaphoreGive(m_mutex);
    }

    /// @brief Returns the native FreeRTOS handle.
    [[nodiscard]] SemaphoreHandle_t nativeHandle() const noexcept
    {
        return m_mutex;
    }
};

}