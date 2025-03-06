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
 * @brief RAII-based lock guard for Mutex.
 */
template<class TLockPrimitive>
class LockIsr {
private:
    TLockPrimitive& m_primitive;
public:
    /// @brief Acquires the mutex on construction.
    explicit inline LockIsr(TLockPrimitive& primitive) noexcept : m_primitive(primitive)
    {
        m_primitive.lockIsr();
    }

    /// @brief Deleted copy constructor.
    LockIsr(const LockIsr&) = delete;

    /// @brief Deleted copy assignment operator.
    LockIsr& operator=(const LockIsr&) = delete;

    /// @brief Releases the mutex on destruction.
    inline ~LockIsr()
    {
        m_primitive.unlockIsr();
    }
};

/**
 * @brief Wrapper around a FreeRTOS semaphore for thread-safe synchronization.
 */

 template<class TDerived>
class SemaphoreBase {
private:
    SemaphoreHandle_t m_semaphore;

public:
    /// @brief Constructs a new semaphore.
    inline SemaphoreBase() noexcept
    {
        m_semaphore = static_cast<TDerived*>(this)->createSemaphore();
        configASSERT(m_semaphore != nullptr);
    }

    /// @brief Destroys the SemaphoreBase.
    inline ~SemaphoreBase()
    {
        if (m_semaphore)
        {
            vSemaphoreDelete(m_semaphore);
        }
    }

    /// @brief Deleted copy constructor.
    SemaphoreBase(const SemaphoreBase&) = delete;

    /// @brief Deleted copy assignment operator.
    SemaphoreBase& operator=(const SemaphoreBase&) = delete;

    /// @brief Attempts to lock the semaphore, blocking indefinitely.
    /// @return `true` if the semaphore was acquired, `false` otherwise.
    [[nodiscard]] inline bool lock() noexcept
    {
        return xSemaphoreTake(m_semaphore, portMAX_DELAY) == pdTRUE;
    }

    /// @brief Attempts to lock the semaphore from an ISR.
    /// @return `true` if the semaphore was acquired, `false` otherwise.
    [[nodiscard]] inline bool lockIsr() noexcept
    {
        return xSemaphoreTakeFromISR(m_semaphore, nullptr) == pdTRUE;
    }

    /**
     * @brief
     * @param timeout Duration to wait before failing.
     * @return `true` if the semaphore was acquired, `false` otherwise.
     */
    template <typename TRep, typename TPeriod>
    [[nodiscard]] inline bool tryLock(const std::chrono::duration<TRep, TPeriod>& timeout) noexcept
    {
        auto ticks = pdMS_TO_TICKS(std::chrono::duration_cast<std::chrono::milliseconds>(timeout).count());
        return xSemaphoreTake(m_semaphore, ticks) == pdTRUE;
    }

    /// @brief Unlocks the semaphore.
    inline void unlock() noexcept
    {
        xSemaphoreGive(m_semaphore);
    }

    /// @brief Unlocks the semaphore from an ISR.
    inline void unlockIsr() noexcept
    {
        xSemaphoreGiveFromISR(m_semaphore, nullptr);
    }

    [[nodiscard]] UBaseType_t count() noexcept
    {
        return uxSemaphoreGetCount(m_semaphore);
    }

    [[nodiscard]] UBaseType_t countIsr() noexcept
    {
        return uxSemaphoreGetCount(m_semaphore);
    }

    /// @brief Returns the native FreeRTOS handle.
    [[nodiscard]] SemaphoreHandle_t nativeHandle() const noexcept
    {
        return m_semaphore;
    }
};

class Mutex : public SemaphoreBase<Mutex>
{
public:

    static inline SemaphoreHandle_t createSemaphore() // TODO: Make protected
    {
        return xSemaphoreCreateMutex();
    }
};

class BinarySemaphore : public SemaphoreBase<BinarySemaphore>
{
public:

    static inline SemaphoreHandle_t createSemaphore() // TODO: Make protected
    {
        return xSemaphoreCreateBinary();
    }
};

}