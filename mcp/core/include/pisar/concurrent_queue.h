#pragma once

#include <queue>
#include <mutex>
#include <condition_variable>
#include <optional>
#include <chrono>

namespace pisar::mcp {

/**
 * @brief A thread-safe queue for inter-thread communication with timeout support.
 * @tparam T The type of elements stored in the queue.
 */
template <typename T>
class ConcurrentQueue
{
private:
    std::queue<T> m_queue;               ///< Internal queue storage.
    mutable std::mutex m_mutex;          ///< Mutex for synchronizing access.
    std::condition_variable m_condition; ///< Condition variable for blocking pop/push.

public:
    /**
     * @brief Pushes an item into the queue.
     * @param item The item to add.
     * @param timeout The maximum time to wait if the queue is full (std::nullopt = infinite wait, 0 = non-blocking).
     * @return True if the item was pushed, false if the timeout was reached.
     */
    template <typename TRep, typename TPeriod>
    bool push(T&& item, std::optional<std::chrono::duration<TRep, TPeriod>> timeout = std::nullopt)
    {
        {
            std::unique_lock lock(m_mutex);
            if (timeout && !m_condition.wait_for(lock, *timeout, [this] { return true; }))
            {
                return false; // Timeout reached
            }

            m_queue.push(std::move(item));
        }
        m_condition.notify_one();
        return true;
    }

    /**
     * @brief Pops an item from the queue.
     * @param timeout The maximum time to wait (std::nullopt = infinite wait, 0 = non-blocking).
     * @return An optional containing the item if available before the timeout, otherwise std::nullopt.
     */
    template <typename TRep, typename TPeriod>
    std::optional<T> pop(std::optional<std::chrono::duration<TRep, TPeriod>> timeout = std::nullopt)
    {
        std::unique_lock lock(m_mutex);

        if (timeout)
        {
            if (!m_condition.wait_for(lock, *timeout, [this] { return !m_queue.empty(); }))
            {
                return std::nullopt; // Timed out
            }
        }
        else
        {
            m_condition.wait(lock, [this] { return !m_queue.empty(); });
        }

        T item = std::move(m_queue.front());
        m_queue.pop();
        return item;
    }

    /**
     * @brief Checks if the queue is empty.
     * @return True if the queue is empty, false otherwise.
     */
    [[nodiscard]] inline bool empty() const
    {
        std::scoped_lock lock(m_mutex);
        return m_queue.empty();
    }

    /**
     * @brief Gets the size of the queue.
     * @return The number of elements in the queue.
     */
    [[nodiscard]] inline size_t size() const
    {
        std::scoped_lock lock(m_mutex);
        return m_queue.size();
    }

    /**
     * @brief Clears the queue.
     *
     */
    [[nodiscard]] inline void clear() const
    {
        std::scoped_lock lock(m_mutex);
        while (!m_queue.empty())
        {
            m_queue.pop();
        }
    }
};

}