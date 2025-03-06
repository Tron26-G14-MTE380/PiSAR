#include <array>
#include <cstddef>
#include <iterator>
#include <algorithm>

namespace pisar {

template <typename T, std::size_t tkCapacity, bool tkOverwrite = false>
class CircularQueue {
public:
    /// The capacity of the circular queue
    static constexpr std::size_t kCapacity = tkCapacity;

    template<bool tkIsConst>
    class Iterator {
    private:
        using QueueType = std::conditional_t<tkIsConst, const CircularQueue, CircularQueue>;
        using ValueType = std::conditional_t<tkIsConst, const T, T>;

        QueueType* m_p_queue;
        std::size_t m_index;
        std::size_t m_count;

    public:
        using iterator_category = std::forward_iterator_tag;
        using value_type = ValueType;
        using difference_type = std::ptrdiff_t;
        using pointer = ValueType*;
        using reference = ValueType&;

        Iterator(QueueType* queue, std::size_t index, std::size_t count)
            : m_p_queue(queue), m_index(index), m_count(count) {}

        bool operator!=(const Iterator& other) const { return m_count != other.m_count; }

        Iterator& operator++()
        {
            m_index = (m_index + 1) % kCapacity;
            ++m_count;
            return *this;
        }

        reference operator*() const { return m_p_queue->m_buffer[m_index]; }
    };

    using IteratorType = Iterator<false>;
    using ConstIteratorType = Iterator<true>;

private:
    std::array<T, tkCapacity> m_buffer; ///< The underlying array buffer for the queue.
    std::size_t m_head;                 ///< The index of the front element.
    std::size_t m_tail;                 ///< The index of the next element to be inserted.
    std::size_t m_size;                 ///< The number of elements in the queue.

public:

    /**
     * @brief Default constructor, initializes the queue as empty.
     */
    constexpr CircularQueue() noexcept : m_head(0), m_tail(0), m_size(0) {}

    /**
     * @brief Copy constructor, copies elements from another circular queue.
     * @param other The queue to copy from.
     */
    constexpr CircularQueue(const CircularQueue& other)
    {
        std::copy(other.begin(), other.begin() + std::min(other.size(), kCapacity), this->m_buffer.begin());
        m_head = other.m_head;
        m_tail = other.m_tail;
        m_size = other.m_size;
    }

    /**
     * @brief Move constructor, moves elements from another circular queue.
     * @param other The queue to move from.
     */
    constexpr CircularQueue(CircularQueue&& other) noexcept
    {
        *this = std::move(other);
    }

    /**
     * @brief Copy assignment operator, copies elements from another circular queue.
     * @param other The queue to copy from.
     * @return A reference to this queue.
     */
    constexpr CircularQueue& operator=(const CircularQueue& other)
    {
        if (this != &other)
        {
            clear();
            std::copy(other.begin(), other.begin() + std::min(other.size(), kCapacity), this->m_buffer.begin());
            m_head = other.m_head;
            m_tail = other.m_tail;
            m_size = other.m_size;
        }
        return *this;
    }

    /**
     * @brief Move assignment operator, moves elements from another circular queue.
     * @param other The queue to move from.
     * @return A reference to this queue.
     */
    constexpr CircularQueue& operator=(CircularQueue&& other) noexcept
    {
        if (this != &other)
        {
            m_head = other.m_head;
            m_tail = other.m_tail;
            m_size = other.m_size;
            m_buffer = std::move(other.m_buffer);
            other.m_size = 0;
        }
        return *this;
    }

    /**
     * @brief Accesses the front element of the queue.
     * @return A reference to the front element.
     * @note Does not throw exceptions; call only when queue is not empty.
     */
    constexpr T& front() noexcept
    {
        return m_buffer[m_head];
    }

    /**
     * @brief Accesses the front element of the queue (const version).
     * @return A const reference to the front element.
     * @note Does not throw exceptions; call only when queue is not empty.
     */
    constexpr const T& front() const noexcept
    {
        return m_buffer[m_head];
    }

    /**
     * @brief Accesses the back element of the queue.
     * @return A reference to the back element.
     * @note Does not throw exceptions; call only when queue is not empty.
     */
    constexpr T& back() noexcept
    {
        return m_buffer[(m_tail == 0) ? kCapacity - 1 : m_tail - 1];
    }

    /**
     * @brief Accesses the back element of the queue (const version).
     * @return A const reference to the back element.
     * @note Does not throw exceptions; call only when queue is not empty.
     */
    constexpr const T& back() const noexcept
    {
        return m_buffer[(m_tail == 0) ? kCapacity - 1 : m_tail - 1];
    }

    /**
     * @brief Checks if the queue is empty.
     * @return True if the queue is empty, false otherwise.
     */
    constexpr bool empty() const noexcept { return m_size == 0; }

    /**
     * @brief Returns the number of elements in the queue.
     * @return The number of elements in the queue.
     */
    constexpr std::size_t size() const noexcept { return m_size; }

    /**
     * @brief Returns the capacity of the queue.
     * @return The capacity of the queue.
     */
    constexpr std::size_t capacity() const noexcept { return kCapacity; }

    /**
     * @brief Returns an iterator to the beginning of the queue.
     * @return A pointer to the first element of the queue.
     */
    constexpr IteratorType begin() noexcept { return IteratorType(this, m_head, 0); }

    /**
     * @brief Returns a const iterator to the beginning of the queue.
     * @return A const pointer to the first element of the queue.
     */
    constexpr ConstIteratorType begin() const noexcept { return ConstIteratorType(this, m_head, 0); }

    /**
     * @brief Returns an iterator to the end of the queue.
     * @return A pointer to the position after the last element of the queue.
     */
    constexpr IteratorType end() noexcept { return IteratorType(this, m_tail, m_size);  }

    /**
     * @brief Returns a const iterator to the end of the queue.
     * @return A const pointer to the position after the last element of the queue.
     */
    constexpr ConstIteratorType end() const noexcept { return ConstIteratorType(this, m_tail, m_size); }

    /**
     * @brief Accesses an element in the queue by index.
     * @param index The index of the element to access.
     * @return A reference to the element at the specified index.
     * @note Index is relative to the front of the queue.
     */
    constexpr T& operator[](std::size_t index) noexcept
    {
        return m_buffer[(m_head + index) % kCapacity];
    }

    /**
     * @brief Accesses an element in the queue by index (const version).
     * @param index The index of the element to access.
     * @return A const reference to the element at the specified index.
     * @note Index is relative to the front of the queue.
     */
    constexpr const T& operator[](std::size_t index) const noexcept
    {
        return m_buffer[(m_head + index) % kCapacity];
    }

    /**
     * @brief Clears the queue, removing all elements.
     */
    constexpr void clear() noexcept
    {
        m_head = m_tail = m_size = 0;
    }

    /**
     * @brief Adds an element to the back of the queue.
     * @param value The value to add to the queue.
     * @return True if the element was added successfully, false if the queue was full and overwriting is disabled.
     */
    [[nodiscard]] constexpr bool push(const T& value) noexcept
    {
        if constexpr (tkOverwrite)
        {
            if (m_size == kCapacity)
            {
                m_head = (m_head + 1) % kCapacity; // Move head forward when overwriting
            }
            else
            {
                ++m_size;
            }
        }
        else
        {
            if (m_size == kCapacity)
            {
                return false; // Queue is full
            }
            ++m_size;
        }

        m_buffer[m_tail] = value;
        m_tail = (m_tail + 1) % kCapacity;
        return true;
    }

    /**
     * @brief Adds an element to the back of the queue (move version).
     * @param value The value to move into the queue.
     * @return True if the element was added successfully, false if the queue was full and overwriting is disabled.
     */
    [[nodiscard]] constexpr bool push(T&& value) noexcept
    {
        if constexpr (tkOverwrite)
        {
            if (m_size == kCapacity)
            {
                m_head = (m_head + 1) % kCapacity; // Move head forward when overwriting
                --m_size;
            }
        }
        else
        {
            if (m_size == kCapacity)
            {
                return false; // Queue is full
            }
        }

        m_buffer[m_tail] = std::move(value);
        m_tail = (m_tail + 1) % kCapacity;
        ++m_size;
        return true;
    }

    /**
     * @brief Removes the front element from the queue.
     * @return True if an element was removed, false if the queue was empty.
     */
    [[nodiscard]] constexpr bool pop() noexcept
    {
        if (empty())
        {
            return false; // Queue is empty
        }

        m_head = (m_head + 1) % kCapacity;
        --m_size;
        return true;
    }

    /**
     * @brief Compares two circular queues for equality.
     * @param lhs The first queue.
     * @param rhs The second queue.
     * @return True if the two queues have the same elements in the same order, false otherwise.
     */
    friend constexpr bool operator==(const CircularQueue& lhs, const CircularQueue& rhs) noexcept
    {
        if (lhs.size() != rhs.size()) return false;
        return std::equal(lhs.begin(), lhs.end(), rhs.begin());
    }

    /**
     * @brief Compares two circular queues for inequality.
     * @param lhs The first queue.
     * @param rhs The second queue.
     * @return True if the two queues differ, false if they are equal.
     */
    friend constexpr bool operator!=(const CircularQueue& lhs, const CircularQueue& rhs) noexcept
    {
        return !(lhs == rhs);
    }
};

}