#pragma once

#include <array>
#include <stdexcept>
#include <initializer_list>
#include <algorithm>
#include <iterator>

#include "pisar/utilities/assert.h"

namespace pisar {

/**
 * @brief A fixed-size vector that tracks the number of elements, similar to std::vector but with a fixed capacity.
 * @tparam T The type of elements stored in the vector.
 * @tparam tkCapacity The maximum number of elements the vector can hold.
 */
template <typename T, std::size_t tkCapacity>
class FixedVector {
public:
    // Type aliases for STL compatibility
    using value_type = T;
    using size_type = std::size_t;
    using difference_type = std::ptrdiff_t;
    using reference = T&;
    using const_reference = const T&;
    using pointer = T*;
    using const_pointer = const T*;
    using iterator = typename std::array<T, tkCapacity>::iterator;
    using const_iterator = typename std::array<T, tkCapacity>::const_iterator;
    using reverse_iterator = typename std::array<T, tkCapacity>::reverse_iterator;
    using const_reverse_iterator = typename std::array<T, tkCapacity>::const_reverse_iterator;

private:
    std::array<T, tkCapacity> m_data;  ///< Internal storage.
    size_type m_size = 0; ///< Number of elements in the vector.

public:
    /**
     * @brief Default constructor. Initializes an empty FixedVector.
     */
    constexpr inline FixedVector() noexcept = default;

    /**
     * @brief Constructs a FixedVector with a given size, initializing elements to the default value.
     * @param count The number of elements to initialize.
     * @throw std::length_error If count exceeds tkCapacity.
     */
    constexpr explicit FixedVector(size_type count)
    {
        PISAR_ASSERT_BASIC_EXCEPTION(count <= tkCapacity, std::length_error, "FixedVector: Exceeded capacity");
        m_size = count;
        std::fill(begin(), begin() + count, T{});
    }

    /**
     * @brief Constructs a FixedVector with a given size, initializing elements to a provided value.
     * @param count The number of elements to initialize.
     * @param value The value to assign to each element.
     * @throw std::length_error If count exceeds tkCapacity.
     */
    constexpr FixedVector(size_type count, const T& value)
    {
        PISAR_ASSERT_BASIC_EXCEPTION(count <= tkCapacity, std::length_error, "FixedVector: Exceeded capacity");
        m_size = count;
        std::fill(begin(), begin() + count, value);
    }

    /**
     * @brief Constructs a FixedVector from an initializer list.
     * @param init The initializer list.
     * @throw std::length_error If init.size() exceeds tkCapacity.
     */
    constexpr FixedVector(std::initializer_list<T> init)
    {
        PISAR_ASSERT_BASIC_EXCEPTION(init.size() <= tkCapacity, std::length_error, "FixedVector: Exceeded capacity");
        std::copy(init.begin(), init.end(), begin());
        m_size = init.size();
    }

    /**
     * @brief Returns the number of elements in the FixedVector.
     * @return The number of elements.
     */
    [[nodiscard]] constexpr inline size_type size() const noexcept
    {
        return m_size;
    }

    /**
     * @brief Returns the maximum capacity of the FixedVector.
     * @return The maximum number of elements that can be stored.
     */
    [[nodiscard]] constexpr inline size_type capacity() const noexcept
    {
        return tkCapacity;
    }

    /**
     * @brief Checks if the FixedVector is empty.
     * @return True if empty, false otherwise.
     */
    [[nodiscard]] constexpr inline bool empty() const noexcept
    {
        return m_size == 0;
    }

    /**
     * @brief Clears the FixedVector, setting its size to zero.
     */
    constexpr inline void clear() noexcept
    {
        m_size = 0;
    }

    /**
     * @brief Accesses an element by index with bounds checking.
     * @param index The index of the element.
     * @return Reference to the element.
     * @throw std::out_of_range If index is out of range.
     */
    constexpr inline reference at(size_type index)
    {
        PISAR_ASSERT_BASIC_EXCEPTION(index < m_size, std::out_of_range, "FixedVector::at: Index out of range");
        return m_data[index];
    }

    /**
     * @brief Accesses an element by index with bounds checking (const version).
     * @param index The index of the element.
     * @return Const reference to the element.
     * @throw std::out_of_range If index is out of range.
     */
    constexpr inline const_reference at(size_type index) const
    {
        PISAR_ASSERT_BASIC_EXCEPTION(index < m_size, std::out_of_range, "FixedVector::at: Index out of range");
        return m_data[index];
    }

    /**
     * @brief Accesses an element by index without bounds checking.
     * @param index The index of the element.
     * @return Reference to the element.
     */
    constexpr inline reference operator[](size_type index) noexcept
    {
        return m_data[index];
    }

    /**
     * @brief Accesses an element by index without bounds checking (const version).
     * @param index The index of the element.
     * @return Const reference to the element.
     */
    constexpr inline const_reference operator[](size_type index) const noexcept
    {
        return m_data[index];
    }

    /// @brief Returns a pointer to the internal data.
    constexpr inline pointer data() noexcept
    {
        return m_data.data();
    }

    /// @brief Returns a pointer to the internal data.
    constexpr inline const_pointer data() const noexcept
    {
        return m_data.data();
    }

    /**
     * @brief Returns an iterator to the beginning of the FixedVector.
     * @return Iterator to the first element.
     */
    constexpr inline iterator begin() noexcept
    {
        return m_data.begin();
    }

    /**
     * @brief Returns a const iterator to the beginning of the FixedVector.
     * @return Const iterator to the first element.
     */
    constexpr inline const_iterator begin() const noexcept
    {
        return m_data.begin();
    }

    /**
     * @brief Returns an iterator to the end of the FixedVector.
     * @return Iterator to one past the last element.
     */
    constexpr inline iterator end() noexcept
    {
        return m_data.begin() + m_size;
    }

    /**
     * @brief Returns a const iterator to the end of the FixedVector.
     * @return Const iterator to one past the last element.
     */
    constexpr inline const_iterator end() const noexcept
    {
        return m_data.begin() + m_size;
    }

    /**
     * @brief Returns a reference to the first element.
     * @return Reference to the first element.
     * @throw std::out_of_range If the FixedVector is empty.
     */
    constexpr inline reference front()
    {
        PISAR_ASSERT_BASIC_EXCEPTION(!empty(), std::out_of_range, "FixedVector::front: Empty vector");
        return m_data[0];
    }

    /**
     * @brief Returns a const reference to the first element.
     * @return Const reference to the first element.
     * @throw std::out_of_range If the FixedVector is empty.
     */
    constexpr inline const_reference front() const
    {
        PISAR_ASSERT_BASIC_EXCEPTION(!empty(), std::out_of_range, "FixedVector::front: Empty vector");
        return m_data[0];
    }

    /**
     * @brief Returns a reference to the last element.
     * @return Reference to the last element.
     * @throw std::out_of_range If the FixedVector is empty.
     */
    constexpr inline reference back()
    {
        PISAR_ASSERT_BASIC_EXCEPTION(!empty(), std::out_of_range, "FixedVector::back: Empty vector");
        return m_data[m_size - 1];
    }

    /**
     * @brief Returns a const reference to the last element.
     * @return Const reference to the last element.
     * @throw std::out_of_range If the FixedVector is empty.
     */
    constexpr inline const_reference back() const
    {
        PISAR_ASSERT_BASIC_EXCEPTION(!empty(), std::out_of_range, "FixedVector::back: Empty vector");
        return m_data[m_size - 1];
    }

        /**
     * @brief Adds an element to the end of the FixedVector.
     * @param value The value to add.
     * @throw std::length_error If the FixedVector is already at full capacity.
     */
    constexpr inline void push_back(const T& value)
    {
        PISAR_ASSERT_BASIC_EXCEPTION(m_size < tkCapacity, std::length_error, "FixedVector::push_back: Exceeded capacity");
        m_data[m_size++] = value;
    }

    /**
     * @brief Adds an element to the end of the FixedVector (move version).
     * @param value The value to add (moved).
     * @throw std::length_error If the FixedVector is already at full capacity.
     */
    constexpr inline void push_back(T&& value)
    {
        PISAR_ASSERT_BASIC_EXCEPTION(m_size < tkCapacity, std::length_error, "FixedVector::push_back: Exceeded capacity");
        m_data[m_size++] = std::move(value);
    }

    /**
     * @brief Removes the last element from the FixedVector.
     * @throw std::out_of_range If the FixedVector is empty.
     */
    constexpr inline void pop_back()
    {
        PISAR_ASSERT_BASIC_EXCEPTION(!empty(), std::out_of_range, "FixedVector::pop_back: Empty vector");
        --m_size;
    }

    /**
     * @brief Inserts an element at the specified position.
     * @param pos Iterator pointing to the position where the element should be inserted.
     * @param value The value to insert.
     * @return Iterator pointing to the inserted element.
     * @throw std::length_error If the FixedVector is already at full capacity.
     */
    constexpr iterator insert(iterator pos, const T& value)
    {
        PISAR_ASSERT_BASIC_EXCEPTION(m_size < tkCapacity, std::length_error, "FixedVector::insert: Exceeded capacity");
        PISAR_ASSERT_BASIC_EXCEPTION(pos >= begin() && pos <= end(), std::out_of_range, "FixedVector::insert: Invalid iterator");

        size_type index = std::distance(begin(), pos);
        std::move_backward(begin() + index, end(), end() + 1);
        m_data[index] = value;
        ++m_size;
        return begin() + index;
    }

    /**
     * @brief Erases an element at the specified position.
     * @param pos Iterator pointing to the element to erase.
     * @return Iterator to the next element after the erased one.
     * @throw std::out_of_range If pos is out of range.
     */
    constexpr iterator erase(iterator pos)
    {
        PISAR_ASSERT_BASIC_EXCEPTION(pos >= begin() && pos < end(), std::out_of_range, "FixedVector::erase: Invalid iterator");

        std::move(pos + 1, end(), pos);
        --m_size;
        return pos;
    }

    /**
     * @brief Erases a range of elements.
     * @param first Iterator to the first element to erase.
     * @param last Iterator to one past the last element to erase.
     * @return Iterator to the next element after the erased range.
     * @throw std::out_of_range If first or last are out of range.
     */
    constexpr iterator erase(iterator first, iterator last)
    {
        PISAR_ASSERT_BASIC_EXCEPTION(first >= begin() && first <= end(), std::out_of_range, "FixedVector::erase: Invalid iterator");
        PISAR_ASSERT_BASIC_EXCEPTION(last >= begin() && last <= end(), std::out_of_range, "FixedVector::erase: Invalid iterator");

        size_type count = std::distance(first, last);
        std::move(last, end(), first);
        m_size -= count;
        return first;
    }

    /**
     * @brief Resizes the FixedVector to the specified size.
     * @param new_size The new size of the vector.
     * @param value The value to initialize new elements with.
     * @throw std::length_error If new_size exceeds tkCapacity.
     */
    constexpr void resize(size_type new_size, const T& value = T{})
    {
        PISAR_ASSERT_BASIC_EXCEPTION(new_size <= tkCapacity, std::length_error, "FixedVector::resize: Exceeded capacity");

        if (new_size > m_size) {
            std::fill(begin() + m_size, begin() + new_size, value);
        }
        m_size = new_size;
    }

    /**
     * @brief Swaps the contents of this FixedVector with another FixedVector of the same capacity.
     * @param other The FixedVector to swap with.
     */
    constexpr void swap(FixedVector& other) noexcept
    {
        std::swap_ranges(begin(), end(), other.begin());
        std::swap(m_size, other.m_size);
    }

    /**
     * @brief Comparison operators for FixedVector.
     */
    [[nodiscard]] constexpr inline bool operator==(const FixedVector& other) const noexcept
    {
        return m_size == other.m_size && std::equal(begin(), end(), other.begin());
    }

    [[nodiscard]] constexpr inline bool operator!=(const FixedVector& other) const noexcept
    {
        return !(*this == other);
    }

    [[nodiscard]] constexpr inline bool operator<(const FixedVector& other) const noexcept
    {
        return std::lexicographical_compare(begin(), end(), other.begin(), other.end());
    }

    [[nodiscard]] constexpr inline bool operator<=(const FixedVector& other) const noexcept
    {
        return !(other < *this);
    }

    [[nodiscard]] constexpr inline bool operator>(const FixedVector& other) const noexcept
    {
        return other < *this;
    }

    [[nodiscard]] constexpr inline bool operator>=(const FixedVector& other) const noexcept
    {
        return !(*this < other);
    }
};

}