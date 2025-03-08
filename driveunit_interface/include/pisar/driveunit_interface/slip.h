#pragma once

#include "pisar/utilities/fixed_vector.h"

#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace pisar::driveunit_interface {

/**
 * @brief Internal constants for SLIP encoding/decoding.
 */
namespace detail {
    constexpr std::byte kSlipEnd = std::byte(0xC0);
    constexpr std::byte kSlipEsc = std::byte(0xDB);
    constexpr std::byte kSlipEscEnd = std::byte(0xDC);
    constexpr std::byte kSlipEscEsc = std::byte(0xDD);
} // namespace detail

/**
 * @brief SLIP Encoder for packet framing.
 */
class SlipEncoder {
public:
    /**
     * @brief Encodes the given input buffer using SLIP encoding.
     *
     * @param input The input buffer.
     * @param output_buffer The output buffer to write the encoded data.
     * @return constexpr std::optional<std::span<std::byte>> The encoded span or nullopt on failure.
     */
    constexpr std::optional<std::span<std::byte>> encode(
        const std::span<const std::byte> input, const std::span<std::byte> output_buffer
    )
    {
        if (output_buffer.size() < maxEncodedSize(output_buffer.size()))
        {
            return std::nullopt; // Ensure buffer is large enough.
        }

        size_t index = 0;
        output_buffer[index++] = detail::kSlipEnd; // Start with SLIP_END

        for (std::byte byte : input) {
            if (byte == detail::kSlipEnd)
            {
                output_buffer[index++] = detail::kSlipEsc;
                output_buffer[index++] = detail::kSlipEscEnd;
            }
            else if (byte == detail::kSlipEsc)
            {
                output_buffer[index++] = detail::kSlipEsc;
                output_buffer[index++] = detail::kSlipEscEsc;
            }
            else
            {
                output_buffer[index++] = byte;
            }
        }

        output_buffer[index++] = detail::kSlipEnd; // End with SLIP_END
        return output_buffer.subspan(0, index);
    }

    /**
     * @brief Returns the max encoded size given the @p message_size.
     *
     * @param message_size The size of the message being encoded.
     * @return constexpr size_t The max size of the message after SLIP encoding.
     */
    static inline constexpr size_t maxEncodedSize(size_t message_size)
    {
        return message_size * 2 + 2;
    }
};

/**
 * @brief SLIP Decoder with robust incremental processing.
 *
 * Fully consumes input, extracts multiple packets, and reports errors without
 * disrupting further decoding.
 */
template<size_t tkMaxPacketSize, size_t tkQueueCapacity>
class SlipDecoder {
public:
    /// Maximum packet size to prevent buffer overflow.
    static constexpr size_t kMaxPacketSize = tkMaxPacketSize;
    static constexpr size_t kMaxEncodedPacketSize = SlipEncoder::maxEncodedSize(kMaxPacketSize);

    /// Maximum number of packets stored in the queue.
    static constexpr size_t kQueueCapacity = tkQueueCapacity;

    enum class ErrorType {
        kBufferOverflow,
        kQueueFull,
        kInvalidEscape
    };

private:
    /// Circular queue for storing decoded packets
    CircularQueue<FixedVector<std::byte, kMaxPacketSize>, kQueueCapacity> m_packet_queue;

    /// Current packet buffer
    FixedVector<std::byte, kMaxPacketSize> m_packet_buffer;

    /// State tracking
    bool m_escape;
    size_t m_error_count;

public:
    SlipDecoder() : m_escape(false), m_error_count(0) {}

    /**
     * @brief Submits a new chunk of data for decoding.
     *
     * @param input The incoming data chunk.
     */
    constexpr void submit(std::span<const std::byte> input)
    {
        for (std::byte byte : input)
        {
            if (byte == detail::kSlipEnd)
            {
                finalizePacket();
                continue;
            }

            if (m_escape)
            {
                if (byte == detail::kSlipEscEnd)
                {
                    appendByte(detail::kSlipEnd);
                }
                else if (byte == detail::kSlipEscEsc)
                {
                    appendByte(detail::kSlipEsc);
                }
                else
                {
                    reportError(ErrorType::kInvalidEscape);
                    resetPacketBuffer();
                }
                m_escape = false;
            }
            else if (byte == detail::kSlipEsc)
            {
                m_escape = true;
            }
            else
            {
                appendByte(byte);
            }
        }
    }

    /**
     * @brief Queries for the next available decoded packet.
     *
     * @param output_buffer Buffer to write packet data into.
     * @return The number of bytes written into output buffer if packet was available otherwise std::nullopt.
     */
    constexpr std::optional<size_t> query(const std::span<std::byte> output_buffer)
    {
        if (m_packet_queue.empty())
        {
            return std::nullopt;
        }

        auto& packet = m_packet_queue.front();
        if (output_buffer.size() < packet.size())
        {
            return std::nullopt;
        }

        std::copy(packet.begin(), packet.end(), output_buffer.begin());

        m_packet_queue.pop();

        return packet.size();
    }

    /// @brief Returns the number of assembled packets read to be queried.
    [[nodiscard]] inline constexpr size_t packetsAvailable() const
    {
        return m_packet_queue.size();
    }

    /**
     * @brief Returns the number of errors detected since last reset.
     *
     * @return The error count.
     */
    [[nodiscard]] inline constexpr size_t errorCount() const
    {
        return m_error_count;
    }

    /**
     * @brief Clears the error counter.
     */
    inline constexpr void clearErrors()
    {
        m_error_count = 0;
    }

    /**
     * @brief Resets the entire decoder state, including incomplete packets.
     */
    inline constexpr void reset()
    {
        resetPacketBuffer();
        m_packet_queue.clear();
        m_escape = false;
        clearErrors();
    }

private:
    /**
     * @brief Reports a decoding error.
     *
     * @param error The error type.
     */
    inline constexpr void reportError(ErrorType error)
    {
        ++m_error_count;
    }

    /**
     * @brief Appends a byte to the current packet buffer.
     *
     * @param byte The byte to append.
     */
    constexpr void appendByte(std::byte byte)
    {
        if (m_packet_buffer.size() < kMaxPacketSize)
        {
            m_packet_buffer.push_back(byte);
        }
        else
        {
            reportError(ErrorType::kBufferOverflow);
            resetPacketBuffer();
        }
    }

    /**
     * @brief Finalizes the current packet and adds it to the queue.
     */
    constexpr void finalizePacket()
    {
        if (m_packet_buffer.size() > 0)
        {
            if (!m_packet_queue.full())
            {
                m_packet_queue.emplace(m_packet_buffer);
            }
            else
            {
                reportError(ErrorType::kQueueFull); // Queue full, drop packet
            }
        }
        resetPacketBuffer();
    }

    /**
     * @brief Clears the packet buffer.
     */
    inline constexpr void resetPacketBuffer()
    {
        m_packet_buffer.clear();
    }

};


} // namespace pisar::driveunit_interface
