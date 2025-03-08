#pragma once

#include "pisar/driveunit_interface/slip.h"

#include <cstdint>
#include <optional>
#include <span>
#include <zpp_bits/zpp_bits.h>

namespace pisar::driveunit_interface {

/**
 * @brief Packet encoder using zpp_bits and SLIP framing.
 */
template<class TPacket, size_t tkMaxPacketSize>
class PacketEncoder {
public:
    static constexpr size_t kMaxPacketSize = tkMaxPacketSize;
    static constexpr size_t kMaxEncodedPacketSize = SlipEncoder::maxEncodedSize(kMaxPacketSize);

    /**
     * @brief Encodes a packet into a user-provided buffer.
     *
     * @param packet The packet to encode.
     * @param encode_buffer The buffer to store encoded data.
     * @return constexpr std::optional<std::span<std::byte>> Encoded span or nullopt on failure.
     */
    constexpr std::optional<std::span<std::byte>> encode(
        const TPacket& packet, std::span<std::byte> encode_buffer
    )
    {
        if (encode_buffer.size() < kMaxEncodedPacketSize)
        {
            return std::nullopt; // Ensure buffer is large enough.
        }

        std::byte temp_buffer[tkMaxPacketSize]; // Temporary buffer for zpp_bits serialization
        auto encoder = zpp::bits::out(temp_buffer);

        if (zpp::bits::failure(encoder(packet)))
        {
            return std::nullopt;
        }

        auto serialized_data = encoder.processed_data();
        SlipEncoder slip;
        return slip.encode(serialized_data, encode_buffer);
    }
};


/**
 * @brief Packet decoder with incremental SLIP decoding.
 */
template<class TPacket, size_t tkMaxPacketSize, size_t tkPacketQueueSize>
class PacketDecoder {
private:
    SlipDecoder<tkMaxPacketSize, tkPacketQueueSize> m_slip_decoder;

public:
    /**
     * @brief Submits a chunk of SLIP-encoded data for processing.
     *
     * @param input The incoming data chunk.
     */
    constexpr void submit(std::span<const std::byte> input)
    {
        m_slip_decoder.submit(input);
    }

    /**
     * @brief Queries for a fully decoded packet.
     *
     * @return std::optional<TPacket> The decoded packet or nullopt if no packet is ready.
     */
    constexpr std::optional<TPacket> query()
    {
        std::array<std::byte, tkMaxPacketSize> slip_buffer;
        auto decoded_size = m_slip_decoder.query(std::span(slip_buffer));
        if (!decoded_size)
        {
            return std::nullopt;
        }

        auto decoder = zpp::bits::in(std::span(slip_buffer.data(), decoded_size.value()));
        TPacket packet;
        if (zpp::bits::failure(decoder(packet)))
        {
            return std::nullopt;
        }

        return packet;
    }

    /// @brief Returns the number of assembled packets read to be queried.
    [[nodiscard]] inline constexpr size_t packetsAvailable() const
    {
        return m_slip_decoder.packetsAvailable();
    }

    /**
     * @brief Returns the number of errors detected since last reset.
     *
     * @return The error count.
     */
    [[nodiscard]] inline constexpr size_t errorCount() const
    {
        return m_slip_decoder.errorCount();
    }

    /**
     * @brief Clears the error counter.
     */
    inline constexpr void clearErrors()
    {
        m_slip_decoder.clearErrors();
    }

    /**
     * @brief Resets the decoder state.
     */
    inline constexpr void reset()
    {
        m_slip_decoder.reset();
    }
};

} // namespace pisar::driveunit_interface
