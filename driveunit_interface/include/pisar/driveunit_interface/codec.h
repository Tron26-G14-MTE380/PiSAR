#pragma once

#include <cstdint>
#include <cstring>
#include <optional>
#include <span>
#include <variant>

#include "zpp_bits.h"

namespace pisar::driveunit_interface {

constexpr std::byte PACKET_HEADER = std::byte(0xAA);
constexpr std::byte PACKET_FOOTER = std::byte(0xFF);

/**
 * @brief Packet encoder used to encode packets.
 * 
 * @tparam TPacket The packet type to encode.
 */
template<class TPacket>
class PacketEncoder {
public:
    /**
     * @brief Encodes a given packet into a buffer.
     * 
     * @param packet The packet to encode.
     * @param encode_buffer The buffer to encode the packet into.
     * @return constexpr std::optional<std::span<std::byte>> Span over the encoded buffer if successful otherwise nullopt. 
     */
    inline constexpr std::optional<std::span<std::byte>> encode(
        const TPacket& packet, const std::span<std::byte> encode_buffer
    )
    {
        auto encoder = zpp::bits::out(encode_buffer);
        
        if (zpp::bits::failure(encoder(PACKET_HEADER)))
        {
            return std::nullopt;
        }

        if (zpp::bits::failure(encoder(packet)))
        {
            return std::nullopt;
        }

        if (zpp::bits::failure(encoder(PACKET_FOOTER)))
        {
            return std::nullopt;
        }

        return encoder.processed_data();
    }
};


/**
 * @brief Mesage encoder used to decode messages.
 * 
 * @tparam TPacket The packet type to decode.
 */
template<class TPacket>
class PacketDecoder {
public:
    /**
     * @brief Decodes a given buffer to the packet. Message must be encoded using the @ref PacketEncoder.
     * 
     * @param encoded_buffer The buffer to decode the packet from. 
     * @return constexpr std::optional<TPacket> The decoded packet if successful otherwise nullopt.
     */
    inline constexpr std::optional<TPacket> decode(const std::span<const std::byte> encoded_buffer)
    {
        auto decoder = zpp::bits::in(encoded_buffer);

        std::byte header;
        if (zpp::bits::failure(decoder(header)) || header != PACKET_HEADER)
        {
            return std::nullopt;
        }
        
        TPacket decoded_packet;
        if (zpp::bits::failure(decoder(decoded_packet)))
        {
            return std::nullopt;
        }

        std::byte footer;
        if (zpp::bits::failure(decoder(footer)) || footer != PACKET_FOOTER)
        {
            return std::nullopt;
        }

        return decoded_packet;
    }
};
}
