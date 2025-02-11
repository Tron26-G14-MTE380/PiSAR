#pragma once

#include <cstdint>
#include <cstring>

#include <optional>
#include <span>
#include <variant>

#include "driveunit_interface/message.h"

namespace pisar::driveunit_interface {

namespace detail {
/**
 * @brief The variant discriminator is the type of value placed at the beginning of the encoded buffer to "discriminate" 
 * the variant alternative the buffer holds. Tells the decoder how to decode essentially.
 * 
 */
using VariantDiscriminator = uint32_t;

/**
 * @brief Encodea variant into a serialized buffer.
 * 
 * @tparam TVariant The variant type. 
 * @param variant The variant to encode.
 * @param encode_buffer The buffer to encode into. Must be big enouph to hold the serialized data and discriminator.
 * @return constexpr std::optional<std::span<std::byte>> Optional containing Span of the encoded data 
 * if successful otherwise nullopt.
 */
template<class TVariant>
constexpr std::optional<std::span<std::byte>> encodeVariant(const TVariant& variant, const std::span<std::byte> encode_buffer)
{
    *((VariantDiscriminator*)encode_buffer.data()) = variant.index();
    const std::span<std::byte> data_buffer = encode_buffer.subspan(sizeof(VariantDiscriminator));
    
    return std::visit([&](const auto& data) -> std::optional<std::span<std::byte>>
    {
        if (sizeof(data) + sizeof(VariantDiscriminator) < encode_buffer.size())
        {
            // Encode buffer is too small
            return std::nullopt;
        }

        std::memcpy(data_buffer.data(), &data, sizeof(data));
        return encode_buffer.subspan(0,sizeof(data) + sizeof(VariantDiscriminator));
    }, variant);
}

/**
 * @brief Helper to decode a variant. This struct will recursively call itself with incrementing indices until
 * the target index is reached. This is essentially reduced to a series of if-else comparisons but the neat thing
 * about this approach is that it will work with any variant without having to create a series of custom if-else cases.
 * 
 * @tparam TVariant The variant type.
 * @tparam tkSearchIndex The current index to check.
 */
template <class TVariant, std::size_t tkSearchIndex=0>
struct VariantDecoderImpl {
    /**
     * @brief Tries to decode the variant in the buffer using the current search index. If the search index matches the 
     * target, we decode using the variant alternative type associated with the index otherwise, we increment the
     * search index and keep going.
     * 
     * @param targetIndex The target index of the alternative held in the buffer.
     * @param buffer The buffer containing the encoded serialized variant data.
     * @return constexpr std::optional<TVariant> The decoded variant if successful otherwise nullopt on failure.
     */
    static constexpr std::optional<TVariant> decode(uint32_t targetIndex, const std::span<std::byte>& buffer) 
    {
        if (targetIndex != tkSearchIndex) 
        {
            return VariantDecoderImpl<TVariant, tkSearchIndex + 1>::decode(targetIndex, buffer);
        }

        // If we get here, then we've "found" the variant value
        using VariantValueType = std::variant_alternative_t<tkSearchIndex, TVariant>;

        // Ensure there's enough data for the selected type
        if (sizeof(VariantValueType) > buffer.size()) 
        {
            return std::nullopt;
        }

        VariantValueType value;
        std::memcpy(&value, buffer.data(), sizeof(VariantValueType));
        return value;
    }
};

/**
 * @brief Decodes a buffer into a variant. The buffer must be created using the @ref encodeVariant function. 
 * 
 * @tparam TVariant The variant type to decode into.
 * @param buffer The buffer to decode from.
 * @return constexpr std::optional<TVariant> The decoded variant if successful otherwise nullopt on failure.
 */
template<class TVariant>
constexpr std::optional<TVariant> decodeVariant(const std::span<const std::byte> buffer)
{
    if (buffer.size() < sizeof(VariantDiscriminator)) 
    {
        // Buffer too small to contain variant index
        return std::nullopt;
    }

    // Read the first 4 bytes as the variant index
    VariantDiscriminator index = 0;
    std::memcpy(&index, buffer.data(), sizeof(VariantDiscriminator));

    const std::span<const std::byte> data_buffer = buffer.subspan(sizeof(VariantDiscriminator));

    // Ensure index is valid
    if (index >= std::variant_size_v<TVariant>)
    {
        return std::nullopt;
    }

    // Recursive branching visitor, basically compiles down to
    // if (index == 0) { // decode into object 0... }
    // else if (index == 1) { // decode into object 1... }
    // else if (index == 2) { // decode into object 2... }
    // ...
    return detail::VariantDecoderImpl<TVariant>::decode(index, buffer);
}

}

/**
 * @brief Message encoder used to encode messages.
 * 
 * @tparam TMessageVariant The message variant type to encode.
 */
template<class TMessageVariant>
class MessageEncoder {
public:
    /**
     * @brief Encodes a given message into a buffer.
     * 
     * @param message The message to encode.
     * @param encode_buffer The buffer to encode the message into.
     * @return constexpr std::optional<std::span<std::byte>> Span over the encoded buffer if successful otherwise nullopt. 
     */
    inline constexpr std::optional<std::span<std::byte>> encode(
        const TMessageVariant& message, const std::span<std::byte> encode_buffer
    )
    {
        return detail::encodeVariant<TMessageVariant>(message, encode_buffer);
    }
};

/**
 * @brief Mesage encoder used to decode messages.
 * 
 * @tparam TMessageVariant The message variant type to decode.
 */
template<class TMessageVariant>
class MessageDecoder {
public:
    /**
     * @brief Decodes a given buffer to the message. Message must be encoded using the @ref MessageEncoder.
     * 
     * @param buffer The buffer to decode the message from. 
     * @return constexpr std::optional<TMessageVariant> The decoded message if successful otherwise nullopt.
     */
    inline constexpr std::optional<TMessageVariant> decode(const std::span<const std::byte> buffer)
    {
        return detail::decodeVariant<TMessageVariant>(buffer);
    }

};

template<class TMessage>
inline constexpr size_t encodedSize()
{
    return sizeof(TMessage) + sizeof(detail::VariantDiscriminator);
}

}
