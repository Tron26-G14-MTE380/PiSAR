#pragma once

#include <cstdint>

#include "tl/optional.hpp"
#include "tcb/span.hpp"
#include "mpark/variant.hpp"

#include "firmware_interface/message.h"

namespace pisar::interface {

namespace detail {

using VariantDiscriminator = uint32_t;

template<class TVariant>
constexpr tl::optional<tcb::span<uint8_t>> encodeVariant(const TVariant& variant, const tcb::span<uint8_t> encode_buffer)
{
    *((VariantDiscriminator*)encode_buffer.data()) = variant.index();
    const tcb::span<uint8_t> data_buffer = encode_buffer.subspan(sizeof(VariantDiscriminator));
    
    return mpark::visit([&](const auto& data) -> tl::optional<tcb::span<uint8_t>>
    {
        if (sizeof(data) + sizeof(VariantDiscriminator) < encode_buffer.size())
        {
            // Encode buffer is too small
            return tl::nullopt;
        }

        std::memcpy(data_buffer.data(), &data, sizeof(data));
        return encode_buffer.subspan(0,sizeof(data) + sizeof(VariantDiscriminator));
    }, variant);
}

// Recursive base case
template <class TVariant, std::size_t tkSearchIndex=0>
struct VariantDecoderImpl {
    static tl::optional<TVariant> decode(uint32_t targetIndex, const tcb::span<uint8_t>& buffer) 
    {
        if (targetIndex != tkSearchIndex) 
        {
            return VariantDecoderImpl<TVariant, tkSearchIndex + 1>::decode(targetIndex, buffer);
        }

        // If we get here, then we've "found" the variant value
        using VariantValueType = mpark::variant_alternative_t<tkSearchIndex, TVariant>;

        // Ensure there's enough data for the selected type
        if (sizeof(VariantValueType) > buffer.size()) 
        {
            return tl::nullopt;
        }

        VariantValueType value;
        std::memcpy(&value, buffer.data(), sizeof(VariantValueType));
        return value;
    }
};

template<class TVariant>
constexpr tl::optional<TVariant> decodeVariant(const tcb::span<const uint8_t> buffer)
{
    if (buffer.size() < sizeof(VariantDiscriminator)) 
    {
        // Buffer too small to contain variant index
        return tl::nullopt;
    }

    // Read the first 4 bytes as the variant index
    VariantDiscriminator index = 0;
    std::memcpy(&index, buffer.data(), sizeof(VariantDiscriminator));

    const tcb::span<const uint8_t> data_buffer = buffer.subspan(sizeof(VariantDiscriminator));

    // Ensure index is valid
    if (index >= mpark::variant_size_v<TVariant>)
    {
        return tl::nullopt;
    }

    // Recursive branching visitor, basically compiles down to
    // if (index == 0) { // decode into object 0... }
    // else if (index == 1) { // decode into object 1... }
    // else if (index == 2) { // decode into object 2... }
    // ...
    return detail::VariantDecoderImpl<TVariant>::decode(index, buffer);
}

}

template<class TMessage>
class MessageEncoder {
public:
    inline constexpr tl::optional<tcb::span<uint8_t>> encode(const TMessage& message, const tcb::span<uint8_t> encode_buffer)
    {
        return detail::encodeVariant<TMessage>(message, encode_buffer);
    }
};

template<class TMessage>
class MessageDecoder {
public:
    inline constexpr tl::optional<TMessage> decode(const tcb::span<const uint8_t> buffer)
    {
        return detail::decodeVariant<TMessage>(buffer);
    }

};

}
