// Auto-generated from ROS IDL and locked field/interface IDs. Do not edit.
#pragma once
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <type_traits>
#include "dp_protocol_portable.hpp"

namespace dp::iface::tlv {
template <typename T> struct Codec;
namespace detail {
inline void put_u16(uint8_t* output, uint16_t value) { output[0] = static_cast<uint8_t>(value); output[1] = static_cast<uint8_t>(value >> 8U); }
inline uint16_t get_u16(const uint8_t* input) { return static_cast<uint16_t>(input[0]) | static_cast<uint16_t>(input[1]) << 8U; }
inline bool reserve(size_t capacity, size_t* used, size_t length) {
    if (length > UINT16_MAX || *used > capacity || capacity - *used < 4U ||
        length > capacity - *used - 4U) return false;
    *used += 4U + length; return true;
}
inline uint8_t* append(uint8_t* output, size_t capacity, size_t* offset, uint16_t tag, size_t length) {
    if (length > UINT16_MAX || *offset > capacity || capacity - *offset < 4U ||
        length > capacity - *offset - 4U) return nullptr;
    put_u16(output + *offset, tag); put_u16(output + *offset + 2U, static_cast<uint16_t>(length));
    uint8_t* payload = output + *offset + 4U; *offset += 4U + length; return payload;
}
template <typename T> void put_scalar(uint8_t* output, T value) {
    static_assert(std::is_integral<T>::value || std::is_floating_point<T>::value);
    if constexpr (std::is_same<T, bool>::value) { output[0] = value ? 1U : 0U; }
    else {
        static_assert(sizeof(T) == 1U || sizeof(T) == 2U || sizeof(T) == 4U || sizeof(T) == 8U);
        if constexpr (std::is_floating_point<T>::value)
            static_assert(std::numeric_limits<T>::is_iec559);
        using Bits = std::conditional_t<sizeof(T) == 1U, uint8_t,
                     std::conditional_t<sizeof(T) == 2U, uint16_t,
                     std::conditional_t<sizeof(T) == 4U, uint32_t, uint64_t>>>;
        Bits bits{}; std::memcpy(&bits, &value, sizeof(T));
        for (size_t i = 0U; i < sizeof(T); ++i) output[i] = static_cast<uint8_t>(bits >> (i * 8U));
    }
}
template <typename T> bool get_scalar(const uint8_t* input, size_t length, T* value) {
    if (length != sizeof(T) || value == nullptr) return false;
    if constexpr (std::is_same<T, bool>::value) { if (input[0] > 1U) return false; *value = input[0] != 0U; }
    else {
        static_assert(std::is_integral<T>::value || std::is_floating_point<T>::value);
        if constexpr (std::is_floating_point<T>::value)
            static_assert(std::numeric_limits<T>::is_iec559);
        using Bits = std::conditional_t<sizeof(T) == 1U, uint8_t,
                     std::conditional_t<sizeof(T) == 2U, uint16_t,
                     std::conditional_t<sizeof(T) == 4U, uint32_t, uint64_t>>>;
        Bits bits{}; for (size_t i = 0U; i < sizeof(T); ++i) bits |= static_cast<Bits>(input[i]) << (i * 8U);
        std::memcpy(value, &bits, sizeof(T));
    }
    return true;
}
}  // namespace detail

template <> struct Codec<dp::iface::CapabilityAnnounceV1> {
    using Value = dp::iface::CapabilityAnnounceV1;
    static constexpr size_t kMaxEncodedSize = 48U;
    static constexpr size_t kScratchSize = sizeof(Value);
    static constexpr size_t kAlignment = alignof(Value);
    static constexpr uint32_t kInterfaceId = 4101U;
    static bool encode(const Value& message, uint8_t* output, size_t capacity, size_t* used) {
        if (used == nullptr) return false;
        *used = 0U;
        if (output == nullptr) return false;
        size_t required_bytes = 0U;
        if (!detail::reserve(capacity, &required_bytes, message.device_id().size() * 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, message.capabilities().size() * 1U)) return false;
        if (required_bytes > kMaxEncodedSize) return false;
        size_t offset = 0U;
        {
            const auto value = message.device_id();
            const size_t length = value.size() * 1U;
            uint8_t* payload = detail::append(output, capacity, &offset, 1U, length);
            if (payload == nullptr) return false;
            for (size_t index = 0U; index < value.size(); ++index)
                detail::put_scalar(payload + index * 1U, value.data()[index]);
        }
        {
            const auto value = message.protocol_version();
            uint8_t* payload = detail::append(output, capacity, &offset, 2U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.capabilities();
            const size_t length = value.size() * 1U;
            uint8_t* payload = detail::append(output, capacity, &offset, 3U, length);
            if (payload == nullptr) return false;
            for (size_t index = 0U; index < value.size(); ++index)
                detail::put_scalar(payload + index * 1U, value.data()[index]);
        }
        *used = offset;
        return true;
    }
    static bool decode(const uint8_t* input, size_t size, Value* output, Value* scratch) {
        if (input == nullptr || output == nullptr || scratch == nullptr || output == scratch ||
            size > UINT16_MAX) return false;
        *scratch = Value{};
        uint64_t seen = 0U;
        size_t offset = 0U;
        while (offset < size) {
            if (size - offset < 4U) return false;
            const uint16_t tag = detail::get_u16(input + offset);
            const size_t length = detail::get_u16(input + offset + 2U);
            offset += 4U;
            if (tag == 0U || length > size - offset) return false;
            const uint8_t* payload = input + offset;
            switch (tag) {
                case 1U: {
                    if ((seen & (uint64_t{1} << 0U)) != 0U) return false;
                    if (length > 16U || length % 1U != 0U) return false;
                    for (size_t item = 0U; item < length / 1U; ++item)
                        if (!detail::get_scalar(payload + item * 1U, 1U,
                                                &scratch->device_id_.data_[item])) return false;
                    scratch->device_id_.size_ = length / 1U;
                    seen |= uint64_t{1} << 0U;
                    break;
                }
                case 2U: {
                    if ((seen & (uint64_t{1} << 1U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_protocol_version(decoded);
                    seen |= uint64_t{1} << 1U;
                    break;
                }
                case 3U: {
                    if ((seen & (uint64_t{1} << 2U)) != 0U) return false;
                    if (length != 16U) return false;
                    for (size_t item = 0U; item < length / 1U; ++item)
                        if (!detail::get_scalar(payload + item * 1U, 1U,
                                                &scratch->capabilities_.data_[item])) return false;
                    seen |= uint64_t{1} << 2U;
                    break;
                }
                default: break;
            }
            offset += length;
        }
        if ((seen & 7ULL) != 7ULL) return false;
        *output = *scratch;
        return true;
    }
};

template <> struct Codec<dp::iface::CapabilityBitmapV1> {
    using Value = dp::iface::CapabilityBitmapV1;
    static constexpr size_t kMaxEncodedSize = 69U;
    static constexpr size_t kScratchSize = sizeof(Value);
    static constexpr size_t kAlignment = alignof(Value);
    static constexpr uint32_t kInterfaceId = 61442U;
    static bool encode(const Value& message, uint8_t* output, size_t capacity, size_t* used) {
        if (used == nullptr) return false;
        *used = 0U;
        if (output == nullptr) return false;
        size_t required_bytes = 0U;
        if (!detail::reserve(capacity, &required_bytes, message.catalog_fingerprint().size() * 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 2U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 2U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, message.capability_bits().size() * 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 2U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 2U)) return false;
        if (required_bytes > kMaxEncodedSize) return false;
        size_t offset = 0U;
        {
            const auto value = message.catalog_fingerprint();
            const size_t length = value.size() * 1U;
            uint8_t* payload = detail::append(output, capacity, &offset, 1U, length);
            if (payload == nullptr) return false;
            for (size_t index = 0U; index < value.size(); ++index)
                detail::put_scalar(payload + index * 1U, value.data()[index]);
        }
        {
            const auto value = message.page_index();
            uint8_t* payload = detail::append(output, capacity, &offset, 2U, 2U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.page_count();
            uint8_t* payload = detail::append(output, capacity, &offset, 3U, 2U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.valid_bytes();
            uint8_t* payload = detail::append(output, capacity, &offset, 4U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.capability_bits();
            const size_t length = value.size() * 1U;
            uint8_t* payload = detail::append(output, capacity, &offset, 5U, length);
            if (payload == nullptr) return false;
            for (size_t index = 0U; index < value.size(); ++index)
                detail::put_scalar(payload + index * 1U, value.data()[index]);
        }
        {
            const auto value = message.codec_profile_bits();
            uint8_t* payload = detail::append(output, capacity, &offset, 6U, 2U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.qos_bits();
            uint8_t* payload = detail::append(output, capacity, &offset, 7U, 2U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        *used = offset;
        return true;
    }
    static bool decode(const uint8_t* input, size_t size, Value* output, Value* scratch) {
        if (input == nullptr || output == nullptr || scratch == nullptr || output == scratch ||
            size > UINT16_MAX) return false;
        *scratch = Value{};
        uint64_t seen = 0U;
        size_t offset = 0U;
        while (offset < size) {
            if (size - offset < 4U) return false;
            const uint16_t tag = detail::get_u16(input + offset);
            const size_t length = detail::get_u16(input + offset + 2U);
            offset += 4U;
            if (tag == 0U || length > size - offset) return false;
            const uint8_t* payload = input + offset;
            switch (tag) {
                case 1U: {
                    if ((seen & (uint64_t{1} << 0U)) != 0U) return false;
                    if (length != 16U) return false;
                    for (size_t item = 0U; item < length / 1U; ++item)
                        if (!detail::get_scalar(payload + item * 1U, 1U,
                                                &scratch->catalog_fingerprint_.data_[item])) return false;
                    seen |= uint64_t{1} << 0U;
                    break;
                }
                case 2U: {
                    if ((seen & (uint64_t{1} << 1U)) != 0U) return false;
                    if (length != 2U) return false;
                    uint16_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_page_index(decoded);
                    seen |= uint64_t{1} << 1U;
                    break;
                }
                case 3U: {
                    if ((seen & (uint64_t{1} << 2U)) != 0U) return false;
                    if (length != 2U) return false;
                    uint16_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_page_count(decoded);
                    seen |= uint64_t{1} << 2U;
                    break;
                }
                case 4U: {
                    if ((seen & (uint64_t{1} << 3U)) != 0U) return false;
                    if (length != 1U) return false;
                    uint8_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_valid_bytes(decoded);
                    seen |= uint64_t{1} << 3U;
                    break;
                }
                case 5U: {
                    if ((seen & (uint64_t{1} << 4U)) != 0U) return false;
                    if (length != 16U) return false;
                    for (size_t item = 0U; item < length / 1U; ++item)
                        if (!detail::get_scalar(payload + item * 1U, 1U,
                                                &scratch->capability_bits_.data_[item])) return false;
                    seen |= uint64_t{1} << 4U;
                    break;
                }
                case 6U: {
                    if ((seen & (uint64_t{1} << 5U)) != 0U) return false;
                    if (length != 2U) return false;
                    uint16_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_codec_profile_bits(decoded);
                    seen |= uint64_t{1} << 5U;
                    break;
                }
                case 7U: {
                    if ((seen & (uint64_t{1} << 6U)) != 0U) return false;
                    if (length != 2U) return false;
                    uint16_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_qos_bits(decoded);
                    seen |= uint64_t{1} << 6U;
                    break;
                }
                default: break;
            }
            offset += length;
        }
        if ((seen & 127ULL) != 127ULL) return false;
        *output = *scratch;
        return true;
    }
};

template <> struct Codec<dp::iface::ControlWatchdogV1> {
    using Value = dp::iface::ControlWatchdogV1;
    static constexpr size_t kMaxEncodedSize = 34U;
    static constexpr size_t kScratchSize = sizeof(Value);
    static constexpr size_t kAlignment = alignof(Value);
    static constexpr uint32_t kInterfaceId = 4104U;
    static bool encode(const Value& message, uint8_t* output, size_t capacity, size_t* used) {
        if (used == nullptr) return false;
        *used = 0U;
        if (output == nullptr) return false;
        size_t required_bytes = 0U;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (required_bytes > kMaxEncodedSize) return false;
        size_t offset = 0U;
        {
            const auto value = message.timestamp_ms();
            uint8_t* payload = detail::append(output, capacity, &offset, 1U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.deadline_miss_count();
            uint8_t* payload = detail::append(output, capacity, &offset, 2U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.max_jitter_ms();
            uint8_t* payload = detail::append(output, capacity, &offset, 3U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.command_fresh();
            uint8_t* payload = detail::append(output, capacity, &offset, 4U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.state_fresh();
            uint8_t* payload = detail::append(output, capacity, &offset, 5U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        *used = offset;
        return true;
    }
    static bool decode(const uint8_t* input, size_t size, Value* output, Value* scratch) {
        if (input == nullptr || output == nullptr || scratch == nullptr || output == scratch ||
            size > UINT16_MAX) return false;
        *scratch = Value{};
        uint64_t seen = 0U;
        size_t offset = 0U;
        while (offset < size) {
            if (size - offset < 4U) return false;
            const uint16_t tag = detail::get_u16(input + offset);
            const size_t length = detail::get_u16(input + offset + 2U);
            offset += 4U;
            if (tag == 0U || length > size - offset) return false;
            const uint8_t* payload = input + offset;
            switch (tag) {
                case 1U: {
                    if ((seen & (uint64_t{1} << 0U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_timestamp_ms(decoded);
                    seen |= uint64_t{1} << 0U;
                    break;
                }
                case 2U: {
                    if ((seen & (uint64_t{1} << 1U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_deadline_miss_count(decoded);
                    seen |= uint64_t{1} << 1U;
                    break;
                }
                case 3U: {
                    if ((seen & (uint64_t{1} << 2U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_max_jitter_ms(decoded);
                    seen |= uint64_t{1} << 2U;
                    break;
                }
                case 4U: {
                    if ((seen & (uint64_t{1} << 3U)) != 0U) return false;
                    if (length != 1U) return false;
                    bool decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_command_fresh(decoded);
                    seen |= uint64_t{1} << 3U;
                    break;
                }
                case 5U: {
                    if ((seen & (uint64_t{1} << 4U)) != 0U) return false;
                    if (length != 1U) return false;
                    bool decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_state_fresh(decoded);
                    seen |= uint64_t{1} << 4U;
                    break;
                }
                default: break;
            }
            offset += length;
        }
        if ((seen & 31ULL) != 31ULL) return false;
        *output = *scratch;
        return true;
    }
};

template <> struct Codec<dp::iface::CrashAnnounceV1> {
    using Value = dp::iface::CrashAnnounceV1;
    static constexpr size_t kMaxEncodedSize = 122U;
    static constexpr size_t kScratchSize = sizeof(Value);
    static constexpr size_t kAlignment = alignof(Value);
    static constexpr uint32_t kInterfaceId = 4105U;
    static bool encode(const Value& message, uint8_t* output, size_t capacity, size_t* used) {
        if (used == nullptr) return false;
        *used = 0U;
        if (output == nullptr) return false;
        size_t required_bytes = 0U;
        if (!detail::reserve(capacity, &required_bytes, message.server_topic().size())) return false;
        if (!detail::reserve(capacity, &required_bytes, message.filename().size())) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (required_bytes > kMaxEncodedSize) return false;
        size_t offset = 0U;
        {
            const auto value = message.server_topic();
            const size_t length = value.size();
            uint8_t* payload = detail::append(output, capacity, &offset, 1U, length);
            if (payload == nullptr) return false;
            if (length != 0U) std::memcpy(payload, value.data(), length);
        }
        {
            const auto value = message.filename();
            const size_t length = value.size();
            uint8_t* payload = detail::append(output, capacity, &offset, 2U, length);
            if (payload == nullptr) return false;
            if (length != 0U) std::memcpy(payload, value.data(), length);
        }
        {
            const auto value = message.crash_seq();
            uint8_t* payload = detail::append(output, capacity, &offset, 3U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.fault_type();
            uint8_t* payload = detail::append(output, capacity, &offset, 4U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.pc();
            uint8_t* payload = detail::append(output, capacity, &offset, 5U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.lr();
            uint8_t* payload = detail::append(output, capacity, &offset, 6U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.dump_size();
            uint8_t* payload = detail::append(output, capacity, &offset, 7U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.dump_crc32();
            uint8_t* payload = detail::append(output, capacity, &offset, 8U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.uploaded();
            uint8_t* payload = detail::append(output, capacity, &offset, 9U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        *used = offset;
        return true;
    }
    static bool decode(const uint8_t* input, size_t size, Value* output, Value* scratch) {
        if (input == nullptr || output == nullptr || scratch == nullptr || output == scratch ||
            size > UINT16_MAX) return false;
        *scratch = Value{};
        uint64_t seen = 0U;
        size_t offset = 0U;
        while (offset < size) {
            if (size - offset < 4U) return false;
            const uint16_t tag = detail::get_u16(input + offset);
            const size_t length = detail::get_u16(input + offset + 2U);
            offset += 4U;
            if (tag == 0U || length > size - offset) return false;
            const uint8_t* payload = input + offset;
            switch (tag) {
                case 1U: {
                    if ((seen & (uint64_t{1} << 0U)) != 0U) return false;
                    if (length > 32U || scratch->set_server_topic(
                            {reinterpret_cast<const char*>(payload), length}) != SetResult::kOk) return false;
                    seen |= uint64_t{1} << 0U;
                    break;
                }
                case 2U: {
                    if ((seen & (uint64_t{1} << 1U)) != 0U) return false;
                    if (length > 32U || scratch->set_filename(
                            {reinterpret_cast<const char*>(payload), length}) != SetResult::kOk) return false;
                    seen |= uint64_t{1} << 1U;
                    break;
                }
                case 3U: {
                    if ((seen & (uint64_t{1} << 2U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_crash_seq(decoded);
                    seen |= uint64_t{1} << 2U;
                    break;
                }
                case 4U: {
                    if ((seen & (uint64_t{1} << 3U)) != 0U) return false;
                    if (length != 1U) return false;
                    uint8_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_fault_type(decoded);
                    seen |= uint64_t{1} << 3U;
                    break;
                }
                case 5U: {
                    if ((seen & (uint64_t{1} << 4U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_pc(decoded);
                    seen |= uint64_t{1} << 4U;
                    break;
                }
                case 6U: {
                    if ((seen & (uint64_t{1} << 5U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_lr(decoded);
                    seen |= uint64_t{1} << 5U;
                    break;
                }
                case 7U: {
                    if ((seen & (uint64_t{1} << 6U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_dump_size(decoded);
                    seen |= uint64_t{1} << 6U;
                    break;
                }
                case 8U: {
                    if ((seen & (uint64_t{1} << 7U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_dump_crc32(decoded);
                    seen |= uint64_t{1} << 7U;
                    break;
                }
                case 9U: {
                    if ((seen & (uint64_t{1} << 8U)) != 0U) return false;
                    if (length != 1U) return false;
                    bool decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_uploaded(decoded);
                    seen |= uint64_t{1} << 8U;
                    break;
                }
                default: break;
            }
            offset += length;
        }
        if ((seen & 255ULL) != 255ULL) return false;
        *output = *scratch;
        return true;
    }
};

template <> struct Codec<dp::iface::CrashDumpV1> {
    using Value = dp::iface::CrashDumpV1;
    static constexpr size_t kMaxEncodedSize = 7518U;
    static constexpr size_t kScratchSize = sizeof(Value);
    static constexpr size_t kAlignment = alignof(Value);
    static constexpr uint32_t kInterfaceId = 4106U;
    static bool encode(const Value& message, uint8_t* output, size_t capacity, size_t* used) {
        if (used == nullptr) return false;
        *used = 0U;
        if (output == nullptr) return false;
        size_t required_bytes = 0U;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 2U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 2U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, message.fw_version().size())) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, message.core_registers().size() * 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, message.fault_registers().size() * 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, message.task_name().size())) return false;
        if (!detail::reserve(capacity, &required_bytes, message.frames().size() * 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, message.stack_snapshot().size() * 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, message.task_snapshot().size() * 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, message.user_data().size() * 1U)) return false;
        if (required_bytes > kMaxEncodedSize) return false;
        size_t offset = 0U;
        {
            const auto value = message.magic();
            uint8_t* payload = detail::append(output, capacity, &offset, 1U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.version();
            uint8_t* payload = detail::append(output, capacity, &offset, 2U, 2U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.size();
            uint8_t* payload = detail::append(output, capacity, &offset, 3U, 2U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.crc32();
            uint8_t* payload = detail::append(output, capacity, &offset, 4U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.fw_version();
            const size_t length = value.size();
            uint8_t* payload = detail::append(output, capacity, &offset, 5U, length);
            if (payload == nullptr) return false;
            if (length != 0U) std::memcpy(payload, value.data(), length);
        }
        {
            const auto value = message.fault_type();
            uint8_t* payload = detail::append(output, capacity, &offset, 6U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.stack_depth();
            uint8_t* payload = detail::append(output, capacity, &offset, 7U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.uptime_ms();
            uint8_t* payload = detail::append(output, capacity, &offset, 8U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.crash_seq();
            uint8_t* payload = detail::append(output, capacity, &offset, 9U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.core_registers();
            const size_t length = value.size() * 1U;
            uint8_t* payload = detail::append(output, capacity, &offset, 10U, length);
            if (payload == nullptr) return false;
            for (size_t index = 0U; index < value.size(); ++index)
                detail::put_scalar(payload + index * 1U, value.data()[index]);
        }
        {
            const auto value = message.fault_registers();
            const size_t length = value.size() * 1U;
            uint8_t* payload = detail::append(output, capacity, &offset, 11U, length);
            if (payload == nullptr) return false;
            for (size_t index = 0U; index < value.size(); ++index)
                detail::put_scalar(payload + index * 1U, value.data()[index]);
        }
        {
            const auto value = message.task_name();
            const size_t length = value.size();
            uint8_t* payload = detail::append(output, capacity, &offset, 12U, length);
            if (payload == nullptr) return false;
            if (length != 0U) std::memcpy(payload, value.data(), length);
        }
        {
            const auto value = message.frames();
            const size_t length = value.size() * 1U;
            uint8_t* payload = detail::append(output, capacity, &offset, 13U, length);
            if (payload == nullptr) return false;
            for (size_t index = 0U; index < value.size(); ++index)
                detail::put_scalar(payload + index * 1U, value.data()[index]);
        }
        {
            const auto value = message.stack_snapshot();
            const size_t length = value.size() * 1U;
            uint8_t* payload = detail::append(output, capacity, &offset, 14U, length);
            if (payload == nullptr) return false;
            for (size_t index = 0U; index < value.size(); ++index)
                detail::put_scalar(payload + index * 1U, value.data()[index]);
        }
        {
            const auto value = message.task_snapshot();
            const size_t length = value.size() * 1U;
            uint8_t* payload = detail::append(output, capacity, &offset, 15U, length);
            if (payload == nullptr) return false;
            for (size_t index = 0U; index < value.size(); ++index)
                detail::put_scalar(payload + index * 1U, value.data()[index]);
        }
        {
            const auto value = message.user_data();
            const size_t length = value.size() * 1U;
            uint8_t* payload = detail::append(output, capacity, &offset, 16U, length);
            if (payload == nullptr) return false;
            for (size_t index = 0U; index < value.size(); ++index)
                detail::put_scalar(payload + index * 1U, value.data()[index]);
        }
        *used = offset;
        return true;
    }
    static bool decode(const uint8_t* input, size_t size, Value* output, Value* scratch) {
        if (input == nullptr || output == nullptr || scratch == nullptr || output == scratch ||
            size > UINT16_MAX) return false;
        *scratch = Value{};
        uint64_t seen = 0U;
        size_t offset = 0U;
        while (offset < size) {
            if (size - offset < 4U) return false;
            const uint16_t tag = detail::get_u16(input + offset);
            const size_t length = detail::get_u16(input + offset + 2U);
            offset += 4U;
            if (tag == 0U || length > size - offset) return false;
            const uint8_t* payload = input + offset;
            switch (tag) {
                case 1U: {
                    if ((seen & (uint64_t{1} << 0U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_magic(decoded);
                    seen |= uint64_t{1} << 0U;
                    break;
                }
                case 2U: {
                    if ((seen & (uint64_t{1} << 1U)) != 0U) return false;
                    if (length != 2U) return false;
                    uint16_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_version(decoded);
                    seen |= uint64_t{1} << 1U;
                    break;
                }
                case 3U: {
                    if ((seen & (uint64_t{1} << 2U)) != 0U) return false;
                    if (length != 2U) return false;
                    uint16_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_size(decoded);
                    seen |= uint64_t{1} << 2U;
                    break;
                }
                case 4U: {
                    if ((seen & (uint64_t{1} << 3U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_crc32(decoded);
                    seen |= uint64_t{1} << 3U;
                    break;
                }
                case 5U: {
                    if ((seen & (uint64_t{1} << 4U)) != 0U) return false;
                    if (length > 64U || scratch->set_fw_version(
                            {reinterpret_cast<const char*>(payload), length}) != SetResult::kOk) return false;
                    seen |= uint64_t{1} << 4U;
                    break;
                }
                case 6U: {
                    if ((seen & (uint64_t{1} << 5U)) != 0U) return false;
                    if (length != 1U) return false;
                    uint8_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_fault_type(decoded);
                    seen |= uint64_t{1} << 5U;
                    break;
                }
                case 7U: {
                    if ((seen & (uint64_t{1} << 6U)) != 0U) return false;
                    if (length != 1U) return false;
                    uint8_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_stack_depth(decoded);
                    seen |= uint64_t{1} << 6U;
                    break;
                }
                case 8U: {
                    if ((seen & (uint64_t{1} << 7U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_uptime_ms(decoded);
                    seen |= uint64_t{1} << 7U;
                    break;
                }
                case 9U: {
                    if ((seen & (uint64_t{1} << 8U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_crash_seq(decoded);
                    seen |= uint64_t{1} << 8U;
                    break;
                }
                case 10U: {
                    if ((seen & (uint64_t{1} << 9U)) != 0U) return false;
                    if (length > 96U || length % 1U != 0U) return false;
                    for (size_t item = 0U; item < length / 1U; ++item)
                        if (!detail::get_scalar(payload + item * 1U, 1U,
                                                &scratch->core_registers_.data_[item])) return false;
                    scratch->core_registers_.size_ = length / 1U;
                    seen |= uint64_t{1} << 9U;
                    break;
                }
                case 11U: {
                    if ((seen & (uint64_t{1} << 10U)) != 0U) return false;
                    if (length > 20U || length % 1U != 0U) return false;
                    for (size_t item = 0U; item < length / 1U; ++item)
                        if (!detail::get_scalar(payload + item * 1U, 1U,
                                                &scratch->fault_registers_.data_[item])) return false;
                    scratch->fault_registers_.size_ = length / 1U;
                    seen |= uint64_t{1} << 10U;
                    break;
                }
                case 12U: {
                    if ((seen & (uint64_t{1} << 11U)) != 0U) return false;
                    if (length > 16U || scratch->set_task_name(
                            {reinterpret_cast<const char*>(payload), length}) != SetResult::kOk) return false;
                    seen |= uint64_t{1} << 11U;
                    break;
                }
                case 13U: {
                    if ((seen & (uint64_t{1} << 12U)) != 0U) return false;
                    if (length > 256U || length % 1U != 0U) return false;
                    for (size_t item = 0U; item < length / 1U; ++item)
                        if (!detail::get_scalar(payload + item * 1U, 1U,
                                                &scratch->frames_.data_[item])) return false;
                    scratch->frames_.size_ = length / 1U;
                    seen |= uint64_t{1} << 12U;
                    break;
                }
                case 14U: {
                    if ((seen & (uint64_t{1} << 13U)) != 0U) return false;
                    if (length > 2048U || length % 1U != 0U) return false;
                    for (size_t item = 0U; item < length / 1U; ++item)
                        if (!detail::get_scalar(payload + item * 1U, 1U,
                                                &scratch->stack_snapshot_.data_[item])) return false;
                    scratch->stack_snapshot_.size_ = length / 1U;
                    seen |= uint64_t{1} << 13U;
                    break;
                }
                case 15U: {
                    if ((seen & (uint64_t{1} << 14U)) != 0U) return false;
                    if (length > 4676U || length % 1U != 0U) return false;
                    for (size_t item = 0U; item < length / 1U; ++item)
                        if (!detail::get_scalar(payload + item * 1U, 1U,
                                                &scratch->task_snapshot_.data_[item])) return false;
                    scratch->task_snapshot_.size_ = length / 1U;
                    seen |= uint64_t{1} << 14U;
                    break;
                }
                case 16U: {
                    if ((seen & (uint64_t{1} << 15U)) != 0U) return false;
                    if (length > 256U || length % 1U != 0U) return false;
                    for (size_t item = 0U; item < length / 1U; ++item)
                        if (!detail::get_scalar(payload + item * 1U, 1U,
                                                &scratch->user_data_.data_[item])) return false;
                    scratch->user_data_.size_ = length / 1U;
                    seen |= uint64_t{1} << 15U;
                    break;
                }
                default: break;
            }
            offset += length;
        }
        if ((seen & 639ULL) != 639ULL) return false;
        *output = *scratch;
        return true;
    }
};

template <> struct Codec<dp::iface::DeviceStatusTinyV1> {
    using Value = dp::iface::DeviceStatusTinyV1;
    static constexpr size_t kMaxEncodedSize = 19U;
    static constexpr size_t kScratchSize = sizeof(Value);
    static constexpr size_t kAlignment = alignof(Value);
    static constexpr uint32_t kInterfaceId = 4100U;
    static bool encode(const Value& message, uint8_t* output, size_t capacity, size_t* used) {
        if (used == nullptr) return false;
        *used = 0U;
        if (output == nullptr) return false;
        size_t required_bytes = 0U;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 2U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (required_bytes > kMaxEncodedSize) return false;
        size_t offset = 0U;
        {
            const auto value = message.uptime_ms();
            uint8_t* payload = detail::append(output, capacity, &offset, 1U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.voltage_mv();
            uint8_t* payload = detail::append(output, capacity, &offset, 2U, 2U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.flags();
            uint8_t* payload = detail::append(output, capacity, &offset, 3U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        *used = offset;
        return true;
    }
    static bool decode(const uint8_t* input, size_t size, Value* output, Value* scratch) {
        if (input == nullptr || output == nullptr || scratch == nullptr || output == scratch ||
            size > UINT16_MAX) return false;
        *scratch = Value{};
        uint64_t seen = 0U;
        size_t offset = 0U;
        while (offset < size) {
            if (size - offset < 4U) return false;
            const uint16_t tag = detail::get_u16(input + offset);
            const size_t length = detail::get_u16(input + offset + 2U);
            offset += 4U;
            if (tag == 0U || length > size - offset) return false;
            const uint8_t* payload = input + offset;
            switch (tag) {
                case 1U: {
                    if ((seen & (uint64_t{1} << 0U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_uptime_ms(decoded);
                    seen |= uint64_t{1} << 0U;
                    break;
                }
                case 2U: {
                    if ((seen & (uint64_t{1} << 1U)) != 0U) return false;
                    if (length != 2U) return false;
                    uint16_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_voltage_mv(decoded);
                    seen |= uint64_t{1} << 1U;
                    break;
                }
                case 3U: {
                    if ((seen & (uint64_t{1} << 2U)) != 0U) return false;
                    if (length != 1U) return false;
                    uint8_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_flags(decoded);
                    seen |= uint64_t{1} << 2U;
                    break;
                }
                default: break;
            }
            offset += length;
        }
        if ((seen & 7ULL) != 7ULL) return false;
        *output = *scratch;
        return true;
    }
};

template <> struct Codec<dp::iface::DeviceStatusV1> {
    using Value = dp::iface::DeviceStatusV1;
    static constexpr size_t kMaxEncodedSize = 19U;
    static constexpr size_t kScratchSize = sizeof(Value);
    static constexpr size_t kAlignment = alignof(Value);
    static constexpr uint32_t kInterfaceId = 4099U;
    static bool encode(const Value& message, uint8_t* output, size_t capacity, size_t* used) {
        if (used == nullptr) return false;
        *used = 0U;
        if (output == nullptr) return false;
        size_t required_bytes = 0U;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 2U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (required_bytes > kMaxEncodedSize) return false;
        size_t offset = 0U;
        {
            const auto value = message.uptime_ms();
            uint8_t* payload = detail::append(output, capacity, &offset, 1U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.voltage_mv();
            uint8_t* payload = detail::append(output, capacity, &offset, 2U, 2U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.flags();
            uint8_t* payload = detail::append(output, capacity, &offset, 3U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        *used = offset;
        return true;
    }
    static bool decode(const uint8_t* input, size_t size, Value* output, Value* scratch) {
        if (input == nullptr || output == nullptr || scratch == nullptr || output == scratch ||
            size > UINT16_MAX) return false;
        *scratch = Value{};
        uint64_t seen = 0U;
        size_t offset = 0U;
        while (offset < size) {
            if (size - offset < 4U) return false;
            const uint16_t tag = detail::get_u16(input + offset);
            const size_t length = detail::get_u16(input + offset + 2U);
            offset += 4U;
            if (tag == 0U || length > size - offset) return false;
            const uint8_t* payload = input + offset;
            switch (tag) {
                case 1U: {
                    if ((seen & (uint64_t{1} << 0U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_uptime_ms(decoded);
                    seen |= uint64_t{1} << 0U;
                    break;
                }
                case 2U: {
                    if ((seen & (uint64_t{1} << 1U)) != 0U) return false;
                    if (length != 2U) return false;
                    uint16_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_voltage_mv(decoded);
                    seen |= uint64_t{1} << 1U;
                    break;
                }
                case 3U: {
                    if ((seen & (uint64_t{1} << 2U)) != 0U) return false;
                    if (length != 1U) return false;
                    uint8_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_flags(decoded);
                    seen |= uint64_t{1} << 2U;
                    break;
                }
                default: break;
            }
            offset += length;
        }
        if ((seen & 7ULL) != 7ULL) return false;
        *output = *scratch;
        return true;
    }
};

template <> struct Codec<dp::iface::DummyDeviceStatusV1> {
    using Value = dp::iface::DummyDeviceStatusV1;
    static constexpr size_t kMaxEncodedSize = 437U;
    static constexpr size_t kScratchSize = sizeof(Value);
    static constexpr size_t kAlignment = alignof(Value);
    static constexpr uint32_t kInterfaceId = 4131U;
    static bool encode(const Value& message, uint8_t* output, size_t capacity, size_t* used) {
        if (used == nullptr) return false;
        *used = 0U;
        if (output == nullptr) return false;
        size_t required_bytes = 0U;
        if (!detail::reserve(capacity, &required_bytes, 2U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 2U)) return false;
        if (!detail::reserve(capacity, &required_bytes, message.device_id().size() * 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 8U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 8U)) return false;
        if (!detail::reserve(capacity, &required_bytes, message.client_instance_id().size() * 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 2U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 2U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 8U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 8U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 8U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 2U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 2U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, message.contract_sha256().size() * 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, message.model_sha256().size() * 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, message.profile_sha256().size() * 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, message.activation_profile_sha256().size() * 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, message.firmware_sha256().size() * 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, message.board_name_ascii().size() * 1U)) return false;
        if (required_bytes > kMaxEncodedSize) return false;
        size_t offset = 0U;
        {
            const auto value = message.schema_major();
            uint8_t* payload = detail::append(output, capacity, &offset, 1U, 2U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.schema_minor();
            uint8_t* payload = detail::append(output, capacity, &offset, 2U, 2U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.device_id();
            const size_t length = value.size() * 1U;
            uint8_t* payload = detail::append(output, capacity, &offset, 3U, length);
            if (payload == nullptr) return false;
            for (size_t index = 0U; index < value.size(); ++index)
                detail::put_scalar(payload + index * 1U, value.data()[index]);
        }
        {
            const auto value = message.boot_epoch();
            uint8_t* payload = detail::append(output, capacity, &offset, 4U, 8U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.session_id();
            uint8_t* payload = detail::append(output, capacity, &offset, 5U, 8U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.client_instance_id();
            const size_t length = value.size() * 1U;
            uint8_t* payload = detail::append(output, capacity, &offset, 6U, length);
            if (payload == nullptr) return false;
            for (size_t index = 0U; index < value.size(); ++index)
                detail::put_scalar(payload + index * 1U, value.data()[index]);
        }
        {
            const auto value = message.reference_epoch();
            uint8_t* payload = detail::append(output, capacity, &offset, 7U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.motion_epoch();
            uint8_t* payload = detail::append(output, capacity, &offset, 8U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.management_epoch();
            uint8_t* payload = detail::append(output, capacity, &offset, 9U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.profile_id();
            uint8_t* payload = detail::append(output, capacity, &offset, 10U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.core_state();
            uint8_t* payload = detail::append(output, capacity, &offset, 11U, 2U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.fault_reason();
            uint8_t* payload = detail::append(output, capacity, &offset, 12U, 2U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.control_mode();
            uint8_t* payload = detail::append(output, capacity, &offset, 13U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.reference_kind();
            uint8_t* payload = detail::append(output, capacity, &offset, 14U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.drive_request();
            uint8_t* payload = detail::append(output, capacity, &offset, 15U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.drive_evidence();
            uint8_t* payload = detail::append(output, capacity, &offset, 16U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.configured_mask();
            uint8_t* payload = detail::append(output, capacity, &offset, 17U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.real_motion_permitted();
            uint8_t* payload = detail::append(output, capacity, &offset, 18U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.permit_remaining_us();
            uint8_t* payload = detail::append(output, capacity, &offset, 19U, 8U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.capabilities();
            uint8_t* payload = detail::append(output, capacity, &offset, 20U, 8U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.operation_id();
            uint8_t* payload = detail::append(output, capacity, &offset, 21U, 8U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.operation_status();
            uint8_t* payload = detail::append(output, capacity, &offset, 22U, 2U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.operation_result_code();
            uint8_t* payload = detail::append(output, capacity, &offset, 23U, 2U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.operation_result_valid();
            uint8_t* payload = detail::append(output, capacity, &offset, 24U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.command_reject_count();
            uint8_t* payload = detail::append(output, capacity, &offset, 25U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.can_error_count();
            uint8_t* payload = detail::append(output, capacity, &offset, 26U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.deadline_miss_count();
            uint8_t* payload = detail::append(output, capacity, &offset, 27U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.contract_sha256();
            const size_t length = value.size() * 1U;
            uint8_t* payload = detail::append(output, capacity, &offset, 28U, length);
            if (payload == nullptr) return false;
            for (size_t index = 0U; index < value.size(); ++index)
                detail::put_scalar(payload + index * 1U, value.data()[index]);
        }
        {
            const auto value = message.model_sha256();
            const size_t length = value.size() * 1U;
            uint8_t* payload = detail::append(output, capacity, &offset, 29U, length);
            if (payload == nullptr) return false;
            for (size_t index = 0U; index < value.size(); ++index)
                detail::put_scalar(payload + index * 1U, value.data()[index]);
        }
        {
            const auto value = message.profile_sha256();
            const size_t length = value.size() * 1U;
            uint8_t* payload = detail::append(output, capacity, &offset, 30U, length);
            if (payload == nullptr) return false;
            for (size_t index = 0U; index < value.size(); ++index)
                detail::put_scalar(payload + index * 1U, value.data()[index]);
        }
        {
            const auto value = message.activation_profile_sha256();
            const size_t length = value.size() * 1U;
            uint8_t* payload = detail::append(output, capacity, &offset, 31U, length);
            if (payload == nullptr) return false;
            for (size_t index = 0U; index < value.size(); ++index)
                detail::put_scalar(payload + index * 1U, value.data()[index]);
        }
        {
            const auto value = message.motion_supported();
            uint8_t* payload = detail::append(output, capacity, &offset, 32U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.hold_ready();
            uint8_t* payload = detail::append(output, capacity, &offset, 33U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.firmware_sha256();
            const size_t length = value.size() * 1U;
            uint8_t* payload = detail::append(output, capacity, &offset, 34U, length);
            if (payload == nullptr) return false;
            for (size_t index = 0U; index < value.size(); ++index)
                detail::put_scalar(payload + index * 1U, value.data()[index]);
        }
        {
            const auto value = message.board_name_ascii();
            const size_t length = value.size() * 1U;
            uint8_t* payload = detail::append(output, capacity, &offset, 35U, length);
            if (payload == nullptr) return false;
            for (size_t index = 0U; index < value.size(); ++index)
                detail::put_scalar(payload + index * 1U, value.data()[index]);
        }
        *used = offset;
        return true;
    }
    static bool decode(const uint8_t* input, size_t size, Value* output, Value* scratch) {
        if (input == nullptr || output == nullptr || scratch == nullptr || output == scratch ||
            size > UINT16_MAX) return false;
        *scratch = Value{};
        uint64_t seen = 0U;
        size_t offset = 0U;
        while (offset < size) {
            if (size - offset < 4U) return false;
            const uint16_t tag = detail::get_u16(input + offset);
            const size_t length = detail::get_u16(input + offset + 2U);
            offset += 4U;
            if (tag == 0U || length > size - offset) return false;
            const uint8_t* payload = input + offset;
            switch (tag) {
                case 1U: {
                    if ((seen & (uint64_t{1} << 0U)) != 0U) return false;
                    if (length != 2U) return false;
                    uint16_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_schema_major(decoded);
                    seen |= uint64_t{1} << 0U;
                    break;
                }
                case 2U: {
                    if ((seen & (uint64_t{1} << 1U)) != 0U) return false;
                    if (length != 2U) return false;
                    uint16_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_schema_minor(decoded);
                    seen |= uint64_t{1} << 1U;
                    break;
                }
                case 3U: {
                    if ((seen & (uint64_t{1} << 2U)) != 0U) return false;
                    if (length != 16U) return false;
                    for (size_t item = 0U; item < length / 1U; ++item)
                        if (!detail::get_scalar(payload + item * 1U, 1U,
                                                &scratch->device_id_.data_[item])) return false;
                    seen |= uint64_t{1} << 2U;
                    break;
                }
                case 4U: {
                    if ((seen & (uint64_t{1} << 3U)) != 0U) return false;
                    if (length != 8U) return false;
                    uint64_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_boot_epoch(decoded);
                    seen |= uint64_t{1} << 3U;
                    break;
                }
                case 5U: {
                    if ((seen & (uint64_t{1} << 4U)) != 0U) return false;
                    if (length != 8U) return false;
                    uint64_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_session_id(decoded);
                    seen |= uint64_t{1} << 4U;
                    break;
                }
                case 6U: {
                    if ((seen & (uint64_t{1} << 5U)) != 0U) return false;
                    if (length != 16U) return false;
                    for (size_t item = 0U; item < length / 1U; ++item)
                        if (!detail::get_scalar(payload + item * 1U, 1U,
                                                &scratch->client_instance_id_.data_[item])) return false;
                    seen |= uint64_t{1} << 5U;
                    break;
                }
                case 7U: {
                    if ((seen & (uint64_t{1} << 6U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_reference_epoch(decoded);
                    seen |= uint64_t{1} << 6U;
                    break;
                }
                case 8U: {
                    if ((seen & (uint64_t{1} << 7U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_motion_epoch(decoded);
                    seen |= uint64_t{1} << 7U;
                    break;
                }
                case 9U: {
                    if ((seen & (uint64_t{1} << 8U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_management_epoch(decoded);
                    seen |= uint64_t{1} << 8U;
                    break;
                }
                case 10U: {
                    if ((seen & (uint64_t{1} << 9U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_profile_id(decoded);
                    seen |= uint64_t{1} << 9U;
                    break;
                }
                case 11U: {
                    if ((seen & (uint64_t{1} << 10U)) != 0U) return false;
                    if (length != 2U) return false;
                    uint16_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_core_state(decoded);
                    seen |= uint64_t{1} << 10U;
                    break;
                }
                case 12U: {
                    if ((seen & (uint64_t{1} << 11U)) != 0U) return false;
                    if (length != 2U) return false;
                    uint16_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_fault_reason(decoded);
                    seen |= uint64_t{1} << 11U;
                    break;
                }
                case 13U: {
                    if ((seen & (uint64_t{1} << 12U)) != 0U) return false;
                    if (length != 1U) return false;
                    uint8_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_control_mode(decoded);
                    seen |= uint64_t{1} << 12U;
                    break;
                }
                case 14U: {
                    if ((seen & (uint64_t{1} << 13U)) != 0U) return false;
                    if (length != 1U) return false;
                    uint8_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_reference_kind(decoded);
                    seen |= uint64_t{1} << 13U;
                    break;
                }
                case 15U: {
                    if ((seen & (uint64_t{1} << 14U)) != 0U) return false;
                    if (length != 1U) return false;
                    uint8_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_drive_request(decoded);
                    seen |= uint64_t{1} << 14U;
                    break;
                }
                case 16U: {
                    if ((seen & (uint64_t{1} << 15U)) != 0U) return false;
                    if (length != 1U) return false;
                    uint8_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_drive_evidence(decoded);
                    seen |= uint64_t{1} << 15U;
                    break;
                }
                case 17U: {
                    if ((seen & (uint64_t{1} << 16U)) != 0U) return false;
                    if (length != 1U) return false;
                    uint8_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_configured_mask(decoded);
                    seen |= uint64_t{1} << 16U;
                    break;
                }
                case 18U: {
                    if ((seen & (uint64_t{1} << 17U)) != 0U) return false;
                    if (length != 1U) return false;
                    bool decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_real_motion_permitted(decoded);
                    seen |= uint64_t{1} << 17U;
                    break;
                }
                case 19U: {
                    if ((seen & (uint64_t{1} << 18U)) != 0U) return false;
                    if (length != 8U) return false;
                    uint64_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_permit_remaining_us(decoded);
                    seen |= uint64_t{1} << 18U;
                    break;
                }
                case 20U: {
                    if ((seen & (uint64_t{1} << 19U)) != 0U) return false;
                    if (length != 8U) return false;
                    uint64_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_capabilities(decoded);
                    seen |= uint64_t{1} << 19U;
                    break;
                }
                case 21U: {
                    if ((seen & (uint64_t{1} << 20U)) != 0U) return false;
                    if (length != 8U) return false;
                    uint64_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_operation_id(decoded);
                    seen |= uint64_t{1} << 20U;
                    break;
                }
                case 22U: {
                    if ((seen & (uint64_t{1} << 21U)) != 0U) return false;
                    if (length != 2U) return false;
                    uint16_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_operation_status(decoded);
                    seen |= uint64_t{1} << 21U;
                    break;
                }
                case 23U: {
                    if ((seen & (uint64_t{1} << 22U)) != 0U) return false;
                    if (length != 2U) return false;
                    uint16_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_operation_result_code(decoded);
                    seen |= uint64_t{1} << 22U;
                    break;
                }
                case 24U: {
                    if ((seen & (uint64_t{1} << 23U)) != 0U) return false;
                    if (length != 1U) return false;
                    bool decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_operation_result_valid(decoded);
                    seen |= uint64_t{1} << 23U;
                    break;
                }
                case 25U: {
                    if ((seen & (uint64_t{1} << 24U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_command_reject_count(decoded);
                    seen |= uint64_t{1} << 24U;
                    break;
                }
                case 26U: {
                    if ((seen & (uint64_t{1} << 25U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_can_error_count(decoded);
                    seen |= uint64_t{1} << 25U;
                    break;
                }
                case 27U: {
                    if ((seen & (uint64_t{1} << 26U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_deadline_miss_count(decoded);
                    seen |= uint64_t{1} << 26U;
                    break;
                }
                case 28U: {
                    if ((seen & (uint64_t{1} << 27U)) != 0U) return false;
                    if (length != 32U) return false;
                    for (size_t item = 0U; item < length / 1U; ++item)
                        if (!detail::get_scalar(payload + item * 1U, 1U,
                                                &scratch->contract_sha256_.data_[item])) return false;
                    seen |= uint64_t{1} << 27U;
                    break;
                }
                case 29U: {
                    if ((seen & (uint64_t{1} << 28U)) != 0U) return false;
                    if (length != 32U) return false;
                    for (size_t item = 0U; item < length / 1U; ++item)
                        if (!detail::get_scalar(payload + item * 1U, 1U,
                                                &scratch->model_sha256_.data_[item])) return false;
                    seen |= uint64_t{1} << 28U;
                    break;
                }
                case 30U: {
                    if ((seen & (uint64_t{1} << 29U)) != 0U) return false;
                    if (length != 32U) return false;
                    for (size_t item = 0U; item < length / 1U; ++item)
                        if (!detail::get_scalar(payload + item * 1U, 1U,
                                                &scratch->profile_sha256_.data_[item])) return false;
                    seen |= uint64_t{1} << 29U;
                    break;
                }
                case 31U: {
                    if ((seen & (uint64_t{1} << 30U)) != 0U) return false;
                    if (length != 32U) return false;
                    for (size_t item = 0U; item < length / 1U; ++item)
                        if (!detail::get_scalar(payload + item * 1U, 1U,
                                                &scratch->activation_profile_sha256_.data_[item])) return false;
                    seen |= uint64_t{1} << 30U;
                    break;
                }
                case 32U: {
                    if ((seen & (uint64_t{1} << 31U)) != 0U) return false;
                    if (length != 1U) return false;
                    bool decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_motion_supported(decoded);
                    seen |= uint64_t{1} << 31U;
                    break;
                }
                case 33U: {
                    if ((seen & (uint64_t{1} << 32U)) != 0U) return false;
                    if (length != 1U) return false;
                    bool decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_hold_ready(decoded);
                    seen |= uint64_t{1} << 32U;
                    break;
                }
                case 34U: {
                    if ((seen & (uint64_t{1} << 33U)) != 0U) return false;
                    if (length != 32U) return false;
                    for (size_t item = 0U; item < length / 1U; ++item)
                        if (!detail::get_scalar(payload + item * 1U, 1U,
                                                &scratch->firmware_sha256_.data_[item])) return false;
                    seen |= uint64_t{1} << 33U;
                    break;
                }
                case 35U: {
                    if ((seen & (uint64_t{1} << 34U)) != 0U) return false;
                    if (length != 16U) return false;
                    for (size_t item = 0U; item < length / 1U; ++item)
                        if (!detail::get_scalar(payload + item * 1U, 1U,
                                                &scratch->board_name_ascii_.data_[item])) return false;
                    seen |= uint64_t{1} << 34U;
                    break;
                }
                default: break;
            }
            offset += length;
        }
        if ((seen & 34359738367ULL) != 34359738367ULL) return false;
        *output = *scratch;
        return true;
    }
};

template <> struct Codec<dp::iface::DummyJointCommandV1> {
    using Value = dp::iface::DummyJointCommandV1;
    static constexpr size_t kMaxEncodedSize = 177U;
    static constexpr size_t kScratchSize = sizeof(Value);
    static constexpr size_t kAlignment = alignof(Value);
    static constexpr uint32_t kInterfaceId = 4132U;
    static bool encode(const Value& message, uint8_t* output, size_t capacity, size_t* used) {
        if (used == nullptr) return false;
        *used = 0U;
        if (output == nullptr) return false;
        size_t required_bytes = 0U;
        if (!detail::reserve(capacity, &required_bytes, 2U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 2U)) return false;
        if (!detail::reserve(capacity, &required_bytes, message.device_id().size() * 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 8U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 8U)) return false;
        if (!detail::reserve(capacity, &required_bytes, message.client_instance_id().size() * 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 8U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, message.position_rad().size() * 8U)) return false;
        if (required_bytes > kMaxEncodedSize) return false;
        size_t offset = 0U;
        {
            const auto value = message.schema_major();
            uint8_t* payload = detail::append(output, capacity, &offset, 1U, 2U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.schema_minor();
            uint8_t* payload = detail::append(output, capacity, &offset, 2U, 2U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.device_id();
            const size_t length = value.size() * 1U;
            uint8_t* payload = detail::append(output, capacity, &offset, 3U, length);
            if (payload == nullptr) return false;
            for (size_t index = 0U; index < value.size(); ++index)
                detail::put_scalar(payload + index * 1U, value.data()[index]);
        }
        {
            const auto value = message.boot_epoch();
            uint8_t* payload = detail::append(output, capacity, &offset, 4U, 8U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.session_id();
            uint8_t* payload = detail::append(output, capacity, &offset, 5U, 8U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.client_instance_id();
            const size_t length = value.size() * 1U;
            uint8_t* payload = detail::append(output, capacity, &offset, 6U, length);
            if (payload == nullptr) return false;
            for (size_t index = 0U; index < value.size(); ++index)
                detail::put_scalar(payload + index * 1U, value.data()[index]);
        }
        {
            const auto value = message.reference_epoch();
            uint8_t* payload = detail::append(output, capacity, &offset, 7U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.motion_epoch();
            uint8_t* payload = detail::append(output, capacity, &offset, 8U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.profile_id();
            uint8_t* payload = detail::append(output, capacity, &offset, 9U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.producer_seq();
            uint8_t* payload = detail::append(output, capacity, &offset, 10U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.freshness_ticket();
            uint8_t* payload = detail::append(output, capacity, &offset, 11U, 8U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.configured_mask();
            uint8_t* payload = detail::append(output, capacity, &offset, 12U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.position_rad();
            const size_t length = value.size() * 8U;
            uint8_t* payload = detail::append(output, capacity, &offset, 13U, length);
            if (payload == nullptr) return false;
            for (size_t index = 0U; index < value.size(); ++index)
                detail::put_scalar(payload + index * 8U, value.data()[index]);
        }
        *used = offset;
        return true;
    }
    static bool decode(const uint8_t* input, size_t size, Value* output, Value* scratch) {
        if (input == nullptr || output == nullptr || scratch == nullptr || output == scratch ||
            size > UINT16_MAX) return false;
        *scratch = Value{};
        uint64_t seen = 0U;
        size_t offset = 0U;
        while (offset < size) {
            if (size - offset < 4U) return false;
            const uint16_t tag = detail::get_u16(input + offset);
            const size_t length = detail::get_u16(input + offset + 2U);
            offset += 4U;
            if (tag == 0U || length > size - offset) return false;
            const uint8_t* payload = input + offset;
            switch (tag) {
                case 1U: {
                    if ((seen & (uint64_t{1} << 0U)) != 0U) return false;
                    if (length != 2U) return false;
                    uint16_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_schema_major(decoded);
                    seen |= uint64_t{1} << 0U;
                    break;
                }
                case 2U: {
                    if ((seen & (uint64_t{1} << 1U)) != 0U) return false;
                    if (length != 2U) return false;
                    uint16_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_schema_minor(decoded);
                    seen |= uint64_t{1} << 1U;
                    break;
                }
                case 3U: {
                    if ((seen & (uint64_t{1} << 2U)) != 0U) return false;
                    if (length != 16U) return false;
                    for (size_t item = 0U; item < length / 1U; ++item)
                        if (!detail::get_scalar(payload + item * 1U, 1U,
                                                &scratch->device_id_.data_[item])) return false;
                    seen |= uint64_t{1} << 2U;
                    break;
                }
                case 4U: {
                    if ((seen & (uint64_t{1} << 3U)) != 0U) return false;
                    if (length != 8U) return false;
                    uint64_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_boot_epoch(decoded);
                    seen |= uint64_t{1} << 3U;
                    break;
                }
                case 5U: {
                    if ((seen & (uint64_t{1} << 4U)) != 0U) return false;
                    if (length != 8U) return false;
                    uint64_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_session_id(decoded);
                    seen |= uint64_t{1} << 4U;
                    break;
                }
                case 6U: {
                    if ((seen & (uint64_t{1} << 5U)) != 0U) return false;
                    if (length != 16U) return false;
                    for (size_t item = 0U; item < length / 1U; ++item)
                        if (!detail::get_scalar(payload + item * 1U, 1U,
                                                &scratch->client_instance_id_.data_[item])) return false;
                    seen |= uint64_t{1} << 5U;
                    break;
                }
                case 7U: {
                    if ((seen & (uint64_t{1} << 6U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_reference_epoch(decoded);
                    seen |= uint64_t{1} << 6U;
                    break;
                }
                case 8U: {
                    if ((seen & (uint64_t{1} << 7U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_motion_epoch(decoded);
                    seen |= uint64_t{1} << 7U;
                    break;
                }
                case 9U: {
                    if ((seen & (uint64_t{1} << 8U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_profile_id(decoded);
                    seen |= uint64_t{1} << 8U;
                    break;
                }
                case 10U: {
                    if ((seen & (uint64_t{1} << 9U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_producer_seq(decoded);
                    seen |= uint64_t{1} << 9U;
                    break;
                }
                case 11U: {
                    if ((seen & (uint64_t{1} << 10U)) != 0U) return false;
                    if (length != 8U) return false;
                    uint64_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_freshness_ticket(decoded);
                    seen |= uint64_t{1} << 10U;
                    break;
                }
                case 12U: {
                    if ((seen & (uint64_t{1} << 11U)) != 0U) return false;
                    if (length != 1U) return false;
                    uint8_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_configured_mask(decoded);
                    seen |= uint64_t{1} << 11U;
                    break;
                }
                case 13U: {
                    if ((seen & (uint64_t{1} << 12U)) != 0U) return false;
                    if (length != 48U) return false;
                    for (size_t item = 0U; item < length / 8U; ++item)
                        if (!detail::get_scalar(payload + item * 8U, 8U,
                                                &scratch->position_rad_.data_[item])) return false;
                    seen |= uint64_t{1} << 12U;
                    break;
                }
                default: break;
            }
            offset += length;
        }
        if ((seen & 8191ULL) != 8191ULL) return false;
        *output = *scratch;
        return true;
    }
};

template <> struct Codec<dp::iface::DummyJointStateV1> {
    using Value = dp::iface::DummyJointStateV1;
    static constexpr size_t kMaxEncodedSize = 500U;
    static constexpr size_t kScratchSize = sizeof(Value);
    static constexpr size_t kAlignment = alignof(Value);
    static constexpr uint32_t kInterfaceId = 4133U;
    static bool encode(const Value& message, uint8_t* output, size_t capacity, size_t* used) {
        if (used == nullptr) return false;
        *used = 0U;
        if (output == nullptr) return false;
        size_t required_bytes = 0U;
        if (!detail::reserve(capacity, &required_bytes, 2U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 2U)) return false;
        if (!detail::reserve(capacity, &required_bytes, message.device_id().size() * 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 8U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 8U)) return false;
        if (!detail::reserve(capacity, &required_bytes, message.client_instance_id().size() * 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 8U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 8U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, message.feedback_seq().size() * 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, message.can_rx_age_us().size() * 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, message.position_rad().size() * 8U)) return false;
        if (!detail::reserve(capacity, &required_bytes, message.estimated_velocity_rad_s().size() * 8U)) return false;
        if (!detail::reserve(capacity, &required_bytes, message.hold_position_rad().size() * 8U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 2U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 2U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, message.raw_motor_position_rev().size() * 8U)) return false;
        if (required_bytes > kMaxEncodedSize) return false;
        size_t offset = 0U;
        {
            const auto value = message.schema_major();
            uint8_t* payload = detail::append(output, capacity, &offset, 1U, 2U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.schema_minor();
            uint8_t* payload = detail::append(output, capacity, &offset, 2U, 2U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.device_id();
            const size_t length = value.size() * 1U;
            uint8_t* payload = detail::append(output, capacity, &offset, 3U, length);
            if (payload == nullptr) return false;
            for (size_t index = 0U; index < value.size(); ++index)
                detail::put_scalar(payload + index * 1U, value.data()[index]);
        }
        {
            const auto value = message.boot_epoch();
            uint8_t* payload = detail::append(output, capacity, &offset, 4U, 8U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.session_id();
            uint8_t* payload = detail::append(output, capacity, &offset, 5U, 8U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.client_instance_id();
            const size_t length = value.size() * 1U;
            uint8_t* payload = detail::append(output, capacity, &offset, 6U, length);
            if (payload == nullptr) return false;
            for (size_t index = 0U; index < value.size(); ++index)
                detail::put_scalar(payload + index * 1U, value.data()[index]);
        }
        {
            const auto value = message.reference_epoch();
            uint8_t* payload = detail::append(output, capacity, &offset, 7U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.motion_epoch();
            uint8_t* payload = detail::append(output, capacity, &offset, 8U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.management_epoch();
            uint8_t* payload = detail::append(output, capacity, &offset, 9U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.profile_id();
            uint8_t* payload = detail::append(output, capacity, &offset, 10U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.state_seq();
            uint8_t* payload = detail::append(output, capacity, &offset, 11U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.produced_at_us();
            uint8_t* payload = detail::append(output, capacity, &offset, 12U, 8U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.freshness_ticket();
            uint8_t* payload = detail::append(output, capacity, &offset, 13U, 8U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.configured_mask();
            uint8_t* payload = detail::append(output, capacity, &offset, 14U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.position_valid_mask();
            uint8_t* payload = detail::append(output, capacity, &offset, 15U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.velocity_valid_mask();
            uint8_t* payload = detail::append(output, capacity, &offset, 16U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.planner_finish_mask();
            uint8_t* payload = detail::append(output, capacity, &offset, 17U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.hold_valid_mask();
            uint8_t* payload = detail::append(output, capacity, &offset, 18U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.accepted_seq_valid();
            uint8_t* payload = detail::append(output, capacity, &offset, 19U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.accepted_producer_seq();
            uint8_t* payload = detail::append(output, capacity, &offset, 20U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.applied_seq_valid();
            uint8_t* payload = detail::append(output, capacity, &offset, 21U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.applied_producer_seq();
            uint8_t* payload = detail::append(output, capacity, &offset, 22U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.tx_group_seq_valid();
            uint8_t* payload = detail::append(output, capacity, &offset, 23U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.tx_group_producer_seq();
            uint8_t* payload = detail::append(output, capacity, &offset, 24U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.submitted_axis_mask();
            uint8_t* payload = detail::append(output, capacity, &offset, 25U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.sent_axis_mask();
            uint8_t* payload = detail::append(output, capacity, &offset, 26U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.feedback_seq();
            const size_t length = value.size() * 4U;
            uint8_t* payload = detail::append(output, capacity, &offset, 27U, length);
            if (payload == nullptr) return false;
            for (size_t index = 0U; index < value.size(); ++index)
                detail::put_scalar(payload + index * 4U, value.data()[index]);
        }
        {
            const auto value = message.can_rx_age_us();
            const size_t length = value.size() * 4U;
            uint8_t* payload = detail::append(output, capacity, &offset, 28U, length);
            if (payload == nullptr) return false;
            for (size_t index = 0U; index < value.size(); ++index)
                detail::put_scalar(payload + index * 4U, value.data()[index]);
        }
        {
            const auto value = message.position_rad();
            const size_t length = value.size() * 8U;
            uint8_t* payload = detail::append(output, capacity, &offset, 29U, length);
            if (payload == nullptr) return false;
            for (size_t index = 0U; index < value.size(); ++index)
                detail::put_scalar(payload + index * 8U, value.data()[index]);
        }
        {
            const auto value = message.estimated_velocity_rad_s();
            const size_t length = value.size() * 8U;
            uint8_t* payload = detail::append(output, capacity, &offset, 30U, length);
            if (payload == nullptr) return false;
            for (size_t index = 0U; index < value.size(); ++index)
                detail::put_scalar(payload + index * 8U, value.data()[index]);
        }
        {
            const auto value = message.hold_position_rad();
            const size_t length = value.size() * 8U;
            uint8_t* payload = detail::append(output, capacity, &offset, 31U, length);
            if (payload == nullptr) return false;
            for (size_t index = 0U; index < value.size(); ++index)
                detail::put_scalar(payload + index * 8U, value.data()[index]);
        }
        {
            const auto value = message.core_state();
            uint8_t* payload = detail::append(output, capacity, &offset, 32U, 2U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.fault_reason();
            uint8_t* payload = detail::append(output, capacity, &offset, 33U, 2U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.hold_ready();
            uint8_t* payload = detail::append(output, capacity, &offset, 34U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.raw_position_valid_mask();
            uint8_t* payload = detail::append(output, capacity, &offset, 35U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.raw_motor_position_rev();
            const size_t length = value.size() * 8U;
            uint8_t* payload = detail::append(output, capacity, &offset, 36U, length);
            if (payload == nullptr) return false;
            for (size_t index = 0U; index < value.size(); ++index)
                detail::put_scalar(payload + index * 8U, value.data()[index]);
        }
        *used = offset;
        return true;
    }
    static bool decode(const uint8_t* input, size_t size, Value* output, Value* scratch) {
        if (input == nullptr || output == nullptr || scratch == nullptr || output == scratch ||
            size > UINT16_MAX) return false;
        *scratch = Value{};
        uint64_t seen = 0U;
        size_t offset = 0U;
        while (offset < size) {
            if (size - offset < 4U) return false;
            const uint16_t tag = detail::get_u16(input + offset);
            const size_t length = detail::get_u16(input + offset + 2U);
            offset += 4U;
            if (tag == 0U || length > size - offset) return false;
            const uint8_t* payload = input + offset;
            switch (tag) {
                case 1U: {
                    if ((seen & (uint64_t{1} << 0U)) != 0U) return false;
                    if (length != 2U) return false;
                    uint16_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_schema_major(decoded);
                    seen |= uint64_t{1} << 0U;
                    break;
                }
                case 2U: {
                    if ((seen & (uint64_t{1} << 1U)) != 0U) return false;
                    if (length != 2U) return false;
                    uint16_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_schema_minor(decoded);
                    seen |= uint64_t{1} << 1U;
                    break;
                }
                case 3U: {
                    if ((seen & (uint64_t{1} << 2U)) != 0U) return false;
                    if (length != 16U) return false;
                    for (size_t item = 0U; item < length / 1U; ++item)
                        if (!detail::get_scalar(payload + item * 1U, 1U,
                                                &scratch->device_id_.data_[item])) return false;
                    seen |= uint64_t{1} << 2U;
                    break;
                }
                case 4U: {
                    if ((seen & (uint64_t{1} << 3U)) != 0U) return false;
                    if (length != 8U) return false;
                    uint64_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_boot_epoch(decoded);
                    seen |= uint64_t{1} << 3U;
                    break;
                }
                case 5U: {
                    if ((seen & (uint64_t{1} << 4U)) != 0U) return false;
                    if (length != 8U) return false;
                    uint64_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_session_id(decoded);
                    seen |= uint64_t{1} << 4U;
                    break;
                }
                case 6U: {
                    if ((seen & (uint64_t{1} << 5U)) != 0U) return false;
                    if (length != 16U) return false;
                    for (size_t item = 0U; item < length / 1U; ++item)
                        if (!detail::get_scalar(payload + item * 1U, 1U,
                                                &scratch->client_instance_id_.data_[item])) return false;
                    seen |= uint64_t{1} << 5U;
                    break;
                }
                case 7U: {
                    if ((seen & (uint64_t{1} << 6U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_reference_epoch(decoded);
                    seen |= uint64_t{1} << 6U;
                    break;
                }
                case 8U: {
                    if ((seen & (uint64_t{1} << 7U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_motion_epoch(decoded);
                    seen |= uint64_t{1} << 7U;
                    break;
                }
                case 9U: {
                    if ((seen & (uint64_t{1} << 8U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_management_epoch(decoded);
                    seen |= uint64_t{1} << 8U;
                    break;
                }
                case 10U: {
                    if ((seen & (uint64_t{1} << 9U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_profile_id(decoded);
                    seen |= uint64_t{1} << 9U;
                    break;
                }
                case 11U: {
                    if ((seen & (uint64_t{1} << 10U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_state_seq(decoded);
                    seen |= uint64_t{1} << 10U;
                    break;
                }
                case 12U: {
                    if ((seen & (uint64_t{1} << 11U)) != 0U) return false;
                    if (length != 8U) return false;
                    uint64_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_produced_at_us(decoded);
                    seen |= uint64_t{1} << 11U;
                    break;
                }
                case 13U: {
                    if ((seen & (uint64_t{1} << 12U)) != 0U) return false;
                    if (length != 8U) return false;
                    uint64_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_freshness_ticket(decoded);
                    seen |= uint64_t{1} << 12U;
                    break;
                }
                case 14U: {
                    if ((seen & (uint64_t{1} << 13U)) != 0U) return false;
                    if (length != 1U) return false;
                    uint8_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_configured_mask(decoded);
                    seen |= uint64_t{1} << 13U;
                    break;
                }
                case 15U: {
                    if ((seen & (uint64_t{1} << 14U)) != 0U) return false;
                    if (length != 1U) return false;
                    uint8_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_position_valid_mask(decoded);
                    seen |= uint64_t{1} << 14U;
                    break;
                }
                case 16U: {
                    if ((seen & (uint64_t{1} << 15U)) != 0U) return false;
                    if (length != 1U) return false;
                    uint8_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_velocity_valid_mask(decoded);
                    seen |= uint64_t{1} << 15U;
                    break;
                }
                case 17U: {
                    if ((seen & (uint64_t{1} << 16U)) != 0U) return false;
                    if (length != 1U) return false;
                    uint8_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_planner_finish_mask(decoded);
                    seen |= uint64_t{1} << 16U;
                    break;
                }
                case 18U: {
                    if ((seen & (uint64_t{1} << 17U)) != 0U) return false;
                    if (length != 1U) return false;
                    uint8_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_hold_valid_mask(decoded);
                    seen |= uint64_t{1} << 17U;
                    break;
                }
                case 19U: {
                    if ((seen & (uint64_t{1} << 18U)) != 0U) return false;
                    if (length != 1U) return false;
                    bool decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_accepted_seq_valid(decoded);
                    seen |= uint64_t{1} << 18U;
                    break;
                }
                case 20U: {
                    if ((seen & (uint64_t{1} << 19U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_accepted_producer_seq(decoded);
                    seen |= uint64_t{1} << 19U;
                    break;
                }
                case 21U: {
                    if ((seen & (uint64_t{1} << 20U)) != 0U) return false;
                    if (length != 1U) return false;
                    bool decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_applied_seq_valid(decoded);
                    seen |= uint64_t{1} << 20U;
                    break;
                }
                case 22U: {
                    if ((seen & (uint64_t{1} << 21U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_applied_producer_seq(decoded);
                    seen |= uint64_t{1} << 21U;
                    break;
                }
                case 23U: {
                    if ((seen & (uint64_t{1} << 22U)) != 0U) return false;
                    if (length != 1U) return false;
                    bool decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_tx_group_seq_valid(decoded);
                    seen |= uint64_t{1} << 22U;
                    break;
                }
                case 24U: {
                    if ((seen & (uint64_t{1} << 23U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_tx_group_producer_seq(decoded);
                    seen |= uint64_t{1} << 23U;
                    break;
                }
                case 25U: {
                    if ((seen & (uint64_t{1} << 24U)) != 0U) return false;
                    if (length != 1U) return false;
                    uint8_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_submitted_axis_mask(decoded);
                    seen |= uint64_t{1} << 24U;
                    break;
                }
                case 26U: {
                    if ((seen & (uint64_t{1} << 25U)) != 0U) return false;
                    if (length != 1U) return false;
                    uint8_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_sent_axis_mask(decoded);
                    seen |= uint64_t{1} << 25U;
                    break;
                }
                case 27U: {
                    if ((seen & (uint64_t{1} << 26U)) != 0U) return false;
                    if (length != 24U) return false;
                    for (size_t item = 0U; item < length / 4U; ++item)
                        if (!detail::get_scalar(payload + item * 4U, 4U,
                                                &scratch->feedback_seq_.data_[item])) return false;
                    seen |= uint64_t{1} << 26U;
                    break;
                }
                case 28U: {
                    if ((seen & (uint64_t{1} << 27U)) != 0U) return false;
                    if (length != 24U) return false;
                    for (size_t item = 0U; item < length / 4U; ++item)
                        if (!detail::get_scalar(payload + item * 4U, 4U,
                                                &scratch->can_rx_age_us_.data_[item])) return false;
                    seen |= uint64_t{1} << 27U;
                    break;
                }
                case 29U: {
                    if ((seen & (uint64_t{1} << 28U)) != 0U) return false;
                    if (length != 48U) return false;
                    for (size_t item = 0U; item < length / 8U; ++item)
                        if (!detail::get_scalar(payload + item * 8U, 8U,
                                                &scratch->position_rad_.data_[item])) return false;
                    seen |= uint64_t{1} << 28U;
                    break;
                }
                case 30U: {
                    if ((seen & (uint64_t{1} << 29U)) != 0U) return false;
                    if (length != 48U) return false;
                    for (size_t item = 0U; item < length / 8U; ++item)
                        if (!detail::get_scalar(payload + item * 8U, 8U,
                                                &scratch->estimated_velocity_rad_s_.data_[item])) return false;
                    seen |= uint64_t{1} << 29U;
                    break;
                }
                case 31U: {
                    if ((seen & (uint64_t{1} << 30U)) != 0U) return false;
                    if (length != 48U) return false;
                    for (size_t item = 0U; item < length / 8U; ++item)
                        if (!detail::get_scalar(payload + item * 8U, 8U,
                                                &scratch->hold_position_rad_.data_[item])) return false;
                    seen |= uint64_t{1} << 30U;
                    break;
                }
                case 32U: {
                    if ((seen & (uint64_t{1} << 31U)) != 0U) return false;
                    if (length != 2U) return false;
                    uint16_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_core_state(decoded);
                    seen |= uint64_t{1} << 31U;
                    break;
                }
                case 33U: {
                    if ((seen & (uint64_t{1} << 32U)) != 0U) return false;
                    if (length != 2U) return false;
                    uint16_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_fault_reason(decoded);
                    seen |= uint64_t{1} << 32U;
                    break;
                }
                case 34U: {
                    if ((seen & (uint64_t{1} << 33U)) != 0U) return false;
                    if (length != 1U) return false;
                    bool decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_hold_ready(decoded);
                    seen |= uint64_t{1} << 33U;
                    break;
                }
                case 35U: {
                    if ((seen & (uint64_t{1} << 34U)) != 0U) return false;
                    if (length != 1U) return false;
                    uint8_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_raw_position_valid_mask(decoded);
                    seen |= uint64_t{1} << 34U;
                    break;
                }
                case 36U: {
                    if ((seen & (uint64_t{1} << 35U)) != 0U) return false;
                    if (length != 48U) return false;
                    for (size_t item = 0U; item < length / 8U; ++item)
                        if (!detail::get_scalar(payload + item * 8U, 8U,
                                                &scratch->raw_motor_position_rev_.data_[item])) return false;
                    seen |= uint64_t{1} << 35U;
                    break;
                }
                default: break;
            }
            offset += length;
        }
        if ((seen & 68719476735ULL) != 68719476735ULL) return false;
        *output = *scratch;
        return true;
    }
};

template <> struct Codec<dp::iface::HeartbeatV1> {
    using Value = dp::iface::HeartbeatV1;
    static constexpr size_t kMaxEncodedSize = 16U;
    static constexpr size_t kScratchSize = sizeof(Value);
    static constexpr size_t kAlignment = alignof(Value);
    static constexpr uint32_t kInterfaceId = 61444U;
    static bool encode(const Value& message, uint8_t* output, size_t capacity, size_t* used) {
        if (used == nullptr) return false;
        *used = 0U;
        if (output == nullptr) return false;
        size_t required_bytes = 0U;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (required_bytes > kMaxEncodedSize) return false;
        size_t offset = 0U;
        {
            const auto value = message.uptime_ms();
            uint8_t* payload = detail::append(output, capacity, &offset, 1U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.health_flags();
            uint8_t* payload = detail::append(output, capacity, &offset, 2U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        *used = offset;
        return true;
    }
    static bool decode(const uint8_t* input, size_t size, Value* output, Value* scratch) {
        if (input == nullptr || output == nullptr || scratch == nullptr || output == scratch ||
            size > UINT16_MAX) return false;
        *scratch = Value{};
        uint64_t seen = 0U;
        size_t offset = 0U;
        while (offset < size) {
            if (size - offset < 4U) return false;
            const uint16_t tag = detail::get_u16(input + offset);
            const size_t length = detail::get_u16(input + offset + 2U);
            offset += 4U;
            if (tag == 0U || length > size - offset) return false;
            const uint8_t* payload = input + offset;
            switch (tag) {
                case 1U: {
                    if ((seen & (uint64_t{1} << 0U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_uptime_ms(decoded);
                    seen |= uint64_t{1} << 0U;
                    break;
                }
                case 2U: {
                    if ((seen & (uint64_t{1} << 1U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_health_flags(decoded);
                    seen |= uint64_t{1} << 1U;
                    break;
                }
                default: break;
            }
            offset += length;
        }
        if ((seen & 3ULL) != 3ULL) return false;
        *output = *scratch;
        return true;
    }
};

template <> struct Codec<dp::iface::JointCommandFrameV1> {
    using Value = dp::iface::JointCommandFrameV1;
    static constexpr size_t kMaxEncodedSize = 12U;
    static constexpr size_t kScratchSize = sizeof(Value);
    static constexpr size_t kAlignment = alignof(Value);
    static constexpr uint32_t kInterfaceId = 4098U;
    static bool encode(const Value& message, uint8_t* output, size_t capacity, size_t* used) {
        if (used == nullptr) return false;
        *used = 0U;
        if (output == nullptr) return false;
        size_t required_bytes = 0U;
        if (!detail::reserve(capacity, &required_bytes, 8U)) return false;
        if (required_bytes > kMaxEncodedSize) return false;
        size_t offset = 0U;
        {
            const auto value = message.motor_position();
            uint8_t* payload = detail::append(output, capacity, &offset, 1U, 8U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        *used = offset;
        return true;
    }
    static bool decode(const uint8_t* input, size_t size, Value* output, Value* scratch) {
        if (input == nullptr || output == nullptr || scratch == nullptr || output == scratch ||
            size > UINT16_MAX) return false;
        *scratch = Value{};
        uint64_t seen = 0U;
        size_t offset = 0U;
        while (offset < size) {
            if (size - offset < 4U) return false;
            const uint16_t tag = detail::get_u16(input + offset);
            const size_t length = detail::get_u16(input + offset + 2U);
            offset += 4U;
            if (tag == 0U || length > size - offset) return false;
            const uint8_t* payload = input + offset;
            switch (tag) {
                case 1U: {
                    if ((seen & (uint64_t{1} << 0U)) != 0U) return false;
                    if (length != 8U) return false;
                    double decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_motor_position(decoded);
                    seen |= uint64_t{1} << 0U;
                    break;
                }
                default: break;
            }
            offset += length;
        }
        if ((seen & 1ULL) != 1ULL) return false;
        *output = *scratch;
        return true;
    }
};

template <> struct Codec<dp::iface::JointStateFrameV1> {
    using Value = dp::iface::JointStateFrameV1;
    static constexpr size_t kMaxEncodedSize = 24U;
    static constexpr size_t kScratchSize = sizeof(Value);
    static constexpr size_t kAlignment = alignof(Value);
    static constexpr uint32_t kInterfaceId = 4097U;
    static bool encode(const Value& message, uint8_t* output, size_t capacity, size_t* used) {
        if (used == nullptr) return false;
        *used = 0U;
        if (output == nullptr) return false;
        size_t required_bytes = 0U;
        if (!detail::reserve(capacity, &required_bytes, 8U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 8U)) return false;
        if (required_bytes > kMaxEncodedSize) return false;
        size_t offset = 0U;
        {
            const auto value = message.motor_position();
            uint8_t* payload = detail::append(output, capacity, &offset, 1U, 8U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.pendulum_position();
            uint8_t* payload = detail::append(output, capacity, &offset, 2U, 8U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        *used = offset;
        return true;
    }
    static bool decode(const uint8_t* input, size_t size, Value* output, Value* scratch) {
        if (input == nullptr || output == nullptr || scratch == nullptr || output == scratch ||
            size > UINT16_MAX) return false;
        *scratch = Value{};
        uint64_t seen = 0U;
        size_t offset = 0U;
        while (offset < size) {
            if (size - offset < 4U) return false;
            const uint16_t tag = detail::get_u16(input + offset);
            const size_t length = detail::get_u16(input + offset + 2U);
            offset += 4U;
            if (tag == 0U || length > size - offset) return false;
            const uint8_t* payload = input + offset;
            switch (tag) {
                case 1U: {
                    if ((seen & (uint64_t{1} << 0U)) != 0U) return false;
                    if (length != 8U) return false;
                    double decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_motor_position(decoded);
                    seen |= uint64_t{1} << 0U;
                    break;
                }
                case 2U: {
                    if ((seen & (uint64_t{1} << 1U)) != 0U) return false;
                    if (length != 8U) return false;
                    double decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_pendulum_position(decoded);
                    seen |= uint64_t{1} << 1U;
                    break;
                }
                default: break;
            }
            offset += length;
        }
        if ((seen & 3ULL) != 3ULL) return false;
        *output = *scratch;
        return true;
    }
};

template <> struct Codec<dp::iface::KeyRotateV1> {
    using Value = dp::iface::KeyRotateV1;
    static constexpr size_t kMaxEncodedSize = 37U;
    static constexpr size_t kScratchSize = sizeof(Value);
    static constexpr size_t kAlignment = alignof(Value);
    static constexpr uint32_t kInterfaceId = 61445U;
    static bool encode(const Value& message, uint8_t* output, size_t capacity, size_t* used) {
        if (used == nullptr) return false;
        *used = 0U;
        if (output == nullptr) return false;
        size_t required_bytes = 0U;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 2U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (required_bytes > kMaxEncodedSize) return false;
        size_t offset = 0U;
        {
            const auto value = message.key_epoch();
            uint8_t* payload = detail::append(output, capacity, &offset, 1U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.current_key_id();
            uint8_t* payload = detail::append(output, capacity, &offset, 2U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.next_key_id();
            uint8_t* payload = detail::append(output, capacity, &offset, 3U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.phase();
            uint8_t* payload = detail::append(output, capacity, &offset, 4U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.key_slot_id();
            uint8_t* payload = detail::append(output, capacity, &offset, 5U, 2U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.activate_after_ms();
            uint8_t* payload = detail::append(output, capacity, &offset, 6U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        *used = offset;
        return true;
    }
    static bool decode(const uint8_t* input, size_t size, Value* output, Value* scratch) {
        if (input == nullptr || output == nullptr || scratch == nullptr || output == scratch ||
            size > UINT16_MAX) return false;
        *scratch = Value{};
        uint64_t seen = 0U;
        size_t offset = 0U;
        while (offset < size) {
            if (size - offset < 4U) return false;
            const uint16_t tag = detail::get_u16(input + offset);
            const size_t length = detail::get_u16(input + offset + 2U);
            offset += 4U;
            if (tag == 0U || length > size - offset) return false;
            const uint8_t* payload = input + offset;
            switch (tag) {
                case 1U: {
                    if ((seen & (uint64_t{1} << 0U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_key_epoch(decoded);
                    seen |= uint64_t{1} << 0U;
                    break;
                }
                case 2U: {
                    if ((seen & (uint64_t{1} << 1U)) != 0U) return false;
                    if (length != 1U) return false;
                    uint8_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_current_key_id(decoded);
                    seen |= uint64_t{1} << 1U;
                    break;
                }
                case 3U: {
                    if ((seen & (uint64_t{1} << 2U)) != 0U) return false;
                    if (length != 1U) return false;
                    uint8_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_next_key_id(decoded);
                    seen |= uint64_t{1} << 2U;
                    break;
                }
                case 4U: {
                    if ((seen & (uint64_t{1} << 3U)) != 0U) return false;
                    if (length != 1U) return false;
                    uint8_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_phase(decoded);
                    seen |= uint64_t{1} << 3U;
                    break;
                }
                case 5U: {
                    if ((seen & (uint64_t{1} << 4U)) != 0U) return false;
                    if (length != 2U) return false;
                    uint16_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_key_slot_id(decoded);
                    seen |= uint64_t{1} << 4U;
                    break;
                }
                case 6U: {
                    if ((seen & (uint64_t{1} << 5U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_activate_after_ms(decoded);
                    seen |= uint64_t{1} << 5U;
                    break;
                }
                default: break;
            }
            offset += length;
        }
        if ((seen & 63ULL) != 63ULL) return false;
        *output = *scratch;
        return true;
    }
};

template <> struct Codec<dp::iface::LinkHelloV1> {
    using Value = dp::iface::LinkHelloV1;
    static constexpr size_t kMaxEncodedSize = 77U;
    static constexpr size_t kScratchSize = sizeof(Value);
    static constexpr size_t kAlignment = alignof(Value);
    static constexpr uint32_t kInterfaceId = 61441U;
    static bool encode(const Value& message, uint8_t* output, size_t capacity, size_t* used) {
        if (used == nullptr) return false;
        *used = 0U;
        if (output == nullptr) return false;
        size_t required_bytes = 0U;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, message.device_id().size() * 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, message.catalog_fingerprint().size() * 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 2U)) return false;
        if (required_bytes > kMaxEncodedSize) return false;
        size_t offset = 0U;
        {
            const auto value = message.protocol_major();
            uint8_t* payload = detail::append(output, capacity, &offset, 1U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.protocol_minor();
            uint8_t* payload = detail::append(output, capacity, &offset, 2U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.product_id();
            uint8_t* payload = detail::append(output, capacity, &offset, 3U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.boot_counter();
            uint8_t* payload = detail::append(output, capacity, &offset, 4U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.device_id();
            const size_t length = value.size() * 1U;
            uint8_t* payload = detail::append(output, capacity, &offset, 5U, length);
            if (payload == nullptr) return false;
            for (size_t index = 0U; index < value.size(); ++index)
                detail::put_scalar(payload + index * 1U, value.data()[index]);
        }
        {
            const auto value = message.catalog_fingerprint();
            const size_t length = value.size() * 1U;
            uint8_t* payload = detail::append(output, capacity, &offset, 6U, length);
            if (payload == nullptr) return false;
            for (size_t index = 0U; index < value.size(); ++index)
                detail::put_scalar(payload + index * 1U, value.data()[index]);
        }
        {
            const auto value = message.security_profile();
            uint8_t* payload = detail::append(output, capacity, &offset, 7U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.max_payload_size();
            uint8_t* payload = detail::append(output, capacity, &offset, 8U, 2U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        *used = offset;
        return true;
    }
    static bool decode(const uint8_t* input, size_t size, Value* output, Value* scratch) {
        if (input == nullptr || output == nullptr || scratch == nullptr || output == scratch ||
            size > UINT16_MAX) return false;
        *scratch = Value{};
        uint64_t seen = 0U;
        size_t offset = 0U;
        while (offset < size) {
            if (size - offset < 4U) return false;
            const uint16_t tag = detail::get_u16(input + offset);
            const size_t length = detail::get_u16(input + offset + 2U);
            offset += 4U;
            if (tag == 0U || length > size - offset) return false;
            const uint8_t* payload = input + offset;
            switch (tag) {
                case 1U: {
                    if ((seen & (uint64_t{1} << 0U)) != 0U) return false;
                    if (length != 1U) return false;
                    uint8_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_protocol_major(decoded);
                    seen |= uint64_t{1} << 0U;
                    break;
                }
                case 2U: {
                    if ((seen & (uint64_t{1} << 1U)) != 0U) return false;
                    if (length != 1U) return false;
                    uint8_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_protocol_minor(decoded);
                    seen |= uint64_t{1} << 1U;
                    break;
                }
                case 3U: {
                    if ((seen & (uint64_t{1} << 2U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_product_id(decoded);
                    seen |= uint64_t{1} << 2U;
                    break;
                }
                case 4U: {
                    if ((seen & (uint64_t{1} << 3U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_boot_counter(decoded);
                    seen |= uint64_t{1} << 3U;
                    break;
                }
                case 5U: {
                    if ((seen & (uint64_t{1} << 4U)) != 0U) return false;
                    if (length != 16U) return false;
                    for (size_t item = 0U; item < length / 1U; ++item)
                        if (!detail::get_scalar(payload + item * 1U, 1U,
                                                &scratch->device_id_.data_[item])) return false;
                    seen |= uint64_t{1} << 4U;
                    break;
                }
                case 6U: {
                    if ((seen & (uint64_t{1} << 5U)) != 0U) return false;
                    if (length != 16U) return false;
                    for (size_t item = 0U; item < length / 1U; ++item)
                        if (!detail::get_scalar(payload + item * 1U, 1U,
                                                &scratch->catalog_fingerprint_.data_[item])) return false;
                    seen |= uint64_t{1} << 5U;
                    break;
                }
                case 7U: {
                    if ((seen & (uint64_t{1} << 6U)) != 0U) return false;
                    if (length != 1U) return false;
                    uint8_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_security_profile(decoded);
                    seen |= uint64_t{1} << 6U;
                    break;
                }
                case 8U: {
                    if ((seen & (uint64_t{1} << 7U)) != 0U) return false;
                    if (length != 2U) return false;
                    uint16_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_max_payload_size(decoded);
                    seen |= uint64_t{1} << 7U;
                    break;
                }
                default: break;
            }
            offset += length;
        }
        if ((seen & 255ULL) != 255ULL) return false;
        *output = *scratch;
        return true;
    }
};

template <> struct Codec<dp::iface::LinkResetV1> {
    using Value = dp::iface::LinkResetV1;
    static constexpr size_t kMaxEncodedSize = 18U;
    static constexpr size_t kScratchSize = sizeof(Value);
    static constexpr size_t kAlignment = alignof(Value);
    static constexpr uint32_t kInterfaceId = 61443U;
    static bool encode(const Value& message, uint8_t* output, size_t capacity, size_t* used) {
        if (used == nullptr) return false;
        *used = 0U;
        if (output == nullptr) return false;
        size_t required_bytes = 0U;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (required_bytes > kMaxEncodedSize) return false;
        size_t offset = 0U;
        {
            const auto value = message.reset_generation();
            uint8_t* payload = detail::append(output, capacity, &offset, 1U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.reason();
            uint8_t* payload = detail::append(output, capacity, &offset, 2U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.mode();
            uint8_t* payload = detail::append(output, capacity, &offset, 3U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        *used = offset;
        return true;
    }
    static bool decode(const uint8_t* input, size_t size, Value* output, Value* scratch) {
        if (input == nullptr || output == nullptr || scratch == nullptr || output == scratch ||
            size > UINT16_MAX) return false;
        *scratch = Value{};
        uint64_t seen = 0U;
        size_t offset = 0U;
        while (offset < size) {
            if (size - offset < 4U) return false;
            const uint16_t tag = detail::get_u16(input + offset);
            const size_t length = detail::get_u16(input + offset + 2U);
            offset += 4U;
            if (tag == 0U || length > size - offset) return false;
            const uint8_t* payload = input + offset;
            switch (tag) {
                case 1U: {
                    if ((seen & (uint64_t{1} << 0U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_reset_generation(decoded);
                    seen |= uint64_t{1} << 0U;
                    break;
                }
                case 2U: {
                    if ((seen & (uint64_t{1} << 1U)) != 0U) return false;
                    if (length != 1U) return false;
                    uint8_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_reason(decoded);
                    seen |= uint64_t{1} << 1U;
                    break;
                }
                case 3U: {
                    if ((seen & (uint64_t{1} << 2U)) != 0U) return false;
                    if (length != 1U) return false;
                    uint8_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_mode(decoded);
                    seen |= uint64_t{1} << 2U;
                    break;
                }
                default: break;
            }
            offset += length;
        }
        if ((seen & 7ULL) != 7ULL) return false;
        *output = *scratch;
        return true;
    }
};

template <> struct Codec<dp::iface::LogRecordTinyV1> {
    using Value = dp::iface::LogRecordTinyV1;
    static constexpr size_t kMaxEncodedSize = 87U;
    static constexpr size_t kScratchSize = sizeof(Value);
    static constexpr size_t kAlignment = alignof(Value);
    static constexpr uint32_t kInterfaceId = 4130U;
    static bool encode(const Value& message, uint8_t* output, size_t capacity, size_t* used) {
        if (used == nullptr) return false;
        *used = 0U;
        if (output == nullptr) return false;
        size_t required_bytes = 0U;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 2U)) return false;
        if (!detail::reserve(capacity, &required_bytes, message.payload().size() * 1U)) return false;
        if (required_bytes > kMaxEncodedSize) return false;
        size_t offset = 0U;
        {
            const auto value = message.timestamp_ms();
            uint8_t* payload = detail::append(output, capacity, &offset, 1U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.level();
            uint8_t* payload = detail::append(output, capacity, &offset, 2U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.tag_id();
            uint8_t* payload = detail::append(output, capacity, &offset, 3U, 2U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.payload();
            const size_t length = value.size() * 1U;
            uint8_t* payload = detail::append(output, capacity, &offset, 4U, length);
            if (payload == nullptr) return false;
            for (size_t index = 0U; index < value.size(); ++index)
                detail::put_scalar(payload + index * 1U, value.data()[index]);
        }
        *used = offset;
        return true;
    }
    static bool decode(const uint8_t* input, size_t size, Value* output, Value* scratch) {
        if (input == nullptr || output == nullptr || scratch == nullptr || output == scratch ||
            size > UINT16_MAX) return false;
        *scratch = Value{};
        uint64_t seen = 0U;
        size_t offset = 0U;
        while (offset < size) {
            if (size - offset < 4U) return false;
            const uint16_t tag = detail::get_u16(input + offset);
            const size_t length = detail::get_u16(input + offset + 2U);
            offset += 4U;
            if (tag == 0U || length > size - offset) return false;
            const uint8_t* payload = input + offset;
            switch (tag) {
                case 1U: {
                    if ((seen & (uint64_t{1} << 0U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_timestamp_ms(decoded);
                    seen |= uint64_t{1} << 0U;
                    break;
                }
                case 2U: {
                    if ((seen & (uint64_t{1} << 1U)) != 0U) return false;
                    if (length != 1U) return false;
                    uint8_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_level(decoded);
                    seen |= uint64_t{1} << 1U;
                    break;
                }
                case 3U: {
                    if ((seen & (uint64_t{1} << 2U)) != 0U) return false;
                    if (length != 2U) return false;
                    uint16_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_tag_id(decoded);
                    seen |= uint64_t{1} << 2U;
                    break;
                }
                case 4U: {
                    if ((seen & (uint64_t{1} << 3U)) != 0U) return false;
                    if (length > 64U || length % 1U != 0U) return false;
                    for (size_t item = 0U; item < length / 1U; ++item)
                        if (!detail::get_scalar(payload + item * 1U, 1U,
                                                &scratch->payload_.data_[item])) return false;
                    scratch->payload_.size_ = length / 1U;
                    seen |= uint64_t{1} << 3U;
                    break;
                }
                default: break;
            }
            offset += length;
        }
        if ((seen & 15ULL) != 15ULL) return false;
        *output = *scratch;
        return true;
    }
};

template <> struct Codec<dp::iface::LogRecordV1> {
    using Value = dp::iface::LogRecordV1;
    static constexpr size_t kMaxEncodedSize = 503U;
    static constexpr size_t kScratchSize = sizeof(Value);
    static constexpr size_t kAlignment = alignof(Value);
    static constexpr uint32_t kInterfaceId = 4102U;
    static bool encode(const Value& message, uint8_t* output, size_t capacity, size_t* used) {
        if (used == nullptr) return false;
        *used = 0U;
        if (output == nullptr) return false;
        size_t required_bytes = 0U;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 2U)) return false;
        if (!detail::reserve(capacity, &required_bytes, message.payload().size() * 1U)) return false;
        if (required_bytes > kMaxEncodedSize) return false;
        size_t offset = 0U;
        {
            const auto value = message.timestamp_ms();
            uint8_t* payload = detail::append(output, capacity, &offset, 1U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.level();
            uint8_t* payload = detail::append(output, capacity, &offset, 2U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.tag_id();
            uint8_t* payload = detail::append(output, capacity, &offset, 3U, 2U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.payload();
            const size_t length = value.size() * 1U;
            uint8_t* payload = detail::append(output, capacity, &offset, 4U, length);
            if (payload == nullptr) return false;
            for (size_t index = 0U; index < value.size(); ++index)
                detail::put_scalar(payload + index * 1U, value.data()[index]);
        }
        *used = offset;
        return true;
    }
    static bool decode(const uint8_t* input, size_t size, Value* output, Value* scratch) {
        if (input == nullptr || output == nullptr || scratch == nullptr || output == scratch ||
            size > UINT16_MAX) return false;
        *scratch = Value{};
        uint64_t seen = 0U;
        size_t offset = 0U;
        while (offset < size) {
            if (size - offset < 4U) return false;
            const uint16_t tag = detail::get_u16(input + offset);
            const size_t length = detail::get_u16(input + offset + 2U);
            offset += 4U;
            if (tag == 0U || length > size - offset) return false;
            const uint8_t* payload = input + offset;
            switch (tag) {
                case 1U: {
                    if ((seen & (uint64_t{1} << 0U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_timestamp_ms(decoded);
                    seen |= uint64_t{1} << 0U;
                    break;
                }
                case 2U: {
                    if ((seen & (uint64_t{1} << 1U)) != 0U) return false;
                    if (length != 1U) return false;
                    uint8_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_level(decoded);
                    seen |= uint64_t{1} << 1U;
                    break;
                }
                case 3U: {
                    if ((seen & (uint64_t{1} << 2U)) != 0U) return false;
                    if (length != 2U) return false;
                    uint16_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_tag_id(decoded);
                    seen |= uint64_t{1} << 2U;
                    break;
                }
                case 4U: {
                    if ((seen & (uint64_t{1} << 3U)) != 0U) return false;
                    if (length > 480U || length % 1U != 0U) return false;
                    for (size_t item = 0U; item < length / 1U; ++item)
                        if (!detail::get_scalar(payload + item * 1U, 1U,
                                                &scratch->payload_.data_[item])) return false;
                    scratch->payload_.size_ = length / 1U;
                    seen |= uint64_t{1} << 3U;
                    break;
                }
                default: break;
            }
            offset += length;
        }
        if ((seen & 15ULL) != 15ULL) return false;
        *output = *scratch;
        return true;
    }
};

template <> struct Codec<dp::iface::OfflineLogCommandAckV1> {
    using Value = dp::iface::OfflineLogCommandAckV1;
    static constexpr size_t kMaxEncodedSize = 269U;
    static constexpr size_t kScratchSize = sizeof(Value);
    static constexpr size_t kAlignment = alignof(Value);
    static constexpr uint32_t kInterfaceId = 4123U;
    static bool encode(const Value& message, uint8_t* output, size_t capacity, size_t* used) {
        if (used == nullptr) return false;
        *used = 0U;
        if (output == nullptr) return false;
        size_t required_bytes = 0U;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, message.command().size())) return false;
        if (!detail::reserve(capacity, &required_bytes, message.detail().size())) return false;
        if (required_bytes > kMaxEncodedSize) return false;
        size_t offset = 0U;
        {
            const auto value = message.ok();
            uint8_t* payload = detail::append(output, capacity, &offset, 1U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.command();
            const size_t length = value.size();
            uint8_t* payload = detail::append(output, capacity, &offset, 2U, length);
            if (payload == nullptr) return false;
            if (length != 0U) std::memcpy(payload, value.data(), length);
        }
        {
            const auto value = message.detail();
            const size_t length = value.size();
            uint8_t* payload = detail::append(output, capacity, &offset, 3U, length);
            if (payload == nullptr) return false;
            if (length != 0U) std::memcpy(payload, value.data(), length);
        }
        *used = offset;
        return true;
    }
    static bool decode(const uint8_t* input, size_t size, Value* output, Value* scratch) {
        if (input == nullptr || output == nullptr || scratch == nullptr || output == scratch ||
            size > UINT16_MAX) return false;
        *scratch = Value{};
        uint64_t seen = 0U;
        size_t offset = 0U;
        while (offset < size) {
            if (size - offset < 4U) return false;
            const uint16_t tag = detail::get_u16(input + offset);
            const size_t length = detail::get_u16(input + offset + 2U);
            offset += 4U;
            if (tag == 0U || length > size - offset) return false;
            const uint8_t* payload = input + offset;
            switch (tag) {
                case 1U: {
                    if ((seen & (uint64_t{1} << 0U)) != 0U) return false;
                    if (length != 1U) return false;
                    bool decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_ok(decoded);
                    seen |= uint64_t{1} << 0U;
                    break;
                }
                case 2U: {
                    if ((seen & (uint64_t{1} << 1U)) != 0U) return false;
                    if (length > 64U || scratch->set_command(
                            {reinterpret_cast<const char*>(payload), length}) != SetResult::kOk) return false;
                    seen |= uint64_t{1} << 1U;
                    break;
                }
                case 3U: {
                    if ((seen & (uint64_t{1} << 2U)) != 0U) return false;
                    if (length > 192U || scratch->set_detail(
                            {reinterpret_cast<const char*>(payload), length}) != SetResult::kOk) return false;
                    seen |= uint64_t{1} << 2U;
                    break;
                }
                default: break;
            }
            offset += length;
        }
        if ((seen & 1ULL) != 1ULL) return false;
        *output = *scratch;
        return true;
    }
};

template <> struct Codec<dp::iface::OfflineLogCommandV1> {
    using Value = dp::iface::OfflineLogCommandV1;
    static constexpr size_t kMaxEncodedSize = 392U;
    static constexpr size_t kScratchSize = sizeof(Value);
    static constexpr size_t kAlignment = alignof(Value);
    static constexpr uint32_t kInterfaceId = 4122U;
    static bool encode(const Value& message, uint8_t* output, size_t capacity, size_t* used) {
        if (used == nullptr) return false;
        *used = 0U;
        if (output == nullptr) return false;
        size_t required_bytes = 0U;
        if (!detail::reserve(capacity, &required_bytes, message.command().size())) return false;
        if (!detail::reserve(capacity, &required_bytes, message.request_id().size())) return false;
        if (required_bytes > kMaxEncodedSize) return false;
        size_t offset = 0U;
        {
            const auto value = message.command();
            const size_t length = value.size();
            uint8_t* payload = detail::append(output, capacity, &offset, 1U, length);
            if (payload == nullptr) return false;
            if (length != 0U) std::memcpy(payload, value.data(), length);
        }
        {
            const auto value = message.request_id();
            const size_t length = value.size();
            uint8_t* payload = detail::append(output, capacity, &offset, 2U, length);
            if (payload == nullptr) return false;
            if (length != 0U) std::memcpy(payload, value.data(), length);
        }
        *used = offset;
        return true;
    }
    static bool decode(const uint8_t* input, size_t size, Value* output, Value* scratch) {
        if (input == nullptr || output == nullptr || scratch == nullptr || output == scratch ||
            size > UINT16_MAX) return false;
        *scratch = Value{};
        uint64_t seen = 0U;
        size_t offset = 0U;
        while (offset < size) {
            if (size - offset < 4U) return false;
            const uint16_t tag = detail::get_u16(input + offset);
            const size_t length = detail::get_u16(input + offset + 2U);
            offset += 4U;
            if (tag == 0U || length > size - offset) return false;
            const uint8_t* payload = input + offset;
            switch (tag) {
                case 1U: {
                    if ((seen & (uint64_t{1} << 0U)) != 0U) return false;
                    if (length > 320U || scratch->set_command(
                            {reinterpret_cast<const char*>(payload), length}) != SetResult::kOk) return false;
                    seen |= uint64_t{1} << 0U;
                    break;
                }
                case 2U: {
                    if ((seen & (uint64_t{1} << 1U)) != 0U) return false;
                    if (length > 64U || scratch->set_request_id(
                            {reinterpret_cast<const char*>(payload), length}) != SetResult::kOk) return false;
                    seen |= uint64_t{1} << 1U;
                    break;
                }
                default: break;
            }
            offset += length;
        }
        if ((seen & 1ULL) != 1ULL) return false;
        *output = *scratch;
        return true;
    }
};

template <> struct Codec<dp::iface::RemoteLevelV1> {
    using Value = dp::iface::RemoteLevelV1;
    static constexpr size_t kMaxEncodedSize = 108U;
    static constexpr size_t kScratchSize = sizeof(Value);
    static constexpr size_t kAlignment = alignof(Value);
    static constexpr uint32_t kInterfaceId = 4103U;
    static bool encode(const Value& message, uint8_t* output, size_t capacity, size_t* used) {
        if (used == nullptr) return false;
        *used = 0U;
        if (output == nullptr) return false;
        size_t required_bytes = 0U;
        if (!detail::reserve(capacity, &required_bytes, message.device_id().size())) return false;
        if (!detail::reserve(capacity, &required_bytes, message.level().size())) return false;
        if (!detail::reserve(capacity, &required_bytes, message.command().size())) return false;
        if (required_bytes > kMaxEncodedSize) return false;
        size_t offset = 0U;
        {
            const auto value = message.device_id();
            const size_t length = value.size();
            uint8_t* payload = detail::append(output, capacity, &offset, 1U, length);
            if (payload == nullptr) return false;
            if (length != 0U) std::memcpy(payload, value.data(), length);
        }
        {
            const auto value = message.level();
            const size_t length = value.size();
            uint8_t* payload = detail::append(output, capacity, &offset, 2U, length);
            if (payload == nullptr) return false;
            if (length != 0U) std::memcpy(payload, value.data(), length);
        }
        {
            const auto value = message.command();
            const size_t length = value.size();
            uint8_t* payload = detail::append(output, capacity, &offset, 3U, length);
            if (payload == nullptr) return false;
            if (length != 0U) std::memcpy(payload, value.data(), length);
        }
        *used = offset;
        return true;
    }
    static bool decode(const uint8_t* input, size_t size, Value* output, Value* scratch) {
        if (input == nullptr || output == nullptr || scratch == nullptr || output == scratch ||
            size > UINT16_MAX) return false;
        *scratch = Value{};
        uint64_t seen = 0U;
        size_t offset = 0U;
        while (offset < size) {
            if (size - offset < 4U) return false;
            const uint16_t tag = detail::get_u16(input + offset);
            const size_t length = detail::get_u16(input + offset + 2U);
            offset += 4U;
            if (tag == 0U || length > size - offset) return false;
            const uint8_t* payload = input + offset;
            switch (tag) {
                case 1U: {
                    if ((seen & (uint64_t{1} << 0U)) != 0U) return false;
                    if (length > 64U || scratch->set_device_id(
                            {reinterpret_cast<const char*>(payload), length}) != SetResult::kOk) return false;
                    seen |= uint64_t{1} << 0U;
                    break;
                }
                case 2U: {
                    if ((seen & (uint64_t{1} << 1U)) != 0U) return false;
                    if (length > 16U || scratch->set_level(
                            {reinterpret_cast<const char*>(payload), length}) != SetResult::kOk) return false;
                    seen |= uint64_t{1} << 1U;
                    break;
                }
                case 3U: {
                    if ((seen & (uint64_t{1} << 2U)) != 0U) return false;
                    if (length > 16U || scratch->set_command(
                            {reinterpret_cast<const char*>(payload), length}) != SetResult::kOk) return false;
                    seen |= uint64_t{1} << 2U;
                    break;
                }
                default: break;
            }
            offset += length;
        }
        if ((seen & 7ULL) != 7ULL) return false;
        *output = *scratch;
        return true;
    }
};

template <> struct Codec<dp::iface::CapabilityQueryV1Request> {
    using Value = dp::iface::CapabilityQueryV1Request;
    static constexpr size_t kMaxEncodedSize = 20U;
    static constexpr size_t kScratchSize = sizeof(Value);
    static constexpr size_t kAlignment = alignof(Value);
    static constexpr uint32_t kInterfaceId = 4112U;
    static bool encode(const Value& message, uint8_t* output, size_t capacity, size_t* used) {
        if (used == nullptr) return false;
        *used = 0U;
        if (output == nullptr) return false;
        size_t required_bytes = 0U;
        if (!detail::reserve(capacity, &required_bytes, message.device_id().size() * 1U)) return false;
        if (required_bytes > kMaxEncodedSize) return false;
        size_t offset = 0U;
        {
            const auto value = message.device_id();
            const size_t length = value.size() * 1U;
            uint8_t* payload = detail::append(output, capacity, &offset, 1U, length);
            if (payload == nullptr) return false;
            for (size_t index = 0U; index < value.size(); ++index)
                detail::put_scalar(payload + index * 1U, value.data()[index]);
        }
        *used = offset;
        return true;
    }
    static bool decode(const uint8_t* input, size_t size, Value* output, Value* scratch) {
        if (input == nullptr || output == nullptr || scratch == nullptr || output == scratch ||
            size > UINT16_MAX) return false;
        *scratch = Value{};
        uint64_t seen = 0U;
        size_t offset = 0U;
        while (offset < size) {
            if (size - offset < 4U) return false;
            const uint16_t tag = detail::get_u16(input + offset);
            const size_t length = detail::get_u16(input + offset + 2U);
            offset += 4U;
            if (tag == 0U || length > size - offset) return false;
            const uint8_t* payload = input + offset;
            switch (tag) {
                case 1U: {
                    if ((seen & (uint64_t{1} << 0U)) != 0U) return false;
                    if (length > 16U || length % 1U != 0U) return false;
                    for (size_t item = 0U; item < length / 1U; ++item)
                        if (!detail::get_scalar(payload + item * 1U, 1U,
                                                &scratch->device_id_.data_[item])) return false;
                    scratch->device_id_.size_ = length / 1U;
                    seen |= uint64_t{1} << 0U;
                    break;
                }
                default: break;
            }
            offset += length;
        }
        if ((seen & 1ULL) != 1ULL) return false;
        *output = *scratch;
        return true;
    }
};

template <> struct Codec<dp::iface::CapabilityQueryV1Response> {
    using Value = dp::iface::CapabilityQueryV1Response;
    static constexpr size_t kMaxEncodedSize = 48U;
    static constexpr size_t kScratchSize = sizeof(Value);
    static constexpr size_t kAlignment = alignof(Value);
    static constexpr uint32_t kInterfaceId = 4113U;
    static bool encode(const Value& message, uint8_t* output, size_t capacity, size_t* used) {
        if (used == nullptr) return false;
        *used = 0U;
        if (output == nullptr) return false;
        size_t required_bytes = 0U;
        if (!detail::reserve(capacity, &required_bytes, message.device_id().size() * 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, message.capabilities().size() * 1U)) return false;
        if (required_bytes > kMaxEncodedSize) return false;
        size_t offset = 0U;
        {
            const auto value = message.device_id();
            const size_t length = value.size() * 1U;
            uint8_t* payload = detail::append(output, capacity, &offset, 1U, length);
            if (payload == nullptr) return false;
            for (size_t index = 0U; index < value.size(); ++index)
                detail::put_scalar(payload + index * 1U, value.data()[index]);
        }
        {
            const auto value = message.protocol_version();
            uint8_t* payload = detail::append(output, capacity, &offset, 2U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.capabilities();
            const size_t length = value.size() * 1U;
            uint8_t* payload = detail::append(output, capacity, &offset, 3U, length);
            if (payload == nullptr) return false;
            for (size_t index = 0U; index < value.size(); ++index)
                detail::put_scalar(payload + index * 1U, value.data()[index]);
        }
        *used = offset;
        return true;
    }
    static bool decode(const uint8_t* input, size_t size, Value* output, Value* scratch) {
        if (input == nullptr || output == nullptr || scratch == nullptr || output == scratch ||
            size > UINT16_MAX) return false;
        *scratch = Value{};
        uint64_t seen = 0U;
        size_t offset = 0U;
        while (offset < size) {
            if (size - offset < 4U) return false;
            const uint16_t tag = detail::get_u16(input + offset);
            const size_t length = detail::get_u16(input + offset + 2U);
            offset += 4U;
            if (tag == 0U || length > size - offset) return false;
            const uint8_t* payload = input + offset;
            switch (tag) {
                case 1U: {
                    if ((seen & (uint64_t{1} << 0U)) != 0U) return false;
                    if (length > 16U || length % 1U != 0U) return false;
                    for (size_t item = 0U; item < length / 1U; ++item)
                        if (!detail::get_scalar(payload + item * 1U, 1U,
                                                &scratch->device_id_.data_[item])) return false;
                    scratch->device_id_.size_ = length / 1U;
                    seen |= uint64_t{1} << 0U;
                    break;
                }
                case 2U: {
                    if ((seen & (uint64_t{1} << 1U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_protocol_version(decoded);
                    seen |= uint64_t{1} << 1U;
                    break;
                }
                case 3U: {
                    if ((seen & (uint64_t{1} << 2U)) != 0U) return false;
                    if (length != 16U) return false;
                    for (size_t item = 0U; item < length / 1U; ++item)
                        if (!detail::get_scalar(payload + item * 1U, 1U,
                                                &scratch->capabilities_.data_[item])) return false;
                    seen |= uint64_t{1} << 2U;
                    break;
                }
                default: break;
            }
            offset += length;
        }
        if ((seen & 5ULL) != 5ULL) return false;
        *output = *scratch;
        return true;
    }
};

template <> struct Codec<dp::iface::DummyManageV1Request> {
    using Value = dp::iface::DummyManageV1Request;
    static constexpr size_t kMaxEncodedSize = 329U;
    static constexpr size_t kScratchSize = sizeof(Value);
    static constexpr size_t kAlignment = alignof(Value);
    static constexpr uint32_t kInterfaceId = 4135U;
    static bool encode(const Value& message, uint8_t* output, size_t capacity, size_t* used) {
        if (used == nullptr) return false;
        *used = 0U;
        if (output == nullptr) return false;
        size_t required_bytes = 0U;
        if (!detail::reserve(capacity, &required_bytes, 2U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 2U)) return false;
        if (!detail::reserve(capacity, &required_bytes, message.device_id().size() * 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 8U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 8U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, message.client_instance_id().size() * 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 8U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 8U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 8U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 8U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 2U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, message.permit_sha256().size() * 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, message.compiled_policy_sha256().size() * 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 8U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, message.terminal_position_rad().size() * 8U)) return false;
        if (required_bytes > kMaxEncodedSize) return false;
        size_t offset = 0U;
        {
            const auto value = message.schema_major();
            uint8_t* payload = detail::append(output, capacity, &offset, 1U, 2U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.schema_minor();
            uint8_t* payload = detail::append(output, capacity, &offset, 2U, 2U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.device_id();
            const size_t length = value.size() * 1U;
            uint8_t* payload = detail::append(output, capacity, &offset, 3U, length);
            if (payload == nullptr) return false;
            for (size_t index = 0U; index < value.size(); ++index)
                detail::put_scalar(payload + index * 1U, value.data()[index]);
        }
        {
            const auto value = message.expected_boot_epoch();
            uint8_t* payload = detail::append(output, capacity, &offset, 4U, 8U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.expected_session_id();
            uint8_t* payload = detail::append(output, capacity, &offset, 5U, 8U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.expected_reference_epoch();
            uint8_t* payload = detail::append(output, capacity, &offset, 6U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.expected_motion_epoch();
            uint8_t* payload = detail::append(output, capacity, &offset, 7U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.expected_management_epoch();
            uint8_t* payload = detail::append(output, capacity, &offset, 8U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.expected_profile_id();
            uint8_t* payload = detail::append(output, capacity, &offset, 9U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.client_instance_id();
            const size_t length = value.size() * 1U;
            uint8_t* payload = detail::append(output, capacity, &offset, 10U, length);
            if (payload == nullptr) return false;
            for (size_t index = 0U; index < value.size(); ++index)
                detail::put_scalar(payload + index * 1U, value.data()[index]);
        }
        {
            const auto value = message.request_id();
            uint8_t* payload = detail::append(output, capacity, &offset, 11U, 8U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.request_seq();
            uint8_t* payload = detail::append(output, capacity, &offset, 12U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.freshness_ticket();
            uint8_t* payload = detail::append(output, capacity, &offset, 13U, 8U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.queried_operation_id();
            uint8_t* payload = detail::append(output, capacity, &offset, 14U, 8U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.queried_request_id();
            uint8_t* payload = detail::append(output, capacity, &offset, 15U, 8U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.operation();
            uint8_t* payload = detail::append(output, capacity, &offset, 16U, 2U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.requested_control_mode();
            uint8_t* payload = detail::append(output, capacity, &offset, 17U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.configured_mask();
            uint8_t* payload = detail::append(output, capacity, &offset, 18U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.operator_confirmed();
            uint8_t* payload = detail::append(output, capacity, &offset, 19U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.permit_sha256();
            const size_t length = value.size() * 1U;
            uint8_t* payload = detail::append(output, capacity, &offset, 20U, length);
            if (payload == nullptr) return false;
            for (size_t index = 0U; index < value.size(); ++index)
                detail::put_scalar(payload + index * 1U, value.data()[index]);
        }
        {
            const auto value = message.compiled_policy_sha256();
            const size_t length = value.size() * 1U;
            uint8_t* payload = detail::append(output, capacity, &offset, 21U, length);
            if (payload == nullptr) return false;
            for (size_t index = 0U; index < value.size(); ++index)
                detail::put_scalar(payload + index * 1U, value.data()[index]);
        }
        {
            const auto value = message.permit_remaining_us();
            uint8_t* payload = detail::append(output, capacity, &offset, 22U, 8U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.terminal_producer_seq();
            uint8_t* payload = detail::append(output, capacity, &offset, 23U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.terminal_position_rad();
            const size_t length = value.size() * 8U;
            uint8_t* payload = detail::append(output, capacity, &offset, 24U, length);
            if (payload == nullptr) return false;
            for (size_t index = 0U; index < value.size(); ++index)
                detail::put_scalar(payload + index * 8U, value.data()[index]);
        }
        *used = offset;
        return true;
    }
    static bool decode(const uint8_t* input, size_t size, Value* output, Value* scratch) {
        if (input == nullptr || output == nullptr || scratch == nullptr || output == scratch ||
            size > UINT16_MAX) return false;
        *scratch = Value{};
        uint64_t seen = 0U;
        size_t offset = 0U;
        while (offset < size) {
            if (size - offset < 4U) return false;
            const uint16_t tag = detail::get_u16(input + offset);
            const size_t length = detail::get_u16(input + offset + 2U);
            offset += 4U;
            if (tag == 0U || length > size - offset) return false;
            const uint8_t* payload = input + offset;
            switch (tag) {
                case 1U: {
                    if ((seen & (uint64_t{1} << 0U)) != 0U) return false;
                    if (length != 2U) return false;
                    uint16_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_schema_major(decoded);
                    seen |= uint64_t{1} << 0U;
                    break;
                }
                case 2U: {
                    if ((seen & (uint64_t{1} << 1U)) != 0U) return false;
                    if (length != 2U) return false;
                    uint16_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_schema_minor(decoded);
                    seen |= uint64_t{1} << 1U;
                    break;
                }
                case 3U: {
                    if ((seen & (uint64_t{1} << 2U)) != 0U) return false;
                    if (length != 16U) return false;
                    for (size_t item = 0U; item < length / 1U; ++item)
                        if (!detail::get_scalar(payload + item * 1U, 1U,
                                                &scratch->device_id_.data_[item])) return false;
                    seen |= uint64_t{1} << 2U;
                    break;
                }
                case 4U: {
                    if ((seen & (uint64_t{1} << 3U)) != 0U) return false;
                    if (length != 8U) return false;
                    uint64_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_expected_boot_epoch(decoded);
                    seen |= uint64_t{1} << 3U;
                    break;
                }
                case 5U: {
                    if ((seen & (uint64_t{1} << 4U)) != 0U) return false;
                    if (length != 8U) return false;
                    uint64_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_expected_session_id(decoded);
                    seen |= uint64_t{1} << 4U;
                    break;
                }
                case 6U: {
                    if ((seen & (uint64_t{1} << 5U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_expected_reference_epoch(decoded);
                    seen |= uint64_t{1} << 5U;
                    break;
                }
                case 7U: {
                    if ((seen & (uint64_t{1} << 6U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_expected_motion_epoch(decoded);
                    seen |= uint64_t{1} << 6U;
                    break;
                }
                case 8U: {
                    if ((seen & (uint64_t{1} << 7U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_expected_management_epoch(decoded);
                    seen |= uint64_t{1} << 7U;
                    break;
                }
                case 9U: {
                    if ((seen & (uint64_t{1} << 8U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_expected_profile_id(decoded);
                    seen |= uint64_t{1} << 8U;
                    break;
                }
                case 10U: {
                    if ((seen & (uint64_t{1} << 9U)) != 0U) return false;
                    if (length != 16U) return false;
                    for (size_t item = 0U; item < length / 1U; ++item)
                        if (!detail::get_scalar(payload + item * 1U, 1U,
                                                &scratch->client_instance_id_.data_[item])) return false;
                    seen |= uint64_t{1} << 9U;
                    break;
                }
                case 11U: {
                    if ((seen & (uint64_t{1} << 10U)) != 0U) return false;
                    if (length != 8U) return false;
                    uint64_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_request_id(decoded);
                    seen |= uint64_t{1} << 10U;
                    break;
                }
                case 12U: {
                    if ((seen & (uint64_t{1} << 11U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_request_seq(decoded);
                    seen |= uint64_t{1} << 11U;
                    break;
                }
                case 13U: {
                    if ((seen & (uint64_t{1} << 12U)) != 0U) return false;
                    if (length != 8U) return false;
                    uint64_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_freshness_ticket(decoded);
                    seen |= uint64_t{1} << 12U;
                    break;
                }
                case 14U: {
                    if ((seen & (uint64_t{1} << 13U)) != 0U) return false;
                    if (length != 8U) return false;
                    uint64_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_queried_operation_id(decoded);
                    seen |= uint64_t{1} << 13U;
                    break;
                }
                case 15U: {
                    if ((seen & (uint64_t{1} << 14U)) != 0U) return false;
                    if (length != 8U) return false;
                    uint64_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_queried_request_id(decoded);
                    seen |= uint64_t{1} << 14U;
                    break;
                }
                case 16U: {
                    if ((seen & (uint64_t{1} << 15U)) != 0U) return false;
                    if (length != 2U) return false;
                    uint16_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_operation(decoded);
                    seen |= uint64_t{1} << 15U;
                    break;
                }
                case 17U: {
                    if ((seen & (uint64_t{1} << 16U)) != 0U) return false;
                    if (length != 1U) return false;
                    uint8_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_requested_control_mode(decoded);
                    seen |= uint64_t{1} << 16U;
                    break;
                }
                case 18U: {
                    if ((seen & (uint64_t{1} << 17U)) != 0U) return false;
                    if (length != 1U) return false;
                    uint8_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_configured_mask(decoded);
                    seen |= uint64_t{1} << 17U;
                    break;
                }
                case 19U: {
                    if ((seen & (uint64_t{1} << 18U)) != 0U) return false;
                    if (length != 1U) return false;
                    bool decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_operator_confirmed(decoded);
                    seen |= uint64_t{1} << 18U;
                    break;
                }
                case 20U: {
                    if ((seen & (uint64_t{1} << 19U)) != 0U) return false;
                    if (length != 32U) return false;
                    for (size_t item = 0U; item < length / 1U; ++item)
                        if (!detail::get_scalar(payload + item * 1U, 1U,
                                                &scratch->permit_sha256_.data_[item])) return false;
                    seen |= uint64_t{1} << 19U;
                    break;
                }
                case 21U: {
                    if ((seen & (uint64_t{1} << 20U)) != 0U) return false;
                    if (length != 32U) return false;
                    for (size_t item = 0U; item < length / 1U; ++item)
                        if (!detail::get_scalar(payload + item * 1U, 1U,
                                                &scratch->compiled_policy_sha256_.data_[item])) return false;
                    seen |= uint64_t{1} << 20U;
                    break;
                }
                case 22U: {
                    if ((seen & (uint64_t{1} << 21U)) != 0U) return false;
                    if (length != 8U) return false;
                    uint64_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_permit_remaining_us(decoded);
                    seen |= uint64_t{1} << 21U;
                    break;
                }
                case 23U: {
                    if ((seen & (uint64_t{1} << 22U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_terminal_producer_seq(decoded);
                    seen |= uint64_t{1} << 22U;
                    break;
                }
                case 24U: {
                    if ((seen & (uint64_t{1} << 23U)) != 0U) return false;
                    if (length != 48U) return false;
                    for (size_t item = 0U; item < length / 8U; ++item)
                        if (!detail::get_scalar(payload + item * 8U, 8U,
                                                &scratch->terminal_position_rad_.data_[item])) return false;
                    seen |= uint64_t{1} << 23U;
                    break;
                }
                default: break;
            }
            offset += length;
        }
        if ((seen & 16777215ULL) != 16777215ULL) return false;
        *output = *scratch;
        return true;
    }
};

template <> struct Codec<dp::iface::DummyManageV1Response> {
    using Value = dp::iface::DummyManageV1Response;
    static constexpr size_t kMaxEncodedSize = 882U;
    static constexpr size_t kScratchSize = sizeof(Value);
    static constexpr size_t kAlignment = alignof(Value);
    static constexpr uint32_t kInterfaceId = 4136U;
    static bool encode(const Value& message, uint8_t* output, size_t capacity, size_t* used) {
        if (used == nullptr) return false;
        *used = 0U;
        if (output == nullptr) return false;
        size_t required_bytes = 0U;
        if (!detail::reserve(capacity, &required_bytes, 2U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 2U)) return false;
        if (!detail::reserve(capacity, &required_bytes, message.device_id().size() * 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, message.client_instance_id().size() * 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, message.owner_client_instance_id().size() * 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 8U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 8U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 2U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 8U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 2U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 8U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 8U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 8U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 8U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, message.feedback_seq().size() * 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, message.can_rx_age_us().size() * 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, message.position_rad().size() * 8U)) return false;
        if (!detail::reserve(capacity, &required_bytes, message.estimated_velocity_rad_s().size() * 8U)) return false;
        if (!detail::reserve(capacity, &required_bytes, message.hold_position_rad().size() * 8U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 2U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 2U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, message.raw_motor_position_rev().size() * 8U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 8U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 8U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 2U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, message.contract_sha256().size() * 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, message.model_sha256().size() * 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, message.profile_sha256().size() * 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, message.activation_profile_sha256().size() * 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, message.firmware_sha256().size() * 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, message.board_name_ascii().size() * 1U)) return false;
        if (required_bytes > kMaxEncodedSize) return false;
        size_t offset = 0U;
        {
            const auto value = message.schema_major();
            uint8_t* payload = detail::append(output, capacity, &offset, 1U, 2U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.schema_minor();
            uint8_t* payload = detail::append(output, capacity, &offset, 2U, 2U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.device_id();
            const size_t length = value.size() * 1U;
            uint8_t* payload = detail::append(output, capacity, &offset, 3U, length);
            if (payload == nullptr) return false;
            for (size_t index = 0U; index < value.size(); ++index)
                detail::put_scalar(payload + index * 1U, value.data()[index]);
        }
        {
            const auto value = message.client_instance_id();
            const size_t length = value.size() * 1U;
            uint8_t* payload = detail::append(output, capacity, &offset, 4U, length);
            if (payload == nullptr) return false;
            for (size_t index = 0U; index < value.size(); ++index)
                detail::put_scalar(payload + index * 1U, value.data()[index]);
        }
        {
            const auto value = message.owner_client_instance_id();
            const size_t length = value.size() * 1U;
            uint8_t* payload = detail::append(output, capacity, &offset, 5U, length);
            if (payload == nullptr) return false;
            for (size_t index = 0U; index < value.size(); ++index)
                detail::put_scalar(payload + index * 1U, value.data()[index]);
        }
        {
            const auto value = message.request_id();
            uint8_t* payload = detail::append(output, capacity, &offset, 6U, 8U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.receipt();
            uint8_t* payload = detail::append(output, capacity, &offset, 7U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.core_request_id();
            uint8_t* payload = detail::append(output, capacity, &offset, 8U, 8U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.accepted();
            uint8_t* payload = detail::append(output, capacity, &offset, 9U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.result_code();
            uint8_t* payload = detail::append(output, capacity, &offset, 10U, 2U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.operation_result_valid();
            uint8_t* payload = detail::append(output, capacity, &offset, 11U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.operation_id();
            uint8_t* payload = detail::append(output, capacity, &offset, 12U, 8U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.operation_status();
            uint8_t* payload = detail::append(output, capacity, &offset, 13U, 2U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.snapshot_valid();
            uint8_t* payload = detail::append(output, capacity, &offset, 14U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.info_valid();
            uint8_t* payload = detail::append(output, capacity, &offset, 15U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.status_valid();
            uint8_t* payload = detail::append(output, capacity, &offset, 16U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.boot_epoch();
            uint8_t* payload = detail::append(output, capacity, &offset, 17U, 8U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.session_id();
            uint8_t* payload = detail::append(output, capacity, &offset, 18U, 8U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.reference_epoch();
            uint8_t* payload = detail::append(output, capacity, &offset, 19U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.motion_epoch();
            uint8_t* payload = detail::append(output, capacity, &offset, 20U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.management_epoch();
            uint8_t* payload = detail::append(output, capacity, &offset, 21U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.profile_id();
            uint8_t* payload = detail::append(output, capacity, &offset, 22U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.state_seq();
            uint8_t* payload = detail::append(output, capacity, &offset, 23U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.produced_at_us();
            uint8_t* payload = detail::append(output, capacity, &offset, 24U, 8U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.freshness_ticket();
            uint8_t* payload = detail::append(output, capacity, &offset, 25U, 8U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.configured_mask();
            uint8_t* payload = detail::append(output, capacity, &offset, 26U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.position_valid_mask();
            uint8_t* payload = detail::append(output, capacity, &offset, 27U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.velocity_valid_mask();
            uint8_t* payload = detail::append(output, capacity, &offset, 28U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.planner_finish_mask();
            uint8_t* payload = detail::append(output, capacity, &offset, 29U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.hold_valid_mask();
            uint8_t* payload = detail::append(output, capacity, &offset, 30U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.accepted_seq_valid();
            uint8_t* payload = detail::append(output, capacity, &offset, 31U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.accepted_producer_seq();
            uint8_t* payload = detail::append(output, capacity, &offset, 32U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.applied_seq_valid();
            uint8_t* payload = detail::append(output, capacity, &offset, 33U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.applied_producer_seq();
            uint8_t* payload = detail::append(output, capacity, &offset, 34U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.tx_group_seq_valid();
            uint8_t* payload = detail::append(output, capacity, &offset, 35U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.tx_group_producer_seq();
            uint8_t* payload = detail::append(output, capacity, &offset, 36U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.submitted_axis_mask();
            uint8_t* payload = detail::append(output, capacity, &offset, 37U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.sent_axis_mask();
            uint8_t* payload = detail::append(output, capacity, &offset, 38U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.feedback_seq();
            const size_t length = value.size() * 4U;
            uint8_t* payload = detail::append(output, capacity, &offset, 39U, length);
            if (payload == nullptr) return false;
            for (size_t index = 0U; index < value.size(); ++index)
                detail::put_scalar(payload + index * 4U, value.data()[index]);
        }
        {
            const auto value = message.can_rx_age_us();
            const size_t length = value.size() * 4U;
            uint8_t* payload = detail::append(output, capacity, &offset, 40U, length);
            if (payload == nullptr) return false;
            for (size_t index = 0U; index < value.size(); ++index)
                detail::put_scalar(payload + index * 4U, value.data()[index]);
        }
        {
            const auto value = message.position_rad();
            const size_t length = value.size() * 8U;
            uint8_t* payload = detail::append(output, capacity, &offset, 41U, length);
            if (payload == nullptr) return false;
            for (size_t index = 0U; index < value.size(); ++index)
                detail::put_scalar(payload + index * 8U, value.data()[index]);
        }
        {
            const auto value = message.estimated_velocity_rad_s();
            const size_t length = value.size() * 8U;
            uint8_t* payload = detail::append(output, capacity, &offset, 42U, length);
            if (payload == nullptr) return false;
            for (size_t index = 0U; index < value.size(); ++index)
                detail::put_scalar(payload + index * 8U, value.data()[index]);
        }
        {
            const auto value = message.hold_position_rad();
            const size_t length = value.size() * 8U;
            uint8_t* payload = detail::append(output, capacity, &offset, 43U, length);
            if (payload == nullptr) return false;
            for (size_t index = 0U; index < value.size(); ++index)
                detail::put_scalar(payload + index * 8U, value.data()[index]);
        }
        {
            const auto value = message.core_state();
            uint8_t* payload = detail::append(output, capacity, &offset, 44U, 2U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.fault_reason();
            uint8_t* payload = detail::append(output, capacity, &offset, 45U, 2U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.hold_ready();
            uint8_t* payload = detail::append(output, capacity, &offset, 46U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.raw_position_valid_mask();
            uint8_t* payload = detail::append(output, capacity, &offset, 47U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.raw_motor_position_rev();
            const size_t length = value.size() * 8U;
            uint8_t* payload = detail::append(output, capacity, &offset, 48U, length);
            if (payload == nullptr) return false;
            for (size_t index = 0U; index < value.size(); ++index)
                detail::put_scalar(payload + index * 8U, value.data()[index]);
        }
        {
            const auto value = message.control_mode();
            uint8_t* payload = detail::append(output, capacity, &offset, 49U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.reference_kind();
            uint8_t* payload = detail::append(output, capacity, &offset, 50U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.drive_request();
            uint8_t* payload = detail::append(output, capacity, &offset, 51U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.drive_evidence();
            uint8_t* payload = detail::append(output, capacity, &offset, 52U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.real_motion_permitted();
            uint8_t* payload = detail::append(output, capacity, &offset, 53U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.permit_remaining_us();
            uint8_t* payload = detail::append(output, capacity, &offset, 54U, 8U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.capabilities();
            uint8_t* payload = detail::append(output, capacity, &offset, 55U, 8U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.operation_result_code();
            uint8_t* payload = detail::append(output, capacity, &offset, 56U, 2U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.command_reject_count();
            uint8_t* payload = detail::append(output, capacity, &offset, 57U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.can_error_count();
            uint8_t* payload = detail::append(output, capacity, &offset, 58U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.deadline_miss_count();
            uint8_t* payload = detail::append(output, capacity, &offset, 59U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.contract_sha256();
            const size_t length = value.size() * 1U;
            uint8_t* payload = detail::append(output, capacity, &offset, 60U, length);
            if (payload == nullptr) return false;
            for (size_t index = 0U; index < value.size(); ++index)
                detail::put_scalar(payload + index * 1U, value.data()[index]);
        }
        {
            const auto value = message.model_sha256();
            const size_t length = value.size() * 1U;
            uint8_t* payload = detail::append(output, capacity, &offset, 61U, length);
            if (payload == nullptr) return false;
            for (size_t index = 0U; index < value.size(); ++index)
                detail::put_scalar(payload + index * 1U, value.data()[index]);
        }
        {
            const auto value = message.profile_sha256();
            const size_t length = value.size() * 1U;
            uint8_t* payload = detail::append(output, capacity, &offset, 62U, length);
            if (payload == nullptr) return false;
            for (size_t index = 0U; index < value.size(); ++index)
                detail::put_scalar(payload + index * 1U, value.data()[index]);
        }
        {
            const auto value = message.activation_profile_sha256();
            const size_t length = value.size() * 1U;
            uint8_t* payload = detail::append(output, capacity, &offset, 63U, length);
            if (payload == nullptr) return false;
            for (size_t index = 0U; index < value.size(); ++index)
                detail::put_scalar(payload + index * 1U, value.data()[index]);
        }
        {
            const auto value = message.motion_supported();
            uint8_t* payload = detail::append(output, capacity, &offset, 64U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.firmware_sha256();
            const size_t length = value.size() * 1U;
            uint8_t* payload = detail::append(output, capacity, &offset, 65U, length);
            if (payload == nullptr) return false;
            for (size_t index = 0U; index < value.size(); ++index)
                detail::put_scalar(payload + index * 1U, value.data()[index]);
        }
        {
            const auto value = message.board_name_ascii();
            const size_t length = value.size() * 1U;
            uint8_t* payload = detail::append(output, capacity, &offset, 66U, length);
            if (payload == nullptr) return false;
            for (size_t index = 0U; index < value.size(); ++index)
                detail::put_scalar(payload + index * 1U, value.data()[index]);
        }
        *used = offset;
        return true;
    }
    static bool decode(const uint8_t* input, size_t size, Value* output, Value* scratch) {
        if (input == nullptr || output == nullptr || scratch == nullptr || output == scratch ||
            size > UINT16_MAX) return false;
        *scratch = Value{};
        uint64_t seen[2U]{};
        size_t offset = 0U;
        while (offset < size) {
            if (size - offset < 4U) return false;
            const uint16_t tag = detail::get_u16(input + offset);
            const size_t length = detail::get_u16(input + offset + 2U);
            offset += 4U;
            if (tag == 0U || length > size - offset) return false;
            const uint8_t* payload = input + offset;
            switch (tag) {
                case 1U: {
                    if ((seen[0U] & (uint64_t{1} << 0U)) != 0U) return false;
                    if (length != 2U) return false;
                    uint16_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_schema_major(decoded);
                    seen[0U] |= uint64_t{1} << 0U;
                    break;
                }
                case 2U: {
                    if ((seen[0U] & (uint64_t{1} << 1U)) != 0U) return false;
                    if (length != 2U) return false;
                    uint16_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_schema_minor(decoded);
                    seen[0U] |= uint64_t{1} << 1U;
                    break;
                }
                case 3U: {
                    if ((seen[0U] & (uint64_t{1} << 2U)) != 0U) return false;
                    if (length != 16U) return false;
                    for (size_t item = 0U; item < length / 1U; ++item)
                        if (!detail::get_scalar(payload + item * 1U, 1U,
                                                &scratch->device_id_.data_[item])) return false;
                    seen[0U] |= uint64_t{1} << 2U;
                    break;
                }
                case 4U: {
                    if ((seen[0U] & (uint64_t{1} << 3U)) != 0U) return false;
                    if (length != 16U) return false;
                    for (size_t item = 0U; item < length / 1U; ++item)
                        if (!detail::get_scalar(payload + item * 1U, 1U,
                                                &scratch->client_instance_id_.data_[item])) return false;
                    seen[0U] |= uint64_t{1} << 3U;
                    break;
                }
                case 5U: {
                    if ((seen[0U] & (uint64_t{1} << 4U)) != 0U) return false;
                    if (length != 16U) return false;
                    for (size_t item = 0U; item < length / 1U; ++item)
                        if (!detail::get_scalar(payload + item * 1U, 1U,
                                                &scratch->owner_client_instance_id_.data_[item])) return false;
                    seen[0U] |= uint64_t{1} << 4U;
                    break;
                }
                case 6U: {
                    if ((seen[0U] & (uint64_t{1} << 5U)) != 0U) return false;
                    if (length != 8U) return false;
                    uint64_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_request_id(decoded);
                    seen[0U] |= uint64_t{1} << 5U;
                    break;
                }
                case 7U: {
                    if ((seen[0U] & (uint64_t{1} << 6U)) != 0U) return false;
                    if (length != 1U) return false;
                    uint8_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_receipt(decoded);
                    seen[0U] |= uint64_t{1} << 6U;
                    break;
                }
                case 8U: {
                    if ((seen[0U] & (uint64_t{1} << 7U)) != 0U) return false;
                    if (length != 8U) return false;
                    uint64_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_core_request_id(decoded);
                    seen[0U] |= uint64_t{1} << 7U;
                    break;
                }
                case 9U: {
                    if ((seen[0U] & (uint64_t{1} << 8U)) != 0U) return false;
                    if (length != 1U) return false;
                    bool decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_accepted(decoded);
                    seen[0U] |= uint64_t{1} << 8U;
                    break;
                }
                case 10U: {
                    if ((seen[0U] & (uint64_t{1} << 9U)) != 0U) return false;
                    if (length != 2U) return false;
                    uint16_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_result_code(decoded);
                    seen[0U] |= uint64_t{1} << 9U;
                    break;
                }
                case 11U: {
                    if ((seen[0U] & (uint64_t{1} << 10U)) != 0U) return false;
                    if (length != 1U) return false;
                    bool decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_operation_result_valid(decoded);
                    seen[0U] |= uint64_t{1} << 10U;
                    break;
                }
                case 12U: {
                    if ((seen[0U] & (uint64_t{1} << 11U)) != 0U) return false;
                    if (length != 8U) return false;
                    uint64_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_operation_id(decoded);
                    seen[0U] |= uint64_t{1} << 11U;
                    break;
                }
                case 13U: {
                    if ((seen[0U] & (uint64_t{1} << 12U)) != 0U) return false;
                    if (length != 2U) return false;
                    uint16_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_operation_status(decoded);
                    seen[0U] |= uint64_t{1} << 12U;
                    break;
                }
                case 14U: {
                    if ((seen[0U] & (uint64_t{1} << 13U)) != 0U) return false;
                    if (length != 1U) return false;
                    bool decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_snapshot_valid(decoded);
                    seen[0U] |= uint64_t{1} << 13U;
                    break;
                }
                case 15U: {
                    if ((seen[0U] & (uint64_t{1} << 14U)) != 0U) return false;
                    if (length != 1U) return false;
                    bool decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_info_valid(decoded);
                    seen[0U] |= uint64_t{1} << 14U;
                    break;
                }
                case 16U: {
                    if ((seen[0U] & (uint64_t{1} << 15U)) != 0U) return false;
                    if (length != 1U) return false;
                    bool decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_status_valid(decoded);
                    seen[0U] |= uint64_t{1} << 15U;
                    break;
                }
                case 17U: {
                    if ((seen[0U] & (uint64_t{1} << 16U)) != 0U) return false;
                    if (length != 8U) return false;
                    uint64_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_boot_epoch(decoded);
                    seen[0U] |= uint64_t{1} << 16U;
                    break;
                }
                case 18U: {
                    if ((seen[0U] & (uint64_t{1} << 17U)) != 0U) return false;
                    if (length != 8U) return false;
                    uint64_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_session_id(decoded);
                    seen[0U] |= uint64_t{1} << 17U;
                    break;
                }
                case 19U: {
                    if ((seen[0U] & (uint64_t{1} << 18U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_reference_epoch(decoded);
                    seen[0U] |= uint64_t{1} << 18U;
                    break;
                }
                case 20U: {
                    if ((seen[0U] & (uint64_t{1} << 19U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_motion_epoch(decoded);
                    seen[0U] |= uint64_t{1} << 19U;
                    break;
                }
                case 21U: {
                    if ((seen[0U] & (uint64_t{1} << 20U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_management_epoch(decoded);
                    seen[0U] |= uint64_t{1} << 20U;
                    break;
                }
                case 22U: {
                    if ((seen[0U] & (uint64_t{1} << 21U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_profile_id(decoded);
                    seen[0U] |= uint64_t{1} << 21U;
                    break;
                }
                case 23U: {
                    if ((seen[0U] & (uint64_t{1} << 22U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_state_seq(decoded);
                    seen[0U] |= uint64_t{1} << 22U;
                    break;
                }
                case 24U: {
                    if ((seen[0U] & (uint64_t{1} << 23U)) != 0U) return false;
                    if (length != 8U) return false;
                    uint64_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_produced_at_us(decoded);
                    seen[0U] |= uint64_t{1} << 23U;
                    break;
                }
                case 25U: {
                    if ((seen[0U] & (uint64_t{1} << 24U)) != 0U) return false;
                    if (length != 8U) return false;
                    uint64_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_freshness_ticket(decoded);
                    seen[0U] |= uint64_t{1} << 24U;
                    break;
                }
                case 26U: {
                    if ((seen[0U] & (uint64_t{1} << 25U)) != 0U) return false;
                    if (length != 1U) return false;
                    uint8_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_configured_mask(decoded);
                    seen[0U] |= uint64_t{1} << 25U;
                    break;
                }
                case 27U: {
                    if ((seen[0U] & (uint64_t{1} << 26U)) != 0U) return false;
                    if (length != 1U) return false;
                    uint8_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_position_valid_mask(decoded);
                    seen[0U] |= uint64_t{1} << 26U;
                    break;
                }
                case 28U: {
                    if ((seen[0U] & (uint64_t{1} << 27U)) != 0U) return false;
                    if (length != 1U) return false;
                    uint8_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_velocity_valid_mask(decoded);
                    seen[0U] |= uint64_t{1} << 27U;
                    break;
                }
                case 29U: {
                    if ((seen[0U] & (uint64_t{1} << 28U)) != 0U) return false;
                    if (length != 1U) return false;
                    uint8_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_planner_finish_mask(decoded);
                    seen[0U] |= uint64_t{1} << 28U;
                    break;
                }
                case 30U: {
                    if ((seen[0U] & (uint64_t{1} << 29U)) != 0U) return false;
                    if (length != 1U) return false;
                    uint8_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_hold_valid_mask(decoded);
                    seen[0U] |= uint64_t{1} << 29U;
                    break;
                }
                case 31U: {
                    if ((seen[0U] & (uint64_t{1} << 30U)) != 0U) return false;
                    if (length != 1U) return false;
                    bool decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_accepted_seq_valid(decoded);
                    seen[0U] |= uint64_t{1} << 30U;
                    break;
                }
                case 32U: {
                    if ((seen[0U] & (uint64_t{1} << 31U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_accepted_producer_seq(decoded);
                    seen[0U] |= uint64_t{1} << 31U;
                    break;
                }
                case 33U: {
                    if ((seen[0U] & (uint64_t{1} << 32U)) != 0U) return false;
                    if (length != 1U) return false;
                    bool decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_applied_seq_valid(decoded);
                    seen[0U] |= uint64_t{1} << 32U;
                    break;
                }
                case 34U: {
                    if ((seen[0U] & (uint64_t{1} << 33U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_applied_producer_seq(decoded);
                    seen[0U] |= uint64_t{1} << 33U;
                    break;
                }
                case 35U: {
                    if ((seen[0U] & (uint64_t{1} << 34U)) != 0U) return false;
                    if (length != 1U) return false;
                    bool decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_tx_group_seq_valid(decoded);
                    seen[0U] |= uint64_t{1} << 34U;
                    break;
                }
                case 36U: {
                    if ((seen[0U] & (uint64_t{1} << 35U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_tx_group_producer_seq(decoded);
                    seen[0U] |= uint64_t{1} << 35U;
                    break;
                }
                case 37U: {
                    if ((seen[0U] & (uint64_t{1} << 36U)) != 0U) return false;
                    if (length != 1U) return false;
                    uint8_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_submitted_axis_mask(decoded);
                    seen[0U] |= uint64_t{1} << 36U;
                    break;
                }
                case 38U: {
                    if ((seen[0U] & (uint64_t{1} << 37U)) != 0U) return false;
                    if (length != 1U) return false;
                    uint8_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_sent_axis_mask(decoded);
                    seen[0U] |= uint64_t{1} << 37U;
                    break;
                }
                case 39U: {
                    if ((seen[0U] & (uint64_t{1} << 38U)) != 0U) return false;
                    if (length != 24U) return false;
                    for (size_t item = 0U; item < length / 4U; ++item)
                        if (!detail::get_scalar(payload + item * 4U, 4U,
                                                &scratch->feedback_seq_.data_[item])) return false;
                    seen[0U] |= uint64_t{1} << 38U;
                    break;
                }
                case 40U: {
                    if ((seen[0U] & (uint64_t{1} << 39U)) != 0U) return false;
                    if (length != 24U) return false;
                    for (size_t item = 0U; item < length / 4U; ++item)
                        if (!detail::get_scalar(payload + item * 4U, 4U,
                                                &scratch->can_rx_age_us_.data_[item])) return false;
                    seen[0U] |= uint64_t{1} << 39U;
                    break;
                }
                case 41U: {
                    if ((seen[0U] & (uint64_t{1} << 40U)) != 0U) return false;
                    if (length != 48U) return false;
                    for (size_t item = 0U; item < length / 8U; ++item)
                        if (!detail::get_scalar(payload + item * 8U, 8U,
                                                &scratch->position_rad_.data_[item])) return false;
                    seen[0U] |= uint64_t{1} << 40U;
                    break;
                }
                case 42U: {
                    if ((seen[0U] & (uint64_t{1} << 41U)) != 0U) return false;
                    if (length != 48U) return false;
                    for (size_t item = 0U; item < length / 8U; ++item)
                        if (!detail::get_scalar(payload + item * 8U, 8U,
                                                &scratch->estimated_velocity_rad_s_.data_[item])) return false;
                    seen[0U] |= uint64_t{1} << 41U;
                    break;
                }
                case 43U: {
                    if ((seen[0U] & (uint64_t{1} << 42U)) != 0U) return false;
                    if (length != 48U) return false;
                    for (size_t item = 0U; item < length / 8U; ++item)
                        if (!detail::get_scalar(payload + item * 8U, 8U,
                                                &scratch->hold_position_rad_.data_[item])) return false;
                    seen[0U] |= uint64_t{1} << 42U;
                    break;
                }
                case 44U: {
                    if ((seen[0U] & (uint64_t{1} << 43U)) != 0U) return false;
                    if (length != 2U) return false;
                    uint16_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_core_state(decoded);
                    seen[0U] |= uint64_t{1} << 43U;
                    break;
                }
                case 45U: {
                    if ((seen[0U] & (uint64_t{1} << 44U)) != 0U) return false;
                    if (length != 2U) return false;
                    uint16_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_fault_reason(decoded);
                    seen[0U] |= uint64_t{1} << 44U;
                    break;
                }
                case 46U: {
                    if ((seen[0U] & (uint64_t{1} << 45U)) != 0U) return false;
                    if (length != 1U) return false;
                    bool decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_hold_ready(decoded);
                    seen[0U] |= uint64_t{1} << 45U;
                    break;
                }
                case 47U: {
                    if ((seen[0U] & (uint64_t{1} << 46U)) != 0U) return false;
                    if (length != 1U) return false;
                    uint8_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_raw_position_valid_mask(decoded);
                    seen[0U] |= uint64_t{1} << 46U;
                    break;
                }
                case 48U: {
                    if ((seen[0U] & (uint64_t{1} << 47U)) != 0U) return false;
                    if (length != 48U) return false;
                    for (size_t item = 0U; item < length / 8U; ++item)
                        if (!detail::get_scalar(payload + item * 8U, 8U,
                                                &scratch->raw_motor_position_rev_.data_[item])) return false;
                    seen[0U] |= uint64_t{1} << 47U;
                    break;
                }
                case 49U: {
                    if ((seen[0U] & (uint64_t{1} << 48U)) != 0U) return false;
                    if (length != 1U) return false;
                    uint8_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_control_mode(decoded);
                    seen[0U] |= uint64_t{1} << 48U;
                    break;
                }
                case 50U: {
                    if ((seen[0U] & (uint64_t{1} << 49U)) != 0U) return false;
                    if (length != 1U) return false;
                    uint8_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_reference_kind(decoded);
                    seen[0U] |= uint64_t{1} << 49U;
                    break;
                }
                case 51U: {
                    if ((seen[0U] & (uint64_t{1} << 50U)) != 0U) return false;
                    if (length != 1U) return false;
                    uint8_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_drive_request(decoded);
                    seen[0U] |= uint64_t{1} << 50U;
                    break;
                }
                case 52U: {
                    if ((seen[0U] & (uint64_t{1} << 51U)) != 0U) return false;
                    if (length != 1U) return false;
                    uint8_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_drive_evidence(decoded);
                    seen[0U] |= uint64_t{1} << 51U;
                    break;
                }
                case 53U: {
                    if ((seen[0U] & (uint64_t{1} << 52U)) != 0U) return false;
                    if (length != 1U) return false;
                    bool decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_real_motion_permitted(decoded);
                    seen[0U] |= uint64_t{1} << 52U;
                    break;
                }
                case 54U: {
                    if ((seen[0U] & (uint64_t{1} << 53U)) != 0U) return false;
                    if (length != 8U) return false;
                    uint64_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_permit_remaining_us(decoded);
                    seen[0U] |= uint64_t{1} << 53U;
                    break;
                }
                case 55U: {
                    if ((seen[0U] & (uint64_t{1} << 54U)) != 0U) return false;
                    if (length != 8U) return false;
                    uint64_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_capabilities(decoded);
                    seen[0U] |= uint64_t{1} << 54U;
                    break;
                }
                case 56U: {
                    if ((seen[0U] & (uint64_t{1} << 55U)) != 0U) return false;
                    if (length != 2U) return false;
                    uint16_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_operation_result_code(decoded);
                    seen[0U] |= uint64_t{1} << 55U;
                    break;
                }
                case 57U: {
                    if ((seen[0U] & (uint64_t{1} << 56U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_command_reject_count(decoded);
                    seen[0U] |= uint64_t{1} << 56U;
                    break;
                }
                case 58U: {
                    if ((seen[0U] & (uint64_t{1} << 57U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_can_error_count(decoded);
                    seen[0U] |= uint64_t{1} << 57U;
                    break;
                }
                case 59U: {
                    if ((seen[0U] & (uint64_t{1} << 58U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_deadline_miss_count(decoded);
                    seen[0U] |= uint64_t{1} << 58U;
                    break;
                }
                case 60U: {
                    if ((seen[0U] & (uint64_t{1} << 59U)) != 0U) return false;
                    if (length != 32U) return false;
                    for (size_t item = 0U; item < length / 1U; ++item)
                        if (!detail::get_scalar(payload + item * 1U, 1U,
                                                &scratch->contract_sha256_.data_[item])) return false;
                    seen[0U] |= uint64_t{1} << 59U;
                    break;
                }
                case 61U: {
                    if ((seen[0U] & (uint64_t{1} << 60U)) != 0U) return false;
                    if (length != 32U) return false;
                    for (size_t item = 0U; item < length / 1U; ++item)
                        if (!detail::get_scalar(payload + item * 1U, 1U,
                                                &scratch->model_sha256_.data_[item])) return false;
                    seen[0U] |= uint64_t{1} << 60U;
                    break;
                }
                case 62U: {
                    if ((seen[0U] & (uint64_t{1} << 61U)) != 0U) return false;
                    if (length != 32U) return false;
                    for (size_t item = 0U; item < length / 1U; ++item)
                        if (!detail::get_scalar(payload + item * 1U, 1U,
                                                &scratch->profile_sha256_.data_[item])) return false;
                    seen[0U] |= uint64_t{1} << 61U;
                    break;
                }
                case 63U: {
                    if ((seen[0U] & (uint64_t{1} << 62U)) != 0U) return false;
                    if (length != 32U) return false;
                    for (size_t item = 0U; item < length / 1U; ++item)
                        if (!detail::get_scalar(payload + item * 1U, 1U,
                                                &scratch->activation_profile_sha256_.data_[item])) return false;
                    seen[0U] |= uint64_t{1} << 62U;
                    break;
                }
                case 64U: {
                    if ((seen[0U] & (uint64_t{1} << 63U)) != 0U) return false;
                    if (length != 1U) return false;
                    bool decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_motion_supported(decoded);
                    seen[0U] |= uint64_t{1} << 63U;
                    break;
                }
                case 65U: {
                    if ((seen[1U] & (uint64_t{1} << 0U)) != 0U) return false;
                    if (length != 32U) return false;
                    for (size_t item = 0U; item < length / 1U; ++item)
                        if (!detail::get_scalar(payload + item * 1U, 1U,
                                                &scratch->firmware_sha256_.data_[item])) return false;
                    seen[1U] |= uint64_t{1} << 0U;
                    break;
                }
                case 66U: {
                    if ((seen[1U] & (uint64_t{1} << 1U)) != 0U) return false;
                    if (length != 16U) return false;
                    for (size_t item = 0U; item < length / 1U; ++item)
                        if (!detail::get_scalar(payload + item * 1U, 1U,
                                                &scratch->board_name_ascii_.data_[item])) return false;
                    seen[1U] |= uint64_t{1} << 1U;
                    break;
                }
                default: break;
            }
            offset += length;
        }
        if ((seen[0U] & 18446744073709551615ULL) != 18446744073709551615ULL) return false;
        if ((seen[1U] & 3ULL) != 3ULL) return false;
        *output = *scratch;
        return true;
    }
};

template <> struct Codec<dp::iface::FileTransferCancelV1Request> {
    using Value = dp::iface::FileTransferCancelV1Request;
    static constexpr size_t kMaxEncodedSize = 13U;
    static constexpr size_t kScratchSize = sizeof(Value);
    static constexpr size_t kAlignment = alignof(Value);
    static constexpr uint32_t kInterfaceId = 4120U;
    static bool encode(const Value& message, uint8_t* output, size_t capacity, size_t* used) {
        if (used == nullptr) return false;
        *used = 0U;
        if (output == nullptr) return false;
        size_t required_bytes = 0U;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (required_bytes > kMaxEncodedSize) return false;
        size_t offset = 0U;
        {
            const auto value = message.opcode();
            uint8_t* payload = detail::append(output, capacity, &offset, 1U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.transfer_id();
            uint8_t* payload = detail::append(output, capacity, &offset, 14U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        *used = offset;
        return true;
    }
    static bool decode(const uint8_t* input, size_t size, Value* output, Value* scratch) {
        if (input == nullptr || output == nullptr || scratch == nullptr || output == scratch ||
            size > UINT16_MAX) return false;
        *scratch = Value{};
        uint64_t seen = 0U;
        size_t offset = 0U;
        while (offset < size) {
            if (size - offset < 4U) return false;
            const uint16_t tag = detail::get_u16(input + offset);
            const size_t length = detail::get_u16(input + offset + 2U);
            offset += 4U;
            if (tag == 0U || length > size - offset) return false;
            const uint8_t* payload = input + offset;
            switch (tag) {
                case 1U: {
                    if ((seen & (uint64_t{1} << 0U)) != 0U) return false;
                    if (length != 1U) return false;
                    uint8_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_opcode(decoded);
                    seen |= uint64_t{1} << 0U;
                    break;
                }
                case 14U: {
                    if ((seen & (uint64_t{1} << 1U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_transfer_id(decoded);
                    seen |= uint64_t{1} << 1U;
                    break;
                }
                default: break;
            }
            offset += length;
        }
        if ((seen & 3ULL) != 3ULL) return false;
        *output = *scratch;
        return true;
    }
};

template <> struct Codec<dp::iface::FileTransferCancelV1Response> {
    using Value = dp::iface::FileTransferCancelV1Response;
    static constexpr size_t kMaxEncodedSize = 18U;
    static constexpr size_t kScratchSize = sizeof(Value);
    static constexpr size_t kAlignment = alignof(Value);
    static constexpr uint32_t kInterfaceId = 4121U;
    static bool encode(const Value& message, uint8_t* output, size_t capacity, size_t* used) {
        if (used == nullptr) return false;
        *used = 0U;
        if (output == nullptr) return false;
        size_t required_bytes = 0U;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (required_bytes > kMaxEncodedSize) return false;
        size_t offset = 0U;
        {
            const auto value = message.opcode();
            uint8_t* payload = detail::append(output, capacity, &offset, 1U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.status();
            uint8_t* payload = detail::append(output, capacity, &offset, 5U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.transfer_id();
            uint8_t* payload = detail::append(output, capacity, &offset, 14U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        *used = offset;
        return true;
    }
    static bool decode(const uint8_t* input, size_t size, Value* output, Value* scratch) {
        if (input == nullptr || output == nullptr || scratch == nullptr || output == scratch ||
            size > UINT16_MAX) return false;
        *scratch = Value{};
        uint64_t seen = 0U;
        size_t offset = 0U;
        while (offset < size) {
            if (size - offset < 4U) return false;
            const uint16_t tag = detail::get_u16(input + offset);
            const size_t length = detail::get_u16(input + offset + 2U);
            offset += 4U;
            if (tag == 0U || length > size - offset) return false;
            const uint8_t* payload = input + offset;
            switch (tag) {
                case 1U: {
                    if ((seen & (uint64_t{1} << 0U)) != 0U) return false;
                    if (length != 1U) return false;
                    uint8_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_opcode(decoded);
                    seen |= uint64_t{1} << 0U;
                    break;
                }
                case 5U: {
                    if ((seen & (uint64_t{1} << 1U)) != 0U) return false;
                    if (length != 1U) return false;
                    uint8_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_status(decoded);
                    seen |= uint64_t{1} << 1U;
                    break;
                }
                case 14U: {
                    if ((seen & (uint64_t{1} << 2U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_transfer_id(decoded);
                    seen |= uint64_t{1} << 2U;
                    break;
                }
                default: break;
            }
            offset += length;
        }
        if ((seen & 7ULL) != 7ULL) return false;
        *output = *scratch;
        return true;
    }
};

template <> struct Codec<dp::iface::FileTransferChunkV1Request> {
    using Value = dp::iface::FileTransferChunkV1Request;
    static constexpr size_t kMaxEncodedSize = 27U;
    static constexpr size_t kScratchSize = sizeof(Value);
    static constexpr size_t kAlignment = alignof(Value);
    static constexpr uint32_t kInterfaceId = 4116U;
    static bool encode(const Value& message, uint8_t* output, size_t capacity, size_t* used) {
        if (used == nullptr) return false;
        *used = 0U;
        if (output == nullptr) return false;
        size_t required_bytes = 0U;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 2U)) return false;
        if (required_bytes > kMaxEncodedSize) return false;
        size_t offset = 0U;
        {
            const auto value = message.opcode();
            uint8_t* payload = detail::append(output, capacity, &offset, 1U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.transfer_id();
            uint8_t* payload = detail::append(output, capacity, &offset, 14U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.offset();
            uint8_t* payload = detail::append(output, capacity, &offset, 9U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.length();
            uint8_t* payload = detail::append(output, capacity, &offset, 10U, 2U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        *used = offset;
        return true;
    }
    static bool decode(const uint8_t* input, size_t size, Value* output, Value* scratch) {
        if (input == nullptr || output == nullptr || scratch == nullptr || output == scratch ||
            size > UINT16_MAX) return false;
        *scratch = Value{};
        uint64_t seen = 0U;
        size_t offset = 0U;
        while (offset < size) {
            if (size - offset < 4U) return false;
            const uint16_t tag = detail::get_u16(input + offset);
            const size_t length = detail::get_u16(input + offset + 2U);
            offset += 4U;
            if (tag == 0U || length > size - offset) return false;
            const uint8_t* payload = input + offset;
            switch (tag) {
                case 1U: {
                    if ((seen & (uint64_t{1} << 0U)) != 0U) return false;
                    if (length != 1U) return false;
                    uint8_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_opcode(decoded);
                    seen |= uint64_t{1} << 0U;
                    break;
                }
                case 14U: {
                    if ((seen & (uint64_t{1} << 1U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_transfer_id(decoded);
                    seen |= uint64_t{1} << 1U;
                    break;
                }
                case 9U: {
                    if ((seen & (uint64_t{1} << 2U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_offset(decoded);
                    seen |= uint64_t{1} << 2U;
                    break;
                }
                case 10U: {
                    if ((seen & (uint64_t{1} << 3U)) != 0U) return false;
                    if (length != 2U) return false;
                    uint16_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_length(decoded);
                    seen |= uint64_t{1} << 3U;
                    break;
                }
                default: break;
            }
            offset += length;
        }
        if ((seen & 15ULL) != 15ULL) return false;
        *output = *scratch;
        return true;
    }
};

template <> struct Codec<dp::iface::FileTransferChunkV1Response> {
    using Value = dp::iface::FileTransferChunkV1Response;
    static constexpr size_t kMaxEncodedSize = 100U;
    static constexpr size_t kScratchSize = sizeof(Value);
    static constexpr size_t kAlignment = alignof(Value);
    static constexpr uint32_t kInterfaceId = 4117U;
    static bool encode(const Value& message, uint8_t* output, size_t capacity, size_t* used) {
        if (used == nullptr) return false;
        *used = 0U;
        if (output == nullptr) return false;
        size_t required_bytes = 0U;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 2U)) return false;
        if (!detail::reserve(capacity, &required_bytes, message.data().size() * 1U)) return false;
        if (required_bytes > kMaxEncodedSize) return false;
        size_t offset = 0U;
        {
            const auto value = message.opcode();
            uint8_t* payload = detail::append(output, capacity, &offset, 1U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.status();
            uint8_t* payload = detail::append(output, capacity, &offset, 5U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.transfer_id();
            uint8_t* payload = detail::append(output, capacity, &offset, 14U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.offset();
            uint8_t* payload = detail::append(output, capacity, &offset, 9U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.length();
            uint8_t* payload = detail::append(output, capacity, &offset, 10U, 2U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.data();
            const size_t length = value.size() * 1U;
            uint8_t* payload = detail::append(output, capacity, &offset, 11U, length);
            if (payload == nullptr) return false;
            for (size_t index = 0U; index < value.size(); ++index)
                detail::put_scalar(payload + index * 1U, value.data()[index]);
        }
        *used = offset;
        return true;
    }
    static bool decode(const uint8_t* input, size_t size, Value* output, Value* scratch) {
        if (input == nullptr || output == nullptr || scratch == nullptr || output == scratch ||
            size > UINT16_MAX) return false;
        *scratch = Value{};
        uint64_t seen = 0U;
        size_t offset = 0U;
        while (offset < size) {
            if (size - offset < 4U) return false;
            const uint16_t tag = detail::get_u16(input + offset);
            const size_t length = detail::get_u16(input + offset + 2U);
            offset += 4U;
            if (tag == 0U || length > size - offset) return false;
            const uint8_t* payload = input + offset;
            switch (tag) {
                case 1U: {
                    if ((seen & (uint64_t{1} << 0U)) != 0U) return false;
                    if (length != 1U) return false;
                    uint8_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_opcode(decoded);
                    seen |= uint64_t{1} << 0U;
                    break;
                }
                case 5U: {
                    if ((seen & (uint64_t{1} << 1U)) != 0U) return false;
                    if (length != 1U) return false;
                    uint8_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_status(decoded);
                    seen |= uint64_t{1} << 1U;
                    break;
                }
                case 14U: {
                    if ((seen & (uint64_t{1} << 2U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_transfer_id(decoded);
                    seen |= uint64_t{1} << 2U;
                    break;
                }
                case 9U: {
                    if ((seen & (uint64_t{1} << 3U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_offset(decoded);
                    seen |= uint64_t{1} << 3U;
                    break;
                }
                case 10U: {
                    if ((seen & (uint64_t{1} << 4U)) != 0U) return false;
                    if (length != 2U) return false;
                    uint16_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_length(decoded);
                    seen |= uint64_t{1} << 4U;
                    break;
                }
                case 11U: {
                    if ((seen & (uint64_t{1} << 5U)) != 0U) return false;
                    if (length > 64U || length % 1U != 0U) return false;
                    for (size_t item = 0U; item < length / 1U; ++item)
                        if (!detail::get_scalar(payload + item * 1U, 1U,
                                                &scratch->data_.data_[item])) return false;
                    scratch->data_.size_ = length / 1U;
                    seen |= uint64_t{1} << 5U;
                    break;
                }
                default: break;
            }
            offset += length;
        }
        if ((seen & 63ULL) != 63ULL) return false;
        *output = *scratch;
        return true;
    }
};

template <> struct Codec<dp::iface::FileTransferCloseV1Request> {
    using Value = dp::iface::FileTransferCloseV1Request;
    static constexpr size_t kMaxEncodedSize = 29U;
    static constexpr size_t kScratchSize = sizeof(Value);
    static constexpr size_t kAlignment = alignof(Value);
    static constexpr uint32_t kInterfaceId = 4118U;
    static bool encode(const Value& message, uint8_t* output, size_t capacity, size_t* used) {
        if (used == nullptr) return false;
        *used = 0U;
        if (output == nullptr) return false;
        size_t required_bytes = 0U;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (required_bytes > kMaxEncodedSize) return false;
        size_t offset = 0U;
        {
            const auto value = message.opcode();
            uint8_t* payload = detail::append(output, capacity, &offset, 1U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.transfer_id();
            uint8_t* payload = detail::append(output, capacity, &offset, 14U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.received_crc32();
            uint8_t* payload = detail::append(output, capacity, &offset, 12U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.total_bytes();
            uint8_t* payload = detail::append(output, capacity, &offset, 13U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        *used = offset;
        return true;
    }
    static bool decode(const uint8_t* input, size_t size, Value* output, Value* scratch) {
        if (input == nullptr || output == nullptr || scratch == nullptr || output == scratch ||
            size > UINT16_MAX) return false;
        *scratch = Value{};
        uint64_t seen = 0U;
        size_t offset = 0U;
        while (offset < size) {
            if (size - offset < 4U) return false;
            const uint16_t tag = detail::get_u16(input + offset);
            const size_t length = detail::get_u16(input + offset + 2U);
            offset += 4U;
            if (tag == 0U || length > size - offset) return false;
            const uint8_t* payload = input + offset;
            switch (tag) {
                case 1U: {
                    if ((seen & (uint64_t{1} << 0U)) != 0U) return false;
                    if (length != 1U) return false;
                    uint8_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_opcode(decoded);
                    seen |= uint64_t{1} << 0U;
                    break;
                }
                case 14U: {
                    if ((seen & (uint64_t{1} << 1U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_transfer_id(decoded);
                    seen |= uint64_t{1} << 1U;
                    break;
                }
                case 12U: {
                    if ((seen & (uint64_t{1} << 2U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_received_crc32(decoded);
                    seen |= uint64_t{1} << 2U;
                    break;
                }
                case 13U: {
                    if ((seen & (uint64_t{1} << 3U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_total_bytes(decoded);
                    seen |= uint64_t{1} << 3U;
                    break;
                }
                default: break;
            }
            offset += length;
        }
        if ((seen & 15ULL) != 15ULL) return false;
        *output = *scratch;
        return true;
    }
};

template <> struct Codec<dp::iface::FileTransferCloseV1Response> {
    using Value = dp::iface::FileTransferCloseV1Response;
    static constexpr size_t kMaxEncodedSize = 18U;
    static constexpr size_t kScratchSize = sizeof(Value);
    static constexpr size_t kAlignment = alignof(Value);
    static constexpr uint32_t kInterfaceId = 4119U;
    static bool encode(const Value& message, uint8_t* output, size_t capacity, size_t* used) {
        if (used == nullptr) return false;
        *used = 0U;
        if (output == nullptr) return false;
        size_t required_bytes = 0U;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (required_bytes > kMaxEncodedSize) return false;
        size_t offset = 0U;
        {
            const auto value = message.opcode();
            uint8_t* payload = detail::append(output, capacity, &offset, 1U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.status();
            uint8_t* payload = detail::append(output, capacity, &offset, 5U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.transfer_id();
            uint8_t* payload = detail::append(output, capacity, &offset, 14U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        *used = offset;
        return true;
    }
    static bool decode(const uint8_t* input, size_t size, Value* output, Value* scratch) {
        if (input == nullptr || output == nullptr || scratch == nullptr || output == scratch ||
            size > UINT16_MAX) return false;
        *scratch = Value{};
        uint64_t seen = 0U;
        size_t offset = 0U;
        while (offset < size) {
            if (size - offset < 4U) return false;
            const uint16_t tag = detail::get_u16(input + offset);
            const size_t length = detail::get_u16(input + offset + 2U);
            offset += 4U;
            if (tag == 0U || length > size - offset) return false;
            const uint8_t* payload = input + offset;
            switch (tag) {
                case 1U: {
                    if ((seen & (uint64_t{1} << 0U)) != 0U) return false;
                    if (length != 1U) return false;
                    uint8_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_opcode(decoded);
                    seen |= uint64_t{1} << 0U;
                    break;
                }
                case 5U: {
                    if ((seen & (uint64_t{1} << 1U)) != 0U) return false;
                    if (length != 1U) return false;
                    uint8_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_status(decoded);
                    seen |= uint64_t{1} << 1U;
                    break;
                }
                case 14U: {
                    if ((seen & (uint64_t{1} << 2U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_transfer_id(decoded);
                    seen |= uint64_t{1} << 2U;
                    break;
                }
                default: break;
            }
            offset += length;
        }
        if ((seen & 7ULL) != 7ULL) return false;
        *output = *scratch;
        return true;
    }
};

template <> struct Codec<dp::iface::FileTransferOpenV1Request> {
    using Value = dp::iface::FileTransferOpenV1Request;
    static constexpr size_t kMaxEncodedSize = 87U;
    static constexpr size_t kScratchSize = sizeof(Value);
    static constexpr size_t kAlignment = alignof(Value);
    static constexpr uint32_t kInterfaceId = 4114U;
    static bool encode(const Value& message, uint8_t* output, size_t capacity, size_t* used) {
        if (used == nullptr) return false;
        *used = 0U;
        if (output == nullptr) return false;
        size_t required_bytes = 0U;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, message.filename().size() * 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 2U)) return false;
        if (required_bytes > kMaxEncodedSize) return false;
        size_t offset = 0U;
        {
            const auto value = message.opcode();
            uint8_t* payload = detail::append(output, capacity, &offset, 1U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.filename();
            const size_t length = value.size() * 1U;
            uint8_t* payload = detail::append(output, capacity, &offset, 2U, length);
            if (payload == nullptr) return false;
            for (size_t index = 0U; index < value.size(); ++index)
                detail::put_scalar(payload + index * 1U, value.data()[index]);
        }
        {
            const auto value = message.resume_offset();
            uint8_t* payload = detail::append(output, capacity, &offset, 3U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.max_chunk_size();
            uint8_t* payload = detail::append(output, capacity, &offset, 4U, 2U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        *used = offset;
        return true;
    }
    static bool decode(const uint8_t* input, size_t size, Value* output, Value* scratch) {
        if (input == nullptr || output == nullptr || scratch == nullptr || output == scratch ||
            size > UINT16_MAX) return false;
        *scratch = Value{};
        uint64_t seen = 0U;
        size_t offset = 0U;
        while (offset < size) {
            if (size - offset < 4U) return false;
            const uint16_t tag = detail::get_u16(input + offset);
            const size_t length = detail::get_u16(input + offset + 2U);
            offset += 4U;
            if (tag == 0U || length > size - offset) return false;
            const uint8_t* payload = input + offset;
            switch (tag) {
                case 1U: {
                    if ((seen & (uint64_t{1} << 0U)) != 0U) return false;
                    if (length != 1U) return false;
                    uint8_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_opcode(decoded);
                    seen |= uint64_t{1} << 0U;
                    break;
                }
                case 2U: {
                    if ((seen & (uint64_t{1} << 1U)) != 0U) return false;
                    if (length > 64U || length % 1U != 0U) return false;
                    for (size_t item = 0U; item < length / 1U; ++item)
                        if (!detail::get_scalar(payload + item * 1U, 1U,
                                                &scratch->filename_.data_[item])) return false;
                    scratch->filename_.size_ = length / 1U;
                    seen |= uint64_t{1} << 1U;
                    break;
                }
                case 3U: {
                    if ((seen & (uint64_t{1} << 2U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_resume_offset(decoded);
                    seen |= uint64_t{1} << 2U;
                    break;
                }
                case 4U: {
                    if ((seen & (uint64_t{1} << 3U)) != 0U) return false;
                    if (length != 2U) return false;
                    uint16_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_max_chunk_size(decoded);
                    seen |= uint64_t{1} << 3U;
                    break;
                }
                default: break;
            }
            offset += length;
        }
        if ((seen & 15ULL) != 15ULL) return false;
        *output = *scratch;
        return true;
    }
};

template <> struct Codec<dp::iface::FileTransferOpenV1Response> {
    using Value = dp::iface::FileTransferOpenV1Response;
    static constexpr size_t kMaxEncodedSize = 40U;
    static constexpr size_t kScratchSize = sizeof(Value);
    static constexpr size_t kAlignment = alignof(Value);
    static constexpr uint32_t kInterfaceId = 4115U;
    static bool encode(const Value& message, uint8_t* output, size_t capacity, size_t* used) {
        if (used == nullptr) return false;
        *used = 0U;
        if (output == nullptr) return false;
        size_t required_bytes = 0U;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 2U)) return false;
        if (required_bytes > kMaxEncodedSize) return false;
        size_t offset = 0U;
        {
            const auto value = message.opcode();
            uint8_t* payload = detail::append(output, capacity, &offset, 1U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.status();
            uint8_t* payload = detail::append(output, capacity, &offset, 5U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.transfer_id();
            uint8_t* payload = detail::append(output, capacity, &offset, 14U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.file_size();
            uint8_t* payload = detail::append(output, capacity, &offset, 6U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.file_crc32();
            uint8_t* payload = detail::append(output, capacity, &offset, 7U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.chunk_size();
            uint8_t* payload = detail::append(output, capacity, &offset, 8U, 2U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        *used = offset;
        return true;
    }
    static bool decode(const uint8_t* input, size_t size, Value* output, Value* scratch) {
        if (input == nullptr || output == nullptr || scratch == nullptr || output == scratch ||
            size > UINT16_MAX) return false;
        *scratch = Value{};
        uint64_t seen = 0U;
        size_t offset = 0U;
        while (offset < size) {
            if (size - offset < 4U) return false;
            const uint16_t tag = detail::get_u16(input + offset);
            const size_t length = detail::get_u16(input + offset + 2U);
            offset += 4U;
            if (tag == 0U || length > size - offset) return false;
            const uint8_t* payload = input + offset;
            switch (tag) {
                case 1U: {
                    if ((seen & (uint64_t{1} << 0U)) != 0U) return false;
                    if (length != 1U) return false;
                    uint8_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_opcode(decoded);
                    seen |= uint64_t{1} << 0U;
                    break;
                }
                case 5U: {
                    if ((seen & (uint64_t{1} << 1U)) != 0U) return false;
                    if (length != 1U) return false;
                    uint8_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_status(decoded);
                    seen |= uint64_t{1} << 1U;
                    break;
                }
                case 14U: {
                    if ((seen & (uint64_t{1} << 2U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_transfer_id(decoded);
                    seen |= uint64_t{1} << 2U;
                    break;
                }
                case 6U: {
                    if ((seen & (uint64_t{1} << 3U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_file_size(decoded);
                    seen |= uint64_t{1} << 3U;
                    break;
                }
                case 7U: {
                    if ((seen & (uint64_t{1} << 4U)) != 0U) return false;
                    if (length != 4U) return false;
                    uint32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_file_crc32(decoded);
                    seen |= uint64_t{1} << 4U;
                    break;
                }
                case 8U: {
                    if ((seen & (uint64_t{1} << 5U)) != 0U) return false;
                    if (length != 2U) return false;
                    uint16_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_chunk_size(decoded);
                    seen |= uint64_t{1} << 5U;
                    break;
                }
                default: break;
            }
            offset += length;
        }
        if ((seen & 63ULL) != 63ULL) return false;
        *output = *scratch;
        return true;
    }
};

template <> struct Codec<dp::iface::OfflineLogCommandRpcV1Request> {
    using Value = dp::iface::OfflineLogCommandRpcV1Request;
    static constexpr size_t kMaxEncodedSize = 324U;
    static constexpr size_t kScratchSize = sizeof(Value);
    static constexpr size_t kAlignment = alignof(Value);
    static constexpr uint32_t kInterfaceId = 4125U;
    static bool encode(const Value& message, uint8_t* output, size_t capacity, size_t* used) {
        if (used == nullptr) return false;
        *used = 0U;
        if (output == nullptr) return false;
        size_t required_bytes = 0U;
        if (!detail::reserve(capacity, &required_bytes, message.command().size())) return false;
        if (required_bytes > kMaxEncodedSize) return false;
        size_t offset = 0U;
        {
            const auto value = message.command();
            const size_t length = value.size();
            uint8_t* payload = detail::append(output, capacity, &offset, 1U, length);
            if (payload == nullptr) return false;
            if (length != 0U) std::memcpy(payload, value.data(), length);
        }
        *used = offset;
        return true;
    }
    static bool decode(const uint8_t* input, size_t size, Value* output, Value* scratch) {
        if (input == nullptr || output == nullptr || scratch == nullptr || output == scratch ||
            size > UINT16_MAX) return false;
        *scratch = Value{};
        uint64_t seen = 0U;
        size_t offset = 0U;
        while (offset < size) {
            if (size - offset < 4U) return false;
            const uint16_t tag = detail::get_u16(input + offset);
            const size_t length = detail::get_u16(input + offset + 2U);
            offset += 4U;
            if (tag == 0U || length > size - offset) return false;
            const uint8_t* payload = input + offset;
            switch (tag) {
                case 1U: {
                    if ((seen & (uint64_t{1} << 0U)) != 0U) return false;
                    if (length > 320U || scratch->set_command(
                            {reinterpret_cast<const char*>(payload), length}) != SetResult::kOk) return false;
                    seen |= uint64_t{1} << 0U;
                    break;
                }
                default: break;
            }
            offset += length;
        }
        if ((seen & 1ULL) != 1ULL) return false;
        *output = *scratch;
        return true;
    }
};

template <> struct Codec<dp::iface::OfflineLogCommandRpcV1Response> {
    using Value = dp::iface::OfflineLogCommandRpcV1Response;
    static constexpr size_t kMaxEncodedSize = 273U;
    static constexpr size_t kScratchSize = sizeof(Value);
    static constexpr size_t kAlignment = alignof(Value);
    static constexpr uint32_t kInterfaceId = 4126U;
    static bool encode(const Value& message, uint8_t* output, size_t capacity, size_t* used) {
        if (used == nullptr) return false;
        *used = 0U;
        if (output == nullptr) return false;
        size_t required_bytes = 0U;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, message.detail().size())) return false;
        if (required_bytes > kMaxEncodedSize) return false;
        size_t offset = 0U;
        {
            const auto value = message.ok();
            uint8_t* payload = detail::append(output, capacity, &offset, 2U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.status();
            uint8_t* payload = detail::append(output, capacity, &offset, 3U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.detail();
            const size_t length = value.size();
            uint8_t* payload = detail::append(output, capacity, &offset, 4U, length);
            if (payload == nullptr) return false;
            if (length != 0U) std::memcpy(payload, value.data(), length);
        }
        *used = offset;
        return true;
    }
    static bool decode(const uint8_t* input, size_t size, Value* output, Value* scratch) {
        if (input == nullptr || output == nullptr || scratch == nullptr || output == scratch ||
            size > UINT16_MAX) return false;
        *scratch = Value{};
        uint64_t seen = 0U;
        size_t offset = 0U;
        while (offset < size) {
            if (size - offset < 4U) return false;
            const uint16_t tag = detail::get_u16(input + offset);
            const size_t length = detail::get_u16(input + offset + 2U);
            offset += 4U;
            if (tag == 0U || length > size - offset) return false;
            const uint8_t* payload = input + offset;
            switch (tag) {
                case 2U: {
                    if ((seen & (uint64_t{1} << 0U)) != 0U) return false;
                    if (length != 1U) return false;
                    bool decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_ok(decoded);
                    seen |= uint64_t{1} << 0U;
                    break;
                }
                case 3U: {
                    if ((seen & (uint64_t{1} << 1U)) != 0U) return false;
                    if (length != 4U) return false;
                    int32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_status(decoded);
                    seen |= uint64_t{1} << 1U;
                    break;
                }
                case 4U: {
                    if ((seen & (uint64_t{1} << 2U)) != 0U) return false;
                    if (length > 256U || scratch->set_detail(
                            {reinterpret_cast<const char*>(payload), length}) != SetResult::kOk) return false;
                    seen |= uint64_t{1} << 2U;
                    break;
                }
                default: break;
            }
            offset += length;
        }
        if ((seen & 0ULL) != 0ULL) return false;
        *output = *scratch;
        return true;
    }
};

template <> struct Codec<dp::iface::OfflineLogCommandV2Request> {
    using Value = dp::iface::OfflineLogCommandV2Request;
    static constexpr size_t kMaxEncodedSize = 516U;
    static constexpr size_t kScratchSize = sizeof(Value);
    static constexpr size_t kAlignment = alignof(Value);
    static constexpr uint32_t kInterfaceId = 4128U;
    static bool encode(const Value& message, uint8_t* output, size_t capacity, size_t* used) {
        if (used == nullptr) return false;
        *used = 0U;
        if (output == nullptr) return false;
        size_t required_bytes = 0U;
        if (!detail::reserve(capacity, &required_bytes, message.request_id().size())) return false;
        if (!detail::reserve(capacity, &required_bytes, message.command().size())) return false;
        if (!detail::reserve(capacity, &required_bytes, message.name().size())) return false;
        if (required_bytes > kMaxEncodedSize) return false;
        size_t offset = 0U;
        {
            const auto value = message.request_id();
            const size_t length = value.size();
            uint8_t* payload = detail::append(output, capacity, &offset, 1U, length);
            if (payload == nullptr) return false;
            if (length != 0U) std::memcpy(payload, value.data(), length);
        }
        {
            const auto value = message.command();
            const size_t length = value.size();
            uint8_t* payload = detail::append(output, capacity, &offset, 2U, length);
            if (payload == nullptr) return false;
            if (length != 0U) std::memcpy(payload, value.data(), length);
        }
        {
            const auto value = message.name();
            const size_t length = value.size();
            uint8_t* payload = detail::append(output, capacity, &offset, 3U, length);
            if (payload == nullptr) return false;
            if (length != 0U) std::memcpy(payload, value.data(), length);
        }
        *used = offset;
        return true;
    }
    static bool decode(const uint8_t* input, size_t size, Value* output, Value* scratch) {
        if (input == nullptr || output == nullptr || scratch == nullptr || output == scratch ||
            size > UINT16_MAX) return false;
        *scratch = Value{};
        uint64_t seen = 0U;
        size_t offset = 0U;
        while (offset < size) {
            if (size - offset < 4U) return false;
            const uint16_t tag = detail::get_u16(input + offset);
            const size_t length = detail::get_u16(input + offset + 2U);
            offset += 4U;
            if (tag == 0U || length > size - offset) return false;
            const uint8_t* payload = input + offset;
            switch (tag) {
                case 1U: {
                    if ((seen & (uint64_t{1} << 0U)) != 0U) return false;
                    if (length > 64U || scratch->set_request_id(
                            {reinterpret_cast<const char*>(payload), length}) != SetResult::kOk) return false;
                    seen |= uint64_t{1} << 0U;
                    break;
                }
                case 2U: {
                    if ((seen & (uint64_t{1} << 1U)) != 0U) return false;
                    if (length > 320U || scratch->set_command(
                            {reinterpret_cast<const char*>(payload), length}) != SetResult::kOk) return false;
                    seen |= uint64_t{1} << 1U;
                    break;
                }
                case 3U: {
                    if ((seen & (uint64_t{1} << 2U)) != 0U) return false;
                    if (length > 120U || scratch->set_name(
                            {reinterpret_cast<const char*>(payload), length}) != SetResult::kOk) return false;
                    seen |= uint64_t{1} << 2U;
                    break;
                }
                default: break;
            }
            offset += length;
        }
        if ((seen & 3ULL) != 3ULL) return false;
        *output = *scratch;
        return true;
    }
};

template <> struct Codec<dp::iface::OfflineLogCommandV2Response> {
    using Value = dp::iface::OfflineLogCommandV2Response;
    static constexpr size_t kMaxEncodedSize = 665U;
    static constexpr size_t kScratchSize = sizeof(Value);
    static constexpr size_t kAlignment = alignof(Value);
    static constexpr uint32_t kInterfaceId = 4129U;
    static bool encode(const Value& message, uint8_t* output, size_t capacity, size_t* used) {
        if (used == nullptr) return false;
        *used = 0U;
        if (output == nullptr) return false;
        size_t required_bytes = 0U;
        if (!detail::reserve(capacity, &required_bytes, message.request_id().size())) return false;
        if (!detail::reserve(capacity, &required_bytes, message.command().size())) return false;
        if (!detail::reserve(capacity, &required_bytes, 1U)) return false;
        if (!detail::reserve(capacity, &required_bytes, 4U)) return false;
        if (!detail::reserve(capacity, &required_bytes, message.detail().size())) return false;
        if (required_bytes > kMaxEncodedSize) return false;
        size_t offset = 0U;
        {
            const auto value = message.request_id();
            const size_t length = value.size();
            uint8_t* payload = detail::append(output, capacity, &offset, 1U, length);
            if (payload == nullptr) return false;
            if (length != 0U) std::memcpy(payload, value.data(), length);
        }
        {
            const auto value = message.command();
            const size_t length = value.size();
            uint8_t* payload = detail::append(output, capacity, &offset, 2U, length);
            if (payload == nullptr) return false;
            if (length != 0U) std::memcpy(payload, value.data(), length);
        }
        {
            const auto value = message.ok();
            uint8_t* payload = detail::append(output, capacity, &offset, 3U, 1U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.status();
            uint8_t* payload = detail::append(output, capacity, &offset, 4U, 4U);
            if (payload == nullptr) return false;
            detail::put_scalar(payload, value);
        }
        {
            const auto value = message.detail();
            const size_t length = value.size();
            uint8_t* payload = detail::append(output, capacity, &offset, 5U, length);
            if (payload == nullptr) return false;
            if (length != 0U) std::memcpy(payload, value.data(), length);
        }
        *used = offset;
        return true;
    }
    static bool decode(const uint8_t* input, size_t size, Value* output, Value* scratch) {
        if (input == nullptr || output == nullptr || scratch == nullptr || output == scratch ||
            size > UINT16_MAX) return false;
        *scratch = Value{};
        uint64_t seen = 0U;
        size_t offset = 0U;
        while (offset < size) {
            if (size - offset < 4U) return false;
            const uint16_t tag = detail::get_u16(input + offset);
            const size_t length = detail::get_u16(input + offset + 2U);
            offset += 4U;
            if (tag == 0U || length > size - offset) return false;
            const uint8_t* payload = input + offset;
            switch (tag) {
                case 1U: {
                    if ((seen & (uint64_t{1} << 0U)) != 0U) return false;
                    if (length > 64U || scratch->set_request_id(
                            {reinterpret_cast<const char*>(payload), length}) != SetResult::kOk) return false;
                    seen |= uint64_t{1} << 0U;
                    break;
                }
                case 2U: {
                    if ((seen & (uint64_t{1} << 1U)) != 0U) return false;
                    if (length > 320U || scratch->set_command(
                            {reinterpret_cast<const char*>(payload), length}) != SetResult::kOk) return false;
                    seen |= uint64_t{1} << 1U;
                    break;
                }
                case 3U: {
                    if ((seen & (uint64_t{1} << 2U)) != 0U) return false;
                    if (length != 1U) return false;
                    bool decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_ok(decoded);
                    seen |= uint64_t{1} << 2U;
                    break;
                }
                case 4U: {
                    if ((seen & (uint64_t{1} << 3U)) != 0U) return false;
                    if (length != 4U) return false;
                    int32_t decoded{};
                    if (!detail::get_scalar(payload, length, &decoded)) return false;
                    (void)scratch->set_status(decoded);
                    seen |= uint64_t{1} << 3U;
                    break;
                }
                case 5U: {
                    if ((seen & (uint64_t{1} << 4U)) != 0U) return false;
                    if (length > 256U || scratch->set_detail(
                            {reinterpret_cast<const char*>(payload), length}) != SetResult::kOk) return false;
                    seen |= uint64_t{1} << 4U;
                    break;
                }
                default: break;
            }
            offset += length;
        }
        if ((seen & 7ULL) != 7ULL) return false;
        *output = *scratch;
        return true;
    }
};

}  // namespace dp::iface::tlv
