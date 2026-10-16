// Auto-generated from ROS IDL and deployment metadata. Do not edit.
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>

namespace dp::protocol::zenbedded {
enum class CodecResult : uint8_t { kOk = 0, kInvalidArgument };
enum class CommandStalePolicy : uint8_t { kHoldLast = 0, kSafeStop, kDisableOutput };
inline constexpr uint32_t kStateInterfaceId = 4097U;
inline constexpr uint32_t kWatchdogInterfaceId = 4104U;
inline constexpr uint32_t kCommandInterfaceId = 4098U;
inline constexpr const char* kProductName = "test_robot";
inline constexpr const char* kNodeName = "zenbedded_control";
inline constexpr const char* kStateEndpoint = "state";
inline constexpr const char* kCommandEndpoint = "command";
inline constexpr uint32_t kControlRateHz = 100U;
inline constexpr uint32_t kControlPeriodMs = 10U;
inline constexpr uint32_t kMaxCommandAgeMs = 100U;
inline constexpr uint32_t kMaxStateAgeMs = 50U;
inline constexpr int kControlThreadPriority = 0;
inline constexpr CommandStalePolicy kCommandStalePolicy = CommandStalePolicy::kSafeStop;
inline constexpr const char* kWatchdogEndpoint = "watchdog";
static_assert(sizeof(float) == 4U && std::numeric_limits<float>::is_iec559);
static_assert(sizeof(double) == 8U && std::numeric_limits<double>::is_iec559);
namespace detail {
inline bool append_text(char* output, size_t capacity, size_t* used, const char* text) {
    if (output == nullptr || used == nullptr || text == nullptr) return false;
    while (*text != '\0') {
        if (*used + 1U >= capacity) return false;
        output[(*used)++] = *text++;
    }
    output[*used] = '\0';
    return true;
}
inline void put_le8(uint8_t value, uint8_t* output) {
    for (size_t i = 0U; i < 1U; ++i) output[i] = static_cast<uint8_t>(value >> (i * 8U));
}
inline uint8_t get_le8(const uint8_t* input) {
    uint8_t value = 0U;
    for (size_t i = 0U; i < 1U; ++i) value |= static_cast<uint8_t>(input[i]) << (i * 8U);
    return value;
}
inline void put_le32(uint32_t value, uint8_t* output) {
    for (size_t i = 0U; i < 4U; ++i) output[i] = static_cast<uint8_t>(value >> (i * 8U));
}
inline uint32_t get_le32(const uint8_t* input) {
    uint32_t value = 0U;
    for (size_t i = 0U; i < 4U; ++i) value |= static_cast<uint32_t>(input[i]) << (i * 8U);
    return value;
}
inline void put_le64(uint64_t value, uint8_t* output) {
    for (size_t i = 0U; i < 8U; ++i) output[i] = static_cast<uint8_t>(value >> (i * 8U));
}
inline uint64_t get_le64(const uint8_t* input) {
    uint64_t value = 0U;
    for (size_t i = 0U; i < 8U; ++i) value |= static_cast<uint64_t>(input[i]) << (i * 8U);
    return value;
}
}  // namespace detail
inline bool format_topic(const char* device_id, const char* endpoint, char* output, size_t capacity) {
    if (device_id == nullptr || endpoint == nullptr || output == nullptr || capacity == 0U) return false;
    bool nonzero = false;
    for (size_t index = 0U; index < 32U; ++index) {
        const char value = device_id[index];
        if (!((value >= '0' && value <= '9') || (value >= 'a' && value <= 'f'))) return false;
        nonzero = nonzero || value != '0';
    }
    if (device_id[32] != '\0' || !nonzero) return false;
    size_t used = 0U;
    output[0] = '\0';
    return detail::append_text(output, capacity, &used, "dpsdk/") &&
           detail::append_text(output, capacity, &used, kProductName) &&
           detail::append_text(output, capacity, &used, "/") &&
           detail::append_text(output, capacity, &used, device_id) &&
           detail::append_text(output, capacity, &used, "/") &&
           detail::append_text(output, capacity, &used, kNodeName) &&
           detail::append_text(output, capacity, &used, "/") &&
           detail::append_text(output, capacity, &used, endpoint);
}

class JointStateFrameV1 {
public:
    double motor_position() const { return motor_position_; }
    void set_motor_position(double value) { motor_position_ = value; }
    double pendulum_position() const { return pendulum_position_; }
    void set_pendulum_position(double value) { pendulum_position_ = value; }

private:
    double motor_position_{};
    double pendulum_position_{};
};

class JointCommandFrameV1 {
public:
    double motor_position() const { return motor_position_; }
    void set_motor_position(double value) { motor_position_ = value; }

private:
    double motor_position_{};
};

class ControlWatchdogV1 {
public:
    uint32_t timestamp_ms() const { return timestamp_ms_; }
    void set_timestamp_ms(uint32_t value) { timestamp_ms_ = value; }
    uint32_t deadline_miss_count() const { return deadline_miss_count_; }
    void set_deadline_miss_count(uint32_t value) { deadline_miss_count_ = value; }
    uint32_t max_jitter_ms() const { return max_jitter_ms_; }
    void set_max_jitter_ms(uint32_t value) { max_jitter_ms_ = value; }
    bool command_fresh() const { return command_fresh_; }
    void set_command_fresh(bool value) { command_fresh_ = value; }
    bool state_fresh() const { return state_fresh_; }
    void set_state_fresh(bool value) { state_fresh_ = value; }

private:
    uint32_t timestamp_ms_{};
    uint32_t deadline_miss_count_{};
    uint32_t max_jitter_ms_{};
    bool command_fresh_{};
    bool state_fresh_{};
};

class JointStateFrameV1Codec {
public:
    static constexpr size_t kEncodedSize = 16U;
    static constexpr size_t kScratchSize = 0U;
    static constexpr size_t kAlignment = alignof(JointStateFrameV1);
    static constexpr uint32_t kMaxAge = 50U;
    static CodecResult encode(const JointStateFrameV1& message, uint8_t* output, size_t size) {
        if (output == nullptr || size != kEncodedSize) return CodecResult::kInvalidArgument;
        size_t offset = 0U;
        double value_motor_position = message.motor_position();
        uint64_t bits_motor_position = 0U;
        std::memcpy(&bits_motor_position, &value_motor_position, 8U);
        detail::put_le64(bits_motor_position, output + offset);
        offset += 8U;
        double value_pendulum_position = message.pendulum_position();
        uint64_t bits_pendulum_position = 0U;
        std::memcpy(&bits_pendulum_position, &value_pendulum_position, 8U);
        detail::put_le64(bits_pendulum_position, output + offset);
        offset += 8U;
        return CodecResult::kOk;
    }
    static CodecResult decode(const uint8_t* input, size_t size, JointStateFrameV1* message) {
        if (input == nullptr || message == nullptr || size != kEncodedSize) return CodecResult::kInvalidArgument;
        size_t offset = 0U;
        uint64_t bits_motor_position = detail::get_le64(input + offset);
        double value_motor_position{};
        std::memcpy(&value_motor_position, &bits_motor_position, 8U);
        message->set_motor_position(value_motor_position);
        offset += 8U;
        uint64_t bits_pendulum_position = detail::get_le64(input + offset);
        double value_pendulum_position{};
        std::memcpy(&value_pendulum_position, &bits_pendulum_position, 8U);
        message->set_pendulum_position(value_pendulum_position);
        offset += 8U;
        return CodecResult::kOk;
    }
};

class JointCommandFrameV1Codec {
public:
    static constexpr size_t kEncodedSize = 8U;
    static constexpr size_t kScratchSize = 0U;
    static constexpr size_t kAlignment = alignof(JointCommandFrameV1);
    static constexpr uint32_t kMaxAge = 100U;
    static CodecResult encode(const JointCommandFrameV1& message, uint8_t* output, size_t size) {
        if (output == nullptr || size != kEncodedSize) return CodecResult::kInvalidArgument;
        size_t offset = 0U;
        double value_motor_position = message.motor_position();
        uint64_t bits_motor_position = 0U;
        std::memcpy(&bits_motor_position, &value_motor_position, 8U);
        detail::put_le64(bits_motor_position, output + offset);
        offset += 8U;
        return CodecResult::kOk;
    }
    static CodecResult decode(const uint8_t* input, size_t size, JointCommandFrameV1* message) {
        if (input == nullptr || message == nullptr || size != kEncodedSize) return CodecResult::kInvalidArgument;
        size_t offset = 0U;
        uint64_t bits_motor_position = detail::get_le64(input + offset);
        double value_motor_position{};
        std::memcpy(&value_motor_position, &bits_motor_position, 8U);
        message->set_motor_position(value_motor_position);
        offset += 8U;
        return CodecResult::kOk;
    }
};

class ControlWatchdogV1Codec {
public:
    static constexpr size_t kEncodedSize = 14U;
    static constexpr size_t kScratchSize = 0U;
    static constexpr size_t kAlignment = alignof(ControlWatchdogV1);
    static constexpr uint32_t kMaxAge = 50U;
    static CodecResult encode(const ControlWatchdogV1& message, uint8_t* output, size_t size) {
        if (output == nullptr || size != kEncodedSize) return CodecResult::kInvalidArgument;
        size_t offset = 0U;
        uint32_t value_timestamp_ms = message.timestamp_ms();
        uint32_t bits_timestamp_ms = 0U;
        std::memcpy(&bits_timestamp_ms, &value_timestamp_ms, 4U);
        detail::put_le32(bits_timestamp_ms, output + offset);
        offset += 4U;
        uint32_t value_deadline_miss_count = message.deadline_miss_count();
        uint32_t bits_deadline_miss_count = 0U;
        std::memcpy(&bits_deadline_miss_count, &value_deadline_miss_count, 4U);
        detail::put_le32(bits_deadline_miss_count, output + offset);
        offset += 4U;
        uint32_t value_max_jitter_ms = message.max_jitter_ms();
        uint32_t bits_max_jitter_ms = 0U;
        std::memcpy(&bits_max_jitter_ms, &value_max_jitter_ms, 4U);
        detail::put_le32(bits_max_jitter_ms, output + offset);
        offset += 4U;
        bool value_command_fresh = message.command_fresh();
        uint8_t bits_command_fresh = 0U;
        std::memcpy(&bits_command_fresh, &value_command_fresh, 1U);
        detail::put_le8(bits_command_fresh, output + offset);
        offset += 1U;
        bool value_state_fresh = message.state_fresh();
        uint8_t bits_state_fresh = 0U;
        std::memcpy(&bits_state_fresh, &value_state_fresh, 1U);
        detail::put_le8(bits_state_fresh, output + offset);
        offset += 1U;
        return CodecResult::kOk;
    }
    static CodecResult decode(const uint8_t* input, size_t size, ControlWatchdogV1* message) {
        if (input == nullptr || message == nullptr || size != kEncodedSize) return CodecResult::kInvalidArgument;
        size_t offset = 0U;
        uint32_t bits_timestamp_ms = detail::get_le32(input + offset);
        uint32_t value_timestamp_ms{};
        std::memcpy(&value_timestamp_ms, &bits_timestamp_ms, 4U);
        message->set_timestamp_ms(value_timestamp_ms);
        offset += 4U;
        uint32_t bits_deadline_miss_count = detail::get_le32(input + offset);
        uint32_t value_deadline_miss_count{};
        std::memcpy(&value_deadline_miss_count, &bits_deadline_miss_count, 4U);
        message->set_deadline_miss_count(value_deadline_miss_count);
        offset += 4U;
        uint32_t bits_max_jitter_ms = detail::get_le32(input + offset);
        uint32_t value_max_jitter_ms{};
        std::memcpy(&value_max_jitter_ms, &bits_max_jitter_ms, 4U);
        message->set_max_jitter_ms(value_max_jitter_ms);
        offset += 4U;
        uint8_t bits_command_fresh = detail::get_le8(input + offset);
        if (bits_command_fresh > 1U) return CodecResult::kInvalidArgument;
        message->set_command_fresh(bits_command_fresh != 0U);
        offset += 1U;
        uint8_t bits_state_fresh = detail::get_le8(input + offset);
        if (bits_state_fresh > 1U) return CodecResult::kInvalidArgument;
        message->set_state_fresh(bits_state_fresh != 0U);
        offset += 1U;
        return CodecResult::kOk;
    }
};

}  // namespace dp::protocol::zenbedded
