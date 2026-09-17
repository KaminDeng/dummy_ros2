// Auto-generated Dummy raw_fixed profile. Do not edit.
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>

namespace dp::protocol::dummy_robot_v2 {
enum class CodecResult : uint8_t { kOk = 0, kInvalidArgument };
namespace detail {
inline void put_le8(uint8_t value, uint8_t* output) {
    for (size_t i = 0U; i < 1U; ++i) output[i] = static_cast<uint8_t>(value >> (i * 8U));
}
inline uint8_t get_le8(const uint8_t* input) {
    uint8_t value = 0U;
    for (size_t i = 0U; i < 1U; ++i) value |= static_cast<uint8_t>(input[i]) << (i * 8U);
    return value;
}
inline void put_le16(uint16_t value, uint8_t* output) {
    for (size_t i = 0U; i < 2U; ++i) output[i] = static_cast<uint8_t>(value >> (i * 8U));
}
inline uint16_t get_le16(const uint8_t* input) {
    uint16_t value = 0U;
    for (size_t i = 0U; i < 2U; ++i) value |= static_cast<uint16_t>(input[i]) << (i * 8U);
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

class DummyDeviceStatusV2 {
public:
    uint16_t schema_major() const { return schema_major_; }
    void set_schema_major(uint16_t value) { schema_major_ = value; }
    uint16_t schema_minor() const { return schema_minor_; }
    void set_schema_minor(uint16_t value) { schema_minor_ = value; }
    const uint8_t (&device_id() const)[16] { return device_id_; }
    void set_device_id(const uint8_t (&value)[16]) { std::memcpy(device_id_, value, sizeof(device_id_)); }
    uint64_t boot_epoch() const { return boot_epoch_; }
    void set_boot_epoch(uint64_t value) { boot_epoch_ = value; }
    uint64_t session_id() const { return session_id_; }
    void set_session_id(uint64_t value) { session_id_ = value; }
    uint32_t reference_epoch() const { return reference_epoch_; }
    void set_reference_epoch(uint32_t value) { reference_epoch_ = value; }
    uint32_t motion_epoch() const { return motion_epoch_; }
    void set_motion_epoch(uint32_t value) { motion_epoch_ = value; }
    uint32_t management_epoch() const { return management_epoch_; }
    void set_management_epoch(uint32_t value) { management_epoch_ = value; }
    uint32_t profile_id() const { return profile_id_; }
    void set_profile_id(uint32_t value) { profile_id_ = value; }
    uint16_t core_state() const { return core_state_; }
    void set_core_state(uint16_t value) { core_state_ = value; }
    uint8_t control_mode() const { return control_mode_; }
    void set_control_mode(uint8_t value) { control_mode_ = value; }
    uint8_t reference_kind() const { return reference_kind_; }
    void set_reference_kind(uint8_t value) { reference_kind_ = value; }
    uint8_t configured_mask() const { return configured_mask_; }
    void set_configured_mask(uint8_t value) { configured_mask_ = value; }
    bool motion_supported() const { return motion_supported_; }
    void set_motion_supported(bool value) { motion_supported_ = value; }
    bool real_motion_permitted() const { return real_motion_permitted_; }
    void set_real_motion_permitted(bool value) { real_motion_permitted_ = value; }
    uint64_t permit_remaining_us() const { return permit_remaining_us_; }
    void set_permit_remaining_us(uint64_t value) { permit_remaining_us_ = value; }
    uint64_t capabilities() const { return capabilities_; }
    void set_capabilities(uint64_t value) { capabilities_ = value; }
    uint64_t operation_id() const { return operation_id_; }
    void set_operation_id(uint64_t value) { operation_id_ = value; }
    uint8_t transaction_phase() const { return transaction_phase_; }
    void set_transaction_phase(uint8_t value) { transaction_phase_ = value; }
    uint16_t first_failure_reason() const { return first_failure_reason_; }
    void set_first_failure_reason(uint16_t value) { first_failure_reason_ = value; }
    uint64_t operation_deadline_us() const { return operation_deadline_us_; }
    void set_operation_deadline_us(uint64_t value) { operation_deadline_us_ = value; }
    uint32_t committed_producer_seq() const { return committed_producer_seq_; }
    void set_committed_producer_seq(uint32_t value) { committed_producer_seq_ = value; }
    uint8_t can_submitted_mask() const { return can_submitted_mask_; }
    void set_can_submitted_mask(uint8_t value) { can_submitted_mask_ = value; }
    uint8_t can_sent_mask() const { return can_sent_mask_; }
    void set_can_sent_mask(uint8_t value) { can_sent_mask_ = value; }
    uint8_t can_failed_mask() const { return can_failed_mask_; }
    void set_can_failed_mask(uint8_t value) { can_failed_mask_ = value; }
    uint32_t command_reject_count() const { return command_reject_count_; }
    void set_command_reject_count(uint32_t value) { command_reject_count_ = value; }
    uint32_t can_error_count() const { return can_error_count_; }
    void set_can_error_count(uint32_t value) { can_error_count_ = value; }
    uint32_t deadline_miss_count() const { return deadline_miss_count_; }
    void set_deadline_miss_count(uint32_t value) { deadline_miss_count_ = value; }
    const uint8_t (&contract_sha256() const)[32] { return contract_sha256_; }
    void set_contract_sha256(const uint8_t (&value)[32]) { std::memcpy(contract_sha256_, value, sizeof(contract_sha256_)); }
    const uint8_t (&model_sha256() const)[32] { return model_sha256_; }
    void set_model_sha256(const uint8_t (&value)[32]) { std::memcpy(model_sha256_, value, sizeof(model_sha256_)); }
    const uint8_t (&profile_sha256() const)[32] { return profile_sha256_; }
    void set_profile_sha256(const uint8_t (&value)[32]) { std::memcpy(profile_sha256_, value, sizeof(profile_sha256_)); }
    const uint8_t (&firmware_sha256() const)[32] { return firmware_sha256_; }
    void set_firmware_sha256(const uint8_t (&value)[32]) { std::memcpy(firmware_sha256_, value, sizeof(firmware_sha256_)); }
    const uint8_t (&board_name_ascii() const)[16] { return board_name_ascii_; }
    void set_board_name_ascii(const uint8_t (&value)[16]) { std::memcpy(board_name_ascii_, value, sizeof(board_name_ascii_)); }

private:
    uint16_t schema_major_{};
    uint16_t schema_minor_{};
    uint8_t device_id_[16]{};
    uint64_t boot_epoch_{};
    uint64_t session_id_{};
    uint32_t reference_epoch_{};
    uint32_t motion_epoch_{};
    uint32_t management_epoch_{};
    uint32_t profile_id_{};
    uint16_t core_state_{};
    uint8_t control_mode_{};
    uint8_t reference_kind_{};
    uint8_t configured_mask_{};
    bool motion_supported_{};
    bool real_motion_permitted_{};
    uint64_t permit_remaining_us_{};
    uint64_t capabilities_{};
    uint64_t operation_id_{};
    uint8_t transaction_phase_{};
    uint16_t first_failure_reason_{};
    uint64_t operation_deadline_us_{};
    uint32_t committed_producer_seq_{};
    uint8_t can_submitted_mask_{};
    uint8_t can_sent_mask_{};
    uint8_t can_failed_mask_{};
    uint32_t command_reject_count_{};
    uint32_t can_error_count_{};
    uint32_t deadline_miss_count_{};
    uint8_t contract_sha256_[32]{};
    uint8_t model_sha256_[32]{};
    uint8_t profile_sha256_[32]{};
    uint8_t firmware_sha256_[32]{};
    uint8_t board_name_ascii_[16]{};
};

class DummyDeviceStatusV2Codec {
public:
    static constexpr size_t kEncodedSize = 257U;
    static constexpr size_t kScratchSize = 0U;
    static constexpr size_t kAlignment = alignof(DummyDeviceStatusV2);
    static constexpr uint32_t kMaxAge = 0U;
    static CodecResult encode(const DummyDeviceStatusV2& message, uint8_t* output, size_t size) {
        if (output == nullptr || size != kEncodedSize) return CodecResult::kInvalidArgument;
        size_t offset = 0U;
        uint16_t value_schema_major = message.schema_major();
        uint16_t bits_schema_major = 0U;
        std::memcpy(&bits_schema_major, &value_schema_major, 2U);
        detail::put_le16(bits_schema_major, output + offset);
        offset += 2U;
        uint16_t value_schema_minor = message.schema_minor();
        uint16_t bits_schema_minor = 0U;
        std::memcpy(&bits_schema_minor, &value_schema_minor, 2U);
        detail::put_le16(bits_schema_minor, output + offset);
        offset += 2U;
        for (size_t index = 0U; index < 16U; ++index) {
            uint8_t value = message.device_id()[index];
            uint8_t bits = 0U;
            std::memcpy(&bits, &value, 1U);
            detail::put_le8(bits, output + offset);
            offset += 1U;
        }
        uint64_t value_boot_epoch = message.boot_epoch();
        uint64_t bits_boot_epoch = 0U;
        std::memcpy(&bits_boot_epoch, &value_boot_epoch, 8U);
        detail::put_le64(bits_boot_epoch, output + offset);
        offset += 8U;
        uint64_t value_session_id = message.session_id();
        uint64_t bits_session_id = 0U;
        std::memcpy(&bits_session_id, &value_session_id, 8U);
        detail::put_le64(bits_session_id, output + offset);
        offset += 8U;
        uint32_t value_reference_epoch = message.reference_epoch();
        uint32_t bits_reference_epoch = 0U;
        std::memcpy(&bits_reference_epoch, &value_reference_epoch, 4U);
        detail::put_le32(bits_reference_epoch, output + offset);
        offset += 4U;
        uint32_t value_motion_epoch = message.motion_epoch();
        uint32_t bits_motion_epoch = 0U;
        std::memcpy(&bits_motion_epoch, &value_motion_epoch, 4U);
        detail::put_le32(bits_motion_epoch, output + offset);
        offset += 4U;
        uint32_t value_management_epoch = message.management_epoch();
        uint32_t bits_management_epoch = 0U;
        std::memcpy(&bits_management_epoch, &value_management_epoch, 4U);
        detail::put_le32(bits_management_epoch, output + offset);
        offset += 4U;
        uint32_t value_profile_id = message.profile_id();
        uint32_t bits_profile_id = 0U;
        std::memcpy(&bits_profile_id, &value_profile_id, 4U);
        detail::put_le32(bits_profile_id, output + offset);
        offset += 4U;
        uint16_t value_core_state = message.core_state();
        uint16_t bits_core_state = 0U;
        std::memcpy(&bits_core_state, &value_core_state, 2U);
        detail::put_le16(bits_core_state, output + offset);
        offset += 2U;
        uint8_t value_control_mode = message.control_mode();
        uint8_t bits_control_mode = 0U;
        std::memcpy(&bits_control_mode, &value_control_mode, 1U);
        detail::put_le8(bits_control_mode, output + offset);
        offset += 1U;
        uint8_t value_reference_kind = message.reference_kind();
        uint8_t bits_reference_kind = 0U;
        std::memcpy(&bits_reference_kind, &value_reference_kind, 1U);
        detail::put_le8(bits_reference_kind, output + offset);
        offset += 1U;
        uint8_t value_configured_mask = message.configured_mask();
        uint8_t bits_configured_mask = 0U;
        std::memcpy(&bits_configured_mask, &value_configured_mask, 1U);
        detail::put_le8(bits_configured_mask, output + offset);
        offset += 1U;
        bool value_motion_supported = message.motion_supported();
        uint8_t bits_motion_supported = 0U;
        std::memcpy(&bits_motion_supported, &value_motion_supported, 1U);
        detail::put_le8(bits_motion_supported, output + offset);
        offset += 1U;
        bool value_real_motion_permitted = message.real_motion_permitted();
        uint8_t bits_real_motion_permitted = 0U;
        std::memcpy(&bits_real_motion_permitted, &value_real_motion_permitted, 1U);
        detail::put_le8(bits_real_motion_permitted, output + offset);
        offset += 1U;
        uint64_t value_permit_remaining_us = message.permit_remaining_us();
        uint64_t bits_permit_remaining_us = 0U;
        std::memcpy(&bits_permit_remaining_us, &value_permit_remaining_us, 8U);
        detail::put_le64(bits_permit_remaining_us, output + offset);
        offset += 8U;
        uint64_t value_capabilities = message.capabilities();
        uint64_t bits_capabilities = 0U;
        std::memcpy(&bits_capabilities, &value_capabilities, 8U);
        detail::put_le64(bits_capabilities, output + offset);
        offset += 8U;
        uint64_t value_operation_id = message.operation_id();
        uint64_t bits_operation_id = 0U;
        std::memcpy(&bits_operation_id, &value_operation_id, 8U);
        detail::put_le64(bits_operation_id, output + offset);
        offset += 8U;
        uint8_t value_transaction_phase = message.transaction_phase();
        uint8_t bits_transaction_phase = 0U;
        std::memcpy(&bits_transaction_phase, &value_transaction_phase, 1U);
        detail::put_le8(bits_transaction_phase, output + offset);
        offset += 1U;
        uint16_t value_first_failure_reason = message.first_failure_reason();
        uint16_t bits_first_failure_reason = 0U;
        std::memcpy(&bits_first_failure_reason, &value_first_failure_reason, 2U);
        detail::put_le16(bits_first_failure_reason, output + offset);
        offset += 2U;
        uint64_t value_operation_deadline_us = message.operation_deadline_us();
        uint64_t bits_operation_deadline_us = 0U;
        std::memcpy(&bits_operation_deadline_us, &value_operation_deadline_us, 8U);
        detail::put_le64(bits_operation_deadline_us, output + offset);
        offset += 8U;
        uint32_t value_committed_producer_seq = message.committed_producer_seq();
        uint32_t bits_committed_producer_seq = 0U;
        std::memcpy(&bits_committed_producer_seq, &value_committed_producer_seq, 4U);
        detail::put_le32(bits_committed_producer_seq, output + offset);
        offset += 4U;
        uint8_t value_can_submitted_mask = message.can_submitted_mask();
        uint8_t bits_can_submitted_mask = 0U;
        std::memcpy(&bits_can_submitted_mask, &value_can_submitted_mask, 1U);
        detail::put_le8(bits_can_submitted_mask, output + offset);
        offset += 1U;
        uint8_t value_can_sent_mask = message.can_sent_mask();
        uint8_t bits_can_sent_mask = 0U;
        std::memcpy(&bits_can_sent_mask, &value_can_sent_mask, 1U);
        detail::put_le8(bits_can_sent_mask, output + offset);
        offset += 1U;
        uint8_t value_can_failed_mask = message.can_failed_mask();
        uint8_t bits_can_failed_mask = 0U;
        std::memcpy(&bits_can_failed_mask, &value_can_failed_mask, 1U);
        detail::put_le8(bits_can_failed_mask, output + offset);
        offset += 1U;
        uint32_t value_command_reject_count = message.command_reject_count();
        uint32_t bits_command_reject_count = 0U;
        std::memcpy(&bits_command_reject_count, &value_command_reject_count, 4U);
        detail::put_le32(bits_command_reject_count, output + offset);
        offset += 4U;
        uint32_t value_can_error_count = message.can_error_count();
        uint32_t bits_can_error_count = 0U;
        std::memcpy(&bits_can_error_count, &value_can_error_count, 4U);
        detail::put_le32(bits_can_error_count, output + offset);
        offset += 4U;
        uint32_t value_deadline_miss_count = message.deadline_miss_count();
        uint32_t bits_deadline_miss_count = 0U;
        std::memcpy(&bits_deadline_miss_count, &value_deadline_miss_count, 4U);
        detail::put_le32(bits_deadline_miss_count, output + offset);
        offset += 4U;
        for (size_t index = 0U; index < 32U; ++index) {
            uint8_t value = message.contract_sha256()[index];
            uint8_t bits = 0U;
            std::memcpy(&bits, &value, 1U);
            detail::put_le8(bits, output + offset);
            offset += 1U;
        }
        for (size_t index = 0U; index < 32U; ++index) {
            uint8_t value = message.model_sha256()[index];
            uint8_t bits = 0U;
            std::memcpy(&bits, &value, 1U);
            detail::put_le8(bits, output + offset);
            offset += 1U;
        }
        for (size_t index = 0U; index < 32U; ++index) {
            uint8_t value = message.profile_sha256()[index];
            uint8_t bits = 0U;
            std::memcpy(&bits, &value, 1U);
            detail::put_le8(bits, output + offset);
            offset += 1U;
        }
        for (size_t index = 0U; index < 32U; ++index) {
            uint8_t value = message.firmware_sha256()[index];
            uint8_t bits = 0U;
            std::memcpy(&bits, &value, 1U);
            detail::put_le8(bits, output + offset);
            offset += 1U;
        }
        for (size_t index = 0U; index < 16U; ++index) {
            uint8_t value = message.board_name_ascii()[index];
            uint8_t bits = 0U;
            std::memcpy(&bits, &value, 1U);
            detail::put_le8(bits, output + offset);
            offset += 1U;
        }
        return CodecResult::kOk;
    }
    static CodecResult decode(const uint8_t* input, size_t size, DummyDeviceStatusV2* message) {
        if (input == nullptr || message == nullptr || size != kEncodedSize) return CodecResult::kInvalidArgument;
        DummyDeviceStatusV2 decoded{};
        size_t offset = 0U;
        uint16_t bits_schema_major = detail::get_le16(input + offset);
        uint16_t value_schema_major{};
        std::memcpy(&value_schema_major, &bits_schema_major, 2U);
        decoded.set_schema_major(value_schema_major);
        offset += 2U;
        uint16_t bits_schema_minor = detail::get_le16(input + offset);
        uint16_t value_schema_minor{};
        std::memcpy(&value_schema_minor, &bits_schema_minor, 2U);
        decoded.set_schema_minor(value_schema_minor);
        offset += 2U;
        uint8_t value_device_id[16]{};
        for (size_t index = 0U; index < 16U; ++index) {
            uint8_t bits = detail::get_le8(input + offset);
            std::memcpy(&value_device_id[index], &bits, 1U);
            offset += 1U;
        }
        decoded.set_device_id(value_device_id);
        uint64_t bits_boot_epoch = detail::get_le64(input + offset);
        uint64_t value_boot_epoch{};
        std::memcpy(&value_boot_epoch, &bits_boot_epoch, 8U);
        decoded.set_boot_epoch(value_boot_epoch);
        offset += 8U;
        uint64_t bits_session_id = detail::get_le64(input + offset);
        uint64_t value_session_id{};
        std::memcpy(&value_session_id, &bits_session_id, 8U);
        decoded.set_session_id(value_session_id);
        offset += 8U;
        uint32_t bits_reference_epoch = detail::get_le32(input + offset);
        uint32_t value_reference_epoch{};
        std::memcpy(&value_reference_epoch, &bits_reference_epoch, 4U);
        decoded.set_reference_epoch(value_reference_epoch);
        offset += 4U;
        uint32_t bits_motion_epoch = detail::get_le32(input + offset);
        uint32_t value_motion_epoch{};
        std::memcpy(&value_motion_epoch, &bits_motion_epoch, 4U);
        decoded.set_motion_epoch(value_motion_epoch);
        offset += 4U;
        uint32_t bits_management_epoch = detail::get_le32(input + offset);
        uint32_t value_management_epoch{};
        std::memcpy(&value_management_epoch, &bits_management_epoch, 4U);
        decoded.set_management_epoch(value_management_epoch);
        offset += 4U;
        uint32_t bits_profile_id = detail::get_le32(input + offset);
        uint32_t value_profile_id{};
        std::memcpy(&value_profile_id, &bits_profile_id, 4U);
        decoded.set_profile_id(value_profile_id);
        offset += 4U;
        uint16_t bits_core_state = detail::get_le16(input + offset);
        uint16_t value_core_state{};
        std::memcpy(&value_core_state, &bits_core_state, 2U);
        decoded.set_core_state(value_core_state);
        offset += 2U;
        uint8_t bits_control_mode = detail::get_le8(input + offset);
        uint8_t value_control_mode{};
        std::memcpy(&value_control_mode, &bits_control_mode, 1U);
        decoded.set_control_mode(value_control_mode);
        offset += 1U;
        uint8_t bits_reference_kind = detail::get_le8(input + offset);
        uint8_t value_reference_kind{};
        std::memcpy(&value_reference_kind, &bits_reference_kind, 1U);
        decoded.set_reference_kind(value_reference_kind);
        offset += 1U;
        uint8_t bits_configured_mask = detail::get_le8(input + offset);
        uint8_t value_configured_mask{};
        std::memcpy(&value_configured_mask, &bits_configured_mask, 1U);
        decoded.set_configured_mask(value_configured_mask);
        offset += 1U;
        uint8_t bits_motion_supported = detail::get_le8(input + offset);
        if (bits_motion_supported > 1U) return CodecResult::kInvalidArgument;
        decoded.set_motion_supported(bits_motion_supported != 0U);
        offset += 1U;
        uint8_t bits_real_motion_permitted = detail::get_le8(input + offset);
        if (bits_real_motion_permitted > 1U) return CodecResult::kInvalidArgument;
        decoded.set_real_motion_permitted(bits_real_motion_permitted != 0U);
        offset += 1U;
        uint64_t bits_permit_remaining_us = detail::get_le64(input + offset);
        uint64_t value_permit_remaining_us{};
        std::memcpy(&value_permit_remaining_us, &bits_permit_remaining_us, 8U);
        decoded.set_permit_remaining_us(value_permit_remaining_us);
        offset += 8U;
        uint64_t bits_capabilities = detail::get_le64(input + offset);
        uint64_t value_capabilities{};
        std::memcpy(&value_capabilities, &bits_capabilities, 8U);
        decoded.set_capabilities(value_capabilities);
        offset += 8U;
        uint64_t bits_operation_id = detail::get_le64(input + offset);
        uint64_t value_operation_id{};
        std::memcpy(&value_operation_id, &bits_operation_id, 8U);
        decoded.set_operation_id(value_operation_id);
        offset += 8U;
        uint8_t bits_transaction_phase = detail::get_le8(input + offset);
        uint8_t value_transaction_phase{};
        std::memcpy(&value_transaction_phase, &bits_transaction_phase, 1U);
        decoded.set_transaction_phase(value_transaction_phase);
        offset += 1U;
        uint16_t bits_first_failure_reason = detail::get_le16(input + offset);
        uint16_t value_first_failure_reason{};
        std::memcpy(&value_first_failure_reason, &bits_first_failure_reason, 2U);
        decoded.set_first_failure_reason(value_first_failure_reason);
        offset += 2U;
        uint64_t bits_operation_deadline_us = detail::get_le64(input + offset);
        uint64_t value_operation_deadline_us{};
        std::memcpy(&value_operation_deadline_us, &bits_operation_deadline_us, 8U);
        decoded.set_operation_deadline_us(value_operation_deadline_us);
        offset += 8U;
        uint32_t bits_committed_producer_seq = detail::get_le32(input + offset);
        uint32_t value_committed_producer_seq{};
        std::memcpy(&value_committed_producer_seq, &bits_committed_producer_seq, 4U);
        decoded.set_committed_producer_seq(value_committed_producer_seq);
        offset += 4U;
        uint8_t bits_can_submitted_mask = detail::get_le8(input + offset);
        uint8_t value_can_submitted_mask{};
        std::memcpy(&value_can_submitted_mask, &bits_can_submitted_mask, 1U);
        decoded.set_can_submitted_mask(value_can_submitted_mask);
        offset += 1U;
        uint8_t bits_can_sent_mask = detail::get_le8(input + offset);
        uint8_t value_can_sent_mask{};
        std::memcpy(&value_can_sent_mask, &bits_can_sent_mask, 1U);
        decoded.set_can_sent_mask(value_can_sent_mask);
        offset += 1U;
        uint8_t bits_can_failed_mask = detail::get_le8(input + offset);
        uint8_t value_can_failed_mask{};
        std::memcpy(&value_can_failed_mask, &bits_can_failed_mask, 1U);
        decoded.set_can_failed_mask(value_can_failed_mask);
        offset += 1U;
        uint32_t bits_command_reject_count = detail::get_le32(input + offset);
        uint32_t value_command_reject_count{};
        std::memcpy(&value_command_reject_count, &bits_command_reject_count, 4U);
        decoded.set_command_reject_count(value_command_reject_count);
        offset += 4U;
        uint32_t bits_can_error_count = detail::get_le32(input + offset);
        uint32_t value_can_error_count{};
        std::memcpy(&value_can_error_count, &bits_can_error_count, 4U);
        decoded.set_can_error_count(value_can_error_count);
        offset += 4U;
        uint32_t bits_deadline_miss_count = detail::get_le32(input + offset);
        uint32_t value_deadline_miss_count{};
        std::memcpy(&value_deadline_miss_count, &bits_deadline_miss_count, 4U);
        decoded.set_deadline_miss_count(value_deadline_miss_count);
        offset += 4U;
        uint8_t value_contract_sha256[32]{};
        for (size_t index = 0U; index < 32U; ++index) {
            uint8_t bits = detail::get_le8(input + offset);
            std::memcpy(&value_contract_sha256[index], &bits, 1U);
            offset += 1U;
        }
        decoded.set_contract_sha256(value_contract_sha256);
        uint8_t value_model_sha256[32]{};
        for (size_t index = 0U; index < 32U; ++index) {
            uint8_t bits = detail::get_le8(input + offset);
            std::memcpy(&value_model_sha256[index], &bits, 1U);
            offset += 1U;
        }
        decoded.set_model_sha256(value_model_sha256);
        uint8_t value_profile_sha256[32]{};
        for (size_t index = 0U; index < 32U; ++index) {
            uint8_t bits = detail::get_le8(input + offset);
            std::memcpy(&value_profile_sha256[index], &bits, 1U);
            offset += 1U;
        }
        decoded.set_profile_sha256(value_profile_sha256);
        uint8_t value_firmware_sha256[32]{};
        for (size_t index = 0U; index < 32U; ++index) {
            uint8_t bits = detail::get_le8(input + offset);
            std::memcpy(&value_firmware_sha256[index], &bits, 1U);
            offset += 1U;
        }
        decoded.set_firmware_sha256(value_firmware_sha256);
        uint8_t value_board_name_ascii[16]{};
        for (size_t index = 0U; index < 16U; ++index) {
            uint8_t bits = detail::get_le8(input + offset);
            std::memcpy(&value_board_name_ascii[index], &bits, 1U);
            offset += 1U;
        }
        decoded.set_board_name_ascii(value_board_name_ascii);
        if (!(decoded.schema_major() == 2U && decoded.schema_minor() == 0U)) return CodecResult::kInvalidArgument;
        if (!(decoded.control_mode() <= 2U && decoded.reference_kind() <= 3U && decoded.transaction_phase() <= 4U)) return CodecResult::kInvalidArgument;
        *message = decoded;
        return CodecResult::kOk;
    }
};

class DummyJointCommandV2 {
public:
    uint16_t schema_major() const { return schema_major_; }
    void set_schema_major(uint16_t value) { schema_major_ = value; }
    uint16_t schema_minor() const { return schema_minor_; }
    void set_schema_minor(uint16_t value) { schema_minor_ = value; }
    const uint8_t (&device_id() const)[16] { return device_id_; }
    void set_device_id(const uint8_t (&value)[16]) { std::memcpy(device_id_, value, sizeof(device_id_)); }
    const uint8_t (&client_instance_id() const)[16] { return client_instance_id_; }
    void set_client_instance_id(const uint8_t (&value)[16]) { std::memcpy(client_instance_id_, value, sizeof(client_instance_id_)); }
    uint64_t boot_epoch() const { return boot_epoch_; }
    void set_boot_epoch(uint64_t value) { boot_epoch_ = value; }
    uint64_t session_id() const { return session_id_; }
    void set_session_id(uint64_t value) { session_id_ = value; }
    uint64_t operation_id() const { return operation_id_; }
    void set_operation_id(uint64_t value) { operation_id_ = value; }
    uint32_t reference_epoch() const { return reference_epoch_; }
    void set_reference_epoch(uint32_t value) { reference_epoch_ = value; }
    uint32_t motion_epoch() const { return motion_epoch_; }
    void set_motion_epoch(uint32_t value) { motion_epoch_ = value; }
    uint32_t profile_id() const { return profile_id_; }
    void set_profile_id(uint32_t value) { profile_id_ = value; }
    uint32_t producer_seq() const { return producer_seq_; }
    void set_producer_seq(uint32_t value) { producer_seq_ = value; }
    uint64_t freshness_ticket() const { return freshness_ticket_; }
    void set_freshness_ticket(uint64_t value) { freshness_ticket_ = value; }
    uint8_t configured_mask() const { return configured_mask_; }
    void set_configured_mask(uint8_t value) { configured_mask_ = value; }
    const double (&position_rad() const)[6] { return position_rad_; }
    void set_position_rad(const double (&value)[6]) { std::memcpy(position_rad_, value, sizeof(position_rad_)); }

private:
    uint16_t schema_major_{};
    uint16_t schema_minor_{};
    uint8_t device_id_[16]{};
    uint8_t client_instance_id_[16]{};
    uint64_t boot_epoch_{};
    uint64_t session_id_{};
    uint64_t operation_id_{};
    uint32_t reference_epoch_{};
    uint32_t motion_epoch_{};
    uint32_t profile_id_{};
    uint32_t producer_seq_{};
    uint64_t freshness_ticket_{};
    uint8_t configured_mask_{};
    double position_rad_[6]{};
};

class DummyJointCommandV2Codec {
public:
    static constexpr size_t kEncodedSize = 133U;
    static constexpr size_t kScratchSize = 0U;
    static constexpr size_t kAlignment = alignof(DummyJointCommandV2);
    static constexpr uint32_t kMaxAge = 0U;
    static CodecResult encode(const DummyJointCommandV2& message, uint8_t* output, size_t size) {
        if (output == nullptr || size != kEncodedSize) return CodecResult::kInvalidArgument;
        size_t offset = 0U;
        uint16_t value_schema_major = message.schema_major();
        uint16_t bits_schema_major = 0U;
        std::memcpy(&bits_schema_major, &value_schema_major, 2U);
        detail::put_le16(bits_schema_major, output + offset);
        offset += 2U;
        uint16_t value_schema_minor = message.schema_minor();
        uint16_t bits_schema_minor = 0U;
        std::memcpy(&bits_schema_minor, &value_schema_minor, 2U);
        detail::put_le16(bits_schema_minor, output + offset);
        offset += 2U;
        for (size_t index = 0U; index < 16U; ++index) {
            uint8_t value = message.device_id()[index];
            uint8_t bits = 0U;
            std::memcpy(&bits, &value, 1U);
            detail::put_le8(bits, output + offset);
            offset += 1U;
        }
        for (size_t index = 0U; index < 16U; ++index) {
            uint8_t value = message.client_instance_id()[index];
            uint8_t bits = 0U;
            std::memcpy(&bits, &value, 1U);
            detail::put_le8(bits, output + offset);
            offset += 1U;
        }
        uint64_t value_boot_epoch = message.boot_epoch();
        uint64_t bits_boot_epoch = 0U;
        std::memcpy(&bits_boot_epoch, &value_boot_epoch, 8U);
        detail::put_le64(bits_boot_epoch, output + offset);
        offset += 8U;
        uint64_t value_session_id = message.session_id();
        uint64_t bits_session_id = 0U;
        std::memcpy(&bits_session_id, &value_session_id, 8U);
        detail::put_le64(bits_session_id, output + offset);
        offset += 8U;
        uint64_t value_operation_id = message.operation_id();
        uint64_t bits_operation_id = 0U;
        std::memcpy(&bits_operation_id, &value_operation_id, 8U);
        detail::put_le64(bits_operation_id, output + offset);
        offset += 8U;
        uint32_t value_reference_epoch = message.reference_epoch();
        uint32_t bits_reference_epoch = 0U;
        std::memcpy(&bits_reference_epoch, &value_reference_epoch, 4U);
        detail::put_le32(bits_reference_epoch, output + offset);
        offset += 4U;
        uint32_t value_motion_epoch = message.motion_epoch();
        uint32_t bits_motion_epoch = 0U;
        std::memcpy(&bits_motion_epoch, &value_motion_epoch, 4U);
        detail::put_le32(bits_motion_epoch, output + offset);
        offset += 4U;
        uint32_t value_profile_id = message.profile_id();
        uint32_t bits_profile_id = 0U;
        std::memcpy(&bits_profile_id, &value_profile_id, 4U);
        detail::put_le32(bits_profile_id, output + offset);
        offset += 4U;
        uint32_t value_producer_seq = message.producer_seq();
        uint32_t bits_producer_seq = 0U;
        std::memcpy(&bits_producer_seq, &value_producer_seq, 4U);
        detail::put_le32(bits_producer_seq, output + offset);
        offset += 4U;
        uint64_t value_freshness_ticket = message.freshness_ticket();
        uint64_t bits_freshness_ticket = 0U;
        std::memcpy(&bits_freshness_ticket, &value_freshness_ticket, 8U);
        detail::put_le64(bits_freshness_ticket, output + offset);
        offset += 8U;
        uint8_t value_configured_mask = message.configured_mask();
        uint8_t bits_configured_mask = 0U;
        std::memcpy(&bits_configured_mask, &value_configured_mask, 1U);
        detail::put_le8(bits_configured_mask, output + offset);
        offset += 1U;
        for (size_t index = 0U; index < 6U; ++index) {
            double value = message.position_rad()[index];
            uint64_t bits = 0U;
            std::memcpy(&bits, &value, 8U);
            detail::put_le64(bits, output + offset);
            offset += 8U;
        }
        return CodecResult::kOk;
    }
    static CodecResult decode(const uint8_t* input, size_t size, DummyJointCommandV2* message) {
        if (input == nullptr || message == nullptr || size != kEncodedSize) return CodecResult::kInvalidArgument;
        DummyJointCommandV2 decoded{};
        size_t offset = 0U;
        uint16_t bits_schema_major = detail::get_le16(input + offset);
        uint16_t value_schema_major{};
        std::memcpy(&value_schema_major, &bits_schema_major, 2U);
        decoded.set_schema_major(value_schema_major);
        offset += 2U;
        uint16_t bits_schema_minor = detail::get_le16(input + offset);
        uint16_t value_schema_minor{};
        std::memcpy(&value_schema_minor, &bits_schema_minor, 2U);
        decoded.set_schema_minor(value_schema_minor);
        offset += 2U;
        uint8_t value_device_id[16]{};
        for (size_t index = 0U; index < 16U; ++index) {
            uint8_t bits = detail::get_le8(input + offset);
            std::memcpy(&value_device_id[index], &bits, 1U);
            offset += 1U;
        }
        decoded.set_device_id(value_device_id);
        uint8_t value_client_instance_id[16]{};
        for (size_t index = 0U; index < 16U; ++index) {
            uint8_t bits = detail::get_le8(input + offset);
            std::memcpy(&value_client_instance_id[index], &bits, 1U);
            offset += 1U;
        }
        decoded.set_client_instance_id(value_client_instance_id);
        uint64_t bits_boot_epoch = detail::get_le64(input + offset);
        uint64_t value_boot_epoch{};
        std::memcpy(&value_boot_epoch, &bits_boot_epoch, 8U);
        decoded.set_boot_epoch(value_boot_epoch);
        offset += 8U;
        uint64_t bits_session_id = detail::get_le64(input + offset);
        uint64_t value_session_id{};
        std::memcpy(&value_session_id, &bits_session_id, 8U);
        decoded.set_session_id(value_session_id);
        offset += 8U;
        uint64_t bits_operation_id = detail::get_le64(input + offset);
        uint64_t value_operation_id{};
        std::memcpy(&value_operation_id, &bits_operation_id, 8U);
        decoded.set_operation_id(value_operation_id);
        offset += 8U;
        uint32_t bits_reference_epoch = detail::get_le32(input + offset);
        uint32_t value_reference_epoch{};
        std::memcpy(&value_reference_epoch, &bits_reference_epoch, 4U);
        decoded.set_reference_epoch(value_reference_epoch);
        offset += 4U;
        uint32_t bits_motion_epoch = detail::get_le32(input + offset);
        uint32_t value_motion_epoch{};
        std::memcpy(&value_motion_epoch, &bits_motion_epoch, 4U);
        decoded.set_motion_epoch(value_motion_epoch);
        offset += 4U;
        uint32_t bits_profile_id = detail::get_le32(input + offset);
        uint32_t value_profile_id{};
        std::memcpy(&value_profile_id, &bits_profile_id, 4U);
        decoded.set_profile_id(value_profile_id);
        offset += 4U;
        uint32_t bits_producer_seq = detail::get_le32(input + offset);
        uint32_t value_producer_seq{};
        std::memcpy(&value_producer_seq, &bits_producer_seq, 4U);
        decoded.set_producer_seq(value_producer_seq);
        offset += 4U;
        uint64_t bits_freshness_ticket = detail::get_le64(input + offset);
        uint64_t value_freshness_ticket{};
        std::memcpy(&value_freshness_ticket, &bits_freshness_ticket, 8U);
        decoded.set_freshness_ticket(value_freshness_ticket);
        offset += 8U;
        uint8_t bits_configured_mask = detail::get_le8(input + offset);
        uint8_t value_configured_mask{};
        std::memcpy(&value_configured_mask, &bits_configured_mask, 1U);
        decoded.set_configured_mask(value_configured_mask);
        offset += 1U;
        double value_position_rad[6]{};
        for (size_t index = 0U; index < 6U; ++index) {
            uint64_t bits = detail::get_le64(input + offset);
            std::memcpy(&value_position_rad[index], &bits, 8U);
            offset += 8U;
        }
        decoded.set_position_rad(value_position_rad);
        if (!(decoded.schema_major() == 2U && decoded.schema_minor() == 0U)) return CodecResult::kInvalidArgument;
        *message = decoded;
        return CodecResult::kOk;
    }
};

class DummyJointStateV2 {
public:
    uint16_t schema_major() const { return schema_major_; }
    void set_schema_major(uint16_t value) { schema_major_ = value; }
    uint16_t schema_minor() const { return schema_minor_; }
    void set_schema_minor(uint16_t value) { schema_minor_ = value; }
    const uint8_t (&device_id() const)[16] { return device_id_; }
    void set_device_id(const uint8_t (&value)[16]) { std::memcpy(device_id_, value, sizeof(device_id_)); }
    uint64_t boot_epoch() const { return boot_epoch_; }
    void set_boot_epoch(uint64_t value) { boot_epoch_ = value; }
    uint64_t session_id() const { return session_id_; }
    void set_session_id(uint64_t value) { session_id_ = value; }
    uint32_t reference_epoch() const { return reference_epoch_; }
    void set_reference_epoch(uint32_t value) { reference_epoch_ = value; }
    uint32_t motion_epoch() const { return motion_epoch_; }
    void set_motion_epoch(uint32_t value) { motion_epoch_ = value; }
    uint32_t management_epoch() const { return management_epoch_; }
    void set_management_epoch(uint32_t value) { management_epoch_ = value; }
    uint32_t profile_id() const { return profile_id_; }
    void set_profile_id(uint32_t value) { profile_id_ = value; }
    uint32_t state_seq() const { return state_seq_; }
    void set_state_seq(uint32_t value) { state_seq_ = value; }
    uint64_t produced_at_us() const { return produced_at_us_; }
    void set_produced_at_us(uint64_t value) { produced_at_us_ = value; }
    uint64_t freshness_ticket() const { return freshness_ticket_; }
    void set_freshness_ticket(uint64_t value) { freshness_ticket_ = value; }
    uint8_t configured_mask() const { return configured_mask_; }
    void set_configured_mask(uint8_t value) { configured_mask_ = value; }
    uint8_t position_valid_mask() const { return position_valid_mask_; }
    void set_position_valid_mask(uint8_t value) { position_valid_mask_ = value; }
    uint8_t velocity_valid_mask() const { return velocity_valid_mask_; }
    void set_velocity_valid_mask(uint8_t value) { velocity_valid_mask_ = value; }
    uint8_t raw_position_valid_mask() const { return raw_position_valid_mask_; }
    void set_raw_position_valid_mask(uint8_t value) { raw_position_valid_mask_ = value; }
    uint32_t committed_producer_seq() const { return committed_producer_seq_; }
    void set_committed_producer_seq(uint32_t value) { committed_producer_seq_ = value; }
    uint32_t admitted_producer_seq() const { return admitted_producer_seq_; }
    void set_admitted_producer_seq(uint32_t value) { admitted_producer_seq_ = value; }
    uint64_t operation_id() const { return operation_id_; }
    void set_operation_id(uint64_t value) { operation_id_ = value; }
    uint8_t transaction_phase() const { return transaction_phase_; }
    void set_transaction_phase(uint8_t value) { transaction_phase_ = value; }
    uint16_t first_failure_reason() const { return first_failure_reason_; }
    void set_first_failure_reason(uint16_t value) { first_failure_reason_ = value; }
    uint64_t operation_deadline_us() const { return operation_deadline_us_; }
    void set_operation_deadline_us(uint64_t value) { operation_deadline_us_ = value; }
    uint8_t can_submitted_mask() const { return can_submitted_mask_; }
    void set_can_submitted_mask(uint8_t value) { can_submitted_mask_ = value; }
    uint8_t can_sent_mask() const { return can_sent_mask_; }
    void set_can_sent_mask(uint8_t value) { can_sent_mask_ = value; }
    uint8_t can_failed_mask() const { return can_failed_mask_; }
    void set_can_failed_mask(uint8_t value) { can_failed_mask_ = value; }
    const uint32_t (&feedback_seq() const)[6] { return feedback_seq_; }
    void set_feedback_seq(const uint32_t (&value)[6]) { std::memcpy(feedback_seq_, value, sizeof(feedback_seq_)); }
    const uint32_t (&can_rx_age_us() const)[6] { return can_rx_age_us_; }
    void set_can_rx_age_us(const uint32_t (&value)[6]) { std::memcpy(can_rx_age_us_, value, sizeof(can_rx_age_us_)); }
    const double (&position_rad() const)[6] { return position_rad_; }
    void set_position_rad(const double (&value)[6]) { std::memcpy(position_rad_, value, sizeof(position_rad_)); }
    const double (&estimated_velocity_rad_s() const)[6] { return estimated_velocity_rad_s_; }
    void set_estimated_velocity_rad_s(const double (&value)[6]) { std::memcpy(estimated_velocity_rad_s_, value, sizeof(estimated_velocity_rad_s_)); }
    const double (&raw_motor_position_rev() const)[6] { return raw_motor_position_rev_; }
    void set_raw_motor_position_rev(const double (&value)[6]) { std::memcpy(raw_motor_position_rev_, value, sizeof(raw_motor_position_rev_)); }
    uint16_t core_state() const { return core_state_; }
    void set_core_state(uint16_t value) { core_state_ = value; }
    bool hold_ready() const { return hold_ready_; }
    void set_hold_ready(bool value) { hold_ready_ = value; }

private:
    uint16_t schema_major_{};
    uint16_t schema_minor_{};
    uint8_t device_id_[16]{};
    uint64_t boot_epoch_{};
    uint64_t session_id_{};
    uint32_t reference_epoch_{};
    uint32_t motion_epoch_{};
    uint32_t management_epoch_{};
    uint32_t profile_id_{};
    uint32_t state_seq_{};
    uint64_t produced_at_us_{};
    uint64_t freshness_ticket_{};
    uint8_t configured_mask_{};
    uint8_t position_valid_mask_{};
    uint8_t velocity_valid_mask_{};
    uint8_t raw_position_valid_mask_{};
    uint32_t committed_producer_seq_{};
    uint32_t admitted_producer_seq_{};
    uint64_t operation_id_{};
    uint8_t transaction_phase_{};
    uint16_t first_failure_reason_{};
    uint64_t operation_deadline_us_{};
    uint8_t can_submitted_mask_{};
    uint8_t can_sent_mask_{};
    uint8_t can_failed_mask_{};
    uint32_t feedback_seq_[6]{};
    uint32_t can_rx_age_us_[6]{};
    double position_rad_[6]{};
    double estimated_velocity_rad_s_[6]{};
    double raw_motor_position_rev_[6]{};
    uint16_t core_state_{};
    bool hold_ready_{};
};

class DummyJointStateV2Codec {
public:
    static constexpr size_t kEncodedSize = 301U;
    static constexpr size_t kScratchSize = 0U;
    static constexpr size_t kAlignment = alignof(DummyJointStateV2);
    static constexpr uint32_t kMaxAge = 0U;
    static CodecResult encode(const DummyJointStateV2& message, uint8_t* output, size_t size) {
        if (output == nullptr || size != kEncodedSize) return CodecResult::kInvalidArgument;
        size_t offset = 0U;
        uint16_t value_schema_major = message.schema_major();
        uint16_t bits_schema_major = 0U;
        std::memcpy(&bits_schema_major, &value_schema_major, 2U);
        detail::put_le16(bits_schema_major, output + offset);
        offset += 2U;
        uint16_t value_schema_minor = message.schema_minor();
        uint16_t bits_schema_minor = 0U;
        std::memcpy(&bits_schema_minor, &value_schema_minor, 2U);
        detail::put_le16(bits_schema_minor, output + offset);
        offset += 2U;
        for (size_t index = 0U; index < 16U; ++index) {
            uint8_t value = message.device_id()[index];
            uint8_t bits = 0U;
            std::memcpy(&bits, &value, 1U);
            detail::put_le8(bits, output + offset);
            offset += 1U;
        }
        uint64_t value_boot_epoch = message.boot_epoch();
        uint64_t bits_boot_epoch = 0U;
        std::memcpy(&bits_boot_epoch, &value_boot_epoch, 8U);
        detail::put_le64(bits_boot_epoch, output + offset);
        offset += 8U;
        uint64_t value_session_id = message.session_id();
        uint64_t bits_session_id = 0U;
        std::memcpy(&bits_session_id, &value_session_id, 8U);
        detail::put_le64(bits_session_id, output + offset);
        offset += 8U;
        uint32_t value_reference_epoch = message.reference_epoch();
        uint32_t bits_reference_epoch = 0U;
        std::memcpy(&bits_reference_epoch, &value_reference_epoch, 4U);
        detail::put_le32(bits_reference_epoch, output + offset);
        offset += 4U;
        uint32_t value_motion_epoch = message.motion_epoch();
        uint32_t bits_motion_epoch = 0U;
        std::memcpy(&bits_motion_epoch, &value_motion_epoch, 4U);
        detail::put_le32(bits_motion_epoch, output + offset);
        offset += 4U;
        uint32_t value_management_epoch = message.management_epoch();
        uint32_t bits_management_epoch = 0U;
        std::memcpy(&bits_management_epoch, &value_management_epoch, 4U);
        detail::put_le32(bits_management_epoch, output + offset);
        offset += 4U;
        uint32_t value_profile_id = message.profile_id();
        uint32_t bits_profile_id = 0U;
        std::memcpy(&bits_profile_id, &value_profile_id, 4U);
        detail::put_le32(bits_profile_id, output + offset);
        offset += 4U;
        uint32_t value_state_seq = message.state_seq();
        uint32_t bits_state_seq = 0U;
        std::memcpy(&bits_state_seq, &value_state_seq, 4U);
        detail::put_le32(bits_state_seq, output + offset);
        offset += 4U;
        uint64_t value_produced_at_us = message.produced_at_us();
        uint64_t bits_produced_at_us = 0U;
        std::memcpy(&bits_produced_at_us, &value_produced_at_us, 8U);
        detail::put_le64(bits_produced_at_us, output + offset);
        offset += 8U;
        uint64_t value_freshness_ticket = message.freshness_ticket();
        uint64_t bits_freshness_ticket = 0U;
        std::memcpy(&bits_freshness_ticket, &value_freshness_ticket, 8U);
        detail::put_le64(bits_freshness_ticket, output + offset);
        offset += 8U;
        uint8_t value_configured_mask = message.configured_mask();
        uint8_t bits_configured_mask = 0U;
        std::memcpy(&bits_configured_mask, &value_configured_mask, 1U);
        detail::put_le8(bits_configured_mask, output + offset);
        offset += 1U;
        uint8_t value_position_valid_mask = message.position_valid_mask();
        uint8_t bits_position_valid_mask = 0U;
        std::memcpy(&bits_position_valid_mask, &value_position_valid_mask, 1U);
        detail::put_le8(bits_position_valid_mask, output + offset);
        offset += 1U;
        uint8_t value_velocity_valid_mask = message.velocity_valid_mask();
        uint8_t bits_velocity_valid_mask = 0U;
        std::memcpy(&bits_velocity_valid_mask, &value_velocity_valid_mask, 1U);
        detail::put_le8(bits_velocity_valid_mask, output + offset);
        offset += 1U;
        uint8_t value_raw_position_valid_mask = message.raw_position_valid_mask();
        uint8_t bits_raw_position_valid_mask = 0U;
        std::memcpy(&bits_raw_position_valid_mask, &value_raw_position_valid_mask, 1U);
        detail::put_le8(bits_raw_position_valid_mask, output + offset);
        offset += 1U;
        uint32_t value_committed_producer_seq = message.committed_producer_seq();
        uint32_t bits_committed_producer_seq = 0U;
        std::memcpy(&bits_committed_producer_seq, &value_committed_producer_seq, 4U);
        detail::put_le32(bits_committed_producer_seq, output + offset);
        offset += 4U;
        uint32_t value_admitted_producer_seq = message.admitted_producer_seq();
        uint32_t bits_admitted_producer_seq = 0U;
        std::memcpy(&bits_admitted_producer_seq, &value_admitted_producer_seq, 4U);
        detail::put_le32(bits_admitted_producer_seq, output + offset);
        offset += 4U;
        uint64_t value_operation_id = message.operation_id();
        uint64_t bits_operation_id = 0U;
        std::memcpy(&bits_operation_id, &value_operation_id, 8U);
        detail::put_le64(bits_operation_id, output + offset);
        offset += 8U;
        uint8_t value_transaction_phase = message.transaction_phase();
        uint8_t bits_transaction_phase = 0U;
        std::memcpy(&bits_transaction_phase, &value_transaction_phase, 1U);
        detail::put_le8(bits_transaction_phase, output + offset);
        offset += 1U;
        uint16_t value_first_failure_reason = message.first_failure_reason();
        uint16_t bits_first_failure_reason = 0U;
        std::memcpy(&bits_first_failure_reason, &value_first_failure_reason, 2U);
        detail::put_le16(bits_first_failure_reason, output + offset);
        offset += 2U;
        uint64_t value_operation_deadline_us = message.operation_deadline_us();
        uint64_t bits_operation_deadline_us = 0U;
        std::memcpy(&bits_operation_deadline_us, &value_operation_deadline_us, 8U);
        detail::put_le64(bits_operation_deadline_us, output + offset);
        offset += 8U;
        uint8_t value_can_submitted_mask = message.can_submitted_mask();
        uint8_t bits_can_submitted_mask = 0U;
        std::memcpy(&bits_can_submitted_mask, &value_can_submitted_mask, 1U);
        detail::put_le8(bits_can_submitted_mask, output + offset);
        offset += 1U;
        uint8_t value_can_sent_mask = message.can_sent_mask();
        uint8_t bits_can_sent_mask = 0U;
        std::memcpy(&bits_can_sent_mask, &value_can_sent_mask, 1U);
        detail::put_le8(bits_can_sent_mask, output + offset);
        offset += 1U;
        uint8_t value_can_failed_mask = message.can_failed_mask();
        uint8_t bits_can_failed_mask = 0U;
        std::memcpy(&bits_can_failed_mask, &value_can_failed_mask, 1U);
        detail::put_le8(bits_can_failed_mask, output + offset);
        offset += 1U;
        for (size_t index = 0U; index < 6U; ++index) {
            uint32_t value = message.feedback_seq()[index];
            uint32_t bits = 0U;
            std::memcpy(&bits, &value, 4U);
            detail::put_le32(bits, output + offset);
            offset += 4U;
        }
        for (size_t index = 0U; index < 6U; ++index) {
            uint32_t value = message.can_rx_age_us()[index];
            uint32_t bits = 0U;
            std::memcpy(&bits, &value, 4U);
            detail::put_le32(bits, output + offset);
            offset += 4U;
        }
        for (size_t index = 0U; index < 6U; ++index) {
            double value = message.position_rad()[index];
            uint64_t bits = 0U;
            std::memcpy(&bits, &value, 8U);
            detail::put_le64(bits, output + offset);
            offset += 8U;
        }
        for (size_t index = 0U; index < 6U; ++index) {
            double value = message.estimated_velocity_rad_s()[index];
            uint64_t bits = 0U;
            std::memcpy(&bits, &value, 8U);
            detail::put_le64(bits, output + offset);
            offset += 8U;
        }
        for (size_t index = 0U; index < 6U; ++index) {
            double value = message.raw_motor_position_rev()[index];
            uint64_t bits = 0U;
            std::memcpy(&bits, &value, 8U);
            detail::put_le64(bits, output + offset);
            offset += 8U;
        }
        uint16_t value_core_state = message.core_state();
        uint16_t bits_core_state = 0U;
        std::memcpy(&bits_core_state, &value_core_state, 2U);
        detail::put_le16(bits_core_state, output + offset);
        offset += 2U;
        bool value_hold_ready = message.hold_ready();
        uint8_t bits_hold_ready = 0U;
        std::memcpy(&bits_hold_ready, &value_hold_ready, 1U);
        detail::put_le8(bits_hold_ready, output + offset);
        offset += 1U;
        return CodecResult::kOk;
    }
    static CodecResult decode(const uint8_t* input, size_t size, DummyJointStateV2* message) {
        if (input == nullptr || message == nullptr || size != kEncodedSize) return CodecResult::kInvalidArgument;
        DummyJointStateV2 decoded{};
        size_t offset = 0U;
        uint16_t bits_schema_major = detail::get_le16(input + offset);
        uint16_t value_schema_major{};
        std::memcpy(&value_schema_major, &bits_schema_major, 2U);
        decoded.set_schema_major(value_schema_major);
        offset += 2U;
        uint16_t bits_schema_minor = detail::get_le16(input + offset);
        uint16_t value_schema_minor{};
        std::memcpy(&value_schema_minor, &bits_schema_minor, 2U);
        decoded.set_schema_minor(value_schema_minor);
        offset += 2U;
        uint8_t value_device_id[16]{};
        for (size_t index = 0U; index < 16U; ++index) {
            uint8_t bits = detail::get_le8(input + offset);
            std::memcpy(&value_device_id[index], &bits, 1U);
            offset += 1U;
        }
        decoded.set_device_id(value_device_id);
        uint64_t bits_boot_epoch = detail::get_le64(input + offset);
        uint64_t value_boot_epoch{};
        std::memcpy(&value_boot_epoch, &bits_boot_epoch, 8U);
        decoded.set_boot_epoch(value_boot_epoch);
        offset += 8U;
        uint64_t bits_session_id = detail::get_le64(input + offset);
        uint64_t value_session_id{};
        std::memcpy(&value_session_id, &bits_session_id, 8U);
        decoded.set_session_id(value_session_id);
        offset += 8U;
        uint32_t bits_reference_epoch = detail::get_le32(input + offset);
        uint32_t value_reference_epoch{};
        std::memcpy(&value_reference_epoch, &bits_reference_epoch, 4U);
        decoded.set_reference_epoch(value_reference_epoch);
        offset += 4U;
        uint32_t bits_motion_epoch = detail::get_le32(input + offset);
        uint32_t value_motion_epoch{};
        std::memcpy(&value_motion_epoch, &bits_motion_epoch, 4U);
        decoded.set_motion_epoch(value_motion_epoch);
        offset += 4U;
        uint32_t bits_management_epoch = detail::get_le32(input + offset);
        uint32_t value_management_epoch{};
        std::memcpy(&value_management_epoch, &bits_management_epoch, 4U);
        decoded.set_management_epoch(value_management_epoch);
        offset += 4U;
        uint32_t bits_profile_id = detail::get_le32(input + offset);
        uint32_t value_profile_id{};
        std::memcpy(&value_profile_id, &bits_profile_id, 4U);
        decoded.set_profile_id(value_profile_id);
        offset += 4U;
        uint32_t bits_state_seq = detail::get_le32(input + offset);
        uint32_t value_state_seq{};
        std::memcpy(&value_state_seq, &bits_state_seq, 4U);
        decoded.set_state_seq(value_state_seq);
        offset += 4U;
        uint64_t bits_produced_at_us = detail::get_le64(input + offset);
        uint64_t value_produced_at_us{};
        std::memcpy(&value_produced_at_us, &bits_produced_at_us, 8U);
        decoded.set_produced_at_us(value_produced_at_us);
        offset += 8U;
        uint64_t bits_freshness_ticket = detail::get_le64(input + offset);
        uint64_t value_freshness_ticket{};
        std::memcpy(&value_freshness_ticket, &bits_freshness_ticket, 8U);
        decoded.set_freshness_ticket(value_freshness_ticket);
        offset += 8U;
        uint8_t bits_configured_mask = detail::get_le8(input + offset);
        uint8_t value_configured_mask{};
        std::memcpy(&value_configured_mask, &bits_configured_mask, 1U);
        decoded.set_configured_mask(value_configured_mask);
        offset += 1U;
        uint8_t bits_position_valid_mask = detail::get_le8(input + offset);
        uint8_t value_position_valid_mask{};
        std::memcpy(&value_position_valid_mask, &bits_position_valid_mask, 1U);
        decoded.set_position_valid_mask(value_position_valid_mask);
        offset += 1U;
        uint8_t bits_velocity_valid_mask = detail::get_le8(input + offset);
        uint8_t value_velocity_valid_mask{};
        std::memcpy(&value_velocity_valid_mask, &bits_velocity_valid_mask, 1U);
        decoded.set_velocity_valid_mask(value_velocity_valid_mask);
        offset += 1U;
        uint8_t bits_raw_position_valid_mask = detail::get_le8(input + offset);
        uint8_t value_raw_position_valid_mask{};
        std::memcpy(&value_raw_position_valid_mask, &bits_raw_position_valid_mask, 1U);
        decoded.set_raw_position_valid_mask(value_raw_position_valid_mask);
        offset += 1U;
        uint32_t bits_committed_producer_seq = detail::get_le32(input + offset);
        uint32_t value_committed_producer_seq{};
        std::memcpy(&value_committed_producer_seq, &bits_committed_producer_seq, 4U);
        decoded.set_committed_producer_seq(value_committed_producer_seq);
        offset += 4U;
        uint32_t bits_admitted_producer_seq = detail::get_le32(input + offset);
        uint32_t value_admitted_producer_seq{};
        std::memcpy(&value_admitted_producer_seq, &bits_admitted_producer_seq, 4U);
        decoded.set_admitted_producer_seq(value_admitted_producer_seq);
        offset += 4U;
        uint64_t bits_operation_id = detail::get_le64(input + offset);
        uint64_t value_operation_id{};
        std::memcpy(&value_operation_id, &bits_operation_id, 8U);
        decoded.set_operation_id(value_operation_id);
        offset += 8U;
        uint8_t bits_transaction_phase = detail::get_le8(input + offset);
        uint8_t value_transaction_phase{};
        std::memcpy(&value_transaction_phase, &bits_transaction_phase, 1U);
        decoded.set_transaction_phase(value_transaction_phase);
        offset += 1U;
        uint16_t bits_first_failure_reason = detail::get_le16(input + offset);
        uint16_t value_first_failure_reason{};
        std::memcpy(&value_first_failure_reason, &bits_first_failure_reason, 2U);
        decoded.set_first_failure_reason(value_first_failure_reason);
        offset += 2U;
        uint64_t bits_operation_deadline_us = detail::get_le64(input + offset);
        uint64_t value_operation_deadline_us{};
        std::memcpy(&value_operation_deadline_us, &bits_operation_deadline_us, 8U);
        decoded.set_operation_deadline_us(value_operation_deadline_us);
        offset += 8U;
        uint8_t bits_can_submitted_mask = detail::get_le8(input + offset);
        uint8_t value_can_submitted_mask{};
        std::memcpy(&value_can_submitted_mask, &bits_can_submitted_mask, 1U);
        decoded.set_can_submitted_mask(value_can_submitted_mask);
        offset += 1U;
        uint8_t bits_can_sent_mask = detail::get_le8(input + offset);
        uint8_t value_can_sent_mask{};
        std::memcpy(&value_can_sent_mask, &bits_can_sent_mask, 1U);
        decoded.set_can_sent_mask(value_can_sent_mask);
        offset += 1U;
        uint8_t bits_can_failed_mask = detail::get_le8(input + offset);
        uint8_t value_can_failed_mask{};
        std::memcpy(&value_can_failed_mask, &bits_can_failed_mask, 1U);
        decoded.set_can_failed_mask(value_can_failed_mask);
        offset += 1U;
        uint32_t value_feedback_seq[6]{};
        for (size_t index = 0U; index < 6U; ++index) {
            uint32_t bits = detail::get_le32(input + offset);
            std::memcpy(&value_feedback_seq[index], &bits, 4U);
            offset += 4U;
        }
        decoded.set_feedback_seq(value_feedback_seq);
        uint32_t value_can_rx_age_us[6]{};
        for (size_t index = 0U; index < 6U; ++index) {
            uint32_t bits = detail::get_le32(input + offset);
            std::memcpy(&value_can_rx_age_us[index], &bits, 4U);
            offset += 4U;
        }
        decoded.set_can_rx_age_us(value_can_rx_age_us);
        double value_position_rad[6]{};
        for (size_t index = 0U; index < 6U; ++index) {
            uint64_t bits = detail::get_le64(input + offset);
            std::memcpy(&value_position_rad[index], &bits, 8U);
            offset += 8U;
        }
        decoded.set_position_rad(value_position_rad);
        double value_estimated_velocity_rad_s[6]{};
        for (size_t index = 0U; index < 6U; ++index) {
            uint64_t bits = detail::get_le64(input + offset);
            std::memcpy(&value_estimated_velocity_rad_s[index], &bits, 8U);
            offset += 8U;
        }
        decoded.set_estimated_velocity_rad_s(value_estimated_velocity_rad_s);
        double value_raw_motor_position_rev[6]{};
        for (size_t index = 0U; index < 6U; ++index) {
            uint64_t bits = detail::get_le64(input + offset);
            std::memcpy(&value_raw_motor_position_rev[index], &bits, 8U);
            offset += 8U;
        }
        decoded.set_raw_motor_position_rev(value_raw_motor_position_rev);
        uint16_t bits_core_state = detail::get_le16(input + offset);
        uint16_t value_core_state{};
        std::memcpy(&value_core_state, &bits_core_state, 2U);
        decoded.set_core_state(value_core_state);
        offset += 2U;
        uint8_t bits_hold_ready = detail::get_le8(input + offset);
        if (bits_hold_ready > 1U) return CodecResult::kInvalidArgument;
        decoded.set_hold_ready(bits_hold_ready != 0U);
        offset += 1U;
        if (!(decoded.schema_major() == 2U && decoded.schema_minor() == 0U && decoded.core_state() <= 9U)) return CodecResult::kInvalidArgument;
        if (!(decoded.transaction_phase() <= 4U)) return CodecResult::kInvalidArgument;
        *message = decoded;
        return CodecResult::kOk;
    }
};

class DummyManageV2Request {
public:
    uint16_t schema_major() const { return schema_major_; }
    void set_schema_major(uint16_t value) { schema_major_ = value; }
    uint16_t schema_minor() const { return schema_minor_; }
    void set_schema_minor(uint16_t value) { schema_minor_ = value; }
    const uint8_t (&device_id() const)[16] { return device_id_; }
    void set_device_id(const uint8_t (&value)[16]) { std::memcpy(device_id_, value, sizeof(device_id_)); }
    const uint8_t (&client_instance_id() const)[16] { return client_instance_id_; }
    void set_client_instance_id(const uint8_t (&value)[16]) { std::memcpy(client_instance_id_, value, sizeof(client_instance_id_)); }
    uint64_t request_id() const { return request_id_; }
    void set_request_id(uint64_t value) { request_id_ = value; }
    uint32_t request_seq() const { return request_seq_; }
    void set_request_seq(uint32_t value) { request_seq_ = value; }
    uint16_t operation() const { return operation_; }
    void set_operation(uint16_t value) { operation_ = value; }
    uint64_t queried_operation_id() const { return queried_operation_id_; }
    void set_queried_operation_id(uint64_t value) { queried_operation_id_ = value; }
    uint64_t expected_boot_epoch() const { return expected_boot_epoch_; }
    void set_expected_boot_epoch(uint64_t value) { expected_boot_epoch_ = value; }
    uint64_t expected_session_id() const { return expected_session_id_; }
    void set_expected_session_id(uint64_t value) { expected_session_id_ = value; }
    uint32_t expected_reference_epoch() const { return expected_reference_epoch_; }
    void set_expected_reference_epoch(uint32_t value) { expected_reference_epoch_ = value; }
    uint32_t expected_motion_epoch() const { return expected_motion_epoch_; }
    void set_expected_motion_epoch(uint32_t value) { expected_motion_epoch_ = value; }
    uint32_t expected_management_epoch() const { return expected_management_epoch_; }
    void set_expected_management_epoch(uint32_t value) { expected_management_epoch_ = value; }
    uint32_t expected_profile_id() const { return expected_profile_id_; }
    void set_expected_profile_id(uint32_t value) { expected_profile_id_ = value; }
    uint64_t freshness_ticket() const { return freshness_ticket_; }
    void set_freshness_ticket(uint64_t value) { freshness_ticket_ = value; }
    uint8_t requested_control_mode() const { return requested_control_mode_; }
    void set_requested_control_mode(uint8_t value) { requested_control_mode_ = value; }
    uint8_t configured_mask() const { return configured_mask_; }
    void set_configured_mask(uint8_t value) { configured_mask_ = value; }
    bool operator_confirmed() const { return operator_confirmed_; }
    void set_operator_confirmed(bool value) { operator_confirmed_ = value; }
    const uint8_t (&permit_sha256() const)[32] { return permit_sha256_; }
    void set_permit_sha256(const uint8_t (&value)[32]) { std::memcpy(permit_sha256_, value, sizeof(permit_sha256_)); }
    const uint8_t (&compiled_policy_sha256() const)[32] { return compiled_policy_sha256_; }
    void set_compiled_policy_sha256(const uint8_t (&value)[32]) { std::memcpy(compiled_policy_sha256_, value, sizeof(compiled_policy_sha256_)); }
    uint64_t permit_remaining_us() const { return permit_remaining_us_; }
    void set_permit_remaining_us(uint64_t value) { permit_remaining_us_ = value; }
    uint32_t terminal_producer_seq() const { return terminal_producer_seq_; }
    void set_terminal_producer_seq(uint32_t value) { terminal_producer_seq_ = value; }
    const double (&terminal_position_rad() const)[6] { return terminal_position_rad_; }
    void set_terminal_position_rad(const double (&value)[6]) { std::memcpy(terminal_position_rad_, value, sizeof(terminal_position_rad_)); }

private:
    uint16_t schema_major_{};
    uint16_t schema_minor_{};
    uint8_t device_id_[16]{};
    uint8_t client_instance_id_[16]{};
    uint64_t request_id_{};
    uint32_t request_seq_{};
    uint16_t operation_{};
    uint64_t queried_operation_id_{};
    uint64_t expected_boot_epoch_{};
    uint64_t expected_session_id_{};
    uint32_t expected_reference_epoch_{};
    uint32_t expected_motion_epoch_{};
    uint32_t expected_management_epoch_{};
    uint32_t expected_profile_id_{};
    uint64_t freshness_ticket_{};
    uint8_t requested_control_mode_{};
    uint8_t configured_mask_{};
    bool operator_confirmed_{};
    uint8_t permit_sha256_[32]{};
    uint8_t compiled_policy_sha256_[32]{};
    uint64_t permit_remaining_us_{};
    uint32_t terminal_producer_seq_{};
    double terminal_position_rad_[6]{};
};

class DummyManageV2RequestCodec {
public:
    static constexpr size_t kEncodedSize = 225U;
    static constexpr size_t kScratchSize = 0U;
    static constexpr size_t kAlignment = alignof(DummyManageV2Request);
    static constexpr uint32_t kMaxAge = 0U;
    static CodecResult encode(const DummyManageV2Request& message, uint8_t* output, size_t size) {
        if (output == nullptr || size != kEncodedSize) return CodecResult::kInvalidArgument;
        size_t offset = 0U;
        uint16_t value_schema_major = message.schema_major();
        uint16_t bits_schema_major = 0U;
        std::memcpy(&bits_schema_major, &value_schema_major, 2U);
        detail::put_le16(bits_schema_major, output + offset);
        offset += 2U;
        uint16_t value_schema_minor = message.schema_minor();
        uint16_t bits_schema_minor = 0U;
        std::memcpy(&bits_schema_minor, &value_schema_minor, 2U);
        detail::put_le16(bits_schema_minor, output + offset);
        offset += 2U;
        for (size_t index = 0U; index < 16U; ++index) {
            uint8_t value = message.device_id()[index];
            uint8_t bits = 0U;
            std::memcpy(&bits, &value, 1U);
            detail::put_le8(bits, output + offset);
            offset += 1U;
        }
        for (size_t index = 0U; index < 16U; ++index) {
            uint8_t value = message.client_instance_id()[index];
            uint8_t bits = 0U;
            std::memcpy(&bits, &value, 1U);
            detail::put_le8(bits, output + offset);
            offset += 1U;
        }
        uint64_t value_request_id = message.request_id();
        uint64_t bits_request_id = 0U;
        std::memcpy(&bits_request_id, &value_request_id, 8U);
        detail::put_le64(bits_request_id, output + offset);
        offset += 8U;
        uint32_t value_request_seq = message.request_seq();
        uint32_t bits_request_seq = 0U;
        std::memcpy(&bits_request_seq, &value_request_seq, 4U);
        detail::put_le32(bits_request_seq, output + offset);
        offset += 4U;
        uint16_t value_operation = message.operation();
        uint16_t bits_operation = 0U;
        std::memcpy(&bits_operation, &value_operation, 2U);
        detail::put_le16(bits_operation, output + offset);
        offset += 2U;
        uint64_t value_queried_operation_id = message.queried_operation_id();
        uint64_t bits_queried_operation_id = 0U;
        std::memcpy(&bits_queried_operation_id, &value_queried_operation_id, 8U);
        detail::put_le64(bits_queried_operation_id, output + offset);
        offset += 8U;
        uint64_t value_expected_boot_epoch = message.expected_boot_epoch();
        uint64_t bits_expected_boot_epoch = 0U;
        std::memcpy(&bits_expected_boot_epoch, &value_expected_boot_epoch, 8U);
        detail::put_le64(bits_expected_boot_epoch, output + offset);
        offset += 8U;
        uint64_t value_expected_session_id = message.expected_session_id();
        uint64_t bits_expected_session_id = 0U;
        std::memcpy(&bits_expected_session_id, &value_expected_session_id, 8U);
        detail::put_le64(bits_expected_session_id, output + offset);
        offset += 8U;
        uint32_t value_expected_reference_epoch = message.expected_reference_epoch();
        uint32_t bits_expected_reference_epoch = 0U;
        std::memcpy(&bits_expected_reference_epoch, &value_expected_reference_epoch, 4U);
        detail::put_le32(bits_expected_reference_epoch, output + offset);
        offset += 4U;
        uint32_t value_expected_motion_epoch = message.expected_motion_epoch();
        uint32_t bits_expected_motion_epoch = 0U;
        std::memcpy(&bits_expected_motion_epoch, &value_expected_motion_epoch, 4U);
        detail::put_le32(bits_expected_motion_epoch, output + offset);
        offset += 4U;
        uint32_t value_expected_management_epoch = message.expected_management_epoch();
        uint32_t bits_expected_management_epoch = 0U;
        std::memcpy(&bits_expected_management_epoch, &value_expected_management_epoch, 4U);
        detail::put_le32(bits_expected_management_epoch, output + offset);
        offset += 4U;
        uint32_t value_expected_profile_id = message.expected_profile_id();
        uint32_t bits_expected_profile_id = 0U;
        std::memcpy(&bits_expected_profile_id, &value_expected_profile_id, 4U);
        detail::put_le32(bits_expected_profile_id, output + offset);
        offset += 4U;
        uint64_t value_freshness_ticket = message.freshness_ticket();
        uint64_t bits_freshness_ticket = 0U;
        std::memcpy(&bits_freshness_ticket, &value_freshness_ticket, 8U);
        detail::put_le64(bits_freshness_ticket, output + offset);
        offset += 8U;
        uint8_t value_requested_control_mode = message.requested_control_mode();
        uint8_t bits_requested_control_mode = 0U;
        std::memcpy(&bits_requested_control_mode, &value_requested_control_mode, 1U);
        detail::put_le8(bits_requested_control_mode, output + offset);
        offset += 1U;
        uint8_t value_configured_mask = message.configured_mask();
        uint8_t bits_configured_mask = 0U;
        std::memcpy(&bits_configured_mask, &value_configured_mask, 1U);
        detail::put_le8(bits_configured_mask, output + offset);
        offset += 1U;
        bool value_operator_confirmed = message.operator_confirmed();
        uint8_t bits_operator_confirmed = 0U;
        std::memcpy(&bits_operator_confirmed, &value_operator_confirmed, 1U);
        detail::put_le8(bits_operator_confirmed, output + offset);
        offset += 1U;
        for (size_t index = 0U; index < 32U; ++index) {
            uint8_t value = message.permit_sha256()[index];
            uint8_t bits = 0U;
            std::memcpy(&bits, &value, 1U);
            detail::put_le8(bits, output + offset);
            offset += 1U;
        }
        for (size_t index = 0U; index < 32U; ++index) {
            uint8_t value = message.compiled_policy_sha256()[index];
            uint8_t bits = 0U;
            std::memcpy(&bits, &value, 1U);
            detail::put_le8(bits, output + offset);
            offset += 1U;
        }
        uint64_t value_permit_remaining_us = message.permit_remaining_us();
        uint64_t bits_permit_remaining_us = 0U;
        std::memcpy(&bits_permit_remaining_us, &value_permit_remaining_us, 8U);
        detail::put_le64(bits_permit_remaining_us, output + offset);
        offset += 8U;
        uint32_t value_terminal_producer_seq = message.terminal_producer_seq();
        uint32_t bits_terminal_producer_seq = 0U;
        std::memcpy(&bits_terminal_producer_seq, &value_terminal_producer_seq, 4U);
        detail::put_le32(bits_terminal_producer_seq, output + offset);
        offset += 4U;
        for (size_t index = 0U; index < 6U; ++index) {
            double value = message.terminal_position_rad()[index];
            uint64_t bits = 0U;
            std::memcpy(&bits, &value, 8U);
            detail::put_le64(bits, output + offset);
            offset += 8U;
        }
        return CodecResult::kOk;
    }
    static CodecResult decode(const uint8_t* input, size_t size, DummyManageV2Request* message) {
        if (input == nullptr || message == nullptr || size != kEncodedSize) return CodecResult::kInvalidArgument;
        DummyManageV2Request decoded{};
        size_t offset = 0U;
        uint16_t bits_schema_major = detail::get_le16(input + offset);
        uint16_t value_schema_major{};
        std::memcpy(&value_schema_major, &bits_schema_major, 2U);
        decoded.set_schema_major(value_schema_major);
        offset += 2U;
        uint16_t bits_schema_minor = detail::get_le16(input + offset);
        uint16_t value_schema_minor{};
        std::memcpy(&value_schema_minor, &bits_schema_minor, 2U);
        decoded.set_schema_minor(value_schema_minor);
        offset += 2U;
        uint8_t value_device_id[16]{};
        for (size_t index = 0U; index < 16U; ++index) {
            uint8_t bits = detail::get_le8(input + offset);
            std::memcpy(&value_device_id[index], &bits, 1U);
            offset += 1U;
        }
        decoded.set_device_id(value_device_id);
        uint8_t value_client_instance_id[16]{};
        for (size_t index = 0U; index < 16U; ++index) {
            uint8_t bits = detail::get_le8(input + offset);
            std::memcpy(&value_client_instance_id[index], &bits, 1U);
            offset += 1U;
        }
        decoded.set_client_instance_id(value_client_instance_id);
        uint64_t bits_request_id = detail::get_le64(input + offset);
        uint64_t value_request_id{};
        std::memcpy(&value_request_id, &bits_request_id, 8U);
        decoded.set_request_id(value_request_id);
        offset += 8U;
        uint32_t bits_request_seq = detail::get_le32(input + offset);
        uint32_t value_request_seq{};
        std::memcpy(&value_request_seq, &bits_request_seq, 4U);
        decoded.set_request_seq(value_request_seq);
        offset += 4U;
        uint16_t bits_operation = detail::get_le16(input + offset);
        uint16_t value_operation{};
        std::memcpy(&value_operation, &bits_operation, 2U);
        decoded.set_operation(value_operation);
        offset += 2U;
        uint64_t bits_queried_operation_id = detail::get_le64(input + offset);
        uint64_t value_queried_operation_id{};
        std::memcpy(&value_queried_operation_id, &bits_queried_operation_id, 8U);
        decoded.set_queried_operation_id(value_queried_operation_id);
        offset += 8U;
        uint64_t bits_expected_boot_epoch = detail::get_le64(input + offset);
        uint64_t value_expected_boot_epoch{};
        std::memcpy(&value_expected_boot_epoch, &bits_expected_boot_epoch, 8U);
        decoded.set_expected_boot_epoch(value_expected_boot_epoch);
        offset += 8U;
        uint64_t bits_expected_session_id = detail::get_le64(input + offset);
        uint64_t value_expected_session_id{};
        std::memcpy(&value_expected_session_id, &bits_expected_session_id, 8U);
        decoded.set_expected_session_id(value_expected_session_id);
        offset += 8U;
        uint32_t bits_expected_reference_epoch = detail::get_le32(input + offset);
        uint32_t value_expected_reference_epoch{};
        std::memcpy(&value_expected_reference_epoch, &bits_expected_reference_epoch, 4U);
        decoded.set_expected_reference_epoch(value_expected_reference_epoch);
        offset += 4U;
        uint32_t bits_expected_motion_epoch = detail::get_le32(input + offset);
        uint32_t value_expected_motion_epoch{};
        std::memcpy(&value_expected_motion_epoch, &bits_expected_motion_epoch, 4U);
        decoded.set_expected_motion_epoch(value_expected_motion_epoch);
        offset += 4U;
        uint32_t bits_expected_management_epoch = detail::get_le32(input + offset);
        uint32_t value_expected_management_epoch{};
        std::memcpy(&value_expected_management_epoch, &bits_expected_management_epoch, 4U);
        decoded.set_expected_management_epoch(value_expected_management_epoch);
        offset += 4U;
        uint32_t bits_expected_profile_id = detail::get_le32(input + offset);
        uint32_t value_expected_profile_id{};
        std::memcpy(&value_expected_profile_id, &bits_expected_profile_id, 4U);
        decoded.set_expected_profile_id(value_expected_profile_id);
        offset += 4U;
        uint64_t bits_freshness_ticket = detail::get_le64(input + offset);
        uint64_t value_freshness_ticket{};
        std::memcpy(&value_freshness_ticket, &bits_freshness_ticket, 8U);
        decoded.set_freshness_ticket(value_freshness_ticket);
        offset += 8U;
        uint8_t bits_requested_control_mode = detail::get_le8(input + offset);
        uint8_t value_requested_control_mode{};
        std::memcpy(&value_requested_control_mode, &bits_requested_control_mode, 1U);
        decoded.set_requested_control_mode(value_requested_control_mode);
        offset += 1U;
        uint8_t bits_configured_mask = detail::get_le8(input + offset);
        uint8_t value_configured_mask{};
        std::memcpy(&value_configured_mask, &bits_configured_mask, 1U);
        decoded.set_configured_mask(value_configured_mask);
        offset += 1U;
        uint8_t bits_operator_confirmed = detail::get_le8(input + offset);
        if (bits_operator_confirmed > 1U) return CodecResult::kInvalidArgument;
        decoded.set_operator_confirmed(bits_operator_confirmed != 0U);
        offset += 1U;
        uint8_t value_permit_sha256[32]{};
        for (size_t index = 0U; index < 32U; ++index) {
            uint8_t bits = detail::get_le8(input + offset);
            std::memcpy(&value_permit_sha256[index], &bits, 1U);
            offset += 1U;
        }
        decoded.set_permit_sha256(value_permit_sha256);
        uint8_t value_compiled_policy_sha256[32]{};
        for (size_t index = 0U; index < 32U; ++index) {
            uint8_t bits = detail::get_le8(input + offset);
            std::memcpy(&value_compiled_policy_sha256[index], &bits, 1U);
            offset += 1U;
        }
        decoded.set_compiled_policy_sha256(value_compiled_policy_sha256);
        uint64_t bits_permit_remaining_us = detail::get_le64(input + offset);
        uint64_t value_permit_remaining_us{};
        std::memcpy(&value_permit_remaining_us, &bits_permit_remaining_us, 8U);
        decoded.set_permit_remaining_us(value_permit_remaining_us);
        offset += 8U;
        uint32_t bits_terminal_producer_seq = detail::get_le32(input + offset);
        uint32_t value_terminal_producer_seq{};
        std::memcpy(&value_terminal_producer_seq, &bits_terminal_producer_seq, 4U);
        decoded.set_terminal_producer_seq(value_terminal_producer_seq);
        offset += 4U;
        double value_terminal_position_rad[6]{};
        for (size_t index = 0U; index < 6U; ++index) {
            uint64_t bits = detail::get_le64(input + offset);
            std::memcpy(&value_terminal_position_rad[index], &bits, 8U);
            offset += 8U;
        }
        decoded.set_terminal_position_rad(value_terminal_position_rad);
        if (!(decoded.schema_major() == 2U && decoded.schema_minor() == 0U && decoded.operation() <= 15U && decoded.requested_control_mode() <= 2U)) return CodecResult::kInvalidArgument;
        *message = decoded;
        return CodecResult::kOk;
    }
};

class DummyManageV2Response {
public:
    uint16_t schema_major() const { return schema_major_; }
    void set_schema_major(uint16_t value) { schema_major_ = value; }
    uint16_t schema_minor() const { return schema_minor_; }
    void set_schema_minor(uint16_t value) { schema_minor_ = value; }
    const uint8_t (&device_id() const)[16] { return device_id_; }
    void set_device_id(const uint8_t (&value)[16]) { std::memcpy(device_id_, value, sizeof(device_id_)); }
    const uint8_t (&client_instance_id() const)[16] { return client_instance_id_; }
    void set_client_instance_id(const uint8_t (&value)[16]) { std::memcpy(client_instance_id_, value, sizeof(client_instance_id_)); }
    uint64_t request_id() const { return request_id_; }
    void set_request_id(uint64_t value) { request_id_ = value; }
    uint8_t receipt() const { return receipt_; }
    void set_receipt(uint8_t value) { receipt_ = value; }
    uint16_t result_code() const { return result_code_; }
    void set_result_code(uint16_t value) { result_code_ = value; }
    uint64_t operation_id() const { return operation_id_; }
    void set_operation_id(uint64_t value) { operation_id_ = value; }
    uint8_t transaction_phase() const { return transaction_phase_; }
    void set_transaction_phase(uint8_t value) { transaction_phase_ = value; }
    uint16_t first_failure_reason() const { return first_failure_reason_; }
    void set_first_failure_reason(uint16_t value) { first_failure_reason_ = value; }
    uint64_t operation_deadline_us() const { return operation_deadline_us_; }
    void set_operation_deadline_us(uint64_t value) { operation_deadline_us_ = value; }
    uint64_t boot_epoch() const { return boot_epoch_; }
    void set_boot_epoch(uint64_t value) { boot_epoch_ = value; }
    uint64_t session_id() const { return session_id_; }
    void set_session_id(uint64_t value) { session_id_ = value; }
    uint32_t reference_epoch() const { return reference_epoch_; }
    void set_reference_epoch(uint32_t value) { reference_epoch_ = value; }
    uint32_t motion_epoch() const { return motion_epoch_; }
    void set_motion_epoch(uint32_t value) { motion_epoch_ = value; }
    uint32_t management_epoch() const { return management_epoch_; }
    void set_management_epoch(uint32_t value) { management_epoch_ = value; }
    uint32_t profile_id() const { return profile_id_; }
    void set_profile_id(uint32_t value) { profile_id_ = value; }
    uint32_t committed_producer_seq() const { return committed_producer_seq_; }
    void set_committed_producer_seq(uint32_t value) { committed_producer_seq_ = value; }
    uint8_t can_submitted_mask() const { return can_submitted_mask_; }
    void set_can_submitted_mask(uint8_t value) { can_submitted_mask_ = value; }
    uint8_t can_sent_mask() const { return can_sent_mask_; }
    void set_can_sent_mask(uint8_t value) { can_sent_mask_ = value; }
    uint8_t can_failed_mask() const { return can_failed_mask_; }
    void set_can_failed_mask(uint8_t value) { can_failed_mask_ = value; }

private:
    uint16_t schema_major_{};
    uint16_t schema_minor_{};
    uint8_t device_id_[16]{};
    uint8_t client_instance_id_[16]{};
    uint64_t request_id_{};
    uint8_t receipt_{};
    uint16_t result_code_{};
    uint64_t operation_id_{};
    uint8_t transaction_phase_{};
    uint16_t first_failure_reason_{};
    uint64_t operation_deadline_us_{};
    uint64_t boot_epoch_{};
    uint64_t session_id_{};
    uint32_t reference_epoch_{};
    uint32_t motion_epoch_{};
    uint32_t management_epoch_{};
    uint32_t profile_id_{};
    uint32_t committed_producer_seq_{};
    uint8_t can_submitted_mask_{};
    uint8_t can_sent_mask_{};
    uint8_t can_failed_mask_{};
};

class DummyManageV2ResponseCodec {
public:
    static constexpr size_t kEncodedSize = 105U;
    static constexpr size_t kScratchSize = 0U;
    static constexpr size_t kAlignment = alignof(DummyManageV2Response);
    static constexpr uint32_t kMaxAge = 0U;
    static CodecResult encode(const DummyManageV2Response& message, uint8_t* output, size_t size) {
        if (output == nullptr || size != kEncodedSize) return CodecResult::kInvalidArgument;
        size_t offset = 0U;
        uint16_t value_schema_major = message.schema_major();
        uint16_t bits_schema_major = 0U;
        std::memcpy(&bits_schema_major, &value_schema_major, 2U);
        detail::put_le16(bits_schema_major, output + offset);
        offset += 2U;
        uint16_t value_schema_minor = message.schema_minor();
        uint16_t bits_schema_minor = 0U;
        std::memcpy(&bits_schema_minor, &value_schema_minor, 2U);
        detail::put_le16(bits_schema_minor, output + offset);
        offset += 2U;
        for (size_t index = 0U; index < 16U; ++index) {
            uint8_t value = message.device_id()[index];
            uint8_t bits = 0U;
            std::memcpy(&bits, &value, 1U);
            detail::put_le8(bits, output + offset);
            offset += 1U;
        }
        for (size_t index = 0U; index < 16U; ++index) {
            uint8_t value = message.client_instance_id()[index];
            uint8_t bits = 0U;
            std::memcpy(&bits, &value, 1U);
            detail::put_le8(bits, output + offset);
            offset += 1U;
        }
        uint64_t value_request_id = message.request_id();
        uint64_t bits_request_id = 0U;
        std::memcpy(&bits_request_id, &value_request_id, 8U);
        detail::put_le64(bits_request_id, output + offset);
        offset += 8U;
        uint8_t value_receipt = message.receipt();
        uint8_t bits_receipt = 0U;
        std::memcpy(&bits_receipt, &value_receipt, 1U);
        detail::put_le8(bits_receipt, output + offset);
        offset += 1U;
        uint16_t value_result_code = message.result_code();
        uint16_t bits_result_code = 0U;
        std::memcpy(&bits_result_code, &value_result_code, 2U);
        detail::put_le16(bits_result_code, output + offset);
        offset += 2U;
        uint64_t value_operation_id = message.operation_id();
        uint64_t bits_operation_id = 0U;
        std::memcpy(&bits_operation_id, &value_operation_id, 8U);
        detail::put_le64(bits_operation_id, output + offset);
        offset += 8U;
        uint8_t value_transaction_phase = message.transaction_phase();
        uint8_t bits_transaction_phase = 0U;
        std::memcpy(&bits_transaction_phase, &value_transaction_phase, 1U);
        detail::put_le8(bits_transaction_phase, output + offset);
        offset += 1U;
        uint16_t value_first_failure_reason = message.first_failure_reason();
        uint16_t bits_first_failure_reason = 0U;
        std::memcpy(&bits_first_failure_reason, &value_first_failure_reason, 2U);
        detail::put_le16(bits_first_failure_reason, output + offset);
        offset += 2U;
        uint64_t value_operation_deadline_us = message.operation_deadline_us();
        uint64_t bits_operation_deadline_us = 0U;
        std::memcpy(&bits_operation_deadline_us, &value_operation_deadline_us, 8U);
        detail::put_le64(bits_operation_deadline_us, output + offset);
        offset += 8U;
        uint64_t value_boot_epoch = message.boot_epoch();
        uint64_t bits_boot_epoch = 0U;
        std::memcpy(&bits_boot_epoch, &value_boot_epoch, 8U);
        detail::put_le64(bits_boot_epoch, output + offset);
        offset += 8U;
        uint64_t value_session_id = message.session_id();
        uint64_t bits_session_id = 0U;
        std::memcpy(&bits_session_id, &value_session_id, 8U);
        detail::put_le64(bits_session_id, output + offset);
        offset += 8U;
        uint32_t value_reference_epoch = message.reference_epoch();
        uint32_t bits_reference_epoch = 0U;
        std::memcpy(&bits_reference_epoch, &value_reference_epoch, 4U);
        detail::put_le32(bits_reference_epoch, output + offset);
        offset += 4U;
        uint32_t value_motion_epoch = message.motion_epoch();
        uint32_t bits_motion_epoch = 0U;
        std::memcpy(&bits_motion_epoch, &value_motion_epoch, 4U);
        detail::put_le32(bits_motion_epoch, output + offset);
        offset += 4U;
        uint32_t value_management_epoch = message.management_epoch();
        uint32_t bits_management_epoch = 0U;
        std::memcpy(&bits_management_epoch, &value_management_epoch, 4U);
        detail::put_le32(bits_management_epoch, output + offset);
        offset += 4U;
        uint32_t value_profile_id = message.profile_id();
        uint32_t bits_profile_id = 0U;
        std::memcpy(&bits_profile_id, &value_profile_id, 4U);
        detail::put_le32(bits_profile_id, output + offset);
        offset += 4U;
        uint32_t value_committed_producer_seq = message.committed_producer_seq();
        uint32_t bits_committed_producer_seq = 0U;
        std::memcpy(&bits_committed_producer_seq, &value_committed_producer_seq, 4U);
        detail::put_le32(bits_committed_producer_seq, output + offset);
        offset += 4U;
        uint8_t value_can_submitted_mask = message.can_submitted_mask();
        uint8_t bits_can_submitted_mask = 0U;
        std::memcpy(&bits_can_submitted_mask, &value_can_submitted_mask, 1U);
        detail::put_le8(bits_can_submitted_mask, output + offset);
        offset += 1U;
        uint8_t value_can_sent_mask = message.can_sent_mask();
        uint8_t bits_can_sent_mask = 0U;
        std::memcpy(&bits_can_sent_mask, &value_can_sent_mask, 1U);
        detail::put_le8(bits_can_sent_mask, output + offset);
        offset += 1U;
        uint8_t value_can_failed_mask = message.can_failed_mask();
        uint8_t bits_can_failed_mask = 0U;
        std::memcpy(&bits_can_failed_mask, &value_can_failed_mask, 1U);
        detail::put_le8(bits_can_failed_mask, output + offset);
        offset += 1U;
        return CodecResult::kOk;
    }
    static CodecResult decode(const uint8_t* input, size_t size, DummyManageV2Response* message) {
        if (input == nullptr || message == nullptr || size != kEncodedSize) return CodecResult::kInvalidArgument;
        DummyManageV2Response decoded{};
        size_t offset = 0U;
        uint16_t bits_schema_major = detail::get_le16(input + offset);
        uint16_t value_schema_major{};
        std::memcpy(&value_schema_major, &bits_schema_major, 2U);
        decoded.set_schema_major(value_schema_major);
        offset += 2U;
        uint16_t bits_schema_minor = detail::get_le16(input + offset);
        uint16_t value_schema_minor{};
        std::memcpy(&value_schema_minor, &bits_schema_minor, 2U);
        decoded.set_schema_minor(value_schema_minor);
        offset += 2U;
        uint8_t value_device_id[16]{};
        for (size_t index = 0U; index < 16U; ++index) {
            uint8_t bits = detail::get_le8(input + offset);
            std::memcpy(&value_device_id[index], &bits, 1U);
            offset += 1U;
        }
        decoded.set_device_id(value_device_id);
        uint8_t value_client_instance_id[16]{};
        for (size_t index = 0U; index < 16U; ++index) {
            uint8_t bits = detail::get_le8(input + offset);
            std::memcpy(&value_client_instance_id[index], &bits, 1U);
            offset += 1U;
        }
        decoded.set_client_instance_id(value_client_instance_id);
        uint64_t bits_request_id = detail::get_le64(input + offset);
        uint64_t value_request_id{};
        std::memcpy(&value_request_id, &bits_request_id, 8U);
        decoded.set_request_id(value_request_id);
        offset += 8U;
        uint8_t bits_receipt = detail::get_le8(input + offset);
        uint8_t value_receipt{};
        std::memcpy(&value_receipt, &bits_receipt, 1U);
        decoded.set_receipt(value_receipt);
        offset += 1U;
        uint16_t bits_result_code = detail::get_le16(input + offset);
        uint16_t value_result_code{};
        std::memcpy(&value_result_code, &bits_result_code, 2U);
        decoded.set_result_code(value_result_code);
        offset += 2U;
        uint64_t bits_operation_id = detail::get_le64(input + offset);
        uint64_t value_operation_id{};
        std::memcpy(&value_operation_id, &bits_operation_id, 8U);
        decoded.set_operation_id(value_operation_id);
        offset += 8U;
        uint8_t bits_transaction_phase = detail::get_le8(input + offset);
        uint8_t value_transaction_phase{};
        std::memcpy(&value_transaction_phase, &bits_transaction_phase, 1U);
        decoded.set_transaction_phase(value_transaction_phase);
        offset += 1U;
        uint16_t bits_first_failure_reason = detail::get_le16(input + offset);
        uint16_t value_first_failure_reason{};
        std::memcpy(&value_first_failure_reason, &bits_first_failure_reason, 2U);
        decoded.set_first_failure_reason(value_first_failure_reason);
        offset += 2U;
        uint64_t bits_operation_deadline_us = detail::get_le64(input + offset);
        uint64_t value_operation_deadline_us{};
        std::memcpy(&value_operation_deadline_us, &bits_operation_deadline_us, 8U);
        decoded.set_operation_deadline_us(value_operation_deadline_us);
        offset += 8U;
        uint64_t bits_boot_epoch = detail::get_le64(input + offset);
        uint64_t value_boot_epoch{};
        std::memcpy(&value_boot_epoch, &bits_boot_epoch, 8U);
        decoded.set_boot_epoch(value_boot_epoch);
        offset += 8U;
        uint64_t bits_session_id = detail::get_le64(input + offset);
        uint64_t value_session_id{};
        std::memcpy(&value_session_id, &bits_session_id, 8U);
        decoded.set_session_id(value_session_id);
        offset += 8U;
        uint32_t bits_reference_epoch = detail::get_le32(input + offset);
        uint32_t value_reference_epoch{};
        std::memcpy(&value_reference_epoch, &bits_reference_epoch, 4U);
        decoded.set_reference_epoch(value_reference_epoch);
        offset += 4U;
        uint32_t bits_motion_epoch = detail::get_le32(input + offset);
        uint32_t value_motion_epoch{};
        std::memcpy(&value_motion_epoch, &bits_motion_epoch, 4U);
        decoded.set_motion_epoch(value_motion_epoch);
        offset += 4U;
        uint32_t bits_management_epoch = detail::get_le32(input + offset);
        uint32_t value_management_epoch{};
        std::memcpy(&value_management_epoch, &bits_management_epoch, 4U);
        decoded.set_management_epoch(value_management_epoch);
        offset += 4U;
        uint32_t bits_profile_id = detail::get_le32(input + offset);
        uint32_t value_profile_id{};
        std::memcpy(&value_profile_id, &bits_profile_id, 4U);
        decoded.set_profile_id(value_profile_id);
        offset += 4U;
        uint32_t bits_committed_producer_seq = detail::get_le32(input + offset);
        uint32_t value_committed_producer_seq{};
        std::memcpy(&value_committed_producer_seq, &bits_committed_producer_seq, 4U);
        decoded.set_committed_producer_seq(value_committed_producer_seq);
        offset += 4U;
        uint8_t bits_can_submitted_mask = detail::get_le8(input + offset);
        uint8_t value_can_submitted_mask{};
        std::memcpy(&value_can_submitted_mask, &bits_can_submitted_mask, 1U);
        decoded.set_can_submitted_mask(value_can_submitted_mask);
        offset += 1U;
        uint8_t bits_can_sent_mask = detail::get_le8(input + offset);
        uint8_t value_can_sent_mask{};
        std::memcpy(&value_can_sent_mask, &bits_can_sent_mask, 1U);
        decoded.set_can_sent_mask(value_can_sent_mask);
        offset += 1U;
        uint8_t bits_can_failed_mask = detail::get_le8(input + offset);
        uint8_t value_can_failed_mask{};
        std::memcpy(&value_can_failed_mask, &bits_can_failed_mask, 1U);
        decoded.set_can_failed_mask(value_can_failed_mask);
        offset += 1U;
        if (!(decoded.schema_major() == 2U && decoded.schema_minor() == 0U && decoded.receipt() <= 4U)) return CodecResult::kInvalidArgument;
        if (!(decoded.result_code() <= 31U && decoded.transaction_phase() <= 4U)) return CodecResult::kInvalidArgument;
        *message = decoded;
        return CodecResult::kOk;
    }
};

constexpr size_t max_object_bytes() {
    size_t result = 0U;
    if (sizeof(DummyDeviceStatusV2) > result) result = sizeof(DummyDeviceStatusV2);
    if (sizeof(DummyJointCommandV2) > result) result = sizeof(DummyJointCommandV2);
    if (sizeof(DummyJointStateV2) > result) result = sizeof(DummyJointStateV2);
    if (sizeof(DummyManageV2Request) > result) result = sizeof(DummyManageV2Request);
    if (sizeof(DummyManageV2Response) > result) result = sizeof(DummyManageV2Response);
    return result;
}
constexpr size_t max_object_alignment() {
    size_t result = 1U;
    if (alignof(DummyDeviceStatusV2) > result) result = alignof(DummyDeviceStatusV2);
    if (alignof(DummyJointCommandV2) > result) result = alignof(DummyJointCommandV2);
    if (alignof(DummyJointStateV2) > result) result = alignof(DummyJointStateV2);
    if (alignof(DummyManageV2Request) > result) result = alignof(DummyManageV2Request);
    if (alignof(DummyManageV2Response) > result) result = alignof(DummyManageV2Response);
    return result;
}
inline constexpr size_t kMaxObjectBytes = max_object_bytes();
inline constexpr size_t kMaxObjectAlignment = max_object_alignment();
}  // namespace dp::protocol::dummy_robot_v2
