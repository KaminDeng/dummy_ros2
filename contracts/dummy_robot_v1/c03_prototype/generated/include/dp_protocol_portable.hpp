// Auto-generated from ROS IDL. Do not edit.
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <type_traits>

namespace dp::iface {
enum class SetResult : uint8_t { kOk = 0, kInvalidArgument };

namespace tlv { template <typename T> struct Codec; }

struct StringView { const char* data_; size_t size_; const char* data() const { return data_; } size_t size() const { return size_; } };
template <typename T> struct Span { T* data_; size_t size_; T* data() const { return data_; } size_t size() const { return size_; } };

template <typename T, size_t N> class FixedArray { static_assert(std::is_trivially_copyable<T>::value); static_assert(N <= SIZE_MAX / sizeof(T)); public: const T* data() const { return data_; } size_t size() const { return N; } SetResult set(Span<const T> value) { if (value.size_ != N || (N != 0U && value.data_ == nullptr)) return SetResult::kInvalidArgument; std::memcpy(data_, value.data_, N * sizeof(T)); return SetResult::kOk; } private: template <typename U> friend struct tlv::Codec; T data_[N]{}; };

template <size_t N> class BoundedString { public: StringView view() const { return {data_, size_}; } SetResult set(StringView value) { if (value.size_ > N || (value.size_ != 0U && value.data_ == nullptr)) return SetResult::kInvalidArgument; if (value.size_ != 0U) std::memcpy(data_, value.data_, value.size_); size_ = value.size_; return SetResult::kOk; } private: char data_[N]{}; size_t size_ = 0U; };

template <typename T, size_t N> class BoundedVector { static_assert(std::is_trivially_copyable<T>::value); static_assert(N <= SIZE_MAX / sizeof(T)); public: Span<const T> view() const { return {data_, size_}; } SetResult set(Span<const T> value) { if (value.size_ > N || (value.size_ != 0U && value.data_ == nullptr)) return SetResult::kInvalidArgument; if (value.size_ != 0U) std::memcpy(data_, value.data_, value.size_ * sizeof(T)); size_ = value.size_; return SetResult::kOk; } private: template <typename U> friend struct tlv::Codec; T data_[N]{}; size_t size_ = 0U; };

class CapabilityAnnounceV1 {
public:
    Span<const uint8_t> device_id() const { return device_id_.view(); }
    SetResult set_device_id(Span<const uint8_t> value) { return device_id_.set(value); }
    uint32_t protocol_version() const { return protocol_version_; }
    SetResult set_protocol_version(uint32_t value) { protocol_version_ = value; return SetResult::kOk; }
    const FixedArray<uint8_t, 16>& capabilities() const { return capabilities_; }
    SetResult set_capabilities(Span<const uint8_t> value) { return capabilities_.set(value); }

private:
    template <typename U> friend struct tlv::Codec;
    BoundedVector<uint8_t, 16> device_id_{};
    uint32_t protocol_version_{};
    FixedArray<uint8_t, 16> capabilities_{};
};

class CapabilityBitmapV1 {
public:
    const FixedArray<uint8_t, 16>& catalog_fingerprint() const { return catalog_fingerprint_; }
    SetResult set_catalog_fingerprint(Span<const uint8_t> value) { return catalog_fingerprint_.set(value); }
    uint16_t page_index() const { return page_index_; }
    SetResult set_page_index(uint16_t value) { page_index_ = value; return SetResult::kOk; }
    uint16_t page_count() const { return page_count_; }
    SetResult set_page_count(uint16_t value) { page_count_ = value; return SetResult::kOk; }
    uint8_t valid_bytes() const { return valid_bytes_; }
    SetResult set_valid_bytes(uint8_t value) { valid_bytes_ = value; return SetResult::kOk; }
    const FixedArray<uint8_t, 16>& capability_bits() const { return capability_bits_; }
    SetResult set_capability_bits(Span<const uint8_t> value) { return capability_bits_.set(value); }
    uint16_t codec_profile_bits() const { return codec_profile_bits_; }
    SetResult set_codec_profile_bits(uint16_t value) { codec_profile_bits_ = value; return SetResult::kOk; }
    uint16_t qos_bits() const { return qos_bits_; }
    SetResult set_qos_bits(uint16_t value) { qos_bits_ = value; return SetResult::kOk; }

private:
    template <typename U> friend struct tlv::Codec;
    FixedArray<uint8_t, 16> catalog_fingerprint_{};
    uint16_t page_index_{};
    uint16_t page_count_{};
    uint8_t valid_bytes_{};
    FixedArray<uint8_t, 16> capability_bits_{};
    uint16_t codec_profile_bits_{};
    uint16_t qos_bits_{};
};

class ControlWatchdogV1 {
public:
    uint32_t timestamp_ms() const { return timestamp_ms_; }
    SetResult set_timestamp_ms(uint32_t value) { timestamp_ms_ = value; return SetResult::kOk; }
    uint32_t deadline_miss_count() const { return deadline_miss_count_; }
    SetResult set_deadline_miss_count(uint32_t value) { deadline_miss_count_ = value; return SetResult::kOk; }
    uint32_t max_jitter_ms() const { return max_jitter_ms_; }
    SetResult set_max_jitter_ms(uint32_t value) { max_jitter_ms_ = value; return SetResult::kOk; }
    bool command_fresh() const { return command_fresh_; }
    SetResult set_command_fresh(bool value) { command_fresh_ = value; return SetResult::kOk; }
    bool state_fresh() const { return state_fresh_; }
    SetResult set_state_fresh(bool value) { state_fresh_ = value; return SetResult::kOk; }

private:
    template <typename U> friend struct tlv::Codec;
    uint32_t timestamp_ms_{};
    uint32_t deadline_miss_count_{};
    uint32_t max_jitter_ms_{};
    bool command_fresh_{};
    bool state_fresh_{};
};

class CrashAnnounceV1 {
public:
    CrashAnnounceV1() {
        uploaded_ = false;
    }
    StringView server_topic() const { return server_topic_.view(); }
    SetResult set_server_topic(StringView value) { return server_topic_.set(value); }
    StringView filename() const { return filename_.view(); }
    SetResult set_filename(StringView value) { return filename_.set(value); }
    uint32_t crash_seq() const { return crash_seq_; }
    SetResult set_crash_seq(uint32_t value) { crash_seq_ = value; return SetResult::kOk; }
    uint8_t fault_type() const { return fault_type_; }
    SetResult set_fault_type(uint8_t value) { fault_type_ = value; return SetResult::kOk; }
    uint32_t pc() const { return pc_; }
    SetResult set_pc(uint32_t value) { pc_ = value; return SetResult::kOk; }
    uint32_t lr() const { return lr_; }
    SetResult set_lr(uint32_t value) { lr_ = value; return SetResult::kOk; }
    uint32_t dump_size() const { return dump_size_; }
    SetResult set_dump_size(uint32_t value) { dump_size_ = value; return SetResult::kOk; }
    uint32_t dump_crc32() const { return dump_crc32_; }
    SetResult set_dump_crc32(uint32_t value) { dump_crc32_ = value; return SetResult::kOk; }
    bool uploaded() const { return uploaded_; }
    SetResult set_uploaded(bool value) { uploaded_ = value; return SetResult::kOk; }

private:
    template <typename U> friend struct tlv::Codec;
    BoundedString<32> server_topic_{};
    BoundedString<32> filename_{};
    uint32_t crash_seq_{};
    uint8_t fault_type_{};
    uint32_t pc_{};
    uint32_t lr_{};
    uint32_t dump_size_{};
    uint32_t dump_crc32_{};
    bool uploaded_{};
};

class CrashDumpV1 {
public:
    CrashDumpV1() {
        uptime_ms_ = 0;
        crash_seq_ = 0;
    }
    uint32_t magic() const { return magic_; }
    SetResult set_magic(uint32_t value) { magic_ = value; return SetResult::kOk; }
    uint16_t version() const { return version_; }
    SetResult set_version(uint16_t value) { version_ = value; return SetResult::kOk; }
    uint16_t size() const { return size_; }
    SetResult set_size(uint16_t value) { size_ = value; return SetResult::kOk; }
    uint32_t crc32() const { return crc32_; }
    SetResult set_crc32(uint32_t value) { crc32_ = value; return SetResult::kOk; }
    StringView fw_version() const { return fw_version_.view(); }
    SetResult set_fw_version(StringView value) { return fw_version_.set(value); }
    uint8_t fault_type() const { return fault_type_; }
    SetResult set_fault_type(uint8_t value) { fault_type_ = value; return SetResult::kOk; }
    uint8_t stack_depth() const { return stack_depth_; }
    SetResult set_stack_depth(uint8_t value) { stack_depth_ = value; return SetResult::kOk; }
    uint32_t uptime_ms() const { return uptime_ms_; }
    SetResult set_uptime_ms(uint32_t value) { uptime_ms_ = value; return SetResult::kOk; }
    uint32_t crash_seq() const { return crash_seq_; }
    SetResult set_crash_seq(uint32_t value) { crash_seq_ = value; return SetResult::kOk; }
    Span<const uint8_t> core_registers() const { return core_registers_.view(); }
    SetResult set_core_registers(Span<const uint8_t> value) { return core_registers_.set(value); }
    Span<const uint8_t> fault_registers() const { return fault_registers_.view(); }
    SetResult set_fault_registers(Span<const uint8_t> value) { return fault_registers_.set(value); }
    StringView task_name() const { return task_name_.view(); }
    SetResult set_task_name(StringView value) { return task_name_.set(value); }
    Span<const uint8_t> frames() const { return frames_.view(); }
    SetResult set_frames(Span<const uint8_t> value) { return frames_.set(value); }
    Span<const uint8_t> stack_snapshot() const { return stack_snapshot_.view(); }
    SetResult set_stack_snapshot(Span<const uint8_t> value) { return stack_snapshot_.set(value); }
    Span<const uint8_t> task_snapshot() const { return task_snapshot_.view(); }
    SetResult set_task_snapshot(Span<const uint8_t> value) { return task_snapshot_.set(value); }
    Span<const uint8_t> user_data() const { return user_data_.view(); }
    SetResult set_user_data(Span<const uint8_t> value) { return user_data_.set(value); }

private:
    template <typename U> friend struct tlv::Codec;
    uint32_t magic_{};
    uint16_t version_{};
    uint16_t size_{};
    uint32_t crc32_{};
    BoundedString<64> fw_version_{};
    uint8_t fault_type_{};
    uint8_t stack_depth_{};
    uint32_t uptime_ms_{};
    uint32_t crash_seq_{};
    BoundedVector<uint8_t, 96> core_registers_{};
    BoundedVector<uint8_t, 20> fault_registers_{};
    BoundedString<16> task_name_{};
    BoundedVector<uint8_t, 256> frames_{};
    BoundedVector<uint8_t, 2048> stack_snapshot_{};
    BoundedVector<uint8_t, 4676> task_snapshot_{};
    BoundedVector<uint8_t, 256> user_data_{};
};

class DeviceStatusTinyV1 {
public:
    uint32_t uptime_ms() const { return uptime_ms_; }
    SetResult set_uptime_ms(uint32_t value) { uptime_ms_ = value; return SetResult::kOk; }
    uint16_t voltage_mv() const { return voltage_mv_; }
    SetResult set_voltage_mv(uint16_t value) { voltage_mv_ = value; return SetResult::kOk; }
    uint8_t flags() const { return flags_; }
    SetResult set_flags(uint8_t value) { flags_ = value; return SetResult::kOk; }

private:
    template <typename U> friend struct tlv::Codec;
    uint32_t uptime_ms_{};
    uint16_t voltage_mv_{};
    uint8_t flags_{};
};

class DeviceStatusV1 {
public:
    uint32_t uptime_ms() const { return uptime_ms_; }
    SetResult set_uptime_ms(uint32_t value) { uptime_ms_ = value; return SetResult::kOk; }
    uint16_t voltage_mv() const { return voltage_mv_; }
    SetResult set_voltage_mv(uint16_t value) { voltage_mv_ = value; return SetResult::kOk; }
    uint8_t flags() const { return flags_; }
    SetResult set_flags(uint8_t value) { flags_ = value; return SetResult::kOk; }

private:
    template <typename U> friend struct tlv::Codec;
    uint32_t uptime_ms_{};
    uint16_t voltage_mv_{};
    uint8_t flags_{};
};

class DummyDeviceStatusV1 {
public:
    uint16_t schema_major() const { return schema_major_; }
    SetResult set_schema_major(uint16_t value) { schema_major_ = value; return SetResult::kOk; }
    uint16_t schema_minor() const { return schema_minor_; }
    SetResult set_schema_minor(uint16_t value) { schema_minor_ = value; return SetResult::kOk; }
    const FixedArray<uint8_t, 16>& device_id() const { return device_id_; }
    SetResult set_device_id(Span<const uint8_t> value) { return device_id_.set(value); }
    uint64_t boot_epoch() const { return boot_epoch_; }
    SetResult set_boot_epoch(uint64_t value) { boot_epoch_ = value; return SetResult::kOk; }
    uint64_t session_id() const { return session_id_; }
    SetResult set_session_id(uint64_t value) { session_id_ = value; return SetResult::kOk; }
    const FixedArray<uint8_t, 16>& client_instance_id() const { return client_instance_id_; }
    SetResult set_client_instance_id(Span<const uint8_t> value) { return client_instance_id_.set(value); }
    uint32_t reference_epoch() const { return reference_epoch_; }
    SetResult set_reference_epoch(uint32_t value) { reference_epoch_ = value; return SetResult::kOk; }
    uint32_t motion_epoch() const { return motion_epoch_; }
    SetResult set_motion_epoch(uint32_t value) { motion_epoch_ = value; return SetResult::kOk; }
    uint32_t management_epoch() const { return management_epoch_; }
    SetResult set_management_epoch(uint32_t value) { management_epoch_ = value; return SetResult::kOk; }
    uint32_t profile_id() const { return profile_id_; }
    SetResult set_profile_id(uint32_t value) { profile_id_ = value; return SetResult::kOk; }
    uint16_t core_state() const { return core_state_; }
    SetResult set_core_state(uint16_t value) { core_state_ = value; return SetResult::kOk; }
    uint16_t fault_reason() const { return fault_reason_; }
    SetResult set_fault_reason(uint16_t value) { fault_reason_ = value; return SetResult::kOk; }
    uint8_t control_mode() const { return control_mode_; }
    SetResult set_control_mode(uint8_t value) { control_mode_ = value; return SetResult::kOk; }
    uint8_t reference_kind() const { return reference_kind_; }
    SetResult set_reference_kind(uint8_t value) { reference_kind_ = value; return SetResult::kOk; }
    uint8_t drive_request() const { return drive_request_; }
    SetResult set_drive_request(uint8_t value) { drive_request_ = value; return SetResult::kOk; }
    uint8_t drive_evidence() const { return drive_evidence_; }
    SetResult set_drive_evidence(uint8_t value) { drive_evidence_ = value; return SetResult::kOk; }
    uint8_t configured_mask() const { return configured_mask_; }
    SetResult set_configured_mask(uint8_t value) { configured_mask_ = value; return SetResult::kOk; }
    bool real_motion_permitted() const { return real_motion_permitted_; }
    SetResult set_real_motion_permitted(bool value) { real_motion_permitted_ = value; return SetResult::kOk; }
    uint64_t permit_remaining_us() const { return permit_remaining_us_; }
    SetResult set_permit_remaining_us(uint64_t value) { permit_remaining_us_ = value; return SetResult::kOk; }
    uint64_t capabilities() const { return capabilities_; }
    SetResult set_capabilities(uint64_t value) { capabilities_ = value; return SetResult::kOk; }
    uint64_t operation_id() const { return operation_id_; }
    SetResult set_operation_id(uint64_t value) { operation_id_ = value; return SetResult::kOk; }
    uint16_t operation_status() const { return operation_status_; }
    SetResult set_operation_status(uint16_t value) { operation_status_ = value; return SetResult::kOk; }
    uint16_t operation_result_code() const { return operation_result_code_; }
    SetResult set_operation_result_code(uint16_t value) { operation_result_code_ = value; return SetResult::kOk; }
    bool operation_result_valid() const { return operation_result_valid_; }
    SetResult set_operation_result_valid(bool value) { operation_result_valid_ = value; return SetResult::kOk; }
    uint32_t command_reject_count() const { return command_reject_count_; }
    SetResult set_command_reject_count(uint32_t value) { command_reject_count_ = value; return SetResult::kOk; }
    uint32_t can_error_count() const { return can_error_count_; }
    SetResult set_can_error_count(uint32_t value) { can_error_count_ = value; return SetResult::kOk; }
    uint32_t deadline_miss_count() const { return deadline_miss_count_; }
    SetResult set_deadline_miss_count(uint32_t value) { deadline_miss_count_ = value; return SetResult::kOk; }
    const FixedArray<uint8_t, 32>& contract_sha256() const { return contract_sha256_; }
    SetResult set_contract_sha256(Span<const uint8_t> value) { return contract_sha256_.set(value); }
    const FixedArray<uint8_t, 32>& model_sha256() const { return model_sha256_; }
    SetResult set_model_sha256(Span<const uint8_t> value) { return model_sha256_.set(value); }
    const FixedArray<uint8_t, 32>& profile_sha256() const { return profile_sha256_; }
    SetResult set_profile_sha256(Span<const uint8_t> value) { return profile_sha256_.set(value); }
    const FixedArray<uint8_t, 32>& activation_profile_sha256() const { return activation_profile_sha256_; }
    SetResult set_activation_profile_sha256(Span<const uint8_t> value) { return activation_profile_sha256_.set(value); }
    bool motion_supported() const { return motion_supported_; }
    SetResult set_motion_supported(bool value) { motion_supported_ = value; return SetResult::kOk; }
    bool hold_ready() const { return hold_ready_; }
    SetResult set_hold_ready(bool value) { hold_ready_ = value; return SetResult::kOk; }
    const FixedArray<uint8_t, 32>& firmware_sha256() const { return firmware_sha256_; }
    SetResult set_firmware_sha256(Span<const uint8_t> value) { return firmware_sha256_.set(value); }
    const FixedArray<uint8_t, 16>& board_name_ascii() const { return board_name_ascii_; }
    SetResult set_board_name_ascii(Span<const uint8_t> value) { return board_name_ascii_.set(value); }

private:
    template <typename U> friend struct tlv::Codec;
    uint16_t schema_major_{};
    uint16_t schema_minor_{};
    FixedArray<uint8_t, 16> device_id_{};
    uint64_t boot_epoch_{};
    uint64_t session_id_{};
    FixedArray<uint8_t, 16> client_instance_id_{};
    uint32_t reference_epoch_{};
    uint32_t motion_epoch_{};
    uint32_t management_epoch_{};
    uint32_t profile_id_{};
    uint16_t core_state_{};
    uint16_t fault_reason_{};
    uint8_t control_mode_{};
    uint8_t reference_kind_{};
    uint8_t drive_request_{};
    uint8_t drive_evidence_{};
    uint8_t configured_mask_{};
    bool real_motion_permitted_{};
    uint64_t permit_remaining_us_{};
    uint64_t capabilities_{};
    uint64_t operation_id_{};
    uint16_t operation_status_{};
    uint16_t operation_result_code_{};
    bool operation_result_valid_{};
    uint32_t command_reject_count_{};
    uint32_t can_error_count_{};
    uint32_t deadline_miss_count_{};
    FixedArray<uint8_t, 32> contract_sha256_{};
    FixedArray<uint8_t, 32> model_sha256_{};
    FixedArray<uint8_t, 32> profile_sha256_{};
    FixedArray<uint8_t, 32> activation_profile_sha256_{};
    bool motion_supported_{};
    bool hold_ready_{};
    FixedArray<uint8_t, 32> firmware_sha256_{};
    FixedArray<uint8_t, 16> board_name_ascii_{};
};

class DummyJointCommandV1 {
public:
    uint16_t schema_major() const { return schema_major_; }
    SetResult set_schema_major(uint16_t value) { schema_major_ = value; return SetResult::kOk; }
    uint16_t schema_minor() const { return schema_minor_; }
    SetResult set_schema_minor(uint16_t value) { schema_minor_ = value; return SetResult::kOk; }
    const FixedArray<uint8_t, 16>& device_id() const { return device_id_; }
    SetResult set_device_id(Span<const uint8_t> value) { return device_id_.set(value); }
    uint64_t boot_epoch() const { return boot_epoch_; }
    SetResult set_boot_epoch(uint64_t value) { boot_epoch_ = value; return SetResult::kOk; }
    uint64_t session_id() const { return session_id_; }
    SetResult set_session_id(uint64_t value) { session_id_ = value; return SetResult::kOk; }
    const FixedArray<uint8_t, 16>& client_instance_id() const { return client_instance_id_; }
    SetResult set_client_instance_id(Span<const uint8_t> value) { return client_instance_id_.set(value); }
    uint32_t reference_epoch() const { return reference_epoch_; }
    SetResult set_reference_epoch(uint32_t value) { reference_epoch_ = value; return SetResult::kOk; }
    uint32_t motion_epoch() const { return motion_epoch_; }
    SetResult set_motion_epoch(uint32_t value) { motion_epoch_ = value; return SetResult::kOk; }
    uint32_t profile_id() const { return profile_id_; }
    SetResult set_profile_id(uint32_t value) { profile_id_ = value; return SetResult::kOk; }
    uint32_t producer_seq() const { return producer_seq_; }
    SetResult set_producer_seq(uint32_t value) { producer_seq_ = value; return SetResult::kOk; }
    uint64_t freshness_ticket() const { return freshness_ticket_; }
    SetResult set_freshness_ticket(uint64_t value) { freshness_ticket_ = value; return SetResult::kOk; }
    uint8_t configured_mask() const { return configured_mask_; }
    SetResult set_configured_mask(uint8_t value) { configured_mask_ = value; return SetResult::kOk; }
    const FixedArray<double, 6>& position_rad() const { return position_rad_; }
    SetResult set_position_rad(Span<const double> value) { return position_rad_.set(value); }

private:
    template <typename U> friend struct tlv::Codec;
    uint16_t schema_major_{};
    uint16_t schema_minor_{};
    FixedArray<uint8_t, 16> device_id_{};
    uint64_t boot_epoch_{};
    uint64_t session_id_{};
    FixedArray<uint8_t, 16> client_instance_id_{};
    uint32_t reference_epoch_{};
    uint32_t motion_epoch_{};
    uint32_t profile_id_{};
    uint32_t producer_seq_{};
    uint64_t freshness_ticket_{};
    uint8_t configured_mask_{};
    FixedArray<double, 6> position_rad_{};
};

class DummyJointStateV1 {
public:
    uint16_t schema_major() const { return schema_major_; }
    SetResult set_schema_major(uint16_t value) { schema_major_ = value; return SetResult::kOk; }
    uint16_t schema_minor() const { return schema_minor_; }
    SetResult set_schema_minor(uint16_t value) { schema_minor_ = value; return SetResult::kOk; }
    const FixedArray<uint8_t, 16>& device_id() const { return device_id_; }
    SetResult set_device_id(Span<const uint8_t> value) { return device_id_.set(value); }
    uint64_t boot_epoch() const { return boot_epoch_; }
    SetResult set_boot_epoch(uint64_t value) { boot_epoch_ = value; return SetResult::kOk; }
    uint64_t session_id() const { return session_id_; }
    SetResult set_session_id(uint64_t value) { session_id_ = value; return SetResult::kOk; }
    const FixedArray<uint8_t, 16>& client_instance_id() const { return client_instance_id_; }
    SetResult set_client_instance_id(Span<const uint8_t> value) { return client_instance_id_.set(value); }
    uint32_t reference_epoch() const { return reference_epoch_; }
    SetResult set_reference_epoch(uint32_t value) { reference_epoch_ = value; return SetResult::kOk; }
    uint32_t motion_epoch() const { return motion_epoch_; }
    SetResult set_motion_epoch(uint32_t value) { motion_epoch_ = value; return SetResult::kOk; }
    uint32_t management_epoch() const { return management_epoch_; }
    SetResult set_management_epoch(uint32_t value) { management_epoch_ = value; return SetResult::kOk; }
    uint32_t profile_id() const { return profile_id_; }
    SetResult set_profile_id(uint32_t value) { profile_id_ = value; return SetResult::kOk; }
    uint32_t state_seq() const { return state_seq_; }
    SetResult set_state_seq(uint32_t value) { state_seq_ = value; return SetResult::kOk; }
    uint64_t produced_at_us() const { return produced_at_us_; }
    SetResult set_produced_at_us(uint64_t value) { produced_at_us_ = value; return SetResult::kOk; }
    uint64_t freshness_ticket() const { return freshness_ticket_; }
    SetResult set_freshness_ticket(uint64_t value) { freshness_ticket_ = value; return SetResult::kOk; }
    uint8_t configured_mask() const { return configured_mask_; }
    SetResult set_configured_mask(uint8_t value) { configured_mask_ = value; return SetResult::kOk; }
    uint8_t position_valid_mask() const { return position_valid_mask_; }
    SetResult set_position_valid_mask(uint8_t value) { position_valid_mask_ = value; return SetResult::kOk; }
    uint8_t velocity_valid_mask() const { return velocity_valid_mask_; }
    SetResult set_velocity_valid_mask(uint8_t value) { velocity_valid_mask_ = value; return SetResult::kOk; }
    uint8_t planner_finish_mask() const { return planner_finish_mask_; }
    SetResult set_planner_finish_mask(uint8_t value) { planner_finish_mask_ = value; return SetResult::kOk; }
    uint8_t hold_valid_mask() const { return hold_valid_mask_; }
    SetResult set_hold_valid_mask(uint8_t value) { hold_valid_mask_ = value; return SetResult::kOk; }
    bool accepted_seq_valid() const { return accepted_seq_valid_; }
    SetResult set_accepted_seq_valid(bool value) { accepted_seq_valid_ = value; return SetResult::kOk; }
    uint32_t accepted_producer_seq() const { return accepted_producer_seq_; }
    SetResult set_accepted_producer_seq(uint32_t value) { accepted_producer_seq_ = value; return SetResult::kOk; }
    bool applied_seq_valid() const { return applied_seq_valid_; }
    SetResult set_applied_seq_valid(bool value) { applied_seq_valid_ = value; return SetResult::kOk; }
    uint32_t applied_producer_seq() const { return applied_producer_seq_; }
    SetResult set_applied_producer_seq(uint32_t value) { applied_producer_seq_ = value; return SetResult::kOk; }
    bool tx_group_seq_valid() const { return tx_group_seq_valid_; }
    SetResult set_tx_group_seq_valid(bool value) { tx_group_seq_valid_ = value; return SetResult::kOk; }
    uint32_t tx_group_producer_seq() const { return tx_group_producer_seq_; }
    SetResult set_tx_group_producer_seq(uint32_t value) { tx_group_producer_seq_ = value; return SetResult::kOk; }
    uint8_t submitted_axis_mask() const { return submitted_axis_mask_; }
    SetResult set_submitted_axis_mask(uint8_t value) { submitted_axis_mask_ = value; return SetResult::kOk; }
    uint8_t sent_axis_mask() const { return sent_axis_mask_; }
    SetResult set_sent_axis_mask(uint8_t value) { sent_axis_mask_ = value; return SetResult::kOk; }
    const FixedArray<uint32_t, 6>& feedback_seq() const { return feedback_seq_; }
    SetResult set_feedback_seq(Span<const uint32_t> value) { return feedback_seq_.set(value); }
    const FixedArray<uint32_t, 6>& can_rx_age_us() const { return can_rx_age_us_; }
    SetResult set_can_rx_age_us(Span<const uint32_t> value) { return can_rx_age_us_.set(value); }
    const FixedArray<double, 6>& position_rad() const { return position_rad_; }
    SetResult set_position_rad(Span<const double> value) { return position_rad_.set(value); }
    const FixedArray<double, 6>& estimated_velocity_rad_s() const { return estimated_velocity_rad_s_; }
    SetResult set_estimated_velocity_rad_s(Span<const double> value) { return estimated_velocity_rad_s_.set(value); }
    const FixedArray<double, 6>& hold_position_rad() const { return hold_position_rad_; }
    SetResult set_hold_position_rad(Span<const double> value) { return hold_position_rad_.set(value); }
    uint16_t core_state() const { return core_state_; }
    SetResult set_core_state(uint16_t value) { core_state_ = value; return SetResult::kOk; }
    uint16_t fault_reason() const { return fault_reason_; }
    SetResult set_fault_reason(uint16_t value) { fault_reason_ = value; return SetResult::kOk; }
    bool hold_ready() const { return hold_ready_; }
    SetResult set_hold_ready(bool value) { hold_ready_ = value; return SetResult::kOk; }
    uint8_t raw_position_valid_mask() const { return raw_position_valid_mask_; }
    SetResult set_raw_position_valid_mask(uint8_t value) { raw_position_valid_mask_ = value; return SetResult::kOk; }
    const FixedArray<double, 6>& raw_motor_position_rev() const { return raw_motor_position_rev_; }
    SetResult set_raw_motor_position_rev(Span<const double> value) { return raw_motor_position_rev_.set(value); }

private:
    template <typename U> friend struct tlv::Codec;
    uint16_t schema_major_{};
    uint16_t schema_minor_{};
    FixedArray<uint8_t, 16> device_id_{};
    uint64_t boot_epoch_{};
    uint64_t session_id_{};
    FixedArray<uint8_t, 16> client_instance_id_{};
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
    uint8_t planner_finish_mask_{};
    uint8_t hold_valid_mask_{};
    bool accepted_seq_valid_{};
    uint32_t accepted_producer_seq_{};
    bool applied_seq_valid_{};
    uint32_t applied_producer_seq_{};
    bool tx_group_seq_valid_{};
    uint32_t tx_group_producer_seq_{};
    uint8_t submitted_axis_mask_{};
    uint8_t sent_axis_mask_{};
    FixedArray<uint32_t, 6> feedback_seq_{};
    FixedArray<uint32_t, 6> can_rx_age_us_{};
    FixedArray<double, 6> position_rad_{};
    FixedArray<double, 6> estimated_velocity_rad_s_{};
    FixedArray<double, 6> hold_position_rad_{};
    uint16_t core_state_{};
    uint16_t fault_reason_{};
    bool hold_ready_{};
    uint8_t raw_position_valid_mask_{};
    FixedArray<double, 6> raw_motor_position_rev_{};
};

class HeartbeatV1 {
public:
    uint32_t uptime_ms() const { return uptime_ms_; }
    SetResult set_uptime_ms(uint32_t value) { uptime_ms_ = value; return SetResult::kOk; }
    uint32_t health_flags() const { return health_flags_; }
    SetResult set_health_flags(uint32_t value) { health_flags_ = value; return SetResult::kOk; }

private:
    template <typename U> friend struct tlv::Codec;
    uint32_t uptime_ms_{};
    uint32_t health_flags_{};
};

class JointCommandFrameV1 {
public:
    double motor_position() const { return motor_position_; }
    SetResult set_motor_position(double value) { motor_position_ = value; return SetResult::kOk; }

private:
    template <typename U> friend struct tlv::Codec;
    double motor_position_{};
};

class JointStateFrameV1 {
public:
    double motor_position() const { return motor_position_; }
    SetResult set_motor_position(double value) { motor_position_ = value; return SetResult::kOk; }
    double pendulum_position() const { return pendulum_position_; }
    SetResult set_pendulum_position(double value) { pendulum_position_ = value; return SetResult::kOk; }

private:
    template <typename U> friend struct tlv::Codec;
    double motor_position_{};
    double pendulum_position_{};
};

class KeyRotateV1 {
public:
    uint32_t key_epoch() const { return key_epoch_; }
    SetResult set_key_epoch(uint32_t value) { key_epoch_ = value; return SetResult::kOk; }
    uint8_t current_key_id() const { return current_key_id_; }
    SetResult set_current_key_id(uint8_t value) { current_key_id_ = value; return SetResult::kOk; }
    uint8_t next_key_id() const { return next_key_id_; }
    SetResult set_next_key_id(uint8_t value) { next_key_id_ = value; return SetResult::kOk; }
    uint8_t phase() const { return phase_; }
    SetResult set_phase(uint8_t value) { phase_ = value; return SetResult::kOk; }
    uint16_t key_slot_id() const { return key_slot_id_; }
    SetResult set_key_slot_id(uint16_t value) { key_slot_id_ = value; return SetResult::kOk; }
    uint32_t activate_after_ms() const { return activate_after_ms_; }
    SetResult set_activate_after_ms(uint32_t value) { activate_after_ms_ = value; return SetResult::kOk; }

private:
    template <typename U> friend struct tlv::Codec;
    uint32_t key_epoch_{};
    uint8_t current_key_id_{};
    uint8_t next_key_id_{};
    uint8_t phase_{};
    uint16_t key_slot_id_{};
    uint32_t activate_after_ms_{};
};

class LinkHelloV1 {
public:
    uint8_t protocol_major() const { return protocol_major_; }
    SetResult set_protocol_major(uint8_t value) { protocol_major_ = value; return SetResult::kOk; }
    uint8_t protocol_minor() const { return protocol_minor_; }
    SetResult set_protocol_minor(uint8_t value) { protocol_minor_ = value; return SetResult::kOk; }
    uint32_t product_id() const { return product_id_; }
    SetResult set_product_id(uint32_t value) { product_id_ = value; return SetResult::kOk; }
    uint32_t boot_counter() const { return boot_counter_; }
    SetResult set_boot_counter(uint32_t value) { boot_counter_ = value; return SetResult::kOk; }
    const FixedArray<uint8_t, 16>& device_id() const { return device_id_; }
    SetResult set_device_id(Span<const uint8_t> value) { return device_id_.set(value); }
    const FixedArray<uint8_t, 16>& catalog_fingerprint() const { return catalog_fingerprint_; }
    SetResult set_catalog_fingerprint(Span<const uint8_t> value) { return catalog_fingerprint_.set(value); }
    uint8_t security_profile() const { return security_profile_; }
    SetResult set_security_profile(uint8_t value) { security_profile_ = value; return SetResult::kOk; }
    uint16_t max_payload_size() const { return max_payload_size_; }
    SetResult set_max_payload_size(uint16_t value) { max_payload_size_ = value; return SetResult::kOk; }

private:
    template <typename U> friend struct tlv::Codec;
    uint8_t protocol_major_{};
    uint8_t protocol_minor_{};
    uint32_t product_id_{};
    uint32_t boot_counter_{};
    FixedArray<uint8_t, 16> device_id_{};
    FixedArray<uint8_t, 16> catalog_fingerprint_{};
    uint8_t security_profile_{};
    uint16_t max_payload_size_{};
};

class LinkResetV1 {
public:
    uint32_t reset_generation() const { return reset_generation_; }
    SetResult set_reset_generation(uint32_t value) { reset_generation_ = value; return SetResult::kOk; }
    uint8_t reason() const { return reason_; }
    SetResult set_reason(uint8_t value) { reason_ = value; return SetResult::kOk; }
    uint8_t mode() const { return mode_; }
    SetResult set_mode(uint8_t value) { mode_ = value; return SetResult::kOk; }

private:
    template <typename U> friend struct tlv::Codec;
    uint32_t reset_generation_{};
    uint8_t reason_{};
    uint8_t mode_{};
};

class LogRecordTinyV1 {
public:
    uint32_t timestamp_ms() const { return timestamp_ms_; }
    SetResult set_timestamp_ms(uint32_t value) { timestamp_ms_ = value; return SetResult::kOk; }
    uint8_t level() const { return level_; }
    SetResult set_level(uint8_t value) { level_ = value; return SetResult::kOk; }
    uint16_t tag_id() const { return tag_id_; }
    SetResult set_tag_id(uint16_t value) { tag_id_ = value; return SetResult::kOk; }
    Span<const uint8_t> payload() const { return payload_.view(); }
    SetResult set_payload(Span<const uint8_t> value) { return payload_.set(value); }

private:
    template <typename U> friend struct tlv::Codec;
    uint32_t timestamp_ms_{};
    uint8_t level_{};
    uint16_t tag_id_{};
    BoundedVector<uint8_t, 64> payload_{};
};

class LogRecordV1 {
public:
    uint32_t timestamp_ms() const { return timestamp_ms_; }
    SetResult set_timestamp_ms(uint32_t value) { timestamp_ms_ = value; return SetResult::kOk; }
    uint8_t level() const { return level_; }
    SetResult set_level(uint8_t value) { level_ = value; return SetResult::kOk; }
    uint16_t tag_id() const { return tag_id_; }
    SetResult set_tag_id(uint16_t value) { tag_id_ = value; return SetResult::kOk; }
    Span<const uint8_t> payload() const { return payload_.view(); }
    SetResult set_payload(Span<const uint8_t> value) { return payload_.set(value); }

private:
    template <typename U> friend struct tlv::Codec;
    uint32_t timestamp_ms_{};
    uint8_t level_{};
    uint16_t tag_id_{};
    BoundedVector<uint8_t, 480> payload_{};
};

class OfflineLogCommandAckV1 {
public:
    bool ok() const { return ok_; }
    SetResult set_ok(bool value) { ok_ = value; return SetResult::kOk; }
    StringView command() const { return command_.view(); }
    SetResult set_command(StringView value) { return command_.set(value); }
    StringView detail() const { return detail_.view(); }
    SetResult set_detail(StringView value) { return detail_.set(value); }

private:
    template <typename U> friend struct tlv::Codec;
    bool ok_{};
    BoundedString<64> command_{};
    BoundedString<192> detail_{};
};

class OfflineLogCommandV1 {
public:
    StringView command() const { return command_.view(); }
    SetResult set_command(StringView value) { return command_.set(value); }
    StringView request_id() const { return request_id_.view(); }
    SetResult set_request_id(StringView value) { return request_id_.set(value); }

private:
    template <typename U> friend struct tlv::Codec;
    BoundedString<320> command_{};
    BoundedString<64> request_id_{};
};

class RemoteLevelV1 {
public:
    StringView device_id() const { return device_id_.view(); }
    SetResult set_device_id(StringView value) { return device_id_.set(value); }
    StringView level() const { return level_.view(); }
    SetResult set_level(StringView value) { return level_.set(value); }
    StringView command() const { return command_.view(); }
    SetResult set_command(StringView value) { return command_.set(value); }

private:
    template <typename U> friend struct tlv::Codec;
    BoundedString<64> device_id_{};
    BoundedString<16> level_{};
    BoundedString<16> command_{};
};

class CapabilityQueryV1Request {
public:
    Span<const uint8_t> device_id() const { return device_id_.view(); }
    SetResult set_device_id(Span<const uint8_t> value) { return device_id_.set(value); }

private:
    template <typename U> friend struct tlv::Codec;
    BoundedVector<uint8_t, 16> device_id_{};
};

class CapabilityQueryV1Response {
public:
    CapabilityQueryV1Response() {
        protocol_version_ = 0;
    }
    Span<const uint8_t> device_id() const { return device_id_.view(); }
    SetResult set_device_id(Span<const uint8_t> value) { return device_id_.set(value); }
    uint32_t protocol_version() const { return protocol_version_; }
    SetResult set_protocol_version(uint32_t value) { protocol_version_ = value; return SetResult::kOk; }
    const FixedArray<uint8_t, 16>& capabilities() const { return capabilities_; }
    SetResult set_capabilities(Span<const uint8_t> value) { return capabilities_.set(value); }

private:
    template <typename U> friend struct tlv::Codec;
    BoundedVector<uint8_t, 16> device_id_{};
    uint32_t protocol_version_{};
    FixedArray<uint8_t, 16> capabilities_{};
};

class DummyManageV1Request {
public:
    uint16_t schema_major() const { return schema_major_; }
    SetResult set_schema_major(uint16_t value) { schema_major_ = value; return SetResult::kOk; }
    uint16_t schema_minor() const { return schema_minor_; }
    SetResult set_schema_minor(uint16_t value) { schema_minor_ = value; return SetResult::kOk; }
    const FixedArray<uint8_t, 16>& device_id() const { return device_id_; }
    SetResult set_device_id(Span<const uint8_t> value) { return device_id_.set(value); }
    uint64_t expected_boot_epoch() const { return expected_boot_epoch_; }
    SetResult set_expected_boot_epoch(uint64_t value) { expected_boot_epoch_ = value; return SetResult::kOk; }
    uint64_t expected_session_id() const { return expected_session_id_; }
    SetResult set_expected_session_id(uint64_t value) { expected_session_id_ = value; return SetResult::kOk; }
    uint32_t expected_reference_epoch() const { return expected_reference_epoch_; }
    SetResult set_expected_reference_epoch(uint32_t value) { expected_reference_epoch_ = value; return SetResult::kOk; }
    uint32_t expected_motion_epoch() const { return expected_motion_epoch_; }
    SetResult set_expected_motion_epoch(uint32_t value) { expected_motion_epoch_ = value; return SetResult::kOk; }
    uint32_t expected_management_epoch() const { return expected_management_epoch_; }
    SetResult set_expected_management_epoch(uint32_t value) { expected_management_epoch_ = value; return SetResult::kOk; }
    uint32_t expected_profile_id() const { return expected_profile_id_; }
    SetResult set_expected_profile_id(uint32_t value) { expected_profile_id_ = value; return SetResult::kOk; }
    const FixedArray<uint8_t, 16>& client_instance_id() const { return client_instance_id_; }
    SetResult set_client_instance_id(Span<const uint8_t> value) { return client_instance_id_.set(value); }
    uint64_t request_id() const { return request_id_; }
    SetResult set_request_id(uint64_t value) { request_id_ = value; return SetResult::kOk; }
    uint32_t request_seq() const { return request_seq_; }
    SetResult set_request_seq(uint32_t value) { request_seq_ = value; return SetResult::kOk; }
    uint64_t freshness_ticket() const { return freshness_ticket_; }
    SetResult set_freshness_ticket(uint64_t value) { freshness_ticket_ = value; return SetResult::kOk; }
    uint64_t queried_operation_id() const { return queried_operation_id_; }
    SetResult set_queried_operation_id(uint64_t value) { queried_operation_id_ = value; return SetResult::kOk; }
    uint64_t queried_request_id() const { return queried_request_id_; }
    SetResult set_queried_request_id(uint64_t value) { queried_request_id_ = value; return SetResult::kOk; }
    uint16_t operation() const { return operation_; }
    SetResult set_operation(uint16_t value) { operation_ = value; return SetResult::kOk; }
    uint8_t requested_control_mode() const { return requested_control_mode_; }
    SetResult set_requested_control_mode(uint8_t value) { requested_control_mode_ = value; return SetResult::kOk; }
    uint8_t configured_mask() const { return configured_mask_; }
    SetResult set_configured_mask(uint8_t value) { configured_mask_ = value; return SetResult::kOk; }
    bool operator_confirmed() const { return operator_confirmed_; }
    SetResult set_operator_confirmed(bool value) { operator_confirmed_ = value; return SetResult::kOk; }
    const FixedArray<uint8_t, 32>& permit_sha256() const { return permit_sha256_; }
    SetResult set_permit_sha256(Span<const uint8_t> value) { return permit_sha256_.set(value); }
    const FixedArray<uint8_t, 32>& compiled_policy_sha256() const { return compiled_policy_sha256_; }
    SetResult set_compiled_policy_sha256(Span<const uint8_t> value) { return compiled_policy_sha256_.set(value); }
    uint64_t permit_remaining_us() const { return permit_remaining_us_; }
    SetResult set_permit_remaining_us(uint64_t value) { permit_remaining_us_ = value; return SetResult::kOk; }
    uint32_t terminal_producer_seq() const { return terminal_producer_seq_; }
    SetResult set_terminal_producer_seq(uint32_t value) { terminal_producer_seq_ = value; return SetResult::kOk; }
    const FixedArray<double, 6>& terminal_position_rad() const { return terminal_position_rad_; }
    SetResult set_terminal_position_rad(Span<const double> value) { return terminal_position_rad_.set(value); }

private:
    template <typename U> friend struct tlv::Codec;
    uint16_t schema_major_{};
    uint16_t schema_minor_{};
    FixedArray<uint8_t, 16> device_id_{};
    uint64_t expected_boot_epoch_{};
    uint64_t expected_session_id_{};
    uint32_t expected_reference_epoch_{};
    uint32_t expected_motion_epoch_{};
    uint32_t expected_management_epoch_{};
    uint32_t expected_profile_id_{};
    FixedArray<uint8_t, 16> client_instance_id_{};
    uint64_t request_id_{};
    uint32_t request_seq_{};
    uint64_t freshness_ticket_{};
    uint64_t queried_operation_id_{};
    uint64_t queried_request_id_{};
    uint16_t operation_{};
    uint8_t requested_control_mode_{};
    uint8_t configured_mask_{};
    bool operator_confirmed_{};
    FixedArray<uint8_t, 32> permit_sha256_{};
    FixedArray<uint8_t, 32> compiled_policy_sha256_{};
    uint64_t permit_remaining_us_{};
    uint32_t terminal_producer_seq_{};
    FixedArray<double, 6> terminal_position_rad_{};
};

class DummyManageV1Response {
public:
    uint16_t schema_major() const { return schema_major_; }
    SetResult set_schema_major(uint16_t value) { schema_major_ = value; return SetResult::kOk; }
    uint16_t schema_minor() const { return schema_minor_; }
    SetResult set_schema_minor(uint16_t value) { schema_minor_ = value; return SetResult::kOk; }
    const FixedArray<uint8_t, 16>& device_id() const { return device_id_; }
    SetResult set_device_id(Span<const uint8_t> value) { return device_id_.set(value); }
    const FixedArray<uint8_t, 16>& client_instance_id() const { return client_instance_id_; }
    SetResult set_client_instance_id(Span<const uint8_t> value) { return client_instance_id_.set(value); }
    const FixedArray<uint8_t, 16>& owner_client_instance_id() const { return owner_client_instance_id_; }
    SetResult set_owner_client_instance_id(Span<const uint8_t> value) { return owner_client_instance_id_.set(value); }
    uint64_t request_id() const { return request_id_; }
    SetResult set_request_id(uint64_t value) { request_id_ = value; return SetResult::kOk; }
    uint8_t receipt() const { return receipt_; }
    SetResult set_receipt(uint8_t value) { receipt_ = value; return SetResult::kOk; }
    uint64_t core_request_id() const { return core_request_id_; }
    SetResult set_core_request_id(uint64_t value) { core_request_id_ = value; return SetResult::kOk; }
    bool accepted() const { return accepted_; }
    SetResult set_accepted(bool value) { accepted_ = value; return SetResult::kOk; }
    uint16_t result_code() const { return result_code_; }
    SetResult set_result_code(uint16_t value) { result_code_ = value; return SetResult::kOk; }
    bool operation_result_valid() const { return operation_result_valid_; }
    SetResult set_operation_result_valid(bool value) { operation_result_valid_ = value; return SetResult::kOk; }
    uint64_t operation_id() const { return operation_id_; }
    SetResult set_operation_id(uint64_t value) { operation_id_ = value; return SetResult::kOk; }
    uint16_t operation_status() const { return operation_status_; }
    SetResult set_operation_status(uint16_t value) { operation_status_ = value; return SetResult::kOk; }
    bool snapshot_valid() const { return snapshot_valid_; }
    SetResult set_snapshot_valid(bool value) { snapshot_valid_ = value; return SetResult::kOk; }
    bool info_valid() const { return info_valid_; }
    SetResult set_info_valid(bool value) { info_valid_ = value; return SetResult::kOk; }
    bool status_valid() const { return status_valid_; }
    SetResult set_status_valid(bool value) { status_valid_ = value; return SetResult::kOk; }
    uint64_t boot_epoch() const { return boot_epoch_; }
    SetResult set_boot_epoch(uint64_t value) { boot_epoch_ = value; return SetResult::kOk; }
    uint64_t session_id() const { return session_id_; }
    SetResult set_session_id(uint64_t value) { session_id_ = value; return SetResult::kOk; }
    uint32_t reference_epoch() const { return reference_epoch_; }
    SetResult set_reference_epoch(uint32_t value) { reference_epoch_ = value; return SetResult::kOk; }
    uint32_t motion_epoch() const { return motion_epoch_; }
    SetResult set_motion_epoch(uint32_t value) { motion_epoch_ = value; return SetResult::kOk; }
    uint32_t management_epoch() const { return management_epoch_; }
    SetResult set_management_epoch(uint32_t value) { management_epoch_ = value; return SetResult::kOk; }
    uint32_t profile_id() const { return profile_id_; }
    SetResult set_profile_id(uint32_t value) { profile_id_ = value; return SetResult::kOk; }
    uint32_t state_seq() const { return state_seq_; }
    SetResult set_state_seq(uint32_t value) { state_seq_ = value; return SetResult::kOk; }
    uint64_t produced_at_us() const { return produced_at_us_; }
    SetResult set_produced_at_us(uint64_t value) { produced_at_us_ = value; return SetResult::kOk; }
    uint64_t freshness_ticket() const { return freshness_ticket_; }
    SetResult set_freshness_ticket(uint64_t value) { freshness_ticket_ = value; return SetResult::kOk; }
    uint8_t configured_mask() const { return configured_mask_; }
    SetResult set_configured_mask(uint8_t value) { configured_mask_ = value; return SetResult::kOk; }
    uint8_t position_valid_mask() const { return position_valid_mask_; }
    SetResult set_position_valid_mask(uint8_t value) { position_valid_mask_ = value; return SetResult::kOk; }
    uint8_t velocity_valid_mask() const { return velocity_valid_mask_; }
    SetResult set_velocity_valid_mask(uint8_t value) { velocity_valid_mask_ = value; return SetResult::kOk; }
    uint8_t planner_finish_mask() const { return planner_finish_mask_; }
    SetResult set_planner_finish_mask(uint8_t value) { planner_finish_mask_ = value; return SetResult::kOk; }
    uint8_t hold_valid_mask() const { return hold_valid_mask_; }
    SetResult set_hold_valid_mask(uint8_t value) { hold_valid_mask_ = value; return SetResult::kOk; }
    bool accepted_seq_valid() const { return accepted_seq_valid_; }
    SetResult set_accepted_seq_valid(bool value) { accepted_seq_valid_ = value; return SetResult::kOk; }
    uint32_t accepted_producer_seq() const { return accepted_producer_seq_; }
    SetResult set_accepted_producer_seq(uint32_t value) { accepted_producer_seq_ = value; return SetResult::kOk; }
    bool applied_seq_valid() const { return applied_seq_valid_; }
    SetResult set_applied_seq_valid(bool value) { applied_seq_valid_ = value; return SetResult::kOk; }
    uint32_t applied_producer_seq() const { return applied_producer_seq_; }
    SetResult set_applied_producer_seq(uint32_t value) { applied_producer_seq_ = value; return SetResult::kOk; }
    bool tx_group_seq_valid() const { return tx_group_seq_valid_; }
    SetResult set_tx_group_seq_valid(bool value) { tx_group_seq_valid_ = value; return SetResult::kOk; }
    uint32_t tx_group_producer_seq() const { return tx_group_producer_seq_; }
    SetResult set_tx_group_producer_seq(uint32_t value) { tx_group_producer_seq_ = value; return SetResult::kOk; }
    uint8_t submitted_axis_mask() const { return submitted_axis_mask_; }
    SetResult set_submitted_axis_mask(uint8_t value) { submitted_axis_mask_ = value; return SetResult::kOk; }
    uint8_t sent_axis_mask() const { return sent_axis_mask_; }
    SetResult set_sent_axis_mask(uint8_t value) { sent_axis_mask_ = value; return SetResult::kOk; }
    const FixedArray<uint32_t, 6>& feedback_seq() const { return feedback_seq_; }
    SetResult set_feedback_seq(Span<const uint32_t> value) { return feedback_seq_.set(value); }
    const FixedArray<uint32_t, 6>& can_rx_age_us() const { return can_rx_age_us_; }
    SetResult set_can_rx_age_us(Span<const uint32_t> value) { return can_rx_age_us_.set(value); }
    const FixedArray<double, 6>& position_rad() const { return position_rad_; }
    SetResult set_position_rad(Span<const double> value) { return position_rad_.set(value); }
    const FixedArray<double, 6>& estimated_velocity_rad_s() const { return estimated_velocity_rad_s_; }
    SetResult set_estimated_velocity_rad_s(Span<const double> value) { return estimated_velocity_rad_s_.set(value); }
    const FixedArray<double, 6>& hold_position_rad() const { return hold_position_rad_; }
    SetResult set_hold_position_rad(Span<const double> value) { return hold_position_rad_.set(value); }
    uint16_t core_state() const { return core_state_; }
    SetResult set_core_state(uint16_t value) { core_state_ = value; return SetResult::kOk; }
    uint16_t fault_reason() const { return fault_reason_; }
    SetResult set_fault_reason(uint16_t value) { fault_reason_ = value; return SetResult::kOk; }
    bool hold_ready() const { return hold_ready_; }
    SetResult set_hold_ready(bool value) { hold_ready_ = value; return SetResult::kOk; }
    uint8_t raw_position_valid_mask() const { return raw_position_valid_mask_; }
    SetResult set_raw_position_valid_mask(uint8_t value) { raw_position_valid_mask_ = value; return SetResult::kOk; }
    const FixedArray<double, 6>& raw_motor_position_rev() const { return raw_motor_position_rev_; }
    SetResult set_raw_motor_position_rev(Span<const double> value) { return raw_motor_position_rev_.set(value); }
    uint8_t control_mode() const { return control_mode_; }
    SetResult set_control_mode(uint8_t value) { control_mode_ = value; return SetResult::kOk; }
    uint8_t reference_kind() const { return reference_kind_; }
    SetResult set_reference_kind(uint8_t value) { reference_kind_ = value; return SetResult::kOk; }
    uint8_t drive_request() const { return drive_request_; }
    SetResult set_drive_request(uint8_t value) { drive_request_ = value; return SetResult::kOk; }
    uint8_t drive_evidence() const { return drive_evidence_; }
    SetResult set_drive_evidence(uint8_t value) { drive_evidence_ = value; return SetResult::kOk; }
    bool real_motion_permitted() const { return real_motion_permitted_; }
    SetResult set_real_motion_permitted(bool value) { real_motion_permitted_ = value; return SetResult::kOk; }
    uint64_t permit_remaining_us() const { return permit_remaining_us_; }
    SetResult set_permit_remaining_us(uint64_t value) { permit_remaining_us_ = value; return SetResult::kOk; }
    uint64_t capabilities() const { return capabilities_; }
    SetResult set_capabilities(uint64_t value) { capabilities_ = value; return SetResult::kOk; }
    uint16_t operation_result_code() const { return operation_result_code_; }
    SetResult set_operation_result_code(uint16_t value) { operation_result_code_ = value; return SetResult::kOk; }
    uint32_t command_reject_count() const { return command_reject_count_; }
    SetResult set_command_reject_count(uint32_t value) { command_reject_count_ = value; return SetResult::kOk; }
    uint32_t can_error_count() const { return can_error_count_; }
    SetResult set_can_error_count(uint32_t value) { can_error_count_ = value; return SetResult::kOk; }
    uint32_t deadline_miss_count() const { return deadline_miss_count_; }
    SetResult set_deadline_miss_count(uint32_t value) { deadline_miss_count_ = value; return SetResult::kOk; }
    const FixedArray<uint8_t, 32>& contract_sha256() const { return contract_sha256_; }
    SetResult set_contract_sha256(Span<const uint8_t> value) { return contract_sha256_.set(value); }
    const FixedArray<uint8_t, 32>& model_sha256() const { return model_sha256_; }
    SetResult set_model_sha256(Span<const uint8_t> value) { return model_sha256_.set(value); }
    const FixedArray<uint8_t, 32>& profile_sha256() const { return profile_sha256_; }
    SetResult set_profile_sha256(Span<const uint8_t> value) { return profile_sha256_.set(value); }
    const FixedArray<uint8_t, 32>& activation_profile_sha256() const { return activation_profile_sha256_; }
    SetResult set_activation_profile_sha256(Span<const uint8_t> value) { return activation_profile_sha256_.set(value); }
    bool motion_supported() const { return motion_supported_; }
    SetResult set_motion_supported(bool value) { motion_supported_ = value; return SetResult::kOk; }
    const FixedArray<uint8_t, 32>& firmware_sha256() const { return firmware_sha256_; }
    SetResult set_firmware_sha256(Span<const uint8_t> value) { return firmware_sha256_.set(value); }
    const FixedArray<uint8_t, 16>& board_name_ascii() const { return board_name_ascii_; }
    SetResult set_board_name_ascii(Span<const uint8_t> value) { return board_name_ascii_.set(value); }

private:
    template <typename U> friend struct tlv::Codec;
    uint16_t schema_major_{};
    uint16_t schema_minor_{};
    FixedArray<uint8_t, 16> device_id_{};
    FixedArray<uint8_t, 16> client_instance_id_{};
    FixedArray<uint8_t, 16> owner_client_instance_id_{};
    uint64_t request_id_{};
    uint8_t receipt_{};
    uint64_t core_request_id_{};
    bool accepted_{};
    uint16_t result_code_{};
    bool operation_result_valid_{};
    uint64_t operation_id_{};
    uint16_t operation_status_{};
    bool snapshot_valid_{};
    bool info_valid_{};
    bool status_valid_{};
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
    uint8_t planner_finish_mask_{};
    uint8_t hold_valid_mask_{};
    bool accepted_seq_valid_{};
    uint32_t accepted_producer_seq_{};
    bool applied_seq_valid_{};
    uint32_t applied_producer_seq_{};
    bool tx_group_seq_valid_{};
    uint32_t tx_group_producer_seq_{};
    uint8_t submitted_axis_mask_{};
    uint8_t sent_axis_mask_{};
    FixedArray<uint32_t, 6> feedback_seq_{};
    FixedArray<uint32_t, 6> can_rx_age_us_{};
    FixedArray<double, 6> position_rad_{};
    FixedArray<double, 6> estimated_velocity_rad_s_{};
    FixedArray<double, 6> hold_position_rad_{};
    uint16_t core_state_{};
    uint16_t fault_reason_{};
    bool hold_ready_{};
    uint8_t raw_position_valid_mask_{};
    FixedArray<double, 6> raw_motor_position_rev_{};
    uint8_t control_mode_{};
    uint8_t reference_kind_{};
    uint8_t drive_request_{};
    uint8_t drive_evidence_{};
    bool real_motion_permitted_{};
    uint64_t permit_remaining_us_{};
    uint64_t capabilities_{};
    uint16_t operation_result_code_{};
    uint32_t command_reject_count_{};
    uint32_t can_error_count_{};
    uint32_t deadline_miss_count_{};
    FixedArray<uint8_t, 32> contract_sha256_{};
    FixedArray<uint8_t, 32> model_sha256_{};
    FixedArray<uint8_t, 32> profile_sha256_{};
    FixedArray<uint8_t, 32> activation_profile_sha256_{};
    bool motion_supported_{};
    FixedArray<uint8_t, 32> firmware_sha256_{};
    FixedArray<uint8_t, 16> board_name_ascii_{};
};

class FileTransferCancelV1Request {
public:
    uint8_t opcode() const { return opcode_; }
    SetResult set_opcode(uint8_t value) { opcode_ = value; return SetResult::kOk; }
    uint32_t transfer_id() const { return transfer_id_; }
    SetResult set_transfer_id(uint32_t value) { transfer_id_ = value; return SetResult::kOk; }

private:
    template <typename U> friend struct tlv::Codec;
    uint8_t opcode_{};
    uint32_t transfer_id_{};
};

class FileTransferCancelV1Response {
public:
    uint8_t opcode() const { return opcode_; }
    SetResult set_opcode(uint8_t value) { opcode_ = value; return SetResult::kOk; }
    uint8_t status() const { return status_; }
    SetResult set_status(uint8_t value) { status_ = value; return SetResult::kOk; }
    uint32_t transfer_id() const { return transfer_id_; }
    SetResult set_transfer_id(uint32_t value) { transfer_id_ = value; return SetResult::kOk; }

private:
    template <typename U> friend struct tlv::Codec;
    uint8_t opcode_{};
    uint8_t status_{};
    uint32_t transfer_id_{};
};

class FileTransferChunkV1Request {
public:
    uint8_t opcode() const { return opcode_; }
    SetResult set_opcode(uint8_t value) { opcode_ = value; return SetResult::kOk; }
    uint32_t transfer_id() const { return transfer_id_; }
    SetResult set_transfer_id(uint32_t value) { transfer_id_ = value; return SetResult::kOk; }
    uint32_t offset() const { return offset_; }
    SetResult set_offset(uint32_t value) { offset_ = value; return SetResult::kOk; }
    uint16_t length() const { return length_; }
    SetResult set_length(uint16_t value) { length_ = value; return SetResult::kOk; }

private:
    template <typename U> friend struct tlv::Codec;
    uint8_t opcode_{};
    uint32_t transfer_id_{};
    uint32_t offset_{};
    uint16_t length_{};
};

class FileTransferChunkV1Response {
public:
    uint8_t opcode() const { return opcode_; }
    SetResult set_opcode(uint8_t value) { opcode_ = value; return SetResult::kOk; }
    uint8_t status() const { return status_; }
    SetResult set_status(uint8_t value) { status_ = value; return SetResult::kOk; }
    uint32_t transfer_id() const { return transfer_id_; }
    SetResult set_transfer_id(uint32_t value) { transfer_id_ = value; return SetResult::kOk; }
    uint32_t offset() const { return offset_; }
    SetResult set_offset(uint32_t value) { offset_ = value; return SetResult::kOk; }
    uint16_t length() const { return length_; }
    SetResult set_length(uint16_t value) { length_ = value; return SetResult::kOk; }
    Span<const uint8_t> data() const { return data_.view(); }
    SetResult set_data(Span<const uint8_t> value) { return data_.set(value); }

private:
    template <typename U> friend struct tlv::Codec;
    uint8_t opcode_{};
    uint8_t status_{};
    uint32_t transfer_id_{};
    uint32_t offset_{};
    uint16_t length_{};
    BoundedVector<uint8_t, 64> data_{};
};

class FileTransferCloseV1Request {
public:
    uint8_t opcode() const { return opcode_; }
    SetResult set_opcode(uint8_t value) { opcode_ = value; return SetResult::kOk; }
    uint32_t transfer_id() const { return transfer_id_; }
    SetResult set_transfer_id(uint32_t value) { transfer_id_ = value; return SetResult::kOk; }
    uint32_t received_crc32() const { return received_crc32_; }
    SetResult set_received_crc32(uint32_t value) { received_crc32_ = value; return SetResult::kOk; }
    uint32_t total_bytes() const { return total_bytes_; }
    SetResult set_total_bytes(uint32_t value) { total_bytes_ = value; return SetResult::kOk; }

private:
    template <typename U> friend struct tlv::Codec;
    uint8_t opcode_{};
    uint32_t transfer_id_{};
    uint32_t received_crc32_{};
    uint32_t total_bytes_{};
};

class FileTransferCloseV1Response {
public:
    uint8_t opcode() const { return opcode_; }
    SetResult set_opcode(uint8_t value) { opcode_ = value; return SetResult::kOk; }
    uint8_t status() const { return status_; }
    SetResult set_status(uint8_t value) { status_ = value; return SetResult::kOk; }
    uint32_t transfer_id() const { return transfer_id_; }
    SetResult set_transfer_id(uint32_t value) { transfer_id_ = value; return SetResult::kOk; }

private:
    template <typename U> friend struct tlv::Codec;
    uint8_t opcode_{};
    uint8_t status_{};
    uint32_t transfer_id_{};
};

class FileTransferOpenV1Request {
public:
    uint8_t opcode() const { return opcode_; }
    SetResult set_opcode(uint8_t value) { opcode_ = value; return SetResult::kOk; }
    Span<const uint8_t> filename() const { return filename_.view(); }
    SetResult set_filename(Span<const uint8_t> value) { return filename_.set(value); }
    uint32_t resume_offset() const { return resume_offset_; }
    SetResult set_resume_offset(uint32_t value) { resume_offset_ = value; return SetResult::kOk; }
    uint16_t max_chunk_size() const { return max_chunk_size_; }
    SetResult set_max_chunk_size(uint16_t value) { max_chunk_size_ = value; return SetResult::kOk; }

private:
    template <typename U> friend struct tlv::Codec;
    uint8_t opcode_{};
    BoundedVector<uint8_t, 64> filename_{};
    uint32_t resume_offset_{};
    uint16_t max_chunk_size_{};
};

class FileTransferOpenV1Response {
public:
    uint8_t opcode() const { return opcode_; }
    SetResult set_opcode(uint8_t value) { opcode_ = value; return SetResult::kOk; }
    uint8_t status() const { return status_; }
    SetResult set_status(uint8_t value) { status_ = value; return SetResult::kOk; }
    uint32_t transfer_id() const { return transfer_id_; }
    SetResult set_transfer_id(uint32_t value) { transfer_id_ = value; return SetResult::kOk; }
    uint32_t file_size() const { return file_size_; }
    SetResult set_file_size(uint32_t value) { file_size_ = value; return SetResult::kOk; }
    uint32_t file_crc32() const { return file_crc32_; }
    SetResult set_file_crc32(uint32_t value) { file_crc32_ = value; return SetResult::kOk; }
    uint16_t chunk_size() const { return chunk_size_; }
    SetResult set_chunk_size(uint16_t value) { chunk_size_ = value; return SetResult::kOk; }

private:
    template <typename U> friend struct tlv::Codec;
    uint8_t opcode_{};
    uint8_t status_{};
    uint32_t transfer_id_{};
    uint32_t file_size_{};
    uint32_t file_crc32_{};
    uint16_t chunk_size_{};
};

class OfflineLogCommandRpcV1Request {
public:
    StringView command() const { return command_.view(); }
    SetResult set_command(StringView value) { return command_.set(value); }

private:
    template <typename U> friend struct tlv::Codec;
    BoundedString<320> command_{};
};

class OfflineLogCommandRpcV1Response {
public:
    OfflineLogCommandRpcV1Response() {
        ok_ = false;
        status_ = 0;
    }
    bool ok() const { return ok_; }
    SetResult set_ok(bool value) { ok_ = value; return SetResult::kOk; }
    int32_t status() const { return status_; }
    SetResult set_status(int32_t value) { status_ = value; return SetResult::kOk; }
    StringView detail() const { return detail_.view(); }
    SetResult set_detail(StringView value) { return detail_.set(value); }

private:
    template <typename U> friend struct tlv::Codec;
    bool ok_{};
    int32_t status_{};
    BoundedString<256> detail_{};
};

class OfflineLogCommandV2Request {
public:
    StringView request_id() const { return request_id_.view(); }
    SetResult set_request_id(StringView value) { return request_id_.set(value); }
    StringView command() const { return command_.view(); }
    SetResult set_command(StringView value) { return command_.set(value); }
    StringView name() const { return name_.view(); }
    SetResult set_name(StringView value) { return name_.set(value); }

private:
    template <typename U> friend struct tlv::Codec;
    BoundedString<64> request_id_{};
    BoundedString<320> command_{};
    BoundedString<120> name_{};
};

class OfflineLogCommandV2Response {
public:
    OfflineLogCommandV2Response() {
        status_ = 0;
    }
    StringView request_id() const { return request_id_.view(); }
    SetResult set_request_id(StringView value) { return request_id_.set(value); }
    StringView command() const { return command_.view(); }
    SetResult set_command(StringView value) { return command_.set(value); }
    bool ok() const { return ok_; }
    SetResult set_ok(bool value) { ok_ = value; return SetResult::kOk; }
    int32_t status() const { return status_; }
    SetResult set_status(int32_t value) { status_ = value; return SetResult::kOk; }
    StringView detail() const { return detail_.view(); }
    SetResult set_detail(StringView value) { return detail_.set(value); }

private:
    template <typename U> friend struct tlv::Codec;
    BoundedString<64> request_id_{};
    BoundedString<320> command_{};
    bool ok_{};
    int32_t status_{};
    BoundedString<256> detail_{};
};

}  // namespace dp::iface
