// Auto-generated dp_comm adapters for ROS-IDL portable types. Do not edit.
#pragma once
#include "dp_comm/dp_comm.hpp"
#include "dp_protocol_tlv.hpp"

namespace dp::comm {
template <>
struct MessageCodec<dp::iface::CapabilityAnnounceV1> {
    using Value = dp::iface::CapabilityAnnounceV1;
    using TlvCodec = dp::iface::tlv::Codec<Value>;
    static constexpr size_t kMaxEncodedSize = TlvCodec::kMaxEncodedSize;
    static constexpr bool kPortable = true;
    static constexpr uint32_t kInterfaceId = 4101U;
    static bool encode(const Value& input, uint8_t* output, size_t capacity, size_t* used) {
        return TlvCodec::encode(input, output, capacity, used);
    }
    static bool decode(const uint8_t* input, size_t size, Value* output) {
        Value scratch{};
        return TlvCodec::decode(input, size, output, &scratch);
    }
};

template <>
struct MessageCodec<dp::iface::CapabilityBitmapV1> {
    using Value = dp::iface::CapabilityBitmapV1;
    using TlvCodec = dp::iface::tlv::Codec<Value>;
    static constexpr size_t kMaxEncodedSize = TlvCodec::kMaxEncodedSize;
    static constexpr bool kPortable = true;
    static constexpr uint32_t kInterfaceId = 61442U;
    static bool encode(const Value& input, uint8_t* output, size_t capacity, size_t* used) {
        return TlvCodec::encode(input, output, capacity, used);
    }
    static bool decode(const uint8_t* input, size_t size, Value* output) {
        Value scratch{};
        return TlvCodec::decode(input, size, output, &scratch);
    }
};

template <>
struct MessageCodec<dp::iface::ControlWatchdogV1> {
    using Value = dp::iface::ControlWatchdogV1;
    using TlvCodec = dp::iface::tlv::Codec<Value>;
    static constexpr size_t kMaxEncodedSize = TlvCodec::kMaxEncodedSize;
    static constexpr bool kPortable = true;
    static constexpr uint32_t kInterfaceId = 4104U;
    static bool encode(const Value& input, uint8_t* output, size_t capacity, size_t* used) {
        return TlvCodec::encode(input, output, capacity, used);
    }
    static bool decode(const uint8_t* input, size_t size, Value* output) {
        Value scratch{};
        return TlvCodec::decode(input, size, output, &scratch);
    }
};

template <>
struct MessageCodec<dp::iface::CrashAnnounceV1> {
    using Value = dp::iface::CrashAnnounceV1;
    using TlvCodec = dp::iface::tlv::Codec<Value>;
    static constexpr size_t kMaxEncodedSize = TlvCodec::kMaxEncodedSize;
    static constexpr bool kPortable = true;
    static constexpr uint32_t kInterfaceId = 4105U;
    static bool encode(const Value& input, uint8_t* output, size_t capacity, size_t* used) {
        return TlvCodec::encode(input, output, capacity, used);
    }
    static bool decode(const uint8_t* input, size_t size, Value* output) {
        Value scratch{};
        return TlvCodec::decode(input, size, output, &scratch);
    }
};

template <>
struct MessageCodec<dp::iface::CrashDumpV1> {
    using Value = dp::iface::CrashDumpV1;
    using TlvCodec = dp::iface::tlv::Codec<Value>;
    static constexpr size_t kMaxEncodedSize = TlvCodec::kMaxEncodedSize;
    static constexpr bool kPortable = true;
    static constexpr uint32_t kInterfaceId = 4106U;
    static bool encode(const Value& input, uint8_t* output, size_t capacity, size_t* used) {
        return TlvCodec::encode(input, output, capacity, used);
    }
    static bool decode(const uint8_t* input, size_t size, Value* output) {
        Value scratch{};
        return TlvCodec::decode(input, size, output, &scratch);
    }
};

template <>
struct MessageCodec<dp::iface::DeviceStatusTinyV1> {
    using Value = dp::iface::DeviceStatusTinyV1;
    using TlvCodec = dp::iface::tlv::Codec<Value>;
    static constexpr size_t kMaxEncodedSize = TlvCodec::kMaxEncodedSize;
    static constexpr bool kPortable = true;
    static constexpr uint32_t kInterfaceId = 4100U;
    static bool encode(const Value& input, uint8_t* output, size_t capacity, size_t* used) {
        return TlvCodec::encode(input, output, capacity, used);
    }
    static bool decode(const uint8_t* input, size_t size, Value* output) {
        Value scratch{};
        return TlvCodec::decode(input, size, output, &scratch);
    }
};

template <>
struct MessageCodec<dp::iface::DeviceStatusV1> {
    using Value = dp::iface::DeviceStatusV1;
    using TlvCodec = dp::iface::tlv::Codec<Value>;
    static constexpr size_t kMaxEncodedSize = TlvCodec::kMaxEncodedSize;
    static constexpr bool kPortable = true;
    static constexpr uint32_t kInterfaceId = 4099U;
    static bool encode(const Value& input, uint8_t* output, size_t capacity, size_t* used) {
        return TlvCodec::encode(input, output, capacity, used);
    }
    static bool decode(const uint8_t* input, size_t size, Value* output) {
        Value scratch{};
        return TlvCodec::decode(input, size, output, &scratch);
    }
};

template <>
struct MessageCodec<dp::iface::DummyDeviceStatusV1> {
    using Value = dp::iface::DummyDeviceStatusV1;
    using TlvCodec = dp::iface::tlv::Codec<Value>;
    static constexpr size_t kMaxEncodedSize = TlvCodec::kMaxEncodedSize;
    static constexpr bool kPortable = true;
    static constexpr uint32_t kInterfaceId = 4131U;
    static bool encode(const Value& input, uint8_t* output, size_t capacity, size_t* used) {
        return TlvCodec::encode(input, output, capacity, used);
    }
    static bool decode(const uint8_t* input, size_t size, Value* output) {
        Value scratch{};
        return TlvCodec::decode(input, size, output, &scratch);
    }
};

template <>
struct MessageCodec<dp::iface::DummyJointCommandV1> {
    using Value = dp::iface::DummyJointCommandV1;
    using TlvCodec = dp::iface::tlv::Codec<Value>;
    static constexpr size_t kMaxEncodedSize = TlvCodec::kMaxEncodedSize;
    static constexpr bool kPortable = true;
    static constexpr uint32_t kInterfaceId = 4132U;
    static bool encode(const Value& input, uint8_t* output, size_t capacity, size_t* used) {
        return TlvCodec::encode(input, output, capacity, used);
    }
    static bool decode(const uint8_t* input, size_t size, Value* output) {
        Value scratch{};
        return TlvCodec::decode(input, size, output, &scratch);
    }
};

template <>
struct MessageCodec<dp::iface::DummyJointStateV1> {
    using Value = dp::iface::DummyJointStateV1;
    using TlvCodec = dp::iface::tlv::Codec<Value>;
    static constexpr size_t kMaxEncodedSize = TlvCodec::kMaxEncodedSize;
    static constexpr bool kPortable = true;
    static constexpr uint32_t kInterfaceId = 4133U;
    static bool encode(const Value& input, uint8_t* output, size_t capacity, size_t* used) {
        return TlvCodec::encode(input, output, capacity, used);
    }
    static bool decode(const uint8_t* input, size_t size, Value* output) {
        Value scratch{};
        return TlvCodec::decode(input, size, output, &scratch);
    }
};

template <>
struct MessageCodec<dp::iface::HeartbeatV1> {
    using Value = dp::iface::HeartbeatV1;
    using TlvCodec = dp::iface::tlv::Codec<Value>;
    static constexpr size_t kMaxEncodedSize = TlvCodec::kMaxEncodedSize;
    static constexpr bool kPortable = true;
    static constexpr uint32_t kInterfaceId = 61444U;
    static bool encode(const Value& input, uint8_t* output, size_t capacity, size_t* used) {
        return TlvCodec::encode(input, output, capacity, used);
    }
    static bool decode(const uint8_t* input, size_t size, Value* output) {
        Value scratch{};
        return TlvCodec::decode(input, size, output, &scratch);
    }
};

template <>
struct MessageCodec<dp::iface::JointCommandFrameV1> {
    using Value = dp::iface::JointCommandFrameV1;
    using TlvCodec = dp::iface::tlv::Codec<Value>;
    static constexpr size_t kMaxEncodedSize = TlvCodec::kMaxEncodedSize;
    static constexpr bool kPortable = true;
    static constexpr uint32_t kInterfaceId = 4098U;
    static bool encode(const Value& input, uint8_t* output, size_t capacity, size_t* used) {
        return TlvCodec::encode(input, output, capacity, used);
    }
    static bool decode(const uint8_t* input, size_t size, Value* output) {
        Value scratch{};
        return TlvCodec::decode(input, size, output, &scratch);
    }
};

template <>
struct MessageCodec<dp::iface::JointStateFrameV1> {
    using Value = dp::iface::JointStateFrameV1;
    using TlvCodec = dp::iface::tlv::Codec<Value>;
    static constexpr size_t kMaxEncodedSize = TlvCodec::kMaxEncodedSize;
    static constexpr bool kPortable = true;
    static constexpr uint32_t kInterfaceId = 4097U;
    static bool encode(const Value& input, uint8_t* output, size_t capacity, size_t* used) {
        return TlvCodec::encode(input, output, capacity, used);
    }
    static bool decode(const uint8_t* input, size_t size, Value* output) {
        Value scratch{};
        return TlvCodec::decode(input, size, output, &scratch);
    }
};

template <>
struct MessageCodec<dp::iface::KeyRotateV1> {
    using Value = dp::iface::KeyRotateV1;
    using TlvCodec = dp::iface::tlv::Codec<Value>;
    static constexpr size_t kMaxEncodedSize = TlvCodec::kMaxEncodedSize;
    static constexpr bool kPortable = true;
    static constexpr uint32_t kInterfaceId = 61445U;
    static bool encode(const Value& input, uint8_t* output, size_t capacity, size_t* used) {
        return TlvCodec::encode(input, output, capacity, used);
    }
    static bool decode(const uint8_t* input, size_t size, Value* output) {
        Value scratch{};
        return TlvCodec::decode(input, size, output, &scratch);
    }
};

template <>
struct MessageCodec<dp::iface::LinkHelloV1> {
    using Value = dp::iface::LinkHelloV1;
    using TlvCodec = dp::iface::tlv::Codec<Value>;
    static constexpr size_t kMaxEncodedSize = TlvCodec::kMaxEncodedSize;
    static constexpr bool kPortable = true;
    static constexpr uint32_t kInterfaceId = 61441U;
    static bool encode(const Value& input, uint8_t* output, size_t capacity, size_t* used) {
        return TlvCodec::encode(input, output, capacity, used);
    }
    static bool decode(const uint8_t* input, size_t size, Value* output) {
        Value scratch{};
        return TlvCodec::decode(input, size, output, &scratch);
    }
};

template <>
struct MessageCodec<dp::iface::LinkResetV1> {
    using Value = dp::iface::LinkResetV1;
    using TlvCodec = dp::iface::tlv::Codec<Value>;
    static constexpr size_t kMaxEncodedSize = TlvCodec::kMaxEncodedSize;
    static constexpr bool kPortable = true;
    static constexpr uint32_t kInterfaceId = 61443U;
    static bool encode(const Value& input, uint8_t* output, size_t capacity, size_t* used) {
        return TlvCodec::encode(input, output, capacity, used);
    }
    static bool decode(const uint8_t* input, size_t size, Value* output) {
        Value scratch{};
        return TlvCodec::decode(input, size, output, &scratch);
    }
};

template <>
struct MessageCodec<dp::iface::LogRecordTinyV1> {
    using Value = dp::iface::LogRecordTinyV1;
    using TlvCodec = dp::iface::tlv::Codec<Value>;
    static constexpr size_t kMaxEncodedSize = TlvCodec::kMaxEncodedSize;
    static constexpr bool kPortable = true;
    static constexpr uint32_t kInterfaceId = 4130U;
    static bool encode(const Value& input, uint8_t* output, size_t capacity, size_t* used) {
        return TlvCodec::encode(input, output, capacity, used);
    }
    static bool decode(const uint8_t* input, size_t size, Value* output) {
        Value scratch{};
        return TlvCodec::decode(input, size, output, &scratch);
    }
};

template <>
struct MessageCodec<dp::iface::LogRecordV1> {
    using Value = dp::iface::LogRecordV1;
    using TlvCodec = dp::iface::tlv::Codec<Value>;
    static constexpr size_t kMaxEncodedSize = TlvCodec::kMaxEncodedSize;
    static constexpr bool kPortable = true;
    static constexpr uint32_t kInterfaceId = 4102U;
    static bool encode(const Value& input, uint8_t* output, size_t capacity, size_t* used) {
        return TlvCodec::encode(input, output, capacity, used);
    }
    static bool decode(const uint8_t* input, size_t size, Value* output) {
        Value scratch{};
        return TlvCodec::decode(input, size, output, &scratch);
    }
};

template <>
struct MessageCodec<dp::iface::OfflineLogCommandAckV1> {
    using Value = dp::iface::OfflineLogCommandAckV1;
    using TlvCodec = dp::iface::tlv::Codec<Value>;
    static constexpr size_t kMaxEncodedSize = TlvCodec::kMaxEncodedSize;
    static constexpr bool kPortable = true;
    static constexpr uint32_t kInterfaceId = 4123U;
    static bool encode(const Value& input, uint8_t* output, size_t capacity, size_t* used) {
        return TlvCodec::encode(input, output, capacity, used);
    }
    static bool decode(const uint8_t* input, size_t size, Value* output) {
        Value scratch{};
        return TlvCodec::decode(input, size, output, &scratch);
    }
};

template <>
struct MessageCodec<dp::iface::OfflineLogCommandV1> {
    using Value = dp::iface::OfflineLogCommandV1;
    using TlvCodec = dp::iface::tlv::Codec<Value>;
    static constexpr size_t kMaxEncodedSize = TlvCodec::kMaxEncodedSize;
    static constexpr bool kPortable = true;
    static constexpr uint32_t kInterfaceId = 4122U;
    static bool encode(const Value& input, uint8_t* output, size_t capacity, size_t* used) {
        return TlvCodec::encode(input, output, capacity, used);
    }
    static bool decode(const uint8_t* input, size_t size, Value* output) {
        Value scratch{};
        return TlvCodec::decode(input, size, output, &scratch);
    }
};

template <>
struct MessageCodec<dp::iface::RemoteLevelV1> {
    using Value = dp::iface::RemoteLevelV1;
    using TlvCodec = dp::iface::tlv::Codec<Value>;
    static constexpr size_t kMaxEncodedSize = TlvCodec::kMaxEncodedSize;
    static constexpr bool kPortable = true;
    static constexpr uint32_t kInterfaceId = 4103U;
    static bool encode(const Value& input, uint8_t* output, size_t capacity, size_t* used) {
        return TlvCodec::encode(input, output, capacity, used);
    }
    static bool decode(const uint8_t* input, size_t size, Value* output) {
        Value scratch{};
        return TlvCodec::decode(input, size, output, &scratch);
    }
};

template <>
struct MessageCodec<dp::iface::CapabilityQueryV1Request> {
    using Value = dp::iface::CapabilityQueryV1Request;
    using TlvCodec = dp::iface::tlv::Codec<Value>;
    static constexpr size_t kMaxEncodedSize = TlvCodec::kMaxEncodedSize;
    static constexpr bool kPortable = true;
    static constexpr uint32_t kInterfaceId = 4112U;
    static bool encode(const Value& input, uint8_t* output, size_t capacity, size_t* used) {
        return TlvCodec::encode(input, output, capacity, used);
    }
    static bool decode(const uint8_t* input, size_t size, Value* output) {
        Value scratch{};
        return TlvCodec::decode(input, size, output, &scratch);
    }
};

template <>
struct MessageCodec<dp::iface::CapabilityQueryV1Response> {
    using Value = dp::iface::CapabilityQueryV1Response;
    using TlvCodec = dp::iface::tlv::Codec<Value>;
    static constexpr size_t kMaxEncodedSize = TlvCodec::kMaxEncodedSize;
    static constexpr bool kPortable = true;
    static constexpr uint32_t kInterfaceId = 4113U;
    static bool encode(const Value& input, uint8_t* output, size_t capacity, size_t* used) {
        return TlvCodec::encode(input, output, capacity, used);
    }
    static bool decode(const uint8_t* input, size_t size, Value* output) {
        Value scratch{};
        return TlvCodec::decode(input, size, output, &scratch);
    }
};

template <>
struct MessageCodec<dp::iface::DummyManageV1Request> {
    using Value = dp::iface::DummyManageV1Request;
    using TlvCodec = dp::iface::tlv::Codec<Value>;
    static constexpr size_t kMaxEncodedSize = TlvCodec::kMaxEncodedSize;
    static constexpr bool kPortable = true;
    static constexpr uint32_t kInterfaceId = 4135U;
    static bool encode(const Value& input, uint8_t* output, size_t capacity, size_t* used) {
        return TlvCodec::encode(input, output, capacity, used);
    }
    static bool decode(const uint8_t* input, size_t size, Value* output) {
        Value scratch{};
        return TlvCodec::decode(input, size, output, &scratch);
    }
};

template <>
struct MessageCodec<dp::iface::DummyManageV1Response> {
    using Value = dp::iface::DummyManageV1Response;
    using TlvCodec = dp::iface::tlv::Codec<Value>;
    static constexpr size_t kMaxEncodedSize = TlvCodec::kMaxEncodedSize;
    static constexpr bool kPortable = true;
    static constexpr uint32_t kInterfaceId = 4136U;
    static bool encode(const Value& input, uint8_t* output, size_t capacity, size_t* used) {
        return TlvCodec::encode(input, output, capacity, used);
    }
    static bool decode(const uint8_t* input, size_t size, Value* output) {
        Value scratch{};
        return TlvCodec::decode(input, size, output, &scratch);
    }
};

template <>
struct MessageCodec<dp::iface::FileTransferCancelV1Request> {
    using Value = dp::iface::FileTransferCancelV1Request;
    using TlvCodec = dp::iface::tlv::Codec<Value>;
    static constexpr size_t kMaxEncodedSize = TlvCodec::kMaxEncodedSize;
    static constexpr bool kPortable = true;
    static constexpr uint32_t kInterfaceId = 4120U;
    static bool encode(const Value& input, uint8_t* output, size_t capacity, size_t* used) {
        return TlvCodec::encode(input, output, capacity, used);
    }
    static bool decode(const uint8_t* input, size_t size, Value* output) {
        Value scratch{};
        return TlvCodec::decode(input, size, output, &scratch);
    }
};

template <>
struct MessageCodec<dp::iface::FileTransferCancelV1Response> {
    using Value = dp::iface::FileTransferCancelV1Response;
    using TlvCodec = dp::iface::tlv::Codec<Value>;
    static constexpr size_t kMaxEncodedSize = TlvCodec::kMaxEncodedSize;
    static constexpr bool kPortable = true;
    static constexpr uint32_t kInterfaceId = 4121U;
    static bool encode(const Value& input, uint8_t* output, size_t capacity, size_t* used) {
        return TlvCodec::encode(input, output, capacity, used);
    }
    static bool decode(const uint8_t* input, size_t size, Value* output) {
        Value scratch{};
        return TlvCodec::decode(input, size, output, &scratch);
    }
};

template <>
struct MessageCodec<dp::iface::FileTransferChunkV1Request> {
    using Value = dp::iface::FileTransferChunkV1Request;
    using TlvCodec = dp::iface::tlv::Codec<Value>;
    static constexpr size_t kMaxEncodedSize = TlvCodec::kMaxEncodedSize;
    static constexpr bool kPortable = true;
    static constexpr uint32_t kInterfaceId = 4116U;
    static bool encode(const Value& input, uint8_t* output, size_t capacity, size_t* used) {
        return TlvCodec::encode(input, output, capacity, used);
    }
    static bool decode(const uint8_t* input, size_t size, Value* output) {
        Value scratch{};
        return TlvCodec::decode(input, size, output, &scratch);
    }
};

template <>
struct MessageCodec<dp::iface::FileTransferChunkV1Response> {
    using Value = dp::iface::FileTransferChunkV1Response;
    using TlvCodec = dp::iface::tlv::Codec<Value>;
    static constexpr size_t kMaxEncodedSize = TlvCodec::kMaxEncodedSize;
    static constexpr bool kPortable = true;
    static constexpr uint32_t kInterfaceId = 4117U;
    static bool encode(const Value& input, uint8_t* output, size_t capacity, size_t* used) {
        return TlvCodec::encode(input, output, capacity, used);
    }
    static bool decode(const uint8_t* input, size_t size, Value* output) {
        Value scratch{};
        return TlvCodec::decode(input, size, output, &scratch);
    }
};

template <>
struct MessageCodec<dp::iface::FileTransferCloseV1Request> {
    using Value = dp::iface::FileTransferCloseV1Request;
    using TlvCodec = dp::iface::tlv::Codec<Value>;
    static constexpr size_t kMaxEncodedSize = TlvCodec::kMaxEncodedSize;
    static constexpr bool kPortable = true;
    static constexpr uint32_t kInterfaceId = 4118U;
    static bool encode(const Value& input, uint8_t* output, size_t capacity, size_t* used) {
        return TlvCodec::encode(input, output, capacity, used);
    }
    static bool decode(const uint8_t* input, size_t size, Value* output) {
        Value scratch{};
        return TlvCodec::decode(input, size, output, &scratch);
    }
};

template <>
struct MessageCodec<dp::iface::FileTransferCloseV1Response> {
    using Value = dp::iface::FileTransferCloseV1Response;
    using TlvCodec = dp::iface::tlv::Codec<Value>;
    static constexpr size_t kMaxEncodedSize = TlvCodec::kMaxEncodedSize;
    static constexpr bool kPortable = true;
    static constexpr uint32_t kInterfaceId = 4119U;
    static bool encode(const Value& input, uint8_t* output, size_t capacity, size_t* used) {
        return TlvCodec::encode(input, output, capacity, used);
    }
    static bool decode(const uint8_t* input, size_t size, Value* output) {
        Value scratch{};
        return TlvCodec::decode(input, size, output, &scratch);
    }
};

template <>
struct MessageCodec<dp::iface::FileTransferOpenV1Request> {
    using Value = dp::iface::FileTransferOpenV1Request;
    using TlvCodec = dp::iface::tlv::Codec<Value>;
    static constexpr size_t kMaxEncodedSize = TlvCodec::kMaxEncodedSize;
    static constexpr bool kPortable = true;
    static constexpr uint32_t kInterfaceId = 4114U;
    static bool encode(const Value& input, uint8_t* output, size_t capacity, size_t* used) {
        return TlvCodec::encode(input, output, capacity, used);
    }
    static bool decode(const uint8_t* input, size_t size, Value* output) {
        Value scratch{};
        return TlvCodec::decode(input, size, output, &scratch);
    }
};

template <>
struct MessageCodec<dp::iface::FileTransferOpenV1Response> {
    using Value = dp::iface::FileTransferOpenV1Response;
    using TlvCodec = dp::iface::tlv::Codec<Value>;
    static constexpr size_t kMaxEncodedSize = TlvCodec::kMaxEncodedSize;
    static constexpr bool kPortable = true;
    static constexpr uint32_t kInterfaceId = 4115U;
    static bool encode(const Value& input, uint8_t* output, size_t capacity, size_t* used) {
        return TlvCodec::encode(input, output, capacity, used);
    }
    static bool decode(const uint8_t* input, size_t size, Value* output) {
        Value scratch{};
        return TlvCodec::decode(input, size, output, &scratch);
    }
};

template <>
struct MessageCodec<dp::iface::OfflineLogCommandRpcV1Request> {
    using Value = dp::iface::OfflineLogCommandRpcV1Request;
    using TlvCodec = dp::iface::tlv::Codec<Value>;
    static constexpr size_t kMaxEncodedSize = TlvCodec::kMaxEncodedSize;
    static constexpr bool kPortable = true;
    static constexpr uint32_t kInterfaceId = 4125U;
    static bool encode(const Value& input, uint8_t* output, size_t capacity, size_t* used) {
        return TlvCodec::encode(input, output, capacity, used);
    }
    static bool decode(const uint8_t* input, size_t size, Value* output) {
        Value scratch{};
        return TlvCodec::decode(input, size, output, &scratch);
    }
};

template <>
struct MessageCodec<dp::iface::OfflineLogCommandRpcV1Response> {
    using Value = dp::iface::OfflineLogCommandRpcV1Response;
    using TlvCodec = dp::iface::tlv::Codec<Value>;
    static constexpr size_t kMaxEncodedSize = TlvCodec::kMaxEncodedSize;
    static constexpr bool kPortable = true;
    static constexpr uint32_t kInterfaceId = 4126U;
    static bool encode(const Value& input, uint8_t* output, size_t capacity, size_t* used) {
        return TlvCodec::encode(input, output, capacity, used);
    }
    static bool decode(const uint8_t* input, size_t size, Value* output) {
        Value scratch{};
        return TlvCodec::decode(input, size, output, &scratch);
    }
};

template <>
struct MessageCodec<dp::iface::OfflineLogCommandV2Request> {
    using Value = dp::iface::OfflineLogCommandV2Request;
    using TlvCodec = dp::iface::tlv::Codec<Value>;
    static constexpr size_t kMaxEncodedSize = TlvCodec::kMaxEncodedSize;
    static constexpr bool kPortable = true;
    static constexpr uint32_t kInterfaceId = 4128U;
    static bool encode(const Value& input, uint8_t* output, size_t capacity, size_t* used) {
        return TlvCodec::encode(input, output, capacity, used);
    }
    static bool decode(const uint8_t* input, size_t size, Value* output) {
        Value scratch{};
        return TlvCodec::decode(input, size, output, &scratch);
    }
};

template <>
struct MessageCodec<dp::iface::OfflineLogCommandV2Response> {
    using Value = dp::iface::OfflineLogCommandV2Response;
    using TlvCodec = dp::iface::tlv::Codec<Value>;
    static constexpr size_t kMaxEncodedSize = TlvCodec::kMaxEncodedSize;
    static constexpr bool kPortable = true;
    static constexpr uint32_t kInterfaceId = 4129U;
    static bool encode(const Value& input, uint8_t* output, size_t capacity, size_t* used) {
        return TlvCodec::encode(input, output, capacity, used);
    }
    static bool decode(const uint8_t* input, size_t size, Value* output) {
        Value scratch{};
        return TlvCodec::decode(input, size, output, &scratch);
    }
};

}  // namespace dp::comm

namespace dp::protocol::services {
struct CapabilityQueryV1 {
    using Request = dp::iface::CapabilityQueryV1Request;
    using Response = dp::iface::CapabilityQueryV1Response;
    static constexpr uint32_t kInterfaceId = 4107U;
    static constexpr uint32_t kRequestInterfaceId = 4112U;
    static constexpr uint32_t kResponseInterfaceId = 4113U;
};

struct DummyManageV1 {
    using Request = dp::iface::DummyManageV1Request;
    using Response = dp::iface::DummyManageV1Response;
    static constexpr uint32_t kInterfaceId = 4134U;
    static constexpr uint32_t kRequestInterfaceId = 4135U;
    static constexpr uint32_t kResponseInterfaceId = 4136U;
};

struct FileTransferCancelV1 {
    using Request = dp::iface::FileTransferCancelV1Request;
    using Response = dp::iface::FileTransferCancelV1Response;
    static constexpr uint32_t kInterfaceId = 4111U;
    static constexpr uint32_t kRequestInterfaceId = 4120U;
    static constexpr uint32_t kResponseInterfaceId = 4121U;
};

struct FileTransferChunkV1 {
    using Request = dp::iface::FileTransferChunkV1Request;
    using Response = dp::iface::FileTransferChunkV1Response;
    static constexpr uint32_t kInterfaceId = 4109U;
    static constexpr uint32_t kRequestInterfaceId = 4116U;
    static constexpr uint32_t kResponseInterfaceId = 4117U;
};

struct FileTransferCloseV1 {
    using Request = dp::iface::FileTransferCloseV1Request;
    using Response = dp::iface::FileTransferCloseV1Response;
    static constexpr uint32_t kInterfaceId = 4110U;
    static constexpr uint32_t kRequestInterfaceId = 4118U;
    static constexpr uint32_t kResponseInterfaceId = 4119U;
};

struct FileTransferOpenV1 {
    using Request = dp::iface::FileTransferOpenV1Request;
    using Response = dp::iface::FileTransferOpenV1Response;
    static constexpr uint32_t kInterfaceId = 4108U;
    static constexpr uint32_t kRequestInterfaceId = 4114U;
    static constexpr uint32_t kResponseInterfaceId = 4115U;
};

struct OfflineLogCommandRpcV1 {
    using Request = dp::iface::OfflineLogCommandRpcV1Request;
    using Response = dp::iface::OfflineLogCommandRpcV1Response;
    static constexpr uint32_t kInterfaceId = 4124U;
    static constexpr uint32_t kRequestInterfaceId = 4125U;
    static constexpr uint32_t kResponseInterfaceId = 4126U;
};

struct OfflineLogCommandV2 {
    using Request = dp::iface::OfflineLogCommandV2Request;
    using Response = dp::iface::OfflineLogCommandV2Response;
    static constexpr uint32_t kInterfaceId = 4127U;
    static constexpr uint32_t kRequestInterfaceId = 4128U;
    static constexpr uint32_t kResponseInterfaceId = 4129U;
};

}  // namespace dp::protocol::services
