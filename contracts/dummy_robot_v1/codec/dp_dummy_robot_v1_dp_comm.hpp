// Auto-generated Dummy dp_comm raw_fixed traits. Do not edit.
#pragma once

#include "dp_comm/dp_comm.hpp"
#include "dp_dummy_robot_v1.hpp"

namespace dp::comm {
template <> struct MessageCodec<dp::protocol::dummy_robot_v1::DummyDeviceStatusV1> {
    static constexpr size_t kEncodedSize = dp::protocol::dummy_robot_v1::DummyDeviceStatusV1Codec::kEncodedSize;
    static constexpr uint32_t kInterfaceId = 4131U;
    static constexpr bool kPortable = true;
    static bool encode(const dp::protocol::dummy_robot_v1::DummyDeviceStatusV1& value, uint8_t* out, size_t size) {
        return dp::protocol::dummy_robot_v1::DummyDeviceStatusV1Codec::encode(value, out, size) == dp::protocol::dummy_robot_v1::CodecResult::kOk;
    }
    static bool decode(const uint8_t* in, size_t size, dp::protocol::dummy_robot_v1::DummyDeviceStatusV1* value) {
        return dp::protocol::dummy_robot_v1::DummyDeviceStatusV1Codec::decode(in, size, value) == dp::protocol::dummy_robot_v1::CodecResult::kOk;
    }
};

template <> struct MessageCodec<dp::protocol::dummy_robot_v1::DummyJointCommandV1> {
    static constexpr size_t kEncodedSize = dp::protocol::dummy_robot_v1::DummyJointCommandV1Codec::kEncodedSize;
    static constexpr uint32_t kInterfaceId = 4132U;
    static constexpr bool kPortable = true;
    static bool encode(const dp::protocol::dummy_robot_v1::DummyJointCommandV1& value, uint8_t* out, size_t size) {
        return dp::protocol::dummy_robot_v1::DummyJointCommandV1Codec::encode(value, out, size) == dp::protocol::dummy_robot_v1::CodecResult::kOk;
    }
    static bool decode(const uint8_t* in, size_t size, dp::protocol::dummy_robot_v1::DummyJointCommandV1* value) {
        return dp::protocol::dummy_robot_v1::DummyJointCommandV1Codec::decode(in, size, value) == dp::protocol::dummy_robot_v1::CodecResult::kOk;
    }
};

template <> struct MessageCodec<dp::protocol::dummy_robot_v1::DummyJointStateV1> {
    static constexpr size_t kEncodedSize = dp::protocol::dummy_robot_v1::DummyJointStateV1Codec::kEncodedSize;
    static constexpr uint32_t kInterfaceId = 4133U;
    static constexpr bool kPortable = true;
    static bool encode(const dp::protocol::dummy_robot_v1::DummyJointStateV1& value, uint8_t* out, size_t size) {
        return dp::protocol::dummy_robot_v1::DummyJointStateV1Codec::encode(value, out, size) == dp::protocol::dummy_robot_v1::CodecResult::kOk;
    }
    static bool decode(const uint8_t* in, size_t size, dp::protocol::dummy_robot_v1::DummyJointStateV1* value) {
        return dp::protocol::dummy_robot_v1::DummyJointStateV1Codec::decode(in, size, value) == dp::protocol::dummy_robot_v1::CodecResult::kOk;
    }
};

template <> struct MessageCodec<dp::protocol::dummy_robot_v1::DummyManageV1Request> {
    static constexpr size_t kEncodedSize = dp::protocol::dummy_robot_v1::DummyManageV1RequestCodec::kEncodedSize;
    static constexpr uint32_t kInterfaceId = 4135U;
    static constexpr bool kPortable = true;
    static bool encode(const dp::protocol::dummy_robot_v1::DummyManageV1Request& value, uint8_t* out, size_t size) {
        return dp::protocol::dummy_robot_v1::DummyManageV1RequestCodec::encode(value, out, size) == dp::protocol::dummy_robot_v1::CodecResult::kOk;
    }
    static bool decode(const uint8_t* in, size_t size, dp::protocol::dummy_robot_v1::DummyManageV1Request* value) {
        return dp::protocol::dummy_robot_v1::DummyManageV1RequestCodec::decode(in, size, value) == dp::protocol::dummy_robot_v1::CodecResult::kOk;
    }
};

template <> struct MessageCodec<dp::protocol::dummy_robot_v1::DummyManageV1Response> {
    static constexpr size_t kEncodedSize = dp::protocol::dummy_robot_v1::DummyManageV1ResponseCodec::kEncodedSize;
    static constexpr uint32_t kInterfaceId = 4136U;
    static constexpr bool kPortable = true;
    static bool encode(const dp::protocol::dummy_robot_v1::DummyManageV1Response& value, uint8_t* out, size_t size) {
        return dp::protocol::dummy_robot_v1::DummyManageV1ResponseCodec::encode(value, out, size) == dp::protocol::dummy_robot_v1::CodecResult::kOk;
    }
    static bool decode(const uint8_t* in, size_t size, dp::protocol::dummy_robot_v1::DummyManageV1Response* value) {
        return dp::protocol::dummy_robot_v1::DummyManageV1ResponseCodec::decode(in, size, value) == dp::protocol::dummy_robot_v1::CodecResult::kOk;
    }
};

}  // namespace dp::comm

namespace dp::protocol::dummy_robot_v1 {
inline constexpr dp::comm::StaticRequirements kStaticRequirements{
    1U, 2U, 1U, 0U, 8U, 618U, 96U, 0U,
    static_cast<uint16_t>(kMaxObjectAlignment), 1U, 1U, 1U, 1U};
}  // namespace dp::protocol::dummy_robot_v1

namespace dp::protocol::services {
struct DummyManageV1RawFixed {
    using Request = dp::protocol::dummy_robot_v1::DummyManageV1Request;
    using Response = dp::protocol::dummy_robot_v1::DummyManageV1Response;
    static constexpr uint32_t kInterfaceId = 4134U;
    static constexpr uint32_t kRequestInterfaceId = 4135U;
    static constexpr uint32_t kResponseInterfaceId = 4136U;
};
}  // namespace dp::protocol::services
