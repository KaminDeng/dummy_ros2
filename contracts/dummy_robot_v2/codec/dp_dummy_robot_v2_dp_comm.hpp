// Auto-generated Dummy dp_comm raw_fixed traits. Do not edit.
#pragma once

#include "dp_comm/dp_comm.hpp"
#include "dp_dummy_robot_v2.hpp"

namespace dp::comm {
template <> struct MessageCodec<dp::protocol::dummy_robot_v2::DummyDeviceStatusV2> {
    static constexpr size_t kEncodedSize = dp::protocol::dummy_robot_v2::DummyDeviceStatusV2Codec::kEncodedSize;
    static constexpr uint32_t kInterfaceId = 4137U;
    static constexpr bool kPortable = true;
    static bool encode(const dp::protocol::dummy_robot_v2::DummyDeviceStatusV2& value, uint8_t* out, size_t size) {
        return dp::protocol::dummy_robot_v2::DummyDeviceStatusV2Codec::encode(value, out, size) == dp::protocol::dummy_robot_v2::CodecResult::kOk;
    }
    static bool decode(const uint8_t* in, size_t size, dp::protocol::dummy_robot_v2::DummyDeviceStatusV2* value) {
        return dp::protocol::dummy_robot_v2::DummyDeviceStatusV2Codec::decode(in, size, value) == dp::protocol::dummy_robot_v2::CodecResult::kOk;
    }
};

template <> struct MessageCodec<dp::protocol::dummy_robot_v2::DummyJointCommandV2> {
    static constexpr size_t kEncodedSize = dp::protocol::dummy_robot_v2::DummyJointCommandV2Codec::kEncodedSize;
    static constexpr uint32_t kInterfaceId = 4138U;
    static constexpr bool kPortable = true;
    static bool encode(const dp::protocol::dummy_robot_v2::DummyJointCommandV2& value, uint8_t* out, size_t size) {
        return dp::protocol::dummy_robot_v2::DummyJointCommandV2Codec::encode(value, out, size) == dp::protocol::dummy_robot_v2::CodecResult::kOk;
    }
    static bool decode(const uint8_t* in, size_t size, dp::protocol::dummy_robot_v2::DummyJointCommandV2* value) {
        return dp::protocol::dummy_robot_v2::DummyJointCommandV2Codec::decode(in, size, value) == dp::protocol::dummy_robot_v2::CodecResult::kOk;
    }
};

template <> struct MessageCodec<dp::protocol::dummy_robot_v2::DummyJointStateV2> {
    static constexpr size_t kEncodedSize = dp::protocol::dummy_robot_v2::DummyJointStateV2Codec::kEncodedSize;
    static constexpr uint32_t kInterfaceId = 4139U;
    static constexpr bool kPortable = true;
    static bool encode(const dp::protocol::dummy_robot_v2::DummyJointStateV2& value, uint8_t* out, size_t size) {
        return dp::protocol::dummy_robot_v2::DummyJointStateV2Codec::encode(value, out, size) == dp::protocol::dummy_robot_v2::CodecResult::kOk;
    }
    static bool decode(const uint8_t* in, size_t size, dp::protocol::dummy_robot_v2::DummyJointStateV2* value) {
        return dp::protocol::dummy_robot_v2::DummyJointStateV2Codec::decode(in, size, value) == dp::protocol::dummy_robot_v2::CodecResult::kOk;
    }
};

template <> struct MessageCodec<dp::protocol::dummy_robot_v2::DummyManageV2Request> {
    static constexpr size_t kEncodedSize = dp::protocol::dummy_robot_v2::DummyManageV2RequestCodec::kEncodedSize;
    static constexpr uint32_t kInterfaceId = 4141U;
    static constexpr bool kPortable = true;
    static bool encode(const dp::protocol::dummy_robot_v2::DummyManageV2Request& value, uint8_t* out, size_t size) {
        return dp::protocol::dummy_robot_v2::DummyManageV2RequestCodec::encode(value, out, size) == dp::protocol::dummy_robot_v2::CodecResult::kOk;
    }
    static bool decode(const uint8_t* in, size_t size, dp::protocol::dummy_robot_v2::DummyManageV2Request* value) {
        return dp::protocol::dummy_robot_v2::DummyManageV2RequestCodec::decode(in, size, value) == dp::protocol::dummy_robot_v2::CodecResult::kOk;
    }
};

template <> struct MessageCodec<dp::protocol::dummy_robot_v2::DummyManageV2Response> {
    static constexpr size_t kEncodedSize = dp::protocol::dummy_robot_v2::DummyManageV2ResponseCodec::kEncodedSize;
    static constexpr uint32_t kInterfaceId = 4142U;
    static constexpr bool kPortable = true;
    static bool encode(const dp::protocol::dummy_robot_v2::DummyManageV2Response& value, uint8_t* out, size_t size) {
        return dp::protocol::dummy_robot_v2::DummyManageV2ResponseCodec::encode(value, out, size) == dp::protocol::dummy_robot_v2::CodecResult::kOk;
    }
    static bool decode(const uint8_t* in, size_t size, dp::protocol::dummy_robot_v2::DummyManageV2Response* value) {
        return dp::protocol::dummy_robot_v2::DummyManageV2ResponseCodec::decode(in, size, value) == dp::protocol::dummy_robot_v2::CodecResult::kOk;
    }
};

}  // namespace dp::comm

namespace dp::protocol::dummy_robot_v2 {
inline constexpr dp::comm::StaticRequirements kStaticRequirements{
    1U, 2U, 1U, 0U, 8U, 301U, 96U, 0U,
    static_cast<uint16_t>(kMaxObjectAlignment), 1U, 1U, 1U, 1U};
}  // namespace dp::protocol::dummy_robot_v2

namespace dp::protocol::services {
struct DummyManageV2RawFixed {
    using Request = dp::protocol::dummy_robot_v2::DummyManageV2Request;
    using Response = dp::protocol::dummy_robot_v2::DummyManageV2Response;
    static constexpr uint32_t kInterfaceId = 4140U;
    static constexpr uint32_t kRequestInterfaceId = 4141U;
    static constexpr uint32_t kResponseInterfaceId = 4142U;
};
}  // namespace dp::protocol::services
