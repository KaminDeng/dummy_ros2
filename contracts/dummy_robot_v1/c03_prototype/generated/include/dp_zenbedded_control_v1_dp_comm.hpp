// Auto-generated dp_comm codec adapters. Do not edit.
#pragma once

#include "dp_comm/dp_comm.hpp"
#include "dp_zenbedded_control_v1.hpp"

namespace dp::comm {
template <>
struct MessageCodec<dp::protocol::zenbedded::JointStateFrameV1> {
    static constexpr size_t kEncodedSize = dp::protocol::zenbedded::JointStateFrameV1Codec::kEncodedSize;
    static constexpr uint32_t kInterfaceId = 4097U;
    static constexpr bool kPortable = true;
    static bool encode(const dp::protocol::zenbedded::JointStateFrameV1& input, uint8_t* output, size_t size) {
        return dp::protocol::zenbedded::JointStateFrameV1Codec::encode(input, output, size) == dp::protocol::zenbedded::CodecResult::kOk;
    }
    static bool decode(const uint8_t* input, size_t size, dp::protocol::zenbedded::JointStateFrameV1* output) {
        return dp::protocol::zenbedded::JointStateFrameV1Codec::decode(input, size, output) == dp::protocol::zenbedded::CodecResult::kOk;
    }
};

template <>
struct MessageCodec<dp::protocol::zenbedded::JointCommandFrameV1> {
    static constexpr size_t kEncodedSize = dp::protocol::zenbedded::JointCommandFrameV1Codec::kEncodedSize;
    static constexpr uint32_t kInterfaceId = 4098U;
    static constexpr bool kPortable = true;
    static bool encode(const dp::protocol::zenbedded::JointCommandFrameV1& input, uint8_t* output, size_t size) {
        return dp::protocol::zenbedded::JointCommandFrameV1Codec::encode(input, output, size) == dp::protocol::zenbedded::CodecResult::kOk;
    }
    static bool decode(const uint8_t* input, size_t size, dp::protocol::zenbedded::JointCommandFrameV1* output) {
        return dp::protocol::zenbedded::JointCommandFrameV1Codec::decode(input, size, output) == dp::protocol::zenbedded::CodecResult::kOk;
    }
};

template <>
struct MessageCodec<dp::protocol::zenbedded::ControlWatchdogV1> {
    static constexpr size_t kEncodedSize = dp::protocol::zenbedded::ControlWatchdogV1Codec::kEncodedSize;
    static constexpr uint32_t kInterfaceId = 4104U;
    static constexpr bool kPortable = true;
    static bool encode(const dp::protocol::zenbedded::ControlWatchdogV1& input, uint8_t* output, size_t size) {
        return dp::protocol::zenbedded::ControlWatchdogV1Codec::encode(input, output, size) == dp::protocol::zenbedded::CodecResult::kOk;
    }
    static bool decode(const uint8_t* input, size_t size, dp::protocol::zenbedded::ControlWatchdogV1* output) {
        return dp::protocol::zenbedded::ControlWatchdogV1Codec::decode(input, size, output) == dp::protocol::zenbedded::CodecResult::kOk;
    }
};

}  // namespace dp::comm
