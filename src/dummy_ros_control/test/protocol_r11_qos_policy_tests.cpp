/** @file protocol_r11_qos_policy_tests.cpp @brief Root沿用中央QoS及8/32窗口，未知类型不降级。 */
#include "dummy_ref/root_endpoint_qos.hpp"
#include <cassert>
int main() {
    namespace r=dummy_ref::root;
    namespace s=dp::protocol::selection;
    dp::comm::QoS qos{};
    assert(r::endpoint_qos<r::QosRole::kNative>(s::dp_protocol_dummy_JointCommand::kTypeId,0,qos));
    assert(qos.reliability==dp::comm::Reliability::kReliable && qos.priority==dp::comm::Priority::kControl);
    assert(r::endpoint_qos<r::QosRole::kNative>(s::dp_protocol_dummy_AttemptEvidence::kTypeId,1,qos));
    assert(qos.reliability==dp::comm::Reliability::kReliable && qos.priority==dp::comm::Priority::kControl && qos.depth==32);
    assert(r::endpoint_qos<r::QosRole::kRosPic>(s::dp_protocol_dummy_AttemptEvidence::kTypeId,1,qos));
    assert(qos.reliability==dp::comm::Reliability::kBestEffort && qos.priority==dp::comm::Priority::kData && qos.depth==1);
    assert(!r::endpoint_qos<r::QosRole::kNative>(UINT32_MAX,0,qos));
    assert(!r::endpoint_qos<r::QosRole::kNative>(s::dp_protocol_dummy_JointCommand::kTypeId,3,qos));
}
