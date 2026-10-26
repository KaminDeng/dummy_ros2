#pragma once
#include "dummy_motion_client/device_profile.hpp"
#include "dummy_motion_client/dp_management_transport.hpp"
#include <dummy_ref/manual_reference_policy.hpp>
#include <map>
#include <openssl/sha.h>
#include <string>
#include <vector>
namespace dummy_motion_client {
struct ManualRecoveryIdentity {
  std::array<std::uint8_t, 16> device{};
  std::uint64_t boot{};
  std::uint32_t profile{};
  DeviceBundleIdentity bundle;
};
enum class ManualRecoveryShape {
  kKnownRestTimeoutFault,
  kRetainedDisabledIdle,
  kInterruptedRestFault,
  kInterruptedReleasedFault
};
inline bool
validate_manual_recovery(const dummy_wire::DummyDeviceStatusV2 &status,
                         const dummy_wire::DummyJointStateV2 &state,
                         const ManualRecoveryIdentity &expected,
                         std::uint64_t now, std::uint64_t state_at,
                         std::uint64_t status_at, ManualRecoveryShape shape,
                         std::string &reason) {
  auto reject = [&](const char *why) {
    reason = why;
    return false;
  };
  if (!state_at || !status_at || state_at > now || status_at > now ||
      now - state_at > 250000 ||
      now - status_at > dummy_ref::host::kManualReferenceStatusMaxAgeUs)
    return reject("STALE_SOURCE");
  if (status.schema_major() != 2 || state.schema_major() != 2 ||
      status.schema_minor() != dummy_ref::protocol::kSchemaMinor ||
      state.schema_minor() != dummy_ref::protocol::kSchemaMinor)
    return reject("SCHEMA_MISMATCH");
  if (!expected.boot || !expected.profile ||
      status.boot_epoch() != expected.boot ||
      state.boot_epoch() != expected.boot ||
      status.profile_id() != expected.profile ||
      state.profile_id() != expected.profile ||
      !std::equal(expected.device.begin(), expected.device.end(),
                  status.device_id()) ||
      !std::equal(expected.device.begin(), expected.device.end(),
                  state.device_id()) ||
      status.configured_mask() != 63 || state.configured_mask() != 63)
    return reject("IDENTITY_MISMATCH");
  const auto match = [](const auto &digest, const auto &wire) {
    return std::any_of(digest.begin(), digest.end(),
                       [](auto b) { return b != 0; }) &&
           std::equal(digest.begin(), digest.end(), wire);
  };
  if (!match(expected.bundle.firmware_hash, status.firmware_sha256()) ||
      !match(expected.bundle.contract_hash, status.contract_sha256()) ||
      !match(expected.bundle.model_hash, status.model_sha256()) ||
      !match(expected.bundle.profile_hash, status.profile_sha256()) ||
      !match(expected.bundle.policy_hash, status.compiled_policy_sha256()))
    return reject("ARTIFACT_HASH_MISMATCH");
  if (status.session_id() != 0 || state.session_id() != 0 ||
      status.reference_epoch() == 0 ||
      status.reference_epoch() != state.reference_epoch() ||
      status.reference_kind() != 1 ||
      status.motion_epoch() != state.motion_epoch() ||
      status.management_epoch() != state.management_epoch() ||
      status.real_motion_permitted() || status.permit_remaining_us() != 0 ||
      state.hold_ready())
    return reject("NOT_RELEASED_RETAINED_REFERENCE");
  if ((shape != ManualRecoveryShape::kInterruptedRestFault &&
       !status.release_committed()) ||
      status.cleanup_state() != 3 || status.evidence_lost())
    return reject("DISABLE_EVIDENCE_MISSING");
  if (state.raw_position_valid_mask() != 63)
    return reject("RAW_FEEDBACK_MISSING");
  for (unsigned i = 0; i < 6; ++i)
    if (state.can_rx_age_us()[i] > 250000)
      return reject("CAN_FEEDBACK_STALE");
  const unsigned core =
      shape == ManualRecoveryShape::kRetainedDisabledIdle ? 2 : 9;
  if (status.core_state() != core || state.core_state() != core)
    return reject("CORE_STATE_MISMATCH");
  if (shape == ManualRecoveryShape::kInterruptedRestFault) {
    if (state.position_valid_mask() != 63 ||
        state.velocity_valid_mask() != 63 || status.release_committed() ||
        status.first_failure_reason() != 13 ||
        state.first_failure_reason() != 13 || status.failure_source() != 4196 ||
        status.can_failed_mask() != 63 || state.can_failed_mask() != 63 ||
        status.attempt_kind() != DP_PROTOCOL_DPSDK_INTERFACES_MSG_DUMMYPROTOCOLV2_MESSAGE_ATTEMPT_KIND_RELEASE || status.run_id() == 0 ||
        status.attempt_id() != status.run_id() || status.parent_run_id() == 0 ||
        status.group_count() != 1 || status.final_state() != 0 ||
        status.transaction_phase() != 4 || state.transaction_phase() != 4)
      return reject("NOT_KNOWN_INTERRUPTED_REST");
    reason.clear();
    return true;
  }
  if (shape == ManualRecoveryShape::kInterruptedReleasedFault &&
      (status.first_failure_reason() != 4196 ||
       status.failure_source() != 4196 || status.attempt_kind() != DP_PROTOCOL_DPSDK_INTERFACES_MSG_DUMMYPROTOCOLV2_MESSAGE_ATTEMPT_KIND_RELEASE ||
       status.final_state() !=
           DP_PROTOCOL_DPSDK_INTERFACES_MSG_DUMMYPROTOCOLV2_MESSAGE_FINAL_STATE_FAULT_LATCHED_AND_RELEASED))
    return reject("NOT_INTERRUPTED_REST_RELEASED");
  // State仅发布transaction.first_failure；Status在事务无错时回落adapter_error_reason。
  // 本轮4199是适配器REST超时，绝不能要求State复制该来源，更不能接受新的事务错误。
  if (state.first_failure_reason() != 0 || state.can_failed_mask() != 0 ||
      status.can_failed_mask() != 0)
    return reject("TRANSACTION_ERROR_PRESENT");
  if (shape == ManualRecoveryShape::kKnownRestTimeoutFault &&
      (status.first_failure_reason() != 0x1067 ||
       status.failure_source() != 0x1067))
    return reject("NOT_KNOWN_REST_TIMEOUT");
  if (shape == ManualRecoveryShape::kRetainedDisabledIdle &&
      status.first_failure_reason() != 0)
    return reject("IDLE_ERROR_PRESENT");
  reason.clear();
  return true;
}
// DPRAW01为Host归档封套；消息字段、证据前缀位置只由安装SDK Codec解释。
struct ManualInterruptedRestProof {
  std::uint64_t boot{}, session{}, run{}, parent{}, group{};
  std::uint32_t reference{}, motion{}, management{}, next_sequence{},
      next_management_sequence{};
  std::array<std::uint8_t, 16> client{}, device{};
  std::uint32_t profile{};
  std::array<std::uint8_t, 32> policy{};
  std::vector<std::uint64_t> request_ids;
  std::array<std::uint8_t, 32> chain{}, file_sha{};
};
inline std::array<std::uint8_t, 32>
recovery_sha(const std::vector<std::uint8_t> &data) {
  std::array<std::uint8_t, 32> out{};
  SHA256(data.data(), data.size(), out.data());
  return out;
}
inline bool
recovery_evidence_chain(const std::vector<std::uint8_t> &bytes,
                        const std::array<std::uint8_t, 32> &previous,
                        dummy_wire::DummyAttemptEvidenceV2 &page) {
  using Codec = dummy_wire::DummyAttemptEvidenceV2Codec;
  if (Codec::decode(bytes.data(), bytes.size(), &page) !=
          dummy_wire::CodecResult::kOk ||
      page.evidence_lost() ||
      !std::equal(previous.begin(), previous.end(),
                  page.previous_chain_sha256()))
    return false;
  std::vector<std::uint8_t> material(previous.begin(), previous.end());
  material.insert(material.end(), bytes.begin(),
                  bytes.begin() + Codec::kOffset_previous_chain_sha256);
  material.push_back(page.evidence_lost() ? 1 : 0);
  const auto digest = recovery_sha(material);
  return std::equal(digest.begin(), digest.end(), page.chain_sha256());
}
inline bool
recovery_disable_group(const dummy_wire::DummyAttemptEvidenceV2 &p) {
  if (p.category() != 3 || !p.final() || p.expected_mask() != 63 ||
      p.submitted_mask() != 63 || p.sent_mask() != 63 || p.failed_mask() != 0 ||
      p.evidence_lost() || !p.group_id())
    return false;
  for (unsigned i = 0; i < 6; ++i)
    if (!p.queued_at_us()[i] || p.queued_at_us()[i] > p.taken_at_us()[i] ||
        p.taken_at_us()[i] > p.submitted_at_us()[i] ||
        p.submitted_at_us()[i] > p.sent_at_us()[i] || !p.sent_at_us()[i] ||
        p.abort_at_us()[i])
      return false;
  return true;
}
inline bool
validate_interrupted_rest_raw(const std::vector<std::uint8_t> &data,
                              const std::array<std::uint8_t, 32> &expected_sha,
                              const ManualRecoveryIdentity &identity,
                              ManualInterruptedRestProof &out,
                              std::string &reason) {
  out = {};
  out.device = identity.device;
  out.profile = identity.profile;
  out.policy = identity.bundle.policy_hash;
  auto reject = [&](const char *why) {
    reason = why;
    return false;
  };
  if (data.size() < 8 ||
      !std::equal(data.begin(), data.begin() + 8, "DPRAW01\0") ||
      recovery_sha(data) != expected_sha)
    return reject("RAW_FILE_SHA_OR_MAGIC");
  struct Chain {
    std::uint32_t next{};
    std::array<std::uint8_t, 32> digest{};
  };
  std::map<std::uint64_t, Chain> chains;
  dummy_wire::DummyDeviceStatusV2 status;
  dummy_wire::DummyJointStateV2 state;
  std::uint64_t status_at = 0, state_at = 0;
  bool begin_seen = false, grant_seen = false, disable_seen = false;
  ManageRequest grant;
  std::uint64_t rejected_parent = 0;
  std::uint32_t begin_motion = 0;
  std::size_t at = 8;
  auto le = [&](unsigned n) {
    std::uint64_t value = 0;
    for (unsigned i = 0; i < n; ++i)
      value |= std::uint64_t(data[at++]) << (i * 8);
    return value;
  };
  while (at < data.size()) {
    if (data.size() - at < 39)
      return reject("RAW_TRUNCATED_HEADER");
    const auto kind = le(1), size = le(2), received = le(8), boot = le(8),
               session = le(8), run = le(8), seq = le(4);
    if (size > data.size() - at || !received || boot != identity.boot)
      return reject("RAW_ENVELOPE_IDENTITY");
    std::vector<std::uint8_t> raw(data.begin() + at, data.begin() + at + size);
    at += size;
    if (kind == 2) {
      if (dummy_wire::DummyJointStateV2Codec::decode(
              raw.data(), raw.size(), &state) != dummy_wire::CodecResult::kOk)
        return reject("RAW_STATE_CODEC");
      state_at = received;
    } else if (kind == 3) {
      if (dummy_wire::DummyDeviceStatusV2Codec::decode(
              raw.data(), raw.size(), &status) != dummy_wire::CodecResult::kOk)
        return reject("RAW_STATUS_CODEC");
      status_at = received;
    } else if (kind == 4) {
      ManageRequest request;
      if (dummy_wire::DummyManageV2RequestCodec::decode(
              raw.data(), raw.size(), &request) != dummy_wire::CodecResult::kOk)
        return reject("RAW_REQUEST_CODEC");
      if (!request.request_seq() || !request.request_id() ||
          request.request_seq() == UINT32_MAX)
        return reject("RAW_REQUEST_SEQUENCE_EXHAUSTED");
      if (std::find(out.request_ids.begin(), out.request_ids.end(),
                    request.request_id()) != out.request_ids.end())
        return reject("RAW_DUPLICATE_REQUEST_ID");
      out.request_ids.push_back(request.request_id());
      if (!out.next_management_sequence ||
          static_cast<std::int32_t>(request.request_seq() + 1 -
                                    out.next_management_sequence) > 0)
        out.next_management_sequence = request.request_seq() + 1;
      if (request.operation() == 2) {
        grant_seen = true;
        grant = request;
      }
      if (request.operation() == 17) {
        if (begin_seen || !grant_seen ||
            grant.expected_session_id() != request.expected_session_id() ||
            !std::equal(std::begin(grant.client_instance_id()),
                        std::end(grant.client_instance_id()),
                        request.client_instance_id()) ||
            grant.expected_boot_epoch() != request.expected_boot_epoch() ||
            grant.expected_reference_epoch() !=
                request.expected_reference_epoch() ||
            grant.expected_profile_id() != request.expected_profile_id() ||
            !request.expected_session_id() ||
            !request.expected_reference_epoch() ||
            request.expected_boot_epoch() != identity.boot ||
            request.expected_profile_id() != identity.profile ||
            !std::equal(identity.device.begin(), identity.device.end(),
                        request.device_id()))
          return reject("RAW_BEGIN_IDENTITY");
        begin_seen = true;
        begin_motion = request.expected_motion_epoch();
        out.boot = identity.boot;
        out.session = request.expected_session_id();
        out.reference = request.expected_reference_epoch();
        std::copy(std::begin(request.client_instance_id()),
                  std::end(request.client_instance_id()), out.client.begin());
        if (std::all_of(out.client.begin(), out.client.end(),
                        [](auto x) { return x == 0; }))
          return reject("RAW_ZERO_CLIENT");
      }
    } else if (kind == 5) {
      dummy_wire::DummyManageV2Response reply;
      if (dummy_wire::DummyManageV2ResponseCodec::decode(
              raw.data(), raw.size(), &reply) != dummy_wire::CodecResult::kOk)
        return reject("RAW_RESPONSE_CODEC");
    } else if (kind == 1) {
      dummy_wire::DummyAttemptEvidenceV2 p;
      if (!begin_seen ||
          dummy_wire::DummyAttemptEvidenceV2Codec::decode(
              raw.data(), raw.size(), &p) != dummy_wire::CodecResult::kOk ||
          p.run_id() != run || p.evidence_seq() != seq ||
          session != out.session || !run)
        return reject("RAW_PAGE_IDENTITY");
      auto found = chains.find(run);
      if (found == chains.end()) {
        std::vector<std::uint8_t> seed(192, 0);
        std::size_t index = 0;
        for (auto value : {run, boot})
          for (unsigned i = 0; i < 8; ++i)
            seed[index++] = std::uint8_t(value >> (i * 8));
        for (auto value : out.client)
          seed[index++] = value;
        for (const auto *hash :
             {&identity.bundle.contract_hash, &identity.bundle.model_hash,
              &identity.bundle.profile_hash, &identity.bundle.firmware_hash})
          for (auto value : *hash)
            seed[index++] = value;
        found = chains.emplace(run, Chain{0, recovery_sha(seed)}).first;
      }
      if (seq != found->second.next ||
          !recovery_evidence_chain(raw, found->second.digest, p))
        return reject("RAW_CHAIN_OR_SEQUENCE");
      ++found->second.next;
      std::copy(std::begin(p.chain_sha256()), std::end(p.chain_sha256()),
                found->second.digest.begin());
      if (p.category() == 4) {
        if (rejected_parent || disable_seen || p.attempt_kind() != 1 ||
            p.attempt_id() != run || p.parent_run_id() != 0 ||
            p.motion_epoch() != begin_motion ||
            p.final_state() !=
                DP_PROTOCOL_DPSDK_INTERFACES_MSG_DUMMYPROTOCOLV2_MESSAGE_FINAL_STATE_REJECTED_BEFORE_MOTION ||
            p.group_count() != 0 || p.terminal_reason() != 14 ||
            p.first_failure_source() != 0 || p.first_failure_axis_mask() != 0 ||
            p.release_committed() || p.motion_complete())
          return reject("RAW_PARENT_NOT_REJECTED_BEFORE_MOTION");
        rejected_parent = run;
      } else if (p.category() == 3) {
        if (disable_seen || !recovery_disable_group(p) ||
            p.attempt_kind() != DP_PROTOCOL_DPSDK_INTERFACES_MSG_DUMMYPROTOCOLV2_MESSAGE_ATTEMPT_KIND_RELEASE || !p.parent_run_id() ||
            p.first_failure_source() != 4196 ||
            p.first_failure_axis_mask() != 63 || p.release_committed() ||
            p.group_count() != 1 || seq != 0)
          return reject("RAW_INTERRUPTED_DISABLE_GROUP");
        disable_seen = true;
        out.run = run;
        out.parent = p.parent_run_id();
        out.group = p.group_id();
        out.motion = p.motion_epoch();
      } else
        return reject("RAW_UNEXPECTED_GROUP");
    } else
      return reject("RAW_UNKNOWN_RECORD_KIND");
  }
  if (!begin_seen || !disable_seen || rejected_parent != out.parent ||
      !chains.count(out.parent) || chains.at(out.parent).next != 1 ||
      !validate_manual_recovery(
          status, state, identity, std::max(state_at, status_at), state_at,
          status_at, ManualRecoveryShape::kInterruptedRestFault, reason) ||
      status.run_id() != out.run || status.parent_run_id() != out.parent ||
      status.motion_epoch() != out.motion ||
      status.reference_epoch() != out.reference)
    return reject("RAW_FINAL_FAULT_BINDING");
  out.management = status.management_epoch();
  out.chain = chains.at(out.run).digest;
  out.next_sequence = chains.at(out.run).next;
  out.file_sha = expected_sha;
  reason.clear();
  return true;
}
inline bool
interrupted_rest_live_matches(const ManualInterruptedRestProof &proof,
                              const dummy_wire::DummyDeviceStatusV2 &status,
                              const dummy_wire::DummyJointStateV2 &state) {
  return proof.boot == status.boot_epoch() &&
         proof.reference == status.reference_epoch() &&
         proof.motion == status.motion_epoch() &&
         proof.management == status.management_epoch() &&
         proof.run == status.run_id() &&
         proof.parent == status.parent_run_id() &&
         proof.boot == state.boot_epoch() &&
         proof.reference == state.reference_epoch() &&
         proof.motion == state.motion_epoch() &&
         proof.management == state.management_epoch() && proof.session &&
         proof.next_sequence == 1;
}
inline bool interrupted_release_request_matches(
    const ManageRequest &request, const ManualInterruptedRestProof &proof,
    const dummy_wire::DummyDeviceStatusV2 &status,
    const dummy_wire::DummyJointStateV2 &state) {
  return interrupted_rest_live_matches(proof, status, state) &&
         request.operation() == 10 && request.request_id() != 0 &&
         request.request_seq() >= proof.next_management_sequence &&
         request.expected_boot_epoch() == proof.boot &&
         std::find(proof.request_ids.begin(), proof.request_ids.end(),
                   request.request_id()) == proof.request_ids.end() &&
         request.expected_profile_id() == proof.profile &&
         request.configured_mask() == 63 &&
         std::equal(proof.device.begin(), proof.device.end(),
                    request.device_id()) &&
         std::equal(proof.policy.begin(), proof.policy.end(),
                    request.compiled_policy_sha256()) &&
         request.expected_session_id() == proof.session &&
         request.expected_reference_epoch() == state.reference_epoch() &&
         request.expected_motion_epoch() == state.motion_epoch() &&
         request.expected_management_epoch() == state.management_epoch() &&
         request.freshness_ticket() == state.freshness_ticket() &&
         std::equal(proof.client.begin(), proof.client.end(),
                    request.client_instance_id());
}

// 新D10必须真实新增Disable组并封Release原终页，不能用Status链或旧SENT代替。
struct InterruptedRestReleaseEvidence {
  ManualInterruptedRestProof proof;
  bool disable_seen = false, terminal_seen = false;
  std::uint64_t terminal_received_us = 0, new_group = 0;
  explicit InterruptedRestReleaseEvidence(ManualInterruptedRestProof value)
      : proof(value) {}
  bool consume(const std::vector<std::uint8_t> &bytes, std::uint64_t received,
               std::string &reason) {
    dummy_wire::DummyAttemptEvidenceV2 page;
    if (!recovery_evidence_chain(bytes, proof.chain, page) ||
        page.run_id() != proof.run || page.attempt_id() != proof.run ||
        page.attempt_kind() != DP_PROTOCOL_DPSDK_INTERFACES_MSG_DUMMYPROTOCOLV2_MESSAGE_ATTEMPT_KIND_RELEASE || page.parent_run_id() != proof.parent ||
        page.evidence_seq() != proof.next_sequence || terminal_seen ||
        !received) {
      reason = "NEW_RELEASE_CHAIN_OR_IDENTITY";
      return false;
    }
    if (page.category() == 3) {
      if (disable_seen || !recovery_disable_group(page) ||
          page.group_id() <= proof.group ||
          page.motion_epoch() != proof.motion + 1 ||
          page.first_failure_source() != 4196 ||
          page.first_failure_axis_mask() != 63 || page.group_count() != 2 ||
          page.release_committed()) {
        reason = "NEW_DISABLE_GROUP_INVALID";
        return false;
      }
      disable_seen = true;
      new_group = page.group_id();
    } else if (page.category() == 4) {
      if (!disable_seen || !page.final() || !page.release_committed() ||
          page.motion_complete() ||
          page.final_state() !=
              DP_PROTOCOL_DPSDK_INTERFACES_MSG_DUMMYPROTOCOLV2_MESSAGE_FINAL_STATE_FAULT_LATCHED_AND_RELEASED ||
          page.first_failure_source() != 4196 ||
          page.first_failure_axis_mask() != 63 || page.group_count() != 2 ||
          page.motion_epoch() != proof.motion) {
        reason = "NEW_RELEASE_TERMINAL_INVALID";
        return false;
      }
      terminal_seen = true;
      terminal_received_us = received;
    } else {
      reason = "UNEXPECTED_NON_DISABLE_RECOVERY_GROUP";
      return false;
    }
    ++proof.next_sequence;
    std::copy(std::begin(page.chain_sha256()), std::end(page.chain_sha256()),
              proof.chain.begin());
    reason.clear();
    return true;
  }
};
} // namespace dummy_motion_client
