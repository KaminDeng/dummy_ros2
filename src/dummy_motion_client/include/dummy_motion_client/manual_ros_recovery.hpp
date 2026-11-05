#pragma once
#include "dummy_motion_client/device_profile.hpp"
#include "dummy_motion_client/dp_management_transport.hpp"
#include <dummy_ref/manual_reference_policy.hpp>
#include <map>
#include <openssl/sha.h>
#include "dummy_ros_control/message_presence.hpp"
#include "dummy_ros_control/capture_record.hpp"
#include "canonical_records.h"
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
validate_manual_recovery(const dummy_ref::messages::Status &status,
                         const dummy_ref::messages::State &state,const dp::comm::SourceId& status_source,const dp::comm::SourceId& state_source,
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
  if(!dummy_ros_control::message_fields_present(status) || !dummy_ros_control::message_fields_present(state))return reject("MISSING_FIELDS");
  if (!expected.boot || !expected.profile ||
      status_source.boot_id != expected.boot ||
      state_source.boot_id != expected.boot ||
      *status.profile_id() != expected.profile ||
      *state.profile_id() != expected.profile ||
      !std::equal(expected.device.begin(), expected.device.end(),
                  status.domain_device_id()->begin()) ||
      !std::equal(expected.device.begin(), expected.device.end(),
                  state.domain_device_id()->begin()) ||
      *status.configured_mask() != 63 || dummy_ros_control::configured_joint_mask(state) != 63)
    return reject("IDENTITY_MISMATCH");
  const auto match = [](const auto &digest, const auto &wire) {
    return std::any_of(digest.begin(), digest.end(),
                       [](auto b) { return b != 0; }) &&
           std::equal(digest.begin(), digest.end(), wire);
  };
  if (!match(expected.bundle.firmware_hash, status.firmware_sha256()->begin()) ||
      !match(expected.bundle.contract_hash, status.contract_sha256()->begin()) ||
      !match(expected.bundle.model_hash, status.model_sha256()->begin()) ||
      !match(expected.bundle.profile_hash, status.profile_sha256()->begin()) ||
      !match(expected.bundle.policy_hash, status.compiled_policy_sha256()->begin()))
    return reject("ARTIFACT_HASH_MISMATCH");
  if (*status.session_id() != 0 || *state.session_id() != 0 ||
      *status.reference_epoch() == 0 ||
      *status.reference_epoch() != *state.reference_epoch() ||
      *status.reference_kind() != 1 ||
      *status.motion_epoch() != *state.motion_epoch() ||
      *status.management_epoch() != *state.management_epoch() ||
      *status.real_motion_permitted() || *status.permit_remaining_us() != 0 ||
      *state.hold_ready())
    return reject("NOT_RELEASED_RETAINED_REFERENCE");
  if ((shape != ManualRecoveryShape::kInterruptedRestFault &&
       !*status.release_committed()) ||
      *status.cleanup_state() != 3 || *status.evidence_lost())
    return reject("DISABLE_EVIDENCE_MISSING");
  if (dummy_ros_control::raw_position_joint_mask(state) != 63)
    return reject("RAW_FEEDBACK_MISSING");
  for (unsigned i = 0; i < 6; ++i)
    if (*dummy_ros_control::joint_for_id(state,i)->can_rx_age_us() > 250000)
      return reject("CAN_FEEDBACK_STALE");
  const unsigned core =
      shape == ManualRecoveryShape::kRetainedDisabledIdle ? 2 : 9;
  if (*status.core_state() != core || *state.core_state() != core)
    return reject("CORE_STATE_MISMATCH");
  if (shape == ManualRecoveryShape::kInterruptedRestFault) {
    if (dummy_ros_control::position_valid_joint_mask(state) != 63 ||
        dummy_ros_control::velocity_valid_joint_mask(state) != 63 || *status.release_committed() ||
        *status.first_failure_reason() != 13 ||
        *state.first_failure_reason() != 13 || *status.failure_source() != 4196 ||
        *status.can_failed_mask() != 63 || *state.can_failed_mask() != 63 ||
        *status.attempt_kind() != static_cast<std::uint8_t>(dp::protocol::dummy::AttemptKind::RELEASE) || *status.run_id() == 0 ||
        *status.attempt_id() != *status.run_id() || *status.parent_run_id() == 0 ||
        *status.group_count() != 1 || *status.final_state() != 0 ||
        *status.transaction_phase() != 4 || *state.transaction_phase() != 4)
      return reject("NOT_KNOWN_INTERRUPTED_REST");
    reason.clear();
    return true;
  }
  if (shape == ManualRecoveryShape::kInterruptedReleasedFault &&
      (*status.first_failure_reason() != 4196 ||
       *status.failure_source() != 4196 || *status.attempt_kind() != static_cast<std::uint8_t>(dp::protocol::dummy::AttemptKind::RELEASE) ||
       *status.final_state() !=
           static_cast<std::uint8_t>(dp::protocol::dummy::FinalState::FAULT_LATCHED_AND_RELEASED)))
    return reject("NOT_INTERRUPTED_REST_RELEASED");
  // State仅发布transaction.first_failure；Status在事务无错时回落adapter_error_reason。
  // 本轮4199是适配器REST超时，绝不能要求State复制该来源，更不能接受新的事务错误。
  if (*state.first_failure_reason() != 0 || *state.can_failed_mask() != 0 ||
      *status.can_failed_mask() != 0)
    return reject("TRANSACTION_ERROR_PRESENT");
  if (shape == ManualRecoveryShape::kKnownRestTimeoutFault &&
      (*status.first_failure_reason() != 0x1067 ||
       *status.failure_source() != 0x1067))
    return reject("NOT_KNOWN_REST_TIMEOUT");
  if (shape == ManualRecoveryShape::kRetainedDisabledIdle &&
      *status.first_failure_reason() != 0)
    return reject("IDLE_ERROR_PRESENT");
  reason.clear();
  return true;
}
// DPRAW02只封装真实来源与接收时刻；消息及哈希均使用中央生成的规范记录。
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
inline bool recovery_evidence_chain(const dummy_ref::messages::Evidence& page,const std::array<std::uint8_t,32>& previous){
  using Traits=dp::protocol::canonical::Traits<dummy_ref::messages::Evidence>;
  if(!dummy_ros_control::message_fields_present(page) || *page.evidence_lost() ||
     !std::equal(previous.begin(),previous.end(),page.previous_chain_sha256()->begin()))return false;
  std::array<std::uint8_t,32U+Traits::kMaxEncodedSize> material{};
  std::copy(previous.begin(),previous.end(),material.begin());std::size_t written=0U;
  if(!Traits::encode(page,material.data()+32U,material.size()-32U,&written,dp::protocol::canonical::Purpose::kEvidenceHash))return false;
  std::array<std::uint8_t,32> digest{};SHA256(material.data(),32U+written,digest.data());
  return std::equal(digest.begin(),digest.end(),page.chain_sha256()->begin());
}
inline bool
recovery_disable_group(const dummy_ref::messages::Evidence &p) {
  if (*p.category() != 3 || !*p.final() || *p.expected_mask() != 63 ||
      *p.submitted_mask() != 63 || *p.sent_mask() != 63 || *p.failed_mask() != 0 ||
      *p.evidence_lost() || !*p.group_id())
    return false;
  for (unsigned i = 0; i < 6; ++i)
    if (!(*p.queued_at_us())[i] || (*p.queued_at_us())[i] > (*p.taken_at_us())[i] ||
        (*p.taken_at_us())[i] > (*p.submitted_at_us())[i] ||
        (*p.submitted_at_us())[i] > (*p.sent_at_us())[i] || !(*p.sent_at_us())[i] ||
        (*p.abort_at_us())[i])
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
      !std::equal(data.begin(),data.begin()+8,dummy_ros_control::kCaptureMagic.begin()) ||
      recovery_sha(data) != expected_sha)
    return reject("RAW_FILE_SHA_OR_MAGIC");
  struct Chain {
    std::uint32_t next{};
    std::array<std::uint8_t, 32> digest{};
  };
  std::map<std::uint64_t, Chain> chains;
  dummy_ref::messages::Status status;
  dummy_ref::messages::State state;
  std::uint64_t status_at = 0, state_at = 0;
  dp::comm::SourceId status_source{},state_source{};
  bool begin_seen = false, grant_seen = false, disable_seen = false;
  ManageRequest grant;
  std::uint64_t rejected_parent = 0;
  std::uint32_t begin_motion = 0;
  std::size_t at = 8;
  while(at<data.size()){
    dummy_ros_control::CaptureRecordHeader header;
    if(!dummy_ros_control::decode_capture_header(data.data()+at,data.size()-at,&header))return reject("RAW_TRUNCATED_HEADER");
    at+=dummy_ros_control::kCaptureHeaderSize;
    const auto kind=static_cast<std::uint8_t>(header.kind);const auto size=header.size;
    const auto received=header.received_at_us,boot=header.source.boot_id,session=header.session_id,run=header.run_id;
    const auto seq=header.evidence_seq;
    if(size>data.size()-at || !received || (kind!=4U && boot!=identity.boot))return reject("RAW_ENVELOPE_IDENTITY");
    const auto* raw=data.data()+at;at+=size;
    if (kind == 2) {
      if ((!dp::protocol::canonical::Traits<dummy_ref::messages::State>::decode(raw,size,&state) || !dummy_ros_control::message_fields_present(state)))
        return reject("RAW_STATE_CODEC");
      state_at = received;state_source=header.source;
    } else if (kind == 3) {
      if ((!dp::protocol::canonical::Traits<dummy_ref::messages::Status>::decode(raw,size,&status) || !dummy_ros_control::message_fields_present(status)))
        return reject("RAW_STATUS_CODEC");
      status_at = received;status_source=header.source;
    } else if (kind == 4) {
      ManageRequest request;
      if ((!dp::protocol::canonical::Traits<ManageRequest>::decode(raw,size,&request) || !dummy_ros_control::message_fields_present(request)))
        return reject("RAW_REQUEST_CODEC");
      if (!*request.request_seq() || !*request.request_id() ||
          *request.request_seq() == UINT32_MAX)
        return reject("RAW_REQUEST_SEQUENCE_EXHAUSTED");
      if (std::find(out.request_ids.begin(), out.request_ids.end(),
                    *request.request_id()) != out.request_ids.end())
        return reject("RAW_DUPLICATE_REQUEST_ID");
      out.request_ids.push_back(*request.request_id());
      if (!out.next_management_sequence ||
          static_cast<std::int32_t>(*request.request_seq() + 1 -
                                    out.next_management_sequence) > 0)
        out.next_management_sequence = *request.request_seq() + 1;
      if (*request.operation() == 2) {
        grant_seen = true;
        grant = request;
      }
      if (*request.operation() == 17) {
        if (begin_seen || !grant_seen ||
            *grant.expected_session_id() != *request.expected_session_id() ||
            !std::equal(grant.client_instance_id()->begin(),
                        grant.client_instance_id()->end(),
                        request.client_instance_id()->begin()) ||
            *grant.expected_boot_epoch() != *request.expected_boot_epoch() ||
            *grant.expected_reference_epoch() !=
                *request.expected_reference_epoch() ||
            *grant.expected_profile_id() != *request.expected_profile_id() ||
            !*request.expected_session_id() ||
            !*request.expected_reference_epoch() ||
            *request.expected_boot_epoch() != identity.boot ||
            *request.expected_profile_id() != identity.profile ||
            !std::equal(identity.device.begin(), identity.device.end(),
                        request.domain_device_id()->begin()))
          return reject("RAW_BEGIN_IDENTITY");
        begin_seen = true;
        begin_motion = *request.expected_motion_epoch();
        out.boot = identity.boot;
        out.session = *request.expected_session_id();
        out.reference = *request.expected_reference_epoch();
        std::copy(request.client_instance_id()->begin(),
                  request.client_instance_id()->end(), out.client.begin());
        if (std::all_of(out.client.begin(), out.client.end(),
                        [](auto x) { return x == 0; }))
          return reject("RAW_ZERO_CLIENT");
      }
    } else if (kind == 5) {
      dummy_ref::messages::ManageResponse reply;
      if ((!dp::protocol::canonical::Traits<dummy_ref::messages::ManageResponse>::decode(raw,size,&reply) || !dummy_ros_control::message_fields_present(reply)))
        return reject("RAW_RESPONSE_CODEC");
    } else if (kind == 1) {
      dummy_ref::messages::Evidence p;
      if (!begin_seen ||
          (!dp::protocol::canonical::Traits<dummy_ref::messages::Evidence>::decode(raw,size,&p) || !dummy_ros_control::message_fields_present(p)) ||
          *p.run_id() != run || *p.evidence_seq() != seq ||
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
          !recovery_evidence_chain(p,found->second.digest))
        return reject("RAW_CHAIN_OR_SEQUENCE");
      ++found->second.next;
      std::copy(p.chain_sha256()->begin(), p.chain_sha256()->end(),
                found->second.digest.begin());
      if (*p.category() == 4) {
        if (rejected_parent || disable_seen || *p.attempt_kind() != 1 ||
            *p.attempt_id() != run || *p.parent_run_id() != 0 ||
            *p.motion_epoch() != begin_motion ||
            *p.final_state() !=
                static_cast<std::uint8_t>(dp::protocol::dummy::FinalState::REJECTED_BEFORE_MOTION) ||
            *p.group_count() != 0 || *p.terminal_reason() != 14 ||
            *p.first_failure_source() != 0 || *p.first_failure_axis_mask() != 0 ||
            *p.release_committed() || *p.motion_complete())
          return reject("RAW_PARENT_NOT_REJECTED_BEFORE_MOTION");
        rejected_parent = run;
      } else if (*p.category() == 3) {
        if (disable_seen || !recovery_disable_group(p) ||
            *p.attempt_kind() != static_cast<std::uint8_t>(dp::protocol::dummy::AttemptKind::RELEASE) || !*p.parent_run_id() ||
            *p.first_failure_source() != 4196 ||
            *p.first_failure_axis_mask() != 63 || *p.release_committed() ||
            *p.group_count() != 1 || seq != 0)
          return reject("RAW_INTERRUPTED_DISABLE_GROUP");
        disable_seen = true;
        out.run = run;
        out.parent = *p.parent_run_id();
        out.group = *p.group_id();
        out.motion = *p.motion_epoch();
      } else
        return reject("RAW_UNEXPECTED_GROUP");
    } else
      return reject("RAW_UNKNOWN_RECORD_KIND");
  }
  if (!begin_seen || !disable_seen || rejected_parent != out.parent ||
      !chains.count(out.parent) || chains.at(out.parent).next != 1 ||
      !validate_manual_recovery(
          status,state,status_source,state_source,identity,std::max(state_at,status_at), state_at,
          status_at, ManualRecoveryShape::kInterruptedRestFault, reason) ||
      *status.run_id() != out.run || *status.parent_run_id() != out.parent ||
      *status.motion_epoch() != out.motion ||
      *status.reference_epoch() != out.reference)
    return reject("RAW_FINAL_FAULT_BINDING");
  out.management = *status.management_epoch();
  out.chain = chains.at(out.run).digest;
  out.next_sequence = chains.at(out.run).next;
  out.file_sha = expected_sha;
  reason.clear();
  return true;
}
inline bool
interrupted_rest_live_matches(const ManualInterruptedRestProof &proof,
                              const dummy_ref::messages::Status &status,
                              const dummy_ref::messages::State &state,const dp::comm::SourceId& status_source,const dp::comm::SourceId& state_source) {
  return proof.boot == status_source.boot_id &&
         proof.reference == *status.reference_epoch() &&
         proof.motion == *status.motion_epoch() &&
         proof.management == *status.management_epoch() &&
         proof.run == *status.run_id() &&
         proof.parent == *status.parent_run_id() &&
         proof.boot == state_source.boot_id &&
         proof.reference == *state.reference_epoch() &&
         proof.motion == *state.motion_epoch() &&
         proof.management == *state.management_epoch() && proof.session &&
         proof.next_sequence == 1;
}
inline bool interrupted_release_request_matches(
    const ManageRequest &request, const ManualInterruptedRestProof &proof,
    const dummy_ref::messages::Status &status,
    const dummy_ref::messages::State &state,const dp::comm::SourceId& status_source,const dp::comm::SourceId& state_source) {
  return interrupted_rest_live_matches(proof,status,state,status_source,state_source) &&
         *request.operation() == 10 && *request.request_id() != 0 &&
         *request.request_seq() >= proof.next_management_sequence &&
         *request.expected_boot_epoch() == proof.boot &&
         std::find(proof.request_ids.begin(), proof.request_ids.end(),
                   *request.request_id()) == proof.request_ids.end() &&
         *request.expected_profile_id() == proof.profile &&
         *request.configured_mask() == 63 &&
         std::equal(proof.device.begin(), proof.device.end(),
                    request.domain_device_id()->begin()) &&
         std::equal(proof.policy.begin(), proof.policy.end(),
                    request.compiled_policy_sha256()->begin()) &&
         *request.expected_session_id() == proof.session &&
         *request.expected_reference_epoch() == *state.reference_epoch() &&
         *request.expected_motion_epoch() == *state.motion_epoch() &&
         *request.expected_management_epoch() == *state.management_epoch() &&
         *request.freshness_ticket() == *state.freshness_ticket() &&
         std::equal(proof.client.begin(), proof.client.end(),
                    request.client_instance_id()->begin());
}

// 新D10必须真实新增Disable组并封Release原终页，不能用Status链或旧SENT代替。
struct InterruptedRestReleaseEvidence {
  ManualInterruptedRestProof proof;
  bool disable_seen = false, terminal_seen = false;
  std::uint64_t terminal_received_us = 0, new_group = 0;
  explicit InterruptedRestReleaseEvidence(ManualInterruptedRestProof value)
      : proof(value) {}
  bool consume(const dummy_ref::messages::Evidence& page,const dp::comm::SourceId& source,std::uint64_t received,
               std::string &reason) {
    if (source.boot_id!=proof.boot || !recovery_evidence_chain(page,proof.chain) ||
        *page.run_id() != proof.run || *page.attempt_id() != proof.run ||
        *page.attempt_kind() != static_cast<std::uint8_t>(dp::protocol::dummy::AttemptKind::RELEASE) || *page.parent_run_id() != proof.parent ||
        *page.evidence_seq() != proof.next_sequence || terminal_seen ||
        !received) {
      reason = "NEW_RELEASE_CHAIN_OR_IDENTITY";
      return false;
    }
    if (*page.category() == 3) {
      if (disable_seen || !recovery_disable_group(page) ||
          *page.group_id() <= proof.group ||
          *page.motion_epoch() != proof.motion + 1 ||
          *page.first_failure_source() != 4196 ||
          *page.first_failure_axis_mask() != 63 || *page.group_count() != 2 ||
          *page.release_committed()) {
        reason = "NEW_DISABLE_GROUP_INVALID";
        return false;
      }
      disable_seen = true;
      new_group = *page.group_id();
    } else if (*page.category() == 4) {
      if (!disable_seen || !*page.final() || !*page.release_committed() ||
          *page.motion_complete() ||
          *page.final_state() !=
              static_cast<std::uint8_t>(dp::protocol::dummy::FinalState::FAULT_LATCHED_AND_RELEASED) ||
          *page.first_failure_source() != 4196 ||
          *page.first_failure_axis_mask() != 63 || *page.group_count() != 2 ||
          *page.motion_epoch() != proof.motion) {
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
    std::copy(page.chain_sha256()->begin(), page.chain_sha256()->end(),
              proof.chain.begin());
    reason.clear();
    return true;
  }
};
} // namespace dummy_motion_client
