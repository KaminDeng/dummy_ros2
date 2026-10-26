#include "dummy_motion_client/manual_ros_recovery.hpp"
#include <cassert>
#include <cstdio>
using namespace dummy_motion_client;
int main() {
  ManualRecoveryIdentity expected;
  expected.device[0] = 1;
  expected.boot = 7;
  expected.profile = 201;
  std::array<std::uint8_t, 32> hash{};
  hash[0] = 42;
  expected.bundle.firmware_hash = expected.bundle.contract_hash =
      expected.bundle.model_hash = expected.bundle.profile_hash =
          expected.bundle.policy_hash = hash;
  dummy_wire::DummyDeviceStatusV2 status;
  dummy_wire::DummyJointStateV2 state;
  status.set_schema_major(2);
  state.set_schema_major(2);
  status.set_schema_minor(dummy_ref::protocol::kSchemaMinor);
  state.set_schema_minor(dummy_ref::protocol::kSchemaMinor);
  std::uint8_t device_wire[16]{};
  device_wire[0] = 1;
  status.set_device_id(device_wire);
  state.set_device_id(device_wire);
  status.set_boot_epoch(7);
  state.set_boot_epoch(7);
  status.set_profile_id(201);
  state.set_profile_id(201);
  status.set_reference_epoch(1);
  state.set_reference_epoch(1);
  status.set_reference_kind(1);
  status.set_configured_mask(63);
  state.set_configured_mask(63);
  state.set_raw_position_valid_mask(63);
  state.set_position_valid_mask(63);
  state.set_velocity_valid_mask(63);
  status.set_core_state(9);
  state.set_core_state(9);
  status.set_first_failure_reason(0x1067);
  state.set_first_failure_reason(0);
  status.set_failure_source(0x1067);
  status.set_release_committed(true);
  status.set_cleanup_state(3);
  std::uint8_t hash_wire[32]{};
  hash_wire[0] = 42;
  status.set_firmware_sha256(hash_wire);
  status.set_contract_sha256(hash_wire);
  status.set_model_sha256(hash_wire);
  status.set_profile_sha256(hash_wire);
  status.set_compiled_policy_sha256(hash_wire);
  std::string error;
  auto valid = [&](ManualRecoveryShape shape =
                       ManualRecoveryShape::kKnownRestTimeoutFault) {
    return validate_manual_recovery(status, state, expected, 1000000, 990000,
                                    750000, shape, error);
  };
  assert(valid());
  // run5真实故障不属于4199旧模式，不能绕过原页证明。
  status.set_release_committed(false);
  status.set_first_failure_reason(13);
  status.set_failure_source(4196);
  state.set_first_failure_reason(13);
  status.set_can_failed_mask(63);
  state.set_can_failed_mask(63);
  status.set_transaction_phase(4);
  state.set_transaction_phase(4);
  status.set_attempt_kind(DP_PROTOCOL_DPSDK_INTERFACES_MSG_DUMMYPROTOCOLV2_MESSAGE_ATTEMPT_KIND_RELEASE);
  status.set_run_id(59);
  status.set_attempt_id(59);
  status.set_parent_run_id(58);
  status.set_group_count(1);
  assert(!valid());
  assert(validate_manual_recovery(
      status, state, expected, 1000000, 990000, 750000,
      ManualRecoveryShape::kInterruptedRestFault, error));
  status.set_release_committed(true);
  status.set_first_failure_reason(0x1067);
  status.set_failure_source(0x1067);
  state.set_first_failure_reason(0);
  status.set_can_failed_mask(0);
  state.set_can_failed_mask(0);
  auto original = status;
  auto original_state = state;
  for (int bad = 0; bad < 20; ++bad) {
    status = original;
    state = original_state;
    if (bad == 0)
      status.set_boot_epoch(8);
    if (bad == 1)
      state.set_boot_epoch(8);
    if (bad == 2)
      status.set_reference_epoch(2);
    if (bad == 3)
      status.set_session_id(3);
    if (bad == 4)
      state.set_session_id(3);
    if (bad == 5)
      status.set_real_motion_permitted(true);
    if (bad == 6)
      status.set_permit_remaining_us(1);
    if (bad == 7)
      status.set_release_committed(false);
    if (bad == 8)
      status.set_cleanup_state(2);
    if (bad == 9)
      status.set_evidence_lost(true);
    if (bad == 10)
      status.set_first_failure_reason(0x100a);
    if (bad == 11)
      state.set_first_failure_reason(0x100a);
    if (bad == 12)
      state.set_raw_position_valid_mask(31);
    if (bad == 13)
      state.set_core_state(4);
    if (bad == 14)
      status.set_reference_kind(0);
    if (bad == 15)
      status.set_motion_epoch(2);
    if (bad == 16)
      status.set_failure_source(0x100a);
    if (bad == 17)
      state.set_first_failure_reason(0x1067);
    if (bad == 18)
      state.set_can_failed_mask(1);
    if (bad == 19)
      status.set_can_failed_mask(1);
    assert(!valid());
  }
  status = original;
  state = original_state;
  assert(!validate_manual_recovery(
      status, state, expected, 1000000, 749999, 750000,
      ManualRecoveryShape::kKnownRestTimeoutFault, error));
  assert(!validate_manual_recovery(
      status, state, expected, 2000000, 1990000, 999999,
      ManualRecoveryShape::kKnownRestTimeoutFault, error));
  std::uint32_t can_ages[6]{};
  can_ages[2] = 250001;
  state.set_can_rx_age_us(can_ages);
  assert(!valid());
  state = original_state;
  hash_wire[0] = 43;
  status.set_firmware_sha256(hash_wire);
  assert(!valid());
  status = original;
  status.set_core_state(2);
  state.set_core_state(2);
  status.set_first_failure_reason(0);
  assert(valid(ManualRecoveryShape::kRetainedDisabledIdle));
  assert(!valid());
  // 独立构造公共Codec原件；seed/逐页链与故障绑定都必须成立。
  status = original;
  state = original_state;
  status.set_release_committed(false);
  status.set_first_failure_reason(13);
  status.set_failure_source(4196);
  state.set_first_failure_reason(13);
  status.set_can_failed_mask(63);
  state.set_can_failed_mask(63);
  status.set_transaction_phase(4);
  state.set_transaction_phase(4);
  status.set_attempt_kind(DP_PROTOCOL_DPSDK_INTERFACES_MSG_DUMMYPROTOCOLV2_MESSAGE_ATTEMPT_KIND_RELEASE);
  status.set_run_id(59);
  status.set_attempt_id(59);
  status.set_parent_run_id(58);
  status.set_group_count(1);
  status.set_final_state(0);
  status.set_motion_epoch(118);
  state.set_motion_epoch(118);
  status.set_management_epoch(181);
  state.set_management_epoch(181);
  state.set_freshness_ticket(99);
  std::uint8_t client_wire[16]{};
  client_wire[0] = 9;
  ManageRequest grant;
  grant.set_schema_major(2);
  grant.set_schema_minor(dummy_ref::protocol::kSchemaMinor);
  grant.set_request_id(1);
  grant.set_request_seq(1);
  grant.set_operation(2);
  grant.set_device_id(device_wire);
  grant.set_client_instance_id(client_wire);
  grant.set_expected_boot_epoch(7);
  grant.set_expected_profile_id(201);
  grant.set_expected_session_id(3);
  grant.set_expected_reference_epoch(1);
  grant.set_configured_mask(63);
  grant.set_compiled_policy_sha256(hash_wire);
  // hash_wire先前被负例改为43；原件应绑定42。
  hash_wire[0] = 42;
  grant.set_compiled_policy_sha256(hash_wire);
  auto begin = grant;
  begin.set_expected_motion_epoch(115);
  begin.set_operation(17);
  begin.set_request_id(2);
  begin.set_request_seq(2);
  std::vector<std::uint8_t> raw{'D', 'P', 'R', 'A', 'W', '0', '1', 0};
  auto record = [&](unsigned kind, std::uint64_t session, std::uint64_t run,
                    unsigned seq, const auto &bytes) {
    auto le = [&](std::uint64_t value, unsigned n) {
      for (unsigned i = 0; i < n; ++i)
        raw.push_back(std::uint8_t(value >> (8 * i)));
    };
    le(kind, 1);
    le(bytes.size(), 2);
    le(1000000, 8);
    le(7, 8);
    le(session, 8);
    le(run, 8);
    le(seq, 4);
    raw.insert(raw.end(), bytes.begin(), bytes.end());
  };
  auto request_bytes = [&](const auto &r) {
    std::vector<std::uint8_t> bytes(
        dummy_wire::DummyManageV2RequestCodec::kEncodedSize);
    assert(dummy_wire::DummyManageV2RequestCodec::encode(
               r, bytes.data(), bytes.size()) == dummy_wire::CodecResult::kOk);
    return bytes;
  };
  record(4, 3, 0, 0, request_bytes(grant));
  record(4, 3, 0, 0, request_bytes(begin));
  auto page_bytes = [&](auto p, std::uint64_t run) {
    std::vector<std::uint8_t> seed(192, 0);
    std::size_t index = 0;
    for (auto value : {run, std::uint64_t(7)})
      for (unsigned i = 0; i < 8; ++i)
        seed[index++] = std::uint8_t(value >> (8 * i));
    for (auto value : client_wire)
      seed[index++] = value;
    for (int i = 0; i < 4; ++i)
      for (auto value : hash_wire)
        seed[index++] = value;
    auto previous = recovery_sha(seed);
    std::uint8_t digest_wire[32];
    std::copy(previous.begin(), previous.end(), digest_wire);
    p.set_previous_chain_sha256(digest_wire);
    std::vector<std::uint8_t> bytes(
        dummy_wire::DummyAttemptEvidenceV2Codec::kEncodedSize);
    assert(dummy_wire::DummyAttemptEvidenceV2Codec::encode(
               p, bytes.data(), bytes.size()) == dummy_wire::CodecResult::kOk);
    std::vector<std::uint8_t> material(previous.begin(), previous.end());
    material.insert(material.end(), bytes.begin(),
                    bytes.begin() + dummy_wire::DummyAttemptEvidenceV2Codec::
                                        kOffset_previous_chain_sha256);
    material.push_back(0);
    auto digest = recovery_sha(material);
    std::copy(digest.begin(), digest.end(), digest_wire);
    p.set_chain_sha256(digest_wire);
    assert(dummy_wire::DummyAttemptEvidenceV2Codec::encode(
               p, bytes.data(), bytes.size()) == dummy_wire::CodecResult::kOk);
    return bytes;
  };
  dummy_wire::DummyAttemptEvidenceV2 parent;
  parent.set_schema_major(2);
  parent.set_schema_minor(dummy_ref::protocol::kSchemaMinor);
  parent.set_run_id(58);
  parent.set_motion_epoch(115);
  parent.set_attempt_id(58);
  parent.set_attempt_kind(1);
  parent.set_category(4);
  parent.set_final(true);
  parent.set_final_state(3);
  parent.set_terminal_reason(14);
  record(1, 3, 58, 0, page_bytes(parent, 58));
  auto page = parent;
  page.set_run_id(59);
  page.set_attempt_id(59);
  page.set_attempt_kind(DP_PROTOCOL_DPSDK_INTERFACES_MSG_DUMMYPROTOCOLV2_MESSAGE_ATTEMPT_KIND_RELEASE);
  page.set_parent_run_id(58);
  page.set_group_id(14143);
  page.set_motion_epoch(118);
  page.set_category(3);
  page.set_expected_mask(63);
  page.set_submitted_mask(63);
  page.set_sent_mask(63);
  page.set_group_count(1);
  page.set_final_state(0);
  page.set_terminal_reason(0);
  page.set_first_failure_source(4196);
  page.set_first_failure_axis_mask(63);
  std::uint64_t times[6];
  std::fill(std::begin(times), std::end(times), 1);
  page.set_queued_at_us(times);
  page.set_taken_at_us(times);
  page.set_submitted_at_us(times);
  page.set_sent_at_us(times);
  record(1, 3, 59, 0, page_bytes(page, 59));
  std::vector<std::uint8_t> sb(
      dummy_wire::DummyDeviceStatusV2Codec::kEncodedSize),
      jb(dummy_wire::DummyJointStateV2Codec::kEncodedSize);
  assert(dummy_wire::DummyDeviceStatusV2Codec::encode(
             status, sb.data(), sb.size()) == dummy_wire::CodecResult::kOk);
  assert(dummy_wire::DummyJointStateV2Codec::encode(
             state, jb.data(), jb.size()) == dummy_wire::CodecResult::kOk);
  record(3, 0, 59, 0, sb);
  record(2, 0, 59, 0, jb);
  ManualInterruptedRestProof proof;
  const auto raw_sha = recovery_sha(raw);
  const bool proof_ok =
      validate_interrupted_rest_raw(raw, raw_sha, expected, proof, error);
  if (!proof_ok)
    std::fprintf(stderr, "proof error=%s\n", error.c_str());
  assert(proof_ok);
  assert(proof.session == 3 && proof.run == 59 && proof.motion == 118 &&
         proof.group == 14143 && proof.next_sequence == 1);
  auto release = grant;
  release.set_operation(10);
  release.set_request_id(100);
  release.set_request_seq(proof.next_management_sequence);
  release.set_expected_motion_epoch(118);
  release.set_expected_management_epoch(181);
  release.set_freshness_ticket(99);
  assert(interrupted_release_request_matches(release, proof, status, state));
  release.set_expected_motion_epoch(117);
  assert(!interrupted_release_request_matches(release, proof, status, state));
  release.set_expected_motion_epoch(118);
  release.set_expected_session_id(0);
  assert(!interrupted_release_request_matches(release, proof, status, state));
  release.set_expected_session_id(3);
  client_wire[0] = 8;
  release.set_client_instance_id(client_wire);
  assert(!interrupted_release_request_matches(release, proof, status, state));
  auto bad_identity = expected;
  bad_identity.boot = 8;
  assert(
      !validate_interrupted_rest_raw(raw, raw_sha, bad_identity, proof, error));
  auto wrong_sha = raw_sha;
  wrong_sha[0] ^= 1;
  assert(
      !validate_interrupted_rest_raw(raw, wrong_sha, expected, proof, error));
  for (auto index :
       {std::size_t(0), std::size_t(8),
        std::size_t(
            8 + 39 + request_bytes(grant).size() + 39 +
            request_bytes(begin).size() + 39 +
            dummy_wire::DummyAttemptEvidenceV2Codec::kOffset_chain_sha256)}) {
    auto damaged = raw;
    damaged[index] ^= 1;
    assert(!validate_interrupted_rest_raw(damaged, recovery_sha(damaged),
                                          expected, proof, error));
  }
  assert(validate_interrupted_rest_raw(raw, raw_sha, expected, proof, error));
  auto encode_chained = [&](auto p, const auto &previous) {
    std::uint8_t digest[32];
    std::copy(previous.begin(), previous.end(), digest);
    p.set_previous_chain_sha256(digest);
    std::vector<std::uint8_t> bytes(
        dummy_wire::DummyAttemptEvidenceV2Codec::kEncodedSize);
    if (dummy_wire::DummyAttemptEvidenceV2Codec::encode(
            p, bytes.data(), bytes.size()) != dummy_wire::CodecResult::kOk)
      return std::vector<std::uint8_t>{};
    std::vector<std::uint8_t> material(previous.begin(), previous.end());
    material.insert(material.end(), bytes.begin(),
                    bytes.begin() + dummy_wire::DummyAttemptEvidenceV2Codec::
                                        kOffset_previous_chain_sha256);
    material.push_back(p.evidence_lost() ? 1 : 0);
    const auto chain = recovery_sha(material);
    std::copy(chain.begin(), chain.end(), digest);
    p.set_chain_sha256(digest);
    if (dummy_wire::DummyAttemptEvidenceV2Codec::encode(
            p, bytes.data(), bytes.size()) != dummy_wire::CodecResult::kOk)
      return std::vector<std::uint8_t>{};
    return bytes;
  };
  auto new_disable = page;
  new_disable.set_evidence_seq(1);
  new_disable.set_group_id(14144);
  new_disable.set_motion_epoch(119);
  new_disable.set_group_count(2);
  for (int bad = 0; bad < 8; ++bad) {
    InterruptedRestReleaseEvidence evidence(proof);
    auto wrong = new_disable;
    if (bad == 0)
      wrong.set_sent_mask(31);
    if (bad == 1)
      wrong.set_failed_mask(1);
    if (bad == 2)
      wrong.set_evidence_lost(true);
    if (bad == 3)
      wrong.set_evidence_seq(2);
    if (bad == 4)
      wrong.set_motion_epoch(118);
    if (bad == 5)
      wrong.set_category(2);
    if (bad == 6)
      wrong.set_parent_run_id(57);
    if (bad == 7)
      wrong.set_release_committed(true);
    assert(
        !evidence.consume(encode_chained(wrong, proof.chain), 2000000, error));
    assert(!evidence.terminal_seen);
  }
  InterruptedRestReleaseEvidence evidence(proof);
  assert(evidence.consume(encode_chained(new_disable, proof.chain), 2000000,
                          error));
  assert(evidence.disable_seen && !evidence.terminal_seen);
  auto terminal = page;
  terminal.set_evidence_seq(2);
  terminal.set_category(4);
  terminal.set_group_id(0);
  terminal.set_group_count(2);
  terminal.set_expected_mask(0);
  terminal.set_submitted_mask(0);
  terminal.set_sent_mask(0);
  terminal.set_final_state(
      DP_PROTOCOL_DPSDK_INTERFACES_MSG_DUMMYPROTOCOLV2_MESSAGE_FINAL_STATE_FAULT_LATCHED_AND_RELEASED);
  terminal.set_release_committed(true);
  std::fill(std::begin(times), std::end(times), 0);
  terminal.set_queued_at_us(times);
  terminal.set_taken_at_us(times);
  terminal.set_submitted_at_us(times);
  terminal.set_sent_at_us(times);
  auto no_release = terminal;
  no_release.set_release_committed(false);
  assert(!evidence.consume(encode_chained(no_release, evidence.proof.chain),
                           2001000, error));
  assert(!evidence.terminal_seen);
  assert(evidence.consume(encode_chained(terminal, evidence.proof.chain),
                          2002000, error));
  assert(evidence.terminal_seen);
  assert(!evidence.consume(encode_chained(terminal, evidence.proof.chain),
                           2003000, error));
}
