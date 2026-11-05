#include "dummy_motion_client/manual_ros_recovery.hpp"
#include <cassert>
#include <cstdio>
#include "../../dummy_ros_control/test/owned_message_fixtures.hpp"
#include "typed_management_fixture.hpp"
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
  auto status=owned_fixture::status();
  auto state=owned_fixture::state();
  auto status_source=typed_management_fixture::source(),state_source=status_source;
  std::uint8_t device_wire[16]{};
  device_wire[0] = 1;
  status.domain_device_id().emplace().assign(device_wire,16U);
  state.domain_device_id().emplace().assign(device_wire,16U);


  status.profile_id()=201;
  state.profile_id()=201;
  status.reference_epoch()=1;
  state.reference_epoch()=1;
  status.reference_kind()=1;
  status.configured_mask()=63;

  for(auto& joint:*state.joints())joint.raw_position_valid_quality()=(63U&(1U<<*joint.joint_id()))!=0U;
  for(auto& joint:*state.joints())joint.position_valid_quality()=(63U&(1U<<*joint.joint_id()))!=0U;
  for(auto& joint:*state.joints())joint.velocity_valid_quality()=(63U&(1U<<*joint.joint_id()))!=0U;
  status.core_state()=9;
  state.core_state()=9;
  status.first_failure_reason()=0x1067;
  state.first_failure_reason()=0;
  status.failure_source()=0x1067;
  status.release_committed()=true;
  status.cleanup_state()=3;
  std::uint8_t hash_wire[32]{};
  hash_wire[0] = 42;
  status.firmware_sha256().emplace().assign(hash_wire,32U);
  status.contract_sha256().emplace().assign(hash_wire,32U);
  status.model_sha256().emplace().assign(hash_wire,32U);
  status.profile_sha256().emplace().assign(hash_wire,32U);
  status.compiled_policy_sha256().emplace().assign(hash_wire,32U);
  std::string error;
  auto valid = [&](ManualRecoveryShape shape =
                       ManualRecoveryShape::kKnownRestTimeoutFault) {
    return validate_manual_recovery(status, state, status_source,state_source,expected, 1000000, 990000,
                                    750000, shape, error);
  };
  assert(valid());
  // run5真实故障不属于4199旧模式，不能绕过原页证明。
  status.release_committed()=false;
  status.first_failure_reason()=13;
  status.failure_source()=4196;
  state.first_failure_reason()=13;
  status.can_failed_mask()=63;
  state.can_failed_mask()=63;
  status.transaction_phase()=4;
  state.transaction_phase()=4;
  status.attempt_kind()=static_cast<std::uint8_t>(dp::protocol::dummy::AttemptKind::RELEASE);
  status.run_id()=59;
  status.attempt_id()=59;
  status.parent_run_id()=58;
  status.group_count()=1;
  assert(!valid());
  assert(validate_manual_recovery(
      status, state, status_source,state_source,expected, 1000000, 990000, 750000,
      ManualRecoveryShape::kInterruptedRestFault, error));
  status.release_committed()=true;
  status.first_failure_reason()=0x1067;
  status.failure_source()=0x1067;
  state.first_failure_reason()=0;
  status.can_failed_mask()=0;
  state.can_failed_mask()=0;
  auto original = status;
  auto original_state = state;
  for (int bad = 0; bad < 20; ++bad) {
    status = original;
    state = original_state;status_source.boot_id=state_source.boot_id=7;
    if (bad == 0)
      status_source.boot_id=8;
    if (bad == 1)
      state_source.boot_id=8;
    if (bad == 2)
      status.reference_epoch()=2;
    if (bad == 3)
      status.session_id()=3;
    if (bad == 4)
      state.session_id()=3;
    if (bad == 5)
      status.real_motion_permitted()=true;
    if (bad == 6)
      status.permit_remaining_us()=1;
    if (bad == 7)
      status.release_committed()=false;
    if (bad == 8)
      status.cleanup_state()=2;
    if (bad == 9)
      status.evidence_lost()=true;
    if (bad == 10)
      status.first_failure_reason()=0x100a;
    if (bad == 11)
      state.first_failure_reason()=0x100a;
    if (bad == 12)
      for(auto& joint:*state.joints())joint.raw_position_valid_quality()=(31U&(1U<<*joint.joint_id()))!=0U;
    if (bad == 13)
      state.core_state()=4;
    if (bad == 14)
      status.reference_kind()=0;
    if (bad == 15)
      status.motion_epoch()=2;
    if (bad == 16)
      status.failure_source()=0x100a;
    if (bad == 17)
      state.first_failure_reason()=0x1067;
    if (bad == 18)
      state.can_failed_mask()=1;
    if (bad == 19)
      status.can_failed_mask()=1;
    assert(!valid());
  }
  status = original;
  state = original_state;status_source.boot_id=state_source.boot_id=7;
  assert(!validate_manual_recovery(
      status, state, status_source,state_source,expected, 1000000, 749999, 750000,
      ManualRecoveryShape::kKnownRestTimeoutFault, error));
  assert(!validate_manual_recovery(
      status, state, status_source,state_source,expected, 2000000, 1990000, 999999,
      ManualRecoveryShape::kKnownRestTimeoutFault, error));
  std::uint32_t can_ages[6]{};
  can_ages[2] = 250001;
  for(auto& joint:*state.joints())joint.can_rx_age_us()=can_ages[*joint.joint_id()];
  assert(!valid());
  state = original_state;
  hash_wire[0] = 43;
  status.firmware_sha256().emplace().assign(hash_wire,32U);
  assert(!valid());
  status = original;
  status.core_state()=2;
  state.core_state()=2;
  status.first_failure_reason()=0;
  assert(valid(ManualRecoveryShape::kRetainedDisabledIdle));
  assert(!valid());
  // 独立构造中央规范记录原件；seed/逐页链与故障绑定都必须成立。
  status = original;
  state = original_state;status_source.boot_id=state_source.boot_id=7;
  status.release_committed()=false;
  status.first_failure_reason()=13;
  status.failure_source()=4196;
  state.first_failure_reason()=13;
  status.can_failed_mask()=63;
  state.can_failed_mask()=63;
  status.transaction_phase()=4;
  state.transaction_phase()=4;
  status.attempt_kind()=static_cast<std::uint8_t>(dp::protocol::dummy::AttemptKind::RELEASE);
  status.run_id()=59;
  status.attempt_id()=59;
  status.parent_run_id()=58;
  status.group_count()=1;
  status.final_state()=0;
  status.motion_epoch()=118;
  state.motion_epoch()=118;
  status.management_epoch()=181;
  state.management_epoch()=181;
  state.freshness_ticket()=99;
  std::uint8_t client_wire[16]{};
  client_wire[0] = 9;
  auto grant=owned_fixture::request();
  grant.request_id()=1;
  grant.request_seq()=1;
  grant.operation()=2;
  grant.domain_device_id().emplace().assign(device_wire,16U);
  grant.client_instance_id().emplace().assign(client_wire,16U);
  grant.expected_boot_epoch()=7;
  grant.expected_profile_id()=201;
  grant.expected_session_id()=3;
  grant.expected_reference_epoch()=1;
  grant.configured_mask()=63;
  grant.compiled_policy_sha256().emplace().assign(hash_wire,32U);
  // hash_wire先前被负例改为43；原件应绑定42。
  hash_wire[0] = 42;
  grant.compiled_policy_sha256().emplace().assign(hash_wire,32U);
  auto begin = grant;
  begin.expected_motion_epoch()=115;
  begin.operation()=17;
  begin.request_id()=2;
  begin.request_seq()=2;
  using namespace dummy_ros_control;
  std::vector<std::uint8_t> raw(kCaptureMagic.begin(),kCaptureMagic.end());
  auto record=[&](unsigned kind,std::uint64_t session,std::uint64_t run,unsigned seq,const auto& bytes){
    auto actual_source=status_source;
    if(kind==4U){actual_source.boot_id=99;actual_source.device.bytes[0]=8;}
    CaptureRecordHeader header{static_cast<CaptureRecordKind>(kind),static_cast<std::uint16_t>(bytes.size()),1000000,actual_source,session,run,seq};
    const auto encoded=encode_capture_header(header);raw.insert(raw.end(),encoded.begin(),encoded.end());raw.insert(raw.end(),bytes.begin(),bytes.end());
  };
  auto encode=[](const auto& value,dp::protocol::canonical::Purpose purpose=dp::protocol::canonical::Purpose::kRecord){
    using Traits=dp::protocol::canonical::Traits<std::decay_t<decltype(value)>>;
    std::vector<std::uint8_t> bytes(Traits::kMaxEncodedSize);std::size_t written=0;
    assert(Traits::encode(value,bytes.data(),bytes.size(),&written,purpose));bytes.resize(written);return bytes;
  };
  auto request_bytes=encode;
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
    p.previous_chain_sha256().emplace().assign(digest_wire,32U);
    auto hash_bytes=encode(p,dp::protocol::canonical::Purpose::kEvidenceHash);
    std::vector<std::uint8_t> material(previous.begin(),previous.end());material.insert(material.end(),hash_bytes.begin(),hash_bytes.end());
    const auto digest=recovery_sha(material);std::copy(digest.begin(),digest.end(),digest_wire);p.chain_sha256().emplace().assign(digest_wire,32U);
    return encode(p);
  };
  auto parent=owned_fixture::evidence();
  parent.run_id()=58;
  parent.motion_epoch()=115;
  parent.attempt_id()=58;
  parent.attempt_kind()=1;
  parent.category()=4;
  parent.final()=true;
  parent.final_state()=3;
  parent.terminal_reason()=14;
  record(1, 3, 58, 0, page_bytes(parent, 58));
  auto page = parent;
  page.run_id()=59;
  page.attempt_id()=59;
  page.attempt_kind()=static_cast<std::uint8_t>(dp::protocol::dummy::AttemptKind::RELEASE);
  page.parent_run_id()=58;
  page.group_id()=14143;
  page.motion_epoch()=118;
  page.category()=3;
  page.expected_mask()=63;
  page.submitted_mask()=63;
  page.sent_mask()=63;
  page.group_count()=1;
  page.final_state()=0;
  page.terminal_reason()=0;
  page.first_failure_source()=4196;
  page.first_failure_axis_mask()=63;
  std::uint64_t times[6];
  std::fill(std::begin(times), std::end(times), 1);
  page.queued_at_us().emplace().assign(times,6U);
  page.taken_at_us().emplace().assign(times,6U);
  page.submitted_at_us().emplace().assign(times,6U);
  page.sent_at_us().emplace().assign(times,6U);
  record(1, 3, 59, 0, page_bytes(page, 59));
  const auto sb=encode(status),jb=encode(state);
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
  release.operation()=10;
  release.request_id()=100;
  release.request_seq()=proof.next_management_sequence;
  release.expected_motion_epoch()=118;
  release.expected_management_epoch()=181;
  release.freshness_ticket()=99;
  assert(interrupted_release_request_matches(release, proof, status, state,status_source,state_source));
  release.expected_motion_epoch()=117;
  assert(!interrupted_release_request_matches(release, proof, status, state,status_source,state_source));
  release.expected_motion_epoch()=118;
  release.expected_session_id()=0;
  assert(!interrupted_release_request_matches(release, proof, status, state,status_source,state_source));
  release.expected_session_id()=3;
  client_wire[0] = 8;
  release.client_instance_id().emplace().assign(client_wire,16U);
  assert(!interrupted_release_request_matches(release, proof, status, state,status_source,state_source));
  auto bad_identity = expected;
  bad_identity.boot = 8;
  assert(
      !validate_interrupted_rest_raw(raw, raw_sha, bad_identity, proof, error));
  auto wrong_sha = raw_sha;
  wrong_sha[0] ^= 1;
  assert(
      !validate_interrupted_rest_raw(raw, wrong_sha, expected, proof, error));
  const auto parent_last_byte=8+3*kCaptureHeaderSize+request_bytes(grant).size()+request_bytes(begin).size()+page_bytes(parent,58).size()-1;
  for(auto index:{std::size_t(0),std::size_t(8),parent_last_byte}){
    auto damaged = raw;
    damaged[index] ^= 1;
    assert(!validate_interrupted_rest_raw(damaged, recovery_sha(damaged),
                                          expected, proof, error));
  }
  assert(validate_interrupted_rest_raw(raw, raw_sha, expected, proof, error));
  auto encode_chained = [&](auto p, const auto &previous) {
    std::uint8_t digest[32];
    std::copy(previous.begin(), previous.end(), digest);
    p.previous_chain_sha256().emplace().assign(digest,32U);
    const auto bytes=encode(p,dp::protocol::canonical::Purpose::kEvidenceHash);
    std::vector<std::uint8_t> material(previous.begin(),previous.end());material.insert(material.end(),bytes.begin(),bytes.end());
    const auto chain=recovery_sha(material);std::copy(chain.begin(),chain.end(),digest);p.chain_sha256().emplace().assign(digest,32U);
    return p;
  };
  auto new_disable = page;
  new_disable.evidence_seq()=1;
  new_disable.group_id()=14144;
  new_disable.motion_epoch()=119;
  new_disable.group_count()=2;
  for (int bad = 0; bad < 8; ++bad) {
    InterruptedRestReleaseEvidence evidence(proof);
    auto wrong = new_disable;
    if (bad == 0)
      wrong.sent_mask()=31;
    if (bad == 1)
      wrong.failed_mask()=1;
    if (bad == 2)
      wrong.evidence_lost()=true;
    if (bad == 3)
      wrong.evidence_seq()=2;
    if (bad == 4)
      wrong.motion_epoch()=118;
    if (bad == 5)
      wrong.category()=2;
    if (bad == 6)
      wrong.parent_run_id()=57;
    if (bad == 7)
      wrong.release_committed()=true;
    assert(
        !evidence.consume(encode_chained(wrong, proof.chain),status_source,2000000, error));
    assert(!evidence.terminal_seen);
  }
  InterruptedRestReleaseEvidence evidence(proof);
  assert(evidence.consume(encode_chained(new_disable, proof.chain),status_source,2000000,
                          error));
  assert(evidence.disable_seen && !evidence.terminal_seen);
  auto terminal = page;
  terminal.evidence_seq()=2;
  terminal.category()=4;
  terminal.group_id()=0;
  terminal.group_count()=2;
  terminal.expected_mask()=0;
  terminal.submitted_mask()=0;
  terminal.sent_mask()=0;
  terminal.final_state()=
      static_cast<std::uint8_t>(dp::protocol::dummy::FinalState::FAULT_LATCHED_AND_RELEASED);
  terminal.release_committed()=true;
  std::fill(std::begin(times), std::end(times), 0);
  terminal.queued_at_us().emplace().assign(times,6U);
  terminal.taken_at_us().emplace().assign(times,6U);
  terminal.submitted_at_us().emplace().assign(times,6U);
  terminal.sent_at_us().emplace().assign(times,6U);
  auto no_release = terminal;
  no_release.release_committed()=false;
  assert(!evidence.consume(encode_chained(no_release, evidence.proof.chain),status_source,2001000, error));
  assert(!evidence.terminal_seen);
  assert(evidence.consume(encode_chained(terminal, evidence.proof.chain),status_source,2002000, error));
  assert(evidence.terminal_seen);
  assert(!evidence.consume(encode_chained(terminal, evidence.proof.chain),status_source,2003000, error));
}
