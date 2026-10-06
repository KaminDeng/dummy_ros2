#include "dummy_motion_client/bootstrap_coordinator.hpp"

#include <cassert>
#include <vector>

struct Client final : dummy_motion_client::ManagementClient {
  std::vector<dummy_motion_client::ManageRequest> requests;
  bool reject=false;
  bool pending=false;
  bool queued=false;
  bool wrong_request_id=false;
  bool prepared_success=false;
  bool release_committed=false;
  std::optional<std::uint64_t> operation_id;
  std::uint64_t submit(const dummy_motion_client::ManageRequest &request, std::uint64_t) override {
    requests.push_back(request); return requests.size();
  }
  std::uint64_t submit_permit(const dummy_motion_client::ManageRequest &request,
                              const dummy_motion_client::PermitBinding &binding,
                              std::uint64_t) override {
    auto sent = request;
    sent.set_request_seq(static_cast<std::uint32_t>(requests.size() + 1U));
    std::array<std::uint8_t,16> owner{};
    std::copy(std::begin(request.client_instance_id()), std::end(request.client_instance_id()), owner.begin());
    std::array<std::uint8_t,32> digest{};
    assert(dummy_motion_client::trusted_bench_permit_digest(
        binding, owner, request.expected_session_id(), sent.request_seq(), &digest));
    std::uint8_t bytes[32]{}; std::copy(digest.begin(), digest.end(), bytes);
    sent.set_permit_sha256(bytes);
    requests.push_back(sent); return requests.size();
  }
  std::optional<dummy_motion_client::ManageReply> poll(std::uint64_t id) override {
    if (pending) return {};
    if (queued) {
      dummy_motion_client::ManageReply reply{};
      reply.request_id=id; reply.queued=true;
      return reply;
    }
    dummy_motion_client::ManageReply reply{}; reply.request_id=wrong_request_id ? id+1U : id;
    reply.result_valid=true;
    reply.succeeded=!reject; reply.rejected=reject; reply.operation_id=operation_id.value_or(id);
    reply.transaction_phase=reject ? 4U : (prepared_success ? 2U : 3U);
    reply.release_committed=release_committed;
    if (reject) { reply.result_code=6U; reply.first_failure_reason=7U; reply.transaction_phase=4U; }
    return reply;
  }
};

int main() {
  using namespace dummy_motion_client;
  Client client;
  ManageRequest identity{};
  identity.set_expected_session_id(42U);
  std::uint8_t device[16]{1U}, owner[16]{2U};
  identity.set_device_id(device); identity.set_client_instance_id(owner);
  std::array<std::uint8_t, 32> compiled_policy{}; compiled_policy[0]=2U;
  BootstrapCoordinator bootstrap(client, identity, compiled_policy, 1000000U);
  BootstrapSnapshot state{7U,0U,0U,0U,0U,9U,2U,1U,true,true};
  state.firmware_sha256[0]=1U; state.contract_sha256[0]=2U;
  state.model_sha256[0]=3U; state.profile_sha256[0]=4U;
  bootstrap.observe(state,0U);
  assert(bootstrap.phase()==BootstrapPhase::kIdle);
  assert(bootstrap.start(state, 1U));
  const std::uint16_t operations[]{1U,2U,3U,5U};
  for (unsigned index=0; index<4; ++index) {
    assert(client.requests.back().operation()==operations[index]);
    if (operations[index]==5U) assert(client.requests.back().compiled_policy_sha256()[0]==2U);
    if (index == 0U) assert(client.requests.back().expected_session_id()==0U);
    else assert(client.requests.back().expected_session_id()==42U);
    bootstrap.tick(2U+index);
    state.management_epoch=index+1U;
    state.control_mode=1U;
    if (index>=1U) state.session_id=42U;
    if (index>=2U) { state.reference_epoch=1U; state.reference_kind=1U; }
    if (index>=3U) { state.real_motion_permitted=true; state.permit_remaining_us=900000U; }
    bootstrap.observe(state,9U+index);
    bootstrap.tick(10U+index);
  }
  assert(bootstrap.ready());
  auto reconnect = state;
  ++reconnect.boot_epoch;
  const auto sent_before_reconnect = client.requests.size();
  assert(!bootstrap.start_operation(reconnect, ManageOperation::kSetPermit, 29U));
  assert(client.requests.size() == sent_before_reconnect);
  const auto authorization=bootstrap.authorization();
  assert(authorization && authorization->identity.expected_session_id()==42U &&
         authorization->identity.expected_reference_epoch()==1U);
  std::array<std::uint8_t,32> first_permit_digest{};
  std::copy(std::begin(client.requests.back().permit_sha256()),
            std::end(client.requests.back().permit_sha256()),first_permit_digest.begin());
  state.real_motion_permitted=false;
  state.permit_remaining_us=0U;
  bootstrap.observe(state,29U);
  assert(bootstrap.start_operation(state,ManageOperation::kSetPermit,30U));
  assert(!std::equal(first_permit_digest.begin(),first_permit_digest.end(),
                     std::begin(client.requests.back().permit_sha256())));
  Client explicit_client;
  BootstrapCoordinator explicit_bootstrap(explicit_client, identity, compiled_policy, 1000000U,30000000U);
  BootstrapSnapshot explicit_state{8U,0U,0U,0U,0U,10U,2U,1U,true,true};
  explicit_state.firmware_sha256=state.firmware_sha256;
  explicit_state.contract_sha256=state.contract_sha256;
  explicit_state.model_sha256=state.model_sha256;
  explicit_state.profile_sha256=state.profile_sha256;
  const ManageOperation explicit_operations[]{ManageOperation::kSelectMode,ManageOperation::kGrantSession,
      ManageOperation::kCaptureReference,ManageOperation::kSetPermit};
  for (const auto operation : explicit_operations) {
    assert(explicit_bootstrap.start_operation(explicit_state,operation,20U));
    assert(explicit_client.requests.back().operation()==wire_value(operation));
    explicit_bootstrap.tick(21U);
    assert(explicit_bootstrap.operation_complete());
    ++explicit_state.management_epoch;
    if(operation==ManageOperation::kSelectMode) explicit_state.control_mode=1U;
    if(operation==ManageOperation::kGrantSession) {
      explicit_state.session_id=42U; explicit_state.control_mode=1U;
    }
    if(operation==ManageOperation::kCaptureReference) {
      explicit_state.reference_epoch=1U; explicit_state.reference_kind=1U;
    }
    if(operation==ManageOperation::kSetPermit) {
      assert(!explicit_bootstrap.authorization().has_value());
      explicit_state.real_motion_permitted=true;
      explicit_state.permit_remaining_us=900000U;
    }
    explicit_bootstrap.observe(explicit_state,22U);
  }
  assert(explicit_bootstrap.last_operation()==ManageOperation::kSetPermit);
  assert(explicit_bootstrap.authorization().has_value());
  explicit_state.motion_epoch=7U;
  explicit_state.management_epoch=4U;
  assert(!explicit_bootstrap.start_renew_lease(explicit_state,0U,40U));
  assert(!explicit_bootstrap.start_renew_lease(explicit_state,3600000001ULL,40U));
  auto new_boot = explicit_state;
  ++new_boot.boot_epoch;
  const auto sent_before_renew = explicit_client.requests.size();
  assert(!explicit_bootstrap.start_renew_lease(new_boot,500000U,40U));
  assert(explicit_client.requests.size() == sent_before_renew);
  assert(!explicit_bootstrap.start_renew_lease(explicit_state,30000000U,5000019U));
  assert(explicit_client.requests.size() == sent_before_renew);
  // A goal spanning the due point can finish in Hold at age 15 s, leaving
  // 10 s before the fail-closed reserve cutoff.
  assert(!explicit_bootstrap.renewal_due(5000019U,30000000U));
  assert(explicit_bootstrap.renewal_due(5000020U,30000000U));
  assert(!explicit_bootstrap.renewal_due(15000020U,20000000U));
  assert(explicit_bootstrap.renewal_due(15000020U,30000000U));
  assert(explicit_bootstrap.start_renew_lease(explicit_state,30000000U,15000020U));
  const auto &renew=explicit_client.requests.back();
  assert(renew.operation()==wire_value(ManageOperation::kRenewLease));
  assert(renew.requested_session_lease_us()==30000000U);
  assert(renew.expected_session_id()==42U && renew.expected_boot_epoch()==8U);
  assert(renew.expected_reference_epoch()==1U && renew.expected_motion_epoch()==7U);
  assert(renew.expected_management_epoch()==4U);
  assert(!explicit_bootstrap.start_renew_lease(explicit_state,30000000U,15000021U));
  explicit_client.queued=true;
  explicit_bootstrap.tick(15000021U);
  assert(explicit_bootstrap.phase()==BootstrapPhase::kRenewLease);
  explicit_client.queued=false;
  explicit_bootstrap.tick(15000022U);
  assert(explicit_bootstrap.operation_complete());
  assert(explicit_bootstrap.last_operation()==ManageOperation::kRenewLease);
  assert(!explicit_bootstrap.authorization().has_value());
  explicit_state.core_state=wire_value(WireCoreState::kRunning);
  explicit_state.state_core_state=wire_value(WireCoreState::kRunning);
  explicit_state.run_id=8U;
  std::uint64_t last_submit=15000020U;
  for(unsigned goal=0;goal<40U;++goal) {
    const auto due=last_submit+5000000U;
    const auto hold=due+10000000U;
    assert(!explicit_bootstrap.renewal_due(due-1U,30000000U));
    assert(explicit_bootstrap.renewal_due(due,30000000U));
    ++explicit_state.management_epoch;
    explicit_bootstrap.observe(explicit_state,hold);
    assert(explicit_bootstrap.start_renew_lease(explicit_state,30000000U,hold));
    explicit_bootstrap.tick(hold+100000U);
    assert(explicit_bootstrap.operation_complete());
    last_submit=hold;
  }
  assert(last_submit>600000000U);
  assert(explicit_state.motion_epoch==7U && explicit_state.run_id==8U);
  const auto next_due=last_submit+5000000U;
  assert(!explicit_bootstrap.start_renew_lease(
    explicit_state,30000000U,last_submit+25000000U));
  assert(explicit_bootstrap.phase()==BootstrapPhase::kOperationComplete);
  assert(explicit_bootstrap.start_renew_lease(explicit_state,30000000U,next_due));
  auto changed_run=explicit_state;
  changed_run.run_id=9U;
  explicit_bootstrap.observe(changed_run,next_due+1U);
  assert(explicit_bootstrap.phase()==BootstrapPhase::kFault);
  assert(std::string(explicit_bootstrap.error())=="RUN_LEASE_BINDING_CHANGED");
  Client mismatch_client;
  BootstrapCoordinator mismatch(mismatch_client,identity,compiled_policy,1000000U,30000000U);
  // A late or unbound RUN renewal cannot be accepted as another lease receipt.
  assert(!mismatch.start_renew_lease(explicit_state,30000000U,UINT64_MAX));
  assert(mismatch_client.requests.empty());
  const auto seed_run_lease=[&](BootstrapCoordinator &coordinator,Client &transport) {
    auto initial=explicit_state;
    initial.session_id=0U;
    initial.reference_epoch=0U;
    initial.motion_epoch=0U;
    initial.run_id=0U;
    initial.core_state=wire_value(WireCoreState::kHold);
    initial.state_core_state=wire_value(WireCoreState::kHold);
    coordinator.observe(initial,0U);
    assert(coordinator.start_operation(initial,ManageOperation::kGrantSession,1U));
    coordinator.tick(2U);
    assert(coordinator.operation_complete() && !transport.requests.empty());
    coordinator.observe(explicit_state,3U);
    assert(coordinator.renewal_due(5000001U,30000000U));
    assert(coordinator.start_renew_lease(explicit_state,30000000U,5000001U));
  };
  Client denied_client;
  BootstrapCoordinator denied(denied_client,identity,compiled_policy,1000000U,30000000U);
  seed_run_lease(denied,denied_client);
  denied_client.reject=true;
  denied.tick(5000002U);
  assert(denied.phase()==BootstrapPhase::kFault);
  Client missing_receipt_client;
  BootstrapCoordinator missing_receipt(missing_receipt_client,identity,compiled_policy,1000000U,30000000U);
  seed_run_lease(missing_receipt,missing_receipt_client);
  missing_receipt_client.operation_id=0U;
  missing_receipt.tick(5000002U);
  assert(missing_receipt.phase()==BootstrapPhase::kFault);
  Client wrong_client;
  BootstrapCoordinator wrong(wrong_client,identity,compiled_policy,1000000U,30000000U);
  seed_run_lease(wrong,wrong_client);
  wrong_client.wrong_request_id=true;
  wrong.tick(5000002U);
  assert(wrong.phase()==BootstrapPhase::kFault);
  assert(std::string(wrong.error())=="LEASE_RENEWAL_UNCONFIRMED");
  Client late_run_client;
  BootstrapCoordinator late_run(late_run_client,identity,compiled_policy,1000000U,30000000U);
  seed_run_lease(late_run,late_run_client);
  late_run_client.queued=true;
  late_run.tick(25000001U);
  assert(late_run.phase()==BootstrapPhase::kFault);
  assert(std::string(late_run.error())=="LEASE_RENEWAL_UNCONFIRMED");
  Client overflow_client;
  BootstrapCoordinator overflow(overflow_client,identity,compiled_policy,1000000U,30000000U);
  auto overflow_state=explicit_state;
  overflow_state.session_id=0U;
  overflow_state.reference_epoch=0U;
  overflow_state.run_id=0U;
  assert(overflow.start_operation(overflow_state,ManageOperation::kGrantSession,UINT64_MAX-1U));
  overflow.tick(UINT64_MAX);
  assert(overflow.phase()==BootstrapPhase::kFault);
  assert(std::string(overflow.error())=="LEASE_DEADLINE_OVERFLOW");
  Client legacy_client;
  BootstrapCoordinator legacy(legacy_client,identity,compiled_policy,1000000U);
  auto legacy_state=explicit_state;
  legacy.observe(legacy_state,0U);
  assert(legacy.start_operation(legacy_state,ManageOperation::kSelectMode,1U));
  legacy.tick(2U);
  assert(legacy.operation_complete());
  assert(!legacy.renewal_due(20000020U,20000000U));
  legacy_state.management_epoch=5U;
  legacy.observe(legacy_state,3U);
  assert(legacy.start_renew_lease(legacy_state,500000U,40U));
  Client unconfirmed_client;
  BootstrapCoordinator unconfirmed(unconfirmed_client, identity, compiled_policy, 1000000U,30000000U);
  auto unconfirmed_state=explicit_state;
  unconfirmed_state.management_epoch=0U;
  unconfirmed_state.session_id=0U;
  unconfirmed_state.reference_epoch=0U;
  unconfirmed.observe(unconfirmed_state,0U);
  assert(unconfirmed.start_operation(unconfirmed_state,ManageOperation::kSelectMode,1U));
  unconfirmed.tick(2U);
  ++unconfirmed_state.management_epoch;
  unconfirmed_state.control_mode=1U;
  unconfirmed.observe(unconfirmed_state,2U);
  assert(unconfirmed.start_operation(unconfirmed_state,ManageOperation::kGrantSession,3U));
  unconfirmed.tick(4U);
  unconfirmed_state.session_id=42U;
  ++unconfirmed_state.management_epoch;
  unconfirmed.observe(unconfirmed_state,4U);
  assert(unconfirmed.start_operation(unconfirmed_state,ManageOperation::kCaptureReference,5U));
  unconfirmed.tick(6U);
  unconfirmed_state.reference_epoch=1U;
  unconfirmed_state.reference_kind=1U;
  ++unconfirmed_state.management_epoch;
  unconfirmed.observe(unconfirmed_state,6U);
  assert(unconfirmed.start_operation(unconfirmed_state,ManageOperation::kSetPermit,7U));
  unconfirmed.tick(8U);
  unconfirmed_state.real_motion_permitted=true;
  unconfirmed_state.permit_remaining_us=900000U;
  ++unconfirmed_state.management_epoch;
  unconfirmed.observe(unconfirmed_state,8U);
  assert(!unconfirmed.renewal_due(5000002U,30000000U));
  assert(unconfirmed.renewal_due(5000003U,30000000U));
  unconfirmed_client.queued=true;
  assert(unconfirmed.start_renew_lease(unconfirmed_state,30000000U,5000003U));
  unconfirmed.tick(5000004U);
  assert(unconfirmed.phase()==BootstrapPhase::kRenewLease);
  unconfirmed.tick(25000003U);
  assert(unconfirmed.phase()==BootstrapPhase::kFault);
  assert(std::string(unconfirmed.error())=="LEASE_RENEWAL_UNCONFIRMED");
  Client malformed_grant_client;
  malformed_grant_client.prepared_success=true;
  BootstrapCoordinator malformed_grant(malformed_grant_client,identity,compiled_policy,1000000U,30000000U);
  auto malformed_state=state;
  malformed_state.session_id=0U;
  malformed_state.reference_epoch=0U;
  malformed_grant.observe(malformed_state,0U);
  assert(malformed_grant.start_operation(malformed_state,ManageOperation::kGrantSession,1U));
  malformed_grant.tick(2U);
  assert(malformed_grant.phase()==BootstrapPhase::kFault);
  assert(std::string(malformed_grant.error())=="LEASE_RENEWAL_UNCONFIRMED");
  Client interrupted_client;
  BootstrapCoordinator interrupted(interrupted_client, identity, compiled_policy, 1000000U);
  assert(interrupted.start(state, 50U));
  interrupted.invalidate("OBSERVATION_UNAVAILABLE");
  assert(interrupted.phase()==BootstrapPhase::kFault);
  const auto interrupted_requests=interrupted_client.requests.size();
  interrupted.tick(51U);
  assert(interrupted_client.requests.size()==interrupted_requests);
  Client rejecting_client;
  rejecting_client.reject=true;
  BootstrapCoordinator rejecting(rejecting_client,identity,compiled_policy,1000000U);
  assert(rejecting.start(state,52U));
  rejecting.tick(53U);
  assert(rejecting.phase()==BootstrapPhase::kFault);
  assert(std::string(rejecting.error())=="BOOTSTRAP_REJECTED phase=1 result=6 reason=7");
  Client release_client;
  BootstrapCoordinator release(release_client,identity,compiled_policy,1000000U);
  auto holding=explicit_state;
  holding.core_state=wire_value(WireCoreState::kHold);
  holding.state_core_state=wire_value(WireCoreState::kHold);
  auto moving=holding;
  moving.state_core_state=wire_value(WireCoreState::kRunning);
  release.observe(moving,98U);
  assert(!release.start_release(moving,99U));
  assert(release_client.requests.empty());
  release.observe(holding,99U);
  assert(release.start_release(holding,100U));
  assert(release_client.requests.back().operation()==wire_value(ManageOperation::kDisableAndRelease));
  assert(release_client.requests.back().expected_session_id()==42U);
  release.tick(101U);
  assert(!release.released());
  assert(release.phase()==BootstrapPhase::kFault);
  assert(!release.confirmed_release());
  Client confirmed_client;
  confirmed_client.release_committed=true;
  confirmed_client.operation_id=91U;
  BootstrapCoordinator confirmed(confirmed_client,identity,compiled_policy,1000000U);
  confirmed.observe(holding,99U);
  assert(confirmed.start_release(holding,100U));
  assert(!confirmed.confirmed_release());
  confirmed.tick(101U);
  assert(confirmed.confirmed_release());
  assert(confirmed.confirmed_release()->request_id==1U);
  assert(confirmed.confirmed_release()->operation_id==91U);
  assert(!confirmed.released());
  auto partial=holding;
  partial.session_id=0U; partial.reference_epoch=0U;
  partial.core_state=1U;
  partial.real_motion_permitted=false; partial.permit_remaining_us=0U;
  confirmed.observe(partial,102U);
  assert(!confirmed.released());
  auto idle=holding;
  idle.session_id=0U; idle.reference_epoch=0U;
  idle.core_state=1U;
  idle.state_core_state=1U;
  idle.real_motion_permitted=false; idle.permit_remaining_us=0U;
  auto referenced_idle=idle;
  referenced_idle.core_state=wire_value(WireCoreState::kIdle);
  referenced_idle.state_core_state=wire_value(WireCoreState::kIdle);
  confirmed.observe(referenced_idle,103U);
  assert(!confirmed.released());
  confirmed.observe(idle,103U);
  assert(confirmed.released());
  auto reoccupied=idle;
  reoccupied.session_id=42U;
  reoccupied.core_state=wire_value(WireCoreState::kHold);
  confirmed.observe(reoccupied,104U);
  assert(confirmed.phase()==BootstrapPhase::kFault);
  assert(!confirmed.confirmed_release());
  Client queried_client;
  queried_client.pending=true;
  queried_client.release_committed=true;
  queried_client.operation_id=92U;
  BootstrapCoordinator queried(queried_client,identity,compiled_policy,1000000U);
  assert(queried.start_release(holding,100U));
  queried.tick(101U);
  assert(!queried.confirmed_release());
  queried_client.pending=false;
  queried.tick(102U);
  assert(queried.confirmed_release());
  assert(queried.confirmed_release()->request_id==1U);
  assert(queried.confirmed_release()->operation_id==92U);
  Client mismatched_client;
  mismatched_client.release_committed=true;
  mismatched_client.wrong_request_id=true;
  BootstrapCoordinator mismatched(mismatched_client,identity,compiled_policy,1000000U);
  assert(mismatched.start_release(holding,100U));
  mismatched.tick(101U);
  assert(mismatched.phase()==BootstrapPhase::kFault);
  assert(!mismatched.confirmed_release());
  Client rejected_release_client;
  rejected_release_client.reject=true;
  BootstrapCoordinator rejected_release(rejected_release_client,identity,compiled_policy,1000000U);
  assert(rejected_release.start_release(holding,100U));
  rejected_release.tick(101U);
  assert(rejected_release.phase()==BootstrapPhase::kFault);
  assert(!rejected_release.confirmed_release());
  Client zero_operation_client;
  zero_operation_client.release_committed=true;
  BootstrapCoordinator zero_operation(zero_operation_client,identity,compiled_policy,1000000U);
  assert(zero_operation.start_release(holding,100U));
  zero_operation_client.operation_id=0U;
  zero_operation.tick(101U);
  assert(zero_operation.phase()==BootstrapPhase::kFault);
  assert(!zero_operation.confirmed_release());
  Client late_client;
  late_client.release_committed=true;
  BootstrapCoordinator late(late_client,identity,compiled_policy,1000000U);
  late.observe(holding,99U);
  assert(late.start_release(holding,100U));
  late.tick(101U);
  assert(!late.released());
  late.observe(idle,5000101U);
  assert(late.phase()==BootstrapPhase::kFault);
  assert(!late.confirmed_release());
  assert(std::string(late.error())=="RELEASE_UNCONFIRMED");
  Client delayed_client;
  delayed_client.pending=true;
  BootstrapCoordinator delayed(delayed_client,identity,compiled_policy,1000000U);
  delayed.observe(holding,99U);
  assert(delayed.start_release(holding,100U));
  delayed.tick(5100100U);
  assert(delayed.phase()==BootstrapPhase::kFault);
  assert(!delayed.released());
  Client changed_client;
  BootstrapCoordinator changed(changed_client,identity,compiled_policy,1000000U);
  changed.observe(holding,99U);
  assert(changed.start_release(holding,100U));
  auto wrong_boot=holding;
  ++wrong_boot.boot_epoch;
  changed.observe(wrong_boot,101U);
  assert(changed.phase()==BootstrapPhase::kFault);
  return 0;
}
