/**
 * @file device_lease_transition_tests.cpp
 * @brief 直接复用device_action_gate实现的租约、释放和准入helper测试。
 *
 * DUMMY_ACTION_GATE_TEST去掉产品main；include生产cpp让本编译单元检查真实helper，不代表启动该ROS节点。
 * 本文件只构造内存对象/人工快照，未发设备请求。有限续租资格、重试拒绝凭据与实际许可严格分开。
 */
#define DUMMY_ACTION_GATE_TEST
#include "../src/device_action_gate.cpp"
#include "action_test_support.hpp"

#include <cassert>

/**
 * @brief 仅记Enable请求且始终无回执的管理端。
 */
struct EnableClient final : dummy_motion_client::ManagementClient {
  dummy_motion_client::ManageRequest submitted{};
  /**
   * @brief 保存最近请求，固定逻辑ID1。
   * @param request 待核Enable身份。
   * @par 未命名参数
   * Hostµs忽略。
   * @return 1，非设备接受确认。
   */
  std::uint64_t submit(const dummy_motion_client::ManageRequest &request,std::uint64_t) override {
    submitted=request;
    return 1U;
  }
  /**
   * @brief 始终无回执。
   * @par 未命名参数
   * 请求ID忽略。
   * @return 空optional。
   */
  std::optional<dummy_motion_client::ManageReply> poll(std::uint64_t) override { return {}; }
};

/**
 * @brief 终点Hold仅在完整ReleaseAvailability下可排队DisableAndRelease。
 * @return 无；本用例的check/assert失败即终止或抛出。
 * status新鲜/management相干、FUNCTIONAL依据、enabled、permit-refresh、bootstrap及双Hold/transport逐项置false均拒绝；pending/busy/releasing也拒绝。内部reject通过成员指针翻转一个前提，不发送管理请求。
 */
void release_after_completed_goal() {
  using dummy_motion_client::ReleaseAvailability;
  using dummy_motion_client::release_operation_available;
  using dummy_motion_client::queue_disable_and_release;
  ReleaseAvailability state{};
  state.status_fresh=true;
  state.management_epoch_matches=true;
  state.functional_development_basis=true;
  state.enabled=true;
  state.permit_refresh_allowed=true;
  state.bootstrap_ready=true;
  state.status_hold=true;
  state.state_hold=true;
  state.transport_ready=true;
  assert(release_operation_available(state));
  std::atomic<bool> available{release_operation_available(state)}, busy{false};
  std::atomic<int> request{0}, stage{0};
  assert(queue_disable_and_release(available,request,busy,stage));
  assert(busy.load() && stage.load()==1);
  assert(request.load()==static_cast<int>(dummy_motion_client::wire_value(
    dummy_motion_client::ManageOperation::kDisableAndRelease)));

  const auto reject=[&state](auto member) {
    auto invalid=state;
    invalid.*member=false;
    assert(!release_operation_available(invalid));
  };
  reject(&ReleaseAvailability::status_fresh);
  reject(&ReleaseAvailability::management_epoch_matches);
  reject(&ReleaseAvailability::functional_development_basis);
  reject(&ReleaseAvailability::enabled);
  reject(&ReleaseAvailability::bootstrap_ready);
  reject(&ReleaseAvailability::status_hold);
  reject(&ReleaseAvailability::state_hold);
  reject(&ReleaseAvailability::transport_ready);
  auto blocked=state;
  blocked.releasing=true;
  assert(!release_operation_available(blocked));
  blocked=state; blocked.busy=true;
  assert(!release_operation_available(blocked));
  blocked=state; blocked.management_request_pending=true;
  assert(!release_operation_available(blocked));
  blocked=state; blocked.permit_refresh_allowed=false;
  assert(!release_operation_available(blocked));
  blocked.gate_ready=true;
  assert(release_operation_available(blocked));
}

/**
 * @brief 只把wire UnreferencedIdle1、session/ref清零视为释放。
 * @return 无；本用例的check/assert失败即终止或抛出。
 * ReferencedIdle2或ref残留都不够，且仍需匹配boot/motion/management。
 */
void unreferenced_idle_is_the_only_release_observation() {
  using namespace dummy_motion_client;
  wire::DummyDeviceStatusV2 status;
  wire::DummyJointStateV2 state;
  status.set_boot_epoch(7U);
  state.set_boot_epoch(7U);
  status.set_motion_epoch(4U);
  state.set_motion_epoch(4U);
  status.set_management_epoch(8U);
  state.set_management_epoch(8U);
  status.set_release_committed(true);
  status.set_core_state(1U);
  state.set_core_state(1U);
  assert(released_idle_observation(status,state));
  status.set_core_state(wire_value(WireCoreState::kIdle));
  assert(!released_idle_observation(status,state));
  status.set_core_state(1U);
  state.set_reference_epoch(1U);
  assert(!released_idle_observation(status,state));
}

/**
 * @brief 释放Idle还需bootstrap同boot、session与epoch相干。
 * @return 无；本用例的check/assert失败即终止或抛出。
 * 测试使用64位boot；不能只凭某一流Idle覆盖另一流旧授权。
 */
void released_idle_requires_coherent_bootstrap_authority() {
  using namespace dummy_motion_client;
  wire::DummyDeviceStatusV2 status;
  wire::DummyJointStateV2 state;
  status.set_boot_epoch(14003434958659749721ULL);
  state.set_boot_epoch(14003434958659749721ULL);
  status.set_core_state(wire_value(WireCoreState::kUnreferencedIdle));
  state.set_core_state(wire_value(WireCoreState::kUnreferencedIdle));
  status.set_motion_epoch(0U);
  status.set_management_epoch(0U);
  state.set_motion_epoch(4U);
  state.set_management_epoch(8U);
  assert(!bootstrap_authority_coherent(status,state));
  status.set_motion_epoch(4U);
  assert(!bootstrap_authority_coherent(status,state));
  status.set_management_epoch(8U);
  assert(bootstrap_authority_coherent(status,state));
  status.set_session_id(1U);
  assert(!bootstrap_authority_coherent(status,state));
}

/**
 * @brief Enable只在boot/session/reference吻合时将授权motion注入GuardedPort。
 * @return 无；本用例的check/assert失败即终止或抛出。
 * 当前motion或允许的下一motion可核，跨boot/session/ref或跳两代拒绝；没有真实Enable动作。
 */
void same_boot_enable_seeds_pending_port_authority() {
  using namespace dummy_motion_client;
  constexpr std::uint64_t boot=9541039847285119850ULL;
  constexpr std::uint64_t session=1052027048572961417ULL;
  EnableClient client;
  GuardedPort port;
  port.identity.boot=boot;
  port.identity.session=session;
  port.identity.reference_epoch=2U;
  JointProfile profile{{"Joint1"},{-1.0},{1.0},{1.0},{2.0},{0.1},{0.1}};
  ActionGate gate(client,port,profile);
  Authorization authorization;
  authorization.session=authorization.reference=authorization.permit=true;
  authorization.identity.set_expected_boot_epoch(boot);
  authorization.identity.set_expected_session_id(session);
  authorization.identity.set_expected_reference_epoch(2U);
  authorization.identity.set_expected_motion_epoch(4U);
  authorization.identity.set_expected_management_epoch(12U);
  const StateAuthority observed{boot,session,2U,4U};
  const auto expected=[&port] {
    return StateAuthority{port.identity.boot,port.identity.session,
                          port.identity.reference_epoch,port.identity.motion_epoch};
  };
  assert(!state_authority_is_valid(observed,expected(),false,false));
  auto wrong=authorization;
  wrong.identity.set_expected_boot_epoch(boot+1U);
  assert(!enable_gate_with_port_authority(gate,port,wrong,1U));
  wrong=authorization;
  wrong.identity.set_expected_session_id(session+1U);
  assert(!enable_gate_with_port_authority(gate,port,wrong,1U));
  wrong=authorization;
  wrong.identity.set_expected_reference_epoch(3U);
  assert(!enable_gate_with_port_authority(gate,port,wrong,1U));
  assert(port.identity.motion_epoch==0U);
  assert(enable_gate_with_port_authority(gate,port,authorization,1U));
  assert(client.submitted.expected_motion_epoch()==4U);
  assert(port.identity.motion_epoch==4U);
  assert(state_authority_is_valid(observed,expected(),false,false));
  assert(state_authority_is_valid({boot,session,2U,5U},expected(),false,false));
  assert(!state_authority_is_valid({boot,session,2U,6U},expected(),false,false));
  assert(!state_authority_is_valid({boot,session+1U,2U,4U},expected(),false,false));
}

/**
 * @brief 只为实际RENEWAL_PENDING保存有界重试拒绝凭据。
 * @return 无；本用例的check/assert失败即终止或抛出。
 * UUID匹配且年龄≤2s才可读，超过1ns即失效；此凭据不恢复授权，也不自动重放目标。内部deny逐项翻转六个准入前提。
 */
void goal_rejection_receipt_is_typed_and_bounded() {
  using namespace dummy_motion_client;
  using Clock=std::chrono::steady_clock;
  GoalRejectionReceipt receipt;
  const Clock::time_point start{};
  rclcpp_action::GoalUUID first{};
  for(std::size_t i=0;i<first.size();++i) first[i]=static_cast<std::uint8_t>(i*16U+i);
  const std::string expected="RENEWAL_PENDING:00112233445566778899aabbccddeeff";
  AdmissionPrerequisites prerequisites{true,true,true,true,true,true};
  assert(prerequisites.ready());
  assert(prerequisites.diagnostic_bits() == 0x3fU);
  // run10真实时序：Hold确认后18.615ms续租预约，随后3.148ms目标被拒；六项前置始终完备。
  std::atomic<int> management{0};
  std::atomic<bool> busy{false}, reserved{false};
  const auto hold_confirmed =
      start + std::chrono::microseconds(235989521468ULL);
  const auto renew_sent = hold_confirmed + std::chrono::microseconds(18615U);
  const auto rejected = renew_sent + std::chrono::microseconds(3148U);
  assert(queue_lease_renewal(management, busy, reserved, false));
  assert(prerequisites.ready() && reserved.load());
  receipt.record(first, GoalRejectionReason::kNotReady, reserved.load(),
                 prerequisites.ready(), 0, false, false, rejected);
  assert(receipt.message(rejected) == expected);
  auto missing_begin = prerequisites;
  missing_begin.transport_begin_ready = false;
  assert(missing_begin.diagnostic_bits() == 0x1fU);
  receipt.record(first, GoalRejectionReason::kNotReady, true,
                 missing_begin.ready(), 0, false, false, rejected);
  assert(!receipt.message(rejected));
  const auto deny=[&prerequisites](auto member) {
    auto missing=prerequisites;
    missing.*member=false;
    assert(!missing.ready());
  };
  deny(&AdmissionPrerequisites::enabled);
  deny(&AdmissionPrerequisites::observation_ready);
  deny(&AdmissionPrerequisites::gate_ready);
  deny(&AdmissionPrerequisites::port_ready);
  deny(&AdmissionPrerequisites::hold_tolerance_bound);
  deny(&AdmissionPrerequisites::transport_begin_ready);

  assert(!receipt.message(start));
  receipt.record(first,GoalRejectionReason::kNotReady,true,true,0,false,false,start);
  assert(receipt.message(start)==expected);
  assert(receipt.message(start+std::chrono::seconds(2))==expected);
  assert(!receipt.message(start+std::chrono::seconds(2)+std::chrono::nanoseconds(1)));

  receipt.record(first,GoalRejectionReason::kNotReady,true,true,0,false,false,start);
  auto second=first;
  second[0]=0xab;
  receipt.record(second,GoalRejectionReason::kNotReady,true,true,0,false,false,start);
  assert(receipt.message(start)=="RENEWAL_PENDING:ab112233445566778899aabbccddeeff");
  receipt.record(first,GoalRejectionReason::kNotReady,true,false,0,false,false,start);
  assert(!receipt.message(start));
  receipt.record(first,GoalRejectionReason::kInvalid,true,true,0,false,false,start);
  assert(!receipt.message(start));

  receipt.record(first,GoalRejectionReason::kBusy,true,true,0,false,false,start);
  assert(!receipt.message(start));
  receipt.record(first,GoalRejectionReason::kNotReady,false,true,0,false,false,start);
  assert(!receipt.message(start));
  receipt.record(first,GoalRejectionReason::kNotReady,true,true,1,false,false,start);
  assert(!receipt.message(start));
  receipt.record(first,GoalRejectionReason::kNotReady,true,true,0,true,false,start);
  assert(!receipt.message(start));
  receipt.record(first,GoalRejectionReason::kNotReady,true,true,0,false,true,start);
  assert(!receipt.message(start));
}

/**
 * @brief Idle与RUN失权均阻断新目标，晚到正回执不能重启。
 * @return 无；本用例的check/assert失败即终止或抛出。
 * 核Port start计数不变，失权不是缓存目标等待自动补发的理由。
 */
void authorization_loss_blocks_admission_without_replaying_a_goal() {
  using namespace dummy_motion_client;
  using action_test::require;
  const Trajectory motion{{"Joint1"},{{{0.2},{0},{},1}},{},{},0};
  action_test::Harness idle;
  idle.enable();
  require(idle.gate.ready());
  require(idle.port.starts==0U);
  idle.gate.invalidate_authorization(2053000U);
  require(idle.gate.phase()==Phase::kFault && !idle.gate.ready());
  require(!idle.gate.submit(motion,2053001U));
  require(idle.port.starts==0U);

  action_test::Harness active;
  active.enable();
  active.begin();
  const auto starts=active.port.starts;
  require(active.gate.phase()==Phase::kRunning && starts>0U);
  active.gate.invalidate_authorization(2053000U);
  require(active.gate.phase()==Phase::kFault && active.gate.terminal()==Terminal::kAborted);
  active.gate.observe(active.response(true),2053001U);
  require(!active.gate.ready() && !active.gate.submit(motion,2053002U));
  require(active.port.starts==starts);
}

/**
 * @brief 核租约范围、七项RUN续租资格及显式状态回执消费。
 * @return 0表示所选断言通过；非零、assert或检查异常表示失败。
 * requested permit须正且不超过已验证profile最大值（例60s/650s），不是允许任意申请长租约。
 * RUN续租只占自己busy；失败清理不能清掉目标占用。显式LEASE_RENEWED只在操作完成且Hold时消费一次；自动续租不伪装显式用户确认。
 */
void manual_permit_restore_requires_closed_previous_goal() {
  using namespace dummy_motion_client;
  ManualPermitRestoreInputs input{true, true, true, true, true,
                                  true, true, true, 12U,  0U};
  assert(input.ready());
  auto deny = [&](auto member) {
    auto bad = input;
    bad.*member = false;
    assert(!bad.ready());
  };
  deny(&ManualPermitRestoreInputs::task_ready);
  deny(&ManualPermitRestoreInputs::enabled);
  deny(&ManualPermitRestoreInputs::no_goal);
  deny(&ManualPermitRestoreInputs::management_idle);
  deny(&ManualPermitRestoreInputs::fresh_bound_hold);
  deny(&ManualPermitRestoreInputs::permit_consumed);
  deny(&ManualPermitRestoreInputs::previous_goal_sealed);
  deny(&ManualPermitRestoreInputs::remaining_budget);
  input.last_restored_epoch = 12U;
  assert(!input.ready());
  input.last_restored_epoch = 0U;
  input.motion_epoch = 0U;
  assert(!input.ready());
}

int main() {
  manual_permit_restore_requires_closed_previous_goal();
  {
    using namespace dummy_motion_client;
    struct RaceClient final : ManagementClient {
      std::vector<ManageRequest> requests;
      bool committed = false;
      std::uint64_t submit(const ManageRequest &request,
                           std::uint64_t) override {
        requests.push_back(request);
        return requests.size();
      }
      std::optional<ManageReply> poll(std::uint64_t id) override {
        if (!committed)
          return {};
        ManageReply reply;
        reply.request_id = id;
        reply.operation_id = id;
        reply.transaction_phase = 3;
        reply.result_valid = true;
        reply.succeeded = true;
        reply.release_committed = requests.back().operation() == 10;
        return reply;
      }
    } client;
    ManageRequest identity;
    identity.set_expected_session_id(7);
    std::array<std::uint8_t, 32> digest{};
    digest[0] = 1;
    BootstrapCoordinator bootstrap(client, identity, digest, 1000000);
    BootstrapSnapshot snapshot;
    snapshot.boot_epoch = 1;
    snapshot.session_id = 7;
    snapshot.reference_epoch = 1;
    snapshot.core_state = 2;
    snapshot.state_core_state = 2;
    snapshot.profile_id = 1;
    snapshot.configured_mask = 63;
    snapshot.raw_ready = true;
    snapshot.motion_supported = true;
    snapshot.freshness_ticket = 1;
    snapshot.management_epoch = 1;
    assert(bootstrap.start_operation(
        snapshot, ManageOperation::kBeginQualification, 100, false, true));
    ManualReleaseIntent intent;
    assert(intent.begin(snapshot, 101));
    assert(bootstrap.management_operation_pending());
    assert(intent.decision(snapshot, 102, bootstrap.release_slot_available(),
                           true, false) == ManualReleaseDecision::kWait);
    assert(client.requests.size() == 1); // 17未终结时不能抢D10槽。
    client.committed = true;
    bootstrap.tick(103);
    assert(intent.decision(snapshot, 104, bootstrap.release_slot_available(),
                           true, false) == ManualReleaseDecision::kWait);
    snapshot.management_epoch = 2;
    snapshot.motion_epoch = 2;
    bootstrap.observe(snapshot, 105);
    bootstrap.tick(105);
    assert(intent.decision(snapshot, 106, bootstrap.release_slot_available(),
                           true, false) == ManualReleaseDecision::kSubmit);
    assert(bootstrap.start_release(snapshot, 106, true, 1));
    assert(client.requests.size() == 2 &&
           client.requests.back().operation() == 10);
    bootstrap.tick(107);
    snapshot.session_id = 0;
    bootstrap.observe(snapshot, 108);
    assert(bootstrap.released());
  }
  {
    using namespace dummy_motion_client;
    ManualReleaseIntent intent;
    BootstrapSnapshot original;
    original.boot_epoch = 1;
    original.session_id = 7;
    original.reference_epoch = 1;
    intent.begin(original, 100);
    // 逐拍：17仍在途，释放保存意图；终态但新鲜State未到也不能提交。
    assert(intent.decision(original, 101, false, true, false) ==
           ManualReleaseDecision::kWait);
    assert(intent.decision(original, 102, true, false, false) ==
           ManualReleaseDecision::kWait);
    auto advanced = original;
    advanced.motion_epoch = 2;
    advanced.management_epoch = 3;
    assert(intent.decision(advanced, 103, true, true, false) ==
           ManualReleaseDecision::kSubmit);
    assert(intent.decision(advanced, 5000100, true, true, false) ==
           ManualReleaseDecision::kUnconfirmed);
    intent.begin(original, 100);
    assert(intent.decision(advanced, 104, false, true, true) ==
           ManualReleaseDecision::kUnconfirmed);
    auto foreign = advanced;
    foreign.session_id = 8;
    assert(intent.decision(foreign, 104, true, true, false) ==
           ManualReleaseDecision::kUnconfirmed);
  }
  {
    dummy_motion_client::wire::DummyDeviceStatusV2 status;
    dummy_motion_client::wire::DummyJointStateV2 state;
    status.set_boot_epoch(1);
    state.set_boot_epoch(1);
    status.set_core_state(1);
    state.set_core_state(1);
    assert(
        dummy_motion_client::readonly_released_idle_observation(status, state));
    assert(
        dummy_motion_client::readonly_release_samples_fresh(100, 101, 102, 10));
    assert(!dummy_motion_client::readonly_release_samples_fresh(100, 101, 120,
                                                                10));
    assert(
        !dummy_motion_client::readonly_release_samples_fresh(0, 101, 102, 10));
    // Status按共享1s规则，State仍严格250ms；500ms资格状态周期不能造成释放误拒。
    for (const auto status_age : {500000ULL, 520000ULL, 540000ULL, 1000000ULL})
      assert(dummy_motion_client::readonly_release_samples_fresh(
          2000000 - status_age, 1990000, 2000000, 250000));
    assert(!dummy_motion_client::readonly_release_samples_fresh(
        999999, 1990000, 2000000, 250000));
    assert(!dummy_motion_client::readonly_release_samples_fresh(
        1500000, 1749999, 2000000, 250000));
    assert(!dummy_motion_client::readonly_release_samples_fresh(
        2000001, 1990000, 2000000, 250000));
    assert(!dummy_motion_client::readonly_release_samples_fresh(
        1500000, 2000001, 2000000, 250000));
    status.set_session_id(3);
    assert(!dummy_motion_client::readonly_released_idle_observation(status,
                                                                    state));
    status.set_session_id(0);
    status.set_reference_epoch(2);
    state.set_reference_epoch(2);
    status.set_core_state(2);
    state.set_core_state(2);
    assert(
        dummy_motion_client::readonly_released_idle_observation(status, state));
    status.set_reference_kind(1);
    status.set_release_committed(true);
    state.set_position_valid_mask(63);
    assert(
        dummy_motion_client::retained_manual_reference_reusable(status, state));
    status.set_release_committed(false);
    assert(!dummy_motion_client::retained_manual_reference_reusable(status,
                                                                    state));
    status.set_release_committed(true);
    status.set_real_motion_permitted(true);
    assert(!dummy_motion_client::retained_manual_reference_reusable(status,
                                                                    state));
    status.set_real_motion_permitted(false);
    state.set_position_valid_mask(0);
    assert(!dummy_motion_client::retained_manual_reference_reusable(status,
                                                                    state));
    state.set_position_valid_mask(63);
    state.set_reference_epoch(3);
    assert(!dummy_motion_client::retained_manual_reference_reusable(status,
                                                                    state));
    assert(!dummy_motion_client::readonly_released_idle_observation(status,
                                                                    state));
  }
  {
    dummy_motion_client::ReleaseAvailability manual;
    manual.manual_mode = true;
    manual.manual_session_owned = true;
    manual.manual_quiet = true;
    manual.status_fresh = true;
    manual.management_epoch_matches = true;
    manual.functional_development_basis = true;
    manual.bootstrap_ready = true;
    manual.transport_ready = true;
    assert(dummy_motion_client::release_operation_available(manual));
    manual.management_request_pending = true;
    assert(!dummy_motion_client::release_operation_available(manual));
    manual.management_request_pending = false;
    manual.manual_session_owned = false;
    assert(!dummy_motion_client::release_operation_available(manual));
    assert(dummy_motion_client::manual_coordinator_tick_required(
        true, dummy_motion_client::ManualReferencePhase::kCaptureAbsolute));
    assert(dummy_motion_client::manual_coordinator_tick_required(
        true, dummy_motion_client::ManualReferencePhase::kRestQualification));
    assert(!dummy_motion_client::manual_coordinator_tick_required(
        false, dummy_motion_client::ManualReferencePhase::kReadOnly));
  }

  using namespace dummy_motion_client;
  DeviceProfile short_candidate{};
  short_candidate.motion_permit_max_us=60000000U;
  DeviceProfile long_candidate{};
  long_candidate.motion_permit_max_us=650000000U;
  assert(validated_requested_motion_permit(60000000,short_candidate));
  assert(!validated_requested_motion_permit(650000000,short_candidate));
  assert(validated_requested_motion_permit(650000000,long_candidate));
  assert(!validated_requested_motion_permit(650000001,long_candidate));
  assert(!validated_requested_motion_permit(0,long_candidate));
  RunLeaseEligibility run{};
  run.status_running=true;
  run.state_running=true;
  run.status_state_coherent=true;
  run.same_goal_run=true;
  run.producer_active=true;
  run.producer_recent=true;
  run.transport_ready=true;
  assert(run_lease_eligible(run));
  auto interrupted_run=run;
  interrupted_run.producer_active=false;
  assert(!run_lease_eligible(interrupted_run));
  interrupted_run=run;
  interrupted_run.status_state_coherent=false;
  assert(!run_lease_eligible(interrupted_run));
  interrupted_run=run;
  interrupted_run.producer_recent=false;
  assert(!run_lease_eligible(interrupted_run));
  std::atomic<int> run_request{0};
  std::atomic<bool> goal_busy{true}, run_reserved{false};
  assert(queue_lease_renewal(run_request,goal_busy,run_reserved,true));
  assert(goal_busy.load() && run_reserved.load());
  assert(!queue_lease_renewal(run_request,goal_busy,run_reserved,true));
  assert(goal_busy.load());
  std::atomic<bool> run_started{false};
  std::atomic<int> run_explicit_status{0};
  clear_failed_lease_renewal(run_request,goal_busy,run_reserved,
    run_started,run_explicit_status,false);
  assert(goal_busy.load());
  goal_rejection_receipt_is_typed_and_bounded();
  authorization_loss_blocks_admission_without_replaying_a_goal();
  release_after_completed_goal();
  unreferenced_idle_is_the_only_release_observation();
  released_idle_requires_coherent_bootstrap_authority();
  same_boot_enable_seeds_pending_port_authority();
  const int renew=static_cast<int>(wire_value(ManageOperation::kRenewLease));
  const int permit=static_cast<int>(wire_value(ManageOperation::kSetPermit));

  std::atomic<int> request{renew};
  std::atomic<bool> busy{true}, reserved{true}, started{false};
  std::atomic<int> explicit_renewal_status{1};
  clear_failed_lease_renewal(request,busy,reserved,started,explicit_renewal_status);
  assert(request.load()==0);
  assert(!busy.load() && !reserved.load() && !started.load());
  assert(explicit_renewal_status.load()==0);
  assert(std::string(bootstrap_status_message(BootstrapPhase::kFault,
    "OBSERVATION_UNAVAILABLE",false,true,true,true))=="OBSERVATION_UNAVAILABLE");

  // A permit callback may observe availability from the previous cycle after
  // renewal was dequeued. It must not acknowledge an operation that cannot run.
  busy=true; reserved=true; request=0;
  assert(!queue_management_request(request,busy,reserved,permit,true));
  assert(request.load()==0);
  busy=false; reserved=false;
  assert(queue_management_request(request,busy,reserved,permit,true));
  assert(request.load()==permit);

  // Automatic renewal leaves a usable Hold reported as ENABLED_HOLD. An
  // explicitly queued renewal gets one correlated completion report.
  explicit_renewal_status=0;
  bool confirmed=take_confirmed_lease_renewal(explicit_renewal_status,true,true,true,ManageOperation::kRenewLease);
  assert(!confirmed);
  assert(std::string(bootstrap_status_message(BootstrapPhase::kOperationComplete,
    "",confirmed,true,true,true))=="ENABLED_HOLD");
  explicit_renewal_status=2;
  confirmed=take_confirmed_lease_renewal(explicit_renewal_status,false,true,true,ManageOperation::kRenewLease);
  assert(!confirmed && explicit_renewal_status.load()==2);
  confirmed=take_confirmed_lease_renewal(explicit_renewal_status,true,true,true,ManageOperation::kRenewLease);
  assert(confirmed);
  explicit_renewal_status=2;
  confirmed=take_confirmed_lease_renewal(explicit_renewal_status,true,true,true,ManageOperation::kRenewLease);
  assert(confirmed && explicit_renewal_status.load()==0);
  explicit_renewal_status=2;
  confirmed=take_confirmed_lease_renewal(explicit_renewal_status,true,true,false,ManageOperation::kRenewLease);
  assert(!confirmed && explicit_renewal_status.load()==2);
  confirmed=take_confirmed_lease_renewal(explicit_renewal_status,true,true,true,ManageOperation::kRenewLease);
  assert(confirmed);
  assert(std::string(bootstrap_status_message(BootstrapPhase::kOperationComplete,
    "",confirmed,true,true,true))=="LEASE_RENEWED");
  confirmed=take_confirmed_lease_renewal(explicit_renewal_status,true,true,true,ManageOperation::kRenewLease);
  assert(!confirmed);
  assert(std::string(bootstrap_status_message(BootstrapPhase::kOperationComplete,
    "",confirmed,true,true,true))=="ENABLED_HOLD");
  assert(std::string(bootstrap_status_message(BootstrapPhase::kOperationComplete,
    "",confirmed,true,false,true))=="OPERATION_COMPLETE");
}
