/**
 * @file direct_hold_handoff_tests.cpp
 * @brief 证据闭合helper与direct Hold/JTC命令生产权交接测试。
 *
 * 前半为人工身份/时间/证据值，后半会初始化rclcpp并激活内存控制器，但不创建USB/CAN设备链。
 * 直接include生产device_action_gate.cpp以核真实helper；继任页、旧终态raw证明、管理强回执及双流快照各有独立职责，不能互相替代。
 */
#define DUMMY_ACTION_GATE_TEST
#include "../src/device_action_gate.cpp"

#include <iostream>
#include <limits>

/**
 * @brief 逐项负例核Cancel/Complete证据继任，再核direct保持与JTC交接。
 * @param argc 进程参数数量。
 * @param argv 模式/输入路径见下述说明。
 * @return 0表示所选断言通过；非零、assert或检查异常表示失败。
 * argc/argv只交给ROS初始化函数；返回值对应失败检查位置。位置rad，Host微秒时间用于500ms刷新预算及RUN租约年龄。
 * 最终controller内存更新证明命令生产权和数值保持，不证明电机已到位或安全停止。
 */
int main(int argc,char **argv) {
  if(dummy_motion_client::evidence_incomplete_result_error("STOP_FAILED")!=
       "STOP_FAILED;EVIDENCE_INCOMPLETE" ||
     dummy_motion_client::evidence_incomplete_result_error("")!=
       "EVIDENCE_INCOMPLETE" ||
     dummy_motion_client::evidence_incomplete_result_error("EVIDENCE_INCOMPLETE")!=
       "EVIDENCE_INCOMPLETE") return 73;
  /**
   * @par 场景与断言
   * 取消信箱容量2，按shared_ptr对象身份匹配；third_goal虽值同1也不能冒充first_goal，取出后不可重复消费。
   */
  dummy_motion_client::PendingGoalCancels<int,2U> cancel_mailbox;
  auto first_goal=std::make_shared<int>(1);
  auto second_goal=std::make_shared<int>(2);
  if(!cancel_mailbox.request(first_goal)) return 54;
  if(cancel_mailbox.take_for({})) return 55;
  if(cancel_mailbox.take_for(second_goal)) return 56;
  if(!cancel_mailbox.request(second_goal) || !cancel_mailbox.take_for(second_goal)) return 57;
  if(cancel_mailbox.take_for(second_goal)) return 58;
  auto third_goal=std::make_shared<int>(1);
  if(!cancel_mailbox.request(first_goal) || !cancel_mailbox.request(second_goal)) return 59;
  if(cancel_mailbox.request(third_goal)) return 60;
  if(cancel_mailbox.take_for(third_goal)) return 61;
  /**
   * @par 场景与断言
   * 控制锁打开目标后才接受对应取消；关闭/换目标后旧对象不能借窗口取消新目标。
   */
  dummy_motion_client::GoalCancelAdmission<int> admission;
  if(admission.request(first_goal,cancel_mailbox)) return 62;
  {
    auto control=admission.try_control();
    if(!control) return 69;
    control.open(first_goal);
  }
  if(admission.request(second_goal,cancel_mailbox)) return 63;
  {
    auto control=admission.try_control();
    if(!control) return 70;
    control.close();
  }
  if(admission.request(first_goal,cancel_mailbox)) return 64;
  {
    auto control=admission.try_control();
    if(!control) return 71;
    control.open(second_goal);
  }
  if(admission.request(first_goal,cancel_mailbox)) return 65;
  if(!admission.request(second_goal,cancel_mailbox)) return 66;
  {
    auto control=admission.try_control();
    if(!control) return 72;
    control.close();
  }
  if(!cancel_mailbox.take_for(second_goal)) return 67;
  if(cancel_mailbox.take_for(first_goal)) return 68;
  using namespace dummy_motion_client;
  /**
   * @par 场景边界
   * 七项RUN续租前提逐项失败；有界完成续接只替代producer_active一项，不省略其它六项。
   */
  RunLeaseEligibility lease_inputs{true,true,true,true,true,true,true};
  if(std::string_view(run_lease_first_failed(lease_inputs))!="none") return 94;
  // Completion defer只可提供producer_active；不替代既有250ms producer freshness。
  auto deferred_completion=lease_inputs;
  deferred_completion.producer_active=true;
  deferred_completion.producer_recent=false;
  if(run_lease_eligible(deferred_completion)) return 97;
  auto frozen_inputs=lease_inputs; frozen_inputs.producer_active=false;
  if(run_lease_eligible(frozen_inputs) ||
     !run_lease_eligible(run_lease_continuation_inputs(frozen_inputs,true,true)) ||
     run_lease_eligible(run_lease_continuation_inputs(frozen_inputs,false,true)) ||
     run_lease_eligible(run_lease_continuation_inputs(frozen_inputs,true,false))) return 98;
  for(unsigned field=0;field<6;++field) {
    auto bad=frozen_inputs;
    switch(field) {
      case 0: bad.status_running=false; break;
      case 1: bad.state_running=false; break;
      case 2: bad.status_state_coherent=false; break;
      case 3: bad.same_goal_run=false; break;
      case 4: bad.producer_recent=false; break;
      case 5: bad.transport_ready=false; break;
    }
    if(run_lease_eligible(run_lease_continuation_inputs(bad,true,true))) return 99;
  }
  const std::array<const char *,7> failures{
    "status_running","state_running","status_state_coherent",
    "same_goal_run","producer_active","producer_recent","transport_ready"};
  for(std::size_t index=0;index<failures.size();++index) {
    auto candidate=lease_inputs;
    switch(index) {
      case 0: candidate.status_running=false; break;
      case 1: candidate.state_running=false; break;
      case 2: candidate.status_state_coherent=false; break;
      case 3: candidate.same_goal_run=false; break;
      case 4: candidate.producer_active=false; break;
      case 5: candidate.producer_recent=false; break;
      case 6: candidate.transport_ready=false; break;
    }
    if(run_lease_eligible(candidate) ||
       std::string_view(run_lease_first_failed(candidate))!=failures[index]) return 95;
  }
  if(run_lease_eligible({false,true,true,true,true,true,true}) ||
     run_lease_age_us(100U,70U)!=30U ||
     run_lease_age_us(100U,0U)!=UINT64_MAX ||
     run_lease_age_us(70U,100U)!=UINT64_MAX) return 96;
  /**
   * @par 场景与断言
   * 身份boot7/session8/ref9/motion17/run8；取消后Hold采用motion18，但仍须同run8的干净final7 raw及安全Hold18证明。
   */
  const TimelineIdentity canceled_run{7U,8U,9U,17U,8U};
  wire::DummyDeviceStatusV2 canceled_status{};
  wire::DummyJointStateV2 canceled_state{};
  canceled_status.set_boot_epoch(7U);
  canceled_status.set_session_id(8U);
  canceled_status.set_reference_epoch(9U);
  canceled_status.set_motion_epoch(18U);
  canceled_status.set_run_id(8U);
  canceled_status.set_attempt_kind(1U);
  canceled_status.set_parent_run_id(0U);
  canceled_status.set_final_state(7U);
  canceled_status.set_motion_complete(false);
  canceled_status.set_core_state(wire_value(WireCoreState::kHold));
  canceled_state.set_boot_epoch(7U);
  canceled_state.set_session_id(8U);
  canceled_state.set_reference_epoch(9U);
  canceled_state.set_motion_epoch(18U);
  canceled_state.set_core_state(wire_value(WireCoreState::kHold));
  canceled_state.set_hold_ready(true);
  canceled_state.set_can_submitted_mask(63U);
  canceled_state.set_can_sent_mask(63U);
  RawTerminalIdentity terminal{7U,8U,8U,0U,1U,7U,0U,0U,
                               false,false,false,true,18U,true};
  /**
   * @par 场景边界
   * 内部canceled_hold回调无实物副作用：将当前Status/State、先前身份及raw终态交给timeline_cancel_hold_confirmed，返回证据是否闭合。
   */
  const auto canceled_hold=[&] {
    return timeline_cancel_hold_confirmed(canceled_status,canceled_state,canceled_run,
                                          terminal,63U);
  };
  if(!canceled_hold()) return 40;
  canceled_status.set_run_id(7U);
  if(canceled_hold()) return 41;
  canceled_status.set_run_id(8U);
  canceled_status.set_final_state(6U);
  canceled_status.set_motion_complete(true);
  if(canceled_hold()) return 51;
  canceled_status.set_final_state(7U);
  canceled_status.set_motion_complete(false);
  canceled_status.set_motion_epoch(17U);
  canceled_state.set_motion_epoch(17U);
  if(canceled_hold()) return 42;
  canceled_status.set_motion_epoch(18U);
  canceled_state.set_motion_epoch(18U);
  canceled_state.set_can_sent_mask(31U);
  if(canceled_hold()) return 43;
  canceled_state.set_can_sent_mask(63U);
  terminal.run_id=7U;
  if(canceled_hold()) return 44;
  terminal.run_id=8U;
  if(!canceled_hold()) return 45;
  terminal.final_state=6U;
  if(canceled_hold()) return 92;
  terminal.final_state=7U;
  terminal.safety_hold_valid=false;
  if(canceled_hold()) return 93;
  terminal.safety_hold_valid=true;
  canceled_status.set_motion_complete(true);
  if(canceled_hold()) return 52;
  canceled_status.set_motion_complete(false);
  canceled_state.set_can_submitted_mask(31U);
  if(canceled_hold()) return 53;
  canceled_state.set_can_submitted_mask(63U);
  if(timeline_terminal_decision(Terminal::kCanceled,true,false,false,true,false,false)
       !=TimelineSuccessDecision::kWait) return 46;
  if(timeline_terminal_decision(Terminal::kCanceled,true,false,true,true,false,false)
       !=TimelineSuccessDecision::kWait) return 47;
  if(timeline_terminal_decision(Terminal::kCanceled,true,false,true,true,false,true)
       !=TimelineSuccessDecision::kSucceed) return 48;
  if(timeline_terminal_decision(Terminal::kAborted,true,false,true,true,false,false)
       !=TimelineSuccessDecision::kWait) return 49;
  if(timeline_terminal_decision(Terminal::kSucceeded,true,false,true,true,false,true)
       !=TimelineSuccessDecision::kSucceed) return 50;
  dummy_motion_client::PermitRefreshLatch permit_latch;
  permit_latch.begin(7U,8U);
  if(!permit_latch.eligible(7U,8U) || permit_latch.eligible(9U,8U) ||
     permit_latch.eligible(7U,9U)) return 31;
  permit_latch.applied();
  if(permit_latch.eligible(7U,8U)) return 32;
  permit_latch.begin(7U,8U);
  if(!permit_latch.eligible(7U,8U)) return 33;
  permit_latch.begin(9U,10U);
  if(permit_latch.eligible(7U,8U) || !permit_latch.eligible(9U,10U)) return 34;
  using dummy_motion_client::GuardedPort;
  using dummy_ros_control::consume_control_update;
  using dummy_ros_control::publish_control_update;
  using dummy_ros_control::set_command_identity;
  /**
   * @par 场景与断言
   * Host时间单位µs；500ms预算边界严格小于，恰好等于截止不再等候。
   */
  constexpr std::uint64_t final_status_us=51909203083ULL;
  constexpr std::uint64_t confirmed_hold_us=51909258659ULL;
  constexpr std::uint64_t refresh_status_us=51909767985ULL;
  constexpr std::uint64_t state_age_budget_us=500000ULL;
  if(confirmed_hold_us-final_status_us>=state_age_budget_us ||
     refresh_status_us-final_status_us<=state_age_budget_us) return 18;
  const dummy_motion_client::TimelineIdentity run8{7U,8U,9U,17U,8U};
  const dummy_motion_client::TimelineIdentity run9{7U,8U,9U,17U,9U};
  dummy_motion_client::RawFirstPageWritten first_page{};
  first_page.boot_epoch=7U;
  first_page.session_id=8U;
  first_page.run_id=9U;
  first_page.motion_epoch=17U;
  first_page.evidence_seq=0U;
  first_page.group_count=1U;
  first_page.attempt_kind=1U;
  first_page.category=1U;
  first_page.expected_mask=first_page.submitted_mask=first_page.sent_mask=63U;
  first_page.final=first_page.chain_valid=first_page.full_six=
      first_page.strict_refresh_successor=true;
  RawTerminalIdentity clean_cancel{7U,8U,8U,0U,1U,7U,0U,0U,
                                   false,false,false,true,18U,true};
  wire::DummyDeviceStatusV2 successor_status=canceled_status;
  successor_status.set_run_id(9U);
  successor_status.set_final_state(0U);
  successor_status.set_management_epoch(25U);
  wire::DummyJointStateV2 successor_state=canceled_state;
  successor_state.set_management_epoch(25U);
  first_page.motion_epoch=18U;
  const CancelReceipt cancel_receipt{101U,202U};
  /**
   * @par 场景边界
   * 内部missing_status_hold回调核旧Hold Status缺失时的严格替代路径：仍要强Cancel receipt、原RUN干净raw与严格继任首页。
   */
  /**
   * @par 场景与断言
   * 内部回调无参数并返回bool：把当前继任Status/State、旧raw、首页及CancelReceipt101/202交真实helper，核旧Hold Status缺失的严格闭合替代。
   */
  const auto missing_status_hold=[&] {
    return timeline_cancel_refresh_hold_confirmed(successor_status,successor_state,
      canceled_run,clean_cancel,first_page,cancel_receipt,true,0U,63U);
  };
  if(!missing_status_hold()) return 74;
  clean_cancel.final_state=6U;
  if(missing_status_hold()) return 75;
  clean_cancel.final_state=7U;
  clean_cancel.chain_valid=false;
  if(missing_status_hold()) return 76;
  clean_cancel.chain_valid=true;
  clean_cancel.run_id=7U;
  if(missing_status_hold()) return 84;
  clean_cancel.run_id=8U;
  clean_cancel.first_failure_source=1U;
  if(missing_status_hold()) return 85;
  clean_cancel.first_failure_source=0U;
  clean_cancel.first_failure_axis_mask=1U;
  if(missing_status_hold()) return 86;
  clean_cancel.first_failure_axis_mask=0U;
  clean_cancel.safety_hold_valid=false;
  if(missing_status_hold()) return 90;
  clean_cancel.safety_hold_valid=true;
  clean_cancel.safety_hold_motion_epoch=17U;
  if(missing_status_hold()) return 91;
  clean_cancel.safety_hold_motion_epoch=18U;
  if(timeline_cancel_refresh_hold_confirmed(successor_status,successor_state,
       canceled_run,clean_cancel,first_page,std::nullopt,true,0U,63U)) return 77;
  if(timeline_cancel_refresh_hold_confirmed(successor_status,successor_state,
       canceled_run,clean_cancel,first_page,cancel_receipt,true,1U,63U)) return 78;
  successor_status.set_run_id(10U);
  if(missing_status_hold()) return 79;
  successor_status.set_run_id(9U);
  successor_state.set_motion_epoch(17U);
  if(missing_status_hold()) return 80;
  successor_state.set_motion_epoch(18U);
  first_page.motion_epoch=17U;
  if(missing_status_hold()) return 81;
  first_page.motion_epoch=18U;
  first_page.chain_valid=false;
  if(missing_status_hold()) return 87;
  first_page.chain_valid=true;
  successor_status.set_reference_epoch(10U);
  successor_state.set_reference_epoch(10U);
  if(missing_status_hold()) return 88;
  successor_status.set_reference_epoch(9U);
  successor_state.set_reference_epoch(9U);
  successor_status.set_run_id(8U);
  successor_status.set_final_state(7U);
  if(missing_status_hold()) return 89;
  successor_status.set_run_id(9U);
  successor_status.set_final_state(0U);
  successor_state.set_can_sent_mask(31U);
  if(missing_status_hold()) return 82;
  successor_state.set_can_sent_mask(63U);
  if(!missing_status_hold()) return 83;
  first_page.motion_epoch=17U;
  if(!dummy_motion_client::verified_refresh_successor(first_page,run9)) return 35;
  first_page.group_count=2U;
  if(!dummy_motion_client::verified_refresh_successor(first_page,run9)) return 94;
  first_page.group_count=0U;
  if(dummy_motion_client::verified_refresh_successor(first_page,run9)) return 95;
  first_page.group_count=1U;
  first_page.strict_refresh_successor=false;
  if(dummy_motion_client::verified_refresh_successor(first_page,run9)) return 36;
  first_page.strict_refresh_successor=true;
  first_page.run_id=10U;
  if(dummy_motion_client::verified_refresh_successor(first_page,run9)) return 37;
  if(!dummy_motion_client::timeline_refresh_wait_allowed(run9,run8,true,true,
       refresh_status_us,refresh_status_us+state_age_budget_us-1U,state_age_budget_us) ||
     dummy_motion_client::timeline_refresh_wait_allowed(run9,run8,true,true,
       refresh_status_us,refresh_status_us+state_age_budget_us,state_age_budget_us) ||
     dummy_motion_client::timeline_refresh_wait_allowed(run9,run8,false,true,
       refresh_status_us,refresh_status_us,state_age_budget_us) ||
     dummy_motion_client::timeline_refresh_wait_allowed(run9,run8,true,false,
       refresh_status_us,refresh_status_us,state_age_budget_us)) return 30;
  dummy_motion_client::wire::DummyDeviceStatusV2 refresh_status{};
  dummy_motion_client::wire::DummyJointStateV2 refresh_state{};
  refresh_status.set_boot_epoch(7U);
  refresh_status.set_session_id(8U);
  refresh_status.set_reference_epoch(9U);
  refresh_status.set_motion_epoch(17U);
  refresh_status.set_management_epoch(25U);
  refresh_state.set_boot_epoch(7U);
  refresh_state.set_session_id(8U);
  refresh_state.set_reference_epoch(9U);
  refresh_state.set_motion_epoch(17U);
  refresh_state.set_management_epoch(25U);
  if(!dummy_motion_client::refresh_observation_coherent(refresh_status,refresh_state)) return 21;
  refresh_status.set_motion_epoch(18U);
  if(dummy_motion_client::refresh_observation_coherent(refresh_status,refresh_state)) return 22;
  refresh_status.set_motion_epoch(17U);
  refresh_status.set_management_epoch(26U);
  if(dummy_motion_client::refresh_observation_coherent(refresh_status,refresh_state)) return 23;
  refresh_status.set_management_epoch(25U);
  refresh_status.set_session_id(10U);
  if(dummy_motion_client::refresh_observation_coherent(refresh_status,refresh_state)) return 24;
  refresh_status.set_session_id(8U);
  const auto verified_refresh=dummy_motion_client::timeline_refresh_closure_identity(
    run9,run8,true,true,true,true,true,true);
  if(!verified_refresh || verified_refresh->run_id!=8U ||
     dummy_motion_client::timeline_refresh_closure_identity(
       run9,run8,true,true,false,true,true,true) ||
     dummy_motion_client::timeline_refresh_closure_identity(
       run9,run8,false,true,true,true,true,true) ||
     dummy_motion_client::timeline_refresh_closure_identity(
       run9,run8,true,false,true,true,true,true) ||
     dummy_motion_client::timeline_refresh_closure_identity(
       run9,run8,true,true,true,false,true,true) ||
     dummy_motion_client::timeline_refresh_closure_identity(
       run9,run8,true,true,true,true,false,true) ||
     dummy_motion_client::timeline_refresh_closure_identity(
       run9,run8,true,true,true,true,true,false)) return 16;
  auto foreign=run9;
  foreign.motion_epoch=18U;
  if(dummy_motion_client::timeline_refresh_wait_allowed(foreign,run8,true,true,
       refresh_status_us,refresh_status_us,state_age_budget_us)) return 31;
  if(dummy_motion_client::timeline_refresh_closure_identity(
       foreign,run8,true,true,true,true,true,true)) return 17;
  foreign=run9;
  foreign.session=10U;
  if(dummy_motion_client::timeline_refresh_wait_allowed(foreign,run8,true,true,
       refresh_status_us,refresh_status_us,state_age_budget_us)) return 32;
  if(dummy_motion_client::timeline_refresh_closure_identity(
       foreign,run8,true,true,true,true,true,true)) return 19;
  foreign=run9;
  foreign.run_id=10U;
  if(dummy_motion_client::timeline_refresh_wait_allowed(foreign,run8,true,true,
       refresh_status_us,refresh_status_us,state_age_budget_us)) return 33;
  if(dummy_motion_client::timeline_refresh_closure_identity(
       foreign,run8,true,true,true,true,true,true)) return 20;

  // ROS50 goal31: run32 Status was visible before its first raw evidence page.
  /**
   * @par 场景与断言
   * 复现run32 Status先于raw首页的顺序；group_count1不证明首页落盘，只可在有界窗口内继续等待，超时须报age_budget。
   */
  const TimelineIdentity run31{6898202070690061525ULL,8779283630540149424ULL,1U,63U,31U};
  const TimelineIdentity run32{run31.boot,run31.session,run31.reference_epoch,
                               run31.motion_epoch,32U};
  constexpr std::uint64_t run32_status_us=1000000ULL;
  wire::DummyDeviceStatusV2 run32_status{};
  run32_status.set_boot_epoch(run32.boot);
  run32_status.set_session_id(run32.session);
  run32_status.set_reference_epoch(run32.reference_epoch);
  run32_status.set_motion_epoch(run32.motion_epoch);
  run32_status.set_run_id(run32.run_id);
  run32_status.set_group_count(1U);
  run32_status.set_attempt_kind(1U);
  run32_status.set_core_state(wire_value(WireCoreState::kHold));
  RawFirstPageWritten run32_page{};
  if(run32_status.group_count()!=1U ||
     verified_refresh_successor(run32_page,run32) ||
     timeline_refresh_closure_identity(run32,run31,true,true,
       verified_refresh_successor(run32_page,run32),true,true,true) ||
     !timeline_refresh_wait_allowed(run32,run31,true,true,
       run32_status_us,run32_status_us,state_age_budget_us)) return 96;
  if(!timeline_refresh_wait_allowed(run32,run31,true,true,
       run32_status_us,run32_status_us+state_age_budget_us-1U,state_age_budget_us) ||
     timeline_refresh_wait_allowed(run32,run31,true,true,
       run32_status_us,run32_status_us+state_age_budget_us,state_age_budget_us)) return 97;
  if(std::string(refresh_closure_failure(run32,run31,true,true,false,true,true,true))!=
       "successor_page" ||
     std::string(refresh_wait_failure(run32,run31,true,true,run32_status_us,
       run32_status_us+state_age_budget_us,state_age_budget_us))!="age_budget" ||
     std::string(refresh_wait_failure(run32,run31,true,true,run32_status_us,
       run32_status_us+state_age_budget_us-1U,state_age_budget_us))!="none") return 100;
  // Captured run 1 final 6 was flushed before its Hold Status was observed.
  /**
   * @par 场景与断言
   * Complete终态raw final6可先于旧Hold Status到达；随后motion3/run2继任首页须与原run1/session及六轴发送一致，不能借Cancel final7。
   */
  const TimelineIdentity completed_run{8550615690737205600ULL,6553720987272504267ULL,
                                       1U,2U,1U};
  const TimelineIdentity completed_successor{completed_run.boot,completed_run.session,
                                             completed_run.reference_epoch,3U,2U};
  wire::DummyDeviceStatusV2 completed_status{};
  completed_status.set_boot_epoch(completed_successor.boot);
  completed_status.set_session_id(completed_successor.session);
  completed_status.set_reference_epoch(completed_successor.reference_epoch);
  completed_status.set_motion_epoch(completed_successor.motion_epoch);
  completed_status.set_management_epoch(7U);
  completed_status.set_run_id(completed_successor.run_id);
  completed_status.set_attempt_kind(1U);
  completed_status.set_parent_run_id(0U);
  completed_status.set_final_state(0U);
  completed_status.set_core_state(wire_value(WireCoreState::kHold));
  wire::DummyJointStateV2 completed_state{};
  completed_state.set_boot_epoch(completed_successor.boot);
  completed_state.set_session_id(completed_successor.session);
  completed_state.set_reference_epoch(completed_successor.reference_epoch);
  completed_state.set_motion_epoch(completed_successor.motion_epoch);
  completed_state.set_management_epoch(7U);
  completed_state.set_core_state(wire_value(WireCoreState::kHold));
  completed_state.set_hold_ready(true);
  completed_state.set_can_submitted_mask(63U);
  completed_state.set_can_sent_mask(63U);
  const RawTerminalIdentity completed_terminal{completed_run.boot,completed_run.session,
    completed_run.run_id,0U,1U,6U,0U,0U,false,true,false,true,0U,false};
  RawFirstPageWritten completed_page{};
  completed_page.boot_epoch=completed_successor.boot;
  completed_page.session_id=completed_successor.session;
  completed_page.run_id=completed_successor.run_id;
  completed_page.motion_epoch=completed_successor.motion_epoch;
  completed_page.evidence_seq=0U;
  completed_page.group_count=2U;
  completed_page.attempt_kind=1U;
  completed_page.category=1U;
  completed_page.expected_mask=completed_page.submitted_mask=completed_page.sent_mask=63U;
  completed_page.final=completed_page.chain_valid=completed_page.full_six=
    completed_page.strict_refresh_successor=true;
  /**
   * @par 场景边界
   * 内部completed_refresh_hold回调核Complete继任；terminal、旧身份、继任page与双流身份必须一致，group_count非零声明不能独立闭合。
   */
  /**
   * @par 场景与断言
   * 内部completed_refresh_hold回调以status/state/prior/terminal/page值为参数，返回timeline_completed_refresh_hold_confirmed结果；不修改真实场景或执行设备。
   */
  const auto completed_refresh_hold=[&](const wire::DummyDeviceStatusV2 &status,
      const wire::DummyJointStateV2 &state,const TimelineIdentity &prior,
      const RawTerminalIdentity &terminal,const RawFirstPageWritten &page) {
    return timeline_completed_refresh_hold_confirmed(status,state,prior,terminal,page,
      true,0U,63U);
  };
  const bool completed_prior_hold=completed_refresh_hold(completed_status,completed_state,
    completed_run,completed_terminal,completed_page);
  const TimelineIdentity inferred_prior_hold{completed_run.boot,completed_run.session,
    completed_run.reference_epoch,completed_successor.motion_epoch,completed_run.run_id};
  if(!completed_prior_hold ||
     !timeline_refresh_closure_identity(completed_successor,inferred_prior_hold,
       completed_prior_hold,true,
       verified_refresh_successor(completed_page,completed_successor),true,true,true)) return 101;
  if(timeline_completed_refresh_hold_confirmed(completed_status,completed_state,
       completed_run,completed_terminal,completed_page,false,0U,63U) ||
     timeline_completed_refresh_hold_confirmed(completed_status,completed_state,
       completed_run,completed_terminal,completed_page,true,1U,63U)) return 115;
  /**
   * @par 场景与断言
   * 依次破坏旧boot/session/run、terminal身份/final/chain/motion_complete、继任page及Status故障位；每次恢复其余字段以定位该前提。
   */
  auto wrong_prior=completed_run;
  wrong_prior.boot++;
  if(completed_refresh_hold(completed_status,completed_state,wrong_prior,
       completed_terminal,completed_page)) return 102;
  wrong_prior=completed_run;
  wrong_prior.session++;
  if(completed_refresh_hold(completed_status,completed_state,wrong_prior,
       completed_terminal,completed_page)) return 103;
  wrong_prior=completed_run;
  wrong_prior.run_id++;
  if(completed_refresh_hold(completed_status,completed_state,wrong_prior,
       completed_terminal,completed_page)) return 104;
  auto bad_terminal=completed_terminal;
  bad_terminal.boot_epoch++;
  if(completed_refresh_hold(completed_status,completed_state,completed_run,
       bad_terminal,completed_page)) return 110;
  bad_terminal=completed_terminal;
  bad_terminal.session_id++;
  if(completed_refresh_hold(completed_status,completed_state,completed_run,
       bad_terminal,completed_page)) return 111;
  bad_terminal=completed_terminal;
  bad_terminal.run_id++;
  if(completed_refresh_hold(completed_status,completed_state,completed_run,
       bad_terminal,completed_page)) return 112;
  bad_terminal=completed_terminal;
  bad_terminal.final_state=7U;
  if(completed_refresh_hold(completed_status,completed_state,completed_run,
       bad_terminal,completed_page)) return 105;
  bad_terminal=completed_terminal;
  bad_terminal.chain_valid=false;
  if(completed_refresh_hold(completed_status,completed_state,completed_run,
       bad_terminal,completed_page)) return 106;
  bad_terminal=completed_terminal;
  bad_terminal.motion_complete=false;
  if(completed_refresh_hold(completed_status,completed_state,completed_run,
       bad_terminal,completed_page)) return 113;
  auto missing_page=completed_page;
  missing_page.run_id=0U;
  if(completed_refresh_hold(completed_status,completed_state,completed_run,
       completed_terminal,missing_page)) return 107;
  missing_page=completed_page;
  missing_page.strict_refresh_successor=false;
  if(completed_refresh_hold(completed_status,completed_state,completed_run,
       completed_terminal,missing_page)) return 114;
  auto stale_status=completed_status;
  stale_status.set_motion_epoch(2U);
  if(completed_refresh_hold(stale_status,completed_state,completed_run,
       completed_terminal,completed_page)) return 108;
  auto fault_status=completed_status;
  fault_status.set_evidence_lost(true);
  if(completed_refresh_hold(fault_status,completed_state,completed_run,
       completed_terminal,completed_page)) return 109;
  fault_status=completed_status;
  fault_status.set_motion_complete(true);
  if(completed_refresh_hold(fault_status,completed_state,completed_run,
       completed_terminal,completed_page)) return 116;
  run32_page.boot_epoch=run32.boot;
  run32_page.session_id=run32.session;
  run32_page.run_id=run32.run_id;
  run32_page.motion_epoch=run32.motion_epoch;
  run32_page.evidence_seq=0U;
  run32_page.group_count=1U;
  run32_page.attempt_kind=1U;
  run32_page.category=1U;
  run32_page.expected_mask=run32_page.submitted_mask=run32_page.sent_mask=63U;
  run32_page.final=run32_page.chain_valid=run32_page.full_six=
    run32_page.strict_refresh_successor=true;
  if(!verified_refresh_successor(run32_page,run32) ||
     !timeline_refresh_closure_identity(run32,run31,true,true,
       verified_refresh_successor(run32_page,run32),true,true,true)) return 98;
  run32_page.group_count=0U;
  if(verified_refresh_successor(run32_page,run32) ||
     timeline_refresh_closure_identity(run32,run31,true,true,
       verified_refresh_successor(run32_page,run32),true,true,true)) return 99;
  /**
   * @par 场景边界
   * 以下开始ROS内存控制器阶段：prime direct保持0.25rad，已有JTC目标0.35rad不应被旧direct刷新覆盖。
   */
  rclcpp::init(argc,argv);

  auto controller=std::make_shared<dummy_motion_client::Controller>();
  const auto options=rclcpp::NodeOptions().parameter_overrides({
    rclcpp::Parameter("joints",std::vector<std::string>{"Joint1"}),
    rclcpp::Parameter("command_interfaces",std::vector<std::string>{"position"}),
    rclcpp::Parameter("state_interfaces",std::vector<std::string>{"position","velocity"})});
  const std::string urdf=R"(<robot name="probe"><link name="base"/><link name="tip"/><joint name="Joint1" type="revolute"><parent link="base"/><child link="tip"/><axis xyz="0 0 1"/><limit lower="-1" upper="1" velocity="1" effort="1"/></joint></robot>)";
  if(controller->init("handoff_probe",urdf,100,"/handoff",options)!=controller_interface::return_type::OK) return 4;
  /**
   * @par 场景边界
   * 控制器activate本身不授予命令发布权；必须adopt当前Hold后才能向GuardedPort发布。
   */
  controller->get_node()->configure();
  /**
   * @par 场景与断言
   * 内存接口：command输出rad，position初始0.35rad、velocity0rad/s；Loaned接口借用这些栈变量直到本main结束。
   */
  double command=0.0,position=0.35,velocity=0.0;
  hardware_interface::CommandInterface command_interface("Joint1","position",&command);
  hardware_interface::StateInterface state_position("Joint1","position",&position),state_velocity("Joint1","velocity",&velocity);
  std::vector<hardware_interface::LoanedCommandInterface> commands;
  commands.emplace_back(command_interface);
  std::vector<hardware_interface::LoanedStateInterface> states;
  states.emplace_back(state_position); states.emplace_back(state_velocity);
  controller->assign_interfaces(std::move(commands),std::move(states));
  controller->get_node()->activate();

  GuardedPort port;
  port.active_count=1;
  port.identity={7,8,9,0,11};
  dummy_motion_client::Position hold{};
  hold[0]=0.25;
  port.prime(hold,10,1);
  const auto initial=consume_control_update(0);
  if(!initial || initial->position[0]!=0.25) return 1;
  if(!port.freeze_hold()) return 21;
  port.refresh_direct_hold();
  if(consume_control_update(initial->generation)) return 22;
  set_command_identity(port.identity);

  port.controller=controller;
  const double jtc_position=0.35;
  const auto jtc_generation=publish_control_update(&jtc_position,1,0,nullptr);
  if(!jtc_generation) return 2;
  port.refresh_direct_hold();
  const auto latest=consume_control_update(jtc_generation-1);
  if(!latest || latest->position[0]!=jtc_position) {
    std::cerr << "old direct hold overwrote JTC update\n";
    return 3;
  }
  port.disable();
  port.cancel();

  /**
   * @par 场景与断言
   * direct 0.25rad在controller inactive时继续保持；controller再次activate仍不能发布，须adopt_controller成功移交同一Hold后才恢复JTC。
   */
  GuardedPort handoff;
  handoff.active_count=1;
  handoff.identity={7,8,9,0,11};
  handoff.prime(hold,10,1);
  const auto hold_generation=consume_control_update(jtc_generation);
  if(!hold_generation) return 5;
  controller->get_node()->deactivate();
  handoff.refresh_direct_hold();
  const auto pre_adoption_hold=consume_control_update(hold_generation->generation);
  if(!pre_adoption_hold || pre_adoption_hold->position[0]!=hold[0]) return 15;
  controller->get_node()->activate();
  controller->update(controller->get_node()->now(),rclcpp::Duration::from_seconds(0.01));
  if(consume_control_update(pre_adoption_hold->generation)) {
    std::cerr << "JTC published before the gate primed it\n";
    return 8;
  }
  handoff.adopt_controller(controller);
  if(handoff.failed || handoff.direct_hold) return 6;
  controller->update(controller->get_node()->now(),rclcpp::Duration::from_seconds(0.01));
  const auto handed_off=consume_control_update(pre_adoption_hold->generation);
  if(!handed_off || handed_off->position[0]!=hold[0] || command!=hold[0]) return 7;
  controller->get_node()->deactivate();
  handoff.refresh_direct_hold();
  const auto fallback=consume_control_update(handed_off->generation);
  if(!fallback || fallback->position[0]!=hold[0] || !handoff.direct_hold) {
    std::cerr << "direct hold did not resume after JTC deactivation state="
              << controller->get_lifecycle_state().id() << " direct=" << handoff.direct_hold
              << " published=" << static_cast<bool>(fallback) << '\n';
    return 9;
  }
  controller->get_node()->activate();
  controller->update(controller->get_node()->now(),rclcpp::Duration::from_seconds(0.01));
  if(consume_control_update(fallback->generation)) return 13;
  handoff.adopt_controller(controller);
  if(handoff.failed || handoff.direct_hold) return 10;
  controller->update(controller->get_node()->now(),rclcpp::Duration::from_seconds(0.01));
  const auto reactivated=consume_control_update(fallback->generation);
  if(!reactivated || reactivated->position[0]!=hold[0]) return 11;
  /**
   * @par 场景与断言
   * 未配置controller的prime须failed且not ready；NaN Hold的adopt也须failed，即使publishing标志仍true亦不得ready。
   */
  GuardedPort failed_prime;
  failed_prime.identity={7,8,9,0,11};
  failed_prime.active_count=1;
  failed_prime.controller=std::make_shared<dummy_motion_client::Controller>();
  failed_prime.prime(hold,10,1);
  if(!failed_prime.failed || failed_prime.control_ready()) return 12;
  handoff.disable();
  GuardedPort failed_adopt;
  failed_adopt.identity={7,8,9,0,11};
  failed_adopt.active_count=1;
  failed_adopt.prime(hold,10,1);
  failed_adopt.hold_position[0]=std::numeric_limits<double>::quiet_NaN();
  failed_adopt.adopt_controller(controller);
  if(!failed_adopt.publishing || !failed_adopt.failed || failed_adopt.control_ready()) return 14;
  failed_adopt.disable();
  failed_adopt.cancel();
  handoff.cancel();
  controller->get_node()->deactivate();
  rclcpp::shutdown();
}
