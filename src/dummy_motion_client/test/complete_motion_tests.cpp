/**
 * @file complete_motion_tests.cpp
 * @brief 核J1完成屏障、同值抑制prime的排空证据及全部到达排列。
 *
 * 复用action_test_support.hpp的内存Client/计数Port/Harness，管理快照是手工合成；不创建ROS节点或执行真实轨迹。位置rad，时间为人工Hostµs，轨迹点time为秒。
 */
#include "action_test_support.hpp"
#include <iostream>
#include <algorithm>
#include <limits>
/**
 * @brief 在Complete后下一Hold epoch排列四类证据，检查只有完整同根屏障才能Succeeded。
 * @param order 四事件排列：0末更新登记、1正式State codec输入、2hold_drain、3管理/观测回复。
 * @param invalid 0正例；1..19逐项破坏quiescent/disposition/身份/代次/位置/掩码/seq/年龄/ready/容差/撤销等。
 * @return 无；require不符抛异常。
 * @details processed_generation12785表示新目标已处理，但committed_seq12784仍指前次已发送组，不能把抑制目标代次误认线序号ACK。
 */
void completion_with_suppressed_prime(std::array<unsigned,4> order,unsigned invalid=0U) {
  using namespace action_test;
  namespace wire=dp::protocol::dummy_robot_v2;
  Harness h;
  Authorization auth; auth.session=auth.reference=auth.permit=true;
  auth.identity.set_expected_session_id(1U); auth.identity.set_configured_mask(1U);
  require(h.gate.enable(auth,1U)); ++h.epoch;
  h.gate.observe(h.response(),2U);
  require(h.gate.record_update(h.gate.generation(),h.epoch,1U,Position{}));
  h.gate.observe(h.response(true),3U); h.begin();
  Position end{}; end[0]=0.2;
  require(h.gate.record_update(h.gate.generation(),h.epoch,12780U,end));
  h.gate.inner_result(h.gate.generation(),Terminal::kSucceeded,"",13U);
  h.gate.observe(h.response(true,12780U),14U);
  ++h.epoch;
  h.gate.observe(h.response(false,12784U),15U);
  wire::DummyJointStateV2 state;
  state.set_schema_major(2U); state.set_schema_minor(dp::protocol::dummy_robot_v2::kSchemaMinor); state.set_state_seq(1U); state.set_session_id(1U);
  state.set_motion_epoch(h.epoch); state.set_core_state(wire_value(WireCoreState::kHold));
  state.set_hold_ready(true); state.set_committed_producer_seq(12784U);
  state.set_configured_mask(1U); state.set_position_valid_mask(1U);
  state.set_can_submitted_mask(1U); state.set_can_sent_mask(1U);
  HoldDrainEvidence processed{0U,1U,12785U,0U,h.epoch,12784U};
  processed.disposition=1U; processed.quiescent=true;
  processed.observed_us=16U; processed.max_state_age_us=250000U;
  processed.tolerance[0]=0.01;
  /**
   * @brief 1非静止、2无disposition、3..6错boot/session/ref/motion、7旧processed代次、8错排空位置；
   * 9/10缺sent/submitted、11错committed、12年龄仅1µs、13State位置错、14not-ready、15管理未成功、16NaN容差、17负容差、18撤销排空、19错reply epoch均不得完成。
   */
  switch(invalid) {
    case 1: processed.quiescent=false; break;
    case 2: processed.disposition=0U; break;
    case 3: processed.boot_epoch=9U; break;
    case 4: ++processed.session_id; break;
    case 5: ++processed.reference_epoch; break;
    case 6: ++processed.motion_epoch; break;
    case 7: processed.processed_generation=12784U; break;
    case 8: processed.position[0]=0.2; break;
    case 9: state.set_can_sent_mask(0U); break;
    case 10: state.set_can_submitted_mask(0U); break;
    case 11: processed.committed_seq=12786U; break;
    case 12: processed.max_state_age_us=1U; break;
    case 13: { double wrong[6]={0.2,0,0,0,0,0}; state.set_position_rad(wrong); break; }
    case 14: state.set_hold_ready(false); break;
    case 16: processed.tolerance[0]=std::numeric_limits<double>::quiet_NaN(); break;
    case 17: processed.tolerance[0]=-0.01; break;
    default: break;
  }
  std::uint64_t now=20U;
  for(const auto event:order) {
    if(event==0U)
      require(h.gate.record_update(h.gate.generation(),h.epoch,12785U,Position{},12785U));
    if(event==1U) {
      std::array<std::uint8_t,wire::DummyJointStateV2Codec::kEncodedSize> bytes{};
      require(wire::DummyJointStateV2Codec::encode(state,bytes.data(),bytes.size())==wire::CodecResult::kOk);
      require(h.gate.observe_state(bytes.data(),bytes.size(),now));
    }
    if(event==2U) {
      h.gate.observe_hold_drain(processed,now);
      if(invalid==18U) h.gate.observe_hold_drain({},now);
    }
    if(event==3U) {
      auto response=h.response(invalid!=15U,12784U);
      if(invalid==19U) --response.motion_epoch;
      h.gate.observe(response,now);
    }
    h.gate.tick(now++);
  }
  require(h.gate.terminal()==(invalid ? Terminal::kNone : Terminal::kSucceeded));
  if(invalid) {
    h.gate.tick(1000014U);
    require(h.gate.phase()==Phase::kStopping && h.gate.error()=="RESULT_UNCONFIRMED");
  }
}
/**
 * @brief 先完成两目标，再检查24种证据排列和19个负例。
 * @return 0表示所选检查通过；非零或断言/require异常表示失败。
 * 原generation记录不能重入；每个负例保持terminal=None，推进到1000014µs后必须Stopping/RESULT_UNCONFIRMED，不把内层Succeeded当外层完成。
 */
int main() {
  using namespace action_test;
  Harness harness; harness.enable(); unsigned successes=0;
  for (unsigned goal=0;goal<2;++goal) {
    harness.begin(); harness.complete();
    successes+=harness.gate.terminal()==Terminal::kSucceeded ? 1:0;
  }
  require(successes==2 && harness.port.starts==2);
  const auto old_generation=harness.gate.generation()-1;
  const bool old_accepted=harness.gate.record_update(old_generation,harness.epoch,2,Position{});
  require(!old_accepted);
  std::array<unsigned,4> order{0U,1U,2U,3U};
  do {
    completion_with_suppressed_prime(order);
  } while(std::next_permutation(order.begin(),order.end()));
  for(unsigned invalid=1;invalid<=19;++invalid) completion_with_suppressed_prime(order,invalid);
  std::cout << "{\"component_successful_goals\":" << successes << ",\"component_completion_barrier_count\":" << successes
    << ",\"component_old_generation_accepted\":" << (old_accepted?"true":"false") << "}\n";
}
