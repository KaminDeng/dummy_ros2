/**
 * @file review_boundary_tests.cpp
 * @brief 按模式核枚举协议值、饱和期限、末点容差和单轴State边界。
 *
 * 复用Harness内存状态机；State走正式codec与公共observe_state，不启动ROS或设备。每次main只跑一个scenario。
 */
#include "action_test_support.hpp"
#include <limits>
#include <iostream>
using namespace action_test;
namespace wire=dp::protocol::dummy_robot_v2;
/**
 * @brief 构造J1 State，检查未使用轴NaN与Hold缺发送掩码的不同语义。
 * @param partial_hold_mask false为EnablingHold3且J6 NaN（未配置轴允许），应prime一次；true为Hold4但组mask不足，不得prime。
 * @return State接纳且prime计数符合所选条件。
 */
bool state_case(bool partial_hold_mask) {
  Harness harness; Authorization auth; auth.session=auth.reference=auth.permit=true;
  auth.identity.set_expected_session_id(1); auth.identity.set_configured_mask(1);
  require(harness.gate.enable(auth,1));
  wire::DummyJointStateV2 state; state.set_schema_major(2); state.set_schema_minor(dp::protocol::dummy_robot_v2::kSchemaMinor);
  state.set_state_seq(1);
  state.set_session_id(1); state.set_configured_mask(1); state.set_position_valid_mask(1);
  state.set_hold_ready(!partial_hold_mask); state.set_motion_epoch(1);
  state.set_core_state(partial_hold_mask?4:3);
  double positions[6]{};
  if (!partial_hold_mask) positions[5]=std::numeric_limits<double>::quiet_NaN();
  state.set_position_rad(positions);
  std::vector<std::uint8_t> bytes(wire::DummyJointStateV2Codec::kEncodedSize);
  require(wire::DummyJointStateV2Codec::encode(state,bytes.data(),bytes.size())==wire::CodecResult::kOk);
  const bool observed=harness.gate.observe_state(bytes.data(),bytes.size(),2);
  return observed && harness.port.primes==(partial_hold_mask?0U:1U);
}
/**
 * @brief 选择enum/deadline/tolerance/unused_nan/hold_mask边界。
 * @param argc 参数个数（含程序名）。
 * @param argv 参数数组，模式/路径位置见下述说明。
 * @return 0表示所选检查通过；非零或断言/require异常表示失败。
 * enum核操作6/7/8/9/10/12与Queued1/Pending23及正式core编码；deadline核UINT64_MAX附近不回绕；tolerance末点0.15距目标0.2超过0.01rad应BAD_TERMINAL_POSITION。
 * unused_nan只检查未配置轴，不能推广为任意NaN都可入库；未知scenario3、缺参数2。
 */
int main(int argc,char **argv) {
  if (argc!=2) return 2; const std::string scenario=argv[1];
  if (scenario=="enum") {
    require(static_cast<unsigned>(ManageOperation::kEnable)==6);
    require(static_cast<unsigned>(ManageOperation::kBegin)==7);
    require(static_cast<unsigned>(ManageOperation::kCancel)==9);
    require(static_cast<unsigned>(ManageOperation::kDisableAndRelease)==10);
    require(static_cast<unsigned>(ManageOperation::kQuery)==12);
    require(static_cast<unsigned>(ManageOperation::kComplete)==8);
    require(wire_value(WireReceipt::kQueued)==1);
    require(wire_value(WireResult::kPending)==23);
    require(wire_value(WireCoreState::kEnablingHold)==3);
    require(wire_value(WireCoreState::kHold)==4);
    require(wire_value(WireCoreState::kRunning)==5);
    require(wire_value(WireCoreState::kFault)==9);
  } else if (scenario=="deadline") {
    Harness harness; Authorization auth; auth.session=auth.reference=auth.permit=true;
    auth.identity.set_expected_session_id(1);
    const auto maximum=std::numeric_limits<std::uint64_t>::max();
    require(harness.gate.enable(auth,maximum-5)); harness.gate.tick(maximum-4);
    require(harness.gate.terminal()==Terminal::kNone);
    harness.gate.tick(maximum); require(harness.gate.terminal()==Terminal::kAborted);
  } else if (scenario=="tolerance") {
    Harness harness; harness.enable();
    const Trajectory trajectory{{"Joint1"},{{{0.2},{0},{},1}},{},{0.01},0};
    require(harness.gate.submit(trajectory,10)); ++harness.epoch;
    harness.gate.observe(harness.response(),11);
    require(harness.gate.record_update(harness.gate.generation(),harness.epoch,1,Position{}));
    harness.gate.observe(harness.response(true),12);
    Position terminal{}; terminal[0]=0.15;
    require(harness.gate.record_update(harness.gate.generation(),harness.epoch,2,terminal));
    harness.gate.inner_result(harness.gate.generation(),Terminal::kSucceeded,"",13);
    require(harness.gate.error()=="BAD_TERMINAL_POSITION");
  } else if (scenario=="unused_nan") require(state_case(false));
  else if (scenario=="hold_mask") require(state_case(true));
  else return 3;
  std::cout << scenario << " boundary verified\n";
}
