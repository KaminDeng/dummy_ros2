/** @file panel_feedback_tests.cpp
 * @brief 中央消息到ROS消息的语义投影，不构造硬件或Native进程。 */
#include "dummy_motion_client/panel_feedback.hpp"
#include <cassert>
#include <array>
#include <vector>
int main() {
  dp::iface::DummyJointStateV2 state;
  state.set_boot_epoch(7); state.set_state_seq(17);
  state.set_position_valid_mask(0); state.set_raw_position_valid_mask(63);
  double raw[6]{1,2,3,4,5,6}; state.set_raw_motor_position_rev(raw);
  const auto projected=dummy_motion_client::panel_state_message(state);
  assert(projected.boot_epoch==7 && projected.state_seq==17);
  assert(projected.position_valid_mask==0 && projected.raw_position_valid_mask==63);
  assert(projected.raw_motor_position_rev[5]==6);
  dp::iface::DummyDeviceStatusV2 status;
  status.set_release_committed(true); status.set_session_id(0);
  status.set_can_sent_mask(63); status.set_run_id(91);
  const auto released=dummy_motion_client::panel_status_message(status);
  assert(released.release_committed && released.session_id==0);
  assert(released.can_sent_mask==63 && released.run_id==91);
  // 实际中央编码消息：控制锁不可得时，鲜诊断仍须独立发布，不造有效位。
  using namespace dummy_motion_client;
  PanelFeedbackCache unavailable;
  state.set_schema_major(dummy_wire_schema::kSchemaMajor);state.set_schema_minor(dummy_wire_schema::kSchemaMinor);
  state.set_core_state(2);status.set_core_state(2);
  status.set_schema_major(dummy_wire_schema::kSchemaMajor);status.set_schema_minor(dummy_wire_schema::kSchemaMinor);
  state.set_state_seq(22);state.set_position_valid_mask(0);
  status.set_run_id(92);
  std::vector<std::uint8_t> status_bytes(dp::iface::DummyDeviceStatusV2Codec::kEncodedSize);
  std::vector<std::uint8_t> state_bytes(dp::iface::DummyJointStateV2Codec::kEncodedSize);
  assert(dp::iface::DummyDeviceStatusV2Codec::encode(status,status_bytes.data(),status_bytes.size())==dp::iface::CodecResult::kOk);
  assert(dp::iface::DummyJointStateV2Codec::encode(state,state_bytes.data(),state_bytes.size())==dp::iface::CodecResult::kOk);
  PanelFeedbackBytes ds{status_bytes.data(),status_bytes.size(),500000};
  PanelFeedbackBytes dq{state_bytes.data(),state_bytes.size(),1000000};
  std::uint64_t sw=0,qw=0;
  const auto selected=select_panel_feedback(unavailable,ds,dq,1010000,250000,1000000,sw,qw);
  assert(selected.status && selected.state);
  assert(selected.status->run_id()==92 && selected.state->state_seq()==22);
  assert(selected.state->position_valid_mask()==0 && sw==500000 && qw==1000000);
  assert(unavailable.status_received_us==0 && unavailable.state_received_us==0);
  const auto duplicate=select_panel_feedback(unavailable,ds,dq,1020000,250000,1000000,sw,qw);
  assert(!duplicate.status && !duplicate.state);
  // 旧帧、未来时刻、坏codec均不得推进水位或给旧数据续鲜。
  ds.received_us=400000;dq.received_us=900000;
  const auto backward=select_panel_feedback(unavailable,ds,dq,1020000,250000,1000000,sw,qw);
  assert(!backward.status && !backward.state && sw==500000 && qw==1000000);
  ds.received_us=1100000;dq.received_us=1100000;
  const auto future=select_panel_feedback(unavailable,ds,dq,1020000,250000,1000000,sw,qw);
  assert(!future.status && !future.state && sw==500000 && qw==1000000);
  ds.received_us=1200000;dq.received_us=1200000;
  const auto stale=select_panel_feedback(unavailable,ds,dq,2300001,250000,1000000,sw,qw);
  assert(!stale.status && !stale.state && sw==500000 && qw==1000000);
  ds.received_us=1300000;dq.received_us=1300000;ds.size=1;dq.size=1;
  const auto invalid=select_panel_feedback(unavailable,ds,dq,1300001,250000,1000000,sw,qw);
  assert(!invalid.status && !invalid.state && sw==500000 && qw==1000000);
  // 控制快照可用时仍择更新诊断；State250ms拒绝而Status1秒保留。
  PanelFeedbackCache cache;cache.available=true;
  cache.status=status;cache.state=state;cache.status_received_us=1400000;cache.state_received_us=1400000;
  ds.size=status_bytes.size();dq.size=state_bytes.size();ds.received_us=1500000;dq.received_us=1500000;
  const auto newer=select_panel_feedback(cache,ds,dq,1500001,250000,1000000,sw,qw);
  assert(newer.status && newer.state && sw==1500000 && qw==1500000);
  sw=0;qw=0;ds.received_us=1500000;dq.received_us=1500000;
  const auto separate=select_panel_feedback(unavailable,ds,dq,1750001,250000,1000000,sw,qw);
  assert(separate.status && !separate.state && sw==1500000 && qw==0);
  return 0;
}
