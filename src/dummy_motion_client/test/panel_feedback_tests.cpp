/** @file panel_feedback_tests.cpp
 * @brief 中央消息到ROS消息的语义投影，不构造硬件或Native进程。 */
#include "dummy_motion_client/panel_feedback.hpp"
#include <cassert>
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
  return 0;
}
