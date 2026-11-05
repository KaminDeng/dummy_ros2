/** @file panel_feedback_tests.cpp
 * @brief 中央Owned到ROS投影保留presence、关节质量和SDK来源；不启动设备。 */
#include "dummy_motion_client/panel_feedback.hpp"
#include "../../dummy_ros_control/test/owned_message_fixtures.hpp"
#include "dummy_ros_control/message_presence.hpp"
#include <cassert>
#include "dpsdk_interfaces/owned_projection.hpp"
/** @brief 生成器边界保留显式零/false/空容器，并清除复用ROS目标的旧值。 */
static void generated_projection_presence(){
  dp::comm::SourceId source{};source.device.bytes[15]=23;source.boot_id=UINT64_C(0xfedcba9876543210);source.context_id=42;
  dummy_ref::messages::State state;
  state.session_id()=0;state.hold_ready()=false;state.joints().emplace();
  dpsdk_interfaces::msg::DummyJointState output;
  dp::protocol::ros::project(state,source,output);
  assert(output.has_session_id&&output.session_id==0&&output.has_hold_ready&&!output.hold_ready);
  assert(output.has_joints&&output.joints.empty()&&!output.has_state_seq);
  assert(output.source.boot_lo==0x76543210U&&output.source.boot_hi==0xfedcba98U&&output.source.device[15]==23&&output.source.context_id==42);
  auto* joint=state.joints()->emplace_back();assert(joint);
  joint->joint_id()=0;joint->configured_quality()=false;joint->position_rad()=0.;
  dp::protocol::ros::project(state,source,output);
  assert(output.joints.size()==1&&output.joints[0].has_joint_id&&output.joints[0].joint_id==0);
  assert(output.joints[0].has_configured_quality&&!output.joints[0].configured_quality);
  assert(output.joints[0].has_position_rad&&output.joints[0].position_rad==0.&&!output.joints[0].has_hold_position_rad);
  state.session_id().reset();state.hold_ready().reset();state.joints().reset();
  output.session_id=99;output.hold_ready=true;
  dp::protocol::ros::project(state,source,output);
  assert(!output.has_session_id&&output.session_id==0&&!output.has_hold_ready&&!output.hold_ready);
  assert(!output.has_joints&&output.joints.empty());
  dp::protocol::system::dummy::SourceDescriptionOwned text;
  text.device_name().emplace().assign("",0);text.robot_id().emplace();text.module_instance_id()=0;
  dpsdk_interfaces::msg::SystemSourceDescription description;
  description.device_name="stale";description.module_name="stale";
  dp::protocol::ros::project(text,source,description);
  assert(description.has_device_name&&description.device_name.empty()&&!description.has_module_name&&description.module_name.empty());
  assert(description.has_robot_id&&description.robot_id.empty()&&description.has_module_instance_id&&description.module_instance_id==0);
}
int main(){
  generated_projection_presence();
  using namespace dummy_motion_client;
  auto state=owned_fixture::state();auto status=owned_fixture::status();
  dp::comm::SourceId source{};source.device.bytes[0]=9;source.boot_id=0x100000007ULL;source.context_id=42;
  state.state_seq()=17;
  for(std::size_t i=0;i<state.joints()->size();++i){auto& joint=(*state.joints())[i];joint.position_valid_quality()=false;joint.raw_motor_position_rev()=double(i+1);}
  const auto projected=panel_state_message(state,source);
  assert(projected.source.boot_lo==7&&projected.source.boot_hi==1&&projected.source.context_id==42&&projected.source.device[0]==9);
  assert(projected.has_state_seq&&projected.state_seq==17&&projected.has_joints&&projected.joints.size()==6);
  assert(projected.joints[0].has_position_valid_quality&&!projected.joints[0].position_valid_quality);
  assert(projected.joints[5].has_raw_motor_position_rev&&projected.joints[5].raw_motor_position_rev==6);
  status.release_committed()=true;status.session_id()=0;status.can_sent_mask()=63;status.run_id()=91;
  auto released=panel_status_message(status,source);
  assert(released.has_release_committed&&released.release_committed&&released.has_session_id&&released.session_id==0);
  assert(released.can_sent_mask==63&&released.run_id==91);
  status.release_committed().reset();released=panel_status_message(status,source);assert(!released.has_release_committed);
  status.release_committed()=false;released=panel_status_message(status,source);assert(released.has_release_committed&&!released.release_committed);
  // 控制锁不可得时使用独立诊断观察；不回填控制缓存，不造有效位。
  PanelFeedbackCache unavailable;
  state.core_state()=2;status.core_state()=2;state.state_seq()=22;status.run_id()=92;
  PanelFeedbackObservation<dummy_ref::messages::Status> ds{&status,source,500000};
  PanelFeedbackObservation<dummy_ref::messages::State> dq{&state,source,1000000};
  std::uint64_t sw=0,qw=0;
  const auto selected=select_panel_feedback(unavailable,ds,dq,1010000,250000,1000000,sw,qw);
  assert(selected.status&&selected.state&&*selected.status->run_id()==92&&*selected.state->state_seq()==22);
  assert(dummy_ros_control::position_valid_joint_mask(*selected.state)==0&&sw==500000&&qw==1000000);
  assert(selected.state_source.boot_id==source.boot_id&&selected.status_source.context_id==42);
  assert(unavailable.status_received_us==0&&unavailable.state_received_us==0);
  const auto duplicate=select_panel_feedback(unavailable,ds,dq,1020000,250000,1000000,sw,qw);assert(!duplicate.status&&!duplicate.state);
  // 旧帧、未来时刻、失效presence不推进水位，也不给旧数据续鲜。
  ds.received_us=400000;dq.received_us=900000;
  auto rejected=select_panel_feedback(unavailable,ds,dq,1020000,250000,1000000,sw,qw);assert(!rejected.status&&!rejected.state&&sw==500000&&qw==1000000);
  ds.received_us=1100000;dq.received_us=1100000;
  rejected=select_panel_feedback(unavailable,ds,dq,1020000,250000,1000000,sw,qw);assert(!rejected.status&&!rejected.state&&sw==500000&&qw==1000000);
  ds.received_us=1200000;dq.received_us=1200000;
  rejected=select_panel_feedback(unavailable,ds,dq,2300001,250000,1000000,sw,qw);assert(!rejected.status&&!rejected.state&&sw==500000&&qw==1000000);
  auto missing_status=status;missing_status.run_id().reset();auto missing_state=state;missing_state.state_seq().reset();
  ds.received_us=dq.received_us=1300000;ds.value=&missing_status;dq.value=&missing_state;
  rejected=select_panel_feedback(unavailable,ds,dq,1300001,250000,1000000,sw,qw);assert(!rejected.status&&!rejected.state&&sw==500000&&qw==1000000);
  PanelFeedbackCache cache;cache.available=true;cache.status=status;cache.state=state;cache.status_source=source;cache.state_source=source;cache.status_received_us=cache.state_received_us=1400000;
  ds.value=&status;dq.value=&state;ds.received_us=dq.received_us=1500000;
  const auto newer=select_panel_feedback(cache,ds,dq,1500001,250000,1000000,sw,qw);assert(newer.status==&status&&newer.state==&state&&sw==1500000&&qw==1500000);
  sw=qw=0;
  const auto separate=select_panel_feedback(unavailable,ds,dq,1750001,250000,1000000,sw,qw);assert(separate.status&&!separate.state&&sw==1500000&&qw==0);
}
