#pragma once
#include "dummy_motion_client/device_profile.hpp"
#include "dummy_motion_client/manual_reference_coordinator.hpp"
#include "dummy_ros_control/device_transport.hpp"
namespace dummy_motion_client {
// 只读绑定产物描述；没有运行许可，也不构成发布资格。
struct ManualPanelDeployment {
  DeviceProfile profile;
  ManualReferenceOptions reference;
  dummy_ros_control::TransportParameters transport;
  std::array<double,6> direction{},gear_ratio{};
  std::string deployment_root,endpoint,device_id,client_id;
  std::uint32_t profile_id=0;
  std::uint8_t joint_mask=0;
  std::uint32_t control_period_us=0,state_period_us=0;
  std::uint64_t max_state_age_us=250000,stop_confirmation_timeout_us=0;
};
ManualPanelDeployment load_manual_panel_deployment(const std::string& deployment_root,
    const std::string& expected_manifest_sha256,const std::string& sdk_bundle_root);
}
