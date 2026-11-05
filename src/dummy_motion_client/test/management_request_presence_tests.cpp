/** @file management_request_presence_tests.cpp
 * @brief 验证真实Owned的缺失/显式零和许可摘要绑定；只运行内存transport，不授予设备许可。
 */
#include "typed_management_fixture.hpp"
#include "dummy_ros_control/message_presence.hpp"
#include <algorithm>
#include <cassert>
using namespace dummy_motion_client;
int main(){
  typed_management_fixture::Transport transport;
  WireManagementClient client(transport);
  ManageRequest missing;
  assert(client.submit(missing,123U)==0U && transport.requests.empty());
  auto request=owned_fixture::request();request.expected_boot_epoch()=0U;
  assert(client.submit(request,123U)!=0U && transport.requests.size()==1U);
  assert(transport.requests.back().expected_boot_epoch() && *transport.requests.back().expected_boot_epoch()==0U);
  typed_management_fixture::Transport permit_transport;
  WireManagementClient permit_client(permit_transport);
  PermitBinding binding{};binding.boot_epoch=7U;binding.reference_epoch=3U;binding.profile_id=101U;
  binding.firmware_sha256[0]=1U;binding.contract_sha256[0]=2U;binding.model_sha256[0]=3U;binding.profile_sha256[0]=4U;
  std::array<std::uint8_t,16> owner{};owner[0]=0x55U;
  request.client_instance_id().emplace().assign(owner.data(),owner.size());
  request.operation()=wire_value(ManageOperation::INSTALL_PERMIT);request.expected_boot_epoch()=7U;request.expected_session_id()=9U;
  request.expected_reference_epoch()=3U;request.expected_profile_id()=101U;
  assert(permit_client.submit_permit(request,binding,456U)==0U);
  std::array<std::uint8_t,32> policy{};policy[0]=4U;request.compiled_policy_sha256().emplace().assign(policy.data(),policy.size());
  assert(permit_client.submit_permit(request,binding,456U)==0U);
  request.operator_confirmed()=true;assert(permit_client.submit_permit(request,binding,456U)==0U);
  request.permit_remaining_us()=1000000U;assert(permit_client.submit_permit(request,binding,456U)!=0U);
  const auto& sent=permit_transport.requests.back();std::array<std::uint8_t,32> expected{};
  assert(trusted_bench_permit_digest(binding,owner,9U,*sent.request_seq(),&expected));
  assert(std::equal(expected.begin(),expected.end(),sent.permit_sha256()->begin()));assert(*sent.request_seq()==1U);
  binding.boot_epoch=8U;typed_management_fixture::Transport stale_transport;WireManagementClient stale_client(stale_transport);
  assert(stale_client.submit_permit(request,binding,456U)==0U && stale_transport.requests.empty());
}
