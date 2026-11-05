/** @file protocol_r11_root_owner_tests.cpp @brief 单一Host SDK
 * owner显式Source及真实方向存储合同。 */
#include "dummy_ref/root_transport.hpp"
#include <cassert>
int main() {
  dummy_ref::root::HostTransport owner;
  dp::comm::SourceId source{};
  dp::comm::DeviceId remote{};
  source.device.bytes[0] = 2;
  source.boot_id = UINT64_C(0xf123456789abcdef);
  source.context_id = 7;
  remote.bytes[0] = 9;
  assert(owner.configure({}, remote) == dp::comm::Result::kInvalidArgument);
  assert(owner.session.source.boot_id == 0 &&
         owner.identity.state.local_count == 0);
  assert(owner.configure(source, remote) == dp::comm::Result::kOk);
  assert(owner.session.source == source);
  assert(owner.identity.state.local_count == 3 &&
         owner.identity.state.desired_count == 5);
  assert(owner.session.arena.completion_count ==
         dp::comm::detail::kStorageMaxPendingRequests);
  assert(owner.session.arena.owned_size == 0);
  assert(owner.session.arena.request_size >=
         sizeof(dp::msg::dummy::DeviceStatus));
  assert(owner.session.arena.response_size >=
         sizeof(dp::msg::dummy::DeviceStatus));
  assert(owner.session.arena.completion_stride >=
         sizeof(dp::msg::system::DescribeResponse));
  assert(owner.configure(source, remote) == dp::comm::Result::kBusy);
}
