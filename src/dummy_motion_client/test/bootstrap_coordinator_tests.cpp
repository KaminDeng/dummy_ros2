#include "dummy_motion_client/bootstrap_coordinator.hpp"

#include <cassert>
#include <vector>

struct Client final : dummy_motion_client::ManagementClient {
  std::vector<dummy_motion_client::ManageRequest> requests;
  std::uint64_t submit(const dummy_motion_client::ManageRequest &request, std::uint64_t) override {
    requests.push_back(request); return requests.size();
  }
  std::optional<dummy_motion_client::ManageReply> poll(std::uint64_t id) override {
    dummy_motion_client::ManageReply reply{}; reply.request_id=id; reply.result_valid=true;
    reply.succeeded=true; reply.operation_id=id; return reply;
  }
};

int main() {
  using namespace dummy_motion_client;
  Client client;
  ManageRequest identity{};
  identity.set_expected_session_id(42U);
  std::uint8_t device[16]{1U}, owner[16]{2U};
  identity.set_device_id(device); identity.set_client_instance_id(owner);
  std::array<std::uint8_t, 32> permit{}; permit[0]=1U;
  BootstrapCoordinator bootstrap(client, identity, permit, 1000000U);
  BootstrapSnapshot state{7U,0U,0U,0U,0U,9U,2U,1U,true,true};
  assert(bootstrap.start(state, 1U));
  const std::uint16_t operations[]{2U,3U,4U,14U};
  for (unsigned index=0; index<4; ++index) {
    assert(client.requests.back().operation()==operations[index]);
    bootstrap.tick(2U+index);
    state.management_epoch=index+1U;
    if (index>=1U) state.session_id=42U;
    if (index>=2U) state.reference_epoch=1U;
    bootstrap.observe(state);
    bootstrap.tick(10U+index);
  }
  assert(bootstrap.ready());
  const auto authorization=bootstrap.authorization();
  assert(authorization && authorization->identity.expected_session_id()==42U &&
         authorization->identity.expected_reference_epoch()==1U);
  return 0;
}
