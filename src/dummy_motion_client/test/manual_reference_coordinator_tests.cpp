#include "dummy_motion_client/manual_reference_coordinator.hpp"
#include "dummy_motion_client/dp_management_transport.hpp"
#include <cassert>
#include <vector>
using namespace dummy_motion_client;
struct ManualClient final: ManagementClient {
  unsigned requests=0;
  bool complete=false,denied=false;
  std::vector<ManageRequest> sent;
  std::uint64_t submit(const ManageRequest& r,std::uint64_t) override {sent.push_back(r);return ++requests;}
  std::uint64_t submit_permit(const ManageRequest& r,const PermitBinding&,std::uint64_t at) override {return submit(r,at);}
  std::optional<ManageReply> poll(std::uint64_t id) override {
    if(!complete)return {};
    ManageReply r;r.request_id=id;r.operation_id=id;r.result_valid=true;
    r.release_committed=!sent.empty() && sent.back().operation()==10;
    r.succeeded=!denied;r.rejected=denied;r.transaction_phase=denied?4:3;r.result_code=denied?10:0;return r;
  }
};
// 使用真实20ms反馈与设备finalSENT+3秒窗口，不能把Host估时当完成。
void dynamic_rest_case(int kind) {
  const bool long_path=kind==1 || kind==2;
  ManualClient client;ManageRequest identity;identity.set_expected_session_id(3);
  std::array<std::uint8_t,32> digest{};digest[0]=1;
  BootstrapCoordinator boot(client,identity,digest,1000000);
  ManualReferenceOptions options;options.deployment_sha256=digest;
  constexpr double rad=3.14159265358979323846/180.;
  options.manual_limit_position={0,-73*rad,180*rad,0,0,0};
  options.rest_position={0,-60*rad,163.97260273972603*rad,0,0,0};
  options.position_min[1]=-73*rad;options.position_max[2]=180*rad;
  ManualReferenceCoordinator manual(boot,options);
  BootstrapSnapshot snap{};snap.boot_epoch=1;snap.profile_id=1;snap.raw_ready=true;
  snap.motion_supported=true;snap.configured_mask=63;snap.freshness_ticket=1;
  snap.reference_epoch=11;snap.reference_kind=1;snap.core_state=snap.state_core_state=2;snap.management_epoch=1;
  ConfirmedManualStart confirm{1,1,digest,false,true};
  std::uint64_t now=100000;std::uint32_t seq=1;
  Position position=long_path?options.manual_limit_position:options.rest_position;
  if(kind==2)position[2]=120*rad; // 仅J2退让，J3在有效域保持120度。
  if(kind==3) {position[1]=-40*rad;position[2]=120*rad;}
  const double initial_j3=position[2];
  position[0]=0.2; // REF冻结非退让轴，不能假设J1也回零。
  assert(manual.start(confirm,snap,now));client.complete=true;
  auto step=[&] {now+=20000;++seq;boot.observe(snap,now);manual.observe(snap,position,seq,now);boot.tick(now);manual.tick(now);};
  snap.control_mode=1;++snap.management_epoch;step();
  snap.session_id=3;++snap.management_epoch;step();
  while(client.sent.back().operation()!=17 && manual.active())step();
  assert(client.sent.back().operation()==17);
  ++snap.motion_epoch;++snap.management_epoch;step();const auto anchor=now;
  // 长轨迹提前出现容差内反馈，Host仍必须等待五次轨迹完整估时。
  if(kind!=3)position[1]=options.rest_position[1];
  position[2]=(kind==2 || kind==3)?initial_j3:options.rest_position[2];
  while(now-anchor<(long_path?(kind==2?11000000U:14000000U):300000U)) {
    step();if(long_path)assert(client.sent.back().operation()==17);
  }
  while(client.sent.back().operation()==17 && now-anchor<18000000U)step();
  assert(client.sent.back().operation()==18);
  const auto elapsed=now-anchor;
  if(long_path)assert(elapsed>=(kind==2?11000000U:14000000U) && elapsed<(kind==2?14000000U:17000000U));
  else assert(elapsed<3000000U);
  ++snap.management_epoch;step();assert(client.sent.back().operation()==19);
  ++snap.management_epoch;step();assert(client.sent.back().operation()==5);
  for(const auto& request:client.sent)assert(request.operation()!=16);
}
int main() {
  for(int kind=0;kind<4;++kind)dynamic_rest_case(kind);
  dummy_wire::DummyDeviceStatusV2 released_status;
  dummy_wire::DummyJointStateV2 released_state;
  released_status.set_boot_epoch(7);released_state.set_boot_epoch(7);
  released_status.set_reference_epoch(9);released_state.set_reference_epoch(9);
  released_status.set_core_state(2);released_state.set_core_state(2);
  released_status.set_release_committed(true);
  assert(retained_reference_released_idle_observation(released_status,released_state,9));
  assert(!released_idle_status_confirmed(released_status,released_state));
  assert(!retained_reference_released_idle_observation(released_status,released_state,8));
  assert(!retained_reference_released_idle_observation(released_status,released_state,0));
  released_state.set_management_epoch(1);
  assert(!retained_reference_released_idle_observation(released_status,released_state,9));
  released_state.set_management_epoch(0);released_state.set_boot_epoch(8);
  assert(!retained_reference_released_idle_observation(released_status,released_state,9));

  ManualClient client; ManageRequest identity; identity.set_expected_session_id(3);
  std::array<std::uint8_t,32> digest{};digest[0]=1;
  BootstrapCoordinator bootstrap(client,identity,digest,1000000);
  ManualReferenceOptions options; options.deployment_sha256=digest;
  ManualReferenceCoordinator manual(bootstrap,options);
  ConfirmedManualStart confirm;confirm.boot_epoch=1;confirm.state_sequence=2;confirm.deployment_sha256=digest;
  BootstrapSnapshot snapshot;snapshot.boot_epoch=1;snapshot.profile_id=1;snapshot.raw_ready=true;
  snapshot.motion_supported=true;snapshot.configured_mask=63;snapshot.freshness_ticket=1;
  assert(!manual.start(confirm,snapshot,100));assert(client.requests==0);
  confirm.operator_confirmed_double_limits=true;
  assert(manual.start(confirm,snapshot,100));assert(client.requests==1);
  assert(!manual.start(confirm,snapshot,101));assert(!manual.authorization());
  snapshot.boot_epoch=2;manual.observe_raw(snapshot,{},3,102);manual.tick(102);
  assert(manual.phase()==ManualReferencePhase::kFault);
  for(bool permit_failure : {false,true}) {
  snapshot={};snapshot.boot_epoch=1;snapshot.profile_id=1;snapshot.raw_ready=true;
  snapshot.motion_supported=true;snapshot.configured_mask=63;snapshot.freshness_ticket=1;
  ManualClient chain_client;BootstrapCoordinator chain_boot(chain_client,identity,digest,1000000);
  options.stationary_window_us=10;options.minimum_samples=2;
  options.max_feedback_age_us=100000;
  ManualReferenceCoordinator chain(chain_boot,options);
  snapshot.boot_epoch=1;snapshot.management_epoch=1;
  std::uint32_t seq=2;std::uint64_t now=100;
  assert(chain.start(confirm,snapshot,now));
  auto observe=[&](bool raw,Position p=Position{}) {
    now+=10000;++seq;chain_boot.observe(snapshot,now);
    if(raw)chain.observe_raw(snapshot,p,seq,now);else chain.observe(snapshot,p,seq,now);
    chain_boot.tick(now);chain.tick(now);
  };
  // QUEUED/无终态和旧管理水位均不能推进。
  observe(true);assert(chain_client.requests==1);
  chain_client.complete=true;observe(true);assert(chain_client.requests==1);
  snapshot.control_mode=1;++snapshot.management_epoch;observe(true);
  assert(chain_client.sent.back().operation()==2);
  snapshot.session_id=3;++snapshot.management_epoch;observe(true);
  observe(true);
  const auto raw_request_count=chain_client.requests;
  chain.observe_raw(snapshot,{},seq,now+1);chain.tick(now+1);
  chain.observe_raw(snapshot,{},seq,now+2);chain.tick(now+2);
  assert(chain_client.requests==raw_request_count);
  observe(true);
  assert(chain_client.sent.back().operation()==16);
  assert(chain_client.sent.back().operator_confirmed());
  assert(chain.qualified_reference_epoch()==0);
  snapshot.reference_epoch=1;snapshot.reference_kind=1;++snapshot.management_epoch;observe(false);
  // 第一次calibrated State在16完成前不计入新窗。
  observe(false);observe(false);
  assert(chain_client.sent.back().operation()==17);
  snapshot.motion_epoch=1;snapshot.core_state=2;snapshot.state_core_state=2;
  ++snapshot.management_epoch;observe(false);
  const auto before=chain_client.requests;
  observe(false);
  assert(chain_client.requests==before);
  observe(false);assert(chain_client.sent.back().operation()==18);
  // 明确PolicyDenied必须等下一独立State，且不重发17。
  chain_client.denied=true;chain_boot.tick(now+1);chain.tick(now+1);
  const auto denied_count=chain_client.requests;chain.tick(now+2);
  assert(chain_client.requests==denied_count);
  chain_client.denied=false;observe(false);assert(chain_client.sent.back().operation()==18);
  ++snapshot.management_epoch;observe(false);assert(chain_client.sent.back().operation()==19);
  ++snapshot.management_epoch;observe(false);assert(chain_client.sent.back().operation()==5);
  assert(chain.qualified_reference_epoch()==snapshot.reference_epoch);
  if(permit_failure) {
    chain_client.denied=true;chain_boot.tick(now+1);chain.tick(now+1);
    assert(chain.phase()==ManualReferencePhase::kFault);
    assert(chain.qualified_reference_epoch()==snapshot.reference_epoch);
    continue;
  }
  snapshot.real_motion_permitted=true;snapshot.permit_remaining_us=1000000;
  ++snapshot.management_epoch;observe(false);
  assert(chain.phase()==ManualReferencePhase::kReady);assert(chain.authorization());
  for(const auto& r:chain_client.sent)assert(r.operation()!=3);
  }


  // 同boot已资格参考只能显式复用；复用绝不能重采16或积累raw窗口。
  for (int reuse_case : {0,1,2,3}) {
    ManualClient reuse_client;
    BootstrapCoordinator reuse_boot(reuse_client,identity,digest,1000000);
    ManualReferenceOptions reuse_options=options;
    reuse_options.stationary_window_us=10;reuse_options.minimum_samples=2;
    reuse_options.max_feedback_age_us=100000;
    reuse_options.manual_limit_position.fill(1.0);
    ManualReferenceCoordinator reuse(reuse_boot,reuse_options);
    BootstrapSnapshot retained{};retained.boot_epoch=1;retained.profile_id=1;
    retained.raw_ready=true;retained.motion_supported=true;retained.configured_mask=63;
    retained.freshness_ticket=1;retained.management_epoch=1;
    retained.reference_epoch=11;retained.reference_kind=1;
    retained.core_state=2;retained.state_core_state=2;
    ConfirmedManualStart explicit_reuse=confirm;
    assert(!reuse.start(explicit_reuse,retained,100));
    explicit_reuse.reuse_qualified_reference=true;
    explicit_reuse.operator_confirmed_double_limits=false;
    assert(reuse.start(explicit_reuse,retained,100));
    assert(reuse.qualified_reference_epoch()==11);
    std::uint64_t at=100;std::uint32_t seq=2;
    auto observed=[&](bool raw=false) {
      at+=10000;++seq;reuse_boot.observe(retained,at);
      Position calibrated{};
      calibrated.fill(0.2);
      if(raw)reuse.observe_raw(retained,{},seq,at);else reuse.observe(retained,calibrated,seq,at);
      reuse_boot.tick(at);reuse.tick(at);
    };
    observed();assert(reuse_client.requests==1);
    reuse_client.complete=true;retained.control_mode=1;++retained.management_epoch;observed();
    assert(reuse_client.sent.back().operation()==2);
    retained.session_id=3;++retained.management_epoch;observed();
    if(reuse_case==1 || reuse_case==3) {
      if(reuse_case==1)retained.reference_epoch=12;else retained.reference_kind=0;
      observed();
      assert(reuse.phase()==ManualReferencePhase::kFault);continue;
    }
    if(reuse_case==2) {
      // 19再核完成前中途退出，既有资格参考仍必须保留并核同值。
      assert(reuse.qualified_reference_epoch()==11);
      reuse.request_release(at+1);
      assert(reuse.phase()==ManualReferencePhase::kReleasing);
      assert(reuse_client.sent.back().operation()==10);
      retained.session_id=0;++retained.management_epoch;observed();
      assert(reuse.phase()==ManualReferencePhase::kReleased);
      assert(reuse.qualified_reference_epoch()==11);
      continue;
    }
    // raw样本和重复已标定样本均不能替代Grant后的独立静止窗。
    observed(true);observed(true);assert(reuse_client.requests==2);
    observed();reuse.observe(retained,{},seq,at+1);reuse.tick(at+1);
    assert(reuse_client.requests==2);observed();
    assert(reuse_client.sent.back().operation()==17);
    retained.motion_epoch=1;++retained.management_epoch;observed();
    for(int i=0;i<5;++i)observed();
    assert(reuse_client.sent.back().operation()==18);
    ++retained.management_epoch;observed();assert(reuse_client.sent.back().operation()==19);
    ++retained.management_epoch;observed();assert(reuse_client.sent.back().operation()==5);
    retained.real_motion_permitted=true;retained.permit_remaining_us=1000000;
    ++retained.management_epoch;observed();
    assert(reuse.phase()==ManualReferencePhase::kReady);
    assert(reuse.qualified_reference_epoch()==11);
    assert(reuse.authorization());
    for(const auto& request:reuse_client.sent)assert(request.operation()!=16 && request.operation()!=3);
  }
  // 显式标记不放宽session/reference kind/双Idle/permit门槛。
  for(int bad=0;bad<7;++bad) {
    ManualClient c;BootstrapCoordinator b(c,identity,digest,1000000);
    ManualReferenceCoordinator m(b,options);
    BootstrapSnapshot r{};r.boot_epoch=1;r.profile_id=1;r.raw_ready=true;
    r.motion_supported=true;r.configured_mask=63;r.freshness_ticket=1;
    r.reference_epoch=11;r.reference_kind=1;r.core_state=2;r.state_core_state=2;
    ConfirmedManualStart ok=confirm;ok.reuse_qualified_reference=true;
    if(bad==0)r.session_id=3;
    if(bad==1)r.reference_epoch=0;
    if(bad==2)r.reference_kind=0;
    if(bad==3)r.core_state=1;
    if(bad==4)r.state_core_state=4;
    if(bad==5)r.real_motion_permitted=true;
    if(bad==6)r.permit_remaining_us=1;
    assert(!m.start(ok,r,100));assert(c.requests==0);
  }
}
