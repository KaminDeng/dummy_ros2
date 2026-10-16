#include "action_test_support.hpp"
#include <iostream>
#include <algorithm>
#include <limits>
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
