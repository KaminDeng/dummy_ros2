#include "dummy_motion_client/dp_management_transport.hpp"
#include "dummy_ros_control/device_management_channel.hpp"
#include "dummy_ros_control/guarded_zenbedded_system.hpp"
#include <cstdio>
#include <memory>
#include <string>
#include <unistd.h>

template <typename Action>
std::string capture_stderr(Action action) {
  auto *output = tmpfile();
  if (!output) return {};
  const int original = dup(STDERR_FILENO);
  if (original < 0 || dup2(fileno(output), STDERR_FILENO) < 0) {
    fclose(output);
    return {};
  }
  action();
  fflush(stderr);
  dup2(original, STDERR_FILENO);
  close(original);
  rewind(output);
  char buffer[512]{};
  const auto count = fread(buffer, 1, sizeof(buffer), output);
  fclose(output);
  return std::string(buffer, count);
}
int main() {
  dummy_ros_control::TransportParameters parameters{"tcp/127.0.0.1:29999",{1},{2},2,1};
  dummy_motion_client::DpManagementTransport absent({parameters.endpoint,{1},{2},2,1},
      dummy_motion_client::DpManagementTransport::Connection::kDeviceWorker);
  std::vector<std::uint8_t> bytes(dp::protocol::dummy_robot_v2::DummyManageV2RequestCodec::kEncodedSize);
  dummy_motion_client::ManageRequest request;
  request.set_schema_major(2U);
  request.set_schema_minor(4U);
  request.set_request_id(42U);
  if (dp::protocol::dummy_robot_v2::DummyManageV2RequestCodec::encode(
          request, bytes.data(), bytes.size()) != dp::protocol::dummy_robot_v2::CodecResult::kOk)
    return 27;
  const auto absent_log = capture_stderr([&] { (void)absent.send(bytes); });
  if (absent_log.find("reason=not_ready request_id=42") == std::string::npos)
    return 24;
  dummy_ros_control::GuardedZenbeddedSystem system;
  auto provider=std::make_unique<dummy_ros_control::DeviceTransport>(system,parameters);
  auto channel=dummy_ros_control::DeviceTransport::management_channel(parameters);
  if(!channel) return 1;
  if (capture_stderr([&] { (void)absent.send(bytes); }).find("reason=not_ready") == std::string::npos)
    return 25;
  channel->ready=true; channel->observation.state_received_us=100; channel->observation.status_received_us=100;
  channel->observation.rejected_ack_boot_epoch=7U;
  channel->observation.rejected_ack_session_id=8U;
  channel->observation.rejected_ack_reference_epoch=9U;
  channel->observation.rejected_ack_motion_epoch=10U;
  channel->observation.rejected_ack_sequence=4145U;
  channel->observation.rejected_ack_received_us=101U;
  dummy_motion_client::DpManagementTransport facade({parameters.endpoint,{1},{2},2,1},
      dummy_motion_client::DpManagementTransport::Connection::kDeviceWorker);
  if(!facade.start() || !facade.ready() || !facade.observe()) return 2;
  const auto rejected=facade.observe();
  if(!rejected || rejected->rejected_ack_boot_epoch!=7U ||
     rejected->rejected_ack_session_id!=8U ||
     rejected->rejected_ack_reference_epoch!=9U ||
     rejected->rejected_ack_motion_epoch!=10U ||
     rejected->rejected_ack_sequence!=4145U ||
     rejected->rejected_ack_received_us!=101U) return 23;
  channel->hold_drain={7U,8U,12785U,9U,11U,12784U};
  channel->hold_drain.disposition=1U; channel->hold_drain.quiescent=true;
  channel->hold_drain.observed_us=100U; channel->hold_drain.max_state_age_us=250000U;
  channel->hold_drain.position[0]=0.2; channel->hold_drain.tolerance[0]=0.003;
  const auto worker=facade.hold_drain_snapshot();
  if(!worker || worker->processed_generation!=12785U || worker->committed_seq!=12784U ||
     worker->disposition!=1U || !worker->quiescent || worker->observed_us!=100U ||
     worker->max_state_age_us!=250000U || worker->position[0]!=0.2 ||
     worker->tolerance[0]!=0.003) return 20;
  channel->hold_drain={};
  const auto pending=facade.hold_drain_snapshot();
  // A newly pending group revokes the prior proof, rather than looking like no new information.
  if(!pending || pending->quiescent || pending->processed_generation!=0U) return 21;
  if(facade.raw_evidence_failure()!=0U) return 12;
  channel->raw_evidence_failure.store(2U,std::memory_order_release);
  if(facade.raw_evidence_failure()!=2U) return 13;
  if(facade.raw_evidence_capture_enabled()) return 14;
  channel->raw_evidence_capture_enabled.store(true,std::memory_order_release);
  if(!facade.raw_evidence_capture_enabled()) return 15;
  const auto flushed=facade.raw_terminal_flushed();
  if(flushed.boot_epoch!=0U || flushed.session_id!=0U || flushed.run_id!=0U) return 16;
  channel->publish_raw_evidence_terminal_flushed(7U,8U,9U);
  const auto terminal=facade.raw_terminal_flushed();
  if(terminal.boot_epoch!=7U || terminal.session_id!=8U || terminal.run_id!=9U) return 17;
  channel->publish_raw_evidence_terminal_flushed(
    {7U,8U,9U,0U,1U,7U,0U,0U,false,false,false,true});
  const auto clean_terminal=facade.raw_terminal_flushed();
  if(clean_terminal.final_state!=7U || clean_terminal.attempt_kind!=1U ||
     clean_terminal.parent_run_id!=0U || clean_terminal.first_failure_source!=0U ||
     clean_terminal.first_failure_axis_mask!=0U || clean_terminal.evidence_lost ||
     clean_terminal.motion_complete || clean_terminal.release_committed ||
     !clean_terminal.chain_valid) return 22;
  if(!dummy_motion_client::raw_terminal_matches(terminal,7U,8U,9U) ||
     dummy_motion_client::raw_terminal_matches(terminal,6U,8U,9U) ||
     dummy_motion_client::raw_terminal_matches(terminal,7U,7U,9U) ||
     dummy_motion_client::raw_terminal_matches(terminal,7U,8U,8U)) return 19;
  if(!dummy_motion_client::observation_is_fresh(*facade.observe(),99) ||
     dummy_motion_client::observation_is_fresh(*facade.observe(),300100) ||
     !dummy_motion_client::observation_is_fresh(*facade.observe(),300100,1000000U) ||
     dummy_motion_client::observation_is_fresh(*facade.observe(),1000101,1000000U)) return 7;
  {
    std::lock_guard<std::mutex> lock(channel->mutex);
    const auto cached=facade.observe();
    if(!cached || cached->state_received_us!=100) return 3;
  }
  if (capture_stderr([&] {
        std::lock_guard<std::mutex> lock(channel->mutex);
        (void)facade.send(bytes);
      }).find("reason=lock_busy") == std::string::npos) return 26;
  auto first=facade.send(bytes);
  if(!first || capture_stderr([&] { (void)facade.send(bytes); }).find("reason=queued") == std::string::npos)
    return 4;
  first.reset(); if(!facade.send(bytes)) return 5;
  auto wrong=parameters; wrong.profile_id=3;
  if(dummy_ros_control::DeviceTransport::management_channel(wrong)) return 6;
  dummy_motion_client::WireManagementClient client(facade);
  dummy_motion_client::ManageRequest query;
  query.set_operation(dummy_motion_client::wire_value(dummy_motion_client::ManageOperation::kQuery));
  if(!client.submit(query,100)) return 8;
  auto cancel=query;
  cancel.set_operation(dummy_motion_client::wire_value(dummy_motion_client::ManageOperation::kCancel));
  if(client.submit(cancel,101)!=0U || !channel->queued) return 9;
  first.reset();
  channel->queued.reset();
  dummy_motion_client::WireManagementClient busy_client(facade);
  {
    std::lock_guard<std::mutex> lock(channel->mutex);
    if(!busy_client.submit(query,200)) return 10;
  }
  if(busy_client.poll(1) || !channel->queued) return 11;
  channel->queued.reset();
  channel->observe_request_sequence(2100000000U);
  channel->observe_request_sequence(2183397839U);
  dummy_motion_client::WireManagementClient resumed_client(facade);
  if (!resumed_client.submit(query, 201U) || !channel->queued) return 28;
  dummy_motion_client::ManageRequest resumed_request;
  if (dp::protocol::dummy_robot_v2::DummyManageV2RequestCodec::decode(
          channel->queued->request.data(), channel->queued->request.size(),
          &resumed_request) != dp::protocol::dummy_robot_v2::CodecResult::kOk ||
      resumed_request.request_seq() != 2183397840U) return 29;
  channel->observe_request_sequence(2183397839U);
  if (channel->reserve_request_sequence() != 2183397841U) return 31;
  channel->observe_request_sequence(UINT32_MAX);
  if (channel->reserve_request_sequence() != 1U) return 30;
  channel->observe_request_sequence(UINT32_MAX);
  if (channel->reserve_request_sequence() != 2U) return 32;
  channel->observe_request_sequence(0x80000002U);
  if (channel->reserve_request_sequence() != 3U) return 33;
  channel->ready=false;
  provider.reset();
  channel.reset();
  if(!facade.raw_evidence_capture_enabled() || facade.raw_evidence_failure()!=2U) return 18;
}
