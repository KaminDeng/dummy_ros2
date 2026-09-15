#pragma once
#include "dummy_motion_client/action_gate.hpp"
#include <stdexcept>
namespace action_test {
using namespace dummy_motion_client;
inline void require(bool value) { if (!value) throw std::runtime_error("action lifecycle requirement failed"); }
struct Client : ManagementClient {
  std::vector<ManageRequest> requests;
  std::uint64_t submit(const ManageRequest &request,std::uint64_t) override {
    requests.push_back(request); return requests.size();
  }
  std::optional<ManageReply> poll(std::uint64_t) override { return {}; }
};
struct Port : ActionPort {
  unsigned primes=0, starts=0;
  void prime(const Position &,std::uint32_t,std::uint64_t) override { ++primes; }
  void start(const Trajectory &,std::uint64_t) override { ++starts; }
  void disable() override {}
  void cancel() override {}
};
struct Harness {
  Client client; Port port;
  ActionGate gate{client,port,{{"Joint1"},{-1},{1},{1},{1},{0.1},{0.1}}};
  std::uint32_t epoch=0;
  ManageReply response(bool success=false,std::uint32_t accepted_seq=1) const {
    ManageReply reply; reply.request_id=client.requests.size(); reply.operation_id=100+reply.request_id;
    reply.accepted=true; reply.snapshot_valid=true; reply.motion_epoch=epoch;
    reply.hold_available=true; reply.result_valid=success; reply.succeeded=success;
    reply.hold_ready=success; reply.accepted_seq=accepted_seq; return reply;
  }
  void enable() {
    Authorization auth; auth.session=auth.reference=auth.permit=true;
    auth.identity.set_expected_session_id(1); require(gate.enable(auth,1));
    ++epoch; gate.observe(response(),2); require(gate.record_update(gate.generation(),epoch,1,Position{}));
    gate.observe(response(true),3); require(gate.ready());
  }
  void begin() {
    const Trajectory trajectory{{"Joint1"},{{{0.2},{0},{},1}},{},{},0};
    require(gate.submit(trajectory,10)); ++epoch; gate.observe(response(),11);
    require(gate.record_update(gate.generation(),epoch,1,Position{})); gate.observe(response(true),12);
  }
  void complete() {
    Position end{}; end[0]=0.2; require(gate.record_update(gate.generation(),epoch,17,end));
    gate.inner_result(gate.generation(),Terminal::kSucceeded,"",13);
    require(gate.terminal()==Terminal::kNone);
    require(!gate.record_update(gate.generation(),epoch,18,end));
    gate.observe(response(true,17),14);
    require(client.requests.back().terminal_producer_seq()==17);
    ++epoch; gate.observe(response(),15); require(gate.terminal()==Terminal::kNone);
    require(gate.record_update(gate.generation(),epoch,1,Position{}));
    gate.observe(response(true),16); require(gate.ready());
  }
};
}  // namespace action_test
