#include <cstdint>

#include "dp_comm/dp_comm.hpp"
#include "dp_dummy_robot_v2_dp_comm.hpp"
#include "dummy_ros_control/dp_comm_worker.hpp"

namespace {
using Message = dp::protocol::dummy_robot_v2::DummyJointCommandV2;
struct Received { unsigned count = 0; Message message{}; };
void receive(dp::comm::BorrowedMessage<Message> borrowed, void *context) {
  auto *received = static_cast<Received *>(context);
  if (borrowed.copy_to(&received->message) == dp::comm::Result::kOk) ++received->count;
}
}  // namespace

int main() {
  dp::comm::StaticStorageConfig config{1U, 2U, 2U, 0U, 6U,
      dp::protocol::dummy_robot_v2::DummyDeviceStatusV2Codec::kEncodedSize, 96U};
  dp::comm::StaticStorage storage(config);
  auto context_result = dp::comm::Context::init(storage, dp::comm::MemoryMode::kStaticOnly);
  if (!context_result.has_value()) return 1;
  auto context = std::move(context_result).take_value();
  dp::comm::NodeOptions options{};
  options.product_name = "dummy_robot";
  options.device_id.bytes[0] = 1U;
  options.node_name = "worker_test";
  auto node_result = context.create_node(options);
  if (!node_result.has_value()) return 2;
  auto node = std::move(node_result).take_value();
  auto publisher_result = node.create_publisher<Message>("command_v2", {});
  Received received{};
  auto subscription_result = node.create_subscription<Message>("command_v2", {}, receive, &received);
  if (!publisher_result.has_value() || !subscription_result.has_value()) return 3;
  auto publisher = std::move(publisher_result).take_value();
  auto subscription = std::move(subscription_result).take_value();
  dummy_ros_control::GuardedZenbeddedSystem system;
  using State = dp::protocol::dummy_robot_v2::DummyJointStateV2;
  const dummy_ros_control::StateExpectation expectation{9U, 7U, 100U, true, 0x3FU, 1000U, 1100U, 150U};
  dummy_ros_control::DpCommStateReceiver state_receiver(&system, expectation);
  auto state_publisher_result = node.create_publisher<State>("state_v2", {});
  auto state_subscription_result = node.create_subscription<State>(
      "state_v2", {}, dummy_ros_control::DpCommStateReceiver::receive, &state_receiver);
  if (!state_publisher_result.has_value() || !state_subscription_result.has_value()) return 9;
  auto state_publisher = std::move(state_publisher_result).take_value();
  auto state_subscription = std::move(state_subscription_result).take_value();
  if (node.activate() != dp::comm::Result::kOk) return 4;
  const dummy_ros_control::CommandIdentity identity{
      9007199254740993ULL, 9007199254740995ULL, 3U, 4U, 9007199254740997ULL};
  const std::array<double, 6> position{0.1, 0.2, 0.7, 0.4, 0.5, 9.42477796076938};
  system.submit_control_update(identity, position);
  if (!system.transfer_control_update()) return 5;
  dummy_ros_control::DpCommCommandWorker worker(&system, &publisher, 7U, 0x3FU);
  if (worker.publish_one() != dp::comm::Result::kOk ||
      worker.publish_one() != dp::comm::Result::kNotReady) return 6;
  State feedback;
  feedback.set_schema_major(2U);
  feedback.set_schema_minor(4U);
  feedback.set_boot_epoch(9U);
  feedback.set_profile_id(7U);
  feedback.set_state_seq(101U);
  feedback.set_produced_at_us(1001U);
  feedback.set_configured_mask(0x3FU);
  feedback.set_position_valid_mask(0x3FU);
  const double state_position[6]{position[0], position[1], position[2],
                                 position[3], position[4], position[5]};
  feedback.set_position_rad(state_position);
  if (state_publisher.publish(feedback) != dp::comm::Result::kOk) return 10;
  dp::comm::StaticExecutor executor;
  if (executor.add_node(node) != dp::comm::Result::kOk || executor.spin_some() != dp::comm::Result::kOk)
    return 7;
  dummy_ros_control::JointStateSnapshot snapshot{};
  return received.count == 1U && received.message.boot_epoch() == identity.boot &&
                 received.message.session_id() == identity.session &&
                 received.message.freshness_ticket() == identity.ticket &&
                 received.message.position_rad()[5] == position[5] &&
                 state_receiver.accepted() && system.copy_state(&snapshot) &&
                 snapshot.position[5] == position[5]
             ? 0
             : 11;
}
