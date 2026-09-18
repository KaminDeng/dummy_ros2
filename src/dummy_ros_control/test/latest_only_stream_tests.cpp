#include <array>

#include "dummy_ros_control/device_transport.hpp"

namespace {
dummy_ros_control::CommandGenerationRecord record(std::uint64_t generation) {
  dummy_ros_control::CommandGenerationRecord value{};
  value.generation = generation;
  return value;
}
}  // namespace

int main() {
  using namespace dummy_ros_control;
  LatestOnlyCommandStream stream;
  stream.offer(record(1U));
  const auto first = stream.begin_send();
  if (!first || first->generation != 1U || !stream.has_in_flight()) return 1;
  stream.offer(record(2U));
  stream.offer(record(3U));
  if (!stream.has_pending() || stream.pending_generation() != 3U) return 2;
  if (stream.acknowledge_committed(2U)) return 3;
  if (stream.begin_send().has_value()) return 4;
  if (!stream.acknowledge_committed(1U) || stream.has_in_flight()) return 5;
  const auto latest = stream.begin_send();
  if (!latest || latest->generation != 3U) return 6;
  if (stream.acknowledge_committed(0U)) return 7;
  if (!stream.acknowledge_committed(3U)) return 8;
  if (stream.has_in_flight() || stream.has_pending() || stream.begin_send().has_value()) return 9;
  stream.offer(record(4U));
  if (!stream.begin_send().has_value()) return 10;
  stream.close_session();
  if (stream.has_in_flight() || stream.has_pending() || stream.begin_send().has_value()) return 11;
  return 0;
}
