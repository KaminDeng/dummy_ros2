#include <cstdint>
#include <cstdio>
#include <utility>

#include "dp_comm/dp_comm.hpp"

namespace {
struct Request { std::uint32_t value; };
struct Response { std::uint32_t value; };
struct Service { using Request = ::Request; using Response = ::Response; };
struct State { std::uint32_t calls = 0U; std::uint32_t responses = 0U; };

dp::comm::Result serve(const Request &request, Response *response,
                       const dp::comm::RequestInfo &, void *context) {
  auto *state = static_cast<State *>(context);
  ++state->calls;
  response->value = request.value;
  return dp::comm::Result::kOk;
}

void receive(dp::comm::Result result, const Response *response,
             const dp::comm::ResponseInfo &, void *context) {
  if (result == dp::comm::Result::kOk && response && response->value == 2U) {
    ++static_cast<State *>(context)->responses;
  }
}
}  // namespace

int main() {
  dp::comm::StaticStorageConfig config{1U, 0U, 0U, 0U, 4U, sizeof(Request), 96U};
  config.services = 1U;
  config.clients = 1U;
  config.pending_requests = 1U;
  dp::comm::StaticStorage storage(config);
  auto context_result = dp::comm::Context::init(storage, dp::comm::MemoryMode::kStaticOnly);
  if (!context_result.has_value()) return 1;
  auto context = std::move(context_result).take_value();
  dp::comm::NodeOptions options{};
  options.product_name = "dummy_robot";
  options.device_id.bytes[0] = 1U;
  options.node_name = "destructor_probe";
  auto node_result = context.create_node(options);
  if (!node_result.has_value()) {
    std::fprintf(stderr, "create_node result=%u\n", static_cast<unsigned>(node_result.result()));
    return 2;
  }
  auto node = std::move(node_result).take_value();
  State state{};
  auto service_result = node.create_service<Service>("manage", serve, &state);
  auto client_result = node.create_client<Service>("manage");
  if (!service_result.has_value() || !client_result.has_value()) return 3;
  auto service = std::move(service_result).take_value();
  auto client = std::move(client_result).take_value();
  if (node.activate() != dp::comm::Result::kOk) return 4;
  {
    auto first_result = client.async_request(
        Request{1U}, receive, &state, dp::comm::Duration::from_milliseconds(100U), 77U);
    if (!first_result.has_value()) return 5;
    auto first = std::move(first_result).take_value();
    if (!first.pending()) return 6;
  }
  auto second_result = client.async_request(
      Request{2U}, receive, &state, dp::comm::Duration::from_milliseconds(100U), 77U);
  if (!second_result.has_value()) return 7;
  auto second = std::move(second_result).take_value();
  dp::comm::StaticExecutor executor;
  if (executor.add_node(node) != dp::comm::Result::kOk ||
      executor.spin_some() != dp::comm::Result::kOk) return 8;
  return state.calls == 1U && state.responses == 1U && !second.pending() ? 0 : 9;
}
