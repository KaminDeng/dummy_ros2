/**
 * @file request_handle_destructor_tests.cpp
 * @brief 验证尚未调度的RequestHandle析构释放本地pending槽。
 *
 * StaticOnly进程内服务只有1个pending槽；先销毁请求句柄再调度。结论不延伸为撤销已发到远端或已执行业务的请求。
 */
#include <cstdint>
#include <cstdio>
#include <utility>

#include "dp_comm/dp_comm.hpp"

namespace {
/**
 * @brief 请求夹具，value区分会被析构清理的1与应实际送达的2。
 */
struct Request { std::uint32_t value; };
/**
 * @brief 响应夹具，回显请求value以确定回调属于第二次请求。
 */
struct Response { std::uint32_t value; };
/**
 * @brief 本地typed service的请求/响应类型绑定，不含额外wire协议。
 */
struct Service { using Request = ::Request; using Response = ::Response; };
/**
 * @brief calls计服务执行次数，responses计结果Ok且回显2的回调次数。
 */
struct State { std::uint32_t calls = 0U; std::uint32_t responses = 0U; };

/**
 * @brief 回显请求并计一次真实服务调用。
 * @param request 本次收到的值夹具。
 * @param response dp_comm提供的可写响应槽。
 * @par 未命名RequestInfo参数
 * 此用例不检查来源身份。
 * @param context 指向main中存活的State计数器。
 * @return kOk，表示本地服务处理成功。
 */
dp::comm::Result serve(const Request &request, Response *response,
                       const dp::comm::RequestInfo &, void *context) {
  auto *state = static_cast<State *>(context);
  ++state->calls;
  response->value = request.value;
  return dp::comm::Result::kOk;
}

/**
 * @brief 只计第二次请求的成功响应。
 * @param result 调度结果；非Ok不计数。
 * @param response 可空响应；只在非空且value=2时计数。
 * @par 未命名ResponseInfo参数
 * 此用例不使用其内容。
 * @param context 指向State。
 * @return 无；main核对最终计数。
 */
void receive(dp::comm::Result result, const Response *response,
             const dp::comm::ResponseInfo &, void *context) {
  if (result == dp::comm::Result::kOk && response && response->value == 2U) {
    ++static_cast<State *>(context)->responses;
  }
}
}  // namespace

/**
 * @brief 用同一correlation=77重用唯一pending槽。
 *
 * 第一请求的句柄在executor运行前离开作用域；第二请求应成功占槽并仅被执行一次。两个请求的100ms超时是dp_comm请求参数，不是本测试强制测得的执行时长。
 * @return 0 表示本文件全部检查通过；非零定位失败检查。
 */
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
