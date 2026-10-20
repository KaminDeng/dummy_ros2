/**
 * @file native_wait_policy_tests.cpp
 * @brief 核Native单worker等待/取消双证据和不可恢复Fault锁。
 *
 * 只给policy人工µs与标志，不做ROS取消。cancel_acknowledged是调用方报告的响应标志，本policy不核目标列表或设备停止。
 */
#include "dummy_motion_client/native_wait_policy.hpp"
#include <limits>
#include <stdexcept>
/**
 * @brief 将值对象断言失败变为异常。
 * @param value 本检查关系。
 * @return 无，false抛runtime_error。
 */
void require(bool value) { if (!value) throw std::runtime_error("native deadline requirement failed"); }
/**
 * @brief 占槽后等终态；取消既要响应标志也要终态，重复cancel不延长截止。
 * @return 0表示所选检查通过；非零或断言/require异常表示失败。
 * 首cancel210+20截止230，第二次229+10000不能重开；少任何证据finish都锁Fault，之后补标志也不能恢复。
 * UINT64_MAX附近截止饱和到最大值，不能因加法回绕立即放行。
 */
int main() {
  using dummy_motion_client::NativeWaitPolicy;
  NativeWaitPolicy policy;
  require(policy.try_acquire()); require(!policy.try_acquire());
  policy.begin(100,50); require(!policy.expired(149)); require(policy.expired(150));
  policy.terminal_confirmed();
  policy.finish(); require(!policy.busy()); require(policy.try_acquire());
  policy.begin(200,100); policy.cancel(210,20);
  policy.cancel(229,10000);
  require(!policy.expired(229)); require(policy.expired(230));
  policy.finish(); require(policy.busy()); require(policy.faulted()); require(!policy.try_acquire());
  policy.cancel_acknowledged(); policy.terminal_confirmed(); policy.finish();
  require(policy.faulted()); require(!policy.try_acquire());
  NativeWaitPolicy acknowledged; require(acknowledged.try_acquire());
  acknowledged.begin(100,50); acknowledged.cancel(150,20);
  require(!acknowledged.expired(151));
  acknowledged.cancel_acknowledged(); require(!acknowledged.cancel_complete());
  acknowledged.terminal_confirmed(); require(acknowledged.cancel_complete());
  acknowledged.finish(); require(!acknowledged.busy()); require(acknowledged.try_acquire());
  NativeWaitPolicy terminal_only; require(terminal_only.try_acquire());
  terminal_only.begin(100,50); terminal_only.cancel(150,20); terminal_only.terminal_confirmed();
  require(!terminal_only.cancel_complete()); terminal_only.finish();
  require(terminal_only.faulted()); require(!terminal_only.try_acquire());
  NativeWaitPolicy ack_only; require(ack_only.try_acquire());
  ack_only.begin(100,50); ack_only.cancel(150,20); ack_only.cancel_acknowledged();
  require(!ack_only.cancel_complete()); ack_only.finish();
  require(ack_only.faulted()); require(!ack_only.try_acquire());
  NativeWaitPolicy restarted; require(restarted.try_acquire());
  const auto maximum=std::numeric_limits<std::uint64_t>::max();
  restarted.begin(maximum-5,50); require(!restarted.expired(maximum-4)); require(restarted.expired(maximum));
  restarted.finish(); require(restarted.busy()); require(restarted.faulted());
}
