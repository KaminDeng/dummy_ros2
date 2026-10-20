/**
 * @file first_sample_handshake_tests.cpp
 * @brief 核Enable/Begin双流首保持采样和真实producer ACK屏障。
 *
 * 复用内存Harness，primes/starts只是Port计数，不是电机实际到位次数。管理快照、record_update与State ACK分别驱动状态，避免Queued提前启动。
 */
#include "action_test_support.hpp"
#include <iostream>
/**
 * @brief Queued不prime；新epoch采样后仍须首有效producer ACK。
 * @return 0表示所选检查通过；非零或断言/require异常表示失败。
 * 0.2rad不连续首样本拒绝；两个相同保持更新1/2后ACK第一条1即可完成握手，不要求ACK最新2。Begin时先prime且starts0，首采样ACK后才starts1。
 */
int main() {
  using namespace action_test;
  Harness harness; Authorization auth; auth.session=auth.reference=auth.permit=true;
  auth.identity.set_expected_session_id(1); require(harness.gate.enable(auth,1));
  ManageReply queued; queued.request_id=1; queued.queued=true;
  harness.gate.observe(queued,2); require(harness.port.primes==0);
  ++harness.epoch; harness.gate.observe(harness.response(),3);
  const bool enable_prime=harness.port.primes==1 && !harness.gate.ready(); require(enable_prime);
  Position discontinuous{}; discontinuous[0]=0.2;
  require(!harness.gate.record_update(harness.gate.generation(),harness.epoch,1,discontinuous));
  require(harness.gate.record_update(harness.gate.generation(),harness.epoch,1,Position{}));
  require(harness.gate.record_update(harness.gate.generation(),harness.epoch,2,Position{}));
  harness.gate.observe(harness.response(true,1),4);
  const bool ack_first_seq_not_latest=harness.gate.ready(); require(ack_first_seq_not_latest);
  const Trajectory trajectory{{"Joint1"},{{{0.2},{0},{},1}},{},{},0};
  require(harness.gate.submit(trajectory,10)); ++harness.epoch;
  harness.gate.observe(harness.response(),11);
  const bool begin_prime_before_trajectory=harness.port.primes==2 && harness.port.starts==0;
  require(begin_prime_before_trajectory);
  require(harness.gate.record_update(harness.gate.generation(),harness.epoch,1,Position{}));
  harness.gate.observe(harness.response(true),12); require(harness.port.starts==1);
  std::cout << std::boolalpha << "{\"enable_prime_before_success\":" << enable_prime
    << ",\"ack_first_seq_not_latest\":" << ack_first_seq_not_latest
    << ",\"begin_prime_before_trajectory\":" << begin_prime_before_trajectory << "}\n";
}
