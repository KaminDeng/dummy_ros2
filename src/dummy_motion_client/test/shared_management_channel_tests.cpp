/**
 * @file shared_management_channel_tests.cpp
 * @brief 核DeviceWorker facade与provider共享快照/排空/原证据状态及有限准入。
 *
 * 构造provider登记共享channel但不start实际transport；手动ready/observation仅为本地夹具。facade start挂registry，不运行ROS/peer或USB。
 * 所有Hostµs和目标rad来自人工字段，末尾provider销毁后仍由facade缓存/共享引用保有证据标志，不意味着设备仍在线。
 */
#include "dummy_motion_client/dp_management_transport.hpp"
#include "dummy_ros_control/device_management_channel.hpp"
#include "dummy_ros_control/guarded_zenbedded_system.hpp"
#include <cstdio>
#include <chrono>
#include <future>
#include <thread>
#include <memory>
#include <string>
#include <unistd.h>

/**
 * @brief 将一次正常返回动作的stderr重定向到tmpfile并读取最多512字节。
 * @tparam Action 零参数测试动作类型。
 * @param action 本地send等预期正常返回的观察动作。
 * @return 截取stderr字符串；创建/重定向失败为空。
 * @note 此helper没有异常RAII恢复路径，不能承诺任意throw动作后stderr仍恢复；只服务当前有限用例。
 */
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
/**
 * @brief 共享观测、try-lock缓存、排空撤销、原证据状态及管理seq重定位。
 * @return 0表示所选检查通过；非零或断言/require异常表示失败。
 * absent/not_ready/lock_busy/queued拒绝原因应可观察；两个client不能并发占同管理单槽，句柄析构可释放本地pending。
 * busy first_poll须≤20ms且无入槽，释放锁后100ms窗口内以原ID/原字节唯一入槽；这是该调度样本的观测阈值，不是通用WCET证明。
 */
int main() {
  dummy_ros_control::TransportParameters parameters{"tcp/127.0.0.1:29999",{1},{2},2,1};
  dummy_motion_client::DpManagementTransport absent({parameters.endpoint,{1},{2},2,1},
      dummy_motion_client::DpManagementTransport::Connection::kDeviceWorker);
  std::vector<std::uint8_t> bytes(dp::protocol::dummy_robot_v2::DummyManageV2RequestCodec::kEncodedSize);
  dummy_motion_client::ManageRequest request;
  request.set_schema_major(2U);
  request.set_schema_minor(dp::protocol::dummy_robot_v2::kSchemaMinor);
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
  /**
   * @brief processed_generation12785可对应committed_seq12784的同值抑制排空；观测时间/容差0.003rad等必须逐项传递。
   * 重设空hold_drain要显式撤销旧quiescent，不能返回无新信息而让消费者沿用旧证明。
   */
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
  /**
   * @brief 原证据失败、capture启用、terminal flush及干净取消标志是独立共享字段。
   * 这里只手动发布/读取快照与身份匹配，不重放SHA链或执行writer，所以不能当完整故障/释放证据测试。
   */
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
  /**
   * @brief 此freshness辅助只检查接收年龄上界；now99小于人工received100仍返回true。
   * 300100在默认门槛拒绝，1s自定义预算可接受直到1000101超界；不等价于完整Source时钟/未来包/身份校验。
   */
  if(!dummy_motion_client::observation_is_fresh(*facade.observe(),99) ||
     dummy_motion_client::observation_is_fresh(*facade.observe(),300100) ||
     !dummy_motion_client::observation_is_fresh(*facade.observe(),300100,1000000U) ||
     dummy_motion_client::observation_is_fresh(*facade.observe(),1000101,1000000U)) return 7;
  {
    std::lock_guard<std::mutex> lock(channel->mutex);
    /**
     * @brief channel mutex被同线程持有时observe使用已有缓存，不能阻塞等待锁；缓存年龄仍由后续调用者另判。
     */
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
  std::promise<void> lock_owned, release_lock;
  auto released=release_lock.get_future();
  /**
   * @par 持锁测试线程
   * 先发lock_owned promise，再等release_lock；main确认锁已持后量一次poll，随后放行join。测试门闩故意阻塞线程，不是生产poll阻塞。
   */
  std::thread holder([&] {
    std::unique_lock<std::mutex> lock(channel->mutex);
    lock_owned.set_value();
    released.wait();
  });
  lock_owned.get_future().wait();
  const auto admission_started=dummy_motion_client::management_now_us();
  const auto busy_id=busy_client.submit(query,admission_started);
  const auto poll_started=dummy_motion_client::management_now_us();
  const auto first_poll=busy_id ? busy_client.poll(busy_id) :
      std::optional<dummy_motion_client::ManageReply>{};
  const auto first_poll_us=dummy_motion_client::management_now_us()-poll_started;
  release_lock.set_value();
  holder.join();
  if(!busy_id || first_poll || channel->queued || first_poll_us>20000U) return 10;
  const auto admission_deadline=admission_started+100000U;
  while(!channel->queued && dummy_motion_client::management_now_us()<admission_deadline) {
    if(busy_client.poll(busy_id)) return 11;
    if(!channel->queued) std::this_thread::sleep_for(std::chrono::microseconds(2000));
  }
  if(!channel->queued) return 11;
  const auto admitted_pending=channel->queued;
  const auto original_bytes=admitted_pending->request;
  dummy_motion_client::ManageRequest admitted_request;
  if(dp::protocol::dummy_robot_v2::DummyManageV2RequestCodec::decode(
      original_bytes.data(),original_bytes.size(),&admitted_request)!=
      dp::protocol::dummy_robot_v2::CodecResult::kOk ||
      admitted_request.request_id()!=busy_id || admitted_request.request_seq()==0U ||
      admitted_request.operation()!=dummy_motion_client::wire_value(dummy_motion_client::ManageOperation::kQuery)) return 11;
  auto expected_request=query; expected_request.set_schema_major(2U); expected_request.set_schema_minor(dp::protocol::dummy_robot_v2::kSchemaMinor);
  expected_request.set_request_id(busy_id);expected_request.set_request_seq(admitted_request.request_seq());
  std::array<std::uint8_t,dp::protocol::dummy_robot_v2::DummyManageV2RequestCodec::kEncodedSize> expected_bytes{};
  if(dp::protocol::dummy_robot_v2::DummyManageV2RequestCodec::encode(
      expected_request,expected_bytes.data(),expected_bytes.size())!=
      dp::protocol::dummy_robot_v2::CodecResult::kOk || expected_bytes!=original_bytes ||
      busy_client.poll(busy_id) || channel->queued!=admitted_pending ||
      admitted_pending->request!=original_bytes) return 11;
  channel->queued.reset();
  /**
   * @brief 共享请求seq水位必须跨client保留：见2183397839后新client用2183397840。
   * 旧值不能回退；UINT32_MAX后跳过0为1，后续旧max/半环距离值也不应重置，依次reserve2/3。
   */
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
