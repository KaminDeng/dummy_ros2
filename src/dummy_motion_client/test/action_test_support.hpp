/**
 * @file action_test_support.hpp
 * @brief ActionGate宿主值/调用次数测试的最小公共夹具。
 *
 * Client只收集ManageRequest，Port只数prime/start次数，所有ManageReply由Harness合成；没有ROS、网络、设备或真实轨迹执行。
 * 默认J1位置[-1,1]rad、速度1rad/s、加速度1rad/s²、path/goal容差0.1rad；gate/tick用人工Hostµs，轨迹点time用秒。
 */
#pragma once
#include "dummy_motion_client/action_gate.hpp"
#include <stdexcept>
namespace action_test {
using namespace dummy_motion_client;
/**
 * @brief 把夹具关系失败转成异常。
 * @param value 本次断言结果。
 * @return 无；false抛runtime_error。
 */
inline void require(bool value) { if (!value) throw std::runtime_error("action lifecycle requirement failed"); }
/**
 * @brief 单线程内存管理客户端，requests顺序定义本地request_id。
 */
struct Client : ManagementClient {
  /**
   * @brief 保存原请求值以核操作/身份；入列表不表示设备已经接纳。
   */
  std::vector<ManageRequest> requests;
  /**
   * @brief 存请求并分配列表长度作为逻辑ID。
   * @param request 管理请求夹具。
   * @par 未命名参数
   * Hostµs时间被本夹具忽略。
   * @return requests.size()非零ID。
   */
  std::uint64_t submit(const ManageRequest &request,std::uint64_t) override {
    requests.push_back(request); return requests.size();
  }
  /**
   * @brief 不主动提供管理回执。
   * @par 未命名参数
   * 传入逻辑ID被忽略。
   * @return 空optional，测试须显式调用gate.observe。
   */
  std::optional<ManageReply> poll(std::uint64_t) override { return {}; }
};
/**
 * @brief 非执行ActionPort，primes/starts仅记录方法调用次数。
 */
struct Port : ActionPort {
  /**
   * @brief 累积prime/start调用数，不是原始CAN组数或物理到位次数。
   */
  unsigned primes=0, starts=0;
  /**
   * @brief 保持位置空执行：未命名Position(rad)/epoch/generation仅匹配接口，primes加1。
   * @return 无。
   */
  void prime(const Position &,std::uint32_t,std::uint64_t) override { ++primes; }
  /**
   * @brief 启动轨迹空执行：未命名Trajectory/generation不被执行，starts加1。
   * @return 无。
   */
  void start(const Trajectory &,std::uint64_t) override { ++starts; }
  /**
   * @brief 空失能，不发送设备Disable。
   * @return 无。
   */
  void disable() override {}
  /**
   * @brief 空取消，不产生Action或设备确认。
   * @return 无。
   */
  void cancel() override {}
};
/**
 * @brief Client、计数Port、J1 Gate和人工epoch组成的可复用场景。
 * 各方法从公开gate接口推进，没有直接赋生产私有状态；回复仍为手写夹具。
 */
struct Harness {
  /**
   * @brief 先构造依赖再构造gate，引用生存期覆盖gate；epoch从0逐步推进。
   */
  Client client; Port port;
  ActionGate gate{client,port,{{"Joint1"},{-1},{1},{1},{1},{0.1},{0.1}}};
  std::uint32_t epoch=0;
  /**
   * @brief 合成与最近本地请求关联的快照/结果。
   * @param success true同时置result_valid/succeeded/hold_ready。
   * @param accepted_seq 人工已接受producer线序号，默认1。
   * @return request_id为请求列表长度、operation_id=100+ID，motion_epoch取本Harness当前epoch。
   */
  ManageReply response(bool success=false,std::uint32_t accepted_seq=1) const {
    ManageReply reply; reply.request_id=client.requests.size(); reply.operation_id=100+reply.request_id;
    reply.accepted=true; reply.snapshot_valid=true; reply.motion_epoch=epoch;
    reply.hold_available=true; reply.result_valid=success; reply.succeeded=success;
    reply.hold_ready=success; reply.accepted_seq=accepted_seq; return reply;
  }
  /**
   * @brief session1/参考/permit声明下推进Enable、prime样本seq1及强回复到ready。
   * @return 无；只建立内存夹具授权，不是现场许可。
   */
  void enable() {
    Authorization auth; auth.session=auth.reference=auth.permit=true;
    auth.identity.set_expected_session_id(1); require(gate.enable(auth,1));
    ++epoch; gate.observe(response(),2); require(gate.record_update(gate.generation(),epoch,1,Position{}));
    gate.observe(response(true),3); require(gate.ready());
  }
  /**
   * @brief 提交J1目标0.2rad/1s，推进新epoch首保持采样后才允许Port start。
   * @return 无；内部assert由require负责。
   */
  void begin() {
    const Trajectory trajectory{{"Joint1"},{{{0.2},{0},{},1}},{},{},0};
    require(gate.submit(trajectory,10)); ++epoch; gate.observe(response(),11);
    require(gate.record_update(gate.generation(),epoch,1,Position{})); gate.observe(response(true),12);
  }
  /**
   * @brief 末seq17冻结后等ACK，再Complete及新epoch Hold样本，最后新permit才重新ready。
   * @return 无；terminal Succeeded与ready分别核，成功不会自动沿用旧permit。
   */
  void complete() {
    Position end{}; end[0]=0.2; require(gate.record_update(gate.generation(),epoch,17,end));
    gate.inner_result(gate.generation(),Terminal::kSucceeded,"",13);
    require(gate.terminal()==Terminal::kNone);
    require(!gate.record_update(gate.generation(),epoch,18,end));
    gate.observe(response(true,17),14);
    require(client.requests.back().terminal_producer_seq()==17);
    ++epoch; gate.observe(response(),15); require(gate.terminal()==Terminal::kNone);
    require(gate.record_update(gate.generation(),epoch,1,Position{}));
    gate.observe(response(true),16); require(!gate.ready());
    Authorization renewed; renewed.session=renewed.reference=renewed.permit=true;
    renewed.identity.set_expected_session_id(1);
    require(gate.renew_permit(renewed));
    require(gate.ready());
  }
};
}  // namespace action_test
