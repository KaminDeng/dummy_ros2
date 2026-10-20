/**
 * @file control_chain_logging_tests.cpp
 * @brief 核Action/Bootstrap失败链的结构化诊断因果记录。
 *
 * HostDiagnosticScope把同线程诊断写入本地journal；assert须启用。所有状态/回执为内存夹具，不实际发送Stop或证明设备Idle。
 */
#include "dummy_motion_client/action_gate.hpp"
#include "dummy_motion_client/bootstrap_coordinator.hpp"
#include "dummy_ros_control/diagnostic_scope.hpp"
#include <cassert>
#include <cstring>
#include "dummy_ros_control/diagnostic_json.hpp"
#include <iostream>
using namespace dummy_motion_client;
/**
 * @brief 每次submit只递增本地ID且poll永远空，使Action回执等待可确定超时。
 */
struct Client final : ManagementClient {
  /**
   * @brief 本地逻辑ID计数，不是远端已接受的operation_id。
   */
  std::uint64_t next=0U;
  /**
   * @brief 记录一次本地提交编号。
   * @par 未命名参数
   * ManageRequest和Hostµs预算未被此夹具使用。
   * @return 自增非零逻辑ID。
   */
  std::uint64_t submit(const ManageRequest&,std::uint64_t) override {return ++next;}
  /**
   * @brief 对任何请求ID返回无回执。
   * @par 未命名参数
   * uint64为请求ID，夹具忽略。
   * @return 空optional。
   */
  std::optional<ManageReply> poll(std::uint64_t) override {return {};}
};
/**
 * @brief 无副作用ActionPort；freeze_hold返回true允许进入完成/停止屏障，其余动作不执行硬件。
 */
struct Port final : ActionPort {
  /**
   * @brief 保持采样空操作；未命名位置rad/epoch/generation仅匹配接口。
   * @return 无。
   */
  void prime(const Position&,std::uint32_t,std::uint64_t) override {}
  /**
   * @brief 轨迹启动空操作；不实际执行传入轨迹或代次。
   * @return 无。
   */
  void start(const Trajectory&,std::uint64_t) override {}
  /**
   * @brief 失能空操作，不代表设备Disable确认。
   * @return 无。
   */
  void disable() override {}
  /**
   * @brief 取消空操作，不生成Action回执。
   * @return 无。
   */
  void cancel() override {}
  /**
   * @brief 合成保持冻结成功。
   * @return true；只是夹具承诺，不检查实际末位置。
   */
  bool freeze_hold() override {return true;}
};
/**
 * @brief 先由手工Enable/prime进入就绪，再制造Hold排空超时与Stop失败。
 * @return 无；核HOLD_DRAIN_TIMEOUT后STOP_FAILED/Aborted，后续main核两阶段诊断身份与µs锚。
 */
void action_chain() {
  Client client;Port port;
  ActionGate gate(client,port,{{"Joint1"},{-1},{1},{1},{2},{0.1},{0.1}});
  Authorization auth;auth.session=auth.reference=auth.permit=true;
  auth.identity.set_expected_boot_epoch(7U);auth.identity.set_expected_session_id(9U);
  auth.identity.set_expected_reference_epoch(3U);auth.identity.set_expected_motion_epoch(1U);
  assert(gate.enable(auth,1U));
  ManageReply reply;reply.request_id=1U;reply.accepted=true;reply.snapshot_valid=true;
  reply.motion_epoch=2U;reply.hold_available=true;reply.session_id=9U;
  gate.observe(reply,2U);
  assert(gate.record_update(gate.generation(),2U,1U,Position{},9U));
  reply.result_valid=reply.succeeded=reply.hold_ready=true;reply.accepted_seq=1U;
  gate.observe(reply,3U);assert(gate.ready());
  assert(gate.submit({{"Joint1"},{{{0.2},{0.0},{},1.0}},{},{},0.0},4U));
  gate.observe_hold_drain({7U,9U,9U,3U,2U,1U},5U);
  gate.tick(1000004U);assert(gate.error()=="HOLD_DRAIN_TIMEOUT");
  gate.tick(2000004U);assert(gate.error()=="STOP_FAILED" && gate.terminal()==Terminal::kAborted);
}
/**
 * @brief 总回同一request1的Committed/release标志，故意不给当前Idle观测。
 */
struct ReplyClient final : ManagementClient {
  /**
   * @brief 合成固定本地request1。
   * @par 未命名参数
   * 请求和时间被忽略。
   * @return 1。
   */
  std::uint64_t submit(const ManageRequest&,std::uint64_t) override {return 1U;}
  /**
   * @brief 返回固定op91/phase3强release字段。
   * @par 未命名参数
   * 请求ID被忽略；业务观测由Bootstrap另核。
   * @return ManageReply；不能单凭此值完成Idle确认。
   */
  std::optional<ManageReply> poll(std::uint64_t) override {
    ManageReply reply;reply.request_id=1U;reply.result_valid=reply.succeeded=reply.release_committed=true;
    reply.transaction_phase=3U;reply.operation_id=91U;return reply;
  }
};
/**
 * @brief 已收到release强回复后仍只给旧Hold观测，要求最终RELEASE_UNCONFIRMED。
 * @return 无；confirmed_release表示回执已知，不等于释放Idle已观察。
 */
void bootstrap_chain() {
  ReplyClient client;ManageRequest identity;identity.set_expected_session_id(9U);
  BootstrapCoordinator coordinator(client,identity,{},1000000U);
  BootstrapSnapshot snapshot;snapshot.boot_epoch=7U;snapshot.session_id=9U;
  snapshot.reference_epoch=3U;snapshot.motion_epoch=4U;snapshot.management_epoch=5U;
  snapshot.profile_id=201U;snapshot.configured_mask=63U;snapshot.freshness_ticket=1U;
  snapshot.raw_ready=snapshot.motion_supported=true;snapshot.core_state=snapshot.state_core_state=4U;
  assert(coordinator.start_release(snapshot,100U));coordinator.tick(101U);
  assert(coordinator.confirmed_release());coordinator.observe(snapshot,5000100U);
  assert(coordinator.phase()==BootstrapPhase::kFault && std::strcmp(coordinator.error(),"RELEASE_UNCONFIRMED")==0);
}
/**
 * @brief 选择action或bootstrap链并核journal首错/分reason记录。
 * @param argc 参数个数（含程序名）。
 * @param argv 参数数组，模式/路径位置见下述说明。
 * @return 0表示所选检查通过；非零或断言/require异常表示失败。
 * 仅argv[1]=bootstrap选Bootstrap，其它走Action。核回复op91与随后timeout保留不同记录；默认details关闭时只计suppressed，最终打印JSON。
 */
int main(int argc,char**argv) {
  dummy_ros_control::DiagnosticJournal journal;
  dummy_ros_control::HostDiagnosticScope scope(journal);
  const bool bootstrap=argc==2 && std::strcmp(argv[1],"bootstrap")==0;
  if(bootstrap)bootstrap_chain();else action_chain();
  using dummy_ros_control::DiagnosticReason;
  if(bootstrap) {
    const auto reply=journal.first_record(DiagnosticReason::kBootstrapReleaseReply,true);
    const auto timeout=journal.first_record(DiagnosticReason::kBootstrapReleaseTimeout,true);
    assert(reply && reply->fields[0]==1U && reply->fields[8]==91U);
    assert(timeout && timeout->fields[0]==10U && timeout->fields[2]==9U && timeout->fields[8]==0U);
    assert(journal.snapshot().first.reason==DiagnosticReason::kBootstrapReleaseTimeout);
  } else {
    const auto timeout=journal.first_record(DiagnosticReason::kActionHoldDrainTimeout,true);
    const auto stop=journal.first_record(DiagnosticReason::kActionStop,true);
    const auto fail=journal.first_record(DiagnosticReason::kActionFail,true);
    assert(timeout && timeout->fields[0]==1U && timeout->fields[1]==9U && timeout->fields[7]==2U);
    assert(stop && stop->fields[3]==1000004U && stop->expected.boot==7U);
    assert(fail && fail->fields[2]==2U && std::strcmp(fail->detail.data(),"STOP_FAILED")==0);
    assert(journal.snapshot().detail_suppressed>=6U);
  }
  std::cout << dummy_ros_control::diagnostic_journal_json(journal) << '\n';
}
