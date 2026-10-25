/**
 * @file queued_renew_cancel_tests.cpp
 * @brief 核续租与完成/取消冻结相遇时的严格身份、队列预约及原期限。
 *
 * 宏重命名包含wire_management_tests.cpp只为复用Transport/Port类型；DUMMY_ACTION_GATE_TEST再包含生产device_action_gate.cpp，排除其main但不自动创建DeviceActionGate节点。
 * 实际实例是内存WireManagementClient与ActionGate，加生产纯queue/lease helper；不能称为完整ROS Node对象、远端设备或实机执行覆盖。
 * 位置rad，轨迹time秒；gate/tick参数为人工Hostµs，续租时长30000000µs。assert须启用；模式仅由main字符串选择。
 */
#define main shared_wire_fixture_main
#include "wire_management_tests.cpp"
#undef main
#define DUMMY_ACTION_GATE_TEST
#include "../src/device_action_gate.cpp"
#include <cassert>

/**
 * @brief 派生自复用Port计数夹具，增加executing布尔模拟producer状态；不是实际JTC线程。
 */
struct CompletionPort : Port {
  /**
   * @brief start置true、disable置false；冻结资格读取此字段，不能据true声称电机在Run。
   */
  bool executing=false;
  /**
   * @brief 沿用Port启动次数统计，再标executing。
   * @param t 轨迹夹具，仅传给无执行副作用的基类。
   * @param g 目标generation。
   * @return 无。
   */
  void start(const Trajectory &t,std::uint64_t g) override {Port::start(t,g);executing=true;}
  /**
   * @brief 停本地executing标志。
   * @return 无；不发送硬件Disable或等待回执。
   */
  void disable() override {executing=false;}
};
/**
 * @brief 撤回尚未发送的RUN续租预约，取消优先且不归还goal busy。
 * @param argc 参数个数（含程序名）。
 * @param argv 参数数组，模式/路径位置见下述说明。
 * @return 0表示所选检查通过；非零或断言/require异常表示失败。
 * queue_lease_renewal先仅把op20放本地单槽并reserved；Cancel实际入Wire后，仅同根bounded意图且未started/无inflight/不owns_busy可撤。
 * 正例要求不发额外Renew，继续原Cancel强回执及新Hold采样后Canceled；busy始终归当前goal，不能顺手释放。
 */
int main(int argc,char **argv) {
  const std::string mode=argc>1?argv[1]:"positive";
  Transport transport;WireManagementClient client(transport);CompletionPort port;
  ActionGate gate(client,port,{{"Joint1"},{-1},{1},{1},{1},{0.1},{0.1}},2000000);
  Authorization auth;auth.session=auth.reference=auth.permit=true;
  auth.identity.set_expected_boot_epoch(7);auth.identity.set_expected_session_id(9);
  auth.identity.set_expected_reference_epoch(1);auth.identity.set_expected_profile_id(201);
  auth.identity.set_expected_management_epoch(1);auth.identity.set_configured_mask(1);
  /**
   * @par 内部commit回调
   * epoch为合成新motion代次；从Transport最后实际请求取request_id，构造固定boot7/session9/ref1/management1/profile201的Committed响应并poll原请求。
   * 返回client解析结果；此处不模拟设备真实操作，只保证测试确经当前wire codec/请求关联路径。
   */
  auto commit=[&](std::uint32_t epoch) {
    const auto request=transport.requests.back();
    wire::DummyManageV2Response response;response.set_schema_major(2);response.set_schema_minor(dummy_ref::protocol::kSchemaMinor);
    response.set_request_id(request.request_id());response.set_boot_epoch(7);response.set_session_id(9);
    response.set_reference_epoch(1);response.set_motion_epoch(epoch);response.set_management_epoch(1);
    response.set_profile_id(201);response.set_receipt(1);response.set_result_code(0);
    response.set_operation_id(request.request_id()+100);response.set_transaction_phase(3);
    transport.respond(response);return client.poll(request.request_id());
  };
  /**
   * @brief 通过Enable首样本及Begin首样本双屏障到J1 Running，保留当前完整身份供随后续租/完成/取消匹配。
   * prime只是Port计数，accepted_seq1为人工回执，不能把它当六轴CAN组实测。
   */
  assert(gate.enable(auth,1));auto reply=commit(1);assert(reply);
  reply->snapshot_valid=true;reply->hold_available=true;reply->hold_ready=false;gate.observe(*reply,2);
  assert(gate.record_update(gate.generation(),1,1,Position{}));reply->hold_ready=true;reply->accepted_seq=1;gate.observe(*reply,3);
  assert(gate.submit({{"Joint1"},{{{0.2},{0},{},1}},{},{},0},10));reply=commit(2);assert(reply);
  reply->snapshot_valid=true;reply->hold_available=true;reply->hold_ready=false;gate.observe(*reply,11);
  assert(gate.record_update(gate.generation(),2,1,Position{}));reply->hold_ready=true;reply->accepted_seq=1;gate.observe(*reply,12);
  assert(gate.phase()==Phase::kRunning);
  const std::string variant=argc>1?argv[1]:"positive";
  auto live=auth.identity;live.set_operation(wire_value(ManageOperation::kCancel));live.set_expected_motion_epoch(2);
  std::atomic<int> queued{0},explicit_status{0};std::atomic<bool> busy{true},reserved{false},running{true},started{false};
  assert(queue_lease_renewal(queued,busy,reserved,true));
  const auto before_cancel=transport.requests.size();
  /**
   * @brief request_missing发送前拒绝、abort/no_cancel、wrong_boot/session/ref/motion/mgmt/profile/mask/device/client、not_cancel、lost_authorization、expired均不得产生bounded可撤意图。
   * 其它started/owns_busy/nonrenew/not_reserved/not_running/inflight可在意图有效时单独阻止队列撤回。
   */
  if(variant=="request_missing")transport.reject_send=true;
  if(variant=="abort")gate.abort("REAL_PRODUCER_LOST",25);else if(variant!="no_cancel")gate.cancel(25);
  if(variant=="wrong_boot")live.set_expected_boot_epoch(8);
  if(variant=="wrong_session")live.set_expected_session_id(10);
  if(variant=="wrong_ref")live.set_expected_reference_epoch(2);
  if(variant=="wrong_motion")live.set_expected_motion_epoch(3);
  if(variant=="wrong_mgmt")live.set_expected_management_epoch(2);
  if(variant=="wrong_profile")live.set_expected_profile_id(202);
  if(variant=="wrong_mask")live.set_configured_mask(63);
  if(variant=="wrong_device"){std::uint8_t v[16]{};v[0]=1;live.set_device_id(v);}
  if(variant=="wrong_client"){std::uint8_t v[16]{};v[0]=1;live.set_client_instance_id(v);}
  if(variant=="not_cancel")live.set_operation(wire_value(ManageOperation::kComplete));
  if(variant=="lost_authorization")gate.invalidate_authorization(26);
  const auto now=variant=="expired"?2000025U:26U;
  const bool bounded=gate.bounded_cancel_intent(live,now);
  bool owns_busy=variant=="owns_busy",inflight=false;
  if(variant=="started")started=true;
  if(variant=="nonrenew")queued=10;
  if(variant=="not_reserved")reserved=false;
  if(variant=="not_running")running=false;
  /**
   * @brief 用另一独立WireManagementClient建立Renew20在途，只证明withdraw helper的renewal_inflight守卫。
   * 此处没有声称同一client可同时发Renew和Cancel，也没有完整Node多回调竞争覆盖。
   */
  if(variant=="actual_inflight") {
    // 独立真实Wire client仅证明helperinflight guard；不假同client可并发Renew和Cancel。
    Transport other;WireManagementClient renewal(other);auto r=auth.identity;
    r.set_operation(wire_value(ManageOperation::kRenewLease));r.set_expected_motion_epoch(2);r.set_requested_session_lease_us(30000000);
    const auto rid=renewal.submit(r,26);assert(rid);auto expected=r;expected.set_operation(wire_value(ManageOperation::kComplete));
    inflight=renewal.matching_inflight_renewal(expected)==rid;assert(inflight);
  }
  /**
   * @brief 正例撤op20→0且清reserved/running/started，保goal busy；负例withdrawn=false。
   * 最后run_lease_eligible七项中producer=false仍严格false，bounded cancel只延续既有预约，不能生成新续租资格。
   */
  const bool withdrawn=withdraw_queued_running_renewal(queued,reserved,running,started,owns_busy,inflight,bounded,explicit_status);
  const bool positive=variant=="positive";
  if(positive) {
    assert(bounded && withdrawn && queued==0 && !reserved && !running && !started && busy);
    assert(gate.phase()==Phase::kStopping && !port.executing && transport.requests.size()==before_cancel+1);
    assert(transport.requests.back().operation()==9);
    // 撤预约后仍按原Cancel回执完成，不增续租、Grant、Enable或deadline。
    auto result=commit(3);assert(result);result->snapshot_valid=true;result->hold_available=true;result->hold_ready=false;gate.observe(*result,27);
    assert(gate.record_update(gate.generation(),3,2,Position{}));result->hold_ready=true;result->accepted_seq=2;gate.observe(*result,28);
    assert(gate.terminal()==Terminal::kCanceled);
  } else {
    assert(!withdrawn && busy);
    if(variant!="started" && variant!="actual_inflight" && variant!="owns_busy" && variant!="nonrenew" && variant!="not_reserved" && variant!="not_running")assert(!bounded);
  }
  // 非executing RUN资格仍严格false，不因为boundedCancel生成新续租。
  assert(!run_lease_eligible({true,true,true,true,false,true,true}));
  std::cerr<<"QUEUED_CANCEL_BOUND_GUARD mode="<<variant<<" bounded="<<bounded<<" withdrawn="<<withdrawn<<" busy="<<busy<<" wire20="<<std::count_if(transport.requests.begin(),transport.requests.end(),[](const auto&r){return r.operation()==20;})<<'\n';
  return 0;
}
