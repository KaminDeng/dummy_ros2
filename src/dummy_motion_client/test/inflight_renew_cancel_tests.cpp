/**
 * @file inflight_renew_cancel_tests.cpp
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
 * @brief 先让真实Wire client续租在途且Committed字节尚未消费，再请求取消。
 * @param argc 参数个数（含程序名）。
 * @param argv 参数数组，模式/路径位置见下述说明。
 * @return 0表示所选检查通过；非零或断言/require异常表示失败。
 * 默认positive；gate.cancel不得因同client尚忙而直接STOP_FAILED，应先保持同根bounded cancel等待续租，原Stop预算2,000,000µs不重开。
 * client.poll续租强结果后tick才发唯一新Cancel，request_id必须不同于旧Renew与Query，且退出cancel_waiting状态。
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
  auto renew=auth.identity;renew.set_operation(wire_value(ManageOperation::kRenewLease));renew.set_expected_motion_epoch(2);renew.set_requested_session_lease_us(30000000);
  /**
   * @brief wrong_boot/session/ref/motion/management/profile/mask/device/client逐项改变在途续租身份；gate取消必须不能借错根续租等待而延缓失败。
   * 这些模式预期STOP_FAILED且cancel_waiting_for_renewal=false，不证明任何设备已经停止。
   */
  if(mode=="wrong_boot")renew.set_expected_boot_epoch(8);
  if(mode=="wrong_session")renew.set_expected_session_id(10);
  if(mode=="wrong_ref")renew.set_expected_reference_epoch(2);
  if(mode=="wrong_motion")renew.set_expected_motion_epoch(3);
  if(mode=="wrong_management")renew.set_expected_management_epoch(2);
  if(mode=="wrong_profile")renew.set_expected_profile_id(202);
  if(mode=="wrong_mask")renew.set_configured_mask(3);
  if(mode=="wrong_device"){uint8_t bytes[16]{};bytes[0]=3;renew.set_device_id(bytes);}
  if(mode=="wrong_client"){uint8_t bytes[16]{};bytes[0]=4;renew.set_client_instance_id(bytes);}
  auto rid=client.submit(renew,20);assert(rid!=0 && transport.requests.back().operation()==20);
  /**
   * @par 内部replywire回调
   * q给当前Query request_id，phase选择Prepared/Committed/Failed；固定响应op=原rid+100及当前tuple，不自动接受所有传入请求身份。
   */
  auto replywire=[&](const ManageRequest&q,std::uint8_t phase){wire::DummyManageV2Response r;r.set_schema_major(2);r.set_schema_minor(dummy_ref::protocol::kSchemaMinor);r.set_request_id(q.request_id());r.set_boot_epoch(7);r.set_session_id(9);r.set_reference_epoch(1);r.set_motion_epoch(2);r.set_management_epoch(1);r.set_profile_id(201);r.set_receipt(1);r.set_operation_id(rid+100);r.set_transaction_phase(phase);r.set_result_code(0);return r;};
  if(mode.rfind("wrong_",0)==0){ gate.cancel(25);assert(gate.error()=="STOP_FAILED");assert(!gate.cancel_waiting_for_renewal(renew,26));return 0;}
  transport.respond(replywire(transport.requests.back(),2));auto prepared=client.poll(rid);
  assert(prepared);
  assert(transport.requests.back().operation()==12);
  auto query=transport.requests.back();transport.respond(replywire(query,3));
  assert(transport.responses.size()==1);const auto wirebefore=transport.requests.size();
  /**
   * @brief no_cancel没有gate.cancel，不能凭匹配Renew就宣称有取消等待；abort是生产者丢失故障，不享受取消延后例外。
   */
  if(mode=="no_cancel"){auto x=auth.identity;x.set_operation(wire_value(ManageOperation::kCancel));x.set_expected_motion_epoch(2);assert(!gate.cancel_waiting_for_renewal(x,25));return 0;}
  if(mode=="abort"){gate.abort("REAL_PRODUCER_LOST",25);assert(gate.error()=="STOP_FAILED");return 0;}
  gate.cancel(25);

  std::cerr<<"REAL_WIRE_COMMITTED_UNCONSUMED_CANCEL renewrid="<<rid<<" queryrid="<<query.request_id()<<" bufferedCommitted="<<transport.responses.size()<<" phase="<<unsigned(gate.phase())<<" error="<<gate.error()<<" wirebefore="<<wirebefore<<" wireafter="<<transport.requests.size()<<" portexecuting="<<port.executing<<"\n";
  assert(gate.error().empty());
  auto live=auth.identity;live.set_expected_motion_epoch(2);live.set_operation(wire_value(ManageOperation::kCancel));
  assert(gate.cancel_waiting_for_renewal(live,26));
  /**
   * @brief 已等待续租时再abort(EVIDENCE_INCOMPLETE)，晚到续租结果不得触发Cancel发送或覆盖首错。
   * invalidate同理保留SESSION_CHANGED；两者都比较wire请求数量保持不变。
   */
  if(mode=="waiting_abort"){
    gate.abort("EVIDENCE_INCOMPLETE",26);
    const auto before=transport.requests.size();auto renewal=client.poll(rid);assert(renewal);gate.tick(27);
    assert(transport.requests.size()==before);assert(gate.error()=="EVIDENCE_INCOMPLETE");return 0;
  }
    if(mode=="invalidate"){
    gate.invalidate_authorization(26);assert(gate.error()=="SESSION_CHANGED");
    const auto before=transport.requests.size();auto renewreply=client.poll(rid);assert(renewreply);gate.tick(27);
    assert(transport.requests.size()==before);assert(gate.error()=="SESSION_CHANGED");return 0;
  }
  /**
   * @brief guard_boot/session/ref/motion/management/profile/mask/operation只破坏cancel_waiting查询键，不修改已在途RPC。
   * 坏键false、原键true；仅已在途且bounded意图可临时保留producer资格，普通producer_active=false仍不可开启新续租；2s边界后原键也false。
   */
  if(mode.rfind("guard_",0)==0){auto bad=live;
    if(mode=="guard_boot")bad.set_expected_boot_epoch(8);
    if(mode=="guard_session")bad.set_expected_session_id(10);
    if(mode=="guard_ref")bad.set_expected_reference_epoch(2);
    if(mode=="guard_motion")bad.set_expected_motion_epoch(3);
    if(mode=="guard_management")bad.set_expected_management_epoch(2);
    if(mode=="guard_profile")bad.set_expected_profile_id(202);
    if(mode=="guard_mask")bad.set_configured_mask(3);
    if(mode=="guard_operation")bad.set_operation(20);
    assert(!gate.cancel_waiting_for_renewal(bad,26));assert(gate.cancel_waiting_for_renewal(live,26));
    assert(!run_lease_eligible(run_lease_continuation_inputs({true,true,true,true,false,true,true},true,false)));
    assert(run_lease_eligible(run_lease_continuation_inputs({true,true,true,true,false,true,true},true,gate.cancel_waiting_for_renewal(live,26))));
    assert(!gate.cancel_waiting_for_renewal(live,2000025));return 0;
  }

  /**
   * @brief expired推进到2000025µs要求STOP_FAILED；failed_renew把已缓存Committed换成Failed10，不能继续Cancel。
   * 两负例与正例分开，不以“响应字节已到队列”代替Host已解析强结果。
   */
  if(mode=="expired"){gate.tick(2000025);assert(gate.error()=="STOP_FAILED");return 0;}
  if(mode=="failed_renew"){transport.responses.clear();auto bad=replywire(query,4);bad.set_result_code(10);transport.respond(bad);}
  auto committed=client.poll(rid);assert(committed);
  if(mode=="failed_renew"){gate.tick(26);assert(gate.error()=="STOP_FAILED");return 0;}
  gate.tick(26);
  assert(transport.requests.back().operation()==9);
  auto cancelrid=transport.requests.back().request_id();assert(cancelrid!=rid && cancelrid!=query.request_id());
  assert(!gate.cancel_waiting_for_renewal(live,27));
  std::cerr<<"CANCEL_AFTER_RENEW_COMMITTED cancelRID="<<cancelrid<<" oldrenewRID="<<rid<<" no_new_renew=1\n";
  assert(gate.phase()==Phase::kStopping);return 0;
}
