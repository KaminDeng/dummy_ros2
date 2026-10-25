/**
 * @file cancel_query_cadence_tests.cpp
 * @brief 在人工20ms轮询节拍下核Cancel/Disable与普通操作的Query节流。
 *
 * Transport是内存编码队列，elapsed只由for循环推进，不运行真实20ms线程或设备。Wire client本身的Query进度与外层原250ms Stop期限分开。actual_late通过表示确认晚于预算的反例观察，不能声称原预算内物理停止。
 */
#include "dummy_motion_client/management_client.hpp"
#include "dummy_motion_client/action_gate.hpp"
#include "dummy_motion_client/bootstrap_coordinator.hpp"
#include "dummy_motion_client/bootstrap_tick_policy.hpp"
#include "dummy_motion_client/dp_management_transport.hpp"
#include <deque>
#include <algorithm>
#include <iostream>
#include <stdexcept>
using namespace dummy_motion_client;
namespace wire=dp::iface;
/**
 * @brief 内存ManagementTransport：requests记录实际编码后请求，responses保存待poll响应。
 * timeout_next仅使一次poll抛异常，reject_send使send返回空；无网络/peer。
 */
struct Transport : ManagementTransport {
  /**
   * @brief 下次Handle::poll一次性故障注入；与reject_send的发送前拒绝是不同阶段。
   */
  bool timeout_next=false;
  bool reject_send=false;
  /**
   * @brief 顺序保留每次send解码的请求，供原ID/QueryID/操作号及节奏断言。
   */
  std::vector<ManageRequest> requests;
  /**
   * @brief 回调响应FIFO，poll取走首条；字节来自当前正式codec。
   */
  std::deque<std::vector<std::uint8_t>> responses;
  /**
   * @brief 借用Transport的局部请求句柄；Transport须比句柄活得更久。
   */
  struct Handle : RequestHandle {
    /**
     * @brief 保存队列上下文引用。
     * @param transport 拥有响应队列和故障标志的对象。
     * @return 构造完成的句柄，不发送任何数据。
     */
    explicit Handle(Transport &transport) : transport(transport) {}
    /**
     * @brief 非拥有引用，仅在同一测试线程访问，不提供并发队列保证。
     */
    Transport &transport;
    /**
     * @brief 一次性抛模拟超时，或弹出一条已有响应。
     * @return 空optional表示暂无响应；有值为原编码bytes，超时抛runtime_error。
     */
    std::optional<std::vector<std::uint8_t>> poll() override {
      if(transport.timeout_next) { transport.timeout_next=false; throw std::runtime_error("simulated query timeout"); }
      if (transport.responses.empty()) return {};
      auto result=transport.responses.front(); transport.responses.pop_front(); return result;
    }
  };
  /**
   * @brief 将请求按当前codec解码并存档。
   * @param bytes 完整管理请求wire字节。
   * @return 新Handle；reject_send为空，解码失败抛runtime_error。
   */
  std::unique_ptr<RequestHandle> send(const std::vector<std::uint8_t> &bytes) override {
    if(reject_send) return {};
    ManageRequest request;
    if (wire::DummyManageV2RequestCodec::decode(bytes.data(),bytes.size(),&request)!=wire::CodecResult::kOk)
      throw std::runtime_error("request codec failed");
    requests.push_back(request); return std::make_unique<Handle>(*this);
  }
  /**
   * @brief 编码一条合成响应送入FIFO。
   * @param response 待提供的wire响应。
   * @return 无；codec失败抛异常。
   */
  void respond(const wire::DummyManageV2Response &response) {
    std::vector<std::uint8_t> bytes(wire::DummyManageV2ResponseCodec::kEncodedSize);
    if (wire::DummyManageV2ResponseCodec::encode(response,bytes.data(),bytes.size())!=wire::CodecResult::kOk)
      throw std::runtime_error("response codec failed");
    responses.push_back(bytes);
  }
};

/**
 * @brief 生成固定boot7/session9/ref1/motion11/management2/profile201响应。
 * @param q 用其request_id关联当前RPC，非原操作ID自动复制。
 * @param phase 默认Prepared2；正例终态显式改Committed3。
 * @param result 默认Pending23，终态改0。
 * @return 合成wire响应；调用者另设operation/run等终态字段。
 */
static wire::DummyManageV2Response reply(const ManageRequest &q,unsigned phase=2,unsigned result=23){
 wire::DummyManageV2Response r;r.set_schema_major(2);r.set_schema_minor(dummy_ref::protocol::kSchemaMinor);r.set_request_id(q.request_id());r.set_boot_epoch(7);r.set_session_id(9);r.set_reference_epoch(1);r.set_motion_epoch(11);r.set_management_epoch(2);r.set_profile_id(201);r.set_receipt(1);r.set_transaction_phase(phase);r.set_result_code(result);return r;
}
/**
 * @brief 驱动同一原请求从Received到多次Query，并量化人工elapsed。
 * @param op 被测操作Cancel/Disable/Renew/Begin。
 * @param early true模拟120000µs后可Committed，false模拟277930µs后。
 * @param wrong 终态响应改session10，预期client抛身份错误。
 * @param missing 永不提供终态，用来检查不会凭Pending成功。
 * @return 对应模式下节流/时刻关系成立；不是固定250ms外层期限控制器。
 */
static bool cadence(ManageOperation op,bool early,bool wrong=false,bool missing=false){
 Transport t;WireManagementClient client(t);ManageRequest original;original.set_operation(wire_value(op));original.set_expected_boot_epoch(7);original.set_expected_session_id(9);original.set_expected_reference_epoch(1);original.set_expected_motion_epoch(10);original.set_expected_management_epoch(1);original.set_expected_profile_id(201);original.set_configured_mask(63);
 const auto rid=client.submit(original,1000000);auto r=reply(t.requests.back(),1);t.respond(r);client.poll(rid); // originalReceived→firstQuery
 std::uint64_t completed=0,query2=0;bool commitKnown=false;unsigned empty=0,pending_queries=0;
 r=reply(t.requests.back());t.respond(r);
 //真实20ms tick，原250k截止；设备变为可COMMIT不自动使Hostreceipt已知。
 for(std::uint64_t elapsed=20000;elapsed<=500000;elapsed+=20000){

   const auto count=t.requests.size();auto result=client.poll(rid);
   if(result && result->succeeded){completed=elapsed;break;}
   if(result && !result->result_valid)++pending_queries;
   if(t.requests.size()>count){if(!query2)query2=elapsed;
     r=reply(t.requests.back());
     if(pending_queries>=2U && elapsed>=(early?120000U:277930U) && !missing){
       r=reply(t.requests.back(),3,0);r.set_operation_id(19);r.set_run_id(3);r.set_attempt_id(3);r.set_attempt_kind(1);
       if(wrong)r.set_session_id(10);commitKnown=true;
     }
     t.responses.clear();t.respond(r);
   } else if(op!=ManageOperation::kCancel && op!=ManageOperation::kDisableAndRelease && !query2)++empty;
 }
 std::cout<<"CADENCE operation="<<wire_value(op)<<" tick_us=20000 first_followup_query_us="<<query2<<" committed_us="<<completed<<" device_ready_us="<<(early?120000:277930)<<" pending_queries="<<pending_queries<<" original_budget250000 empty="<<empty<<"\n";
 /**
  * @brief actual_late仅要求模拟最终确认>250ms，展示晚物理链无法纳入未变预算。
  * Cancel/Disable后续Query应≤40ms出现；Renew/Begin保留普通退避，首后续Query约220ms。
  */
 if(!early)return completed>250000; // actual late physical chain remains impossible under unchanged budget
 if(missing)return completed==0;
 if(op==ManageOperation::kCancel)return completed>0 && completed<=250000 && query2<=40000;
 if(op==ManageOperation::kDisableAndRelease)return query2<=40000;
 return query2==220000; // original first pending at20k plus9 empty polls+next sends
}
/**
 * @brief 按模式核Query优先级/缺终态/错身份。
 * @param argc 参数个数（含程序名）。
 * @param argv 参数数组，模式/路径位置见下述说明。
 * @return 0表示所选检查通过；非零或断言/require异常表示失败。
 * argv[1]默认cancel；actual_late、renew、begin、no_terminal、wrong_identity各独立。wrong_identity须抛runtime_error才通过，其它异常87，未知模式88。
 */
int main(int argc,char**argv){std::string mode=argc>1?argv[1]:"cancel";
 try {
 if(mode=="cancel")return cadence(ManageOperation::kCancel,true)?0:81;
 if(mode=="actual_late")return cadence(ManageOperation::kCancel,false)?0:82;
 if(mode=="renew")return cadence(ManageOperation::kRenewLease,true)?0:83;
 if(mode=="begin")return cadence(ManageOperation::kBegin,true)?0:84;
 if(mode=="no_terminal")return cadence(ManageOperation::kCancel,true,false,true)?0:85;
 if(mode=="wrong_identity"){try{cadence(ManageOperation::kCancel,true,true);return 86;}catch(const std::runtime_error&){return 0;}}
 }catch(const std::exception&e){std::cerr<<e.what()<<"\n";return 87;}return 88;
}
