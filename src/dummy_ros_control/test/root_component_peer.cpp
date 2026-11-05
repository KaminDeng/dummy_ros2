/** @file root_component_peer.cpp
 * @brief 独立受控Root peer：直接生成Owned及typed服务，无旧业务codec/转换桥。 */
#include "root_component_stimulus.hpp"
#include "dummy_ref/root_transport.hpp"
#include "dummy_ref/root_deployment.hpp"
#include "dummy_ref/root_endpoint_qos.hpp"
#include "dummy_ros_control/message_presence.hpp"
#include "dp_host_instance_identity.h"
#include "dp_comm_port_api.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <new>
#include <poll.h>
#include <thread>
#include <type_traits>
#include <unistd.h>
using State=dummy_ref::messages::State;
using Status=dummy_ref::messages::Status;
using Evidence=dummy_ref::messages::Evidence;
using Command=dummy_ref::messages::Command;
using Management=dummy_ref::root::Manage;
using Stimulus=root_fixture::Stimulus;
using namespace dummy_ros_control;
static bool source_cancel_coordination=false;
static const char* source_cancel_fault=nullptr;
static unsigned release_receipt_fault=0;
static bool persistent_stop_cache_test=false;
static bool replayed_stop_host=false;
static bool inject_peer_state_queue_full_once=false;
struct PersistentStopCache {unsigned round=0;Management::Request stop=owned_fixture::request();Management::Response terminal=owned_fixture::response();dp::comm::SourceId terminal_source{};std::uint64_t request_ids[3]{};};
static PersistentStopCache* persistent_stop_cache=nullptr;
template <typename Publish, typename Pump>
dp::comm::Result publish_peer_with_retry(Publish publish, Pump pump) {
  constexpr unsigned kMaxAttempts = 16U;
  for (unsigned attempt = 0U; attempt < kMaxAttempts; ++attempt) {
    const auto result = publish();
    if (result != dp::comm::Result::kQueueFull || attempt + 1U == kMaxAttempts)
      return result;
    const auto pumped = pump();
    if (pumped != dp::comm::Result::kOk && pumped != dp::comm::Result::kBusy)
      return pumped;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return dp::comm::Result::kQueueFull;
}
struct Peer {
  /**
   * @brief 命令观测pipe写端；传递宿主Command对象，不是把对象内存当作串行协议字节。
   */
  dp::comm::SourceId source{};
  std::uint64_t response_source_boot=7;
  dp::comm::detail::RootEndpoint* management_endpoint=nullptr;
  int commands_fd;
  /**
   * @brief 管理观测pipe写端；父进程按操作号检查实际请求顺序。
   */
  int stops_fd;
  dummy_ref::root::DomainDeviceId domain_device{}; ///< 产品领域ID，与Root HAL UID独立。
  /**
   * @brief 当前Stop原始逻辑请求ID；Query的queried_operation_id必须指向它。
   */
  std::uint64_t stop_request_id = 0U;
  /**
   * @brief Stop提交时的motion锚；回执Pending保持，终态按夹具推进1。
   */
  std::uint32_t stop_motion_epoch = 0U;
  /**
   * @brief Stop提交时的管理代次锚；用于避免Query回执借用别的事务。
   */
  std::uint32_t stop_management_epoch = 0U;
  /**
   * @brief 当前Stop Query次数；通常首次Pending，之后才构造终态。
   */
  unsigned stop_queries = 0U;
  /**
   * @brief 已看到Cancel发送，用于触发重叠Disable的旧tuple拒绝夹具。
   */
  bool overlap_cancel_observed = false;
  /**
   * @brief 标记重叠Disable仍用了错误旧tuple，后续Query给失败而不伪造确认。
   */
  bool overlap_disable_old_tuple = false;
  /**
   * @brief 让所有Stop Query保持Pending，验证cleanup保留unconfirmed而非把超时当成功。
   */
  bool never_complete_stop = false;
  /**
   * @brief 在Pending回执先清session/ref，不能据此冒认release已Committed。
   */
  bool revoke_owner_before_stop_receipt = false;
  /**
   * @brief 记录SelectMode序号水位，Stop必须使用其后继而非从旧计数重新开始。
   */
  bool sequence_rebase_mode = false;
  /**
   * @brief 终态响应故意release_committed=false，即使后来Idle也不应确认Stop。
   */
  bool uncommitted_stop_mode = false;
  /**
   * @brief 最近SelectMode请求序号，夹具用普通大小比较拒绝不前进的Stop；非通用回绕算法。
   */
  std::uint32_t sequence_floor = 0U;
  /**
   * @brief request80的Disable直接返回Committed，跳过本夹具默认的Pending再Query路径。
   */
  bool direct_gate_release = false;
  /**
   * @brief 记录显式release逻辑ID，使后续Query与该release关联。
   */
  std::uint64_t gate_release_request_id = 0U;
};
dp::comm::Result receive_management(const Management::Request &request,
                                    Management::Response *response,
                                    const dp::comm::SourceId &, void *context) {
  auto &peer = *static_cast<Peer *>(context);
  if ((*request.operation()) == 9U) {
    peer.overlap_cancel_observed=true;
    if (!root_fixture::write_value(peer.stops_fd,request)) return dp::comm::Result::kInvalidArgument;
  }
  if ((*request.operation()) == 10U &&
      !root_fixture::write_value(peer.stops_fd,request))
    return dp::comm::Result::kInvalidArgument;
  if ((*request.operation()) == 8U &&
      !root_fixture::write_value(peer.stops_fd,request))
    return dp::comm::Result::kInvalidArgument;
  /**
   * @brief 只有QueryResult指向当前Stop原请求ID才走Stop结果缓存；Query本身request_id必须独立。
   */
  const bool stop_query = (*request.operation()) == 12U && peer.stop_request_id != 0U &&
      (*request.queried_operation_id()) == peer.stop_request_id;
  if (peer.sequence_rebase_mode && (*request.operation()) == 1U)
    peer.sequence_floor = (*request.request_seq());
  if (stop_query && !root_fixture::write_value(peer.stops_fd,request))
    return dp::comm::Result::kInvalidArgument;
  if ((*request.operation()) != 6U && (*request.operation()) != 8U && (*request.operation()) != 0U &&
      (*request.operation()) != 12U && (*request.operation()) != 10U && (*request.operation()) != 1U && (*request.operation()) != 9U)
    return dp::comm::Result::kInvalidArgument;
  *response=owned_fixture::response();

  response->domain_device_id().emplace().assign((request.domain_device_id()->data()), 16);
  response->client_instance_id().emplace().assign((request.client_instance_id()->data()), 16);
  response->request_id()=((*request.request_id()));
  response->receipt()=(1U);
  response->result_code()=(0U);
  /**
   * @brief 普通fixture默认Committed、operation42，特定request45故意给777；request43故意跳两个motion代次。
   * 这些固定应答用于验证Host拒绝不相关/越代回复，不模拟全部设备业务规则。
   */
  response->operation_id()=((*request.request_id()) == 45U ? 777U : 42U);
  response->transaction_phase()=(3U);
  peer.response_source_boot=peer.source.boot_id;
  response->session_id()=((*request.expected_session_id()));
  response->reference_epoch()=((*request.expected_reference_epoch()));
  response->motion_epoch()=((*request.expected_motion_epoch()) +
      ((*request.request_id()) == 43U ? 2U : 1U));
  response->management_epoch()=((*request.expected_management_epoch()) + 1U);
  response->profile_id()=((*request.expected_profile_id()));
  // SOURCE模型只复现once已见TX旧tuple和设备Rejected，不宣未观测CancelCOMMITTED。
  if ((*request.operation())==9U) {
    response->result_code()=(23U);response->transaction_phase()=(1U);response->operation_id()=(0U);
    response->motion_epoch()=((*request.expected_motion_epoch()));
    response->management_epoch()=((*request.expected_management_epoch()));
    return dp::comm::Result::kOk;
  }
  /**
   * @brief 主组件源用固定Cancel Query终态模拟已Committed的新tuple。
   * source_cancel_fault逐项破坏boot/session/ref/run/owner或改Failed/Pending；它不包含真实RobotCore计算。
   */
  if(peer.overlap_cancel_observed && (*request.operation())==12U && (*request.queried_operation_id())==4693763926145815138ULL) {
    response->result_code()=(0U);response->transaction_phase()=(3U);response->operation_id()=(9009U);
    response->run_id()=(9U);response->attempt_id()=(9U);response->attempt_kind()=(1U);
    response->motion_epoch()=(11U);response->management_epoch()=(2U);
    if(source_cancel_fault) {
      if(!std::strcmp(source_cancel_fault,"boot"))peer.response_source_boot=8U;
      if(!std::strcmp(source_cancel_fault,"session"))response->session_id()=(9U);
      if(!std::strcmp(source_cancel_fault,"ref"))response->reference_epoch()=(10U);
      if(!std::strcmp(source_cancel_fault,"run"))response->run_id()=(10U);
      if(!std::strcmp(source_cancel_fault,"owner")){std::uint8_t other[16]{3U};response->client_instance_id().emplace().assign(other, 16);}
      if(!std::strcmp(source_cancel_fault,"failed")){response->result_code()=(10U);response->transaction_phase()=(4U);response->operation_id()=(0U);}
      if(!std::strcmp(source_cancel_fault,"pending")){response->result_code()=(23U);response->transaction_phase()=(2U);response->operation_id()=(0U);}
    }
    return dp::comm::Result::kOk;
  }
  /**
   * @brief 重叠Cancel后仍用旧motion/management提交Disable，fixture保留Pending而让后续Query明确失败。
   * 这里的11/2是共同合成peer分支的约定，不覆盖Core专属28673/77/1/3/4身份。
   */
  if (peer.overlap_cancel_observed && (*request.operation())==10U &&
      ((*request.expected_motion_epoch())!=11U ||(*request.expected_management_epoch())!=2U)) {
    peer.overlap_disable_old_tuple=true;peer.stop_request_id=(*request.request_id());
    peer.stop_motion_epoch=(*request.expected_motion_epoch())+1U;
    peer.stop_management_epoch=(*request.expected_management_epoch())+1U;
    response->result_code()=(23U);response->transaction_phase()=(1U);response->operation_id()=(0U);
    response->motion_epoch()=((*request.expected_motion_epoch()));
    response->management_epoch()=((*request.expected_management_epoch()));
    return dp::comm::Result::kOk;
  }
  if (peer.overlap_disable_old_tuple && stop_query) {
    response->result_code()=(10U);response->transaction_phase()=(4U);response->operation_id()=(0U);
    response->motion_epoch()=(peer.stop_motion_epoch);response->management_epoch()=(peer.stop_management_epoch);
    response->run_id()=(0U);response->release_committed()=(false);response->motion_complete()=(false);
    return dp::comm::Result::kOk;
  }
  /**
   * @brief Complete47先收到Pending回执；Query47才给可关联终态，便于排列Status/State/回执不同先后。
   */
  if ((*request.operation()) == 8U && (*request.request_id()) == 47U) {
    response->operation_id()=(0U);
    response->transaction_phase()=(1U);
    response->result_code()=(23U);
    response->motion_epoch()=((*request.expected_motion_epoch()));
    response->management_epoch()=((*request.expected_management_epoch()));
  /**
   * @brief 显式gate release的Query回执用于注入错operation/motion/management、Pending或未Committed。
   * 故障参数由main的release_deactivate_mode12..16映射，不能被早到Idle缓存绕过。
   */
  } else if ((*request.operation()) == 12U && (*request.queried_operation_id()) == 47U) {
    response->operation_id()=(47U);
  }
  if ((*request.operation()) == 10U) {
    if (peer.sequence_rebase_mode && (*request.request_seq()) <= peer.sequence_floor) {
      response->result_code()=(16U);
      response->transaction_phase()=(4U);
      response->operation_id()=(0U);
      return dp::comm::Result::kOk;
    }
    /**
     * @brief 显式gate release80：默认先Pending，direct_gate_release时直接Committed并清session/ref。
     * 后续是否免重复Stop仍须合法Idle反馈，不能只看这个RPC返回。
     */
    if ((*request.request_id()) == 80U) {
      peer.gate_release_request_id = (*request.request_id());
      response->run_id()=(2U);
      response->attempt_kind()=(1U);
      if (peer.direct_gate_release) {
        response->operation_id()=(80U);
        response->session_id()=(0U);
        response->reference_epoch()=(0U);
        response->release_committed()=(true);
        return dp::comm::Result::kOk;
      }
      response->operation_id()=(0U);
      response->transaction_phase()=(1U);
      response->result_code()=(23U);
      response->motion_epoch()=((*request.expected_motion_epoch()));
      response->management_epoch()=((*request.expected_management_epoch()));
      return dp::comm::Result::kOk;
    }
    /**
     * @brief 跨exec复用模式保存每轮Stop逻辑ID，拒绝任意两Host复用同ID；最多支持三轮。
     */
    if (persistent_stop_cache_test) {
      ++persistent_stop_cache->round;
      if (persistent_stop_cache->round > 3U) return dp::comm::Result::kInvalidArgument;
      for (unsigned index = 0U; index + 1U < persistent_stop_cache->round; ++index)
        if (persistent_stop_cache->request_ids[index] == (*request.request_id()))
          return dp::comm::Result::kInvalidArgument;
      persistent_stop_cache->request_ids[persistent_stop_cache->round - 1U] = (*request.request_id());
    }
    if (persistent_stop_cache_test && persistent_stop_cache->round == 1U) persistent_stop_cache->stop = request;
    peer.stop_request_id = (*request.request_id());
    peer.stop_motion_epoch = (*request.expected_motion_epoch());
    peer.stop_management_epoch = (*request.expected_management_epoch());
    peer.stop_queries = 0U;
    response->operation_id()=(0U);
    response->transaction_phase()=(1U);
    response->result_code()=(23U);
    response->motion_epoch()=(peer.stop_motion_epoch);
    response->management_epoch()=(peer.stop_management_epoch);
    if (peer.revoke_owner_before_stop_receipt) {
      response->session_id()=(0U);
      response->reference_epoch()=(0U);
    }
  } else if (stop_query) {
    ++peer.stop_queries;
    /**
     * @brief 第2轮原样重放首轮完整Query终态，包括旧request/operation关联。
     * 不替换为新Query ID来制造伪相关；新Host即使随后见旧Idle也必须保持Stop未确认。
     */
    if (persistent_stop_cache_test && persistent_stop_cache->round == 2U) {
      // 保留第一进程的完整 QueryResult，不改写 request/operation identity。
      if ((*persistent_stop_cache->terminal.request_id()) == (*request.request_id()) ||
          (*persistent_stop_cache->terminal.operation_id()) != 900U ||
          persistent_stop_cache->terminal_source.boot_id != (*request.expected_boot_epoch()))
        return dp::comm::Result::kInvalidArgument;
      *response = persistent_stop_cache->terminal;
      std::fprintf(stderr, "跨exec peer重放旧回执 old_stop=%llu new_stop=%llu old_reply=%llu op=%llu\n",
          (unsigned long long)(*persistent_stop_cache->stop.request_id()),
          (unsigned long long)peer.stop_request_id,
          (unsigned long long)(*response->request_id()),
          (unsigned long long)(*response->operation_id()));
      return dp::comm::Result::kOk;
    }
    /**
     * @brief 首Query（或never_complete全部Query）只返回Pending/op0；clear owner不等于release_committed。
     * 普通后继Query才清session/ref、推进epochs并发布release标志，uncommitted模式反向验证该标志必需。
     */
    if (peer.stop_queries == 1U || peer.never_complete_stop) {
      response->operation_id()=(0U);
      response->transaction_phase()=(2U);
      response->result_code()=(23U);
      response->motion_epoch()=(peer.stop_motion_epoch);
      response->management_epoch()=(peer.stop_management_epoch);
      if (peer.revoke_owner_before_stop_receipt) {
        response->session_id()=(0U);
        response->reference_epoch()=(0U);
      }
    } else {
      response->operation_id()=(persistent_stop_cache_test
          ? 899U + persistent_stop_cache->round : 900U);
      response->session_id()=(0U);
      response->reference_epoch()=(0U);
      response->motion_epoch()=(peer.stop_motion_epoch + 1U);
      response->management_epoch()=(peer.stop_management_epoch + 1U);
      response->release_committed()=(!peer.uncommitted_stop_mode);
      if (persistent_stop_cache_test && persistent_stop_cache->round == 1U) {
        persistent_stop_cache->terminal = *response;
        persistent_stop_cache->terminal_source.boot_id=peer.response_source_boot;
      }
    }
  } else if ((*request.operation()) == 12U &&
             (*request.queried_operation_id()) == peer.gate_release_request_id) {
    response->run_id()=(2U);
    response->attempt_kind()=(1U);
    response->session_id()=(0U);
    response->reference_epoch()=(0U);
    response->release_committed()=(true);
    if(release_receipt_fault==1U) response->operation_id()=(43U);
    if(release_receipt_fault==2U) response->motion_epoch()=(12U);
    if(release_receipt_fault==3U) response->management_epoch()=(3U);
    if(release_receipt_fault==4U) { response->transaction_phase()=(2U); response->result_code()=(23U); }
    if(release_receipt_fault==5U) response->release_committed()=(false);
  }
  return dp::comm::Result::kOk;
}
int run_peer(const std::string &endpoint, int input, int output, int stops_output,
             bool never_complete_stop, bool revoke_owner_before_stop_receipt,
             bool direct_gate_release, bool sequence_rebase_mode,
             bool uncommitted_stop_mode) {
  // 独立peer可执行文件有自己的真实资源角色；不把3发布者塞入Host PIC的1发布者ABI。
  struct RootPeerOwner {
    alignas(8) std::array<uint8_t,dummy_ref::root::kWorkspaceBytes> workspace{};
    alignas(8) std::array<uint8_t,dummy_ref::root::kRequestBytes> request{};
    alignas(8) std::array<uint8_t,dummy_ref::root::kResponseBytes> response{};
    alignas(8) std::array<uint8_t,dummy_ref::root::kCompletionBytes> completion{};
    std::array<dp::comm::detail::RootEndpoint,8> endpoints{};
    dp::comm::IdentityArena<5,3,8,2,false> identity;
    dp::comm::DeviceSession session;
    explicit RootPeerOwner(uint64_t generation=7) : session({},1,{}) {
      // 软件peer与当前安装SDK使用同一显式HAL部署，领域device01及合成boot不变。
      session.source.device=dummy_ref::root::configured_ref_device();
      session.source.boot_id=generation;session.source.context_id=1;
      session.client_instance=1;session.mode=dp::comm::BindingMode::kManaged;
      session.arena={workspace.data(),workspace.size(),request.data(),response.data(),0,
        completion.data(),completion.size(),1,&identity.state,endpoints.data(),endpoints.size(),request.size(),response.size()};
      identity.state.policy.names=false;identity.state.policy.transport_publishers=16;identity.state.policy.transport_subscriptions=16;
      session.set_endpoint_qos_resolver(&dummy_ref::root::endpoint_qos<dummy_ref::root::QosRole::kRef>);
    }
  } owner;
  using Role=dp::comm::BindingRole;
  const auto declare=[&](uint32_t slot,uint32_t type,uint32_t service,uint32_t endpoint,const char* logical,size_t maxwire) {
    dp::comm::BindingSelector selector{};selector.device=owner.session.source.device;selector.slot=slot;
    selector.role=service?Role::kServer:Role::kPublisher;selector.type=type;selector.service=service;selector.endpoint=service?0:endpoint;
    return owner.identity.state.declare(selector,logical,1,uint16_t(maxwire),1536);
  };
  using Cmd=dp::protocol::selection::dp_protocol_dummy_JointCommand;
  const auto prepare_bindings=[&]() -> int {
#define PEER_DECLARE(SLOT,NAME) do {using D=dp::protocol::selection::dp_protocol_dummy_##NAME; \
  if(declare(SLOT,D::kTypeId,D::kServiceId,D::kEndpointId,D::kLogicalKey,D::kMaxWireSize)!=dp::comm::Result::kOk)return 11;}while(0)
  PEER_DECLARE(dummy_ref::root::kStateSlot,JointState);
  PEER_DECLARE(dummy_ref::root::kStatusSlot,DeviceStatus);
  PEER_DECLARE(dummy_ref::root::kEvidenceSlot,AttemptEvidence);
  PEER_DECLARE(dummy_ref::root::kManageSlot,ManageRequest);
  PEER_DECLARE(dummy_ref::root::kMotorParameterSlot,MotorParameterRequest);
#undef PEER_DECLARE
  dp_host_instance_identity_t host{};if(dp_host_instance_identity_create(&host)!=0)return 12;
  dp::comm::BindingSelector command_selector{};std::memcpy(command_selector.device.bytes,host.device_id,16);
  command_selector.slot=dummy_ref::root::kHostCommandSlot;command_selector.role=Role::kPublisher;
  command_selector.type=Cmd::kTypeId;command_selector.endpoint=Cmd::kEndpointId;
  if(owner.identity.state.desire(command_selector,Cmd::kLogicalKey)!=dp::comm::Result::kOk)return 13;
  // 服务端明确声明已知Host requester；不能从普通RPC正文晋升来源身份。
  using ManageDescriptor=dp::protocol::selection::dp_protocol_dummy_ManageRequest;
  auto requester=command_selector;requester.slot=dummy_ref::root::kHostManageClientSlot;
  requester.role=Role::kClient;requester.type=ManageDescriptor::kTypeId;requester.service=ManageDescriptor::kServiceId;requester.endpoint=0;
  if(owner.identity.state.desire(requester,ManageDescriptor::kLogicalKey)!=dp::comm::Result::kOk)return 15;
  using MotorDescriptor=dp::protocol::selection::dp_protocol_dummy_MotorParameterRequest;
  requester.slot=dummy_ref::root::kHostMotorClientSlot;requester.type=MotorDescriptor::kTypeId;requester.service=MotorDescriptor::kServiceId;
  if(owner.identity.state.desire(requester,MotorDescriptor::kLogicalKey)!=dp::comm::Result::kOk)return 16;
    return 0;
  };
  if(const int result=prepare_bindings())return result;
  dp::comm::StaticStorage storage({1,3,2,0,8,1536,173,3,1,1,3});
  dp::comm::StaticStorage restarted_storage({1,3,2,0,8,1536,173,3,1,1,3});
  bool restarted=false;
  const bool routed_tcp=std::getenv("DUMMY_R11_TCP_ROUTER")!=nullptr;
  dp::comm::TransportConfig config{routed_tcp?"client":"peer", routed_tcp?endpoint.c_str():"", routed_tcp?"":endpoint.c_str(), "component_peer"};
  config.root_device=&owner.session.source.device;
  auto initialized=dp::comm::Context::init(storage,{dp::comm::MemoryMode::kStaticOnly,&config,nullptr});
  if(!initialized.has_value()){std::fprintf(stderr,"root_peer init=%u\n",unsigned(initialized.result()));return 1;}
  auto context=std::move(initialized).take_value();
  dp::comm::NodeOptions options{};options.product_name="dummy_robot";options.device_id=owner.session.source.device;options.node_name="ref_control";
  auto created=context.create_node(options);if(!created.has_value())return 2;
  auto node=std::move(created).take_value();
  dp::comm::TypedSdk sdk(node,owner.session);if(sdk.result()!=dp::comm::Result::kOk){std::fprintf(stderr,"root_peer sdk=%u\n",unsigned(sdk.result()));return 14;}
  using namespace dummy_ref::root;
  auto state_publisher=sdk.publisher<dummy_ref::root::State>(selected::dp_protocol_dummy_JointState::kLogicalKey);
  auto status_publisher=sdk.publisher<Status>(selected::dp_protocol_dummy_DeviceStatus::kLogicalKey);
  auto evidence_publisher=sdk.publisher<Evidence>(selected::dp_protocol_dummy_AttemptEvidence::kLogicalKey);
  Peer peer{};peer.commands_fd=output;peer.stops_fd=stops_output;peer.source=owner.session.source;peer.domain_device[0]=1;peer.never_complete_stop=never_complete_stop;
  peer.revoke_owner_before_stop_receipt=revoke_owner_before_stop_receipt;peer.direct_gate_release=direct_gate_release;
  peer.sequence_rebase_mode=sequence_rebase_mode;peer.uncommitted_stop_mode=uncommitted_stop_mode;
  dp::comm::RootHandle subscription,service,motor_service;
  const auto install_services=[&] {
  subscription=sdk.subscribe<dummy_ref::root::Command>(Cmd::kLogicalKey,
    +[](const dummy_ref::root::Command& root,const dp::comm::ReceiveInfo& info,void* context) {
      if(root.domain_device_id() && root.domain_device_id()->size()==16 && info.source().boot_id &&
         std::equal(root.domain_device_id()->begin(),root.domain_device_id()->end(),static_cast<Peer*>(context)->domain_device.begin()))
        (void)root_fixture::write_value(static_cast<Peer*>(context)->commands_fd,root);
    },&peer);
  service=sdk.service<dummy_ref::root::Manage>(selected::dp_protocol_dummy_ManageRequest::kLogicalKey,
    +[](const dp::msg::dummy::ManageRequest& root,dp::msg::dummy::ManageResponse& output,const dp::comm::ReceiveInfo& info,void* context) {
      auto& peer=*static_cast<Peer*>(context);
      if(!info.source().boot_id || !message_fields_present(root))return;
      if(receive_management(root,&output,info.source(),context)==dp::comm::Result::kOk) {
        if(peer.management_endpoint)peer.management_endpoint->source.boot_id=peer.response_source_boot;
        std::fprintf(stderr,"ROOT_MANAGE_INVOKED request=%llu\n",(unsigned long long)*root.request_id());
      }
    },&peer);
  motor_service=sdk.service<dummy_ref::root::MotorParameter>(selected::dp_protocol_dummy_MotorParameterRequest::kLogicalKey,
    +[](const dp::msg::dummy::MotorParameterRequest& request,dp::msg::dummy::MotorParameterResponse& response,const dp::comm::ReceiveInfo&,void*) {
      if(!request.sequence() || !request.value() || !request.node() || !request.parameter() || !request.version())return;
      response.sequence()=*request.sequence();response.value()=*request.value();response.node()=*request.node();
      response.parameter()=*request.parameter();response.status()=0;response.version()=*request.version();
      std::fprintf(stderr,"ROOT_MOTOR_INVOKED sequence=%u\n",*request.sequence());
    },nullptr);
  };
  install_services();
  for(auto& endpoint:owner.endpoints)if(endpoint.in_use && endpoint.kind==2 && endpoint.request && endpoint.request->wire_type==selected::dp_protocol_dummy_ManageRequest::kTypeId)peer.management_endpoint=&endpoint;
  if(subscription.result()!=dp::comm::Result::kOk || service.result()!=dp::comm::Result::kOk){std::fprintf(stderr,"root_peer sub=%u service=%u\n",unsigned(subscription.result()),unsigned(service.result()));return 3;}
  dp::comm::StaticExecutor executor;
  if(executor.add_node(node)!=dp::comm::Result::kOk || node.activate()!=dp::comm::Result::kOk){std::fprintf(stderr,"root_peer activate failed\n");return 4;}
  std::array<int16_t,3> malformed_writers{{-1,-1,-1}};
  /** @brief 仅测试发送叶层绕过TypedPublisher的完整组检查，正式Traits编码真正缺失字段。
   * @details 同一caller Owned RootEndpoint分配sample token，正式instance_key/Source28，
   * 不手写WireID、token位布局或Root字段；接收端仍是完整生产SDK/DeviceTransport。 */
  const auto publish_partial_state=[&](const auto& value,uint64_t invalid_boot=0,bool invalid_file_id=false) {
    using T=std::decay_t<decltype(value)>;
    using D=std::conditional_t<std::is_same_v<T,State>,selected::dp_protocol_dummy_JointState,
      std::conditional_t<std::is_same_v<T,Status>,selected::dp_protocol_dummy_DeviceStatus,selected::dp_protocol_dummy_AttemptEvidence>>;
    auto& malformed_state_writer=malformed_writers[std::is_same_v<T,State>?0:std::is_same_v<T,Status>?1:2];
    dp::comm::detail::RootEndpoint* endpoint=nullptr;
    for(auto& candidate:owner.endpoints)if(candidate.in_use && candidate.kind==0 && candidate.request && candidate.request->wire_type==D::kTypeId){endpoint=&candidate;break;}
    if(!endpoint)return dp::comm::Result::kInvalidHandle;
    if(malformed_state_writer<0) {
      char key[173]{};
      if(!owner.session.instance_key(D::kLogicalKey,endpoint->source,dp::comm::detail::Role::Sample,key,sizeof(key)))return dp::comm::Result::kTopicTooLong;
      dp_comm_qos_t qos{};qos.priority=3;
      if(dp_comm_port_advertise_root_scoped(key,D::kTypeId,qos,DP_COMM_ROOT_DATA,&malformed_state_writer)!=0)return dp::comm::Result::kNoMemory;
    }
    flatbuffers::bounded::SdkMetadata metadata{};uint32_t token=0;
    if(!dp::comm::detail::encode_token(dp::comm::detail::Role::Sample,endpoint->sequence++,token))return dp::comm::Result::kSequenceExhausted;
    metadata.token=token;metadata.source=dp::comm::detail::to_wire(endpoint->source);
    if(invalid_boot) {auto invalid=endpoint->source;invalid.boot_id=invalid_boot;metadata.source=dp::comm::detail::to_wire(invalid);}
    flatbuffers::bounded::BufferView bytes{};
    if(D::Traits::encode(value,D::policy(),metadata,owner.workspace.data(),owner.workspace.size(),bytes)!=flatbuffers::bounded::Status::ok)return dp::comm::Result::kEncodeError;
    // 只在对抗发送叶层损坏正式FBS文件标识；地址由官方API定位，不手写头偏移或WireID。
    if(invalid_file_id) {auto* identifier=const_cast<char*>(flatbuffers::GetBufferIdentifier(bytes.data));identifier[0]^=0x20;}
    const auto result=dp_comm_port_publish_transport(malformed_state_writer,bytes.data,bytes.size);
    return result==0?dp::comm::Result::kOk:dp::comm::Result::kQueueFull;
  };
  // R在串口场景表示正式Managed Command reader已安装，不能仅以Node激活替代发现完成。
  if(endpoint.rfind("serial/",0)==0 || routed_tcp) {
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(3);
    while(!owner.identity.state.desired[0].ready && std::chrono::steady_clock::now()<deadline) {
      (void)executor.spin_some();std::this_thread::yield();
    }
    if(!owner.identity.state.desired[0].ready){std::fprintf(stderr,"ROOT_INITIAL_BINDING_NOT_READY phase=%u\n",unsigned(owner.identity.state.phase));return 22;}
  }
  const char ready = 'R';
  if (!persistent_stop_cache_test && ::write(output, &ready, 1) != 1) return 5;
  for (;;) {
    pollfd request{input, POLLIN, 0};
    if (::poll(&request, 1, 1) > 0) {
      Stimulus stimulus{};
      if (!root_fixture::read_stimulus(input,stimulus)) return 6;
      if (stimulus.done) {
        // pico serial Context析构会等待reader；worker退出，设备缓存由supervisor保留。
        if (persistent_stop_cache_test) ::_exit(0);
        return 0;
      }
      /** @brief 显式重建测试owner代次；旧Context静默释放后以新Source/SYSTEM重建，绝不改attached Source。
       * @details 第二份固定storage承接新Context，最多一次；正式排他port先关闭后重开，不伪造并行lease。 */
      if(stimulus.root_generation && stimulus.root_generation!=owner.session.source.boot_id) {
        if(restarted)return 17;
        // 先归还全部旧Owner句柄，再复用其地址，保证旧端点静默寿命先结束。
        subscription.reset();service.reset();motor_service.reset();
        state_publisher.reset();status_publisher.reset();evidence_publisher.reset();
        context.shutdown();
        malformed_writers={{-1,-1,-1}};
        auto replacement=dp::comm::Context::init(restarted_storage,{dp::comm::MemoryMode::kStaticOnly,&config,nullptr});
        if(!replacement.has_value()){std::fprintf(stderr,"ROOT_RESTART_CONTEXT=%u\n",unsigned(replacement.result()));return 18;}
        owner.~RootPeerOwner();new(&owner)RootPeerOwner(stimulus.root_generation);peer.source=owner.session.source;peer.management_endpoint=nullptr;
        if(const int result=prepare_bindings())return result;
        context=std::move(replacement).take_value();
        auto replacement_node=context.create_node(options);if(!replacement_node.has_value())return 19;
        node=std::move(replacement_node).take_value();
        sdk.~TypedSdk();new(&sdk)dp::comm::TypedSdk(node,owner.session);
        if(sdk.result()!=dp::comm::Result::kOk){std::fprintf(stderr,"ROOT_RESTART_SDK=%u\n",unsigned(sdk.result()));return 20;}
        state_publisher=sdk.publisher<dummy_ref::root::State>(selected::dp_protocol_dummy_JointState::kLogicalKey);
        status_publisher=sdk.publisher<Status>(selected::dp_protocol_dummy_DeviceStatus::kLogicalKey);
        evidence_publisher=sdk.publisher<Evidence>(selected::dp_protocol_dummy_AttemptEvidence::kLogicalKey);
        install_services();
        for(auto& endpoint:owner.endpoints)if(endpoint.in_use && endpoint.kind==2 && endpoint.request && endpoint.request->wire_type==selected::dp_protocol_dummy_ManageRequest::kTypeId)peer.management_endpoint=&endpoint;
        executor.~StaticExecutor();new(&executor)dp::comm::StaticExecutor;
        if(executor.add_node(node)!=dp::comm::Result::kOk || node.activate()!=dp::comm::Result::kOk)return 21;
        restarted=true;
        // 重启段只发布新Source反馈，不再发送命令；订阅发现随后续executor继续推进。
        // 原R握手要求首个Command reader已就绪，不属于此处新反馈发布的前置条件。
        std::fprintf(stderr,"ROOT_OWNER_REBUILT boot=%llu\n",(unsigned long long)owner.session.source.boot_id);
      }
      /**
       * @par 内部发送回调
       * pump推进同一executor；Status、State、Evidence按该顺序尝试发布，send_*可独立关闭。
       * publish_state可消费一次QueueFull注入；各topic有限重试后任一失败使peer返回7，不能默默丢页还宣称证据完整。
       */
      const auto pump = [&] { const auto result=executor.spin_some();
        if(peer.management_endpoint)peer.management_endpoint->source=peer.source;
        return result; };
      const auto status_result = stimulus.send_status
          ? publish_peer_with_retry([&] { return stimulus.status_source.boot_id && stimulus.status_source.boot_id!=peer.source.boot_id
              ? publish_partial_state(stimulus.status,stimulus.status_source.boot_id,false)
              : status_publisher.publish(stimulus.status); }, pump)
          : dp::comm::Result::kOk;
      const auto publish_state = [&] {
        if (inject_peer_state_queue_full_once) {
          inject_peer_state_queue_full_once = false;
          std::fprintf(stderr, "peer injected state queue full once\n");
          return dp::comm::Result::kQueueFull;
        }
        auto root_value=stimulus.state;
        if(stimulus.omit_state_field==1)root_value.hold_ready().reset();
        if(stimulus.omit_state_field==2 && root_value.joints() && !root_value.joints()->empty())(*root_value.joints())[0].position_rad().reset();
        return stimulus.omit_state_field || stimulus.invalid_root_source_boot || stimulus.invalid_root_file_id || (stimulus.state_source.boot_id && stimulus.state_source.boot_id!=peer.source.boot_id)
          ? publish_partial_state(root_value,stimulus.invalid_root_source_boot ? stimulus.invalid_root_source_boot : stimulus.state_source.boot_id,stimulus.invalid_root_file_id)
          : state_publisher.publish(root_value);
      };
      const auto state_result = stimulus.send_state
          ? publish_peer_with_retry(publish_state, pump)
          : dp::comm::Result::kOk;
      const auto evidence_result = stimulus.send_evidence
          ? publish_peer_with_retry([&] { return stimulus.evidence_source.boot_id && stimulus.evidence_source.boot_id!=peer.source.boot_id
              ? publish_partial_state(stimulus.evidence,stimulus.evidence_source.boot_id,false)
              : evidence_publisher.publish(stimulus.evidence); }, pump)
          : dp::comm::Result::kOk;
      if (status_result != dp::comm::Result::kOk || state_result != dp::comm::Result::kOk ||
          evidence_result != dp::comm::Result::kOk) {
        std::fprintf(stderr, "peer publish failed status=%u state=%u evidence=%u\n",
                     static_cast<unsigned>(status_result), static_cast<unsigned>(state_result),
                     static_cast<unsigned>(evidence_result));
        return 7;
      }
    }
    const auto spun = executor.spin_some();
    if(peer.management_endpoint)peer.management_endpoint->source=peer.source;
    if (spun != dp::comm::Result::kOk && spun != dp::comm::Result::kBusy) return 8;
  }
}
int main(int argc,char** argv){
 if(argc!=13)return 34;
 source_cancel_fault=argv[10][0]?argv[10]:nullptr;
 source_cancel_coordination=source_cancel_fault!=nullptr;
 release_receipt_fault=unsigned(std::strtoul(argv[11],nullptr,10));
 inject_peer_state_queue_full_once=argv[12][0]=='1';
 return run_peer(argv[1],std::atoi(argv[2]),std::atoi(argv[3]),std::atoi(argv[4]),argv[5][0]=='1',argv[6][0]=='1',argv[7][0]=='1',argv[8][0]=='1',argv[9][0]=='1');
}
