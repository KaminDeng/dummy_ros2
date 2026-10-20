/**
 * @file dp_management_transport.cpp
 * @brief 实现管理RPC的独立worker或共享设备worker模式，二者不同时创建通信owner。
 * DeviceWorker只通过同身份共享channel交接字节/状态；StandaloneWorker拥有自己的
 * Context/节点/执行器及单槽请求。原始观测接收锚为宿主steady微秒，独立模式仅做
 * schema/device/profile/mask过滤，没有设备worker完整源时钟、能力和会话校验。
 * 句柄返回/ready/响应复制均不等于设备COMMITTED、运动授权或CAN SENT。
 */
#include "dummy_motion_client/dp_management_transport.hpp"
#include "dummy_ros_control/diagnostic_scope.hpp"
#include "dummy_ros_control/diagnostic_json.hpp"
#include "dp_comm/dp_comm.hpp"
#include "dp_dummy_robot_v2_dp_comm.hpp"
#include "dummy_ros_control/device_management_channel.hpp"
#include "dummy_ros_control/management_admission_retry.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <mutex>
#include <thread>
#include <stdexcept>
namespace dummy_motion_client {
/**
 * @brief 生成wire codec别名；Service使用定长raw管理服务，避免把ROS消息结构体内存当载荷。
 */
namespace wire=dp::protocol::dummy_robot_v2;
using Service=dp::protocol::services::DummyManageV2RawFixed;
namespace {
/**
 * @brief 本地拒绝类别：未ready、无channel、长度错、锁忙、queued占槽和codec失败；不是设备fault/result编号。
 */
enum class AdmissionReason : std::size_t {
  kNotReady, kNoChannel, kWrongSize, kLockBusy, kQueued, kDecodeFailed, kCount
};
/**
 * @brief 记录准入失败的固定诊断，未安装scope时按类别限64次同步stderr。
 * @param reason 本地准入原因。
 * @param bytes 原管理请求字节，尝试解码仅为提取日志身份/请求ID。
 * @note 有scope写journal而不格式化stderr；无scope的失败路径含I/O，不能将所有send调用称为硬实时无I/O。
 */
void log_admission(AdmissionReason reason,const std::vector<std::uint8_t> &bytes) noexcept {
  if(auto* journal=dummy_ros_control::current_host_diagnostics) {
    Service::Request request;
    const bool valid=wire::DummyManageV2RequestCodec::decode(bytes.data(),bytes.size(),&request)==wire::CodecResult::kOk;
    const dummy_ros_control::DiagnosticIdentity expected{request.expected_boot_epoch(),request.expected_session_id(),
      request.expected_reference_epoch(),request.expected_motion_epoch(),request.request_seq(),request.expected_management_epoch(),valid};
    journal->reject(dummy_ros_control::DiagnosticReason::kManagementRejected,{},expected,management_now_us(),
      "management_admission",static_cast<std::uint32_t>(reason));
    journal->detail("MANAGE_ADMISSION_DIAG reason=%u bytes=%zu",static_cast<unsigned>(reason),bytes.size());
    return;
  }
  static std::atomic<unsigned> counts[static_cast<std::size_t>(AdmissionReason::kCount)]{};
  constexpr const char *names[]={"not_ready","no_channel","wrong_size","lock_busy","queued","decode_failed"};
  const auto index=static_cast<std::size_t>(reason);
  if(counts[index].fetch_add(1,std::memory_order_relaxed)>=64U) return;
  Service::Request request;
  const bool decoded=wire::DummyManageV2RequestCodec::decode(bytes.data(),bytes.size(),&request)==wire::CodecResult::kOk;
  std::fprintf(stderr,"MANAGE_ADMISSION_DIAG reason=%s request_id=%llu bytes=%zu\n",
      names[index],static_cast<unsigned long long>(decoded ? request.request_id() : 0U),bytes.size());
}
}  // namespace
/**
 * @brief 读取本管理传输的宿主监督时钟。
 * @return steady_clock epoch以来µs，不能直接减设备produced_at或ROS仿真时间。
 */
std::uint64_t management_now_us() noexcept {
  return std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}
/**
 * @brief 唯一实现对象，冻结参数和模式；独立线程/请求缓存与共享channel准入owner分开管理。
 */
struct DpManagementTransport::Impl {
  /**
   * @brief 共享模式本地等待句柄，拥有请求shared_ptr；可先在有限admission阶段，成功排入channel后再等worker响应。
   * 析构只置canceled，不保证已发送远端mutation撤回，也不停止共享worker。
   */
  struct SharedHandle final : ManagementTransport::RequestHandle {
    /**
     * @brief 构造已经排入共享单槽的等待句柄。
     * @param value 已复制完整请求的共享对象，保证回调context寿命。
     */
    explicit SharedHandle(std::shared_ptr<dummy_ros_control::DeviceManagementRequest> value):pending(std::move(value)) {}
    /**
     * @brief 构造尚因共享主锁忙而未入槽的有限准入句柄。
     * @param value 仅Host RAM中保存的请求。
     * @param channel 弱借用目标channel，不能把它存活当ready。
     * @param owner 本transport生命周期准入标志，stop使旧owner失效。
     * @param started_us 原始宿主µs起点；100ms总预算、2ms间隔、最多50次重试安排不因poll续期。
     */
    SharedHandle(std::shared_ptr<dummy_ros_control::DeviceManagementRequest> value,
                 const std::shared_ptr<dummy_ros_control::DeviceManagementChannel> &channel,
                 const std::shared_ptr<std::atomic<bool>> &owner, std::uint64_t started_us)
        :pending(std::move(value)),admission_channel(channel),admission_owner(owner),
         admission(std::in_place,started_us,true),admitted(false) {
      (void)admission->on_result(dp::comm::Result::kBusy,started_us,false);
    }
    /**
     * @brief 撤销本地请求观察；worker可能已提交远端操作，取消标志不是设备取消/释放回执。
     */
    ~SharedHandle() override { pending->canceled=true; }
    /**
     * @brief 非等待地推进共享准入或交付一次完整响应。
     * @return 未ready到退避时刻/未complete/已交付返回空，成功返回拥有响应副本。
     * @throws std::runtime_error 请求failed/canceled、owner/channel失效、准入超时或槽占用。
     * @note try_lock不等共享主锁，但shared_ptr/vector可能分配或有内部同步，不能当零分配硬实时API。
     */
    std::optional<std::vector<std::uint8_t>> poll() override {
      if(pending->failed || pending->canceled) throw std::runtime_error("shared manage_v2 request failed");
      if(!admitted) {
        const auto channel=admission_channel.lock();
        const auto owner=admission_owner.lock();
        const auto now=management_now_us();
        /**
         * @brief 本地准入失败统一锁请求failed并抛异常；不自行发重复管理mutation或设备Stop。
         */
        const auto reject=[this]() {
          pending->failed=true;
          throw std::runtime_error("shared manage_v2 admission failed");
        };
        if(!channel || !owner || !owner->load() || !channel->ready.load() || admission->expired(now)) reject();
        if(!admission->ready(now)) return {};
        std::unique_lock<std::mutex> lock(channel->mutex,std::try_to_lock);
        if(!lock) {
          if(admission->on_result(dp::comm::Result::kBusy,now,false)==
             dummy_ros_control::ManagementAdmissionRetry::Decision::kFail) reject();
          return {};
        }
        if(!owner->load() || !channel->ready.load() || pending->canceled || pending->failed ||
           (channel->queued && !channel->queued->canceled)) reject();
        if(admission->on_result(dp::comm::Result::kOk,now,false)!=
           dummy_ros_control::ManagementAdmissionRetry::Decision::kAccepted) reject();
        if(!owner->load() || pending->canceled || pending->failed) reject();
        /**
         * @brief 锁内再次核owner/ready与原预算后才发布共享请求，admitted以后不再重复入槽；仅代表worker可消费。
         */
        channel->queued=pending;
        admitted=true;
        admission.reset();
      }
      if(delivered || !pending->complete.load(std::memory_order_acquire)) return {};
      delivered=true;
      return std::vector<std::uint8_t>(pending->response.begin(),pending->response.end());
    }
    /**
     * @brief 共享拥有请求及响应字节；complete由worker在字节填完后release发布，poll以acquire读取。
     */
    std::shared_ptr<dummy_ros_control::DeviceManagementRequest> pending;
    /**
     * @brief 一次性交付标志，仅本地poll上下文修改，防止同响应重复交付。
     */
    bool delivered=false;
    /**
     * @brief 未准入阶段的弱目标，连接消失不靠句柄延长其ready或权限。
     */
    std::weak_ptr<dummy_ros_control::DeviceManagementChannel> admission_channel;
    /**
     * @brief 弱生命周期门，stop后尚未准入的旧句柄必须失败；下次start可创建新的owner。
     */
    std::weak_ptr<std::atomic<bool>> admission_owner;
    /**
     * @brief 只在未准入阶段存在的有限重试预算，不处理已发送RPC的超时重发。
     */
    std::optional<dummy_ros_control::ManagementAdmissionRetry> admission;
    /**
     * @brief 已经实际写入channel单槽时为true，不能当作设备接纳位。
     */
    bool admitted=true;
  };
  /**
   * @brief 独立模式的请求共享对象：响应vector由mutex保护，canceled/failed供控制线程与worker交接。
   */
  struct Pending {
    /**
     * @brief 独立模式拥有的完整wire请求值副本，不借用调用方vector。
     */
    ManageRequest request;
    /**
     * @brief 该专用mutex保护响应optional的写入/移动，poll仅try_lock，回调使用普通lock_guard。
     */
    std::mutex mutex;
    std::optional<std::vector<std::uint8_t>> response;
    /**
     * @brief 本地撤销/传输失败标志；不描述设备操作终态。
     */
    std::atomic<bool> canceled{false}, failed{false};
  };
  /**
   * @brief 独立模式请求句柄，拥有Pending，析构通知本地worker取消等待。
   */
  struct Handle final : ManagementTransport::RequestHandle {
    /**
     * @brief 保存独立请求的共享所有权。
     * @param value 已复制wire请求的非空Pending。
     */
    explicit Handle(std::shared_ptr<Pending> value): pending(std::move(value)) {}
    /**
     * @brief 置本地canceled，实际原生RequestHandle由独立worker同线程reset。
     */
    ~Handle() override { pending->canceled=true; }
    /**
     * @brief try_lock移动交付一次独立响应vector，取走后清optional。
     * @return 锁忙/未回应为空，成功为拥有字节副本。
     * @throws std::runtime_error worker报告传输失败。
     * @note 本方法未独立检查canceled，撤销主要通过句柄析构和worker消费完成。
     */
    std::optional<std::vector<std::uint8_t>> poll() override {
      if(pending->failed) throw std::runtime_error("manage_v2 transport request failed");
      std::unique_lock<std::mutex> lock(pending->mutex,std::try_to_lock);
      if(!lock || !pending->response) return {};
      auto result=std::move(pending->response); pending->response.reset(); return result;
    }
    /**
     * @brief 独立句柄持有请求共享寿命；不拥有Context或通信线程。
     */
    std::shared_ptr<Pending> pending;
  };
  /**
   * @brief 冻结端点/身份及连接模式，构造共享注册表查找所需参数。
   * @param value 值配置，构造不访问设备。
   * @param mode 明确StandaloneWorker或DeviceWorker。
   */
  explicit Impl(DpManagementParameters value,Connection mode): parameters(std::move(value)),connection(mode),
      shared_parameters{parameters.endpoint,parameters.device,parameters.client,parameters.profile,parameters.mask} {}
  /**
   * @brief 不可变设备端点、device/client、profile/mask；这些本地参数不是设备在线能力证据。
   */
  const DpManagementParameters parameters;
  /**
   * @brief 对象寿命内固定连接模式，防止临时调用隐式增加第二通信owner。
   */
  const Connection connection;
  /**
   * @brief 共享channel仅按endpoint/device/client/profile/mask定位，其他transport限制不在此补造。
   */
  const dummy_ros_control::TransportParameters shared_parameters;
  /**
   * @brief 控制侧最近观测值缓存，离线/锁忙可返回旧值；缓存返回不重写接收锚。
   */
  std::optional<DeviceObservation> last_observation;
  /**
   * @brief 通过shared_ptr原子load/store交接的借用缓存，存活不等于channel.ready或许可有效。
   */
  mutable std::shared_ptr<dummy_ros_control::DeviceManagementChannel> cached_channel;
  /**
   * @brief 优先复用仍ready缓存，否则try一次注册表查找并更新缓存。
   * @return 共享channel或旧缓存/空；返回旧离线对象可用于读取历史证据，不是新的控制准入。
   */
  std::shared_ptr<dummy_ros_control::DeviceManagementChannel> shared_channel() const noexcept {
    const auto cached=std::atomic_load(&cached_channel);
    if(cached && cached->ready.load(std::memory_order_acquire)) return cached;
    const auto fresh=dummy_ros_control::DeviceTransport::management_channel(shared_parameters);
    if(fresh) { std::atomic_store(&cached_channel,fresh); return fresh; }
    return cached;
  }
  /**
   * @brief 独立worker寿命与端点创建就绪标志；共享模式ready读取外部channel。
   */
  std::atomic<bool> running{false}, connected{false};
  /**
   * @brief 本对象准入生命周期门，stop置false；只阻止尚未入槽句柄，不证明远端请求已撤销。
   */
  std::shared_ptr<std::atomic<bool>> admission_owner=std::make_shared<std::atomic<bool>>(true);
  /**
   * @brief Standalone独有通信线程；其Context、原生请求句柄与所有dp_comm端点均同线程生灭。
   */
  std::thread worker;
  /**
   * @brief 独立主槽及观测组合由该mutex保护；外部send/observe try_lock，worker取槽/回调可普通加锁。
   */
  std::mutex mutex;
  std::shared_ptr<Pending> queued;
  /**
   * @brief 独立模式最近通过schema/device/profile/mask过滤的字节及Host接收锚，无完整源时钟/权限验证。
   */
  DeviceObservation observation;
  /**
   * @brief 独立worker RPC响应回调，只编码复制字节，不解释设备result/phase。
   * @param result dp_comm本次RPC结果。
   * @param response 回调期借用响应，可空。
   * @param context 非空Pending，由active shared_ptr保持存活。
   * @note 匿名ResponseInfo当前不用；失败置failed，成功mutex内发布response vector。
   */
  static void reply(dp::comm::Result result,const Service::Response *response,
                    const dp::comm::ResponseInfo &,void *context) {
    auto &pending=*static_cast<Pending *>(context);
    if(result!=dp::comm::Result::kOk || !response) { pending.failed=true; return; }
    std::vector<std::uint8_t> bytes(wire::DummyManageV2ResponseCodec::kEncodedSize);
    if(wire::DummyManageV2ResponseCodec::encode(*response,bytes.data(),bytes.size())!=wire::CodecResult::kOk) {
      pending.failed=true; return;
    }
    std::lock_guard<std::mutex> lock(pending.mutex); pending.response=std::move(bytes);
  }
  /**
   * @brief 复制借用消息，做最小schema/device/profile/mask过滤后缓存原codec字节。
   * @tparam Message State或Status生成类型。
   * @tparam Codec 对应wire codec。
   * @tparam IsState true写State缓存，false写Status。
   * @param borrowed 仅回调期有效，必须copy_to而非保存借用地址。
   * @param context 本独立Impl。
   * @note 接收锚取缓存写入时的Host steady µs；本路径不核源produced年龄、epoch授权或能力hash，不等同DeviceTransport接纳State。
   */
  template<typename Message,typename Codec,bool IsState>
  static void receive(dp::comm::BorrowedMessage<Message> borrowed,void *context) {
    Message value; if(borrowed.copy_to(&value)!=dp::comm::Result::kOk) return;
    auto &self=*static_cast<Impl *>(context);
    if(value.schema_major()!=2 || value.schema_minor()!=wire::kSchemaMinor ||
       value.profile_id()!=self.parameters.profile || value.configured_mask()!=self.parameters.mask ||
       !std::equal(self.parameters.device.begin(),self.parameters.device.end(),value.device_id())) return;
    std::vector<std::uint8_t> bytes(Codec::kEncodedSize);
    if(Codec::encode(value,bytes.data(),bytes.size())!=wire::CodecResult::kOk) return;
    std::lock_guard<std::mutex> lock(self.mutex);
    if constexpr(IsState) { self.observation.state=std::move(bytes); self.observation.state_received_us=management_now_us(); }
    else { self.observation.status=std::move(bytes); self.observation.status_received_us=management_now_us(); }
  }
  /**
   * @brief 独立worker建立StaticOnly Context、管理client和State/Status订阅，串行处理单槽。
   * 2ms是spin_some的空闲等待预算，不是回调总耗时上界。退出后connected/running清零并将未处理queued标failed；std异常在本线程同步stderr。
   */
  void run() noexcept {
    std::shared_ptr<Pending> active;
    try {
      dp::comm::StaticStorage storage({1,0,2,0,8,wire::DummyDeviceStatusV2Codec::kEncodedSize,96,0,1,1,1});
      const dp::comm::TransportConfig config{"client",parameters.endpoint.c_str(),"","device_action_gate"};
      auto initialized=dp::comm::Context::init(storage,{dp::comm::MemoryMode::kStaticOnly,&config,nullptr});
      if(!initialized.has_value()) throw std::runtime_error("dp_comm init");
      auto context=std::move(initialized).take_value();
      dp::comm::NodeOptions options{}; options.product_name="dummy_robot"; options.node_name="action_gate";
      std::copy(parameters.client.begin(),parameters.client.end(),options.device_id.bytes);
      auto made=context.create_node(options); if(!made.has_value()) throw std::runtime_error("dp_comm node");
      auto node=std::move(made).take_value();
      const char *hex="0123456789abcdef"; std::string route="/dpsdk/dummy_robot/";
      for(auto byte:parameters.device) { route+=hex[byte>>4]; route+=hex[byte&15]; }
      route+="/ref_control/";
      auto created=node.create_client<Service>((route+"manage_v2").c_str());
      auto states=node.create_subscription<wire::DummyJointStateV2>((route+"state_v2").c_str(),{},
          receive<wire::DummyJointStateV2,wire::DummyJointStateV2Codec,true>,this);
      auto statuses=node.create_subscription<wire::DummyDeviceStatusV2>((route+"status_v2").c_str(),{},
          receive<wire::DummyDeviceStatusV2,wire::DummyDeviceStatusV2Codec,false>,this);
      if(!created.has_value() || !states.has_value() || !statuses.has_value()) throw std::runtime_error("dp_comm entities");
      auto client=std::move(created).take_value(); auto state=std::move(states).take_value();
      auto status=std::move(statuses).take_value(); dp::comm::StaticExecutor executor;
      if(executor.add_node(node)!=dp::comm::Result::kOk || node.activate()!=dp::comm::Result::kOk)
        throw std::runtime_error("dp_comm activation");
      // Native handle is destroyed before client, node, context and storage, always on this worker.
      /**
       * @brief 原生请求句柄声明在client/node/context/storage之后，保证反向析构先结束请求借用，所有操作同worker。
       */
      std::optional<dp::comm::RequestHandle<Service>> native;
      connected=true;
      while(running) {
        if(active && active->canceled) { native.reset(); active.reset(); }
        std::shared_ptr<Pending> next;
        { std::lock_guard<std::mutex> lock(mutex); next=std::move(queued); }
        if(next) {
          /**
           * @brief 新单槽取出后替换旧本地等待句柄，active继续持有回调context；不声称远端旧mutation因此撤回。
           */
          native.reset(); active=std::move(next);
          auto request=client.async_request(active->request,reply,active.get(),dp::comm::Duration::from_milliseconds(1000));
          if(!request.has_value()) active->failed=true;
          else native.emplace(std::move(request).take_value());
        }
        const auto spun=executor.spin_some(dp::comm::Duration::from_milliseconds(2));
        if(spun!=dp::comm::Result::kOk && spun!=dp::comm::Result::kBusy)
          throw std::runtime_error("dp_comm spin result "+std::to_string(static_cast<unsigned>(spun)));
      }
    } catch(const std::exception &error) {
      fprintf(stderr,"device management transport: %s\n",error.what());
      if(active) active->failed=true;
    } catch(...) { if(active) active->failed=true; }
    connected=false; running=false;
    std::lock_guard<std::mutex> lock(mutex);
    if(queued) queued->failed=true;
    queued.reset();
  }
};
/**
 * @brief 创建唯一Impl，不启动独立网络或共享worker。
 * @param parameters 明确部署端点和身份值。
 * @param mode 连接模式，硬件集成应显式选DeviceWorker。
 */
DpManagementTransport::DpManagementTransport(DpManagementParameters parameters,Connection mode):impl_(std::make_unique<Impl>(std::move(parameters),mode)) {}
/**
 * @brief 撤销本对象准入owner并join自有worker；共享模式不关闭硬件worker或主动失能设备。
 */
DpManagementTransport::~DpManagementTransport() { stop(); }
/**
 * @brief 校验本地端点/非零ID/profile/mask并启动所选模式。
 * @return 共享模式建立owner或独立线程启动成功为true；不是在线ready、State新鲜或设备授权证明。
 */
bool DpManagementTransport::start() {
  const auto &p=impl_->parameters;
  if(p.endpoint.empty() || p.endpoint.size()>256 || p.endpoint.find_first_of(" \t\r\n")!=std::string::npos ||
     p.profile==0 || (p.mask!=1 && p.mask!=63) ||
     std::all_of(p.device.begin(),p.device.end(),[](auto v){return v==0;}) ||
     std::all_of(p.client.begin(),p.client.end(),[](auto v){return v==0;})) return false;
  if(impl_->connection==Connection::kDeviceWorker) {
    if(!impl_->admission_owner->load()) impl_->admission_owner=std::make_shared<std::atomic<bool>>(true);
    return true;
  }
  if(impl_->running.exchange(true)) return true;
  if(impl_->worker.joinable()) impl_->worker.join();
  try { impl_->worker=std::thread([this]{impl_->run();}); }
  catch(...) { impl_->running=false; return false; }
  return true;
}
/**
 * @brief 设置共享worker的受限inactive prime转交标志，不直接发命令。
 * @param allowed 准备保持流程是否允许转交；其余身份/票据/Stop条件仍由worker核对。
 * @return 共享模式且找到channel时true。
 */
bool DpManagementTransport::allow_inactive_prime(bool allowed) noexcept {
  if (impl_->connection != Connection::kDeviceWorker) return false;
  const auto channel = impl_->shared_channel();
  if (!channel) return false;
  channel->inactive_prime_allowed.store(allowed, std::memory_order_release);
  return true;
}
/**
 * @brief 全部校验后发布配置轴的Complete-Hold逐轴容差。
 * @param values 长度须匹配mask为1或6，每项有限正rad。
 * @return 共享模式/channel/形状/数值全部满足时true，不分部分合法元素提交。
 */
bool DpManagementTransport::set_post_complete_hold_tolerance(const std::vector<double> &values) noexcept {
  const std::size_t count=impl_->parameters.mask==63U ? 6U : 1U;
  if(impl_->connection!=Connection::kDeviceWorker || values.size()!=count) return false;
  const auto channel=impl_->shared_channel();
  if(!channel) return false;
  for(std::size_t axis=0U;axis<values.size() && axis<6U;++axis) {
    if(!std::isfinite(values[axis]) || values[axis]<=0.0) return false;
  }
  for(std::size_t axis=0U;axis<values.size() && axis<6U;++axis)
    channel->post_complete_hold_tolerance_rad[axis].store(values[axis],std::memory_order_release);
  return true;
}
/**
 * @brief 使未准入句柄owner失效并停止/join自有线程；共享channel可能仍ready，函数不发D10或证明设备停止。
 */
void DpManagementTransport::stop() { impl_->admission_owner->store(false); impl_->running=false; if(impl_->worker.joinable()) impl_->worker.join(); }
/**
 * @brief 共享模式从同worker序号空间预留；其他模式或无channel保留建议值。
 * @param proposed 调用方非零request_seq建议。
 * @return 实际预留序号或proposed回退值，不代表事务已发出。
 */
std::uint32_t DpManagementTransport::reserve_sequence(std::uint32_t proposed) noexcept {
  if (impl_->connection != Connection::kDeviceWorker) return proposed;
  const auto channel = impl_->shared_channel();
  return channel ? channel->reserve_request_sequence(proposed) : proposed;
}
/**
 * @brief 查询独立端点或共享channel的局部可用性。
 * @return connected或channel.ready；不检查会话/参考/permit，也不要求本transport的owner仍准入。
 */
bool DpManagementTransport::ready() const noexcept {
  if(impl_->connection==Connection::kDeviceWorker) {
    const auto channel=impl_->shared_channel(); return channel && channel->ready;
  }
  return impl_->connected;
}
/**
 * @brief 通过注册表读取共享诊断并在非控制侧格式化JSON。
 * @return channel诊断字符串或文本null；会分配，不是实时API。
 * @note 此查询不更新control侧cached_channel，注册表只try_lock一次。
 */
std::string DpManagementTransport::diagnostic_json() const {
  // 诊断读取不获取/更新control侧cached_channel共享指针锁；registry只try一次。
  const auto channel=dummy_ros_control::DeviceTransport::management_channel(impl_->shared_parameters);
  return channel ? dummy_ros_control::diagnostic_journal_json(channel->diagnostics) : "null";
}
/**
 * @brief 读取共享collector首错编号。
 * @return 独立模式/无channel为0，共享模式读sticky failure；0不能单独说明捕获已启用或run完整。
 */
std::uint8_t DpManagementTransport::raw_evidence_failure() const noexcept {
  if(impl_->connection!=Connection::kDeviceWorker) return 0U;
  const auto channel=impl_->shared_channel();
  return channel ? channel->raw_evidence_failure.load(std::memory_order_acquire) : 0U;
}
/**
 * @brief 查询共享worker是否配置捕获，不验证文件/终页。
 * @return 独立模式/无channel为false，其余读配置开关。
 */
bool DpManagementTransport::raw_evidence_capture_enabled() const noexcept {
  if(impl_->connection!=Connection::kDeviceWorker) return false;
  const auto channel=impl_->shared_channel();
  return channel && channel->raw_evidence_capture_enabled.load(std::memory_order_acquire);
}
/**
 * @brief 查询Complete后下一Begin的本地证据屏障。
 * @return 共享模式channel标志，独立模式兼容true；不等价完整设备Ready。
 */
bool DpManagementTransport::post_complete_begin_ready() const noexcept {
  if(impl_->connection!=Connection::kDeviceWorker) return true;
  const auto channel=impl_->shared_channel();
  return channel && channel->post_complete_begin_ready.load(std::memory_order_acquire);
}
/**
 * @brief try_lock复制共享worker排空现场。
 * @return 共享/ready且锁可用时值快照，其他为空；调用方仍需核quiescent、身份及原年龄。
 */
std::optional<HoldDrainEvidence> DpManagementTransport::hold_drain_snapshot() const noexcept {
  if(impl_->connection!=Connection::kDeviceWorker) return {};
  const auto channel=impl_->shared_channel();
  if(!channel || !channel->ready.load(std::memory_order_acquire)) return {};
  std::unique_lock<std::mutex> lock(channel->mutex,std::try_to_lock);
  if(!lock) return {};
  const auto &value=channel->hold_drain;
  return HoldDrainEvidence{value.boot_epoch,value.session_id,value.processed_generation,
                           value.reference_epoch,value.motion_epoch,value.committed_seq,
                           value.disposition,value.quiescent,value.observed_us,value.max_state_age_us,
                           value.position,value.tolerance};
}
/**
 * @brief 复制共享writer已flush终页回执，可能为历史离线记录。
 * @return 值或默认空；不检查channel仍ready，须按boot/session/run和proof组合使用。
 * @note flush不是fsync、机械到位或完整释放验收。
 */
RawTerminalIdentity DpManagementTransport::raw_terminal_flushed() const noexcept {
  if(impl_->connection!=Connection::kDeviceWorker) return {};
  const auto channel=impl_->shared_channel();
  if(!channel) return {};
  const auto value=channel->raw_evidence_terminal_flushed();
  return {value.boot_epoch,value.session_id,value.run_id,value.parent_run_id,
    value.attempt_kind,value.final_state,value.first_failure_source,
    value.first_failure_axis_mask,value.evidence_lost,value.motion_complete,
    value.release_committed,value.chain_valid,value.safety_hold_motion_epoch,
    value.safety_hold_valid,value.attempt_motion_epoch,value.fault_stop_motion_epoch,
    value.terminal_received_us,value.terminal_chain,value.full_fault_groups_valid,value.identity_hashes,value.terminal_group_count};
}
/**
 * @brief 读取经过writer与cleanup generation屏障的故障清理专用原始观测。
 * @return 独立模式/无channel为空；可信State接收锚非零才置raw_cleanup_producer_valid。
 * @note 此lane不恢复正常admission、session/参考/permit或运动资格。
 */
DeviceObservation DpManagementTransport::raw_cleanup_observation() const noexcept {
  if(impl_->connection!=Connection::kDeviceWorker)return {};
  const auto channel=impl_->shared_channel();if(!channel)return {};
  const auto v=channel->raw_cleanup_observation();
  DeviceObservation out;out.state.assign(v.state.begin(),v.state.end());out.status.assign(v.status.begin(),v.status.end());
  out.state_received_us=v.state_received_us;out.status_received_us=v.status_received_us;out.raw_cleanup_producer_valid=v.state_received_us!=0U;return out;
}
/**
 * @brief 复制共享writer首非终原页回执。
 * @return 值或空；不单独证明全run完整，仍须核身份/motion及first proof标志。
 */
RawFirstPageWritten DpManagementTransport::raw_first_page_written() const noexcept {
  if(impl_->connection!=Connection::kDeviceWorker) return {};
  const auto channel=impl_->shared_channel();
  if(!channel) return {};
  const auto value=channel->raw_evidence_first_page_written();
  return {value.boot_epoch,value.session_id,value.run_id,
      value.motion_epoch,value.evidence_seq,value.group_count,
      value.attempt_kind,value.category,value.expected_mask,value.submitted_mask,
      value.sent_mask,value.failed_mask,value.final,value.evidence_lost,
      value.chain_valid,value.full_six,value.strict_refresh_successor};
}
/**
 * @brief 复制完整wire请求到所选有限单槽或返回受预算约束的共享准入句柄。
 * @param bytes 原请求codec字节，调用返回后不保留其vector指针。
 * @return 独占本地等待句柄或空；共享主锁忙可返回尚未入queued的SharedHandle。
 * @note queued已有未取消请求立即拒绝，不形成无界队列。句柄存在不等于设备Received，更不等于COMMITTED。
 */
std::unique_ptr<ManagementTransport::RequestHandle> DpManagementTransport::send(const std::vector<std::uint8_t> &bytes) {
  if(!ready()) { log_admission(AdmissionReason::kNotReady,bytes); return {}; }
  if(impl_->connection==Connection::kDeviceWorker) {
    const auto channel=impl_->shared_channel();
    if(!channel) { log_admission(AdmissionReason::kNoChannel,bytes); return {}; }
    if(bytes.size()!=wire::DummyManageV2RequestCodec::kEncodedSize) {
      log_admission(AdmissionReason::kWrongSize,bytes); return {};
    }
    auto pending=std::make_shared<dummy_ros_control::DeviceManagementRequest>();
    std::copy(bytes.begin(),bytes.end(),pending->request.begin());
    std::unique_lock<std::mutex> lock(channel->mutex,std::try_to_lock);
    if(!lock) {
      log_admission(AdmissionReason::kLockBusy,bytes);
      Service::Request decoded;
      if(wire::DummyManageV2RequestCodec::decode(bytes.data(),bytes.size(),&decoded)!=wire::CodecResult::kOk ||
         decoded.schema_major()!=2U || decoded.schema_minor()!=wire::kSchemaMinor || decoded.request_id()==0U) return {};
      return std::make_unique<Impl::SharedHandle>(std::move(pending),channel,impl_->admission_owner,management_now_us());
    }
    if(channel->queued && !channel->queued->canceled) {
      lock.unlock();
      log_admission(AdmissionReason::kQueued,bytes); return {};
    }
    channel->queued=pending; return std::make_unique<Impl::SharedHandle>(std::move(pending));
  }
  auto pending=std::make_shared<Impl::Pending>();
  if(wire::DummyManageV2RequestCodec::decode(bytes.data(),bytes.size(),&pending->request)!=wire::CodecResult::kOk) {
    log_admission(AdmissionReason::kDecodeFailed,bytes); return {};
  }
  std::unique_lock<std::mutex> lock(impl_->mutex,std::try_to_lock);
  if(!lock) { log_admission(AdmissionReason::kLockBusy,bytes); return {}; }
  if(impl_->queued && !impl_->queued->canceled) {
    lock.unlock();
    log_admission(AdmissionReason::kQueued,bytes); return {};
  }
  impl_->queued=pending; return std::make_unique<Impl::Handle>(std::move(pending));
}
/**
 * @brief try_lock复制最新观测，锁忙/共享离线时可返回旧缓存。
 * @return 拥有字节快照或空；返回缓存不刷新接收时刻，应按原锚重新判年龄。
 * @note 独立缓存缺完整源时钟和权限校验；共享观测也须由上层组合具体准入条件。
 */
std::optional<DeviceObservation> DpManagementTransport::observe() {
  if(impl_->connection==Connection::kDeviceWorker) {
    const auto channel=impl_->shared_channel(); if(!channel || !channel->ready) return impl_->last_observation;
    std::unique_lock<std::mutex> lock(channel->mutex,std::try_to_lock); if(!lock) return impl_->last_observation;
    const auto &source=channel->observation; DeviceObservation result;
    if(source.state_received_us) result.state.assign(source.state.begin(),source.state.end());
    if(source.status_received_us) result.status.assign(source.status.begin(),source.status.end());
    result.state_received_us=source.state_received_us; result.status_received_us=source.status_received_us;
    result.rejected_ack_boot_epoch=source.rejected_ack_boot_epoch;
    result.rejected_ack_session_id=source.rejected_ack_session_id;
    result.rejected_ack_reference_epoch=source.rejected_ack_reference_epoch;
    result.rejected_ack_motion_epoch=source.rejected_ack_motion_epoch;
    result.rejected_ack_sequence=source.rejected_ack_sequence;
    result.rejected_ack_received_us=source.rejected_ack_received_us;
    impl_->last_observation=result; return result;
  }
  std::unique_lock<std::mutex> lock(impl_->mutex,std::try_to_lock);
  if(!lock) return impl_->last_observation;
  impl_->last_observation=impl_->observation; return impl_->last_observation;
}
}  // namespace dummy_motion_client
