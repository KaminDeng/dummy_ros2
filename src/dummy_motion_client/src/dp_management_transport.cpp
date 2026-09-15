#include "dummy_motion_client/dp_management_transport.hpp"
#include "dp_comm/dp_comm.hpp"
#include "dp_dummy_robot_v1_dp_comm.hpp"
#include "dummy_ros_control/device_management_channel.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>
#include <stdexcept>
namespace dummy_motion_client {
namespace wire=dp::protocol::dummy_robot_v1;
using Service=dp::protocol::services::DummyManageV1RawFixed;
std::uint64_t management_now_us() noexcept {
  return std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}
struct DpManagementTransport::Impl {
  struct SharedHandle final : ManagementTransport::RequestHandle {
    explicit SharedHandle(std::shared_ptr<dummy_ros_control::DeviceManagementRequest> value):pending(std::move(value)) {}
    ~SharedHandle() override { pending->canceled=true; }
    std::optional<std::vector<std::uint8_t>> poll() override {
      if(pending->failed) throw std::runtime_error("shared manage_v1 request failed");
      if(delivered || !pending->complete.load(std::memory_order_acquire)) return {};
      delivered=true;
      return std::vector<std::uint8_t>(pending->response.begin(),pending->response.end());
    }
    std::shared_ptr<dummy_ros_control::DeviceManagementRequest> pending;
    bool delivered=false;
  };
  struct Pending {
    ManageRequest request;
    std::mutex mutex;
    std::optional<std::vector<std::uint8_t>> response;
    std::atomic<bool> canceled{false}, failed{false};
  };
  struct Handle final : ManagementTransport::RequestHandle {
    explicit Handle(std::shared_ptr<Pending> value): pending(std::move(value)) {}
    ~Handle() override { pending->canceled=true; }
    std::optional<std::vector<std::uint8_t>> poll() override {
      if(pending->failed) throw std::runtime_error("manage_v1 transport request failed");
      std::unique_lock<std::mutex> lock(pending->mutex,std::try_to_lock);
      if(!lock || !pending->response) return {};
      auto result=std::move(pending->response); pending->response.reset(); return result;
    }
    std::shared_ptr<Pending> pending;
  };
  explicit Impl(DpManagementParameters value,Connection mode): parameters(std::move(value)),connection(mode),
      shared_parameters{parameters.endpoint,parameters.device,parameters.client,parameters.profile,parameters.mask} {}
  const DpManagementParameters parameters;
  const Connection connection;
  const dummy_ros_control::TransportParameters shared_parameters;
  std::optional<DeviceObservation> last_observation;
  std::shared_ptr<dummy_ros_control::DeviceManagementChannel> shared_channel() const noexcept {
    return dummy_ros_control::DeviceTransport::management_channel(shared_parameters);
  }
  std::atomic<bool> running{false}, connected{false};
  std::thread worker;
  std::mutex mutex;
  std::shared_ptr<Pending> queued;
  DeviceObservation observation;
  static void reply(dp::comm::Result result,const Service::Response *response,
                    const dp::comm::ResponseInfo &,void *context) {
    auto &pending=*static_cast<Pending *>(context);
    if(result!=dp::comm::Result::kOk || !response) { pending.failed=true; return; }
    std::vector<std::uint8_t> bytes(wire::DummyManageV1ResponseCodec::kEncodedSize);
    if(wire::DummyManageV1ResponseCodec::encode(*response,bytes.data(),bytes.size())!=wire::CodecResult::kOk) {
      pending.failed=true; return;
    }
    std::lock_guard<std::mutex> lock(pending.mutex); pending.response=std::move(bytes);
  }
  template<typename Message,typename Codec,bool IsState>
  static void receive(dp::comm::BorrowedMessage<Message> borrowed,void *context) {
    Message value; if(borrowed.copy_to(&value)!=dp::comm::Result::kOk) return;
    auto &self=*static_cast<Impl *>(context);
    if(value.schema_major()!=1 || value.schema_minor()!=0 ||
       value.profile_id()!=self.parameters.profile || value.configured_mask()!=self.parameters.mask ||
       !std::equal(self.parameters.device.begin(),self.parameters.device.end(),value.device_id())) return;
    std::vector<std::uint8_t> bytes(Codec::kEncodedSize);
    if(Codec::encode(value,bytes.data(),bytes.size())!=wire::CodecResult::kOk) return;
    std::lock_guard<std::mutex> lock(self.mutex);
    if constexpr(IsState) { self.observation.state=std::move(bytes); self.observation.state_received_us=management_now_us(); }
    else { self.observation.status=std::move(bytes); self.observation.status_received_us=management_now_us(); }
  }
  void run() noexcept {
    std::shared_ptr<Pending> active;
    try {
      dp::comm::StaticStorage storage({1,0,2,0,8,618,96,0,1,1,1});
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
      auto created=node.create_client<Service>((route+"manage_v1").c_str());
      auto states=node.create_subscription<wire::DummyJointStateV1>((route+"state_v1").c_str(),{},
          receive<wire::DummyJointStateV1,wire::DummyJointStateV1Codec,true>,this);
      auto statuses=node.create_subscription<wire::DummyDeviceStatusV1>((route+"status_v1").c_str(),{},
          receive<wire::DummyDeviceStatusV1,wire::DummyDeviceStatusV1Codec,false>,this);
      if(!created.has_value() || !states.has_value() || !statuses.has_value()) throw std::runtime_error("dp_comm entities");
      auto client=std::move(created).take_value(); auto state=std::move(states).take_value();
      auto status=std::move(statuses).take_value(); dp::comm::StaticExecutor executor;
      if(executor.add_node(node)!=dp::comm::Result::kOk || node.activate()!=dp::comm::Result::kOk)
        throw std::runtime_error("dp_comm activation");
      // Native handle is destroyed before client, node, context and storage, always on this worker.
      std::optional<dp::comm::RequestHandle<Service>> native;
      connected=true;
      while(running) {
        if(active && active->canceled) { native.reset(); active.reset(); }
        std::shared_ptr<Pending> next;
        { std::lock_guard<std::mutex> lock(mutex); next=std::move(queued); }
        if(next) {
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
DpManagementTransport::DpManagementTransport(DpManagementParameters parameters,Connection mode):impl_(std::make_unique<Impl>(std::move(parameters),mode)) {}
DpManagementTransport::~DpManagementTransport() { stop(); }
bool DpManagementTransport::start() {
  const auto &p=impl_->parameters;
  if(p.endpoint.empty() || p.endpoint.size()>256 || p.endpoint.find_first_of(" \t\r\n")!=std::string::npos ||
     p.profile==0 || (p.mask!=1 && p.mask!=63) ||
     std::all_of(p.device.begin(),p.device.end(),[](auto v){return v==0;}) ||
     std::all_of(p.client.begin(),p.client.end(),[](auto v){return v==0;})) return false;
  if(impl_->connection==Connection::kDeviceWorker) return true;
  if(impl_->running.exchange(true)) return true;
  if(impl_->worker.joinable()) impl_->worker.join();
  try { impl_->worker=std::thread([this]{impl_->run();}); }
  catch(...) { impl_->running=false; return false; }
  return true;
}
bool DpManagementTransport::allow_inactive_prime(bool allowed) noexcept {
  if (impl_->connection != Connection::kDeviceWorker) return false;
  const auto channel = impl_->shared_channel();
  if (!channel) return false;
  channel->inactive_prime_allowed.store(allowed, std::memory_order_release);
  return true;
}
void DpManagementTransport::stop() { impl_->running=false; if(impl_->worker.joinable()) impl_->worker.join(); }
bool DpManagementTransport::ready() const noexcept {
  if(impl_->connection==Connection::kDeviceWorker) {
    const auto channel=impl_->shared_channel(); return channel && channel->ready;
  }
  return impl_->connected;
}
std::unique_ptr<ManagementTransport::RequestHandle> DpManagementTransport::send(const std::vector<std::uint8_t> &bytes) {
  if(!ready()) return {};
  if(impl_->connection==Connection::kDeviceWorker) {
    const auto channel=impl_->shared_channel();
    if(!channel || bytes.size()!=233) return {};
    auto pending=std::make_shared<dummy_ros_control::DeviceManagementRequest>();
    std::copy(bytes.begin(),bytes.end(),pending->request.begin());
    std::unique_lock<std::mutex> lock(channel->mutex,std::try_to_lock);
    if(!lock || (channel->queued && !channel->queued->canceled)) return {};
    channel->queued=pending; return std::make_unique<Impl::SharedHandle>(std::move(pending));
  }
  auto pending=std::make_shared<Impl::Pending>();
  if(wire::DummyManageV1RequestCodec::decode(bytes.data(),bytes.size(),&pending->request)!=wire::CodecResult::kOk) return {};
  std::unique_lock<std::mutex> lock(impl_->mutex,std::try_to_lock);
  if(!lock || (impl_->queued && !impl_->queued->canceled)) return {};
  impl_->queued=pending; return std::make_unique<Impl::Handle>(std::move(pending));
}
std::optional<DeviceObservation> DpManagementTransport::observe() {
  if(impl_->connection==Connection::kDeviceWorker) {
    const auto channel=impl_->shared_channel(); if(!channel || !channel->ready) return impl_->last_observation;
    std::unique_lock<std::mutex> lock(channel->mutex,std::try_to_lock); if(!lock) return impl_->last_observation;
    const auto &source=channel->observation; DeviceObservation result;
    if(source.state_received_us) result.state.assign(source.state.begin(),source.state.end());
    if(source.status_received_us) result.status.assign(source.status.begin(),source.status.end());
    result.state_received_us=source.state_received_us; result.status_received_us=source.status_received_us;
    impl_->last_observation=result; return result;
  }
  std::unique_lock<std::mutex> lock(impl_->mutex,std::try_to_lock);
  if(!lock) return impl_->last_observation;
  impl_->last_observation=impl_->observation; return impl_->last_observation;
}
}  // namespace dummy_motion_client
