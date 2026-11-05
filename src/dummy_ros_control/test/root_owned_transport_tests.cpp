/** @file root_owned_transport_tests.cpp
 * @brief 独立Owned peer经真实PTY/正式SDK向生产DeviceTransport交付完整消息与真实Source；不连设备。 */
#include "root_component_stimulus.hpp"
#include "dummy_ros_control/command_channel.hpp"
#include "dummy_ros_control/device_transport.hpp"
#include "dummy_ros_control/device_management_channel.hpp"
#include "dummy_ros_control/guarded_zenbedded_system.hpp"
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <sys/wait.h>
#include <termios.h>
#include <thread>
#include <type_traits>
#include <unistd.h>
using namespace dummy_ros_control;

class SerialBridge {
 public:
  /**
   * @brief 建立两对非阻塞PTY并取得slave路径。
   * @return 两对open/grant/unlock/ptsname全部成功为true；任一步失败false，已开fd留给析构回收。
   * @note 端点后缀baudrate=115200是transport解析配置，在PTY上不等于真实线速测量。
   */
  bool open_pair() {
    for (int index = 0; index < 2; ++index) {
      masters_[index] = ::posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
      if (masters_[index] < 0 || ::grantpt(masters_[index]) != 0 ||
          ::unlockpt(masters_[index]) != 0) return false;
      char name[128]{};
      if (::ptsname_r(masters_[index], name, sizeof(name)) != 0) return false;
      // PTY初始默认echo会把首个INIT反射给Host；在任何连接前设置叶层原始模式。
      const int slave=::open(name,O_RDWR|O_NOCTTY|O_NONBLOCK);
      termios attributes{};
      if(slave<0)return false;
      if(::tcgetattr(slave,&attributes)!=0){::close(slave);return false;}
      ::cfmakeraw(&attributes);
      const int configured=::tcsetattr(slave,TCSANOW,&attributes);::close(slave);
      if(configured!=0)return false;
      endpoints_[index] = std::string("serial/") + name + "#baudrate=115200";
    }
    return true;
  }
  /**
   * @brief 返回一侧串行slave端点。
   * @param index 调用方限定为0或1；本函数不做越界检查。
   * @return 类内字符串引用，必须在bridge生存期内使用。
   */
  const std::string &endpoint(int index) const { return endpoints_[index]; }
  /**
   * @brief fork后子进程关闭继承的master副本，由父进程桥独占转发。
   * @return 无；不改变父进程自己的fd表。
   */
  void close_child_fds() const { for (int fd : masters_) ::close(fd); }
  /**
   * @brief 开启父进程字节转发线程。
   * @return 无；本夹具每对象只调用一次，不支持重复start自动join旧线程。
   * @par 内部线程
   * 轮询任一master的可读事件，按方向调用forward；running=false后退出，由析构join。
   */
  void start() {
    running_ = true;
    worker_ = std::thread([this] {
      while (running_) {
        pollfd fds[2]{{masters_[0], POLLIN, 0}, {masters_[1], POLLIN, 0}};
        if (::poll(fds, 2, 20) <= 0) continue;
        if (fds[0].revents & POLLIN) forward(0, 1);
        if (fds[1].revents & POLLIN) forward(1, 0);
      }
    });
  }
  /**
   * @brief 停转发、join线程再关闭master；保证线程不继续访问已销毁对象或已关闭fd。
   * @return 无。
   */
  ~SerialBridge() {
    running_ = false;
    if (worker_.joinable()) worker_.join();
    for (int fd : masters_) if (fd >= 0) ::close(fd);
  }
 private:
  /**
   * @brief 非阻塞转发最多一次read取得的512字节批次。
   * @param source 源master索引0/1。
   * @param target 目标master索引0/1。
   * @return 无；EINTR重试，EAGAIN等可写，其他错误/EOF返回而不伪装写满。
   * @note offset只按实际写字节推进；退出running可中断未完成转发，这不是可靠存储队列。
   */
  void forward(int source, int target) {
    std::uint8_t bytes[512];
    const auto count = ::read(masters_[source], bytes, sizeof(bytes));
    if (count <= 0) return;
    for (ssize_t offset = 0; running_ && offset < count;) {
      const auto written = ::write(masters_[target], bytes + offset,
                                   static_cast<std::size_t>(count - offset));
      if (written > 0) offset += written;
      else if (written < 0 && errno == EINTR) continue;
      else if (written < 0 && errno == EAGAIN) {
        pollfd writable{masters_[target], POLLOUT, 0};
        ::poll(&writable, 1, 20);
      } else return;
    }
  }
  /**
   * @brief PTY master拥有句柄，-1表示未创建；析构只关闭非负fd。
   */
  int masters_[2]{-1, -1};
  /**
   * @brief 对应两个slave的transport端点文本，open_pair后固定。
   */
  std::string endpoints_[2];
  /**
   * @brief 父进程线程运行标志；与fork后子进程的副本不是跨进程同步变量。
   */
  std::atomic<bool> running_{false};
  /**
   * @brief 拥有字节桥线程；析构负责join，避免悬空this。
   */
  std::thread worker_;
};
int main(int argc,char** argv){
 const std::string order=argc>1?argv[1]:"none";
 if(order!="none"&&order!="state_first"&&order!="status_first")return 90;
 SerialBridge bridge;if(!bridge.open_pair())return 1;
 int input[2],output[2],stops[2];if(pipe(input)||pipe(output)||pipe(stops))return 2;
 const auto peer=fork();if(peer<0)return 3;
 if(!peer){
  bridge.close_child_fds();close(input[1]);close(output[0]);close(stops[0]);
  char in[16],out[16],stop[16];snprintf(in,sizeof(in),"%d",input[0]);snprintf(out,sizeof(out),"%d",output[1]);snprintf(stop,sizeof(stop),"%d",stops[1]);
  execl(DUMMY_REF_ROOT_COMPONENT_PEER_PATH,DUMMY_REF_ROOT_COMPONENT_PEER_PATH,bridge.endpoint(0).c_str(),in,out,stop,"0","0","0","0","0","","0","0",static_cast<char*>(nullptr));_exit(99);
 }
 struct Cleanup{pid_t pid;~Cleanup(){kill(pid,SIGTERM);int result;waitpid(pid,&result,0);}}cleanup{peer};
 bridge.start();close(input[0]);close(output[1]);close(stops[1]);
 TransportParameters p{};p.endpoint=bridge.endpoint(1);p.device_id[0]=1;p.client_instance_id[0]=2;p.profile_id=1;p.configured_mask=1;
 p.max_state_age_us=1000000;p.control_period_us=10000;p.state_publish_period_us=1000000;
 p.contract_hash[0]=1;p.model_hash[0]=2;p.profile_hash[0]=3;p.firmware_hash[0]=4;p.policy_hash[0]=5;
 p.position_min.fill(-1);p.position_max.fill(1);p.velocity_max.fill(1);p.acceleration_max.fill(2);
 GuardedZenbeddedSystem system;DeviceTransport transport(system,p);if(!transport.start())return 4;
 pollfd ready{output[0],POLLIN,0};char marker=0;if(poll(&ready,1,5000)!=1||read(output[0],&marker,1)!=1||marker!='R')return 5;
 root_fixture::Stimulus value;
 value.state.domain_device_id().emplace().assign(p.device_id.data(),p.device_id.size());
 value.status.domain_device_id().emplace().assign(p.device_id.data(),p.device_id.size());
 value.state.profile_id()=1;value.status.profile_id()=1;value.state.core_state()=1;value.status.core_state()=1;value.status.configured_mask()=1;
 value.status.capabilities()=kPositionOnlyCapability;value.status.control_period_us()=p.control_period_us;value.status.state_publish_period_us()=p.state_publish_period_us;
 value.status.contract_sha256().emplace().assign(p.contract_hash.data(),32);value.status.model_sha256().emplace().assign(p.model_hash.data(),32);
 value.status.profile_sha256().emplace().assign(p.profile_hash.data(),32);value.status.firmware_sha256().emplace().assign(p.firmware_hash.data(),32);value.status.compiled_policy_sha256().emplace().assign(p.policy_hash.data(),32);
 value.status.position_min_rad().emplace().assign(p.position_min.data(),6);value.status.position_max_rad().emplace().assign(p.position_max.data(),6);
 value.status.velocity_max_rad_s().emplace().assign(p.velocity_max.data(),6);value.status.acceleration_max_rad_s().emplace().assign(p.acceleration_max.data(),6);
 for(size_t axis=0;axis<6;++axis){auto& joint=(*value.state.joints())[axis];joint.configured_quality()=axis==0;joint.position_valid_quality()=false;joint.raw_position_valid_quality()=axis==0;}
 const auto channel=DeviceTransport::management_channel(p);bool observed=false;
 // 故意在未接到任何反馈时向现有控制队列交接记录；本地输入不能冒充设备授权。
 const CommandIdentity premature{7,8,9,10,11};std::array<double,6> premature_position{};
 if(!system.submit_control_update(premature,premature_position)||!system.transfer_control_update())return 96;
 // 三条实际接收顺序均从完全未有快照开始；不会为缺失缓存填默认业务值。
 std::this_thread::sleep_for(std::chrono::milliseconds(120));
 {std::lock_guard<std::mutex> lock(channel->mutex);if(channel->observation.state_received_us||channel->observation.status_received_us)return 91;}
 if(transport.statistics().commands_published||transport.statistics().rejected_commands==0)return 97;
 pollfd premature_output{output[0],POLLIN,0};if(poll(&premature_output,1,0)>0)return 98;
 if(order!="none"){
  auto first=value;first.send_state=order=="state_first";first.send_status=order=="status_first";
  first.state.state_seq()=1;first.state.produced_at_us()=5000000000ULL;
  if(!root_fixture::write_stimulus(input[1],first))return 92;
  const auto first_deadline=std::chrono::steady_clock::now()+std::chrono::seconds(2);bool arrived=false;
  while(std::chrono::steady_clock::now()<first_deadline&&!arrived){
   if(order=="state_first")arrived=transport.statistics().state_callbacks>0;
   else {std::lock_guard<std::mutex> lock(channel->mutex);arrived=channel->observation.status_received_us!=0;}
   std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  if(!arrived)return 93;
  {std::lock_guard<std::mutex> lock(channel->mutex);
   if(channel->observation.state_received_us)return 94;
   if(order=="state_first"&&channel->observation.status_received_us)return 95;
  }
 }

 const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(4);
 for(unsigned seq=1;std::chrono::steady_clock::now()<deadline&&!observed;++seq){
  value.state.state_seq()=seq;value.state.produced_at_us()=5000000000ULL+seq*10000ULL;
  if(!root_fixture::write_stimulus(input[1],value))return 6;
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  if(channel){std::lock_guard<std::mutex> lock(channel->mutex);const auto& actual=channel->observation;
   observed=actual.state_received_us&&actual.status_received_us&&actual.state_source.boot_id==7&&actual.status_source.boot_id==7&&*actual.state.profile_id()==1&&*actual.status.profile_id()==1&&actual.state_source.device==actual.status_source.device;
  }
 }
 if(!observed){const auto stats=transport.statistics();std::fprintf(stderr,"owned peer state=%llu status=%llu rejected=%llu bootstrap=%llu\n",(unsigned long long)stats.state_messages,(unsigned long long)stats.status_messages,(unsigned long long)stats.rejected_states,(unsigned long long)stats.bootstrap_rejections);return 7;}
 // 两份大于2^53的业务ID经真正typed RPC分别往返，不能沿用旧raw correlation映射。
 for(unsigned round=0;round<2;++round){
  auto pending=std::make_shared<DeviceManagementRequest>();pending->request=dummy_ref::messages::make_manage_request();
  pending->request.operation()=0;pending->request.expected_boot_epoch()=7;pending->request.expected_profile_id()=1;pending->request.configured_mask()=1;
  pending->request.domain_device_id().emplace().assign(p.device_id.data(),16);pending->request.client_instance_id().emplace().assign(p.client_instance_id.data(),16);
  pending->request.request_id()=UINT64_C(9007199254740993)+round*2;pending->request.request_seq()=17+round;
  {std::lock_guard<std::mutex> lock(channel->mutex);if(channel->queued)return 8;channel->queued=pending;}
  const auto rpc_deadline=std::chrono::steady_clock::now()+std::chrono::seconds(3);
  while(!pending->complete.load(std::memory_order_acquire)&&!pending->failed.load()&&std::chrono::steady_clock::now()<rpc_deadline){
   value.state.state_seq()=*value.state.state_seq()+1;value.state.produced_at_us()=*value.state.produced_at_us()+10000;
   if(!root_fixture::write_stimulus(input[1],value))return 9;
   std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  if(!pending->complete.load(std::memory_order_acquire)||pending->failed.load()||!message_fields_present(pending->response)||
     *pending->response.request_id()!=*pending->request.request_id()||pending->response_source.boot_id!=7)return 10;
 }
 transport.stop();std::printf("ROOT_OWNED_TRANSPORT_PASS order=%s; startup presence; actual Source.boot=7; two full uint64 typed RPC replies\n",order.c_str());return 0;
}
