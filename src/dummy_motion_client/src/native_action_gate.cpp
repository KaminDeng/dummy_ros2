/**
 * @file native_action_gate.cpp
 * @brief 把公开FollowJointTrajectory Action转交内部控制器的Native代理节点。
 * ROS多线程executor处理请求/反馈，非原子等待字段只由一条action worker顺序访问；
 * admission用原子阶段占单槽，回收旧worker时会join。这里不走设备bootstrap、原始证据、
 * USB或CAN确认；Native内部Action结果不能扩张为机械资格。取消响应当前仅检查非空，
 * 没有核对return_code/goals_canceling，必须如实区分“收到响应”和“取消被接受”。
 */
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <unordered_map>
#include <unordered_set>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "dummy_motion_client/native_goal_validation.hpp"
#include "dummy_motion_client/native_wait_policy.hpp"

namespace dummy_motion_client {
/**
 * @brief 公开与内部端点使用相同ROS Action消息类型；Goal包含rad轨迹和秒/纳秒Duration。
 */
using Follow = control_msgs::action::FollowJointTrajectory;
using ServerHandle = rclcpp_action::ServerGoalHandle<Follow>;
using ClientHandle = rclcpp_action::ClientGoalHandle<Follow>;
using namespace std::chrono_literals;

/**
 * @brief 拥有Action client/server和一条等待worker的ROS节点；wait_policy阶段可供admission原子读取。
 */
class NativeActionGate final : public rclcpp::Node {
 public:
  /**
   * @brief 建立内部Action客户端与公开代理server；构造不发目标或连接实体电机。
   * 三个server回调分别负责验证/占槽、接纳外部取消意图、创建唯一执行worker。
   */
  NativeActionGate() : Node("native_action_gate") {
    client_ = rclcpp_action::create_client<Follow>(this, "/_internal_arm_controller/follow_joint_trajectory");
    server_ = rclcpp_action::create_server<Follow>(
        this, "/arm_controller/follow_joint_trajectory",
        /**
         * @brief admission回调只检查Native支持字段/轨迹形状并原子占槽。
         * @note 匿名GoalUUID在此不使用；goal为只读共享目标。成功返回ACCEPT_AND_EXECUTE只是本代理接纳，未获内部Action或设备接受。
         */
        [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Follow::Goal> goal) {
          if (!valid_native_goal(*goal)) return rclcpp_action::GoalResponse::REJECT;
          return wait_policy_.try_acquire()
                     ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE
                     : rclcpp_action::GoalResponse::REJECT;
        },
        /**
         * @brief 外部取消回调固定返回ACCEPT，只接纳代理层取消意图。
         * @note 匿名共享句柄不使用，内部取消响应及终态须由worker另外等待，不能据此说电机已停止。
         */
        [](std::shared_ptr<ServerHandle>) { return rclcpp_action::CancelResponse::ACCEPT; },
        /**
         * @brief accepted回调在mutex下join旧worker后启动一条新线程。
         * @param handle 本次外部Goal共享句柄，按值捕获保证worker使用期存活。
         * @note worker捕获std::exception后锁Fault并尝试Aborted；Fault不因finish自动恢复为Idle。
         */
        [this](std::shared_ptr<ServerHandle> handle) {
          std::lock_guard<std::mutex> lock(worker_mutex_);
          if (worker_.joinable()) worker_.join();
          worker_ = std::thread([this, handle = std::move(handle)] {
            try { execute(handle); }
            catch (const std::exception &error) {
              wait_policy_.latch_fault();
              auto failure=std::make_shared<Follow::Result>();
              failure->error_code=Follow::Result::INVALID_GOAL;
              failure->error_string=std::string("INTERNAL_RESULT_UNCONFIRMED: ")+error.what();
              finish(handle,failure,rclcpp_action::ResultCode::ABORTED);
            }
          });
        });
  }

  /**
   * @brief 在worker_mutex下等待现有worker退出；析构可能等待服务器/结果预算，不发独立设备D10或硬件失能。
   */
  ~NativeActionGate() override {
    std::lock_guard<std::mutex> lock(worker_mutex_);
    if (worker_.joinable()) worker_.join();
  }

 private:
  /**
   * @brief 尝试向外部Goal投递终态，再按既有等待证据结束本地占槽。
   * @param outer 当前外部Goal句柄。
   * @param result 要回传的结果对象。
   * @param code SUCCEEDED/CANCELED走对应终态，其余走abort。
   * @note 外部终态投递异常只记录ROS错误；wait_policy.finish仍执行，不能把本函数返回当客户端一定收到结果。
   */
  void finish(const std::shared_ptr<ServerHandle> &outer, const std::shared_ptr<Follow::Result> &result,
              rclcpp_action::ResultCode code) {
    try {
      if (code == rclcpp_action::ResultCode::SUCCEEDED) outer->succeed(result);
      else if (code == rclcpp_action::ResultCode::CANCELED) outer->canceled(result);
      else outer->abort(result);
    } catch (const std::exception &error) {
      RCLCPP_ERROR(get_logger(), "Action terminal delivery failed: %s", error.what());
    }
    wait_policy_.finish();
  }

  /**
   * @brief 读取worker监督所用宿主单调时钟。
   * @return steady_clock epoch以来µs，不是ROS仿真时间或设备生产时间。
   */
  static std::uint64_t now_us() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  /**
   * @brief 将已验证的非负ROS Duration折成微秒，纳秒不足1µs部分截断。
   * @param duration 相对时长，调用前由Native Goal校验保证秒/纳秒合法。
   * @return sec×1000000+nanosec/1000；此函数不自行验证负秒。
   */
  static std::uint64_t duration_us(const builtin_interfaces::msg::Duration &duration) {
    return static_cast<std::uint64_t>(duration.sec)*1000000U+duration.nanosec/1000U;
  }

  /**
   * @brief 单worker转交原Goal并等待内部结果，必要时请求取消并等待双证据。
   * @param outer 外部Goal共享句柄，持续到本次执行收尾。
   * @note 服务器出现与内部Goal接纳分别最多等待10秒；接纳未确认锁Fault。执行预算为最后路点时间+goal_time_tolerance+10秒，饱和计算。
   */
  void execute(const std::shared_ptr<ServerHandle> &outer) {
    auto failure = std::make_shared<Follow::Result>();
    failure->error_code = Follow::Result::INVALID_GOAL;
    /**
     * @brief 内部服务器10秒内未出现则外部Aborted，仍处Admitted可正常归还占槽；与已发Goal但接纳结果未知的Fault路径分开。
     */
    if (!client_->wait_for_action_server(10s)) {
      finish(outer, failure, rclcpp_action::ResultCode::ABORTED);
      return;
    }
    rclcpp_action::Client<Follow>::SendGoalOptions options;
    /**
     * @brief 反馈回调只在外部Goal仍active时复制并转发内部反馈。
     * @note 匿名内部Goal句柄不使用；反馈共享指针只借用当前回调，不推进wait_policy非原子证据。
     */
    options.feedback_callback = [outer](ClientHandle::SharedPtr,
                                        const std::shared_ptr<const Follow::Feedback> feedback) {
      if (outer->is_active()) outer->publish_feedback(std::make_shared<Follow::Feedback>(*feedback));
    };
    /**
     * @brief 内部Goal迟到且外部已不active时尽力取消，避免代理不再等待却留下内部执行。
     * @param internal 内部Goal句柄，可空代表拒绝；该回调不独立验收取消终态。
     */
    options.goal_response_callback = [this,outer](ClientHandle::SharedPtr internal) {
      if (internal && !outer->is_active()) client_->async_cancel_goal(internal);
    };
    /**
     * @brief 转交完整原Goal而非仅末点。async_send_goal返回future不代表内部已经接纳，10秒超时后状态未知必须锁Fault。
     */
    auto sent = client_->async_send_goal(*outer->get_goal(), options);
    if (sent.wait_for(10s) != std::future_status::ready) {
      wait_policy_.latch_fault();
      failure->error_string="INTERNAL_GOAL_ACCEPTANCE_UNCONFIRMED";
      finish(outer, failure, rclcpp_action::ResultCode::ABORTED);
      return;
    }
    const auto internal = sent.get();
    if (!internal) {
      finish(outer, failure, rclcpp_action::ResultCode::ABORTED);
      return;
    }
    /**
     * @brief 内部接纳后才等待结果；取消响应future和内部终态future分别保留，不以其中单独一项解除取消等待。
     */
    auto result = client_->async_get_result(internal);
    using CancelResponse=rclcpp_action::Client<Follow>::CancelResponse;
    std::optional<std::shared_future<CancelResponse::SharedPtr>> cancel_response;
    bool result_timed_out=false;
    const auto goal=outer->get_goal();
    /**
     * @brief 两个saturating_deadline在这里用于相对预算饱和相加，再由wait_policy.begin加宿主now建立绝对截止。
     * 该时长是软件监督，不保证轨迹在硬件上按此时间到位。
     */
    const auto budget=saturating_deadline(duration_us(goal->trajectory.points.back().time_from_start),
        saturating_deadline(duration_us(goal->goal_time_tolerance),10000000U));
    wait_policy_.begin(now_us(),budget);
    while (rclcpp::ok()) {
      if (!cancel_response && (outer->is_canceling() || wait_policy_.expired(now_us()))) {
        result_timed_out=!outer->is_canceling();
        cancel_response=client_->async_cancel_goal(internal);
        wait_policy_.cancel(now_us(),2000000U);
      }
      /**
       * @brief 每轮最多等待结果future 20ms；回调/线程调度及其余工作不受这个wait_for参数硬抢占。只认三种明确Action终码，UNKNOWN不能当已终结。
       */
      const bool result_ready=result.wait_for(20ms)==std::future_status::ready;
      const bool terminal_confirmed=result_ready &&
          (result.get().code==rclcpp_action::ResultCode::SUCCEEDED ||
           result.get().code==rclcpp_action::ResultCode::CANCELED ||
           result.get().code==rclcpp_action::ResultCode::ABORTED);
      if (terminal_confirmed) wait_policy_.terminal_confirmed();
      if (!cancel_response && terminal_confirmed) {
        const auto wrapped=result.get(); finish(outer,wrapped.result,wrapped.code); return;
      }
      /**
       * @brief 结果future已就绪但没有认可终码，也转入取消而不把它当成功；取消预算首次设2秒，重复调用不延长。
       */
      if (!cancel_response && result_ready) {
        result_timed_out=true;
        cancel_response=client_->async_cancel_goal(internal);
        wait_policy_.cancel(now_us(),2000000U);
      }
      if (!cancel_response) continue;
      /**
       * @brief 当前只检查future就绪且response指针非空，未检查取消返回码或目标列表。
       * 这里只置“取消响应到达”标志，不能据此证明内部已接受取消；仍需独立终态才能cancel_complete。
       */
      if (cancel_response->wait_for(0ms)==std::future_status::ready && cancel_response->get())
        wait_policy_.cancel_acknowledged();
      /**
       * @brief 双证据齐备后：因执行超时而取消则对外Aborted/INTERNAL_RESULT_TIMEOUT；用户取消则回传实际内部终码，不强行伪造成Canceled。
       */
      if (wait_policy_.cancel_complete()) {
        const auto wrapped=result.get();
        if (result_timed_out) {
          failure->error_string="INTERNAL_RESULT_TIMEOUT";
          finish(outer,failure,rclcpp_action::ResultCode::ABORTED);
        } else finish(outer,wrapped.result,wrapped.code);
        return;
      }
      /**
       * @brief 取消两秒仍无完整双证据则锁Fault/Aborted；需重启gate才可接受新目标，不靠清本地future假装停车成功。
       */
      if (wait_policy_.expired(now_us())) {
        wait_policy_.latch_fault();
        failure->error_string="INTERNAL_CANCEL_RESULT_UNCONFIRMED: restart gate before another goal";
        finish(outer,failure,rclcpp_action::ResultCode::ABORTED);
        return;
      }
    }
    /**
     * @brief ROS关闭使等待循环退出时也锁Fault，防止没有内部终态确认却重新开放admission。
     */
    wait_policy_.latch_fault();
  }

  /**
   * @brief phase原子可跨admission/worker读取；deadline、响应/终态位仅worker访问，不是整个策略对象都可多线程写。
   */
  NativeWaitPolicy wait_policy_;
  /**
   * @brief 内部Action客户端由节点拥有，可被executor回调和worker调用其ROS线程安全接口。
   */
  rclcpp_action::Client<Follow>::SharedPtr client_;
  /**
   * @brief 公开代理server，不是设备管理端点或运动许可对象。
   */
  rclcpp_action::Server<Follow>::SharedPtr server_;
  /**
   * @brief 仅串行accepted回调/析构对worker线程句柄的join/替换，不包住所有Action等待过程。
   */
  std::mutex worker_mutex_;
  /**
   * @brief 唯一执行等待线程；成功/失败由NativeWaitPolicy决定是否允许下一Goal，异常锁故障。
   */
  std::thread worker_;
};
}  // namespace dummy_motion_client

/**
 * @brief 初始化ROS并以MultiThreadedExecutor运行代理节点。
 * @param argc 进程参数数量。
 * @param argv ROS/应用参数，交给rclcpp初始化。
 * @return executor正常退出并调用shutdown后为0；不进行设备释放确认或物理运动验收。
 */
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<dummy_motion_client::NativeActionGate>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
