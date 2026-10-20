/**
 * @file limited_goal_budget.hpp
 * @brief 有限功能试验的一次性目标计数器，与 boot/session 绑定。
 * @details 该对象本身没有锁；只有 admission 的外层 ready 快照为原子，并不使计数器可多线程并发访问。
 */
#pragma once
#include <cstdint>
namespace dummy_motion_client {
// 单一control owner；外部ready原子快照供admission读，不并发读写本计数。
/**
 * @brief 单控制 owner 持有的最多1或2次目标预算；不能通过重新 configure 补充。
 */
class LimitedGoalBudget {
 public:
  /**
   * @brief 一生只允许成功配置一次；拒绝非1/2的上限及零 boot/session。
   * @return 配置成功返回 true；失败不修改现有绑定。
   */
  bool configure(unsigned limit, std::uint64_t boot, std::uint64_t session) noexcept {
    if(configured_ || (limit!=1U && limit!=2U) || boot==0U || session==0U) return false;
    limit_=limit; boot_=boot; session_=session; configured_=true; return true;
  }
  /**
   * @brief 已配置、未锁故障且消耗次数小于上限时才有剩余预算。
   */
  bool available() const noexcept { return configured_ && !failed_ && consumed_<limit_; }
  /**
   * @brief 仅在预算可用且 boot/session 完全匹配时消耗一次目标。
   * @details 计数对应此方法被调用成功；不代表目标物理完成，失败目标是否占次数由调用时机决定。
   */
  bool consume(std::uint64_t boot,std::uint64_t session) noexcept {
    if(!available() || boot!=boot_ || session!=session_) return false;
    ++consumed_; return true;
  }
  /**
   * @brief 永久锁住本对象；后续 configure 不会解除 failed_。
   */
  void fail() noexcept { failed_=true; }
  /**
   * @brief 返回已成功 consume 的次数，供单 owner 诊断读取。
   */
  unsigned consumed() const noexcept { return consumed_; }
 private:
  /**
   * @brief 配置时锁存的设备上电身份与控制会话；不随新观测自动变化。
   */
  std::uint64_t boot_{},session_{};
  /**
   * @brief 目标上限及已用次数；消耗时不借用反馈计数。
   */
  unsigned limit_{},consumed_{};
  /**
   * @brief 一次性配置标志和故障锁；均只有单控制 owner 可写。
   */
  bool configured_{},failed_{};
};
}  // namespace dummy_motion_client
