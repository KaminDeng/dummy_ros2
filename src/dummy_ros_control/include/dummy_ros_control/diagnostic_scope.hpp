/**
 * @file diagnostic_scope.hpp
 * @brief 为宿主调用链临时绑定固定容量诊断journal。
 * thread_local只作用于当前线程，同一RAII作用域可嵌套并恢复旧指针；不会跨线程自动传播，不拥有journal，也不写设备wire或授予动作资格。
 */
#pragma once
#include "dummy_ros_control/diagnostic_journal.hpp"
#include <chrono>
namespace dummy_ros_control {
// ROS宿主控制调用链的私有scope；无scope的NativeCLI保留既有日志行为。
/**
 * @brief 当前宿主线程借用journal指针；无scope时为nullptr，原NativeCLI日志行为由调用方保留。
 */
inline thread_local DiagnosticJournal* current_host_diagnostics=nullptr;
/**
 * @brief 取得宿主steady微秒时间戳。
 * @return steady_clock epoch以来µs，不是ROS仿真或MCU时钟。
 */
inline std::uint64_t host_diagnostic_now_us() noexcept {
  return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count());
}
/**
 * @brief 在当前线程安装并恢复journal的不可复制RAII借用。
 */
class HostDiagnosticScope final {
 public:
  /**
   * @brief 保存前一绑定并安装新journal。
   * @param value 必须覆盖scope寿命的journal引用。
   */
  explicit HostDiagnosticScope(DiagnosticJournal& value) noexcept :prior_(current_host_diagnostics) {current_host_diagnostics=&value;}
  /**
   * @brief 恢复同线程前一个journal，不清除已积累首错。
   */
  ~HostDiagnosticScope() {current_host_diagnostics=prior_;}
  /**
   * @brief 禁止复制scope，避免重复恢复旧线程绑定。
   */
  HostDiagnosticScope(const HostDiagnosticScope&)=delete;
  /**
   * @brief 禁止赋值scope。
   */
  HostDiagnosticScope& operator=(const HostDiagnosticScope&)=delete;
 private:
  /**
   * @brief 进入scope前的借用指针，可空；scope不释放其目标。
   */
  DiagnosticJournal* prior_;
};
} // namespace dummy_ros_control
