#pragma once
#include "dummy_ros_control/diagnostic_journal.hpp"
#include <chrono>
namespace dummy_ros_control {
// ROS宿主控制调用链的私有scope；无scope的NativeCLI保留既有日志行为。
inline thread_local DiagnosticJournal* current_host_diagnostics=nullptr;
inline std::uint64_t host_diagnostic_now_us() noexcept {
  return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count());
}
class HostDiagnosticScope final {
 public:
  explicit HostDiagnosticScope(DiagnosticJournal& value) noexcept :prior_(current_host_diagnostics) {current_host_diagnostics=&value;}
  ~HostDiagnosticScope() {current_host_diagnostics=prior_;}
  HostDiagnosticScope(const HostDiagnosticScope&)=delete;
  HostDiagnosticScope& operator=(const HostDiagnosticScope&)=delete;
 private:
  DiagnosticJournal* prior_;
};
} // namespace dummy_ros_control
