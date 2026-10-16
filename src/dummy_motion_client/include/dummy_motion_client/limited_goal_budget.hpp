#pragma once
#include <cstdint>
namespace dummy_motion_client {
// 单一control owner；外部ready原子快照供admission读，不并发读写本计数。
class LimitedGoalBudget {
 public:
  bool configure(unsigned limit, std::uint64_t boot, std::uint64_t session) noexcept {
    if(configured_ || (limit!=1U && limit!=2U) || boot==0U || session==0U) return false;
    limit_=limit; boot_=boot; session_=session; configured_=true; return true;
  }
  bool available() const noexcept { return configured_ && !failed_ && consumed_<limit_; }
  bool consume(std::uint64_t boot,std::uint64_t session) noexcept {
    if(!available() || boot!=boot_ || session!=session_) return false;
    ++consumed_; return true;
  }
  void fail() noexcept { failed_=true; }
  unsigned consumed() const noexcept { return consumed_; }
 private:
  std::uint64_t boot_{},session_{};
  unsigned limit_{},consumed_{};
  bool configured_{},failed_{};
};
}  // namespace dummy_motion_client
