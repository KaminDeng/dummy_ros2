#pragma once
#include <array>
#include <string>
#include "dummy_motion_client/management_client.hpp"
namespace dummy_motion_client {
struct DpManagementParameters {
  std::string endpoint;
  std::array<std::uint8_t,16> device{}, client{};
  std::uint32_t profile=0;
  std::uint8_t mask=0;
};
struct DeviceObservation {
  std::vector<std::uint8_t> state, status;
  std::uint64_t state_received_us=0, status_received_us=0;
};
inline bool observation_is_fresh(const DeviceObservation &observation,std::uint64_t now) noexcept {
  if(!observation.state_received_us || !observation.status_received_us) return false;
  // Receipt stamps are local. A receive after the caller sampled `now` has zero age.
  const auto state_age=now>observation.state_received_us ? now-observation.state_received_us : 0;
  const auto status_age=now>observation.status_received_us ? now-observation.status_received_us : 0;
  return state_age<=250000 && status_age<=1000000;
}
class DpManagementTransport final : public ManagementTransport {
 public:
  enum class Connection { kStandaloneWorker, kDeviceWorker };
  explicit DpManagementTransport(DpManagementParameters, Connection=Connection::kStandaloneWorker);
  ~DpManagementTransport() override;
  bool start();
  void stop();
  bool ready() const noexcept;
  std::unique_ptr<RequestHandle> send(const std::vector<std::uint8_t> &) override;
  std::optional<DeviceObservation> observe();
  bool allow_inactive_prime(bool allowed) noexcept;
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
std::uint64_t management_now_us() noexcept;
}  // namespace dummy_motion_client
