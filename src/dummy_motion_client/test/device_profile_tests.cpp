/**
 * @file device_profile_tests.cpp
 * @brief 核当前SIL画像/独立外来包摘要、permit预算及同快照YAML解析。
 *
 * 需要argv[1]当前安装bundle、argv[2]由其临时生成的外来合同目录；只读输入，临时复制包并改副本制造漂移。assert须启用；load成功不表示固件已绑定或取得真实运动许可。
 */
#include <cassert>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <unistd.h>

#include <yaml-cpp/yaml.h>

#include "dummy_motion_client/device_profile.hpp"

namespace dummy_motion_client::detail {
/**
 * @brief 声明被测内部快照读取接口，用钩子隔离“读完文件后被替换”的确定时序。
 * @param path 待读YAML。
 * @param expected_sha256 本次原字节SHA。
 * @param after_snapshot 已取字节后执行替换回调。
 * @return 对最初已核字节的YAML解析，非重新读被替换路径。
 */
YAML::Node load_pinned_yaml(const std::filesystem::path &path, const std::string &expected_sha256,
                            const std::function<void()> &after_snapshot);
}

/**
 * @brief 比较SIL能力、拒绝跨包/未合格BENCH及文件漂移。
 * @param argc 参数个数（含程序名）。
 * @param argv 参数数组，模式/路径位置见下述说明。
 * @return 0表示所选检查通过；非零或断言/require异常表示失败。
 * permit短目标默认60s，630s长目标需显式至少639.5s且≤660s，缺省/639499999µs/超trial均拒绝。
 * SIL动态v0.25rad/s、a0.5rad/s²、path0.15rad/goal0.001rad，firmware_bound=false仍要求Status固件hash非零；当前固定候选profile载荷SHA与profile身份相等。
 */
int main(int argc, char **argv) {
  if (argc != 3) return 2;
  using dummy_motion_client::validated_motion_permit_limit;
  assert(validated_motion_permit_limit(10000000U,660000000U,1500000U,3000000U,
                                       5000000U,std::nullopt)==60000000U);
  assert(validated_motion_permit_limit(630000000U,660000000U,1500000U,3000000U,
                                       5000000U,650000000U)==650000000U);
  for(const auto request: {std::optional<std::uint64_t>{},
                           std::optional<std::uint64_t>{639499999U},
                           std::optional<std::uint64_t>{660000001U}}) {
    bool denied=false;
    try { (void)validated_motion_permit_limit(630000000U,660000000U,1500000U,
                                               3000000U,5000000U,request); }
    catch(const std::invalid_argument &) { denied=true; }
    assert(denied);
  }
  const auto profile = dummy_motion_client::load_device_profile(argv[1], "sil_6dof", 1U, 63U);
  assert(profile.joints.joints.size() == 6U);
  assert(profile.joints.joints.front() == "Joint1");
  assert(profile.joints.lower.at(2) == -0.3);
  assert(profile.joints.upper.at(2) == 0.3);
  const auto frozen = dummy_motion_client::inspect_device_bundle_identity(argv[2], "sil_j1");
  assert(frozen.bundle_hash != profile.bundle_hash);
  assert(frozen.contract_hash == profile.contract_hash);
  bool foreign_sil_rejected = false;
  try { (void)dummy_motion_client::load_device_profile(argv[2], "sil_j1", 2U, 1U); }
  catch (const std::invalid_argument &) { foreign_sil_rejected = true; }
  assert(foreign_sil_rejected);
  assert(profile.joints.velocity.at(0) == 0.25);
  assert(profile.joints.acceleration.at(0) == 0.5);
  assert(profile.joints.path_tolerance.at(0) == 0.15);
  assert(profile.joints.goal_tolerance.at(0) == 0.001);
  assert(profile.candidate_profile_payload_hash == profile.profile_hash);
  assert(!profile.firmware_bound);
  dummy_motion_client::DeviceStatusIdentity status{};
  status.contract_hash = profile.contract_hash;
  status.model_hash = profile.model_hash;
  status.profile_hash = profile.profile_hash;
  status.policy_hash = profile.policy_hash;
  status.firmware_hash[0] = 1U;
  assert(dummy_motion_client::status_matches_bundle_identity(profile, status));
  status.firmware_hash.fill(0U);
  assert(!dummy_motion_client::status_matches_bundle_identity(profile, status));
  bool unqualified_bench = false;
  try { (void)dummy_motion_client::load_device_profile(argv[1], "f405_6dof_bench", 201U, 63U); }
  catch (const std::invalid_argument &) { unqualified_bench = true; }
  assert(unqualified_bench);
  bool wrong_bench = false;
  try { (void)dummy_motion_client::load_device_profile(argv[1], "f405_6dof_bench", 201U, 1U); }
  catch (const std::invalid_argument &) { wrong_bench = true; }
  assert(wrong_bench);
  bool rejected = false;
  try { (void)dummy_motion_client::load_device_profile(argv[1], "sil_6dof", 1U, 1U); }
  catch (const std::invalid_argument &) { rejected = true; }
  assert(rejected);
  /**
   * @brief 只改临时副本中的profile注释也必须触发原字节SHA漂移拒绝；随后删除副本，不修改输入合同。
   */
  const auto copy = std::filesystem::temp_directory_path() / ("device_profile_" + std::to_string(getpid()));
  std::filesystem::remove_all(copy);
  std::filesystem::copy(argv[1], copy, std::filesystem::copy_options::recursive);
  { std::ofstream drift(copy / "profiles/sil_6dof.yaml", std::ios::app); drift << "\n# payload drift\n"; }
  rejected = false;
  try { (void)dummy_motion_client::load_device_profile(copy.string(), "sil_6dof", 1U, 63U); }
  catch (const std::invalid_argument &) { rejected = true; }
  std::filesystem::remove_all(copy);
  assert(rejected);
  const auto activation = std::filesystem::temp_directory_path() /
                          ("device_profile_activation_" + std::to_string(getpid()) + ".yaml");
  {
    std::ofstream output(activation);
    output << "scope: NO_LOAD_DEV\n";
  }
  const auto pinned = dummy_motion_client::detail::load_pinned_yaml(
      activation, "c8fdca54aef747dab564237233407b0cae847743a1e70df1c34774176236dc9b",
      /**
       * @par 内部after_snapshot回调
       * 将同路径scope改FORMAL；最终pinned仍须为原NO_LOAD_DEV，证明解析同一已核快照，不把被替换文件内容当已验证。
       */
      [&activation] {
        std::ofstream replacement(activation, std::ios::trunc);
        replacement << "scope: FORMAL\n";
      });
  assert(pinned["scope"].as<std::string>() == "NO_LOAD_DEV");
  std::filesystem::remove(activation);
  std::cout << "{\"device_profile\":true,\"six_axis\":true,\"mask_drift_rejected\":true,\"payload_drift_rejected\":true}\n";
}
