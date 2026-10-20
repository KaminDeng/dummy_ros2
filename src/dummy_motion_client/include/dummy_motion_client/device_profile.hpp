/**
 * @file device_profile.hpp
 * @brief 声明manifest钉住的设备profile、开发activation与许可预算加载。
 * 这些接口只读本机产物并验证绑定，不连接USB、不授权运行。SIL有固定可信fixture摘要，f405无负载开发另要求外部activation pin和报告/原页链；UNQUALIFIED/SOFTWARE_ONLY/功能开发范围不能被注释提升成正式发布资格。
 */
#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>

#include "dummy_motion_client/action_gate.hpp"

namespace dummy_motion_client {
/**
 * @brief 32字节身份值：bundle为排序payload聚合，普通profile的contract/policy可采用聚合/profile摘要；bench按manifest分开绑定源契约、编译policy和固件。candidate_profile_payload_hash保候选文件摘要，live profile可能由activation调整。
 */
struct DeviceBundleIdentity {
  std::array<std::uint8_t, 32U> bundle_hash{};
  std::array<std::uint8_t, 32U> contract_hash{};
  std::array<std::uint8_t, 32U> model_hash{};
  std::array<std::uint8_t, 32U> candidate_profile_payload_hash{};
  std::array<std::uint8_t, 32U> profile_hash{};
  std::array<std::uint8_t, 32U> policy_hash{};
  std::array<std::uint8_t, 32U> firmware_hash{};
  /**
   * @brief True要求实际Status固件hash精确等expected；False仍要求实际固件hash非零，但不把未绑定值猜成expected。
   */
  bool firmware_bound = false;
};

/**
 * @brief 从已验证wire Status提取的五hash值；此结构不含boot/session或新鲜度，身份hash匹配不能单独代表控制授权。
 */
struct DeviceStatusIdentity {
  std::array<std::uint8_t, 32U> contract_hash{};
  std::array<std::uint8_t, 32U> model_hash{};
  std::array<std::uint8_t, 32U> profile_hash{};
  std::array<std::uint8_t, 32U> policy_hash{};
  std::array<std::uint8_t, 32U> firmware_hash{};
};

/**
 * @brief 开发依据分类：软件证据、50轮功能依据或受限功能依据；均不消除NO_LOAD_DEV/UNQUALIFIED限定。
 */
enum class NoLoadBasis : std::uint8_t { kSoftwareOnly, kFunctional50, kFunctionalLimited };

/**
 * @brief 加载后值对象：继承产物身份，joints含rad/rad/s/rad/s²限制及容差；motion_permit_max_us为允许预算而非已授设备许可。
 */
struct DeviceProfile : DeviceBundleIdentity {
  JointProfile joints;
  std::uint64_t motion_permit_max_us = 60000000U;
  bool bench_only = false;
  bool software_only_development = false;
  bool functional_development_basis = false;
  NoLoadBasis no_load_basis = NoLoadBasis::kSoftwareOnly;
  /**
   * @brief LIMITED分支额外绑定1/2目标次数、特定HIL boot及尚未创建的一次性时间线路径；普通profile这些字段保持默认空/0，不可据0解释为无限硬件授权。
   */
  unsigned goal_limit = 0U;
  std::uint64_t limited_boot_epoch = 0U;
  std::string limited_goal_timeline_path;
};

/**
 * @brief 检查目标/停车/完成/Hold预算都被有限trial与permit覆盖。
 * 使用逐项剩余预算校验防无符号加法溢出；只返回部署预算，不签发permit，不等同session lease。
 * @param goal_us 最大单目标时长µs，非零。
 * @param trial_us 本轮总时长µs，非零且≤1小时。
 * @param stop_us 停车预留µs，非零。
 * @param completion_us 完成收敛/确认预留µs，非零。
 * @param hold_us 本地Hold预算µs，非零。
 * @param explicit_permit_us 可选明确permit时长；长目标≥600秒必须显式提供。
 * @return 合法permit µs；无显式值时min(60秒,trial)，非法/覆盖不足抛invalid_argument。
 */
std::uint64_t validated_motion_permit_limit(std::uint64_t goal_us,
    std::uint64_t trial_us, std::uint64_t stop_us,
    std::uint64_t completion_us, std::uint64_t hold_us,
    std::optional<std::uint64_t> explicit_permit_us);

/**
 * @brief 调用者明确给出的activation路径/pin、源/预构建/编译policy与实际固件/host/map/resource等路径，以及设备/USB端点身份；loader只读核对，不自行搜索最新构建。
 */
struct NoLoadDevActivationRequest {
  std::string activation_path;
  // Must be pinned independently of activation_path; this loader never derives authority from the file itself.
  /**
   * @brief 外部独立钉住的activation原字节SHA-256；不得从同一待验文件现算后当授权依据。
   */
  std::string expected_activation_sha256;
  std::string source_bundle_root;
  std::string prebuild_bundle_manifest_path;
  std::string compiled_policy_manifest_path;
  std::string firmware_image_path;
  std::string host_executable_path;
  std::string map_file_path;
  std::string resource_report_path;
  /**
   * @brief SOFTWARE_ONLY使用qualification report与post-build绑定；功能开发分支用D回归/出处、HIL及其raw索引/ELF，不能混作正式资源测量通过。
   */
  std::string qualification_report_path;
  std::string device_id;
  std::string usb_serial;
  std::string endpoint;
  /**
   * @brief D回归及出处、HIL报告/索引/根目录、ELF与resource相对路径根等证据定位值，均受activation实际hash/范围检查。
   */
  std::string d_regression_report_path;
  std::string d_regression_provenance_path;
  std::string hil_report_path;
  std::string hil_raw_index_path;
  std::string hil_evidence_root;
  std::string elf_path;
  std::string resource_base_root;
};

// Inspects payload identity without granting runtime authority.
/**
 * @brief 核完整bundle payload及聚合后提取指定profile身份。
 * 不要求/授予runtime authority，不代替load_device_profile的轴/策略/activation验证。
 * @param bundle_root 候选bundle目录。
 * @param profile_name 限定安全名称的profile键。
 * @return DeviceBundleIdentity；缺文件、摘要/manifest/profile错误抛invalid_argument。
 */
DeviceBundleIdentity inspect_device_bundle_identity(const std::string &bundle_root,
                                                    const std::string &profile_name);
/**
 * @brief 比较实际五hash与bundle期望，不做时钟或session判断。
 * @param expected 从bundle提取的期望身份。
 * @param actual Status提取hash。
 * @return bool；契约/模型/profile/policy精确相同，实际固件非零，firmware_bound时再精确比固件。
 */
bool status_matches_bundle_identity(const DeviceBundleIdentity &expected,
                                    const DeviceStatusIdentity &actual) noexcept;

// Loads a manifest-pinned CI bundle. Any missing field, digest drift, or axis/profile mismatch throws.
/**
 * @brief 按manifest和profile/轴/策略加载可信软件或可用开发描述。
 * 四参数重载没有activation，不能加载要求activation的f405 bench；普通profile须test_only，绝不允许motion_permitted_on_real_hardware标真。仅本地数据加载，不启用设备。
 * @param bundle_root bundle目录，逐payload/排序聚合及钉住快照校验。
 * @param profile_name 安全名称；SIL固定可信fixture，bench有专门规则。
 * @param expected_id 非零profile ID。
 * @param expected_mask 只能1或63，必须匹配active_axes。
 * @return DeviceProfile；不满足任何绑定抛invalid_argument。
 */
DeviceProfile load_device_profile(const std::string &bundle_root, const std::string &profile_name,
                                  std::uint32_t expected_id, std::uint8_t expected_mask);
/**
 * @brief 在相同bundle/轴验证上核外部NO_LOAD_DEV activation及证据。
 * 验activation原字节、源/候选/预构建/policy/实际产物及设备端点；功能依据查D回归和HIL原页（50轮或LIMITED一轮），软件依据保SOFTWARE_ONLY。功能依据的资源报告仍按静态未评估/非发布资格处理；此loader不核实时USB枚举、不发许可。
 * @param bundle_root 候选目录。
 * @param profile_name 通常f405_6dof_bench，必须满足profile自身规则。
 * @param expected_id 须与profile及manifest能力一致；bench为201。
 * @param expected_mask 配置掩码；bench为63。
 * @param activation 明确路径、外部pin及设备/报告绑定值。
 * @return 带实际开发范围和LIMITED限制的DeviceProfile，失败抛invalid_argument。
 */
DeviceProfile load_device_profile(const std::string &bundle_root, const std::string &profile_name,
                                  std::uint32_t expected_id, std::uint8_t expected_mask,
                                  const NoLoadDevActivationRequest &activation);
}  // namespace dummy_motion_client
