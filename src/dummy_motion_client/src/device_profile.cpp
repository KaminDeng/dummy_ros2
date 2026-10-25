/**
 * @file device_profile.cpp
 * @brief 从本机manifest/profile及明确开发activation证据加载设备描述和预算。
 * bundle摘要列表首先只能建立内部自洽；SIL另钉固定可信聚合，f405空载开发必须由
 * 调用方提供独立activation原字节pin，再核候选/来源/产物和报告绑定。不得把同一待验
 * 文件现算摘要当外部授权来源。所有文件读取/解析发生非实时配置端，不连接USB、
 * 不枚举实际设备、不发送许可；SOFTWARE_ONLY/NO_LOAD_DEV/UNQUALIFIED保持各自范围。
 */
#include "dummy_motion_client/device_profile.hpp"
#include "dummy_ref/wire_validation.hpp"
#include "dp_protocol_wire_contract.h"
#ifndef DUMMY_INSTALLED_ROBOT_MANIFEST_SHA256
#define DUMMY_INSTALLED_ROBOT_MANIFEST_SHA256 ""
#endif

#include <algorithm>
#include <array>
#include <charconv>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iterator>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <openssl/evp.h>
#include <openssl/sha.h>
#include <yaml-cpp/yaml.h>

namespace dummy_motion_client {
namespace detail {
/**
 * @brief 声明按外部期望摘要读取一次字节快照后再解析YAML/JSON的测试可注入入口。
 * @param path 文件路径。
 * @param expected_sha256 期望的小写64字符SHA256，可信来源由调用者建立。
 * @param after_snapshot 可选快照后钩子，测试可修改路径验证解析仍使用已核字节。
 * @return 已核快照解析结果；读取、摘要或格式错误抛异常。
 */
YAML::Node load_pinned_yaml(const std::filesystem::path &path, const std::string &expected_sha256,
                            const std::function<void()> &after_snapshot);
}
namespace {
/**
 * @brief 文件系统类型别名；这里的文件/报告是Host软件证据，不是正在运行的设备状态。
 */
namespace fs = std::filesystem;

/**
 * @brief 将配置/证据拒绝统一转换为invalid_argument。
 * @param message 失败原因C文本。
 * @note 只拒绝本次加载，不停止物理设备或修复文件。
 */
[[noreturn]] void invalid(const char *message) { throw std::invalid_argument(message); }

/**
 * @brief 流式计算文件原始字节SHA256，不按YAML/JSON语义规范化。
 * @param path 需要存在且可读的文件，符号链接/范围限制由上层另外检查。
 * @return 64字符小写hex；打开、读取或EVP失败抛invalid_argument。
 */
std::string sha256_file(const fs::path &path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) invalid("bundle payload missing");
  EVP_MD_CTX *context = EVP_MD_CTX_new();
  if (context == nullptr) invalid("sha256 init failed");
  /**
   * @brief 本函数各EVP失败分支显式释放context，不把hash计算失败返回伪摘要；不是文件所有权的RAII封装。
   */
  const auto release = [&context] { EVP_MD_CTX_free(context); };
  if (EVP_DigestInit_ex(context, EVP_sha256(), nullptr) != 1) { release(); invalid("sha256 init failed"); }
  std::array<char, 4096U> bytes{};
  while (input.good()) {
    input.read(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    if (input.gcount() > 0 && EVP_DigestUpdate(context, bytes.data(), static_cast<std::size_t>(input.gcount())) != 1) {
      release(); invalid("sha256 update failed");
    }
  }
  if (!input.eof()) { release(); invalid("bundle payload unreadable"); }
  std::array<unsigned char, SHA256_DIGEST_LENGTH> digest{};
  unsigned int digest_size = 0U;
  if (EVP_DigestFinal_ex(context, digest.data(), &digest_size) != 1 || digest_size != digest.size()) {
    release(); invalid("sha256 final failed");
  }
  release();
  static constexpr char hex[] = "0123456789abcdef";
  std::string result(64U, '0');
  for (std::size_t i = 0; i < digest.size(); ++i) {
    result[i * 2U] = hex[digest[i] >> 4U];
    result[i * 2U + 1U] = hex[digest[i] & 15U];
  }
  return result;
}

/**
 * @brief 将恰64个小写十六进制字符还原为32字节。
 * @param text 原始hex，不接受大写或其他字符。
 * @return 摘要数组；非法抛invalid_argument。该转换本身允许全零，非空身份要求由调用处另核。
 */
std::array<std::uint8_t, 32U> parse_sha256(const std::string &text) {
  if (text.size() != 64U) invalid("invalid sha256");
  /**
   * @brief 单字符hex解码，仅接0..9/a..f。
   * @param value 待转换字符。
   * @return 0..15，其他字符为-1。
   */
  const auto nibble = [](char value) -> int {
    if (value >= '0' && value <= '9') return value - '0';
    if (value >= 'a' && value <= 'f') return value - 'a' + 10;
    return -1;
  };
  std::array<std::uint8_t, 32U> result{};
  for (std::size_t i = 0; i < result.size(); ++i) {
    const int high = nibble(text[i * 2U]);
    const int low = nibble(text[i * 2U + 1U]);
    if (high < 0 || low < 0) invalid("invalid sha256");
    result[i] = static_cast<std::uint8_t>((high << 4U) | low);
  }
  return result;
}

/**
 * @brief 对已拥有的原字节字符串计算SHA256，包含所有空白/换行。
 * @param text 文件快照或排序聚合清单内容。
 * @return 小写64字符hex，摘要函数失败抛invalid_argument。
 */
std::string sha256_text(const std::string &text) {
  std::array<unsigned char, SHA256_DIGEST_LENGTH> digest{};
  if (SHA256(reinterpret_cast<const unsigned char *>(text.data()), text.size(), digest.data()) == nullptr)
    invalid("sha256 text failed");
  static constexpr char hex[] = "0123456789abcdef";
  std::string result(64U, '0');
  for (std::size_t i = 0; i < digest.size(); ++i) {
    result[i * 2U] = hex[digest[i] >> 4U];
    result[i * 2U + 1U] = hex[digest[i] & 15U];
  }
  return result;
}

/**
 * @brief 以二进制读取整份本机文件到拥有型字符串。
 * @param path 来源文件，调用者负责路径/信任边界。
 * @return 原字节快照；打开或流bad错误抛invalid_argument，不建立长期文件锁。
 */
std::string read_file(const fs::path &path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) invalid("bundle payload missing");
  const std::string bytes((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
  if (input.bad()) invalid("bundle payload unreadable");
  return bytes;
}

/**
 * @brief 读取一次后解析YAML/JSON，但本入口不额外钉原字节SHA。
 * @param path 已由调用流程定位的文件。
 * @return YAML节点；语义身份字段仍由使用者逐项核对，不能把本函数当完整来源认证。
 */
YAML::Node load_yaml_snapshot(const fs::path &path) { return YAML::Load(read_file(path)); }

/**
 * @brief 限定可拼入profiles路径的短名称。
 * @param name 1..64字符，仅isalnum判定的字母数字或下划线。
 * @return 符合形状为true；不证明该profile存在/可信或可用于实体运动。
 */
bool safe_profile_name(const std::string &name) {
  return !name.empty() && name.size() <= 64U && std::all_of(name.begin(), name.end(), [](unsigned char value) {
    return std::isalnum(value) || value == '_';
  });
}

/**
 * @brief 读取必需字段，缺失即拒绝，不自行填默认值。
 * @param node 父YAML节点。
 * @param field 字段名。
 * @return 子节点；类型/范围由后续as和业务检查负责。
 */
const YAML::Node require(const YAML::Node &node, const char *field) {
  const YAML::Node value = node[field];
  if (!value) invalid("required bundle field missing");
  return value;
}

/**
 * @brief 将唯一支持的Joint1..Joint6名称转为配置位。
 * @param name 必须是6字符Joint加1..6。
 * @return 1<<(轴号-1)，非法名称抛invalid_argument。
 */
std::uint8_t axis_bit(const std::string &name) {
  if (name.size() != 6U || name.substr(0U, 5U) != "Joint" || name[5] < '1' || name[5] > '6')
    invalid("invalid active axis");
  return static_cast<std::uint8_t>(1U << static_cast<unsigned>(name[5] - '1'));
}

/**
 * @brief 从YAML转换有限正实数，用于速度/加速度/误差限制。
 * @param node 数值节点。
 * @return double值，非有限或≤0拒绝；物理单位由调用字段决定。
 */
double finite_positive(const YAML::Node &node) {
  const double value = node.as<double>();
  if (!std::isfinite(value) || value <= 0.0) invalid("invalid profile limit");
  return value;
}

/**
 * @brief 本机bundle自洽核验结果，拥有规范根路径、manifest节点/原摘要、payload映射与排序聚合。
 * 该类型不是运行时authority；还须SIL固定聚合或外部activation来源绑定。
 */
struct VerifiedBundle {
  /**
   * @brief weakly_canonical解析后的bundle根；后续payload必须在相对路径规则下且无符号链接组件。
   */
  fs::path root;
  /**
   * @brief 从单次manifest字节读取解析的元数据，声明payload列表和能力等，本身不能自证外部可信。
   */
  YAML::Node manifest;
  /**
   * @brief manifest原始字节SHA，不是payload_sorted聚合，也不是已运行固件摘要。
   */
  std::string manifest_sha256;
  /**
   * @brief 已经逐文件验证的相对路径→原字节SHA映射，禁止重复和乱序条目。
   */
  std::unordered_map<std::string, std::string> payloads;
  /**
   * @brief 按严格路径序拼接“path:sha”行后的SHA，行间单换行、没有人为补出的尾换行。
   */
  std::string aggregate;
};

/**
 * @brief 核根下相对路径每一组件非符号链接，最终为普通文件。
 * @param root 已确定根目录。
 * @param relative 由调用处限制非绝对/无父目录穿越的相对路径。
 * @return 检查成功true；这里不持有fd做整个后续读取的不可变锁。
 */
bool regular_unlinked_payload(const fs::path &root, const fs::path &relative) {
  fs::path current = root;
  for (const auto &part : relative) {
    current /= part;
    std::error_code error;
    const auto status = fs::symlink_status(current, error);
    if (error || fs::is_symlink(status)) return false;
  }
  return fs::is_regular_file(current);
}

/**
 * @brief 核当前wire版本、严格排序payload列表、每文件摘要和聚合。
 * @param bundle_root 候选目录，先weakly_canonical再要求目录存在。
 * @return 自洽VerifiedBundle；任何缺失/路径/摘要/重复/顺序错误均拒绝。
 * @note 不从待验manifest自身推导运行许可；普通SIL另比固定可信聚合，bench另核外部activation pin。
 */
VerifiedBundle verify_bundle(const std::string &bundle_root) {
    const fs::path root = fs::weakly_canonical(fs::path(bundle_root));
    if (!fs::is_directory(root)) invalid("bundle root unavailable");
    const fs::path manifest_path = root / "manifest.json";
    const std::string manifest_bytes = read_file(manifest_path);
    const YAML::Node manifest = YAML::Load(manifest_bytes);
    /**
     * @brief release_id必须匹配本次编译wire schema major/minor，防止拿旧codec目录拼接当前runtime。
     */
    const std::string expected_release = "dummy-wire-v" +
        std::to_string(dummy_ref::protocol::kSchemaMajor) + "." +
        std::to_string(dummy_ref::protocol::kSchemaMinor);
    if (require(manifest, "format").as<std::string>() != "dummy_wire_release_v2" ||
        require(manifest, "release_id").as<std::string>() != expected_release) invalid("unsupported bundle release");
    const YAML::Node payloads = require(manifest, "payloads");
    if (!payloads.IsSequence() || payloads.size() == 0U) invalid("invalid bundle payload list");
    std::unordered_map<std::string, std::string> verified;
    std::string listing, previous;
    /**
     * @brief 每条要求相对路径、无“..”子串、逐组件无symlink、文件SHA匹配、路径唯一且严格递增；
     * 清单只证明所列payload自洽，不把目录内任意额外文件自动纳入可信集合。
     */
    for (const YAML::Node &payload : payloads) {
      const std::string relative = require(payload, "path").as<std::string>();
      const std::string expected = require(payload, "sha256").as<std::string>();
      const fs::path path(relative);
      if (path.is_absolute() || relative.find("..") != std::string::npos || !regular_unlinked_payload(root, path) ||
          sha256_file(root / path) != expected || !verified.emplace(relative, expected).second ||
          (!previous.empty() && previous >= relative)) invalid("bundle payload hash mismatch");
      if (!listing.empty()) listing.push_back('\n');
      listing += relative + ":" + expected;
      previous = relative;
    }
    const std::string aggregate = require(manifest, "payload_sorted_sha256").as<std::string>();
    if (aggregate != sha256_text(listing)) invalid("bundle aggregate hash mismatch");
    const bool protocol_payload = std::any_of(verified.begin(), verified.end(), [](const auto& item) {
      return item.first.rfind("protocol/", 0U) == 0U;
    });
    if (manifest["contract_sha256"] || manifest["runtime_contract_sha256"] ||
        manifest["protocol_release"] || manifest["wire_contract"] || protocol_payload) {
      if (require(manifest, "schema_major").as<unsigned>() != dummy_ref::protocol::kSchemaMajor ||
          require(manifest, "schema_minor").as<unsigned>() != dummy_ref::protocol::kSchemaMinor)
        invalid("current protocol wrapper schema mismatch");
      const auto contract = require(manifest, "contract_sha256").as<std::string>();
      if (manifest["runtime_contract_sha256"] &&
          manifest["runtime_contract_sha256"].as<std::string>() != contract)
        invalid("current protocol runtime identity mismatch");
      const auto reference = require(manifest, "protocol_release");
      const auto wire = require(manifest, "wire_contract");
      const std::string release_path = "protocol/manifest.json";
      const std::string wire_path = "protocol/include/wire_contract.json";
      if (contract != DP_PROTOCOL_WIRE_CONTRACT_SHA256 ||
          require(reference, "path").as<std::string>() != release_path ||
          verified.find(release_path) == verified.end() ||
          require(reference, "sha256").as<std::string>() != verified.at(release_path) ||
          require(wire, "path").as<std::string>() != wire_path ||
          verified.find(wire_path) == verified.end() ||
          require(wire, "sha256").as<std::string>() != contract)
        invalid("current protocol release identity mismatch");
      const auto release = detail::load_pinned_yaml(root/release_path, verified.at(release_path), {});
      const auto descriptor = detail::load_pinned_yaml(root/wire_path, verified.at(wire_path), {});
      const auto selected = release;
      const auto declared = require(reference, "selection");
      if (require(release, "wire_contract_sha256").as<std::string>() != contract ||
          require(descriptor, "sha256").as<std::string>() != contract ||
          require(selected, "protocol_set").as<std::string>() != "dummy" ||
          require(selected, "codec").as<std::string>() != "raw-fixed")
        invalid("current protocol codec/wire identity mismatch");
      for (const char* field : {"protocol_set", "profile", "codec"})
        if (require(selected, field).as<std::string>() != require(declared, field).as<std::string>())
          invalid("current protocol selection mismatch");
      const auto actual_source = require(release, "source_snapshot");
      const auto declared_source = require(reference, "source_snapshot");
      if (!actual_source.IsMap() || actual_source.size() == 0U || !declared_source.IsMap() ||
          actual_source.size() != declared_source.size())
        invalid("current protocol source snapshot mismatch");
      for (const auto& field : actual_source) {
        const auto key = field.first.as<std::string>();
        if (!field.second.IsScalar() ||
            field.second.as<std::string>() != require(declared_source, key.c_str()).as<std::string>())
          invalid("current protocol source snapshot mismatch");
      }
      const auto release_payloads = require(release, "payloads");
      if (!release_payloads.IsSequence() || release_payloads.size() == 0U)
        invalid("current protocol payload list missing");
      for (const auto& item : release_payloads) {
        const auto path = "protocol/" + require(item, "path").as<std::string>();
        if (verified.find(path) == verified.end() ||
            verified.at(path) != require(item, "sha256").as<std::string>())
          invalid("current protocol payload binding mismatch");
      }
    }
    return {root, manifest, sha256_text(manifest_bytes), std::move(verified), aggregate};
}

/**
 * @brief 从自洽bundle提取指定profile的产物身份，区分普通与f405 bench映射。
 * @param bundle 已核payload集合。
 * @param profile_name 安全名称，对应model/model.yaml和profiles/name.yaml必须列在manifest。
 * @return 普通contract用bundle聚合、policy用profile摘要；bench从manifest绑定源contract/compiled policy/固件并置firmware_bound。
 * @note 身份提取不等于轴/策略/activation资格检查。
 */
DeviceBundleIdentity bundle_identity(const VerifiedBundle &bundle, const std::string &profile_name) {
    if (!safe_profile_name(profile_name)) invalid("invalid requested device profile");
    const std::string model_key = "model/model.yaml";
    const std::string profile_key = "profiles/" + profile_name + ".yaml";
    if (bundle.payloads.find(model_key) == bundle.payloads.end() ||
        bundle.payloads.find(profile_key) == bundle.payloads.end()) invalid("unmanifested model or profile");
    DeviceBundleIdentity output;
    output.bundle_hash = parse_sha256(bundle.aggregate);
    // 冻结回归保原聚合含义；现行包显式提供稳定wire身份。
    output.contract_hash = bundle.manifest["contract_sha256"] ?
        parse_sha256(bundle.manifest["contract_sha256"].as<std::string>()) : output.bundle_hash;
    output.model_hash = parse_sha256(bundle.payloads.at(model_key));
    output.profile_hash = parse_sha256(bundle.payloads.at(profile_key));
    output.candidate_profile_payload_hash = output.profile_hash;
    output.policy_hash = output.profile_hash;
    if (profile_name == "f405_6dof_bench") {
      output.firmware_bound = true;
      output.contract_hash = parse_sha256(require(bundle.manifest, "source_contract_sha256").as<std::string>());
      output.policy_hash = parse_sha256(require(bundle.manifest, "compiled_policy_sha256").as<std::string>());
      output.firmware_hash = parse_sha256(require(bundle.manifest, "firmware_image_sha256").as<std::string>());
      if (std::all_of(output.contract_hash.begin(), output.contract_hash.end(), [](auto byte) { return byte == 0U; }) ||
          std::all_of(output.firmware_hash.begin(), output.firmware_hash.end(), [](auto byte) { return byte == 0U; }))
        invalid("invalid bundle identity");
    }
    return output;
}

/**
 * @brief 从JUnit标签文本用受限正则提取双引号属性，不是完整XML解析器。
 * @param tag 单个标签文本。
 * @param name 必需属性名。
 * @return 属性字符串；缺失抛invalid_argument。
 */
std::string xml_attribute(const std::string &tag, const char *name) {
  std::smatch match;
  if (!std::regex_search(tag, match, std::regex(std::string("\\b") + name + "=\"([^\"]*)\"")))
    invalid("D252 JUnit attribute missing");
  return match[1].str();
}

/**
 * @brief 核钉住的JUnit字节、至少252项及无失败/跳过的报告形状。
 * @param path JUnit文件路径。
 * @param expected_sha256 activation外部pin约束下声明的报告原SHA。
 * @return 报告timestamp与tests计数。
 * @note 核testsuite声明failures/skipped为0、无failure/error/skipped标签、testcase status=run且名称唯一/数量一致；不在此重新运行测试或验证时间戳真实发生。
 */
std::pair<std::string, unsigned> verify_d_regression_junit(
    const fs::path &path, const std::string &expected_sha256) {
  const std::string xml = read_file(path);
  if (parse_sha256(sha256_text(xml)) != parse_sha256(expected_sha256))
    invalid("D regression JUnit SHA mismatch");
  std::smatch suite;
  if (!std::regex_search(xml, suite, std::regex(R"(<testsuite\b[^>]*>)")))
    invalid("D regression JUnit suite missing");
  const std::string count_text = xml_attribute(suite.str(), "tests");
  unsigned test_count = 0U;
  const auto count = std::from_chars(count_text.data(), count_text.data() + count_text.size(), test_count);
  if (count.ec != std::errc{} || count.ptr != count_text.data() + count_text.size() ||
      test_count < 252U ||
      xml_attribute(suite.str(), "failures") != "0" ||
      xml_attribute(suite.str(), "skipped") != "0" ||
      std::regex_search(xml, std::regex(R"(<(failure|error|skipped)\b)")))
    invalid("D regression JUnit did not pass");
  std::unordered_set<std::string> cases;
  const std::regex test_case(R"(<testcase\b[^>]*>)");
  for (std::sregex_iterator item(xml.begin(), xml.end(), test_case), end; item != end; ++item) {
    const std::string tag = item->str();
    if (xml_attribute(tag, "status") != "run" ||
        !cases.insert(xml_attribute(tag, "name")).second)
      invalid("D regression JUnit testcase invalid");
  }
  if (cases.size() != test_count) invalid("D regression JUnit testcase count invalid");
  return {xml_attribute(suite.str(), "timestamp"), test_count};
}

/**
 * @brief 将D回归JUnit与预构建源、固件、Host、合同、模型、policy和构建配置身份交叉绑定。
 * @param provenance 钉住的出处元数据。
 * @param activation 外部原SHA核过的开发声明。
 * @param prebuild 钉住的预构建manifest。
 * @param identity 候选bundle身份。
 * @param request 实际报告/产物定位值。
 * @param junit_timestamp 刚核JUnit内的timestamp，要求字段精确一致。
 * @param test_count JUnit用例数量。
 * @note 只核rerun_argv是ctest --test-dir ... --output-junit ...及DIRTY_UNQUALIFIED等字段；不执行命令，不把出处声明升级成正式发布资格。
 */
void verify_d_regression_provenance(const YAML::Node &provenance, const YAML::Node &activation,
                            const YAML::Node &prebuild, const DeviceBundleIdentity &identity,
                            const NoLoadDevActivationRequest &request,
                            const std::string &junit_timestamp, unsigned test_count) {
  const YAML::Node build_identity = require(prebuild, "bundle_identity");
  const YAML::Node rerun = require(provenance, "rerun_argv");
  const fs::path build_dir(require(provenance, "build_dir").as<std::string>());
  if (require(provenance, "schema").as<std::string>() != "dummy_d_ctest_provenance_v1" ||
      !std::regex_match(require(provenance, "generated_at_utc").as<std::string>(),
                        std::regex(R"(\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}(\.\d+)?Z)")) ||
      require(provenance, "junit_timestamp").as<std::string>() != junit_timestamp ||
      !build_dir.is_absolute() || !rerun.IsSequence() || rerun.size() != 5U ||
      rerun[0].as<std::string>() != "ctest" || rerun[1].as<std::string>() != "--test-dir" ||
      rerun[2].as<std::string>() != build_dir.string() ||
      rerun[3].as<std::string>() != "--output-junit" ||
      rerun[4].as<std::string>() != fs::absolute(request.d_regression_report_path).string() ||
      require(provenance, "source_hash_kind").as<std::string>() != "PREBUILD_INPUTS" ||
      require(provenance, "source_state").as<std::string>() != "DIRTY_UNQUALIFIED" ||
      require(provenance, "source_sha256").as<std::string>() !=
          require(build_identity, "source_sha256").as<std::string>() ||
      require(provenance, "firmware_sha256").as<std::string>() !=
          require(activation, "firmware_sha256").as<std::string>() ||
      require(provenance, "host_sha256").as<std::string>() !=
          require(activation, "host_sha256").as<std::string>() ||
      parse_sha256(require(provenance, "contract_sha256").as<std::string>()) != identity.contract_hash ||
      parse_sha256(require(provenance, "model_sha256").as<std::string>()) != identity.model_hash ||
      parse_sha256(require(provenance, "compiled_policy_sha256").as<std::string>()) != identity.policy_hash ||
      require(provenance, "build_config_sha256").as<std::string>() !=
          require(activation, "build_config_sha256").as<std::string>() ||
      require(provenance, "junit_sha256").as<std::string>() !=
          require(activation, "d_regression_report_sha256").as<std::string>() ||
      require(provenance, "test_count").as<unsigned>() != test_count)
    invalid("D regression provenance binding mismatch");
}

/**
 * @brief 将HIL证据相对文件限定到已规范的证据根。
 * @param root 证据根目录。
 * @param relative 非空、非绝对且无“..”组件的相对文件。
 * @return 普通且逐组件无symlink的文件路径，否则拒绝。
 */
fs::path bounded_evidence_file(const fs::path &root, const std::string &relative) {
  const fs::path path(relative);
  if (relative.empty() || path.is_absolute() ||
      std::find(path.begin(), path.end(), fs::path("..")) != path.end() ||
      !regular_unlinked_payload(root, path)) invalid("HIL evidence path invalid");
  return root / path;
}

/**
 * @brief 按预期轮数核HIL摘要、原索引与每个原页文件SHA及index内motion/release字段。
 * @param hil 已钉住的HIL汇总。
 * @param raw_index 已钉住的跨轮索引。
 * @param request 提供实际证据根目录。
 * @param expected_cycles 普通功能为50，LIMITED为1。
 * @note 本函数不重新解码page.bin或重算页链；它绑定原页字节与已钉索引元数据，不能从索引自声明单独建立外部信任。
 */
void verify_hil_pages_count(const YAML::Node &hil, const YAML::Node &raw_index,
                      const NoLoadDevActivationRequest &request, std::size_t expected_cycles) {
  const YAML::Node cycles = require(raw_index, "cycles");
  const YAML::Node hil_cycles = require(hil, "cycles");
  const YAML::Node boots = require(hil, "boot_epochs");
  if (require(raw_index, "schema").as<std::string>() != "dummy_no_load_dev_functional_raw_index_v1" ||
      !cycles.IsSequence() || cycles.size() != expected_cycles || !hil_cycles.IsSequence() ||
      hil_cycles.size() != expected_cycles || !boots.IsSequence() || boots.size() != 1U)
    invalid("HIL raw index shape invalid");
  const std::string boot = boots[0].as<std::string>();
  if (!std::regex_match(boot, std::regex("[0-9a-f]{16}"))) invalid("HIL boot identity invalid");
  const fs::path root = fs::weakly_canonical(request.hil_evidence_root);
  if (!fs::is_directory(root)) invalid("HIL evidence root unavailable");
  /**
   * @brief 要求只有一个16字符小写hex boot、轮号从1连续、cycle-NNN/index.json定位一致；页面路径唯一且每轮至少两页。
   */
  std::unordered_set<std::string> seen_pages;
  for (std::size_t slot = 0; slot < expected_cycles; ++slot) {
    const YAML::Node cycle = cycles[slot], summary = hil_cycles[slot];
    const auto number = static_cast<unsigned>(slot + 1U);
    std::ostringstream name;
    name << "cycle-" << std::setw(3) << std::setfill('0') << number;
    const fs::path directory(name.str());
    const std::string index_file = (directory / "index.json").string();
    const std::string release_run = require(cycle, "run_id").as<std::string>();
    if (require(cycle, "cycle").as<unsigned>() != number ||
        require(cycle, "boot_epoch").as<std::string>() != boot ||
        require(cycle, "index_file").as<std::string>() != index_file ||
        require(summary, "cycle").as<unsigned>() != number || !require(summary, "passed").as<bool>() ||
        require(require(summary, "observed"), "run_id").as<std::string>() != release_run ||
        require(require(summary, "observed"), "evidence_run_id").as<std::string>() != release_run)
      invalid("HIL cycle identity mismatch");
    const fs::path index_path = bounded_evidence_file(root, index_file);
    const YAML::Node original = detail::load_pinned_yaml(
        index_path, require(cycle, "index_sha256").as<std::string>(), {});
    const YAML::Node pages = require(cycle, "pages");
    if (!original.IsSequence() || !pages.IsSequence() || pages.size() != original.size() ||
        pages.size() < 2U) invalid("HIL page coverage incomplete");
    bool motion = false, release = false;
    for (std::size_t page = 0; page < pages.size(); ++page) {
      const YAML::Node bound = pages[page], entry = original[page];
      const std::string relative = require(bound, "file").as<std::string>();
      const fs::path page_path = bounded_evidence_file(root, relative);
      const std::string run_id = require(bound, "run_id").as<std::string>();
      const unsigned sequence = require(bound, "evidence_seq").as<unsigned>();
      if (fs::path(relative).parent_path() != directory ||
          !std::regex_match(page_path.filename().string(), std::regex(R"(page-\d{4}\.bin)")) ||
          !seen_pages.insert(relative).second ||
          require(entry, "file").as<std::string>() != page_path.filename().string() ||
          std::to_string(require(entry, "run_id").as<std::uint64_t>()) != run_id ||
          require(entry, "evidence_seq").as<unsigned>() != sequence ||
          parse_sha256(require(bound, "raw_sha256").as<std::string>()) !=
              parse_sha256(require(entry, "raw_sha256").as<std::string>()) ||
          parse_sha256(require(bound, "raw_sha256").as<std::string>()) !=
              parse_sha256(sha256_file(page_path))) invalid("HIL page identity or SHA mismatch");
      /**
       * @brief 这里只按索引声明字段识别完整六轴motion与最终release子尝试，不把文件SHA匹配误称为本层又重演一次设备SENT/链校验。
       */
      motion |= require(entry, "attempt_kind").as<unsigned>() == 1U &&
                require(entry, "expected_mask").as<unsigned>() == 63U &&
                require(entry, "sent_mask").as<unsigned>() == 63U &&
                require(entry, "failed_mask").as<unsigned>() == 0U;
      release |= require(entry, "attempt_kind").as<unsigned>() == 3U &&
                 require(entry, "release_committed").as<bool>() &&
                 require(entry, "final_state").as<unsigned>() == 8U && run_id == release_run;
    }
    if (!motion || !release) invalid("HIL terminal evidence missing");
    for (const auto &file : fs::directory_iterator(root / directory)) {
      const std::string relative = (directory / file.path().filename()).string();
      if (file.path().filename().string().rfind("page-", 0U) == 0U &&
          seen_pages.find(relative) == seen_pages.end()) invalid("HIL unindexed page present");
    }
  }
}

/**
 * @brief 普通功能开发固定要求50轮的包装。
 * @param hil 已钉HIL汇总。
 * @param raw_index 已钉原页索引。
 * @param request 实际原件路径。
 */
void verify_hil_pages(const YAML::Node &hil, const YAML::Node &raw_index,
                      const NoLoadDevActivationRequest &request) {
  verify_hil_pages_count(hil, raw_index, request, 50U);
}

/**
 * @brief 核50轮空载功能开发依据，明确资源报告仍为静态未评估/非发布资格。
 * @param activation 外部pin锁住的NO_LOAD_DEV声明，禁止混入software qualification字段。
 * @param identity 候选/固件等固定身份。
 * @param request 实际D报告/出处/HIL索引/ELF/resource位置。
 * @param prebuild 钉住的预构建记录。
 * @note 要求resource result=FAIL、NOT_ASSESSED/static_link_only、release_qualified=false，HIL PASS不把这些改成正式资源/实机运行资格。
 */
void verify_functional_basis(const YAML::Node &activation, const DeviceBundleIdentity &identity,
                             const NoLoadDevActivationRequest &request, const YAML::Node &prebuild) {
  /**
   * @brief 核本分支报告/ELF是普通非symlink文件，并按activation声明比原始SHA。
   * @param field 外部pin锁住的activation字段名。
   * @param path 调用方明确提供的报告或产物位置。
   * @return 文件属性与摘要同时满足时true；不能由此证明文件所述事件在当前实体上重演。
   */
  const auto bound_file = [&activation](const char *field, const fs::path &path) {
    return fs::is_regular_file(path) && !fs::is_symlink(path) &&
           parse_sha256(require(activation, field).as<std::string>()) == parse_sha256(sha256_file(path));
  };
  if (!bound_file("d_regression_report_sha256", request.d_regression_report_path) ||
      !bound_file("d_regression_provenance_sha256", request.d_regression_provenance_path) ||
      !bound_file("hil_report_sha256", request.hil_report_path) ||
      !bound_file("hil_raw_index_sha256", request.hil_raw_index_path) ||
      !bound_file("elf_sha256", request.elf_path) ||
      activation["qualification_report_sha256"])
    invalid("functional development evidence binding missing");
  const YAML::Node resource = detail::load_pinned_yaml(
      request.resource_report_path, require(activation, "resource_report_sha256").as<std::string>(), {});
  fs::path reported_elf(require(resource, "elf").as<std::string>());
  if (!reported_elf.is_absolute()) {
    if (request.resource_base_root.empty()) invalid("resource ELF base root missing");
    reported_elf = fs::path(request.resource_base_root) / reported_elf;
  }
  if (require(resource, "result").as<std::string>() != "FAIL" ||
      require(resource, "measurement_provenance").as<std::string>() != "NOT_ASSESSED" ||
      require(resource, "measurement_source").as<std::string>() != "static_link_only" ||
      require(resource, "release_qualified").as<bool>() ||
      fs::weakly_canonical(reported_elf) != fs::weakly_canonical(request.elf_path))
    invalid("functional resource report is not static-only");
  const auto [timestamp, test_count] = verify_d_regression_junit(
      request.d_regression_report_path,
      require(activation, "d_regression_report_sha256").as<std::string>());
  const YAML::Node provenance = detail::load_pinned_yaml(
      request.d_regression_provenance_path,
      require(activation, "d_regression_provenance_sha256").as<std::string>(), {});
  verify_d_regression_provenance(
      provenance, activation, prebuild, identity, request, timestamp, test_count);
  const YAML::Node hil = detail::load_pinned_yaml(
      request.hil_report_path, require(activation, "hil_report_sha256").as<std::string>(), {});
  if (require(hil, "format").as<std::string>() != "dummy_v2_hil_v2" ||
      require(hil, "qualification_scope").as<std::string>() != "APPLICATION_HOLD" ||
      !require(hil, "application_hold").as<bool>() ||
      require(hil, "result").as<std::string>() != "PASS" ||
      require(hil, "requested_cycles").as<unsigned>() != 50U ||
      require(hil, "executed_cycles").as<unsigned>() != 50U ||
      require(hil, "passed_cycles").as<unsigned>() != 50U ||
      parse_sha256(require(hil, "firmware_sha256").as<std::string>()) != identity.firmware_hash ||
      parse_sha256(require(hil, "cli_sha256").as<std::string>()) !=
          parse_sha256(require(activation, "host_sha256").as<std::string>()) ||
      parse_sha256(require(hil, "contract_sha256").as<std::string>()) != identity.contract_hash ||
      parse_sha256(require(hil, "model_sha256").as<std::string>()) != identity.model_hash ||
      parse_sha256(require(hil, "profile_sha256").as<std::string>()) != identity.policy_hash ||
      require(hil, "profile_id").as<unsigned>() != 201U ||
      require(hil, "configured_mask").as<unsigned>() != 63U)
    invalid("functional HIL report mismatch");
  verify_hil_pages(hil, detail::load_pinned_yaml(
      request.hil_raw_index_path, require(activation, "hil_raw_index_sha256").as<std::string>(), {}), request);
}

/**
 * @brief 核一次功能依据及1/2目标的受限开发激活，不扩充为50轮资格。
 * @param activation 独立原SHA钉住的LIMITED声明。
 * @param identity 候选身份。
 * @param request 真实产物/报告/原页定位。
 * @param prebuild 固定预构建记录。
 * @note goal_limit和人工空载确认须为明确无引号标量，timeline为绝对且尚不存在路径；只检查声明和文件，不感知当前实体是否空载或人工支撑。
 */
void verify_limited_functional_basis(const YAML::Node &activation, const DeviceBundleIdentity &identity,
                             const NoLoadDevActivationRequest &request, const YAML::Node &prebuild) {
  /**
   * @brief LIMITED只接1或2目标、operator_powered_fixed_no_load=true；计数预算由后续gate使用，loader不在此消耗目标数。
   */
  const auto limit_text = require(activation, "goal_limit").as<std::string>();
  const auto operator_text = require(activation, "operator_powered_fixed_no_load").as<std::string>();
  const fs::path timeline(require(activation, "goal_timeline_path").as<std::string>());
  if (require(activation, "goal_limit").Tag() != "?" ||
      require(activation, "operator_powered_fixed_no_load").Tag() != "?" ||
      (limit_text != "1" && limit_text != "2") || operator_text != "true" ||
      !timeline.is_absolute() || timeline.filename().empty() || fs::exists(timeline) ||
      fs::is_symlink(timeline)) invalid("limited functional budget or invocation invalid");
  /**
   * @brief 核本分支报告/ELF是普通非symlink文件，并按activation声明比原始SHA。
   * @param field 外部pin锁住的activation字段名。
   * @param path 调用方明确提供的报告或产物位置。
   * @return 文件属性与摘要同时满足时true；不能由此证明文件所述事件在当前实体上重演。
   */
  const auto bound_file = [&activation](const char *field, const fs::path &path) {
    return fs::is_regular_file(path) && !fs::is_symlink(path) &&
           parse_sha256(require(activation, field).as<std::string>()) == parse_sha256(sha256_file(path));
  };
  if (!bound_file("d_regression_report_sha256", request.d_regression_report_path) ||
      !bound_file("d_regression_provenance_sha256", request.d_regression_provenance_path) ||
      !bound_file("hil_report_sha256", request.hil_report_path) ||
      !bound_file("hil_raw_index_sha256", request.hil_raw_index_path) ||
      !bound_file("elf_sha256", request.elf_path) ||
      activation["qualification_report_sha256"])
    invalid("functional development evidence binding missing");
  const YAML::Node resource = detail::load_pinned_yaml(
      request.resource_report_path, require(activation, "resource_report_sha256").as<std::string>(), {});
  fs::path reported_elf(require(resource, "elf").as<std::string>());
  if (!reported_elf.is_absolute()) {
    if (request.resource_base_root.empty()) invalid("resource ELF base root missing");
    reported_elf = fs::path(request.resource_base_root) / reported_elf;
  }
  if (require(resource, "result").as<std::string>() != "FAIL" ||
      require(resource, "measurement_provenance").as<std::string>() != "NOT_ASSESSED" ||
      require(resource, "measurement_source").as<std::string>() != "static_link_only" ||
      require(resource, "release_qualified").as<bool>() ||
      fs::weakly_canonical(reported_elf) != fs::weakly_canonical(request.elf_path))
    invalid("functional resource report is not static-only");
  const auto [timestamp, test_count] = verify_d_regression_junit(
      request.d_regression_report_path,
      require(activation, "d_regression_report_sha256").as<std::string>());
  const YAML::Node provenance = detail::load_pinned_yaml(
      request.d_regression_provenance_path,
      require(activation, "d_regression_provenance_sha256").as<std::string>(), {});
  verify_d_regression_provenance(
      provenance, activation, prebuild, identity, request, timestamp, test_count);
  const YAML::Node hil = detail::load_pinned_yaml(
      request.hil_report_path, require(activation, "hil_report_sha256").as<std::string>(), {});
  for (const char *field : {"requested_cycles", "executed_cycles", "passed_cycles"}) {
    if (require(hil, field).Tag() != "?" || require(hil, field).as<std::string>() != "1")
      invalid("limited Native APP count must be integer one");
  }
  if (require(hil, "format").as<std::string>() != "dummy_v2_hil_v2" ||
      require(hil, "qualification_scope").as<std::string>() != "APPLICATION_HOLD" ||
      !require(hil, "application_hold").as<bool>() ||
      require(hil, "result").as<std::string>() != "PASS" ||
      require(hil, "requested_cycles").as<unsigned>() != 1U ||
      require(hil, "executed_cycles").as<unsigned>() != 1U ||
      require(hil, "passed_cycles").as<unsigned>() != 1U ||
      parse_sha256(require(hil, "firmware_sha256").as<std::string>()) != identity.firmware_hash ||
      parse_sha256(require(hil, "cli_sha256").as<std::string>()) !=
          parse_sha256(require(activation, "host_sha256").as<std::string>()) ||
      parse_sha256(require(hil, "contract_sha256").as<std::string>()) != identity.contract_hash ||
      parse_sha256(require(hil, "model_sha256").as<std::string>()) != identity.model_hash ||
      parse_sha256(require(hil, "profile_sha256").as<std::string>()) != identity.policy_hash ||
      require(hil, "profile_id").as<unsigned>() != 201U ||
      require(hil, "configured_mask").as<unsigned>() != 63U)
    invalid("functional HIL report mismatch");
  verify_hil_pages_count(hil, detail::load_pinned_yaml(
      request.hil_raw_index_path, require(activation, "hil_raw_index_sha256").as<std::string>(), {}), request, 1U);
}

/**
 * @brief 内部激活验证结果：依据种类与实际live profile hash；保留NO_LOAD_DEV/UNQUALIFIED范围。
 */
struct NoLoadActivationIdentity {
  /**
   * @brief true表示通过功能或LIMITED报告绑定，false仍是SOFTWARE_ONLY开发依据，不是发布资格位。
   */
  bool functional_basis;
  /**
   * @brief 区分SoftwareOnly、Functional50、FunctionalLimited，不能仅用一个PASS布尔掩盖限制差异。
   */
  NoLoadBasis basis;
  /**
   * @brief 功能开发采用已绑定live profile，软件依据保候选profile；不得盲用candidate文件SHA替换设备实际profile身份。
   */
  std::array<std::uint8_t, 32U> live_profile_hash;
};

/**
 * @brief 从外部独立pin读取activation，再核来源/候选/预构建/产物/报告和端点身份。
 * @param candidate 已自洽候选bundle。
 * @param identity 候选所声明的contract/model/profile/policy/固件映射。
 * @param request 显式外部activation期望SHA和所有实际文件/设备定位值。
 * @return 绑定通过后的依据和live profile摘要；任何矛盾拒绝。
 * @note 可信起点是调用者独立给定expected_activation_sha256；不能先对同一待验文件算hash再自称授权。此处只核本机字节/字段，既不枚举USB也不安装设备permit。
 */
NoLoadActivationIdentity verify_no_load_activation(const VerifiedBundle &candidate,
                                                   const DeviceBundleIdentity &identity,
                                                   const NoLoadDevActivationRequest &request) {
  const fs::path activation_path(request.activation_path);
  if (!fs::is_regular_file(activation_path) || fs::is_symlink(activation_path))
    invalid("activation file hash mismatch");
  /**
   * @brief 先核activation原字节pin再解析scope=NO_LOAD_DEV、qualification=UNQUALIFIED；文件自己的schema/摘要字段不能跳过这一步。
   */
  const YAML::Node activation = detail::load_pinned_yaml(
      activation_path, request.expected_activation_sha256, {});
  if (require(activation, "schema").as<std::string>() != "dummy_no_load_dev_activation_v1" ||
      require(activation, "scope").as<std::string>() != "NO_LOAD_DEV" ||
      require(activation, "qualification").as<std::string>() != "UNQUALIFIED")
    invalid("invalid no-load development scope");
  const YAML::Node basis = activation["basis"];
  const bool limited_basis = basis && basis.as<std::string>() == "NO_LOAD_DEV_FUNCTIONAL_LIMITED";
  const bool functional_basis = limited_basis || (basis && basis.as<std::string>() == "NO_LOAD_DEV_FUNCTIONAL");
  if (basis && !functional_basis && basis.as<std::string>() != "SOFTWARE_ONLY")
    invalid("unverified no-load development basis");
  /**
   * @brief 将activation字段与调用链独立推导的非零预期hash比较。
   * @param field activation内hash字段名。
   * @param expected 实际文件或已绑定来源身份。
   * @return 预期非零且解析后精确相等为true；不是直接相信文件自报值。
   */
  const auto equal_hash = [&activation](const char *field, const std::array<std::uint8_t, 32U> &expected) {
    const auto actual = parse_sha256(require(activation, field).as<std::string>());
    return std::any_of(expected.begin(), expected.end(), [](auto byte) { return byte != 0U; }) &&
           actual == expected;
  };
  /**
   * @brief 要求实际文件普通/非symlink并计算原SHA后交equal_hash。
   * @param field activation绑定字段。
   * @param path 实际产物路径。
   * @return 字节身份精确匹配时true，不检验产物已在设备上执行。
   */
  const auto equal_file_hash = [&equal_hash](const char *field, const fs::path &path) {
    return fs::is_regular_file(path) && !fs::is_symlink(path) &&
           equal_hash(field, parse_sha256(sha256_file(path)));
  };
  /**
   * @brief 来源bundle另做payload自洽核验，并与外部pin锁住的activation源manifest/contract/model相互绑定，防候选和来源各自重写后自洽。
   */
  const auto source = verify_bundle(request.source_bundle_root);
  const fs::path prebuild_path(request.prebuild_bundle_manifest_path);
  const fs::path compiled_path(request.compiled_policy_manifest_path);
  if (!fs::is_regular_file(prebuild_path) || fs::is_symlink(prebuild_path) ||
      !fs::is_regular_file(compiled_path) || fs::is_symlink(compiled_path))
    invalid("no-load development binding missing");
  const std::string prebuild_sha256 = require(activation, "prebuild_bundle_manifest_sha256").as<std::string>();
  const YAML::Node prebuild = detail::load_pinned_yaml(prebuild_path, prebuild_sha256, {});
  /**
   * @brief 编译policy manifest在这里读取一次字节快照并核其中policy/firmware/post-build字段；本函数没有给此文件自身另设原字节SHA锁，不能误述为每份JSON都做同样原字节pin。
   */
  const YAML::Node compiled = load_yaml_snapshot(compiled_path);
  const auto source_model = source.payloads.find("model/model.yaml");
  if (!equal_hash("candidate_manifest_sha256", parse_sha256(candidate.manifest_sha256)) ||
      !equal_hash("candidate_payload_sorted_sha256", identity.bundle_hash) ||
      !equal_hash("source_wire_manifest_sha256", parse_sha256(source.manifest_sha256)) ||
      !equal_hash("source_contract_sha256", identity.contract_hash) ||
      identity.contract_hash != (source.manifest["contract_sha256"] ?
          parse_sha256(source.manifest["contract_sha256"].as<std::string>()) : parse_sha256(source.aggregate)) ||
      source_model == source.payloads.end() ||
      identity.model_hash != parse_sha256(source_model->second) ||
      !equal_hash("model_sha256", identity.model_hash) ||
      !equal_hash("profile_sha256", identity.candidate_profile_payload_hash) ||
      !equal_hash("compiled_policy_sha256", identity.policy_hash) ||
      !equal_hash("firmware_sha256", identity.firmware_hash) ||
      !equal_hash("prebuild_bundle_manifest_sha256", parse_sha256(prebuild_sha256)) ||
      require(prebuild, "format").as<std::string>() != "dummy_robot_bundle_v1" ||
      require(prebuild, "profile_id").as<std::uint32_t>() != 201U ||
      require(prebuild, "configured_mask").as<std::uint32_t>() != 63U ||
      parse_sha256(require(prebuild, "contract_sha256").as<std::string>()) != identity.contract_hash ||
      !equal_hash("prebuild_model_sha256", parse_sha256(require(prebuild, "model_sha256").as<std::string>())) ||
      !equal_hash("prebuild_profile_sha256", parse_sha256(require(prebuild, "profile_sha256").as<std::string>())) ||
      !equal_hash("prebuild_compiled_policy_sha256",
                  parse_sha256(require(prebuild, "compiled_policy_sha256").as<std::string>())) ||
      !equal_hash("build_config_sha256",
                  parse_sha256(require(require(prebuild, "bundle_identity"), "build_config_sha256").as<std::string>())) ||
      require(compiled, "format").as<std::string>() != "dummy_compiled_policy_manifest_v1" ||
      !require(compiled, "runtime_enabled").as<bool>() ||
      (functional_basis ?
           parse_sha256(require(compiled, "policy_sha256").as<std::string>()) != identity.policy_hash :
           !equal_hash("prebuild_compiled_policy_sha256",
                       parse_sha256(require(compiled, "policy_sha256").as<std::string>()))) ||
      parse_sha256(require(compiled, "firmware_image_sha256").as<std::string>()) != identity.firmware_hash ||
      !equal_file_hash("firmware_sha256", request.firmware_image_path) ||
      !equal_file_hash("host_sha256", request.host_executable_path) ||
      !equal_file_hash("map_sha256", request.map_file_path) ||
      !equal_file_hash("resource_report_sha256", request.resource_report_path))
    invalid("no-load development binding mismatch");
  /**
   * @brief 功能依据必须live profile等于编译policy，且不允许冒称formal post_build_bindings；分50轮与LIMITED一轮核报告。
   */
  if (functional_basis) {
    if (!equal_hash("live_profile_sha256", identity.policy_hash))
      invalid("functional live profile identity mismatch");
    if (compiled["post_build_bindings"]) invalid("functional basis cannot claim formal post-build bindings");
    if (limited_basis) verify_limited_functional_basis(activation, identity, request, prebuild);
    else verify_functional_basis(activation, identity, request, prebuild);
  } else {
    /**
     * @brief 软件依据要求真实qualification报告与post-build中产物身份一致，报告scope必须SOFTWARE_ONLY；PASS只限该软件范围。
     */
    const YAML::Node post_build = require(compiled, "post_build_bindings");
    if (!equal_file_hash("qualification_report_sha256", request.qualification_report_path) ||
        parse_sha256(require(post_build, "bundle_manifest_sha256").as<std::string>()) !=
            parse_sha256(prebuild_sha256) ||
        parse_sha256(require(post_build, "firmware_sha256").as<std::string>()) != identity.firmware_hash ||
        !equal_hash("host_sha256", parse_sha256(require(post_build, "host_sha256").as<std::string>())) ||
        !equal_hash("map_sha256", parse_sha256(require(post_build, "map_sha256").as<std::string>())) ||
        !equal_hash("resource_report_sha256",
                    parse_sha256(require(post_build, "resource_report_sha256").as<std::string>())) ||
        !equal_hash("qualification_report_sha256",
                    parse_sha256(require(post_build, "qualification_report_sha256").as<std::string>())))
      invalid("software-only post-build binding mismatch");
    const YAML::Node report = detail::load_pinned_yaml(
        request.qualification_report_path,
        require(activation, "qualification_report_sha256").as<std::string>(), {});
    if (require(report, "qualification_scope").as<std::string>() != "SOFTWARE_ONLY" ||
        require(report, "result").as<std::string>() != "PASS")
      invalid("software-only qualification report required");
  }
  /**
   * @brief 按明确device/VID/PID/interface/serial及固定串口端点字符串比对activation；只是配置身份匹配，不进行实时USB枚举、打开串口或实机校验。
   */
  static const std::regex id_pattern("[0-9a-f]{32}");
  static const std::regex serial_pattern("[A-Za-z0-9]+");
  if (!std::regex_match(request.device_id, id_pattern) ||
      request.device_id == std::string(32U, '0') ||
      !std::regex_match(request.usb_serial, serial_pattern) ||
      require(activation, "device_id").as<std::string>() != request.device_id ||
      require(activation, "usb_vid").as<unsigned>() != 0x1209U ||
      require(activation, "usb_pid").as<unsigned>() != 0x7461U ||
      require(activation, "usb_interface").as<unsigned>() != 0U ||
      require(activation, "usb_serial").as<std::string>() != request.usb_serial ||
      request.endpoint != "serial//dev/serial/by-id/usb-Dummy_Robotics_STM32_Virtual_ComPort_in_FS_Mode_" +
                          request.usb_serial + "-if00#baudrate=115200" ||
      require(activation, "endpoint").as<std::string>() != request.endpoint)
    invalid("no-load development device identity mismatch");
  return {functional_basis, limited_basis ? NoLoadBasis::kFunctionalLimited :
          (functional_basis ? NoLoadBasis::kFunctional50 : NoLoadBasis::kSoftwareOnly),
          functional_basis ? parse_sha256(require(activation, "live_profile_sha256").as<std::string>()) :
                             identity.profile_hash};
}
}  // namespace

namespace detail {
/**
 * @brief 只读取一次文件字节，先对外部期望SHA，再从同一快照解析。
 * @param path 待验证文件。
 * @param expected_sha256 小写64字符期望原SHA。
 * @param after_snapshot 可空测试钩子，核SHA后调用，即使它修改磁盘也不改变本次已核字节。
 * @return YAML解析节点；错误抛异常，不能把解析成功等同于设备授权。
 */
YAML::Node load_pinned_yaml(const fs::path &path, const std::string &expected_sha256,
                            const std::function<void()> &after_snapshot) {
  const std::string bytes = read_file(path);
  if (parse_sha256(sha256_text(bytes)) != parse_sha256(expected_sha256))
    invalid("bundle payload hash mismatch");
  if (after_snapshot) after_snapshot();
  return YAML::Load(bytes);
}
}  // namespace detail

/**
 * @brief 在完整bundle payload自洽后提取指定profile身份。
 * @param bundle_root 候选根目录。
 * @param profile_name 安全名称，model/profile须在manifest中。
 * @return DeviceBundleIdentity，错误统一拒绝为invalid_argument。
 * @note 不替代load_device_profile的轴/限制/activation检查，也不建立在线会话。
 */
DeviceBundleIdentity inspect_device_bundle_identity(const std::string &bundle_root,
                                                    const std::string &profile_name) {
  try {
    const auto bundle = verify_bundle(bundle_root);
    return bundle_identity(bundle, profile_name);
  } catch (const std::invalid_argument &) {
    throw;
  } catch (const std::exception &) {
    invalid("invalid device profile bundle");
  }
}

/**
 * @brief 比较Status提取的五类hash与已加载部署期望。
 * @param expected 固定bundle身份及firmware_bound策略。
 * @param actual 来自实际Status的hash值，本结构无session/新鲜度。
 * @return contract/model/profile/policy精确相同，实际固件非零，绑定固件时再要求精确相同。
 */
bool status_matches_bundle_identity(const DeviceBundleIdentity &expected,
                                    const DeviceStatusIdentity &actual) noexcept {
  const bool firmware_present = std::any_of(actual.firmware_hash.begin(), actual.firmware_hash.end(),
                                            [](auto byte) { return byte != 0U; });
  return expected.contract_hash == actual.contract_hash && expected.model_hash == actual.model_hash &&
         expected.profile_hash == actual.profile_hash && expected.policy_hash == actual.policy_hash &&
         firmware_present && (!expected.firmware_bound || expected.firmware_hash == actual.firmware_hash);
}

/**
 * @brief 校验目标及停车/完成/Hold预算的无溢出覆盖关系。
 * @param goal_us 单目标上限µs，非零。
 * @param trial_us 本轮总上限µs，非零且≤1小时。
 * @param stop_us 停车预留µs。
 * @param completion_us 完成收敛/确认预留µs。
 * @param hold_us 本地Hold预留µs。
 * @param explicit_permit_us 可选明确permit，长目标≥600秒必须提供。
 * @return 合法permit上限µs；默认min(60秒,trial)，非法/覆盖不足抛invalid_argument。
 * @note 只计算部署接受域，不签发permit，不延长session；逐项用剩余量检查后才求最终和。
 */
std::uint64_t validated_motion_permit_limit(std::uint64_t goal_us,
    std::uint64_t trial_us, std::uint64_t stop_us,
    std::uint64_t completion_us, std::uint64_t hold_us,
    std::optional<std::uint64_t> explicit_permit_us) {
  constexpr std::uint64_t kMaximumBudgetUs=3600000000ULL;
  if(goal_us==0U || trial_us==0U || stop_us==0U || completion_us==0U ||
     hold_us==0U || trial_us>kMaximumBudgetUs || goal_us>trial_us ||
     stop_us>trial_us-goal_us ||
     completion_us>trial_us-goal_us-stop_us ||
     hold_us>trial_us-goal_us-stop_us-completion_us)
    invalid("invalid bounded motion permit policy");
  if(!explicit_permit_us && goal_us>=600000000U)
    invalid("long goal requires explicit motion permit budget");
  const auto permit=explicit_permit_us.value_or(std::min<std::uint64_t>(60000000U,trial_us));
  if(permit==0U || permit>trial_us ||
     permit<goal_us+stop_us+completion_us+hold_us)
    invalid("motion permit does not cover bounded goal and cleanup");
  return permit;
}

/**
 * @brief 按profile角色加载轴/动态/范围和可用开发依据，所有检查通过才返回值。
 * @param bundle_root bundle根。
 * @param profile_name SIL或明确候选profile名称。
 * @param expected_id 非零部署profile ID。
 * @param expected_mask 仅1或63，须与active_axes构成掩码相同。
 * @param activation 可空；f405_6dof_bench必须有显式外部pin及证据，普通test-only不需该开发声明。
 * @return 完整DeviceProfile；错误抛invalid_argument。
 * @note 本地加载结果不是设备当前启用/许可或正式发布资格；所有profile都拒绝motion_permitted_on_real_hardware=true。
 */
DeviceProfile load_device_profile_impl(const std::string &bundle_root, const std::string &profile_name,
                                       std::uint32_t expected_id, std::uint8_t expected_mask,
                                       const NoLoadDevActivationRequest *activation) {
  try {
    if (!safe_profile_name(profile_name) || expected_id == 0U || (expected_mask != 1U && expected_mask != 63U))
      invalid("invalid requested device profile");
    const auto bundle = verify_bundle(bundle_root);
    /**
     * @brief SIL固定可信聚合，不允许任意重新算过自洽manifest的伪fixture；此固定来源约束独立于bundle的一致性校验。
     */
    if ((profile_name == "sil_j1" || profile_name == "sil_6dof") &&
        bundle.aggregate != "0afb58ddfe9ce339c75da3c527b31abe875a261b3ac270e89d445ed96eda497b" &&
        bundle.manifest_sha256 != DUMMY_INSTALLED_ROBOT_MANIFEST_SHA256)
      invalid("untrusted SIL fixture bundle");
    const auto identity = bundle_identity(bundle, profile_name);
    DeviceProfile output;
    static_cast<DeviceBundleIdentity &>(output) = identity;
    const std::string model_key = "model/model.yaml";
    const std::string profile_key = "profiles/" + profile_name + ".yaml";
    const YAML::Node model = detail::load_pinned_yaml(
        bundle.root / model_key, bundle.payloads.at(model_key), {});
    const YAML::Node profile = detail::load_pinned_yaml(
        bundle.root / profile_key, bundle.payloads.at(profile_key), {});
    const bool bench_only = profile_name == "f405_6dof_bench";
    if (require(profile, "profile_name").as<std::string>() != profile_name ||
        require(profile, "profile_id").as<std::uint32_t>() != expected_id ||
        !require(profile, "uses_dpsdk_device").as<bool>() ||
        require(profile, "motion_permitted_on_real_hardware").as<bool>()) invalid("profile authority mismatch");
    /**
     * @brief f405 bench固定ID201/掩码63、BENCH_SESSION、APPLICATION_HOLD/无负载/非重力/指定终态和runtime_enabled=false候选；必须另外通过activation，不靠profile文字自授权。
     */
    if (bench_only) {
      const YAML::Node capability = require(bundle.manifest, "capability");
      if (expected_id != 201U || expected_mask != 63U || require(profile, "test_only").as<bool>() ||
          !require(profile, "motion_supported").as<bool>() ||
          require(profile, "reference_kind").as<std::string>() != "BENCH_SESSION" ||
          require(bundle.manifest, "runtime_enabled").as<bool>() ||
          require(capability, "profile_id").as<std::uint32_t>() != expected_id ||
          require(capability, "configured_mask").as<std::uint32_t>() != expected_mask ||
          require(capability, "motion_permitted_on_real_hardware").as<bool>())
        invalid("bench profile authority mismatch");
      if (require(bundle.manifest, "artifact_kind").as<std::string>() != "application_hold_no_load_candidate" ||
          require(profile, "no_load_bench").as<bool>() != true ||
          require(profile, "gravity_loaded").as<bool>() != false ||
          require(profile, "deployment_stop_policy").as<std::string>() != "APPLICATION_HOLD" ||
          require(profile, "safe_terminal_action").as<std::string>() != "DisableAndReleaseNoLoadBench" ||
          activation == nullptr) invalid("no-load development activation required");
      const auto activated = verify_no_load_activation(bundle, identity, *activation);
      output.functional_development_basis = activated.functional_basis;
      output.no_load_basis = activated.basis;
      /**
       * @brief LIMITED从已钉activation取目标数/新timeline路径，从已钉HIL取唯一非零16进制boot；后续运行必须保持这些限制，loader不把它们变成无限预算。
       */
      if (activated.basis == NoLoadBasis::kFunctionalLimited) {
        const auto limited = detail::load_pinned_yaml(
            activation->activation_path, activation->expected_activation_sha256, {});
        output.goal_limit = require(limited, "goal_limit").as<unsigned>();
        output.limited_goal_timeline_path = require(limited, "goal_timeline_path").as<std::string>();
        const auto hil = detail::load_pinned_yaml(activation->hil_report_path,
            require(limited, "hil_report_sha256").as<std::string>(), {});
        const auto boot_text = require(hil, "boot_epochs")[0].as<std::string>();
        const auto parsed = std::from_chars(boot_text.data(), boot_text.data()+boot_text.size(),
                                            output.limited_boot_epoch, 16);
        if(parsed.ec!=std::errc{} || parsed.ptr!=boot_text.data()+boot_text.size() ||
           output.limited_boot_epoch==0U) invalid("limited functional boot invalid");
      }
      output.profile_hash = activated.live_profile_hash;
      output.bench_only = true;
      output.software_only_development = !output.functional_development_basis;
    } else if (!require(profile, "test_only").as<bool>()) invalid("profile authority mismatch");
    const YAML::Node active = require(profile, "active_axes");
    const YAML::Node model_joints = require(model, "joints");
    if (!active.IsSequence() || !model_joints.IsSequence() || (active.size() != 1U && active.size() != 6U)) invalid("invalid active axes");
    /**
     * @brief sil_6dof采用相对bench工作域；普通profile用模型机械上下限，bench用编译policy逐轴trial范围，不能混作同一绝对坐标范围。
     */
    const bool relative_sil = profile_name == "sil_6dof";
    YAML::Node bench_axes;
    if (bench_only) {
      const std::string policy_key = "policy/application_hold_no_load.json";
      if (require(profile, "source_policy").as<std::string>() != policy_key ||
          bundle.payloads.find(policy_key) == bundle.payloads.end())
        invalid("bench policy payload missing");
      const YAML::Node policy = detail::load_pinned_yaml(
          bundle.root / policy_key, bundle.payloads.at(policy_key), {});
      const YAML::Node timing = require(policy, "timing");
      /**
       * @brief 从policy timing读明确或默认permit上限，同时保留目标/停车/完成/Hold全部预算，范围不足即拒绝。
       */
      std::optional<std::uint64_t> explicit_permit;
      if(timing["motion_permit_us"])
        explicit_permit=timing["motion_permit_us"].as<std::uint64_t>();
      output.motion_permit_max_us=validated_motion_permit_limit(
          require(timing,"max_goal_duration_us").as<std::uint64_t>(),
          require(timing,"trial_max_duration_us").as<std::uint64_t>(),
          require(timing,"stop_max_duration_us").as<std::uint64_t>(),
          require(timing,"completion_max_duration_us").as<std::uint64_t>(),
          require(timing,"local_hold_budget_us").as<std::uint64_t>(),
          explicit_permit);
      const YAML::Node policy_active = require(policy, "active_axes");
      bench_axes = require(policy, "axes");
      if (require(policy, "reference_kind").as<std::string>() != "BENCH_SESSION" ||
          require(policy, "model_sha256").as<std::string>() != bundle.payloads.at(model_key) ||
          !policy_active.IsSequence() || policy_active.size() != active.size() ||
          !bench_axes.IsSequence() || bench_axes.size() != active.size())
        invalid("bench policy reference frame mismatch");
      for (std::size_t axis = 0U; axis < active.size(); ++axis)
        if (policy_active[axis].as<std::string>() != active[axis].as<std::string>())
          invalid("bench policy axis order mismatch");
    }
    double trial_lower = 0.0, trial_upper = 0.0;
    if (relative_sil) {
      if (require(profile, "reference_kind").as<std::string>() != "BENCH_SESSION" ||
          !require(profile, "test_only").as<bool>()) invalid("SIL reference frame mismatch");
      const YAML::Node range = require(profile, "bench_position_range_rad");
      const double travel = finite_positive(require(require(profile, "simulation"), "max_trial_travel_rad"));
      if (!range.IsSequence() || range.size() != 2U) invalid("invalid SIL trial range");
      trial_lower = range[0U].as<double>();
      trial_upper = range[1U].as<double>();
      if (!std::isfinite(trial_lower) || !std::isfinite(trial_upper) || trial_lower >= 0.0 ||
          trial_upper <= 0.0 || trial_lower < -travel || trial_upper > travel)
        invalid("SIL trial range exceeds Core travel");
    }
    /**
     * @brief 模型关节名必须唯一；按active_axes顺序装配lower/upper并核配置掩码，位置单位rad。
     */
    std::unordered_map<std::string, YAML::Node> limits;
    for (const YAML::Node &joint : model_joints) {
      const std::string name = require(joint, "name").as<std::string>();
      if (!limits.emplace(name, require(joint, "model_limits_rad")).second) invalid("duplicate model joint");
    }
    std::uint8_t mask = 0U;
    for (std::size_t axis_index = 0U; axis_index < active.size(); ++axis_index) {
      const YAML::Node axis = active[axis_index];
      const std::string name = axis.as<std::string>();
      const std::uint8_t bit = axis_bit(name);
      if ((mask & bit) != 0U || limits.find(name) == limits.end()) invalid("profile/model axis mismatch");
      const YAML::Node range = limits.at(name);
      if (!range.IsSequence() || range.size() != 2U) invalid("invalid model limits");
      const double mechanical_lower = range[0U].as<double>(), mechanical_upper = range[1U].as<double>();
      if (!std::isfinite(mechanical_lower) || !std::isfinite(mechanical_upper) ||
          mechanical_lower >= mechanical_upper) invalid("invalid model limits");
      double lower = relative_sil ? trial_lower : mechanical_lower;
      double upper = relative_sil ? trial_upper : mechanical_upper;
      if (bench_only) {
        lower = require(bench_axes[axis_index], "trial_min_rad").as<double>();
        upper = require(bench_axes[axis_index], "trial_max_rad").as<double>();
        if (!std::isfinite(lower) || !std::isfinite(upper) || lower >= 0.0 ||
            upper <= 0.0) invalid("invalid bench policy trial limits");
      }
      output.joints.joints.push_back(name); output.joints.lower.push_back(lower); output.joints.upper.push_back(upper); mask |= bit;
    }
    if (mask != expected_mask) invalid("configured mask drift");
    /**
     * @brief 速度rad/s、加速度rad/s²、path/goal误差rad从profile simulation字段读取有限正值；字段名不使其自动等于实际设备动态能力。
     */
    const YAML::Node simulation = require(profile, "simulation");
    const double velocity = finite_positive(require(simulation, "velocity_limit_rad_s"));
    const double acceleration = finite_positive(require(simulation, "acceleration_limit_rad_s2"));
    const double path_tolerance = finite_positive(require(simulation, "max_tracking_error_rad"));
    const double goal_tolerance = finite_positive(require(simulation, "hold_position_tolerance_rad"));
    output.joints.velocity.assign(active.size(), velocity);
    output.joints.acceleration.assign(active.size(), acceleration);
    output.joints.path_tolerance.assign(active.size(), path_tolerance);
    output.joints.goal_tolerance.assign(active.size(), goal_tolerance);
    return output;
  } catch (const std::invalid_argument &) {
    throw;
  } catch (const std::exception &) {
    invalid("invalid device profile bundle");
  }
}

/**
 * @brief 无activation的普通加载包装；不能用于要求显式激活的f405 bench。
 * @param bundle_root bundle根目录。
 * @param profile_name 目标profile名。
 * @param expected_id 预期非零ID。
 * @param expected_mask 1或63。
 * @return 已验证本机描述或抛invalid_argument，不连接设备。
 */
DeviceProfile load_device_profile(const std::string &bundle_root, const std::string &profile_name,
                                  std::uint32_t expected_id, std::uint8_t expected_mask) {
  return load_device_profile_impl(bundle_root, profile_name, expected_id, expected_mask, nullptr);
}

/**
 * @brief 带外部独立activation pin及真实产物路径的开发加载包装。
 * @param bundle_root 候选目录。
 * @param profile_name 目标profile名，bench有专门规则。
 * @param expected_id 预期ID。
 * @param expected_mask 1或63，bench固定63。
 * @param activation 外部摘要、证据根和设备端点绑定。
 * @return 保留SoftwareOnly/Functional50/FunctionalLimited限制的DeviceProfile，失败抛invalid_argument。
 */
DeviceProfile load_device_profile(const std::string &bundle_root, const std::string &profile_name,
                                  std::uint32_t expected_id, std::uint8_t expected_mask,
                                  const NoLoadDevActivationRequest &activation) {
  return load_device_profile_impl(bundle_root, profile_name, expected_id, expected_mask, &activation);
}
}  // namespace dummy_motion_client
