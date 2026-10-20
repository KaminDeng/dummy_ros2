/**
 * @file bench_permit.cpp
 * @brief 台架许可摘要的固定布局编码；与设备侧域标签、字节序和字段顺序一致。
 */
#include "dummy_motion_client/bench_permit.hpp"

#include <algorithm>
#include <cstring>
#include <openssl/sha.h>

namespace dummy_motion_client {
namespace {
/**
 * @brief 将整数逐字节写成小端表示，不依赖Host结构体布局或主机字节序。
 * @param output 调用方保证至少sizeof(Integer)字节可写；当前实例输入为无符号整型。
 */
template <typename Integer>
void append_le(std::uint8_t *output, Integer value) noexcept {
  for (std::size_t index = 0; index < sizeof(Integer); ++index)
    output[index] = static_cast<std::uint8_t>(value >> (index * 8U));
}
/**
 * @brief 摘要至少有一个非零字节即通过身份非空检查；不验证摘要真实性。
 */
bool nonzero(const std::array<std::uint8_t, 32> &value) noexcept {
  return std::any_of(value.begin(), value.end(), [](auto byte) { return byte != 0U; });
}
}  // namespace

/**
 * @brief 验证必填身份，再按固定域/boot/session/client/seq/ref/profile/四摘要顺序计算SHA256。
 * @details 域字符串不包含末尾NUL；总长193字节，没有结构体填充。摘要只是绑定这些输入，不提供密钥认证。
 */
bool trusted_bench_permit_digest(const PermitBinding &binding,
                                 const std::array<std::uint8_t, 16> &client,
                                 std::uint64_t session_id, std::uint32_t request_seq,
                                 std::array<std::uint8_t, 32> *output) noexcept {
  if (output == nullptr || binding.boot_epoch == 0U || binding.reference_epoch == 0U ||
      binding.profile_id == 0U || session_id == 0U || request_seq == 0U ||
      !nonzero(binding.firmware_sha256) || !nonzero(binding.contract_sha256) ||
      !nonzero(binding.model_sha256) || !nonzero(binding.profile_sha256)) return false;
  /**
   * @brief 域分离标签，防止与其他用途的相同字段拼接混淆；sizeof减1排除NUL。
   */
  constexpr char domain[] = "DUMMY-BENCH-PERMIT-V1";
  /**
   * @brief 固定大小的栈缓冲，offset每次按实际宽度推进，最终必须恰好填满。
   */
  std::array<std::uint8_t, 193> bytes{};
  std::size_t offset = sizeof(domain) - 1U;
  std::memcpy(bytes.data(), domain, offset);
  append_le(bytes.data() + offset, binding.boot_epoch); offset += 8U;
  append_le(bytes.data() + offset, session_id); offset += 8U;
  std::copy(client.begin(), client.end(), bytes.begin() + offset); offset += client.size();
  append_le(bytes.data() + offset, request_seq); offset += 4U;
  append_le(bytes.data() + offset, binding.reference_epoch); offset += 4U;
  append_le(bytes.data() + offset, binding.profile_id); offset += 4U;
  /**
   * @brief 四摘要顺序属于协议：固件、合同、模型、profile；交换顺序会改变许可摘要。
   */
  for (const auto *hash : {&binding.firmware_sha256, &binding.contract_sha256,
                           &binding.model_sha256, &binding.profile_sha256}) {
    std::copy(hash->begin(), hash->end(), bytes.begin() + offset);
    offset += hash->size();
  }
  return offset == bytes.size() && SHA256(bytes.data(), bytes.size(), output->data()) != nullptr;
}
}  // namespace dummy_motion_client
