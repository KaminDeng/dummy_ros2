/**
 * @file bench_permit.hpp
 * @brief 空载台架许可摘要输入及确定性 SHA-256 接口。
 * @details 摘要将上电/参考、客户端、会话、请求序号和固件/合同/模型/profile身份绑定；它是确定性绑定值，不是带密钥签名，也不会自行授予运动。
 */
#pragma once

#include <array>
#include <cstdint>

namespace dummy_motion_client {
/**
 * @brief 参与许可摘要的设备身份快照；所有整数按固定宽度小端序编码。
 * @details boot_epoch/reference_epoch/profile_id 与四个32字节摘要均需非零；本结构不包含会话和客户端，这两项从函数参数提供。
 */
struct PermitBinding {
  std::uint64_t boot_epoch = 0;
  std::uint32_t reference_epoch = 0;
  std::uint32_t profile_id = 0;
  std::array<std::uint8_t, 32> firmware_sha256{};
  std::array<std::uint8_t, 32> contract_sha256{};
  std::array<std::uint8_t, 32> model_sha256{};
  std::array<std::uint8_t, 32> profile_sha256{};
};

/**
 * @brief 将固定域标签及绑定身份编码为193字节，再计算SHA-256。
 * @param binding 设备上电/参考/profile和四个产物摘要。
 * @param client 16字节客户端标识，当前函数不另检查它是否全零。
 * @param session_id 非零控制会话。
 * @param request_seq 非零请求序号，防止不同请求共用同一摘要输入。
 * @param output 非空32字节输出；前置校验失败时不写入。
 * @return 前置条件、编码长度及SHA256调用均成功才返回 true；false 不应被当作可使用的许可摘要。
 */
bool trusted_bench_permit_digest(const PermitBinding &binding,
                                 const std::array<std::uint8_t, 16> &client,
                                 std::uint64_t session_id, std::uint32_t request_seq,
                                 std::array<std::uint8_t, 32> *output) noexcept;
}  // namespace dummy_motion_client
