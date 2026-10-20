/**
 * @file bench_permit_tests.cpp
 * @brief 核可信BENCH permit摘要黄金值和拒绝输入。
 *
 * 纯宿主内存摘要测试；assert须在验证构建中启用。固定hash/boot/ref/client/session/seq仅为夹具，不签发运行许可或验证现场批准来源。
 */
#include "dummy_motion_client/bench_permit.hpp"

#include <cassert>
#include <string>

/**
 * @brief 计算固定绑定的permit SHA并逐字节对黄金向量。
 * @return 0表示所选检查通过；非零或断言/require异常表示失败。
 * boot7/ref3/profile101及四hash绑定client0x55/session9/请求seq11；拒绝seq0和全零固件hash。摘要正确不代表真实固件或机械资格。
 */
int main() {
  using namespace dummy_motion_client;
  PermitBinding binding{};
  binding.boot_epoch = 7U;
  binding.reference_epoch = 3U;
  binding.profile_id = 101U;
  binding.firmware_sha256[0] = 1U;
  binding.contract_sha256[0] = 2U;
  binding.model_sha256[0] = 3U;
  binding.profile_sha256[0] = 4U;
  std::array<std::uint8_t, 16> client{};
  client[0] = 0x55U;
  std::array<std::uint8_t, 32> digest{};
  assert(trusted_bench_permit_digest(binding, client, 9U, 11U, &digest));
  constexpr char hex[] = "0123456789abcdef";
  std::string actual;
  for (auto byte : digest) { actual += hex[byte >> 4U]; actual += hex[byte & 15U]; }
  assert(actual == "c43173a8461f3048b9cf82b6b5c6443f9e3861319cde7e70c2ce78032ee3b151");
  assert(!trusted_bench_permit_digest(binding, client, 9U, 0U, &digest));
  binding.firmware_sha256 = {};
  assert(!trusted_bench_permit_digest(binding, client, 9U, 11U, &digest));
}
