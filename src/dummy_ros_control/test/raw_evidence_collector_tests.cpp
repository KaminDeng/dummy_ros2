/**
 * @file raw_evidence_collector_tests.cpp
 * @brief 离线验证原始证据收集器的哈希链、文件writer和共享回执边界。
 *
 * 页面和管理帧全部由本文件合成；SHA-256在夹具端调用中央规范Hash purpose重算，避免篡改正文却意外只测试坏hash。
 * 测试涉及本地临时文件、writer线程、并发reader和fork隔离的文件大小限制；不连接peer或操作硬件。
 * 所有offer的接收时间是人工Host µs，与设备Source时钟无映射关系。first-page-written/terminal-flushed是采集文件写入/flush回执，不表示电机运动完成、物理停止或管理release提交。
 */
#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <chrono>
#include <string>
#include <sys/stat.h>
#include <sys/resource.h>
#include <sys/wait.h>
#include <signal.h>
#include <thread>
#include <unistd.h>

#include <openssl/sha.h>
#include "dummy_ref/wire_validation.hpp"
#include "owned_message_fixtures.hpp"
#include "canonical_records.h"
#include <vector>
#include "dummy_ros_control/device_management_channel.hpp"
#include "dummy_ros_control/raw_evidence_collector.hpp"

using dummy_ros_control::RawEvidenceCollector;
using dummy_ros_control::RawEvidenceBinding;
using dummy_ros_control::RawEvidenceBindings;
using dummy_ros_control::RawEvidenceIdentity;
using Evidence=dummy_ref::messages::Evidence;


namespace {
/**
 * @brief 建立固定采集信任锚；client和四种hash首字节故意各异。
 * @return 其余字节为零的测试身份，不是已部署设备真实身份。
 */
RawEvidenceIdentity identity() {
  RawEvidenceIdentity value{};
  value.client_id[0] = 1U;
  value.contract_hash[0] = 2U;
  value.model_hash[0] = 3U;
  value.profile_hash[0] = 4U;
  value.firmware_hash[0] = 5U;
  return value;
}

/**
 * @brief 按小端run/boot和信任身份组成192字节genesis输入并求SHA-256。
 * @param trust 本次采集使用的client/contract/model/profile/firmware锚。
 * @param run 逻辑attempt/run标识，非时间。
 * @param boot 设备启动代次，非主机进程号。
 * @return 固定32字节链起点；尾部未写入区保持零填充。
 */
std::array<std::uint8_t, 32U> genesis(const RawEvidenceIdentity &trust,
                                       std::uint64_t run, std::uint64_t boot) {
  std::array<std::uint8_t, 192U> input{};
  for (unsigned i = 0; i < 8U; ++i) {
    input[i] = static_cast<std::uint8_t>(run >> (i * 8U));
    input[8U + i] = static_cast<std::uint8_t>(boot >> (i * 8U));
  }
  std::copy(trust.client_id.begin(), trust.client_id.end(), input.begin() + 16U);
  std::copy(trust.contract_hash.begin(), trust.contract_hash.end(), input.begin() + 32U);
  std::copy(trust.model_hash.begin(), trust.model_hash.end(), input.begin() + 64U);
  std::copy(trust.profile_hash.begin(), trust.profile_hash.end(), input.begin() + 96U);
  std::copy(trust.firmware_hash.begin(), trust.firmware_hash.end(), input.begin() + 128U);
  std::array<std::uint8_t, 32U> result{};
  ::SHA256(input.data(), input.size(), result.data());
  return result;
}

/**
 * @brief 直接生成实际Owned并计算规范链，并计算当前链摘要。
 * @param run 写入run_id和attempt_id的同一运行标识。
 * @param seq 此run内证据序号，首个页面应为0。
 * @param previous 前一页摘要；首个页面使用genesis。
 * @param terminal true生成category=4终态，false生成category=2组页。
 * @param group_count 报告组数；UINT32_MAX选择夹具默认（无组的首终态为0，否则1）。
 * @param final_flag 页面final位，不等于运动完成标志。
 * @param kind attempt_kind夹具值，默认为1。
 * @param parent 后继attempt的父run标识，普通run为0。
 * @param mask 非终态页的expected/submitted轴位掩码，默认63表示六轴。
 * @param sent_mask 实际发送位夹具；UINT8_MAX表示与mask相同。
 * @param failed_mask 发送失败轴位夹具。
 * @return boot=11/session=12的独立页面；编码失败abort使夹具构造立即失败。
 */
Evidence page(std::uint64_t run, std::uint32_t seq,
                     const std::array<std::uint8_t, 32U> &previous,
                     bool terminal, std::uint32_t group_count = UINT32_MAX,
                     bool final_flag = true, std::uint8_t kind = 1U,
                     std::uint64_t parent = 0U, std::uint8_t mask = 63U,
                     std::uint8_t sent_mask = UINT8_MAX, std::uint8_t failed_mask = 0U) {
  if (group_count == UINT32_MAX) group_count = terminal && seq == 0U ? 0U : 1U;
  auto message=owned_fixture::evidence();


  message.run_id()=run;
  message.attempt_id()=run;
  message.evidence_seq()=seq;
  message.attempt_kind()=kind;
  message.parent_run_id()=parent;
  message.category()=terminal ? 4U : 2U;
  message.final()=final_flag ? 1U : 0U;
  message.group_count()=group_count;
  message.motion_epoch()=10U;
  if (terminal) {
    message.final_state()=3U;
    if (group_count == 0U) message.terminal_reason()=14U;
  }
  else {
    message.group_id()=7U;
    message.expected_mask()=mask;
    message.submitted_mask()=mask;
    message.sent_mask()=sent_mask == UINT8_MAX ? mask : sent_mask;
    message.failed_mask()=failed_mask;
  }
  std::uint8_t previous_bytes[32]{};
  std::copy(previous.begin(), previous.end(), previous_bytes);
  message.previous_chain_sha256().emplace().assign(previous_bytes,32U);
  using Traits=dp::protocol::canonical::Traits<Evidence>;
  std::array<std::uint8_t,Traits::kMaxEncodedSize> encoded{};std::size_t written=0;
  if(!Traits::encode(message,encoded.data(),encoded.size(),&written,dp::protocol::canonical::Purpose::kEvidenceHash))std::abort();
  std::vector<std::uint8_t> input(previous.begin(),previous.end());input.insert(input.end(),encoded.begin(),encoded.begin()+written);
  std::array<std::uint8_t,32> next{};SHA256(input.data(),input.size(),next.data());message.chain_sha256().emplace().assign(next.data(),next.size());return message;
}

/**
 * @brief 改过实际Owned字段后重算规范hash，以隔离语义负例和坏链负例。
 * @param value 要更新raw及chain_sha256的页面；previous_chain_sha256沿用其现值。
 * @param message 调用方已修改的wire正文，previous/chain字段由此函数覆盖。
 * @return 无；任一步编码失败立即abort。
 * @note 页面只有一份实际Owned，回写时一次替换，来源与会话绑定另行传入。
 */
template<class Message>std::vector<std::uint8_t> canonical_bytes(const Message& value,dp::protocol::canonical::Purpose purpose=dp::protocol::canonical::Purpose::kRecord){
 using Traits=dp::protocol::canonical::Traits<Message>;std::vector<std::uint8_t> bytes(Traits::kMaxEncodedSize);std::size_t written=0;
 if(!Traits::encode(value,bytes.data(),bytes.size(),&written,purpose))std::abort();bytes.resize(written);return bytes;
}
std::array<std::uint8_t,32> chain_of(const Evidence& value){std::array<std::uint8_t,32> result{};std::copy(value.chain_sha256()->begin(),value.chain_sha256()->end(),result.begin());return result;}
dp::comm::SourceId test_source(std::uint64_t boot=11){dp::comm::SourceId value{};value.device.bytes[0]=9;value.boot_id=boot;value.context_id=1;return value;}
bool offer(RawEvidenceCollector& collector,const Evidence& value,std::uint64_t received,std::uint64_t boot=11,std::uint64_t session=12){return collector.offer_page(value,test_source(boot),session,received);}
void rewrite_page(Evidence& value,Evidence& message){
 message.previous_chain_sha256().emplace().assign(value.previous_chain_sha256()->begin(),32);
 auto bytes=canonical_bytes(message,dp::protocol::canonical::Purpose::kEvidenceHash);
 std::vector<std::uint8_t> input(value.previous_chain_sha256()->begin(),value.previous_chain_sha256()->end());input.insert(input.end(),bytes.begin(),bytes.end());
 std::array<std::uint8_t,32> next{};SHA256(input.data(),input.size(),next.data());message.chain_sha256().emplace().assign(next.data(),next.size());value=message;
}

/**
 * @brief 重新计算合法hash后拒绝attempt_id与run_id不同的页面。
 *
 * 解码首组页后将attempt_id改为4、run保持3；应归类kInvalid，而不是仅因链摘要不匹配被拒绝。
 * @return true 表示本用例全部预期成立；false表示夹具准备、异步期限或结果断言失败。
 */
bool test_attempt_id_mismatch_rejected() {
  const auto trust = identity();
  auto first = page(3U, 0U, genesis(trust, 3U, 11U), false);
  auto decoded=owned_fixture::evidence();
  decoded=first;
  decoded.attempt_id()=4U;
  rewrite_page(first, decoded);
  RawEvidenceCollector collector(trust);
  return !offer(collector,first, 100U) &&
      collector.failure() == RawEvidenceCollector::Failure::kInvalid;
}

/**
 * @brief 区分组数预报领先与非终态回退。
 *
 * 非终态报告2→4→4可领先于已见组数，终态以实际3组闭合；另建collector把非终态4回退3必须报kSequence。
 * @return true 表示本用例全部预期成立；false表示夹具准备、异步期限或结果断言失败。
 */
bool test_reported_group_count_may_lead_but_not_regress() {
  const auto trust = identity();
  auto first = page(3U, 0U, genesis(trust, 3U, 11U), false, 2U);
  auto second = page(3U, 1U, chain_of(first), false, 4U);
  auto third = page(3U, 2U, chain_of(second), false, 4U);
  auto terminal = page(3U, 3U, chain_of(third), true, 3U);
  RawEvidenceCollector legal(trust);
  if (!legal.expect_run(3U) || !offer(legal,first, 100U) ||
      !offer(legal,second, 101U) || !offer(legal,third, 102U) ||
      !offer(legal,terminal, 103U)) return false;
  legal.stop();
  auto regressed = page(3U, 2U, chain_of(second), false, 3U);
  RawEvidenceCollector invalid(trust);
  return legal.failure() == RawEvidenceCollector::Failure::kNone &&
      invalid.expect_run(3U) && offer(invalid,first, 100U) &&
      offer(invalid,second, 101U) &&
      !offer(invalid,regressed, 102U) &&
      invalid.failure() == RawEvidenceCollector::Failure::kSequence;
}


/**
 * @brief 覆盖完整/缺终态run、缺页和坏链的收尾判定。
 *
 * 已expect的run只有组页或完全没页，stop后均kIncomplete；非final组页kInvalid、首终态声称存在未见组kSequence，genesis或当前hash破坏kChain。
 * @return true 表示本用例全部预期成立；false表示夹具准备、异步期限或结果断言失败。
 */
bool test_lifecycle_and_chain() {
  const auto trust = identity();
  const auto group = page(3U, 0U, genesis(trust, 3U, 11U), false);
  const auto terminal = page(3U, 1U, chain_of(group), true);
  RawEvidenceCollector complete(trust);
  if (!complete.expect_run(3U) || !offer(complete,group, 100U) ||
      !offer(complete,terminal, 101U)) return false;
  complete.stop();
  if (complete.failure() != RawEvidenceCollector::Failure::kNone) return false;
  RawEvidenceCollector incomplete(trust);
  if (!incomplete.expect_run(3U) || !offer(incomplete,group, 100U)) return false;
  incomplete.stop();
  if (incomplete.failure() != RawEvidenceCollector::Failure::kIncomplete) return false;
  RawEvidenceCollector no_page(trust);
  if (!no_page.expect_run(3U)) return false;
  no_page.stop();
  if (no_page.failure() != RawEvidenceCollector::Failure::kIncomplete) return false;
  RawEvidenceCollector bad_genesis(trust);
  auto altered = group;
  (*altered.previous_chain_sha256())[0] ^= 1U;
  if (offer(bad_genesis,altered, 100U) ||
      bad_genesis.failure() != RawEvidenceCollector::Failure::kChain) return false;
  RawEvidenceCollector nonfinal_group(trust);
  const auto nonfinal = page(3U, 0U, genesis(trust, 3U, 11U), false, 1U, false);
  if (offer(nonfinal_group,nonfinal, 100U) ||
      nonfinal_group.failure() != RawEvidenceCollector::Failure::kInvalid) return false;
  RawEvidenceCollector missing_group(trust);
  const auto impossible = page(3U, 0U, genesis(trust, 3U, 11U), true, 1U);
  if (offer(missing_group,impossible, 100U) ||
      missing_group.failure() != RawEvidenceCollector::Failure::kSequence) return false;
  RawEvidenceCollector bad_digest(trust);
  altered = terminal;
  (*altered.chain_sha256())[0]^=1U;
  return offer(bad_digest,group, 100U) && !offer(bad_digest,altered, 101U) &&
      bad_digest.failure() == RawEvidenceCollector::Failure::kChain;
}

/**
 * @brief 逐字节读回管理请求/响应的中央规范采集文件。
 *
 * 以request_id80/request_seq81及终态响应写入，读入8字节magic但此用例不比较其内容；检查39字节记录头中的选定字节、完整规范记录载荷及EOF。未完整解码Host接收时刻，也不校验远端执行。
 * @return true 表示本用例全部预期成立；false表示夹具准备、异步期限或结果断言失败。
 */
bool test_management_wire_records() {
  const auto trust = identity();
  char path[] = "/tmp/dummy-raw-management-XXXXXX";
  const int fd = ::mkstemp(path);
  if (fd < 0) return false;
  ::close(fd);
  ::unlink(path);
  RawEvidenceCollector collector(trust);
  if (!collector.start(path)) return false;
  auto request=owned_fixture::request();


  std::uint8_t client[16]{};
  client[0] = 1U;
  request.client_instance_id().emplace().assign(client,16U);
  request.request_id()=80U;
  request.request_seq()=81U;
  request.operation()=10U;
  request.expected_boot_epoch()=11U;
  request.expected_session_id()=12U;
  request.expected_reference_epoch()=9U;
  request.expected_motion_epoch()=10U;
  request.expected_management_epoch()=1U;
  auto request_raw=canonical_bytes(request);
  auto response=owned_fixture::response();


  response.client_instance_id().emplace().assign(client,16U);
  response.request_id()=80U;

  response.session_id()=12U;
  response.run_id()=2U;
  response.attempt_kind()=1U;
  response.operation_id()=80U;
  response.receipt()=1U;
  response.transaction_phase()=3U;
  response.release_committed()=true;
  auto response_raw=canonical_bytes(response);
  if (!collector.offer_management_request(request,test_source(), 100U) ||
      !collector.offer_management_response(response,test_source(), 101U)) return false;
  collector.stop();
  if (collector.failure() != RawEvidenceCollector::Failure::kNone) return false;
  std::FILE *file = std::fopen(path, "rb");
  if (!file) return false;
  std::uint8_t magic[8]{}, request_header[dummy_ros_control::kCaptureHeaderSize]{}, response_header[dummy_ros_control::kCaptureHeaderSize]{};
  std::vector<std::uint8_t> recorded_request(request_raw.size());
  std::vector<std::uint8_t> recorded_response(response_raw.size());
  const bool read = std::fread(magic, 1U, 8U, file) == 8U &&
      std::fread(request_header, 1U, dummy_ros_control::kCaptureHeaderSize, file) == dummy_ros_control::kCaptureHeaderSize &&
      std::fread(recorded_request.data(), 1U, recorded_request.size(), file) == recorded_request.size() &&
      std::fread(response_header, 1U, dummy_ros_control::kCaptureHeaderSize, file) == dummy_ros_control::kCaptureHeaderSize &&
      std::fread(recorded_response.data(), 1U, recorded_response.size(), file) == recorded_response.size() &&
      std::fgetc(file) == EOF;
  std::fclose(file);
  ::unlink(path);
  return read && request_header[0] == 4U && request_header[1] == (request_raw.size()&255U) &&
      request_header[3] == 100U && request_header[11] == 11U &&
      request_header[19] == 12U && request_header[27] == 0U && request_header[35] == 81U &&
      response_header[0] == 5U && response_header[1] == (response_raw.size()&255U) &&
      response_header[3] == 101U && response_header[11] == 11U &&
      response_header[19] == 12U && response_header[27] == 2U &&
      response_header[35] == 0U && recorded_request == request_raw &&
      recorded_response == response_raw;
}

/**
 * @brief 管理记录必须位于已打开且未stop的capture窗口。
 *
 * 未start或stop后offer应kIo；关闭后文件仅8字节magic、written=0，分别覆盖请求和响应。
 * @return true 表示本用例全部预期成立；false表示夹具准备、异步期限或结果断言失败。
 */
bool test_management_offers_require_open_capture() {
  auto request=owned_fixture::request();


  auto request_raw=canonical_bytes(request);
  auto response=owned_fixture::response();


  auto response_raw=canonical_bytes(response);
  RawEvidenceCollector never_started(identity());
  if (never_started.offer_management_request(request,test_source(), 10U) ||
      never_started.failure() != RawEvidenceCollector::Failure::kIo) return false;
  RawEvidenceCollector never_started_response(identity());
  if (never_started_response.offer_management_response(response,test_source(), 11U) ||
      never_started_response.failure() != RawEvidenceCollector::Failure::kIo) return false;
  char path[] = "/tmp/dummy-raw-management-closed-XXXXXX";
  const int fd = ::mkstemp(path);
  if (fd < 0) return false;
  ::close(fd);
  ::unlink(path);
  RawEvidenceCollector closed(identity());
  if (!closed.start(path)) return false;
  closed.stop();
  const bool rejected = !closed.offer_management_response(response,test_source(), 12U) &&
      closed.failure() == RawEvidenceCollector::Failure::kIo && closed.written() == 0U;
  struct stat file_stat{};
  const bool only_magic = ::stat(path, &file_stat) == 0 && file_stat.st_size == 8;
  ::unlink(path);
  char request_path[] = "/tmp/dummy-raw-management-request-closed-XXXXXX";
  const int request_fd = ::mkstemp(request_path);
  if (request_fd < 0) return false;
  ::close(request_fd);
  ::unlink(request_path);
  RawEvidenceCollector closed_request(identity());
  if (!closed_request.start(request_path)) return false;
  closed_request.stop();
  const bool request_rejected =
      !closed_request.offer_management_request(request,test_source(), 13U) &&
      closed_request.failure() == RawEvidenceCollector::Failure::kIo &&
      closed_request.written() == 0U;
  const bool request_only_magic = ::stat(request_path, &file_stat) == 0 && file_stat.st_size == 8;
  ::unlink(request_path);
  return rejected && only_magic && request_rejected && request_only_magic;
}

/**
 * @brief 采集器忠实记录不相关管理ID，不把关联过滤当作采集职责。
 *
 * request_id999/run4响应可入原始文件，读回必须保持原始字节；接收原文不是将该响应判定为当前请求成功。
 * @return true 表示本用例全部预期成立；false表示夹具准备、异步期限或结果断言失败。
 */
bool test_management_wrong_ids_are_preserved() {
  char path[] = "/tmp/dummy-raw-management-ids-XXXXXX";
  const int fd = ::mkstemp(path);
  if (fd < 0) return false;
  ::close(fd);
  ::unlink(path);
  RawEvidenceCollector collector(identity());
  if (!collector.start(path)) return false;
  auto response=owned_fixture::response();


  response.request_id()=999U;
  response.run_id()=4U;

  response.session_id()=12U;
  auto raw=canonical_bytes(response);
  const bool offered = collector.offer_management_response(response,test_source(), 12U) &&
      collector.failure() == RawEvidenceCollector::Failure::kNone;
  collector.stop();
  std::FILE *file = std::fopen(path, "rb");
  if (!file) return false;
  std::vector<std::uint8_t> recorded(8U + dummy_ros_control::kCaptureHeaderSize + raw.size());
  const bool exact = std::fread(recorded.data(), 1U, recorded.size(), file) == recorded.size() &&
      std::fgetc(file) == EOF;
  std::fclose(file);
  ::unlink(path);
  return offered && exact && collector.failure() == RawEvidenceCollector::Failure::kNone &&
      recorded[8U] == 5U && recorded[19U] == 11U && recorded[27U] == 12U &&
      recorded[35U] == 4U &&
      std::equal(raw.begin(), raw.end(), recorded.begin() + (8U+dummy_ros_control::kCaptureHeaderSize));
}

/**
 * @brief 截短管理请求在文件生命周期检查前按形状拒绝。
 *
 * 固定长度减1应kInvalid；此处collector没有start，故同时可辨认错误应来自长度而非kIo。
 * @return true 表示本用例全部预期成立；false表示夹具准备、异步期限或结果断言失败。
 */
bool test_management_length_fails_closed() {
  auto request=owned_fixture::request();


  request.request_id()=80U;
  request.request_seq()=80U;
  auto raw=canonical_bytes(request);
  RawEvidenceCollector short_record(identity());request.operation().reset();
  if (short_record.offer_management_request(request,test_source(),1U) ||
      short_record.failure() != RawEvidenceCollector::Failure::kInvalid) return false;
  return true;
}

/**
 * @brief 拒绝重复/跳跃证据序号与会话漂移。
 *
 * seq0重复和从0跳2均kSequence；新页面外层session由12改13，应kIdentity。
 * @return true 表示本用例全部预期成立；false表示夹具准备、异步期限或结果断言失败。
 */
bool test_sequence_and_identity() {
  const auto trust = identity();
  const auto group = page(3U, 0U, genesis(trust, 3U, 11U), false);
  RawEvidenceCollector duplicate(trust);
  if (!offer(duplicate,group, 100U) || offer(duplicate,group, 101U) ||
      duplicate.failure() != RawEvidenceCollector::Failure::kSequence) return false;
  RawEvidenceCollector missing(trust);
  const auto skipped = page(3U, 2U, chain_of(group), true);
  if (!offer(missing,group, 100U) || offer(missing,skipped, 101U) ||
      missing.failure() != RawEvidenceCollector::Failure::kSequence) return false;
  RawEvidenceCollector identity_check(trust);
  auto wrong = page(3U, 1U, chain_of(group), true);

  return offer(identity_check,group, 100U) && !offer(identity_check,wrong,101U,11U,13U) &&
      identity_check.failure() == RawEvidenceCollector::Failure::kIdentity;
}

/**
 * @brief 覆盖固定队列溢出和单终态页落盘格式。
 *
 * 未启动writer时填满kCapacity观测记录，额外Status必须kOverflow；独立capture记录无组终态，检查DPRAW02、总长度和raw字节。
 * @return true 表示本用例全部预期成立；false表示夹具准备、异步期限或结果断言失败。
 */
bool test_bounded_queue_and_file() {
  const auto trust = identity();
  RawEvidenceCollector full(trust);
  auto sample=owned_fixture::state();sample.session_id()=12;
  for (std::size_t i = 0; i < RawEvidenceCollector::kCapacity; ++i)
    if (!full.offer_observation(sample,test_source(),i)) return false;
  if (full.offer_observation(sample,test_source(),99U) ||
      full.failure() != RawEvidenceCollector::Failure::kOverflow) return false;
  char path[] = "/tmp/dummy-raw-evidence-XXXXXX";
  const int fd = ::mkstemp(path);
  if (fd < 0) return false;
  ::close(fd);
  ::unlink(path);
  RawEvidenceCollector file(trust);
  const auto terminal = page(3U, 0U, genesis(trust, 3U, 11U), true);
  const bool offered = file.start(path) && file.expect_run(3U) && offer(file,terminal, 100U);
  file.stop();
  std::FILE *input = std::fopen(path, "rb");
  if (!input) return false;
  std::array<std::uint8_t, 1024U> bytes{};
  const auto size = std::fread(bytes.data(), 1U, bytes.size(), input);
  std::fclose(input);
  ::unlink(path);
  const auto terminal_bytes=canonical_bytes(terminal);
  return offered && file.failure() == RawEvidenceCollector::Failure::kNone &&
      file.written() == 1U && size == 8U + dummy_ros_control::kCaptureHeaderSize + terminal_bytes.size() &&
      std::equal(bytes.begin(), bytes.begin() + 7U, "DPRAW02") &&
      bytes[8] == static_cast<std::uint8_t>(RawEvidenceCollector::Kind::kPage) &&
      std::equal(terminal_bytes.begin(), terminal_bytes.end(), bytes.begin() + (8U+dummy_ros_control::kCaptureHeaderSize));
}

/**
 * @brief 拒绝符号链接父目录和不存在的父目录。
 *
 * 先将real移到moved再以symlink替换real，start必须失败且目标文件不产生；仅覆盖这个确定的路径替换fixture，不宣称验证所有文件系统竞态。
 * @return true 表示本用例全部预期成立；false表示夹具准备、异步期限或结果断言失败。
 */
bool test_capture_path_rejects_swapped_parent() {
  char root[] = "/tmp/dummy-raw-path-XXXXXX";
  if (!::mkdtemp(root)) return false;
  const std::string real = std::string(root) + "/real";
  const std::string moved = std::string(root) + "/moved";
  const std::string output = real + "/capture.bin";
  if (::mkdir(real.c_str(), 0700) != 0 || ::rename(real.c_str(), moved.c_str()) != 0 ||
      ::symlink(moved.c_str(), real.c_str()) != 0) return false;
  RawEvidenceCollector swapped(identity());
  const bool rejected = !swapped.start(output) &&
      swapped.failure() == RawEvidenceCollector::Failure::kIo;
  const bool no_output = ::access((moved + "/capture.bin").c_str(), F_OK) != 0;
  RawEvidenceCollector missing(identity());
  const bool missing_rejected = !missing.start(std::string(root) + "/missing/capture.bin") &&
      missing.failure() == RawEvidenceCollector::Failure::kIo;
  ::unlink(real.c_str());
  ::rmdir(moved.c_str());
  ::rmdir(root);
  return rejected && no_output && missing_rejected;
}

/**
 * @brief 用reader观察链失败码首次出现后不回到零。
 *
 * reader在2s窗口内应看到kChain，且stop后错误仍保留；线程标志只是主机测试同步，不对应设备故障时钟。
 * @return true 表示本用例全部预期成立；false表示夹具准备、异步期限或结果断言失败。
 */
bool test_shared_failure_is_sticky_for_concurrent_readers() {
  dummy_ros_control::DeviceManagementChannel channel({});
  if (channel.raw_evidence_capture_enabled.load(std::memory_order_acquire)) return false;
  if (channel.raw_evidence_failure.load(std::memory_order_acquire) != 0U) return false;
  RawEvidenceCollector collector(identity(), &channel);
  std::atomic<bool> reading{true};
  std::atomic<bool> observed{false};
  std::atomic<bool> regressed{false};
  std::thread reader([&] {
    bool saw_failure = false;
    while (reading.load(std::memory_order_acquire)) {
      const auto value = channel.raw_evidence_failure.load(std::memory_order_acquire);
      if (value != 0U) { saw_failure = true; observed = true; }
      if (saw_failure && value == 0U) regressed = true;
    }
  });
  auto bad = page(3U, 0U, genesis(identity(), 3U, 11U), false);
  (*bad.chain_sha256())[0]^=1U;
  const bool rejected = !offer(collector,bad, 100U);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (!observed.load(std::memory_order_acquire) &&
         std::chrono::steady_clock::now() < deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  reading = false;
  reader.join();
  const auto code = static_cast<std::uint8_t>(RawEvidenceCollector::Failure::kChain);
  if (!rejected || !observed || regressed ||
      channel.raw_evidence_failure.load(std::memory_order_acquire) != code) return false;
  collector.stop();
  return channel.raw_evidence_failure.load(std::memory_order_acquire) == code;
}

/**
 * @brief 检查三类本地采集失败同步到管理共享通道。
 *
 * 队列满为Overflow、缺父目录为Io、expect后无终态stop为Incomplete；溢出不能发布伪terminal-flushed。
 * @return true 表示本用例全部预期成立；false表示夹具准备、异步期限或结果断言失败。
 */
bool test_shared_failure_reports_overflow_io_and_incomplete() {
  dummy_ros_control::DeviceManagementChannel overflow_channel({});
  RawEvidenceCollector overflow(identity(), &overflow_channel);
  auto raw=owned_fixture::state();raw.session_id()=12;
  for (std::size_t i = 0U; i < RawEvidenceCollector::kCapacity; ++i)
    if (!overflow.offer_observation(raw,test_source(),i)) return false;
  if (overflow.offer_observation(raw,test_source(),999U) ||
      overflow_channel.raw_evidence_failure.load(std::memory_order_acquire) !=
          static_cast<std::uint8_t>(RawEvidenceCollector::Failure::kOverflow) ||
      overflow_channel.raw_evidence_terminal_flushed().run_id != 0U) return false;
  dummy_ros_control::DeviceManagementChannel io_channel({});
  RawEvidenceCollector io(identity(), &io_channel);
  if (io.start("/tmp/dummy-raw-missing-parent/forbidden.bin") ||
      io_channel.raw_evidence_failure.load(std::memory_order_acquire) !=
          static_cast<std::uint8_t>(RawEvidenceCollector::Failure::kIo)) return false;
  dummy_ros_control::DeviceManagementChannel incomplete_channel({});
  RawEvidenceCollector incomplete(identity(), &incomplete_channel);
  if (!incomplete.expect_run(3U)) return false;
  incomplete.stop();
  return incomplete_channel.raw_evidence_failure.load(std::memory_order_acquire) ==
      static_cast<std::uint8_t>(RawEvidenceCollector::Failure::kIncomplete);
}

/**
 * @brief 暂停writer终态flush阶段的测试门闩。
 * entered由writer release-store发布，release由主线程放行；acquire读取使等待者看见对应阶段。
 * 这是一种人为阻塞钩子，只安排可重复时序，不是生产实时路径。
 */
struct FlushGate {
  /**
   * @brief writer已到达flush前钩子；main用期限等待它，避免未进入即开始溢出/回执检查。
   */
  std::atomic<bool> entered{false};
  /**
   * @brief 允许writer继续；所有正常/超时收尾都应放行后再stop，避免join卡在钩子。
   */
  std::atomic<bool> release{false};
};

/**
 * @brief 在writer线程等待主线程放开终态flush测试阶段。
 * @param context 指向生命周期覆盖collector.stop的FlushGate。
 * @return true 表示放行后继续正常flush；不在本钩子注入I/O失败。
 * @note 每次等待sleep 1ms，属于测试同步，不应解释成实际设备flush耗时。
 */
bool wait_before_terminal_flush(void *context) noexcept {
  auto &gate = *static_cast<FlushGate *>(context);
  gate.entered.store(true, std::memory_order_release);
  while (!gate.release.load(std::memory_order_acquire))
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  return true;
}

/**
 * @brief 暂停终态flush，使正在运行的writer仍能确定触发队列满。
 *
 * 先等writer进入门闩，再塞入管理请求；额外记录应Overflow，放行并stop后仍保持首个溢出失败。
 * @return true 表示本用例全部预期成立；false表示夹具准备、异步期限或结果断言失败。
 */
bool test_management_overflow_with_running_writer() {
  char path[] = "/tmp/dummy-raw-management-overflow-XXXXXX";
  const int fd = ::mkstemp(path);
  if (fd < 0) return false;
  ::close(fd);
  ::unlink(path);
  FlushGate gate;
  const auto trust = identity();
  RawEvidenceCollector collector(trust, nullptr, wait_before_terminal_flush, &gate);
  const auto terminal = page(3U, 0U, genesis(trust, 3U, 11U), true);
  if (!collector.start(path) || !collector.expect_run(3U) ||
      !offer(collector,terminal, 1U)) return false;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (!gate.entered.load(std::memory_order_acquire) &&
         std::chrono::steady_clock::now() < deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  if (!gate.entered.load(std::memory_order_acquire)) {
    gate.release = true;
    collector.stop();
    ::unlink(path);
    return false;
  }
  auto request=owned_fixture::request();


  auto raw=canonical_bytes(request);
  bool accepted = true;
  for (std::size_t i = 1U; accepted && i < RawEvidenceCollector::kCapacity; ++i)
    accepted = collector.offer_management_request(request,test_source(), i + 1U);
  const bool overflow = !collector.offer_management_request(request,test_source(), 999U) &&
      collector.failure() == RawEvidenceCollector::Failure::kOverflow;
  gate.release = true;
  collector.stop();
  ::unlink(path);
  return accepted && overflow && collector.failure() == RawEvidenceCollector::Failure::kOverflow;
}

/**
 * @brief 终态回执只能在writer完成受控flush阶段后出现。
 *
 * writer在门闩中暂停时terminal_written_run_id和共享run均为0；reader不能看到提前或拼接身份，放行后应见boot11/session12/run3。
 * @return true 表示本用例全部预期成立；false表示夹具准备、异步期限或结果断言失败。
 */
bool test_terminal_ack_waits_for_flush() {
  char path[] = "/tmp/dummy-raw-flush-XXXXXX";
  const int fd = ::mkstemp(path);
  if (fd < 0) return false;
  ::close(fd);
  ::unlink(path);
  dummy_ros_control::TransportParameters config{};
  config.raw_evidence_path = path;
  dummy_ros_control::DeviceManagementChannel channel(config);
  FlushGate gate;
  RawEvidenceCollector collector(identity(), &channel, wait_before_terminal_flush, &gate);
  if (!channel.raw_evidence_capture_enabled.load(std::memory_order_acquire) ||
      !collector.start(path) || !collector.expect_run(3U)) return false;
  const auto group = page(3U, 0U, genesis(identity(), 3U, 11U), false);
  const auto terminal = page(3U, 1U, chain_of(group), true);
  if (!offer(collector,group, 100U) || !offer(collector,terminal, 101U)) return false;
  std::atomic<bool> reading{true};
  std::atomic<bool> premature{false};
  std::atomic<bool> observed{false};
  std::thread reader([&] {
    while (reading.load(std::memory_order_acquire)) {
      const auto current = channel.raw_evidence_terminal_flushed();
      if (current.run_id != 0U) {
        if (!gate.release.load(std::memory_order_acquire) || current.boot_epoch != 11U ||
            current.session_id != 12U || current.run_id != 3U) premature = true;
        observed = true;
      }
    }
  });
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (!gate.entered.load(std::memory_order_acquire) &&
         std::chrono::steady_clock::now() < deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  const auto before = channel.raw_evidence_terminal_flushed();
  const bool withheld = gate.entered.load(std::memory_order_acquire) &&
      before.run_id == 0U && collector.terminal_written_run_id() == 0U &&
      channel.raw_evidence_failure.load(std::memory_order_acquire) == 0U;
  gate.release.store(true, std::memory_order_release);
  while (!observed.load(std::memory_order_acquire) &&
         std::chrono::steady_clock::now() < deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  reading.store(false, std::memory_order_release);
  reader.join();
  collector.stop();
  ::unlink(path);
  const auto after = channel.raw_evidence_terminal_flushed();
  return withheld && observed && !premature && after.boot_epoch == 11U && after.session_id == 12U &&
      after.run_id == 3U && collector.failure() == RawEvidenceCollector::Failure::kNone;
}

/**
 * @brief 允许当前run等待flush时先接收后继run页面。
 *
 * run3终态在门闩中时排入run4组/终态；共享回执仍为0。放行并stop应无失败，最终回执run4，避免writer进度阻塞合法序列验证。
 * @return true 表示本用例全部预期成立；false表示夹具准备、异步期限或结果断言失败。
 */
bool test_successor_queued_before_terminal_flush() {
  char path[] = "/tmp/dummy-raw-successor-pending-XXXXXX";
  const int fd = ::mkstemp(path);
  if (fd < 0) return false;
  ::close(fd);
  ::unlink(path);
  dummy_ros_control::DeviceManagementChannel channel({});
  FlushGate gate;
  const auto trust = identity();
  RawEvidenceCollector collector(trust, &channel, wait_before_terminal_flush, &gate);
  auto first = page(3U, 0U, genesis(trust, 3U, 11U), false);
  auto terminal = page(3U, 1U, chain_of(first), true);
  if (!collector.start(path) || !collector.expect_run(3U) ||
      !offer(collector,first, 100U) || !offer(collector,terminal, 101U)) return false;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (!gate.entered.load(std::memory_order_acquire) &&
         std::chrono::steady_clock::now() < deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  auto next = page(4U, 0U, genesis(trust, 4U, 11U), false);
  auto next_terminal = page(4U, 1U, chain_of(next), true);
  const bool queued = gate.entered.load(std::memory_order_acquire) &&
      collector.expect_run(4U) && offer(collector,next, 102U) &&
      offer(collector,next_terminal, 103U) &&
      channel.raw_evidence_terminal_flushed().run_id == 0U;
  gate.release.store(true, std::memory_order_release);
  collector.stop();
  ::unlink(path);
  return queued && collector.failure() == RawEvidenceCollector::Failure::kNone &&
      channel.raw_evidence_terminal_flushed().run_id == 4U;
}

/**
 * @brief 在子进程文件大小受限时，终态写失败不能发布flush回执。
 *
 * 由中央编码实际长度计算的上限允许magic和完整首组页，但容不下完整终态记录；等待首组页写成后提供终态，应kIo且共享run仍0。fork隔离RLIMIT_FSIZE/SIGXFSZ，不影响父进程。
 * @return true 表示本用例全部预期成立；false表示夹具准备、异步期限或结果断言失败。
 */
bool test_terminal_write_error_never_acknowledges() {
  char path[] = "/tmp/dummy-raw-write-error-XXXXXX";
  const int fd = ::mkstemp(path);
  if (fd < 0) return false;
  ::close(fd);
  ::unlink(path);
  const pid_t child = ::fork();
  if (child < 0) return false;
  if (child == 0) {
    ::signal(SIGXFSZ, SIG_IGN);
    const auto complete_first=page(3U,0U,genesis(identity(),3U,11U),false);
    const rlim_t limit_bytes=8U+dummy_ros_control::kCaptureHeaderSize+canonical_bytes(complete_first).size()+10U;
    const rlimit limit{limit_bytes,limit_bytes};
    if (::setrlimit(RLIMIT_FSIZE, &limit) != 0) ::_exit(2);
    dummy_ros_control::TransportParameters config{};
    config.raw_evidence_path = path;
    dummy_ros_control::DeviceManagementChannel channel(config);
    RawEvidenceCollector collector(identity(), &channel);
    const auto group = page(3U, 0U, genesis(identity(), 3U, 11U), false);
    const auto terminal = page(3U, 1U, chain_of(group), true);
    if (!collector.start(path) || !collector.expect_run(3U) ||
        !offer(collector,group, 100U)) ::_exit(3);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (collector.pages_written() != 1U && std::chrono::steady_clock::now() < deadline)
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    if (!offer(collector,terminal, 101U)) ::_exit(4);
    while (channel.raw_evidence_failure.load(std::memory_order_acquire) == 0U &&
           std::chrono::steady_clock::now() < deadline)
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    collector.stop();
    const bool pass = channel.raw_evidence_failure.load(std::memory_order_acquire) ==
            static_cast<std::uint8_t>(RawEvidenceCollector::Failure::kIo) &&
        channel.raw_evidence_terminal_flushed().run_id == 0U;
    ::_exit(pass ? 0 : 5);
  }
  int status = 0;
  const bool waited = ::waitpid(child, &status, 0) == child;
  ::unlink(path);
  return waited && WIFEXITED(status) && WEXITSTATUS(status) == 0;
}

/**
 * @brief 压力读取terminal-flushed三字段快照不能跨发布拼接。
 *
 * writer依次写50,000个boot，并使session=boot+100/run=boot+200；reader验证关系，join后核对最后值。此调度样本不代替形式化内存模型证明。
 * @return true 表示本用例全部预期成立；false表示夹具准备、异步期限或结果断言失败。
 */
bool test_terminal_snapshot_is_coherent() {
  dummy_ros_control::DeviceManagementChannel channel({});
  std::atomic<bool> done{false};
  std::thread writer([&] {
    for (std::uint64_t boot = 1U; boot <= 50000U; ++boot)
      channel.publish_raw_evidence_terminal_flushed(boot, boot + 100U, boot + 200U);
    done.store(true, std::memory_order_release);
  });
  bool coherent = true;
  while (!done.load(std::memory_order_acquire)) {
    const auto snapshot = channel.raw_evidence_terminal_flushed();
    if (snapshot.run_id != 0U &&
        (snapshot.session_id != snapshot.boot_epoch + 100U ||
         snapshot.run_id != snapshot.boot_epoch + 200U)) coherent = false;
  }
  writer.join();
  const auto final = channel.raw_evidence_terminal_flushed();
  return coherent && final.boot_epoch == 50000U && final.session_id == 50100U &&
      final.run_id == 50200U;
}

/**
 * @brief 首组页完整写入并flush后发布包含轴掩码的首写回执。
 *
 * 回执必须携带run3/motion10/seq0、全六轴且无失败、chain_valid/final；随后补终态避免stop报Incomplete。首写回执本身不是terminal回执。
 * @return true 表示本用例全部预期成立；false表示夹具准备、异步期限或结果断言失败。
 */
bool test_first_page_receipt_after_write() {
  char path[] = "/tmp/dummy-raw-first-XXXXXX";
  const int fd = ::mkstemp(path);
  if (fd < 0) return false;
  ::close(fd);
  ::unlink(path);
  dummy_ros_control::TransportParameters config{};
  config.raw_evidence_path = path;
  dummy_ros_control::DeviceManagementChannel channel(config);
  const auto trust = identity();
  RawEvidenceCollector collector(trust, &channel);
  const auto first = page(3U, 0U, genesis(trust, 3U, 11U), false);
  if (!collector.start(path) || !collector.expect_run(3U) ||
      channel.raw_evidence_first_page_written().run_id != 0U ||
      !offer(collector,first, 100U)) return false;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (channel.raw_evidence_first_page_written().run_id == 0U &&
         std::chrono::steady_clock::now() < deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  const auto receipt = channel.raw_evidence_first_page_written();
  const bool valid = receipt.boot_epoch == 11U && receipt.session_id == 12U &&
      receipt.run_id == 3U && receipt.motion_epoch == 10U && receipt.evidence_seq == 0U &&
      receipt.attempt_kind == 1U && receipt.category == 2U &&
      receipt.expected_mask == 63U && receipt.submitted_mask == 63U &&
      receipt.sent_mask == 63U && receipt.failed_mask == 0U &&
      receipt.group_count == 1U && receipt.final && !receipt.evidence_lost &&
      receipt.chain_valid && receipt.full_six && !receipt.strict_refresh_successor;
  const auto terminal = page(3U, 1U, chain_of(first), true);
  const bool ended = offer(collector,terminal, 101U);
  collector.stop();
  ::unlink(path);
  return valid && ended && collector.failure() == RawEvidenceCollector::Failure::kNone;
}

/**
 * @brief 首页面链锚错误时绝不发布首写回执。
 *
 * 只翻转外层previous摘要，offer应失败，channel first-page-written run仍0。
 * @return true 表示本用例全部预期成立；false表示夹具准备、异步期限或结果断言失败。
 */
bool test_first_page_receipt_rejects_bad_chain() {
  char path[] = "/tmp/dummy-raw-first-chain-XXXXXX";
  const int fd = ::mkstemp(path);
  if (fd < 0) return false;
  ::close(fd);
  ::unlink(path);
  dummy_ros_control::DeviceManagementChannel channel({});
  RawEvidenceCollector collector(identity(), &channel);
  auto first = page(3U, 0U, genesis(identity(), 3U, 11U), false);
  (*first.previous_chain_sha256())[0] ^= 1U;
  const bool rejected = collector.start(path) && collector.expect_run(3U) &&
      !offer(collector,first, 100U) &&
      channel.raw_evidence_first_page_written().run_id == 0U;
  collector.stop();
  ::unlink(path);
  return rejected;
}

/**
 * @brief 首记录只写了一部分时不能确认首写成功。
 *
 * 子进程文件限制100字节，小于magic+完整首记录；等待Io失败后stop，首写共享run必须仍0。
 * @return true 表示本用例全部预期成立；false表示夹具准备、异步期限或结果断言失败。
 */
bool test_first_page_partial_write_never_publishes() {
  char path[] = "/tmp/dummy-raw-first-partial-XXXXXX";
  const int fd = ::mkstemp(path);
  if (fd < 0) return false;
  ::close(fd);
  ::unlink(path);
  const pid_t child = ::fork();
  if (child < 0) return false;
  if (child == 0) {
    ::signal(SIGXFSZ, SIG_IGN);
    const rlimit limit{100U, 100U};
    if (::setrlimit(RLIMIT_FSIZE, &limit) != 0) ::_exit(2);
    dummy_ros_control::DeviceManagementChannel channel({});
    RawEvidenceCollector collector(identity(), &channel);
    const auto first = page(3U, 0U, genesis(identity(), 3U, 11U), false);
    if (!collector.start(path) || !collector.expect_run(3U) ||
        !offer(collector,first, 100U)) ::_exit(3);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (collector.failure() == RawEvidenceCollector::Failure::kNone &&
           std::chrono::steady_clock::now() < deadline)
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    collector.stop();
    ::_exit(collector.failure() == RawEvidenceCollector::Failure::kIo &&
        channel.raw_evidence_first_page_written().run_id == 0U ? 0 : 4);
  }
  int status = 0;
  const bool waited = ::waitpid(child, &status, 0) == child;
  ::unlink(path);
  return waited && WIFEXITED(status) && WEXITSTATUS(status) == 0;
}

/**
 * @brief 首页flush故障注入计数；calls由writer更新，主线程原子读取。
 */
struct FirstPageFlushProbe {
  std::atomic<unsigned> calls{0U};
};

/**
 * @brief 替代首记录flush并确定返回失败。
 * @param context 指向FirstPageFlushProbe，记录被调用次数。
 * @par 未命名参数
 * FILE*是writer提供的输出流；钩子故意不操作它。
 * @return EOF，模拟flush失败，collector应发布kIo而不是首写回执。
 */
int fail_first_page_flush(std::FILE *, void *context) noexcept {
  static_cast<FirstPageFlushProbe *>(context)->calls.fetch_add(1U, std::memory_order_relaxed);
  return EOF;
}

/**
 * @brief 注入首记录flush返回EOF时不发布首写回执。
 *
 * 即使offer已接收，writer的flush钩子一次失败也应kIo，run仍0；不把排队/写部分字节当作采集完成。
 * @return true 表示本用例全部预期成立；false表示夹具准备、异步期限或结果断言失败。
 */
bool test_first_page_flush_failure_never_publishes() {
  char path[] = "/tmp/dummy-raw-first-flush-XXXXXX";
  const int fd = ::mkstemp(path);
  if (fd < 0) return false;
  ::close(fd);
  ::unlink(path);
  dummy_ros_control::DeviceManagementChannel channel({});
  FirstPageFlushProbe probe;
  RawEvidenceCollector collector(identity(), &channel, nullptr, &probe,
                                 fail_first_page_flush);
  const auto first = page(3U, 0U, genesis(identity(), 3U, 11U), false);
  if (!collector.start(path) || !collector.expect_run(3U) ||
      !offer(collector,first, 100U)) return false;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (collector.failure() == RawEvidenceCollector::Failure::kNone &&
         std::chrono::steady_clock::now() < deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  const bool rejected = probe.calls.load(std::memory_order_relaxed) == 1U &&
      collector.failure() == RawEvidenceCollector::Failure::kIo &&
      channel.raw_evidence_first_page_written().run_id == 0U;
  collector.stop();
  ::unlink(path);
  return rejected;
}

/**
 * @brief 首写回执按run更新，full_six要求发送完整且无失败。
 *
 * run3仅mask3，run4六轴但sent62/failed1，run5六轴全发送；只有run5 full_six=true，三个run均等待各自终态flush。
 * @return true 表示本用例全部预期成立；false表示夹具准备、异步期限或结果断言失败。
 */
bool test_first_page_mask_and_next_run() {
  char path[] = "/tmp/dummy-raw-first-next-XXXXXX";
  const int fd = ::mkstemp(path);
  if (fd < 0) return false;
  ::close(fd);
  ::unlink(path);
  dummy_ros_control::DeviceManagementChannel channel({});
  const auto trust = identity();
  RawEvidenceCollector collector(trust, &channel);
  if (!collector.start(path)) return false;
  for (std::uint64_t run = 3U; run <= 5U; ++run) {
    const auto first = page(run, 0U, genesis(trust, run, 11U), false,
                            1U, true, 1U, 0U, run == 3U ? 3U : 63U,
                            run == 4U ? 62U : UINT8_MAX, run == 4U ? 1U : 0U);
    const auto terminal = page(run, 1U, chain_of(first), true);
    if (!collector.expect_run(run) || !offer(collector,first, run) ||
        !offer(collector,terminal, run + 1U)) return false;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (channel.raw_evidence_first_page_written().run_id != run &&
           std::chrono::steady_clock::now() < deadline)
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    const auto receipt = channel.raw_evidence_first_page_written();
    if (receipt.run_id != run || receipt.expected_mask != (run == 3U ? 3U : 63U) ||
        receipt.submitted_mask != receipt.expected_mask ||
        receipt.sent_mask != (run == 4U ? 62U : receipt.expected_mask) ||
        receipt.failed_mask != (run == 4U ? 1U : 0U) ||
        receipt.full_six != (run == 5U)) return false;
    while (channel.raw_evidence_terminal_flushed().run_id != run &&
           std::chrono::steady_clock::now() < deadline)
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    if (channel.raw_evidence_terminal_flushed().run_id != run) return false;
  }
  collector.stop();
  ::unlink(path);
  return collector.failure() == RawEvidenceCollector::Failure::kNone;
}

/**
 * @brief 覆盖strict_refresh_successor派生位的组号条件。
 *
 * 把首组页category改1并重算hash；group_id非零时run3/4应true，run5清零组号后应false，不将组数预报2误判为非后继。
 * @return true 表示本用例全部预期成立；false表示夹具准备、异步期限或结果断言失败。
 */
bool test_strict_refresh_successor_receipt() {
  const auto trust = identity();
  char path[] = "/tmp/dummy-raw-strict-first-XXXXXX";
  const int fd = ::mkstemp(path);
  if (fd < 0) return false;
  ::close(fd);
  ::unlink(path);
  dummy_ros_control::DeviceManagementChannel channel({});
  RawEvidenceCollector collector(trust, &channel);
  if (!collector.start(path)) return false;
  for (std::uint64_t run = 3U; run <= 5U; ++run) {
    auto first = page(run, 0U, genesis(trust, run, 11U), false,
                      run == 4U ? 2U : 1U);
    auto decoded=owned_fixture::evidence();
    decoded=first;
    decoded.category()=1U;
    if (run == 5U) decoded.group_id()=0U;
    first.category() = 1U;
    rewrite_page(first, decoded);
    const auto terminal = page(run, 1U, chain_of(first), true);
    if (!collector.expect_run(run) || !offer(collector,first, run) ||
        !offer(collector,terminal, run + 1U)) return false;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (channel.raw_evidence_first_page_written().run_id != run &&
           std::chrono::steady_clock::now() < deadline)
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    const auto receipt = channel.raw_evidence_first_page_written();
    if (receipt.run_id != run || receipt.strict_refresh_successor != (run != 5U)) return false;
    while (channel.raw_evidence_terminal_flushed().run_id != run &&
           std::chrono::steady_clock::now() < deadline)
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    if (channel.raw_evidence_terminal_flushed().run_id != run) return false;
  }
  collector.stop();
  ::unlink(path);
  return collector.failure() == RawEvidenceCollector::Failure::kNone;
}

/**
 * @brief 从完整链提取干净取消终态与安全hold证明字段。
 *
 * run3三个组motion_epoch递进5/6/7，取消终态回到原motion5；回执应safety_hold_valid、hold_epoch7，但motion_complete/release_committed均false。后续run4缺后继组、run5发送不全、run6运动代次回退均不得产生有效安全hold。
 * @return true 表示本用例全部预期成立；false表示夹具准备、异步期限或结果断言失败。
 */
bool test_flushed_clean_cancel_receipt() {
  const auto trust = identity();
  char path[] = "/tmp/dummy-raw-cancel-terminal-XXXXXX";
  const int fd = ::mkstemp(path);
  if (fd < 0) return false;
  ::close(fd);
  ::unlink(path);
  dummy_ros_control::DeviceManagementChannel channel({});
  RawEvidenceCollector collector(trust, &channel);
  if (!collector.start(path)) return false;
  auto first = page(3U, 0U, genesis(trust, 3U, 11U), false);
  auto decoded=owned_fixture::evidence();
  decoded=first;
  decoded.motion_epoch()=5U;
  rewrite_page(first, decoded);
  auto motion = page(3U, 1U, chain_of(first), false, 2U);
  decoded=motion;
  decoded.motion_epoch()=6U;
  rewrite_page(motion, decoded);
  auto safety = page(3U, 2U, chain_of(motion), false, 3U);
  decoded=safety;
  decoded.motion_epoch()=7U;
  rewrite_page(safety, decoded);
  auto terminal = page(3U, 3U, chain_of(safety), true, 3U);
  decoded=terminal;
  decoded.final_state()=7U;
  decoded.motion_epoch()=5U;
  terminal.final_state() = 7U;
  rewrite_page(terminal, decoded);
  if (!collector.expect_run(3U) || !offer(collector,first, 100U) ||
      !offer(collector,motion, 101U) ||
      !offer(collector,safety, 102U) ||
      !offer(collector,terminal, 103U)) return false;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (channel.raw_evidence_terminal_flushed().run_id != 3U &&
         std::chrono::steady_clock::now() < deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  /**
   * @brief 保留run3干净取消回执再执行负例，避免用后续run覆盖的共享快照冒充run3证明。
   */
  const auto receipt = channel.raw_evidence_terminal_flushed();
  for (std::uint64_t run = 4U; run <= 6U; ++run) {
    auto next_first = page(run, 0U, genesis(trust, run, 11U), false);
    if (run == 6U) {
      auto first_decoded=owned_fixture::evidence();
      first_decoded=next_first;
      first_decoded.motion_epoch()=1U;
      rewrite_page(next_first, first_decoded);
    }
    auto previous = chain_of(next_first);
    std::uint32_t terminal_seq = 1U, group_count = 1U;
    if (run >= 5U) {
      auto wrong_epoch = page(run, 1U, previous, false, 2U);
      auto wrong_decoded=owned_fixture::evidence();
      wrong_decoded=wrong_epoch;
      wrong_decoded.motion_epoch()=run == 5U ? 2U : 3U;
      if (run == 5U) wrong_decoded.sent_mask()=31U;
      rewrite_page(wrong_epoch, wrong_decoded);
      if (!collector.expect_run(run) || !offer(collector,next_first, 200U) ||
          !offer(collector,wrong_epoch, 201U)) return false;
      previous = chain_of(wrong_epoch);
      terminal_seq = 2U; group_count = 2U;
      if (run == 6U) {
        auto stale = page(run, 2U, previous, false, 3U);
        wrong_decoded=stale;
        wrong_decoded.motion_epoch()=2U;
        rewrite_page(stale, wrong_decoded);
        if (!offer(collector,stale, 202U)) return false;
        previous = chain_of(stale);
        terminal_seq = 3U; group_count = 3U;
      }
    } else if (!collector.expect_run(run) ||
               !offer(collector,next_first, 200U)) return false;
    auto next_terminal = page(run, terminal_seq, previous, true, group_count);
    if (run == 5U) {
      auto terminal_decoded=owned_fixture::evidence();
      terminal_decoded=next_terminal;
      terminal_decoded.motion_epoch()=2U;
      rewrite_page(next_terminal, terminal_decoded);
    }
    if (!offer(collector,next_terminal, 202U)) return false;
    while (channel.raw_evidence_terminal_flushed().run_id != run &&
           std::chrono::steady_clock::now() < deadline)
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    if (channel.raw_evidence_terminal_flushed().run_id != run ||
        channel.raw_evidence_terminal_flushed().safety_hold_valid) return false;
  }
  collector.stop();
  ::unlink(path);
  return collector.failure() == RawEvidenceCollector::Failure::kNone &&
      receipt.run_id == 3U && receipt.final_state == 7U &&
      receipt.attempt_kind == 1U && receipt.parent_run_id == 0U &&
      receipt.first_failure_source == 0U && receipt.first_failure_axis_mask == 0U &&
      !receipt.evidence_lost && !receipt.motion_complete &&
      !receipt.release_committed && receipt.chain_valid &&
      receipt.safety_hold_valid && receipt.safety_hold_motion_epoch == 7U;
}

/**
 * @brief 同一capture内boot切换必须失败关闭。
 *
 * run3完整后构造以boot12为genesis的新run4；即使run不同，也不能在一个capture混入新启动身份，预期kIdentity。
 * @return true 表示本用例全部预期成立；false表示夹具准备、异步期限或结果断言失败。
 */
bool test_same_capture_reboot_fails_closed() {
  const auto trust = identity();
  RawEvidenceCollector collector(trust);
  const auto first = page(3U, 0U, genesis(trust, 3U, 11U), false);
  const auto terminal = page(3U, 1U, chain_of(first), true);
  if (!offer(collector,first, 1U) || !offer(collector,terminal, 2U)) return false;
  auto rebooted = page(4U, 0U, genesis(trust, 4U, 12U), false);

  return !offer(collector,rebooted,3U,12U) &&
      collector.failure() == RawEvidenceCollector::Failure::kIdentity;
}

/**
 * @brief 首写页必须从seq0开始，且共享回执具备一致性和旧值保护。
 *
 * 首seq1拒kSequence、UINT32_MAX拒kInvalid；writer发布20,000组相关字段，reader检查不拼接，最后发布run1旧值也不能覆盖较新run20200。
 * @return true 表示本用例全部预期成立；false表示夹具准备、异步期限或结果断言失败。
 */
bool test_first_page_sequence_and_snapshot() {
  dummy_ros_control::DeviceManagementChannel channel({});
  const auto trust = identity();
  RawEvidenceCollector missing(trust, &channel);
  const auto seq_one = page(3U, 1U, genesis(trust, 3U, 11U), false);
  if (offer(missing,seq_one, 1U) ||
      missing.failure() != RawEvidenceCollector::Failure::kSequence ||
      channel.raw_evidence_first_page_written().run_id != 0U) return false;
  RawEvidenceCollector wrapped(trust, &channel);
  const auto max_seq = page(3U, UINT32_MAX, genesis(trust, 3U, 11U), false);
  if (offer(wrapped,max_seq, 1U) ||
      wrapped.failure() != RawEvidenceCollector::Failure::kInvalid ||
      channel.raw_evidence_first_page_written().run_id != 0U) return false;
  std::atomic<bool> done{false};
  std::thread writer([&] {
    for (std::uint64_t run = 1U; run <= 20000U; ++run) {
      dummy_ros_control::RawEvidenceFirstPageWritten receipt{};
      receipt.boot_epoch = run;
      receipt.session_id = run + 100U;
      receipt.run_id = run + 200U;
      receipt.motion_epoch = static_cast<std::uint32_t>(run);
      receipt.expected_mask = static_cast<std::uint8_t>(run % 64U);
      channel.publish_raw_evidence_first_page_written(receipt);
    }
    done.store(true, std::memory_order_release);
  });
  bool coherent = true;
  while (!done.load(std::memory_order_acquire)) {
    const auto receipt = channel.raw_evidence_first_page_written();
    if (receipt.run_id != 0U &&
        (receipt.session_id != receipt.boot_epoch + 100U ||
         receipt.run_id != receipt.boot_epoch + 200U ||
         receipt.motion_epoch != receipt.boot_epoch ||
         receipt.expected_mask != receipt.boot_epoch % 64U)) coherent = false;
  }
  writer.join();
  dummy_ros_control::RawEvidenceFirstPageWritten stale{};
  stale.run_id = 1U;
  channel.publish_raw_evidence_first_page_written(stale);
  return coherent && channel.raw_evidence_first_page_written().run_id == 20200U;
}

/**
 * @brief 循环run回收有限binding槽，拒绝已经退休的迟到旧run。
 *
 * run3..104共102个run、204页，末两run使用不同attempt_kind及父run。每轮等待写页与终态flush后再复用槽；旧run3不能再find/observe，另一capture超过槽窗口后offer旧页应kIdentity。
 * @return true 表示本用例全部预期成立；false表示夹具准备、异步期限或结果断言失败。
 */
bool test_many_runs_and_recycled_late_page() {
  const auto trust = identity();
  char path[] = "/tmp/dummy-raw-many-XXXXXX";
  const int fd = ::mkstemp(path);
  if (fd < 0) return false;
  ::close(fd);
  ::unlink(path);
  dummy_ros_control::TransportParameters config{};
  config.raw_evidence_path = path;
  dummy_ros_control::DeviceManagementChannel channel(config);
  RawEvidenceCollector collector(trust, &channel);
  RawEvidenceBindings bindings;
  if (!collector.start(path)) return false;
  /**
   * @par 内部wait_written回调
   * count是期望累计页数，run是期望最新终态flush标识；在steady_clock的3s窗口内等待两者同时满足。
   * 返回时要求页数恰好相等，防止只检查终态ID而漏掉重复落盘；不等待真实设备运动。
   */
  auto wait_written = [&](std::uint64_t count, std::uint64_t run) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while ((collector.pages_written() < count ||
            channel.raw_evidence_terminal_flushed().run_id != run) &&
           std::chrono::steady_clock::now() < deadline)
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    return collector.pages_written() == count &&
           channel.raw_evidence_terminal_flushed().run_id == run;
  };
  for (std::uint64_t run = 3U; run <= 104U; ++run) {
    const std::uint8_t kind = run == 103U ? 2U : run == 104U ? 3U : 1U;
    const std::uint64_t parent = kind == 1U ? 0U : run - 1U;
    /**
     * @brief binding把run与boot/session/parent/kind关联。
     * 回收水位使用terminal_written_run_id，只有已完成文件终态flush的run才能让有限槽位安全复用。
     */
    const RawEvidenceBinding binding{run, 11U, 12U, parent, kind};
    if (!bindings.observe(binding, collector.terminal_written_run_id()) ||
        !bindings.find(run)) return false;
    const auto group = page(run, 0U, genesis(trust, run, 11U), false,
                            1U, true, kind, parent);
    const auto terminal = page(run, 1U, chain_of(group), true,
                               1U, true, kind, parent);
    if (!collector.expect_run(run) || !offer(collector,group, run * 10U) ||
        !offer(collector,terminal, run * 10U + 1U) ||
        !wait_written((run - 2U) * 2U, run)) return false;
    const auto flushed = channel.raw_evidence_terminal_flushed();
    if (flushed.boot_epoch != 11U || flushed.session_id != 12U ||
        flushed.run_id != run) return false;
  }
  collector.stop();
  ::unlink(path);
  if (collector.failure() != RawEvidenceCollector::Failure::kNone ||
      collector.pages_written() != 204U) return false;
  if (bindings.find(3U) ||
      bindings.observe({3U, 11U, 12U, 0U, 1U}, collector.terminal_written_run_id()))
    return false;

  char late_path[] = "/tmp/dummy-raw-late-XXXXXX";
  const int late_fd = ::mkstemp(late_path);
  if (late_fd < 0) return false;
  ::close(late_fd);
  ::unlink(late_path);
  RawEvidenceCollector late(trust);
  if (!late.start(late_path)) return false;
  for (std::uint64_t run = 3U; run <= 11U; ++run) {
    const auto group = page(run, 0U, genesis(trust, run, 11U), false);
    const auto terminal = page(run, 1U, chain_of(group), true);
    if (!late.expect_run(run) || !offer(late,group, run) ||
        !offer(late,terminal, run + 1U)) return false;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (late.pages_written() < (run - 2U) * 2U &&
           std::chrono::steady_clock::now() < deadline)
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  const auto old = page(3U, 0U, genesis(trust, 3U, 11U), false);
  const bool rejected = !offer(late,old, 999U) &&
      late.failure() == RawEvidenceCollector::Failure::kIdentity;
  late.stop();
  ::unlink(late_path);
  return rejected;
}
// 按GUI41原件重现：普通Stop可先于Fault，末Disable与完整flush仍可仅证明原owner已失能。
bool test_autonomous_owner_disable_receipt() {
 for(unsigned mode=0;mode<9;++mode) {
  char path[]="/tmp/dummy-owner-disable-XXXXXX";const int fd=::mkstemp(path);
  if(fd<0)return false;::close(fd);::unlink(path);
  const auto trust=identity();dummy_ros_control::DeviceManagementChannel channel({});
  FlushGate gate;RawEvidenceCollector collector(trust,&channel,wait_before_terminal_flush,&gate);
  if(!collector.start(path))return false;
  auto previous=genesis(trust,9,11);
  for(unsigned seq=0;seq<8;++seq) {
   auto item=page(9,seq,previous,false,seq+1);auto decoded=owned_fixture::evidence();
   decoded=item;
   item.category()=seq<4?1U:seq<7?2U:3U;decoded.category()=*item.category();decoded.group_id()=100+seq;
   decoded.motion_epoch()=seq<2?18U:seq<4?19U:seq==4?20U:21U;
   if(seq==7) {decoded.first_failure_source()=0x100a;decoded.first_failure_axis_mask()=63;}
   std::uint64_t q[6],take[6],sub[6],sent[6],abort[6]{};
   for(unsigned axis=0;axis<6;++axis) {q[axis]=1000+seq*200;take[axis]=q[axis]+1;sub[axis]=q[axis]+2;sent[axis]=q[axis]+3;}
   if(seq==7) {
    if(mode==1)decoded.sent_mask()=31;
    if(mode==2)decoded.failed_mask()=1;
    if(mode==3)q[2]=0;
    if(mode==4)sent[2]=q[2]+100001;
    if(mode==5)abort[2]=q[2]+3;
    if(mode==6)decoded.first_failure_source()=0x1008;
    if(mode==8)decoded.first_failure_axis_mask()=2;
   }
   decoded.queued_at_us().emplace().assign(q,6U);decoded.taken_at_us().emplace().assign(take,6U);decoded.submitted_at_us().emplace().assign(sub,6U);decoded.sent_at_us().emplace().assign(sent,6U);decoded.abort_at_us().emplace().assign(abort,6U);
   rewrite_page(item,decoded);if(!offer(collector,item,100+seq))return false;previous=chain_of(item);
  }
  auto terminal=page(9,8,previous,true,8);terminal.final_state()=4;
  auto decoded=owned_fixture::evidence();
  decoded=terminal;
  decoded.motion_epoch()=18;decoded.final_state()=4;decoded.release_committed()=true;
  decoded.first_failure_source()=0x100a;decoded.first_failure_axis_mask()=mode==8?2U:63U;rewrite_page(terminal,decoded);
  if(!offer(collector,terminal,200))return false;
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(2);
  while(!gate.entered && std::chrono::steady_clock::now()<deadline)std::this_thread::sleep_for(std::chrono::milliseconds(1));
  const bool not_published=gate.entered && channel.raw_evidence_terminal_flushed().run_id==0;
  if(mode==7)channel.reset_worker_release_connection();
  gate.release=true;collector.stop();::unlink(path);
  const auto proof=channel.raw_evidence_terminal_flushed();
  if(!not_published || collector.failure()!=RawEvidenceCollector::Failure::kNone || proof.run_id!=9 ||
     proof.owner_retirement_disable_valid!=(mode==0 || mode==7 || mode==8) || proof.owner_retirement_motion_epoch!=21 ||
     proof.connection_generation!=1 || proof.full_fault_groups_valid)return false;
  if(mode==7 && proof.connection_generation==channel.worker_connection_generation_)return false;
 }
 return true;
}
}  // namespace

/**
 * @brief 顺序运行27个raw采集生命周期、序列、链与writer用例。
 *
 * 遇第一个false打印对应stderr标签并返回1；每个异步用例用自身steady_clock期限协调线程。测试执行会创建本地临时文件/子进程，但不连接硬件。
 * @return 0 全部用例通过；1 第一个失败用例返回false。
 */
int main() {
  if(!test_autonomous_owner_disable_receipt()) {std::fprintf(stderr,"autonomous owner Disable proof failed\n");return 1;}
  if (!test_management_wire_records()) { std::fprintf(stderr, "management records failed\n"); return 1; }
  if (!test_management_offers_require_open_capture()) { std::fprintf(stderr, "management lifecycle failed\n"); return 1; }
  if (!test_management_wrong_ids_are_preserved()) { std::fprintf(stderr, "management wrong IDs failed\n"); return 1; }
  if (!test_management_length_fails_closed()) { std::fprintf(stderr, "management length failed\n"); return 1; }
  if (!test_management_overflow_with_running_writer()) { std::fprintf(stderr, "management overflow failed\n"); return 1; }
  if (!test_lifecycle_and_chain()) { std::fprintf(stderr, "lifecycle/chain failed\n"); return 1; }
  if (!test_attempt_id_mismatch_rejected()) { std::fprintf(stderr, "attempt id mismatch failed\n"); return 1; }
  if (!test_sequence_and_identity()) { std::fprintf(stderr, "sequence/identity failed\n"); return 1; }
  if (!test_bounded_queue_and_file()) { std::fprintf(stderr, "queue/file failed\n"); return 1; }
  if (!test_capture_path_rejects_swapped_parent()) { std::fprintf(stderr, "path failed\n"); return 1; }
  if (!test_shared_failure_is_sticky_for_concurrent_readers()) { std::fprintf(stderr, "shared sticky failed\n"); return 1; }
  if (!test_shared_failure_reports_overflow_io_and_incomplete()) { std::fprintf(stderr, "shared failures failed\n"); return 1; }
  if (!test_terminal_ack_waits_for_flush()) { std::fprintf(stderr, "terminal flush failed\n"); return 1; }
  if (!test_successor_queued_before_terminal_flush()) { std::fprintf(stderr, "pending successor failed\n"); return 1; }
  if (!test_terminal_write_error_never_acknowledges()) { std::fprintf(stderr, "terminal write error failed\n"); return 1; }
  if (!test_terminal_snapshot_is_coherent()) { std::fprintf(stderr, "terminal snapshot failed\n"); return 1; }
  if (!test_first_page_receipt_after_write()) { std::fprintf(stderr, "first page receipt failed\n"); return 1; }
  if (!test_first_page_receipt_rejects_bad_chain()) { std::fprintf(stderr, "first page chain receipt failed\n"); return 1; }
  if (!test_first_page_partial_write_never_publishes()) { std::fprintf(stderr, "first page write error failed\n"); return 1; }
  if (!test_first_page_flush_failure_never_publishes()) { std::fprintf(stderr, "first page flush error failed\n"); return 1; }
  if (!test_first_page_mask_and_next_run()) { std::fprintf(stderr, "first page masks/runs failed\n"); return 1; }
  if (!test_strict_refresh_successor_receipt()) { std::fprintf(stderr, "strict first page failed\n"); return 1; }
  if (!test_reported_group_count_may_lead_but_not_regress()) { std::fprintf(stderr, "reported group count failed\n"); return 1; }
  if (!test_flushed_clean_cancel_receipt()) { std::fprintf(stderr, "clean cancel receipt failed\n"); return 1; }
  if (!test_same_capture_reboot_fails_closed()) { std::fprintf(stderr, "same capture reboot failed\n"); return 1; }
  if (!test_first_page_sequence_and_snapshot()) { std::fprintf(stderr, "first page sequence/snapshot failed\n"); return 1; }
  if (!test_many_runs_and_recycled_late_page()) { std::fprintf(stderr, "many runs failed\n"); return 1; }
  return 0;
}
