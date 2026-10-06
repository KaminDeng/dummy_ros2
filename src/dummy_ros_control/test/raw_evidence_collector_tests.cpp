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
#include "dp_dummy_robot_v2.hpp"
#include "dummy_ros_control/device_management_channel.hpp"
#include "dummy_ros_control/raw_evidence_collector.hpp"

using dummy_ros_control::RawEvidenceCollector;
using dummy_ros_control::RawEvidenceBinding;
using dummy_ros_control::RawEvidenceBindings;
using dummy_ros_control::RawEvidenceIdentity;
using dummy_ros_control::RawEvidencePage;
namespace wire = dp::protocol::dummy_robot_v2;

namespace {
RawEvidenceIdentity identity() {
  RawEvidenceIdentity value{};
  value.client_id[0] = 1U;
  value.contract_hash[0] = 2U;
  value.model_hash[0] = 3U;
  value.profile_hash[0] = 4U;
  value.firmware_hash[0] = 5U;
  return value;
}

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

RawEvidencePage page(std::uint64_t run, std::uint32_t seq,
                     const std::array<std::uint8_t, 32U> &previous,
                     bool terminal, std::uint32_t group_count = UINT32_MAX,
                     bool final_flag = true, std::uint8_t kind = 1U,
                     std::uint64_t parent = 0U, std::uint8_t mask = 63U,
                     std::uint8_t sent_mask = UINT8_MAX, std::uint8_t failed_mask = 0U) {
  if (group_count == UINT32_MAX) group_count = terminal && seq == 0U ? 0U : 1U;
  wire::DummyAttemptEvidenceV2 message{};
  message.set_schema_major(2U);
  message.set_schema_minor(4U);
  message.set_run_id(run);
  message.set_attempt_id(run);
  message.set_evidence_seq(seq);
  message.set_attempt_kind(kind);
  message.set_parent_run_id(parent);
  message.set_category(terminal ? 4U : 2U);
  message.set_final(final_flag ? 1U : 0U);
  message.set_group_count(group_count);
  message.set_motion_epoch(10U);
  if (terminal) {
    message.set_final_state(3U);
    if (group_count == 0U) message.set_terminal_reason(14U);
  }
  else {
    message.set_group_id(7U);
    message.set_expected_mask(mask);
    message.set_submitted_mask(mask);
    message.set_sent_mask(sent_mask == UINT8_MAX ? mask : sent_mask);
    message.set_failed_mask(failed_mask);
  }
  std::uint8_t previous_bytes[32]{};
  std::copy(previous.begin(), previous.end(), previous_bytes);
  message.set_previous_chain_sha256(previous_bytes);
  RawEvidencePage result{};
  result.boot_epoch = 11U;
  result.session_id = 12U;
  result.run_id = run;
  result.parent_run_id = parent;
  result.evidence_seq = seq;
  result.attempt_kind = kind;
  result.category = terminal ? 4U : 2U;
  result.group_count = group_count;
  result.final_state = terminal ? 3U : 0U;
  result.terminal_reason = terminal && group_count == 0U ? 14U : 0U;
  result.final = final_flag;
  result.previous_chain_sha256 = previous;
  if (wire::DummyAttemptEvidenceV2Codec::encode(
          message, result.raw.data(), result.raw.size()) != wire::CodecResult::kOk) std::abort();
  std::array<std::uint8_t, 32U + wire::DummyAttemptEvidenceV2Codec::kCanonicalPrefixSize + 1U> input{};
  std::copy(previous.begin(), previous.end(), input.begin());
  std::copy_n(result.raw.begin(), wire::DummyAttemptEvidenceV2Codec::kCanonicalPrefixSize,
              input.begin() + 32U);
  input.back() = result.raw[wire::DummyAttemptEvidenceV2Codec::kEvidenceLostOffset];
  ::SHA256(input.data(), input.size(), result.chain_sha256.data());
  std::uint8_t chain_bytes[32]{};
  std::copy(result.chain_sha256.begin(), result.chain_sha256.end(), chain_bytes);
  message.set_chain_sha256(chain_bytes);
  if (wire::DummyAttemptEvidenceV2Codec::encode(
          message, result.raw.data(), result.raw.size()) != wire::CodecResult::kOk) std::abort();
  return result;
}

void rewrite_page(RawEvidencePage &value, wire::DummyAttemptEvidenceV2 &message) {
  std::uint8_t previous_bytes[32]{};
  std::copy(value.previous_chain_sha256.begin(), value.previous_chain_sha256.end(), previous_bytes);
  message.set_previous_chain_sha256(previous_bytes);
  std::uint8_t empty_chain[32]{};
  message.set_chain_sha256(empty_chain);
  if (wire::DummyAttemptEvidenceV2Codec::encode(
          message, value.raw.data(), value.raw.size()) != wire::CodecResult::kOk) std::abort();
  std::array<std::uint8_t, 32U + wire::DummyAttemptEvidenceV2Codec::kCanonicalPrefixSize + 1U> input{};
  std::copy(value.previous_chain_sha256.begin(), value.previous_chain_sha256.end(), input.begin());
  std::copy_n(value.raw.begin(), wire::DummyAttemptEvidenceV2Codec::kCanonicalPrefixSize,
              input.begin() + 32U);
  input.back() = value.raw[wire::DummyAttemptEvidenceV2Codec::kEvidenceLostOffset];
  ::SHA256(input.data(), input.size(), value.chain_sha256.data());
  std::uint8_t chain[32]{};
  std::copy(value.chain_sha256.begin(), value.chain_sha256.end(), chain);
  message.set_chain_sha256(chain);
  if (wire::DummyAttemptEvidenceV2Codec::encode(
          message, value.raw.data(), value.raw.size()) != wire::CodecResult::kOk) std::abort();
}

bool test_attempt_id_mismatch_rejected() {
  const auto trust = identity();
  auto first = page(3U, 0U, genesis(trust, 3U, 11U), false);
  wire::DummyAttemptEvidenceV2 decoded{};
  if (wire::DummyAttemptEvidenceV2Codec::decode(
          first.raw.data(), first.raw.size(), &decoded) != wire::CodecResult::kOk) return false;
  decoded.set_attempt_id(4U);
  rewrite_page(first, decoded);
  RawEvidenceCollector collector(trust);
  return !collector.offer_page(first, 100U) &&
      collector.failure() == RawEvidenceCollector::Failure::kInvalid;
}

bool test_reported_group_count_may_lead_but_not_regress() {
  const auto trust = identity();
  auto first = page(3U, 0U, genesis(trust, 3U, 11U), false, 2U);
  auto second = page(3U, 1U, first.chain_sha256, false, 4U);
  auto third = page(3U, 2U, second.chain_sha256, false, 4U);
  auto terminal = page(3U, 3U, third.chain_sha256, true, 3U);
  RawEvidenceCollector legal(trust);
  if (!legal.expect_run(3U) || !legal.offer_page(first, 100U) ||
      !legal.offer_page(second, 101U) || !legal.offer_page(third, 102U) ||
      !legal.offer_page(terminal, 103U)) return false;
  legal.stop();
  auto regressed = page(3U, 2U, second.chain_sha256, false, 3U);
  RawEvidenceCollector invalid(trust);
  return legal.failure() == RawEvidenceCollector::Failure::kNone &&
      invalid.expect_run(3U) && invalid.offer_page(first, 100U) &&
      invalid.offer_page(second, 101U) &&
      !invalid.offer_page(regressed, 102U) &&
      invalid.failure() == RawEvidenceCollector::Failure::kSequence;
}


bool test_lifecycle_and_chain() {
  const auto trust = identity();
  const auto group = page(3U, 0U, genesis(trust, 3U, 11U), false);
  const auto terminal = page(3U, 1U, group.chain_sha256, true);
  RawEvidenceCollector complete(trust);
  if (!complete.expect_run(3U) || !complete.offer_page(group, 100U) ||
      !complete.offer_page(terminal, 101U)) return false;
  complete.stop();
  if (complete.failure() != RawEvidenceCollector::Failure::kNone) return false;
  RawEvidenceCollector incomplete(trust);
  if (!incomplete.expect_run(3U) || !incomplete.offer_page(group, 100U)) return false;
  incomplete.stop();
  if (incomplete.failure() != RawEvidenceCollector::Failure::kIncomplete) return false;
  RawEvidenceCollector no_page(trust);
  if (!no_page.expect_run(3U)) return false;
  no_page.stop();
  if (no_page.failure() != RawEvidenceCollector::Failure::kIncomplete) return false;
  RawEvidenceCollector bad_genesis(trust);
  auto altered = group;
  altered.previous_chain_sha256[0] ^= 1U;
  if (bad_genesis.offer_page(altered, 100U) ||
      bad_genesis.failure() != RawEvidenceCollector::Failure::kChain) return false;
  RawEvidenceCollector nonfinal_group(trust);
  const auto nonfinal = page(3U, 0U, genesis(trust, 3U, 11U), false, 1U, false);
  if (nonfinal_group.offer_page(nonfinal, 100U) ||
      nonfinal_group.failure() != RawEvidenceCollector::Failure::kInvalid) return false;
  RawEvidenceCollector missing_group(trust);
  const auto impossible = page(3U, 0U, genesis(trust, 3U, 11U), true, 1U);
  if (missing_group.offer_page(impossible, 100U) ||
      missing_group.failure() != RawEvidenceCollector::Failure::kSequence) return false;
  RawEvidenceCollector bad_digest(trust);
  altered = terminal;
  altered.chain_sha256[0] ^= 1U;
  return bad_digest.offer_page(group, 100U) && !bad_digest.offer_page(altered, 101U) &&
      bad_digest.failure() == RawEvidenceCollector::Failure::kChain;
}

bool test_management_wire_records() {
  const auto trust = identity();
  char path[] = "/tmp/dummy-raw-management-XXXXXX";
  const int fd = ::mkstemp(path);
  if (fd < 0) return false;
  ::close(fd);
  ::unlink(path);
  RawEvidenceCollector collector(trust);
  if (!collector.start(path)) return false;
  wire::DummyManageV2Request request{};
  request.set_schema_major(2U);
  request.set_schema_minor(4U);
  std::uint8_t client[16]{};
  client[0] = 1U;
  request.set_client_instance_id(client);
  request.set_request_id(80U);
  request.set_request_seq(81U);
  request.set_operation(10U);
  request.set_expected_boot_epoch(11U);
  request.set_expected_session_id(12U);
  request.set_expected_reference_epoch(9U);
  request.set_expected_motion_epoch(10U);
  request.set_expected_management_epoch(1U);
  std::array<std::uint8_t, wire::DummyManageV2RequestCodec::kEncodedSize> request_raw{};
  if (wire::DummyManageV2RequestCodec::encode(request, request_raw.data(), request_raw.size()) !=
      wire::CodecResult::kOk) return false;
  wire::DummyManageV2Response response{};
  response.set_schema_major(2U);
  response.set_schema_minor(4U);
  response.set_client_instance_id(client);
  response.set_request_id(80U);
  response.set_boot_epoch(11U);
  response.set_session_id(12U);
  response.set_run_id(2U);
  response.set_attempt_kind(1U);
  response.set_operation_id(80U);
  response.set_receipt(1U);
  response.set_transaction_phase(3U);
  response.set_release_committed(true);
  std::array<std::uint8_t, wire::DummyManageV2ResponseCodec::kEncodedSize> response_raw{};
  if (wire::DummyManageV2ResponseCodec::encode(response, response_raw.data(), response_raw.size()) !=
      wire::CodecResult::kOk ||
      !collector.offer_management_request(request_raw.data(), request_raw.size(), 100U) ||
      !collector.offer_management_response(response_raw.data(), response_raw.size(), 101U)) return false;
  collector.stop();
  if (collector.failure() != RawEvidenceCollector::Failure::kNone) return false;
  std::FILE *file = std::fopen(path, "rb");
  if (!file) return false;
  std::uint8_t magic[8]{}, request_header[39]{}, response_header[39]{};
  std::array<std::uint8_t, request_raw.size()> recorded_request{};
  std::array<std::uint8_t, response_raw.size()> recorded_response{};
  const bool read = std::fread(magic, 1U, 8U, file) == 8U &&
      std::fread(request_header, 1U, 39U, file) == 39U &&
      std::fread(recorded_request.data(), 1U, recorded_request.size(), file) == recorded_request.size() &&
      std::fread(response_header, 1U, 39U, file) == 39U &&
      std::fread(recorded_response.data(), 1U, recorded_response.size(), file) == recorded_response.size() &&
      std::fgetc(file) == EOF;
  std::fclose(file);
  ::unlink(path);
  return read && request_header[0] == 4U && request_header[1] == 233U &&
      request_header[3] == 100U && request_header[11] == 11U &&
      request_header[19] == 12U && request_header[27] == 0U && request_header[35] == 81U &&
      response_header[0] == 5U && response_header[1] == 132U &&
      response_header[3] == 101U && response_header[11] == 11U &&
      response_header[19] == 12U && response_header[27] == 2U &&
      response_header[35] == 0U && recorded_request == request_raw &&
      recorded_response == response_raw;
}

bool test_management_offers_require_open_capture() {
  wire::DummyManageV2Request request{};
  request.set_schema_major(2U);
  request.set_schema_minor(4U);
  std::array<std::uint8_t, wire::DummyManageV2RequestCodec::kEncodedSize> request_raw{};
  wire::DummyManageV2Response response{};
  response.set_schema_major(2U);
  response.set_schema_minor(4U);
  std::array<std::uint8_t, wire::DummyManageV2ResponseCodec::kEncodedSize> response_raw{};
  if (wire::DummyManageV2RequestCodec::encode(request, request_raw.data(), request_raw.size()) !=
          wire::CodecResult::kOk ||
      wire::DummyManageV2ResponseCodec::encode(response, response_raw.data(), response_raw.size()) !=
          wire::CodecResult::kOk) return false;
  RawEvidenceCollector never_started(identity());
  if (never_started.offer_management_request(request_raw.data(), request_raw.size(), 10U) ||
      never_started.failure() != RawEvidenceCollector::Failure::kIo) return false;
  RawEvidenceCollector never_started_response(identity());
  if (never_started_response.offer_management_response(response_raw.data(), response_raw.size(), 11U) ||
      never_started_response.failure() != RawEvidenceCollector::Failure::kIo) return false;
  char path[] = "/tmp/dummy-raw-management-closed-XXXXXX";
  const int fd = ::mkstemp(path);
  if (fd < 0) return false;
  ::close(fd);
  ::unlink(path);
  RawEvidenceCollector closed(identity());
  if (!closed.start(path)) return false;
  closed.stop();
  const bool rejected = !closed.offer_management_response(response_raw.data(), response_raw.size(), 12U) &&
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
      !closed_request.offer_management_request(request_raw.data(), request_raw.size(), 13U) &&
      closed_request.failure() == RawEvidenceCollector::Failure::kIo &&
      closed_request.written() == 0U;
  const bool request_only_magic = ::stat(request_path, &file_stat) == 0 && file_stat.st_size == 8;
  ::unlink(request_path);
  return rejected && only_magic && request_rejected && request_only_magic;
}

bool test_management_wrong_ids_are_preserved() {
  char path[] = "/tmp/dummy-raw-management-ids-XXXXXX";
  const int fd = ::mkstemp(path);
  if (fd < 0) return false;
  ::close(fd);
  ::unlink(path);
  RawEvidenceCollector collector(identity());
  if (!collector.start(path)) return false;
  wire::DummyManageV2Response response{};
  response.set_schema_major(2U);
  response.set_schema_minor(4U);
  response.set_request_id(999U);
  response.set_run_id(4U);
  response.set_boot_epoch(11U);
  response.set_session_id(12U);
  std::array<std::uint8_t, wire::DummyManageV2ResponseCodec::kEncodedSize> raw{};
  const bool offered = wire::DummyManageV2ResponseCodec::encode(response, raw.data(), raw.size()) ==
      wire::CodecResult::kOk &&
      collector.offer_management_response(raw.data(), raw.size(), 12U) &&
      collector.failure() == RawEvidenceCollector::Failure::kNone;
  collector.stop();
  std::FILE *file = std::fopen(path, "rb");
  if (!file) return false;
  std::array<std::uint8_t, 8U + 39U + raw.size()> recorded{};
  const bool exact = std::fread(recorded.data(), 1U, recorded.size(), file) == recorded.size() &&
      std::fgetc(file) == EOF;
  std::fclose(file);
  ::unlink(path);
  return offered && exact && collector.failure() == RawEvidenceCollector::Failure::kNone &&
      recorded[8U] == 5U && recorded[19U] == 11U && recorded[27U] == 12U &&
      recorded[35U] == 4U &&
      std::equal(raw.begin(), raw.end(), recorded.begin() + 47U);
}

bool test_management_length_fails_closed() {
  wire::DummyManageV2Request request{};
  request.set_schema_major(2U);
  request.set_schema_minor(4U);
  request.set_request_id(80U);
  request.set_request_seq(80U);
  std::array<std::uint8_t, wire::DummyManageV2RequestCodec::kEncodedSize> raw{};
  if (wire::DummyManageV2RequestCodec::encode(request, raw.data(), raw.size()) !=
      wire::CodecResult::kOk) return false;
  RawEvidenceCollector short_record(identity());
  if (short_record.offer_management_request(raw.data(), raw.size() - 1U, 1U) ||
      short_record.failure() != RawEvidenceCollector::Failure::kInvalid) return false;
  return true;
}

bool test_sequence_and_identity() {
  const auto trust = identity();
  const auto group = page(3U, 0U, genesis(trust, 3U, 11U), false);
  RawEvidenceCollector duplicate(trust);
  if (!duplicate.offer_page(group, 100U) || duplicate.offer_page(group, 101U) ||
      duplicate.failure() != RawEvidenceCollector::Failure::kSequence) return false;
  RawEvidenceCollector missing(trust);
  const auto skipped = page(3U, 2U, group.chain_sha256, true);
  if (!missing.offer_page(group, 100U) || missing.offer_page(skipped, 101U) ||
      missing.failure() != RawEvidenceCollector::Failure::kSequence) return false;
  RawEvidenceCollector identity_check(trust);
  auto wrong = page(3U, 1U, group.chain_sha256, true);
  wrong.session_id = 13U;
  return identity_check.offer_page(group, 100U) && !identity_check.offer_page(wrong, 101U) &&
      identity_check.failure() == RawEvidenceCollector::Failure::kIdentity;
}

bool test_bounded_queue_and_file() {
  const auto trust = identity();
  RawEvidenceCollector full(trust);
  std::array<std::uint8_t, 16U> sample{};
  for (std::size_t i = 0; i < RawEvidenceCollector::kCapacity; ++i)
    if (!full.offer_observation(RawEvidenceCollector::Kind::kState,
                                sample.data(), sample.size(), i, 11U, 12U)) return false;
  if (full.offer_observation(RawEvidenceCollector::Kind::kStatus,
                             sample.data(), sample.size(), 99U, 11U, 12U) ||
      full.failure() != RawEvidenceCollector::Failure::kOverflow) return false;
  char path[] = "/tmp/dummy-raw-evidence-XXXXXX";
  const int fd = ::mkstemp(path);
  if (fd < 0) return false;
  ::close(fd);
  ::unlink(path);
  RawEvidenceCollector file(trust);
  const auto terminal = page(3U, 0U, genesis(trust, 3U, 11U), true);
  const bool offered = file.start(path) && file.expect_run(3U) && file.offer_page(terminal, 100U);
  file.stop();
  std::FILE *input = std::fopen(path, "rb");
  if (!input) return false;
  std::array<std::uint8_t, 1024U> bytes{};
  const auto size = std::fread(bytes.data(), 1U, bytes.size(), input);
  std::fclose(input);
  ::unlink(path);
  return offered && file.failure() == RawEvidenceCollector::Failure::kNone &&
      file.written() == 1U && size == 8U + 39U + terminal.raw.size() &&
      std::equal(bytes.begin(), bytes.begin() + 7U, "DPRAW01") &&
      bytes[8] == static_cast<std::uint8_t>(RawEvidenceCollector::Kind::kPage) &&
      std::equal(terminal.raw.begin(), terminal.raw.end(), bytes.begin() + 47U);
}

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
  bad.chain_sha256[0] ^= 1U;
  const bool rejected = !collector.offer_page(bad, 100U);
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

bool test_shared_failure_reports_overflow_io_and_incomplete() {
  dummy_ros_control::DeviceManagementChannel overflow_channel({});
  RawEvidenceCollector overflow(identity(), &overflow_channel);
  std::array<std::uint8_t, 1U> raw{};
  for (std::size_t i = 0U; i < RawEvidenceCollector::kCapacity; ++i)
    if (!overflow.offer_observation(RawEvidenceCollector::Kind::kState,
                                    raw.data(), raw.size(), i, 11U, 12U)) return false;
  if (overflow.offer_observation(RawEvidenceCollector::Kind::kState,
                                 raw.data(), raw.size(), 999U, 11U, 12U) ||
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

struct FlushGate {
  std::atomic<bool> entered{false};
  std::atomic<bool> release{false};
};

bool wait_before_terminal_flush(void *context) noexcept {
  auto &gate = *static_cast<FlushGate *>(context);
  gate.entered.store(true, std::memory_order_release);
  while (!gate.release.load(std::memory_order_acquire))
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  return true;
}

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
      !collector.offer_page(terminal, 1U)) return false;
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
  wire::DummyManageV2Request request{};
  request.set_schema_major(2U);
  request.set_schema_minor(4U);
  std::array<std::uint8_t, wire::DummyManageV2RequestCodec::kEncodedSize> raw{};
  bool accepted = wire::DummyManageV2RequestCodec::encode(request, raw.data(), raw.size()) ==
      wire::CodecResult::kOk;
  for (std::size_t i = 1U; accepted && i < RawEvidenceCollector::kCapacity; ++i)
    accepted = collector.offer_management_request(raw.data(), raw.size(), i + 1U);
  const bool overflow = !collector.offer_management_request(raw.data(), raw.size(), 999U) &&
      collector.failure() == RawEvidenceCollector::Failure::kOverflow;
  gate.release = true;
  collector.stop();
  ::unlink(path);
  return accepted && overflow && collector.failure() == RawEvidenceCollector::Failure::kOverflow;
}

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
  const auto terminal = page(3U, 1U, group.chain_sha256, true);
  if (!collector.offer_page(group, 100U) || !collector.offer_page(terminal, 101U)) return false;
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
  auto terminal = page(3U, 1U, first.chain_sha256, true);
  if (!collector.start(path) || !collector.expect_run(3U) ||
      !collector.offer_page(first, 100U) || !collector.offer_page(terminal, 101U)) return false;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (!gate.entered.load(std::memory_order_acquire) &&
         std::chrono::steady_clock::now() < deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  auto next = page(4U, 0U, genesis(trust, 4U, 11U), false);
  auto next_terminal = page(4U, 1U, next.chain_sha256, true);
  const bool queued = gate.entered.load(std::memory_order_acquire) &&
      collector.expect_run(4U) && collector.offer_page(next, 102U) &&
      collector.offer_page(next_terminal, 103U) &&
      channel.raw_evidence_terminal_flushed().run_id == 0U;
  gate.release.store(true, std::memory_order_release);
  collector.stop();
  ::unlink(path);
  return queued && collector.failure() == RawEvidenceCollector::Failure::kNone &&
      channel.raw_evidence_terminal_flushed().run_id == 4U;
}

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
    const rlimit limit{429U, 429U};
    if (::setrlimit(RLIMIT_FSIZE, &limit) != 0) ::_exit(2);
    dummy_ros_control::TransportParameters config{};
    config.raw_evidence_path = path;
    dummy_ros_control::DeviceManagementChannel channel(config);
    RawEvidenceCollector collector(identity(), &channel);
    const auto group = page(3U, 0U, genesis(identity(), 3U, 11U), false);
    const auto terminal = page(3U, 1U, group.chain_sha256, true);
    if (!collector.start(path) || !collector.expect_run(3U) ||
        !collector.offer_page(group, 100U)) ::_exit(3);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (collector.pages_written() != 1U && std::chrono::steady_clock::now() < deadline)
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    if (!collector.offer_page(terminal, 101U)) ::_exit(4);
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
      !collector.offer_page(first, 100U)) return false;
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
  const auto terminal = page(3U, 1U, first.chain_sha256, true);
  const bool ended = collector.offer_page(terminal, 101U);
  collector.stop();
  ::unlink(path);
  return valid && ended && collector.failure() == RawEvidenceCollector::Failure::kNone;
}

bool test_first_page_receipt_rejects_bad_chain() {
  char path[] = "/tmp/dummy-raw-first-chain-XXXXXX";
  const int fd = ::mkstemp(path);
  if (fd < 0) return false;
  ::close(fd);
  ::unlink(path);
  dummy_ros_control::DeviceManagementChannel channel({});
  RawEvidenceCollector collector(identity(), &channel);
  auto first = page(3U, 0U, genesis(identity(), 3U, 11U), false);
  first.previous_chain_sha256[0] ^= 1U;
  const bool rejected = collector.start(path) && collector.expect_run(3U) &&
      !collector.offer_page(first, 100U) &&
      channel.raw_evidence_first_page_written().run_id == 0U;
  collector.stop();
  ::unlink(path);
  return rejected;
}

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
        !collector.offer_page(first, 100U)) ::_exit(3);
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

struct FirstPageFlushProbe {
  std::atomic<unsigned> calls{0U};
};

int fail_first_page_flush(std::FILE *, void *context) noexcept {
  static_cast<FirstPageFlushProbe *>(context)->calls.fetch_add(1U, std::memory_order_relaxed);
  return EOF;
}

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
      !collector.offer_page(first, 100U)) return false;
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
    const auto terminal = page(run, 1U, first.chain_sha256, true);
    if (!collector.expect_run(run) || !collector.offer_page(first, run) ||
        !collector.offer_page(terminal, run + 1U)) return false;
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
    wire::DummyAttemptEvidenceV2 decoded{};
    if (wire::DummyAttemptEvidenceV2Codec::decode(
            first.raw.data(), first.raw.size(), &decoded) != wire::CodecResult::kOk) return false;
    decoded.set_category(1U);
    if (run == 5U) decoded.set_group_id(0U);
    first.category = 1U;
    rewrite_page(first, decoded);
    const auto terminal = page(run, 1U, first.chain_sha256, true);
    if (!collector.expect_run(run) || !collector.offer_page(first, run) ||
        !collector.offer_page(terminal, run + 1U)) return false;
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
  wire::DummyAttemptEvidenceV2 decoded{};
  if (wire::DummyAttemptEvidenceV2Codec::decode(
          first.raw.data(), first.raw.size(), &decoded) != wire::CodecResult::kOk) return false;
  decoded.set_motion_epoch(5U);
  rewrite_page(first, decoded);
  auto motion = page(3U, 1U, first.chain_sha256, false, 2U);
  if (wire::DummyAttemptEvidenceV2Codec::decode(
          motion.raw.data(), motion.raw.size(), &decoded) != wire::CodecResult::kOk) return false;
  decoded.set_motion_epoch(6U);
  rewrite_page(motion, decoded);
  auto safety = page(3U, 2U, motion.chain_sha256, false, 3U);
  if (wire::DummyAttemptEvidenceV2Codec::decode(
          safety.raw.data(), safety.raw.size(), &decoded) != wire::CodecResult::kOk) return false;
  decoded.set_motion_epoch(7U);
  rewrite_page(safety, decoded);
  auto terminal = page(3U, 3U, safety.chain_sha256, true, 3U);
  if (wire::DummyAttemptEvidenceV2Codec::decode(
          terminal.raw.data(), terminal.raw.size(), &decoded) != wire::CodecResult::kOk) return false;
  decoded.set_final_state(7U);
  decoded.set_motion_epoch(5U);
  terminal.final_state = 7U;
  rewrite_page(terminal, decoded);
  if (!collector.expect_run(3U) || !collector.offer_page(first, 100U) ||
      !collector.offer_page(motion, 101U) ||
      !collector.offer_page(safety, 102U) ||
      !collector.offer_page(terminal, 103U)) return false;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (channel.raw_evidence_terminal_flushed().run_id != 3U &&
         std::chrono::steady_clock::now() < deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  const auto receipt = channel.raw_evidence_terminal_flushed();
  for (std::uint64_t run = 4U; run <= 6U; ++run) {
    auto next_first = page(run, 0U, genesis(trust, run, 11U), false);
    if (run == 6U) {
      wire::DummyAttemptEvidenceV2 first_decoded{};
      if (wire::DummyAttemptEvidenceV2Codec::decode(
            next_first.raw.data(), next_first.raw.size(), &first_decoded)
          != wire::CodecResult::kOk) return false;
      first_decoded.set_motion_epoch(1U);
      rewrite_page(next_first, first_decoded);
    }
    auto previous = next_first.chain_sha256;
    std::uint32_t terminal_seq = 1U, group_count = 1U;
    if (run >= 5U) {
      auto wrong_epoch = page(run, 1U, previous, false, 2U);
      wire::DummyAttemptEvidenceV2 wrong_decoded{};
      if (wire::DummyAttemptEvidenceV2Codec::decode(
            wrong_epoch.raw.data(), wrong_epoch.raw.size(), &wrong_decoded)
          != wire::CodecResult::kOk) return false;
      wrong_decoded.set_motion_epoch(run == 5U ? 2U : 3U);
      if (run == 5U) wrong_decoded.set_sent_mask(31U);
      rewrite_page(wrong_epoch, wrong_decoded);
      if (!collector.expect_run(run) || !collector.offer_page(next_first, 200U) ||
          !collector.offer_page(wrong_epoch, 201U)) return false;
      previous = wrong_epoch.chain_sha256;
      terminal_seq = 2U; group_count = 2U;
      if (run == 6U) {
        auto stale = page(run, 2U, previous, false, 3U);
        if (wire::DummyAttemptEvidenceV2Codec::decode(
              stale.raw.data(), stale.raw.size(), &wrong_decoded)
            != wire::CodecResult::kOk) return false;
        wrong_decoded.set_motion_epoch(2U);
        rewrite_page(stale, wrong_decoded);
        if (!collector.offer_page(stale, 202U)) return false;
        previous = stale.chain_sha256;
        terminal_seq = 3U; group_count = 3U;
      }
    } else if (!collector.expect_run(run) ||
               !collector.offer_page(next_first, 200U)) return false;
    auto next_terminal = page(run, terminal_seq, previous, true, group_count);
    if (run == 5U) {
      wire::DummyAttemptEvidenceV2 terminal_decoded{};
      if (wire::DummyAttemptEvidenceV2Codec::decode(
            next_terminal.raw.data(), next_terminal.raw.size(), &terminal_decoded)
          != wire::CodecResult::kOk) return false;
      terminal_decoded.set_motion_epoch(2U);
      rewrite_page(next_terminal, terminal_decoded);
    }
    if (!collector.offer_page(next_terminal, 202U)) return false;
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

bool test_same_capture_reboot_fails_closed() {
  const auto trust = identity();
  RawEvidenceCollector collector(trust);
  const auto first = page(3U, 0U, genesis(trust, 3U, 11U), false);
  const auto terminal = page(3U, 1U, first.chain_sha256, true);
  if (!collector.offer_page(first, 1U) || !collector.offer_page(terminal, 2U)) return false;
  auto rebooted = page(4U, 0U, genesis(trust, 4U, 12U), false);
  rebooted.boot_epoch = 12U;
  return !collector.offer_page(rebooted, 3U) &&
      collector.failure() == RawEvidenceCollector::Failure::kIdentity;
}

bool test_first_page_sequence_and_snapshot() {
  dummy_ros_control::DeviceManagementChannel channel({});
  const auto trust = identity();
  RawEvidenceCollector missing(trust, &channel);
  const auto seq_one = page(3U, 1U, genesis(trust, 3U, 11U), false);
  if (missing.offer_page(seq_one, 1U) ||
      missing.failure() != RawEvidenceCollector::Failure::kSequence ||
      channel.raw_evidence_first_page_written().run_id != 0U) return false;
  RawEvidenceCollector wrapped(trust, &channel);
  const auto max_seq = page(3U, UINT32_MAX, genesis(trust, 3U, 11U), false);
  if (wrapped.offer_page(max_seq, 1U) ||
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
    const RawEvidenceBinding binding{run, 11U, 12U, parent, kind};
    if (!bindings.observe(binding, collector.terminal_written_run_id()) ||
        !bindings.find(run)) return false;
    const auto group = page(run, 0U, genesis(trust, run, 11U), false,
                            1U, true, kind, parent);
    const auto terminal = page(run, 1U, group.chain_sha256, true,
                               1U, true, kind, parent);
    if (!collector.expect_run(run) || !collector.offer_page(group, run * 10U) ||
        !collector.offer_page(terminal, run * 10U + 1U) ||
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
    const auto terminal = page(run, 1U, group.chain_sha256, true);
    if (!late.expect_run(run) || !late.offer_page(group, run) ||
        !late.offer_page(terminal, run + 1U)) return false;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (late.pages_written() < (run - 2U) * 2U &&
           std::chrono::steady_clock::now() < deadline)
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  const auto old = page(3U, 0U, genesis(trust, 3U, 11U), false);
  const bool rejected = !late.offer_page(old, 999U) &&
      late.failure() == RawEvidenceCollector::Failure::kIdentity;
  late.stop();
  ::unlink(late_path);
  return rejected;
}
}  // namespace

int main() {
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
