/** @file historical_idle_baseline_fixture.hpp
 * @brief 保留原历史释放/清错谱系断言，实际Owned与Source分开构造。 */
#pragma once
#include "dummy_ros_control/device_transport.hpp"
#include "owned_message_fixtures.hpp"
namespace root_fixture {
inline int historical_idle_baseline_cases() {
  using namespace dummy_ros_control;
  // 旧raw correlation映射已退役；真实typed RPC业务ID隔离由root_owned_transport_tests覆盖。
  // 清错只清当前故障；三种合法历史谱系都可恢复只读，未知历史源不自行变成新尝试。
  for (const auto kind : {1U, 2U, 3U}) {
    for (const auto source : {0x100aU, 0x1068U}) {
      HistoricalIdleBaselineContext context{};context.manual_reference_mode=true;
      auto value=owned_fixture::status();dp::comm::SourceId value_source{};value_source.boot_id=(8290386956477645553ULL);value.run_id()=(9U);
      value.attempt_kind()=(kind);value.parent_run_id()=(kind==1U?0U:8U);
      value.final_state()=(4U);value.core_state()=(2U);value.reference_epoch()=(1U);
      value.cleanup_state()=(3U);value.release_committed()=(true);
      value.failure_source()=(source);value.failure_axis_mask()=(2U);value.configured_mask()=(63U);
      // 首次零链固定后不能被迟到的非零摘要替换；这里只识别历史，不验收历史原件。
      if(!historical_released_idle_baseline(value,context,value_source))return 430;
      // Native只读恢复不以历史轴掩码授资格，零掩码不应阻断冷接入。
      value.failure_axis_mask()=(0U);
      if(!historical_released_idle_baseline(value,context,value_source))return 433;
      context.current_boot=value_source.boot_id;context.historical_run_boot=value_source.boot_id;
      context.historical_run_id=9U;context.historical_reference_epoch=1U;
      if(!historical_released_idle_baseline(value,context,value_source))return 431;
      std::uint8_t changed_chain[32]{1};value.evidence_chain_sha256().emplace().assign(changed_chain,32);
      if(historical_released_idle_baseline(value,context,value_source))return 432;
    }
  }
  for (const auto known_source : {0x100aU, 0x1068U, 0x1067U, 0x1024U, 0x1008U}) {
    HistoricalIdleBaselineContext manual{};
    manual.manual_reference_mode = true;
    auto recovered=owned_fixture::status();dp::comm::SourceId recovered_source{};
    recovered_source.boot_id=(7);
    recovered.run_id()=(57);
    recovered.attempt_kind()=(1);
    recovered.final_state()=(4);
    recovered.core_state()=(2);
    recovered.reference_epoch()=(1);
    recovered.release_committed()=(true);
    recovered.cleanup_state()=(3);
    recovered.failure_source()=(known_source);
    recovered.failure_axis_mask()=(63U);recovered.configured_mask()=(63U);
    // GUI42已显式ClearFault：当前Idle，历史Motion故障保留，链摘要尚为全零也合法。
    if (known_source==0x100aU) {
      recovered_source.boot_id=(8290386956477645553ULL);recovered.run_id()=(9U);
      recovered.motion_epoch()=(21U);recovered.management_epoch()=(38U);
    }
    if (known_source==0x1024U) {
      recovered_source.boot_id=(17820284845494219766ULL);
      recovered.run_id()=(99U); // panel-interface-02当前已Clear的历史Fault4。
    }
    if (known_source == 0x1008U) {
      recovered_source.boot_id=(1754154692906416797ULL);
      recovered.run_id()=(15U); // GUI HOME已显式清除导数拒绝，历史失败仍保留。
    }
    if (!historical_released_idle_baseline(recovered,manual,recovered_source))
      return 360;
    auto partial=recovered;auto partial_source=recovered_source;partial.failure_axis_mask()=(2U);
    if (!historical_released_idle_baseline(partial,manual,partial_source))return 394;
    auto bench = manual;
    bench.manual_reference_mode = false;
    if (historical_released_idle_baseline(recovered,bench,recovered_source))
      return 361;
    for (unsigned i = 0; i < 14; ++i) {
      auto bad=recovered;auto bad_source=recovered_source;
      switch (i) {
      case 0:
        bad.first_failure_reason()=(0x1067);
        break;
      case 1:
        bad.core_state()=(9);
        break;
      case 2:
        bad.failure_source()=(0);
        break;
      case 3:
        bad.cleanup_state()=(2);
        break;
      case 4:
        bad.can_failed_mask()=(1);
        break;
      case 5:
        bad.session_id()=(8);
        break;
      case 6:
        bad.reference_epoch()=(0);
        break;
      case 7:
        bad.parent_run_id()=(56);
        break;
      case 8:
        bad.release_committed()=(false);
        break;
      case 9: bad.evidence_lost()=(true); break;
      case 10: bad.real_motion_permitted()=(true); break;
      case 11: bad.permit_remaining_us()=(1U); break;
      case 12: bad.attempt_kind()=(2U); break;
      case 13: bad_source.boot_id=(0U); break;
      }
      if (historical_released_idle_baseline(bad,manual,bad_source))
        return 362 + i;
    }
    manual.current_boot = recovered_source.boot_id;
    manual.historical_run_boot = recovered_source.boot_id;
    manual.historical_run_id = (*recovered.run_id());
    manual.historical_reference_epoch = 1;
    if (!historical_released_idle_baseline(recovered,manual,recovered_source))
      return 371;
    auto changed=recovered;auto changed_source=recovered_source;
    changed.reference_epoch()=(2);
    if (historical_released_idle_baseline(changed,manual,changed_source))
      return 372;
    manual.pages_written = 1;
    if (historical_released_idle_baseline(recovered,manual,recovered_source))
      return 373;
    manual.pages_written=0;
    manual.terminal_written_run_id=(*recovered.run_id());
    if(historical_released_idle_baseline(recovered,manual,recovered_source)) return 390;
    manual.terminal_written_run_id=0;
    changed=recovered;changed_source=recovered_source; changed_source.boot_id=(recovered_source.boot_id+1);
    if(historical_released_idle_baseline(changed,manual,changed_source)) return 391;
    changed=recovered;changed_source=recovered_source; changed.run_id()=((*recovered.run_id())+1);
    if(historical_released_idle_baseline(changed,manual,changed_source)) return 392;
    manual.historical_chain[0]=1;
    if(historical_released_idle_baseline(recovered,manual,recovered_source)) return 393;

  }
  {
    HistoricalIdleBaselineContext manual{};
    manual.manual_reference_mode = true;
    auto recovered=owned_fixture::status();dp::comm::SourceId recovered_source{};
    recovered_source.boot_id=(7);
    recovered.run_id()=(59);
    recovered.parent_run_id()=(58);
    recovered.attempt_kind()=(3);
    recovered.final_state()=(4);
    recovered.core_state()=(2);
    recovered.reference_epoch()=(1);
    recovered.release_committed()=(true);
    recovered.cleanup_state()=(3);
    recovered.failure_source()=(0x1064);
    recovered.failure_axis_mask()=(63U);recovered.configured_mask()=(63U);
    if (!historical_released_idle_baseline(recovered,manual,recovered_source))
      return 390;
    auto bench = manual;
    bench.manual_reference_mode = false;
    if (historical_released_idle_baseline(recovered,bench,recovered_source))
      return 391;
    for (unsigned i = 0; i < 8; ++i) {
      auto bad=recovered;auto bad_source=recovered_source;
      switch (i) {
      case 0:
        bad.first_failure_reason()=(0x1064);
        break;
      case 1:
        bad.core_state()=(9);
        break;
      case 2:
        bad.release_committed()=(false);
        break;
      case 3:
        bad.failure_source()=(0);
        break;
      case 4:
        bad.attempt_kind()=(4);
        break;
      case 5:
        bad.parent_run_id()=(57);
        break;
      case 6:
        bad.cleanup_state()=(2);
        break;
      case 7:
        bad.can_failed_mask()=(1);
        break;
      }
      if (historical_released_idle_baseline(bad,manual,bad_source))
        return 392 + i;
    }
  }
  {
    // GUI25原件：ClearFault后仅当前故障清零，历史Safety失败不能伪装成成功运动。
    HistoricalIdleBaselineContext manual{};
    manual.manual_reference_mode = true;
    auto recovered=owned_fixture::status();dp::comm::SourceId recovered_source{};
    recovered_source.boot_id=(1754154692906416797ULL);
    recovered.run_id()=(61);
    recovered.parent_run_id()=(60);
    recovered.attempt_kind()=(2);
    recovered.final_state()=(4);
    recovered.core_state()=(2);
    recovered.reference_epoch()=(2);
    recovered.release_committed()=(true);
    recovered.cleanup_state()=(3);
    recovered.failure_source()=(0x1002);
    recovered.failure_axis_mask()=(63U);recovered.configured_mask()=(63U);
    if (!historical_released_idle_baseline(recovered,manual,recovered_source)) return 410;
    auto bench = manual;
    bench.manual_reference_mode = false;
    if (historical_released_idle_baseline(recovered,bench,recovered_source)) return 411;
    for (unsigned i = 0; i < 13; ++i) {
      auto bad=recovered;auto bad_source=recovered_source;
      switch (i) {
      case 0: bad.first_failure_reason()=(0x1002); break;
      case 1: bad.core_state()=(9); break;
      case 2: bad.release_committed()=(false); break;
      case 3: bad.failure_source()=(0U); break;
      case 4: bad.parent_run_id()=(59); break;
      case 5: bad.cleanup_state()=(2); break;
      case 6: bad.can_failed_mask()=(1); break;
      case 7: bad.evidence_lost()=(true); break;
      case 8: bad.session_id()=(1); break;
      case 9: bad.real_motion_permitted()=(true); break;
      case 10: bad.permit_remaining_us()=(1); break;
      case 11: bad.reference_epoch()=(0); break;
      case 12: bad.final_state()=(2); break;
      }
      if (historical_released_idle_baseline(bad,manual,bad_source)) return 412 + i;
    }
    manual.pages_written = 1;
    if (historical_released_idle_baseline(recovered,manual,recovered_source)) return 425;
    manual.pages_written = 0;
    manual.terminal_written_run_id = 61;
    if (historical_released_idle_baseline(recovered,manual,recovered_source)) return 426;
    manual.terminal_written_run_id = 0;
    manual.current_boot = 1;
    if (historical_released_idle_baseline(recovered,manual,recovered_source)) return 427;
  }
  {
    HistoricalIdleBaselineContext manual{};
    manual.manual_reference_mode = true;
    auto retained=owned_fixture::status();dp::comm::SourceId retained_source{};
    retained_source.boot_id=(7);
    retained.run_id()=(113);
    retained.attempt_kind()=(1);
    retained.final_state()=(2);
    retained.core_state()=(2);
    retained.reference_epoch()=(1);
    retained.release_committed()=(true);
    if (!historical_released_idle_baseline(retained,manual,retained_source))
      return 350;
    auto bench = manual;
    bench.manual_reference_mode = false;
    if (historical_released_idle_baseline(retained,bench,retained_source))
      return 351;
    auto bad=retained;auto bad_source=retained_source;
    bad.session_id()=(8);
    if (historical_released_idle_baseline(bad,manual,bad_source))
      return 352;
    bad=retained;bad_source=retained_source;
    bad.release_committed()=(false);
    if (historical_released_idle_baseline(bad,manual,bad_source))
      return 353;
    manual.current_boot = 7;
    manual.historical_run_boot = 7;
    manual.historical_run_id = 113;
    manual.historical_reference_epoch = 1;
    if (!historical_released_idle_baseline(retained,manual,retained_source))
      return 354;
    bad=retained;bad_source=retained_source;
    bad.reference_epoch()=(2);
    if (historical_released_idle_baseline(bad,manual,bad_source))
      return 355;
    manual.pages_written = 1;
    if (historical_released_idle_baseline(retained,manual,retained_source))
      return 356;
  }

  /**
   * @brief context全部为零表示新采集尚未建立当前boot/run证据。
   * 随后逐项赋值pages_written、historical_run_*、terminal_written_run_id验证“历史基线例外”不会覆盖本次已有证据。
   */
  HistoricalIdleBaselineContext context{};
  auto status=owned_fixture::status();dp::comm::SourceId status_source{};
  status_source.boot_id=(7U);
  status.run_id()=(212U);
  status.parent_run_id()=(211U);
  status.attempt_kind()=(3U);
  status.final_state()=(8U);
  status.core_state()=(1U);
  status.session_id()=(0U);
  status.reference_epoch()=(0U);
  status.release_committed()=(true);
  status.real_motion_permitted()=(false);
  status.permit_remaining_us()=(0U);
  status.evidence_lost()=(false);
  if (!historical_released_idle_baseline(status,context,status_source)) return 301;
  auto prior_motion=status;auto prior_motion_source=status_source;
  prior_motion.attempt_kind()=(1U);
  prior_motion.parent_run_id()=(0U);
  prior_motion.final_state()=(2U);
  if (!historical_released_idle_baseline(prior_motion,context,prior_motion_source)) return 302;
  auto rejected=status;auto rejected_source=status_source;
  rejected.final_state()=(2U);
  if (historical_released_idle_baseline(rejected,context,rejected_source)) return 303;
  rejected=status;rejected_source=status_source;
  rejected.parent_run_id()=(0U);
  if (historical_released_idle_baseline(rejected,context,rejected_source)) return 304;
  rejected.parent_run_id()=(212U);
  if (historical_released_idle_baseline(rejected,context,rejected_source)) return 305;
  rejected=status;rejected_source=status_source;
  std::uint8_t nonzero_chain[32]{};
  nonzero_chain[0] = 1U;
  rejected.evidence_chain_sha256().emplace().assign(nonzero_chain,32);
  if (!historical_released_idle_baseline(rejected,context,rejected_source)) return 306;
  context.pages_written = 1U;
  if (historical_released_idle_baseline(rejected,context,rejected_source)) return 318;
  context.pages_written = 0U;
  rejected=status;rejected_source=status_source;
  rejected_source.boot_id=(0U);
  if (historical_released_idle_baseline(rejected,context,rejected_source)) return 307;
  rejected=status;rejected_source=status_source;
  rejected.session_id()=(8U);
  if (historical_released_idle_baseline(rejected,context,rejected_source)) return 308;
  rejected=status;rejected_source=status_source;
  rejected.permit_remaining_us()=(1U);
  if (historical_released_idle_baseline(rejected,context,rejected_source)) return 309;
  rejected=status;rejected_source=status_source;
  rejected.evidence_lost()=(true);
  if (historical_released_idle_baseline(rejected,context,rejected_source)) return 310;
  rejected=status;rejected_source=status_source;
  rejected.core_state()=(4U);
  if (historical_released_idle_baseline(rejected,context,rejected_source)) return 311;
  context.current_boot = 7U;
  if (historical_released_idle_baseline(status,context,status_source)) return 312;
  context.current_boot = 0U;
  context.pages_written = 1U;
  if (historical_released_idle_baseline(status,context,status_source)) return 313;
  context.pages_written = 0U;
  context.historical_run_boot = 7U;
  context.historical_run_id = 212U;
  if (!historical_released_idle_baseline(status,context,status_source)) return 314;
  rejected=status;rejected_source=status_source;
  rejected.evidence_chain_sha256().emplace().assign(nonzero_chain,32);
  context.historical_chain[0] = 1U;
  if (!historical_released_idle_baseline(rejected,context,rejected_source)) return 319;
  if (historical_released_idle_baseline(status,context,status_source)) return 320;
  context.pages_written = 1U;
  if (historical_released_idle_baseline(rejected,context,rejected_source)) return 321;
  context.pages_written = 0U;
  context.terminal_written_run_id = 211U;
  if (historical_released_idle_baseline(rejected,context,rejected_source)) return 322;
  context.terminal_written_run_id = 0U;
  context.historical_chain[0] = 0U;
  context.historical_run_boot = 8U;
  if (historical_released_idle_baseline(status,context,status_source)) return 315;
  context.historical_run_boot = 0U;
  context.historical_run_id = 0U;
  context.terminal_written_run_id = 211U;
  if (historical_released_idle_baseline(status,context,status_source)) return 316;
  context.terminal_written_run_id = 0U;
  context.historical_run_boot = 8U;
  if (historical_released_idle_baseline(status,context,status_source)) return 317;
  return 0;
}
}
