/**
 * @file raw_sequence_logging_tests.cpp
 * @brief 复用raw collector夹具检查序号/组数拒绝诊断。
 *
 * 通过main宏重命名包含collector测试源，避免复制造页逻辑；本文件自己的main只运行两个诊断负例，不自动运行被包含文件的完整main。
 */
#define main original_raw_collector_main
#include "raw_evidence_collector_tests.cpp"
#undef main
#include "dummy_ros_control/diagnostic_json.hpp"
/**
 * @brief 制造页序号跳跃和group_count不一致并核对首拒。
 *
 * 先接收seq0，再分别喂seq2或seq1但group_count=0；对应reason记录应保存预期序号，默认关闭详情时suppressed增加2。输出JSON用于人工/上层核对，不代表证据链已被成功采集。
 * @return 0 表示本文件全部检查通过；非零定位失败检查。
 */
int main() {
  dummy_ros_control::DeviceManagementChannel channel({});
  const auto trust=identity();
  /**
   * @brief 分别创建collector隔离页序列状态，但共享journal保留两种reason。
   * sequence=true故意跳到seq2；false使用seq1但终态group_count=0，与已接纳的1组矛盾。
   */
  for(bool sequence : {true,false}) {
    RawEvidenceCollector collector(trust,&channel);
    const auto first=page(3U,0U,genesis(trust,3U,11U),false);
    if(!collector.expect_run(3U) || !collector.offer_page(first,100U))return 1;
    const auto bad=page(3U,sequence?2U:1U,first.chain_sha256,true,sequence?1U:0U);
    if(collector.offer_page(bad,101U) || collector.failure()!=RawEvidenceCollector::Failure::kSequence)return 2;
  }
  using dummy_ros_control::DiagnosticReason;
  const auto seq=channel.diagnostics.first_record(DiagnosticReason::kRawSequence,false);
  const auto group=channel.diagnostics.first_record(DiagnosticReason::kRawGroupCount,false);
  if(!seq || !group || seq->fields[1]!=2U || seq->fields[2]!=1U || group->fields[3]!=0U)return 3;
  if(channel.diagnostics.snapshot().detail_suppressed!=2U)return 4;
  std::puts(dummy_ros_control::diagnostic_journal_json(channel.diagnostics).c_str());
}
