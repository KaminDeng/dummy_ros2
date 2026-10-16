#define main original_raw_collector_main
#include "raw_evidence_collector_tests.cpp"
#undef main
#include "dummy_ros_control/diagnostic_json.hpp"
int main() {
  dummy_ros_control::DeviceManagementChannel channel({});
  const auto trust=identity();
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
