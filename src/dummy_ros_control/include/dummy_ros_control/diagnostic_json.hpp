#pragma once
// 仅非控制读取端调用；worker/control只能发布DiagnosticJournal固定记录。
#include "dummy_ros_control/diagnostic_journal.hpp"
#include <sstream>
#include <string>
namespace dummy_ros_control {
inline void diagnostic_json_text(std::ostream& out,const char* text) {
    out << '"';
    if(text)for(const auto* p=text;*p;++p) {
        const auto c=static_cast<unsigned char>(*p);
        if(c=='"' || c=='\\')out << '\\' << static_cast<char>(c);
        else if(c<32U || c>=127U) {
            constexpr char hex[]="0123456789abcdef";
            out << "\\u00" << hex[c>>4U] << hex[c&15U];
        } else out << static_cast<char>(c);
    }
    out << '"';
}
inline void diagnostic_json_identity(std::ostream& out,const DiagnosticIdentity& id) {
    out << "{\"valid\":" << (id.valid?"true":"false") << ",\"boot\":" << id.boot
        << ",\"session\":" << id.session << ",\"reference_epoch\":" << id.reference
        << ",\"motion_epoch\":" << id.motion << ",\"sequence\":" << id.sequence
        << ",\"management_epoch\":" << id.management << '}';
}
inline void diagnostic_json_rejection(std::ostream& out,const DiagnosticRejection& record) {
    out << "{\"reason\":" << static_cast<unsigned>(record.reason) << ",\"host_us\":" << record.host_us
        << ",\"detail_code\":" << record.detail_code << ",\"detail\":";
    diagnostic_json_text(out,record.detail.data());
    out << ",\"detail_truncated\":" << (record.detail_truncated?"true":"false") << ",\"observed\":";
    diagnostic_json_identity(out,record.observed);out << ",\"expected\":";
    diagnostic_json_identity(out,record.expected);out << '}';
}
inline std::string diagnostic_journal_json(const DiagnosticJournal& journal) {
    const auto snapshot=journal.snapshot(); std::ostringstream out;
    out << "{\"schema\":\"DUMMY_ROS_DIAGNOSTICS_V1\",\"scope\":\"OBJECT_LIFETIME\",\"first\":";
    if(snapshot.first_present)diagnostic_json_rejection(out,snapshot.first);else out << "null";
    out << ",\"first_by_reason\":["; bool comma=false;
    for(unsigned domain=0U;domain<2U;++domain)for(std::size_t reason=1U;reason<DiagnosticJournal::kReasonCount;++reason) {
        const auto record=journal.first_record(static_cast<DiagnosticReason>(reason),domain!=0U);
        if(!record)continue;
        if(comma)out << ',';
        comma=true;
        out << "{\"domain\":\"" << (domain?"bound_identity":"pre_identity") << "\",\"record\":";
        diagnostic_json_rejection(out,*record);out << '}';
    }
    out << "],\"counts\":{\"rejections\":" << snapshot.rejections
        << ",\"first_suppressed\":" << snapshot.first_suppressed
        << ",\"detail_suppressed\":" << snapshot.detail_suppressed
        << ",\"detail_dropped\":" << snapshot.detail_dropped
        << ",\"detail_attempted\":" << snapshot.detail_attempted << "},\"stop_policy\":{\"count\":"
        << snapshot.stop_policy_count << ",\"record\":";
    if(snapshot.stop_policy_present) {
        const auto& p=snapshot.stop_policy;
        out << "{\"version\":1,\"clock\":\"HOST_TRANSPORT_STEADY_US\",\"start_known\":" << (p.start_known?1:0)
            << ",\"started_us\":" << p.started_us << ",\"confirmed_us\":" << p.confirmed_us
            << ",\"deadline_us\":" << p.deadline_us << ",\"timeout_us\":" << p.timeout_us
            << ",\"request_id\":" << p.request_id << ",\"operation_id\":" << p.operation_id
            << ",\"boot\":" << p.identity.boot << ",\"session\":" << p.identity.session
            << ",\"ref\":" << p.identity.reference << ",\"motion\":" << p.identity.motion
            << ",\"mgmt\":" << p.identity.management << ",\"profile\":" << p.profile
            << ",\"mask\":" << unsigned(p.mask) << ",\"status_run\":" << p.status_run
            << ",\"status_motion\":" << p.status_motion << ",\"status_mgmt\":" << p.status_management << '}';
    } else out << "null";
    out << "},\"release_idle_revalidation\":{\"count\":" << snapshot.release_idle_count << ",\"record\":";
    if(snapshot.release_idle_present) {
        const auto& p=snapshot.release_idle;
        out << "{\"version\":1,\"clock\":\"HOST_TRANSPORT_STEADY_US\",\"request_id\":" << p.request_id
            << ",\"operation_id\":" << p.operation_id << ",\"run_id\":" << p.run_id << ",\"sent_us\":" << p.sent_us
            << ",\"state_received_us\":" << p.state_received_us << ",\"status_received_us\":" << p.status_received_us
            << ",\"commit_processed_us\":" << p.commit_processed_us << ",\"revalidated_us\":" << p.revalidated_us << '}';
    } else out << "null";
    out << "},\"safe_prime_blob\":{\"dropped\":" << snapshot.blob_dropped << ",\"record\":";
    if(const auto blob=journal.blob_record()) {
        out << "{\"label\":";diagnostic_json_text(out,blob->label.data());
        out << ",\"scope\":\"LOCAL_PUBLISH_ARGUMENT\",\"valid\":" << (blob->valid?"true":"false")
            << ",\"payload_size\":" << blob->payload_size << ",\"payload_hex\":\"";
        if(blob->valid) {
            constexpr char hex[]="0123456789abcdef";
            for(std::size_t i=0U;i<blob->payload_size;++i)out << hex[blob->payload[i]>>4U] << hex[blob->payload[i]&15U];
        }
        out << "\"}";
    } else out << "null";
    out << "}}"; return out.str();
}
} // namespace dummy_ros_control
