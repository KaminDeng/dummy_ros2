/**
 * @file diagnostic_json.hpp
 * @brief 在非控制线程把固定诊断记录格式化为JSON。
 * 此层使用ostream/字符串分配，不应从实时update/read/write或通信回调调用。JSON标识OBJECT_LIFETIME和HOST_TRANSPORT_STEADY_US，首blob另标LOCAL_PUBLISH_ARGUMENT，防止将宿主诊断混作设备wire/总线原始证据。
 */
#pragma once
// 仅非控制读取端调用；worker/control只能发布DiagnosticJournal固定记录。
#include "dummy_ros_control/diagnostic_journal.hpp"
#include <sstream>
#include <string>
namespace dummy_ros_control {
/**
 * @brief 输出带引号的逐字节JSON文本。
 * 引号/反斜杠转义，非ASCII及控制字节写成逐字节JSON Unicode转义；不按UTF-8码点转换Unicode。
 * @param out 非实时输出流。
 * @param text 可空C文本。
 */
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
/**
 * @brief 输出身份字段及valid标志。
 * 字段ID/epoch不带角度或时间单位，valid不等同运动授权。
 * @param out 非实时输出流。
 * @param id 宿主观测或预期身份。
 */
inline void diagnostic_json_identity(std::ostream& out,const DiagnosticIdentity& id) {
    out << "{\"valid\":" << (id.valid?"true":"false") << ",\"boot\":" << id.boot
        << ",\"session\":" << id.session << ",\"reference_epoch\":" << id.reference
        << ",\"motion_epoch\":" << id.motion << ",\"sequence\":" << id.sequence
        << ",\"management_epoch\":" << id.management << '}';
}
/**
 * @brief 按reason为固定fields命名并输出拒绝现场。
 * 已知原因输出各自具名字段；未列举原因fields为空对象，保留数值reason、detail和observed/expected。
 * @param out 输出流。
 * @param record 已发布拒绝值副本。
 */
inline void diagnostic_json_rejection(std::ostream& out,const DiagnosticRejection& record) {
    out << "{\"reason\":" << static_cast<unsigned>(record.reason) << ",\"host_us\":" << record.host_us
        << ",\"detail_code\":" << record.detail_code << ",\"detail\":";
    diagnostic_json_text(out,record.detail.data());
    out << ",\"detail_truncated\":" << (record.detail_truncated?"true":"false") << ",\"observed\":";
    diagnostic_json_identity(out,record.observed);out << ",\"expected\":";
    diagnostic_json_identity(out,record.expected);
    const char* names[12]{};
    switch(record.reason) {
      case DiagnosticReason::kActionStop: names[0]="phase";names[1]="operation";names[2]="request";names[3]="now";break;
      case DiagnosticReason::kActionFail: names[0]="phase";names[1]="operation";names[2]="request";break;
      case DiagnosticReason::kActionHoldDrainTimeout:
        {const char* fields[]={"frozen","generation","local","processed","target","committed","state","epoch","submitted","sent"};
         for(unsigned i=0U;i<10U;++i)names[i]=fields[i];}break;
      case DiagnosticReason::kBootstrapReleaseTimeout:
        {const char* fields[]={"phase","request","session","ref","core","state_core","permit_us","idle","overdue_us"};
         for(unsigned i=0U;i<9U;++i)names[i]=fields[i];}break;
      case DiagnosticReason::kBootstrapReleaseReply:
        {const char* fields[]={"request","response","phase","result","valid","success","rejected","release_committed","opid"};
         for(unsigned i=0U;i<9U;++i)names[i]=fields[i];}break;
      case DiagnosticReason::kRawSequence:
        {const char* fields[]={"run","seq","expected_seq","group_count","accepted_pages","category"};
         for(unsigned i=0U;i<6U;++i)names[i]=fields[i];}break;
      case DiagnosticReason::kRawGroupCount:
        {const char* fields[]={"run","seq","expected_seq","group_count","accepted_pages","last_reported","category"};
         for(unsigned i=0U;i<7U;++i)names[i]=fields[i];}break;
      default:break;
    }
    out << ",\"fields\":{";
    for(unsigned i=0U;i<12U && names[i];++i) {if(i)out << ',';diagnostic_json_text(out,names[i]);out << ':' << record.fields[i];}
    out << "}}";
}
/**
 * @brief 读取journal已发布槽并生成宿主诊断文档。
 * 输出总首拒、两个身份域各reason首条、计数、首停止策略、释放Idle再验证和首blob；optional detail槽由其他接口单独读取。
 * @param journal 寿命覆盖本次读取的只读journal引用。
 * @return JSON字符串，不含实时磁盘写入。
 */
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
