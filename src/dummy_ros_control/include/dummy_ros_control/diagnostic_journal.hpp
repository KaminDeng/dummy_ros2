#pragma once
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <type_traits>
#include <utility>

namespace dummy_ros_control {
// 仅ROS宿主私有诊断，不属于设备wire。scope是持有对象的完整生命周期；
// 不随boot/session/重连清除首拒。复位需销毁旧对象并在生产者停止后创建新对象。
enum class DiagnosticReason : std::uint16_t {
    kStateRejected=1, kManagementRejected, kStopUnconfirmed, kCommandRejected,
    kGoalRejected, kGoalValidation, kGoalNotReady, kGoalBusy,
    kBootstrapRejected, kGateFault, kObservationRejected, kEvidenceFailure,
    kStateIdentity, kStateAuthority, kStateSequence, kStateClock, kStateAge,
    kStateCodec, kStateStopLatched, kStateIngest
};
struct DiagnosticIdentity {
    std::uint64_t boot=0U, session=0U;
    std::uint32_t reference=0U, motion=0U, sequence=0U, management=0U;
    bool valid=false;
};
struct DiagnosticRejection {
    DiagnosticReason reason=DiagnosticReason::kStateRejected;
    DiagnosticIdentity observed{}, expected{};
    std::uint64_t host_us=0U;
    std::uint32_t detail_code=0U;
    std::array<char,96> detail{};
    bool detail_truncated=false;
};
struct DiagnosticStopPolicy {
    bool start_known=false;
    std::uint64_t started_us=0U,confirmed_us=0U,deadline_us=0U,timeout_us=0U;
    std::uint64_t request_id=0U,operation_id=0U;
    DiagnosticIdentity identity{};
    std::uint32_t profile=0U;
    std::uint8_t mask=0U;
    std::uint64_t status_run=0U;
    std::uint32_t status_motion=0U,status_management=0U;
};
struct DiagnosticReleaseIdle {
    std::uint64_t request_id=0U,operation_id=0U,run_id=0U,sent_us=0U;
    std::uint64_t state_received_us=0U,status_received_us=0U,commit_processed_us=0U,revalidated_us=0U;
};
struct DiagnosticSnapshot {
    static constexpr std::uint32_t kObjectLifetimeScope=1U;
    std::uint32_t scope=kObjectLifetimeScope;
    bool first_present=false;
    DiagnosticRejection first{};
    bool stop_policy_present=false;
    DiagnosticStopPolicy stop_policy{};
    std::uint64_t stop_policy_count=0U;
    bool release_idle_present=false;
    DiagnosticReleaseIdle release_idle{};
    std::uint64_t release_idle_count=0U;
    std::uint64_t rejections=0U, first_suppressed=0U;
    std::uint64_t detail_suppressed=0U, detail_dropped=0U, detail_attempted=0U;
    bool blob_present=false;
    std::uint64_t blob_dropped=0U;
};
struct DiagnosticArgument {
    enum class Kind : std::uint8_t { kUnsigned, kSigned, kReal, kText };
    Kind kind=Kind::kUnsigned;
    std::uint64_t bits=0U;
    std::array<char,48> text{};
    bool text_truncated=false;
};
struct DiagnosticBlob {
    std::array<char,96> label{};
    std::array<std::uint8_t,384> payload{};
    std::size_t payload_size=0U;
    bool valid=false;
};
struct DiagnosticDetail {
    std::array<char,192> format{};
    bool format_truncated=false;
    std::array<DiagnosticArgument,64> arguments{};
    std::size_t argument_count=0U;
    bool arguments_truncated=false;
    std::array<std::uint8_t,384> payload{};
    std::size_t payload_size=0U;
};
class DiagnosticJournal final {
 public:
    static constexpr std::size_t kDetailCapacity=8U;
    static constexpr std::size_t kReasonCount=21U; // 枚举0保留；每reason区分无身份/已绑定身份。

    static_assert(std::atomic<std::uint64_t>::is_always_lock_free,
                  "宿主诊断计数必须原生无锁");
    DiagnosticJournal() noexcept {
        for(auto& ready:detail_ready_)ready.store(false,std::memory_order_relaxed);
        for(auto& field:context_)field.store(0U,std::memory_order_relaxed);
        for(auto& slot:reason_first_)slot.state.store(0U,std::memory_order_relaxed);
    }
    void record_release_idle(const DiagnosticReleaseIdle& value) noexcept {
        release_idle_count_.fetch_add(1U,std::memory_order_relaxed);
        unsigned empty=0U;
        if(!release_idle_state_.compare_exchange_strong(empty,1U,std::memory_order_acq_rel))return;
        release_idle_=value; release_idle_state_.store(2U,std::memory_order_release);
    }
    void record_stop_policy(const DiagnosticStopPolicy& value) noexcept {
        stop_count_.fetch_add(1U,std::memory_order_relaxed);
        unsigned empty=0U;
        if(!stop_state_.compare_exchange_strong(empty,1U,std::memory_order_acq_rel))return;
        stop_policy_=value; stop_state_.store(2U,std::memory_order_release);
    }
    void enable_details(bool value) noexcept { enabled_.store(value,std::memory_order_relaxed); }
    void reject(DiagnosticReason reason,DiagnosticIdentity observed,DiagnosticIdentity expected,
                std::uint64_t host_us,const char* detail,std::uint32_t code=0U) noexcept {
        rejections_.fetch_add(1U,std::memory_order_relaxed);
        unsigned empty=0U;
        if(first_state_.compare_exchange_strong(empty,1U,std::memory_order_acq_rel)) {
            first_.reason=reason; first_.observed=observed; first_.expected=expected;
            first_.host_us=host_us; first_.detail_code=code;
            first_.detail_truncated=copy_text(first_.detail,detail);
            first_state_.store(2U,std::memory_order_release);
        } else first_suppressed_.fetch_add(1U,std::memory_order_relaxed);
        const auto reason_index=static_cast<std::size_t>(reason);
        const bool bound=expected.valid && expected.boot!=0U;
        if(reason_index<kReasonCount) {
            auto& slot=reason_first_[reason_index+(bound?kReasonCount:0U)];
            unsigned empty_slot=0U;
            if(slot.state.compare_exchange_strong(empty_slot,1U,std::memory_order_acq_rel)) {
                slot.value.reason=reason; slot.value.observed=observed; slot.value.expected=expected;
                slot.value.host_us=host_us; slot.value.detail_code=code;
                slot.value.detail_truncated=copy_text(slot.value.detail,detail);
                slot.state.store(2U,std::memory_order_release);
            }
        }
    }
    std::optional<DiagnosticRejection> first_record(DiagnosticReason reason,bool bound_identity) const noexcept {
        const auto index=static_cast<std::size_t>(reason)+(bound_identity?kReasonCount:0U);
        if(index>=reason_first_.size() || reason_first_[index].state.load(std::memory_order_acquire)!=2U)return {};
        return reason_first_[index].value;
    }
    void reject_from_context(DiagnosticReason reason,std::uint64_t host_us,
                             const char* detail,std::uint32_t code=0U) noexcept {
        const auto identities=context(); reject(reason,identities.first,identities.second,host_us,detail,code);
    }
    template<class... Args> void detail(const char* format,Args... args) noexcept {
        auto* record=reserve_detail(); if(!record)return;
        record->format_truncated=copy_text(record->format,format);
        (append_argument(*record,args),...);
        detail_ready_[static_cast<std::size_t>(record-details_.data())].store(true,std::memory_order_release);
    }
    // 必要safeprime原页独立于可选detail；首槽保完整原长度，后续/超长明确计数。
    void blob(const char* label,const std::uint8_t* bytes,std::size_t size) noexcept {
        unsigned empty=0U;
        if(!blob_state_.compare_exchange_strong(empty,1U,std::memory_order_acq_rel)) {
            blob_dropped_.fetch_add(1U,std::memory_order_relaxed); return;
        }
        copy_text(blob_.label,label); blob_.payload_size=size;
        blob_.valid=bytes && size<=blob_.payload.size();
        if(blob_.valid)std::memcpy(blob_.payload.data(),bytes,size);
        else blob_dropped_.fetch_add(1U,std::memory_order_relaxed);
        blob_state_.store(2U,std::memory_order_release);
    }
    std::optional<DiagnosticBlob> blob_record() const noexcept {
        if(blob_state_.load(std::memory_order_acquire)!=2U)return {};
        return blob_;
    }
    DiagnosticSnapshot snapshot() const noexcept {
        DiagnosticSnapshot output{};
        output.release_idle_count=release_idle_count_.load(std::memory_order_relaxed);
        output.release_idle_present=release_idle_state_.load(std::memory_order_acquire)==2U;
        if(output.release_idle_present)output.release_idle=release_idle_;
        output.stop_policy_count=stop_count_.load(std::memory_order_relaxed);
        output.stop_policy_present=stop_state_.load(std::memory_order_acquire)==2U;
        if(output.stop_policy_present)output.stop_policy=stop_policy_;
        output.first_present=first_state_.load(std::memory_order_acquire)==2U;
        if(output.first_present) output.first=first_;
        output.rejections=rejections_.load(std::memory_order_relaxed);
        output.first_suppressed=first_suppressed_.load(std::memory_order_relaxed);
        output.detail_suppressed=detail_suppressed_.load(std::memory_order_relaxed);
        output.detail_dropped=detail_dropped_.load(std::memory_order_relaxed);
        output.detail_attempted=detail_next_.load(std::memory_order_relaxed);
        output.blob_present=blob_state_.load(std::memory_order_acquire)==2U;
        output.blob_dropped=blob_dropped_.load(std::memory_order_relaxed);
        return output;
    }
    std::optional<DiagnosticDetail> detail_record(std::size_t index) const noexcept {
        if(index>=kDetailCapacity || !detail_ready_[index].load(std::memory_order_acquire))return {};
        return details_[index]; // append-only槽发布后不修改，reader不阻塞producer。
    }
    // 单control/worker writer发布identity；异步goal callback只尝试读取一次，
    // 读到正在更新的context则明确valid=false，不忙等、不读非原子共享字段。
    void publish_context(DiagnosticIdentity observed,DiagnosticIdentity expected) noexcept {
        const auto version=context_version_.load(std::memory_order_seq_cst);
        context_version_.store(version+1U,std::memory_order_seq_cst);
        store_identity(0U,observed); store_identity(7U,expected);
        context_version_.store(version+2U,std::memory_order_seq_cst);
    }
    std::pair<DiagnosticIdentity,DiagnosticIdentity> context() const noexcept {
        const auto before=context_version_.load(std::memory_order_seq_cst);
        if(before==0U || (before&1U))return {};
        const auto observed=load_identity(0U), expected=load_identity(7U);
        if(before!=context_version_.load(std::memory_order_seq_cst))return {};
        return {observed,expected};
    }
 private:
    template<std::size_t N> static bool copy_text(std::array<char,N>& output,const char* input) noexcept {
        std::size_t i=0U;
        if(input)for(;i+1U<N && input[i];++i)output[i]=input[i];
        output[i]='\0';
        return input && input[i]!='\0';
    }
    DiagnosticDetail* reserve_detail() noexcept {
        if(!enabled_.load(std::memory_order_relaxed)) {
            detail_suppressed_.fetch_add(1U,std::memory_order_relaxed); return nullptr;
        }
        const auto index=detail_next_.fetch_add(1U,std::memory_order_relaxed);
        if(index>=kDetailCapacity) { detail_dropped_.fetch_add(1U,std::memory_order_relaxed); return nullptr; }
        // 每个调用返回自己独占的槽；不复用，不依赖消费者进度。
        return &details_[static_cast<std::size_t>(index)];
    }
    template<class T> static void append_argument(DiagnosticDetail& record,T value) noexcept {
        if(record.argument_count>=record.arguments.size()) {record.arguments_truncated=true;return;}
        auto& output=record.arguments[record.argument_count++];
        if constexpr(std::is_convertible_v<T,const char*>) {
            output.kind=DiagnosticArgument::Kind::kText; output.text_truncated=copy_text(output.text,value);
        } else if constexpr(std::is_floating_point_v<T>) {
            output.kind=DiagnosticArgument::Kind::kReal;
            const double real=static_cast<double>(value); std::memcpy(&output.bits,&real,sizeof(real));
        } else if constexpr(std::is_signed_v<T>) {
            output.kind=DiagnosticArgument::Kind::kSigned; output.bits=static_cast<std::uint64_t>(value);
        } else {
            output.kind=DiagnosticArgument::Kind::kUnsigned; output.bits=static_cast<std::uint64_t>(value);
        }
    }
    void store_identity(std::size_t at,DiagnosticIdentity id) noexcept {
        const std::uint64_t values[]={id.boot,id.session,id.reference,id.motion,id.sequence,id.management,id.valid?1U:0U};
        for(std::size_t j=0U;j<7U;++j)context_[at+j].store(values[j],std::memory_order_seq_cst);
    }
    DiagnosticIdentity load_identity(std::size_t at) const noexcept {
        return {context_[at].load(std::memory_order_seq_cst),context_[at+1U].load(std::memory_order_seq_cst),
            static_cast<std::uint32_t>(context_[at+2U].load(std::memory_order_seq_cst)),
            static_cast<std::uint32_t>(context_[at+3U].load(std::memory_order_seq_cst)),
            static_cast<std::uint32_t>(context_[at+4U].load(std::memory_order_seq_cst)),
            static_cast<std::uint32_t>(context_[at+5U].load(std::memory_order_seq_cst)),
            context_[at+6U].load(std::memory_order_seq_cst)!=0U};
    }
    struct FirstSlot {std::atomic<unsigned> state{0U}; DiagnosticRejection value{};};
    std::array<FirstSlot,2U*kReasonCount> reason_first_{};
    std::atomic<unsigned> release_idle_state_{0U};
    DiagnosticReleaseIdle release_idle_{};
    std::atomic<std::uint64_t> release_idle_count_{0U};
    std::atomic<unsigned> stop_state_{0U};
    DiagnosticStopPolicy stop_policy_{};
    std::atomic<std::uint64_t> stop_count_{0U};
    std::atomic<unsigned> blob_state_{0U};
    DiagnosticBlob blob_{};
    std::atomic<std::uint64_t> blob_dropped_{0U};
    std::atomic<unsigned> first_state_{0U};
    DiagnosticRejection first_{};
    std::atomic<std::uint64_t> rejections_{0U},first_suppressed_{0U};
    std::atomic<std::uint64_t> detail_suppressed_{0U},detail_dropped_{0U},detail_next_{0U};
    std::atomic<bool> enabled_{false};
    std::array<DiagnosticDetail,kDetailCapacity> details_{};
    std::array<std::atomic<bool>,kDetailCapacity> detail_ready_{};
    std::array<std::atomic<std::uint64_t>,14> context_{};
    std::atomic<std::uint64_t> context_version_{0U};
};
} // namespace dummy_ros_control
