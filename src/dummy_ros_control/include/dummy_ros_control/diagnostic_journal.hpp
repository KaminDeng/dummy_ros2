/**
 * @file diagnostic_journal.hpp
 * @brief 用固定原子槽记录宿主对象生命周期首错与有限诊断详情。
 * 此结构仅ROS宿主私有，不改变设备wire；boot/session重连不清首拒。首错、每reason/身份域首条、8个details和首份blob分开发布，控制/worker端不做字符串流格式化或文件I/O；消费JSON在非控制侧完成。
 */
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
/**
 * @brief 稳定宿主拒绝原因枚举；0保留，不是设备fault码，JSON按reason解释fields。
 */
enum class DiagnosticReason : std::uint16_t {
    kStateRejected=1, kManagementRejected, kStopUnconfirmed, kCommandRejected,
    kGoalRejected, kGoalValidation, kGoalNotReady, kGoalBusy,
    kBootstrapRejected, kGateFault, kObservationRejected, kEvidenceFailure,
    kStateIdentity, kStateAuthority, kStateSequence, kStateClock, kStateAge,
    kStateCodec, kStateStopLatched, kStateIngest,
    kActionStop, kActionFail, kActionHoldDrainTimeout, kBootstrapReleaseTimeout, kBootstrapReleaseReply, kRawSequence, kRawGroupCount
};
/**
 * @brief boot/session为64位ID，reference/motion/sequence/management为32位水位；valid标记是否已取得身份而非授权成功。
 */
struct DiagnosticIdentity {
    std::uint64_t boot=0U, session=0U;
    std::uint32_t reference=0U, motion=0U, sequence=0U, management=0U;
    bool valid=false;
};
/**
 * @brief 首拒现场：observed/expected身份，host_us为宿主steady µs；detail固定96字节含终止符，fields为12个原因相关整数。
 */
struct DiagnosticRejection {
    DiagnosticReason reason=DiagnosticReason::kStateRejected;
    DiagnosticIdentity observed{}, expected{};
    std::uint64_t host_us=0U;
    std::uint32_t detail_code=0U;
    std::array<char,96> detail{};
    bool detail_truncated=false;
    std::array<std::uint64_t,12> fields{}; // 仅宿主私有首错现场；JSON按reason命名。
};
/**
 * @brief 一次停止确认策略现场：宿主started/confirmed/deadline/timeout均µs；request/operation/status run与设备身份独立绑定。
 */
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
/**
 * @brief 释放后Idle再验证的宿主时序：sent/state/status/commit_processed/revalidated全部µs，非设备生产时间。
 */
struct DiagnosticReleaseIdle {
    std::uint64_t request_id=0U,operation_id=0U,run_id=0U,sent_us=0U;
    std::uint64_t state_received_us=0U,status_received_us=0U,commit_processed_us=0U,revalidated_us=0U;
};
/**
 * @brief 首条记录有present标志，计数为对象寿命累计；快照各原子分开采样，并非整组瞬时事务。
 */
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
/**
 * @brief 类型化参数：整数或double位模式放bits，文本固定48字节并显式标记截断；不保存外部字符串指针。
 */
struct DiagnosticArgument {
    enum class Kind : std::uint8_t { kUnsigned, kSigned, kReal, kText };
    Kind kind=Kind::kUnsigned;
    std::uint64_t bits=0U;
    std::array<char,48> text{};
    bool text_truncated=false;
};
/**
 * @brief 必要首原载荷：标签96字节、载荷最多384字节，payload_size保原长度；valid=false不可把截断内容当完整原页。
 */
struct DiagnosticBlob {
    std::array<char,96> label{};
    std::array<std::uint8_t,384> payload{};
    std::size_t payload_size=0U;
    bool valid=false;
};
/**
 * @brief 可选详情：格式192字节、最多64个类型参数和384字节载荷；截断独立标记，消费者延后格式化。
 */
struct DiagnosticDetail {
    std::array<char,192> format{};
    bool format_truncated=false;
    std::array<DiagnosticArgument,64> arguments{};
    std::size_t argument_count=0U;
    bool arguments_truncated=false;
    std::array<std::uint8_t,384> payload{};
    std::size_t payload_size=0U;
};
/**
 * @brief 只追加固定槽的诊断容器，首错/详情发布后不再改其内容。
 */
class DiagnosticJournal final {
 public:
    static constexpr std::size_t kDetailCapacity=8U;
    static constexpr std::size_t kReasonCount=28U; // 枚举0保留；每reason区分无身份/已绑定身份。

    static_assert(std::atomic<std::uint64_t>::is_always_lock_free,
                  "宿主诊断计数必须原生无锁");
    /**
     * @brief 初始化详情ready和身份context原子槽，不启用可选详情。
     */
    DiagnosticJournal() noexcept {
        for(auto& ready:detail_ready_)ready.store(false,std::memory_order_relaxed);
        for(auto& field:context_)field.store(0U,std::memory_order_relaxed);
        for(auto& slot:reason_first_)slot.state.store(0U,std::memory_order_relaxed);
    }
    /**
     * @brief 累计释放Idle观察次数并只发布第一份现场。
     * 0→1 CAS抢唯一写槽，填完以2 release发布；后续只增加count，不覆盖首次记录。
     * @param value 已由调用方核对的宿主释放再验证记录。
     */
    void record_release_idle(const DiagnosticReleaseIdle& value) noexcept {
        release_idle_count_.fetch_add(1U,std::memory_order_relaxed);
        unsigned empty=0U;
        if(!release_idle_state_.compare_exchange_strong(empty,1U,std::memory_order_acq_rel))return;
        release_idle_=value; release_idle_state_.store(2U,std::memory_order_release);
    }
    /**
     * @brief 累计停止策略次数并保留首个策略现场。
     * @param value 停止管理流程的身份、时刻和预算值。
     */
    void record_stop_policy(const DiagnosticStopPolicy& value) noexcept {
        stop_count_.fetch_add(1U,std::memory_order_relaxed);
        unsigned empty=0U;
        if(!stop_state_.compare_exchange_strong(empty,1U,std::memory_order_acq_rel))return;
        stop_policy_=value; stop_state_.store(2U,std::memory_order_release);
    }
    /**
     * @brief 控制未来可选detail记录是否允许占槽。
     * 不清已经发布的首错、详情或blob。
     * @param value True开启；False只累计suppressed。
     */
    void enable_details(bool value) noexcept { enabled_.store(value,std::memory_order_relaxed); }
    /**
     * @brief 保存总首拒及按reason/身份绑定域的首现场。
     * 并发争抢首槽不等待；首记录写完后state=2才可读取。此方法不抛异常、不格式化printf，也不触发设备Stop。
     * @param reason 宿主原因枚举。
     * @param observed 实际观测身份。
     * @param expected 期望身份，valid且boot非零归bound域。
     * @param host_us 宿主steady µs。
     * @param detail 调用期间有效C文本，拷贝到固定96字节。
     * @param code 原因相关整数子码。
     * @param fields 12个原因相关数值现场。
     * @param is_rejection False时不增加总拒绝/总首拒，但仍可发布该reason域首现场。
     */
    void reject(DiagnosticReason reason,DiagnosticIdentity observed,DiagnosticIdentity expected,
                std::uint64_t host_us,const char* detail,std::uint32_t code=0U,
                std::array<std::uint64_t,12> fields={},bool is_rejection=true) noexcept {
        if(is_rejection) {
        rejections_.fetch_add(1U,std::memory_order_relaxed);
        unsigned empty=0U;
        if(first_state_.compare_exchange_strong(empty,1U,std::memory_order_acq_rel)) {
            first_.reason=reason; first_.observed=observed; first_.expected=expected;
            first_.host_us=host_us; first_.detail_code=code; first_.fields=fields;
            first_.detail_truncated=copy_text(first_.detail,detail);
            first_state_.store(2U,std::memory_order_release);
        } else first_suppressed_.fetch_add(1U,std::memory_order_relaxed);
        }
        const auto reason_index=static_cast<std::size_t>(reason);
        const bool bound=expected.valid && expected.boot!=0U;
        if(reason_index<kReasonCount) {
            auto& slot=reason_first_[reason_index+(bound?kReasonCount:0U)];
            unsigned empty_slot=0U;
            if(slot.state.compare_exchange_strong(empty_slot,1U,std::memory_order_acq_rel)) {
                slot.value.reason=reason; slot.value.observed=observed; slot.value.expected=expected;
                slot.value.host_us=host_us; slot.value.detail_code=code; slot.value.fields=fields;
                slot.value.detail_truncated=copy_text(slot.value.detail,detail);
                slot.state.store(2U,std::memory_order_release);
            }
        }
    }
    /**
     * @brief 读取指定reason和身份域首条。
     * @param reason 宿主拒绝原因。
     * @param bound_identity True读取有绑定身份域，False读取预身份域。
     * @return optional值；槽未发布或索引无效则空。
     */
    std::optional<DiagnosticRejection> first_record(DiagnosticReason reason,bool bound_identity) const noexcept {
        const auto index=static_cast<std::size_t>(reason)+(bound_identity?kReasonCount:0U);
        if(index>=reason_first_.size() || reason_first_[index].state.load(std::memory_order_acquire)!=2U)return {};
        return reason_first_[index].value;
    }
    /**
     * @brief 以当前一致context身份记录拒绝。
     * context正在更新时得到valid=false身份，不忙等追求最新。
     * @param reason 原因。
     * @param host_us 宿主steady µs。
     * @param detail 固定拷贝文本。
     * @param code 可选子码。
     */
    void reject_from_context(DiagnosticReason reason,std::uint64_t host_us,
                             const char* detail,std::uint32_t code=0U) noexcept {
        const auto identities=context(); reject(reason,identities.first,identities.second,host_us,detail,code);
    }
    /**
     * @brief 预留一个独占详情槽并复制格式和类型参数。
     * 开关关闭计suppressed；超过8槽计dropped；不复用消费者已读槽，生命周期内容量固定。
     * @param format C格式文本，最多191个字符加终止符。
     * @param args 值参数包；数值按位保存，C文本立即复制。
     */
    template<class... Args> void detail(const char* format,Args... args) noexcept {
        auto* record=reserve_detail(); if(!record)return;
        record->format_truncated=copy_text(record->format,format);
        (append_argument(*record,args),...);
        detail_ready_[static_cast<std::size_t>(record-details_.data())].store(true,std::memory_order_release);
    }
    // 必要safeprime原页独立于可选detail；首槽保完整原长度，后续/超长明确计数。
    /**
     * @brief 保存首份必要参数原载荷，与可选detail开关独立。
     * 后续blob不覆盖首槽；该载荷通常是本地publish参数，不代表传输或设备接收。
     * @param label 载荷标签C文本。
     * @param bytes 借用原始字节，调用期有效。
     * @param size 原始长度，超过384或空指针则valid=false且计dropped。
     */
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
    /**
     * @brief 读取已发布的首blob，包括无效长度标志。
     * @return optional副本；未发布返回空。
     */
    std::optional<DiagnosticBlob> blob_record() const noexcept {
        if(blob_state_.load(std::memory_order_acquire)!=2U)return {};
        return blob_;
    }
    /**
     * @brief 读取首错、停止/释放首现场和累积计数。
     * @return 快照值；仅当对应state=2才复制非原子内容。
     */
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
    /**
     * @brief 读取一个已release发布的追加详情槽。
     * @param index 0..7槽索引。
     * @return optional副本；越界/未ready为空。
     */
    std::optional<DiagnosticDetail> detail_record(std::size_t index) const noexcept {
        if(index>=kDetailCapacity || !detail_ready_[index].load(std::memory_order_acquire))return {};
        return details_[index]; // append-only槽发布后不修改，reader不阻塞producer。
    }
    // 单control/worker writer发布identity；异步goal callback只尝试读取一次，
    // 读到正在更新的context则明确valid=false，不忙等、不读非原子共享字段。
    /**
     * @brief 由单个control/worker writer发布观测及期望身份。
     * 版本先奇后偶，14个字段原子顺序一致存取；调用方保证单writer，不能推广为任意多writer seqlock。
     * @param observed 最近实际身份。
     * @param expected 当前预期绑定身份。
     */
    void publish_context(DiagnosticIdentity observed,DiagnosticIdentity expected) noexcept {
        const auto version=context_version_.load(std::memory_order_seq_cst);
        context_version_.store(version+1U,std::memory_order_seq_cst);
        store_identity(0U,observed); store_identity(7U,expected);
        context_version_.store(version+2U,std::memory_order_seq_cst);
    }
    /**
     * @brief 只尝试一次读取首尾版本一致的身份对。
     * @return observed/expected；未建立、更新中或跨版本时返回默认valid=false。
     */
    std::pair<DiagnosticIdentity,DiagnosticIdentity> context() const noexcept {
        const auto before=context_version_.load(std::memory_order_seq_cst);
        if(before==0U || (before&1U))return {};
        const auto observed=load_identity(0U), expected=load_identity(7U);
        if(before!=context_version_.load(std::memory_order_seq_cst))return {};
        return {observed,expected};
    }
 private:
    /**
     * @brief 有限复制C文本并始终写终止符。
     * @param output 固定数组，模板N须大于0。
     * @param input 可空C文本。
     * @return 原文本未完全容纳时true；空输入得到空文本。
     */
    template<std::size_t N> static bool copy_text(std::array<char,N>& output,const char* input) noexcept {
        std::size_t i=0U;
        if(input)for(;i+1U<N && input[i];++i)output[i]=input[i];
        output[i]='\0';
        return input && input[i]!='\0';
    }
    /**
     * @brief 按开关和递增索引分配不可复用详情槽。
     * @return 独占槽指针；关闭/耗尽为nullptr并计数。
     */
    DiagnosticDetail* reserve_detail() noexcept {
        if(!enabled_.load(std::memory_order_relaxed)) {
            detail_suppressed_.fetch_add(1U,std::memory_order_relaxed); return nullptr;
        }
        const auto index=detail_next_.fetch_add(1U,std::memory_order_relaxed);
        if(index>=kDetailCapacity) { detail_dropped_.fetch_add(1U,std::memory_order_relaxed); return nullptr; }
        // 每个调用返回自己独占的槽；不复用，不依赖消费者进度。
        return &details_[static_cast<std::size_t>(index)];
    }
    /**
     * @brief 按C文本/实数/有符号/无符号分类追加参数。
     * 超过64参数仅置arguments_truncated；double通过memcpy保存位模式，不进行流格式化。
     * @param record 本调用独占详情记录。
     * @param value 待复制值；不会保留字符串指针。
     */
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
    /**
     * @brief 向七个连续原子context字段写身份。
     * @param at 0或7的字段基址，由内部调用保证范围。
     * @param id 待保存身份值。
     */
    void store_identity(std::size_t at,DiagnosticIdentity id) noexcept {
        const std::uint64_t values[]={id.boot,id.session,id.reference,id.motion,id.sequence,id.management,id.valid?1U:0U};
        for(std::size_t j=0U;j<7U;++j)context_[at+j].store(values[j],std::memory_order_seq_cst);
    }
    /**
     * @brief 从七个原子字段恢复身份。
     * @param at 0或7基址。
     * @return DiagnosticIdentity值；外层版本检查决定是否一致。
     */
    DiagnosticIdentity load_identity(std::size_t at) const noexcept {
        return {context_[at].load(std::memory_order_seq_cst),context_[at+1U].load(std::memory_order_seq_cst),
            static_cast<std::uint32_t>(context_[at+2U].load(std::memory_order_seq_cst)),
            static_cast<std::uint32_t>(context_[at+3U].load(std::memory_order_seq_cst)),
            static_cast<std::uint32_t>(context_[at+4U].load(std::memory_order_seq_cst)),
            static_cast<std::uint32_t>(context_[at+5U].load(std::memory_order_seq_cst)),
            context_[at+6U].load(std::memory_order_seq_cst)!=0U};
    }
    /**
     * @brief state=0空、1写入中、2发布完成；非原子value只在发布后读取且不再更改。
     */
    struct FirstSlot {std::atomic<unsigned> state{0U}; DiagnosticRejection value{};};
    /**
     * @brief 按reason×预身份/绑定身份两个域划分的独占首槽；其余首现场各有独立发布状态。
     */
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
    /**
     * @brief 8个追加详情和14原子身份字段；context_version仅由指定单writer更新，读者不阻塞writer。
     */
    std::array<DiagnosticDetail,kDetailCapacity> details_{};
    std::array<std::atomic<bool>,kDetailCapacity> detail_ready_{};
    std::array<std::atomic<std::uint64_t>,14> context_{};
    std::atomic<std::uint64_t> context_version_{0U};
};
} // namespace dummy_ros_control
