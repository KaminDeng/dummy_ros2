// Auto-generated from ROS IDL and deployment metadata. Do not edit.
#pragma once

/* Static memory requirements for the ROS-like control path. */
#define DP_COMM_STATIC_RUNTIME_STORAGE_BYTES 15360U
#define DP_COMM_STATIC_EXCHANGE_STORAGE_BUDGET_BYTES 256U
#define DP_COMM_STATIC_NODE_SLOTS 1U
#define DP_COMM_STATIC_PUBLISHER_SLOTS 2U
#define DP_COMM_STATIC_SUBSCRIPTION_SLOTS 1U
#define DP_COMM_STATIC_TIMER_SLOTS 0U
#define DP_COMM_STATIC_EVENT_QUEUE_SLOTS 1U
#define DP_COMM_STATIC_SERVICE_SLOTS 0U
#define DP_COMM_STATIC_MIN_MESSAGE_BYTES 16U
#define DP_COMM_STATIC_MIN_TOPIC_BYTES 77U
#define DP_COMM_STATIC_STATE_CODEC_SCRATCH_BYTES 0U
#define DP_COMM_STATIC_COMMAND_CODEC_SCRATCH_BYTES 0U
#define DP_COMM_STATIC_ACK_CACHE_BYTES 0U
#define DP_COMM_STATIC_CAPABILITY_PAGE_BYTES 0U
#define DP_COMM_STATIC_HMAC_SCRATCH_BYTES 0U
#define DP_COMM_STATIC_TRANSFER_SLOT_BYTES 0U
#define DP_COMM_STATIC_DIAGNOSTIC_QUEUE_BYTES 14U
#define DP_COMM_STATIC_TOTAL_AUXILIARY_BYTES (256U + 14U)
#define DP_COMM_STATIC_CODEC_ALIGNMENT 8U
#define DP_COMM_STATIC_EXECUTOR_STACK_BYTES 20480U
#define DP_COMM_STATIC_CONTROL_STACK_BYTES 4096U

#ifdef __cplusplus
#include <cstddef>
namespace dp::protocol::zenbedded::detail {
template <typename... ExchangeT>
constexpr size_t static_exchange_storage_bytes() {
    return (sizeof(ExchangeT) + ... + 0U);
}
constexpr size_t checked_static_total_bytes(const size_t *values, size_t count) {
    size_t total = 0U;
    for (size_t index = 0U; index < count; ++index) {
        if (values[index] > SIZE_MAX - total) return SIZE_MAX;
        total += values[index];
    }
    return total;
}
}  // namespace dp::protocol::zenbedded::detail
#define DP_COMM_STATIC_EXCHANGE_STORAGE_BYTES(...) \
    (::dp::protocol::zenbedded::detail::static_exchange_storage_bytes<__VA_ARGS__>())
#endif
