// Auto-generated Dummy static requirements. Do not edit.
#pragma once
#define DP_DUMMY_RAW_FIXED_MAX_MESSAGE_BYTES 618U
#define DP_DUMMY_RAW_FIXED_SERVICE_SLOTS 1U
#define DP_DUMMY_RAW_FIXED_REQUEST_SLOTS 1U
#define DP_DUMMY_RAW_FIXED_RESPONSE_SLOTS 1U
#if defined(DP_COMM_RUNTIME_MAX_MESSAGE_BYTES) && DP_COMM_RUNTIME_MAX_MESSAGE_BYTES < DP_DUMMY_RAW_FIXED_MAX_MESSAGE_BYTES
#error "dp_comm message capacity is below Dummy raw_fixed requirement"
#endif
#if defined(DP_COMM_RUNTIME_MAX_SERVICES) && DP_COMM_RUNTIME_MAX_SERVICES < DP_DUMMY_RAW_FIXED_SERVICE_SLOTS
#error "dp_comm service capacity is below Dummy requirement"
#endif
#if defined(DP_COMM_RUNTIME_MAX_PENDING_REQUESTS) && DP_COMM_RUNTIME_MAX_PENDING_REQUESTS < DP_DUMMY_RAW_FIXED_REQUEST_SLOTS
#error "dp_comm request capacity is below Dummy requirement"
#endif
#if defined(DP_COMM_RUNTIME_MAX_COMPLETED_RESPONSES) && DP_COMM_RUNTIME_MAX_COMPLETED_RESPONSES < DP_DUMMY_RAW_FIXED_RESPONSE_SLOTS
#error "dp_comm response capacity is below Dummy requirement"
#endif
