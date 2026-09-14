/* Auto-generated from ROS IDL and locked catalog. Do not edit. */
#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define DP_PROTOCOL_LITE_PROTOCOL_MAJOR 2u
#define DP_PROTOCOL_LITE_PROTOCOL_MINOR 0u
#define DP_PROTOCOL_LITE_CATALOG_FINGERPRINT_SIZE 16u
#define DP_PROTOCOL_LITE_CAPABILITY_PAGE_BYTES 16u

#define DP_PROTOCOL_LITE_MAX_CAPABILITY_PAGES 8u

static const uint8_t dp_protocol_lite_catalog_fingerprint[] = {
    0xc3u, 0xf5u, 0x21u, 0xa5u, 0xf3u, 0x92u, 0x7fu, 0xb6u, 0x46u, 0x27u, 0xd3u, 0x4fu, 0xfcu, 0x24u, 0x90u, 0xc8u
};

#define DP_PROTOCOL_LITE_CAPABILITY_LOG_STREAM_V1_ID 1u
#define DP_PROTOCOL_LITE_CAPABILITY_LOG_REMOTE_LEVEL_V1_ID 2u
#define DP_PROTOCOL_LITE_CAPABILITY_CRASH_DUMP_PUBLISH_V1_ID 3u
#define DP_PROTOCOL_LITE_CAPABILITY_CRASH_FILE_TRANSFER_V1_ID 4u
#define DP_PROTOCOL_LITE_CAPABILITY_CRASH_ANNOUNCE_V1_ID 5u
#define DP_PROTOCOL_LITE_CAPABILITY_OFFLINE_LOG_FILE_TRANSFER_V1_ID 6u
#define DP_PROTOCOL_LITE_CAPABILITY_OFFLINE_LOG_FILE_TRANSFER_V2_ID 7u
#define DP_PROTOCOL_LITE_CAPABILITY_OFFLINE_LOG_MANIFEST_V2_ID 8u
#define DP_PROTOCOL_LITE_CAPABILITY_OFFLINE_LOG_PREVIEW_V1_ID 9u
#define DP_PROTOCOL_LITE_CAPABILITY_OFFLINE_LOG_COMMAND_V1_ID 10u
#define DP_PROTOCOL_LITE_CAPABILITY_OFFLINE_LOG_COMMAND_V2_ID 11u
#define DP_PROTOCOL_LITE_CAPABILITY_FILE_TRANSFER_SERVER_V1_ID 12u
#define DP_PROTOCOL_LITE_CAPABILITY_DEVICE_STATUS_V1_ID 13u
#define DP_PROTOCOL_LITE_CAPABILITY_DP_PROTOCOL_CAPABILITY_ANNOUNCE_V1_ID 14u
#define DP_PROTOCOL_LITE_CAPABILITY_DP_PROTOCOL_CAPABILITY_QUERY_V1_ID 15u
#define DP_PROTOCOL_LITE_CAPABILITY_CONTROL_JOINT_STATE_V1_ID 16u
#define DP_PROTOCOL_LITE_CAPABILITY_CONTROL_JOINT_COMMAND_V1_ID 17u
#define DP_PROTOCOL_LITE_CAPABILITY_LOG_STREAM_TINY_V1_ID 18u
#define DP_PROTOCOL_LITE_PRODUCT_DP_STM32F746_DISC_OFFLINE_LOG_SERVER_USB_ECM_ID 1u
#define DP_PROTOCOL_LITE_CODEC_RAW_FIXED_BIT (1u << 0u)
#define DP_PROTOCOL_LITE_CODEC_BOUNDED_TLV_BIT (1u << 1u)
#define DP_PROTOCOL_LITE_QOS_BEST_EFFORT_SENSOR_BIT (1u << 0u)
#define DP_PROTOCOL_LITE_QOS_RELIABLE_CONTROL_BIT (1u << 1u)

typedef enum {
    DP_PROTOCOL_QOS_RELIABILITY_BEST_EFFORT = 0,
    DP_PROTOCOL_QOS_RELIABILITY_RELIABLE = 1,
} dp_protocol_qos_reliability_t;

typedef enum {
    DP_PROTOCOL_QOS_HISTORY_KEEP_LAST = 0,
} dp_protocol_qos_history_t;

typedef enum {
    DP_PROTOCOL_QOS_DURABILITY_VOLATILE = 0,
} dp_protocol_qos_durability_t;

typedef enum {
    DP_PROTOCOL_QOS_BACKPRESSURE_DROP = 0,
    DP_PROTOCOL_QOS_BACKPRESSURE_BLOCK = 1,
} dp_protocol_qos_backpressure_t;

typedef struct {
    const char *name;
    uint16_t profile_bit;
    uint8_t reliability;
    uint8_t history;
    uint16_t depth;
    uint8_t durability;
    uint8_t priority;
    uint8_t backpressure;
    uint32_t deadline_ms;
} dp_protocol_qos_profile_descriptor_t;

static const dp_protocol_qos_profile_descriptor_t dp_protocol_lite_qos_profiles[] = {
    {"best_effort_sensor", DP_PROTOCOL_LITE_QOS_BEST_EFFORT_SENSOR_BIT, DP_PROTOCOL_QOS_RELIABILITY_BEST_EFFORT, DP_PROTOCOL_QOS_HISTORY_KEEP_LAST, 1u, DP_PROTOCOL_QOS_DURABILITY_VOLATILE, 0u, DP_PROTOCOL_QOS_BACKPRESSURE_DROP, 0u},
    {"reliable_control", DP_PROTOCOL_LITE_QOS_RELIABLE_CONTROL_BIT, DP_PROTOCOL_QOS_RELIABILITY_RELIABLE, DP_PROTOCOL_QOS_HISTORY_KEEP_LAST, 1u, DP_PROTOCOL_QOS_DURABILITY_VOLATILE, 10u, DP_PROTOCOL_QOS_BACKPRESSURE_BLOCK, 20u},
};

/* Row i contains the requested profile bits compatible with offered profile i. */
static const uint16_t dp_protocol_lite_qos_compatibility_matrix[] = {
    DP_PROTOCOL_LITE_QOS_BEST_EFFORT_SENSOR_BIT,
    DP_PROTOCOL_LITE_QOS_BEST_EFFORT_SENSOR_BIT | DP_PROTOCOL_LITE_QOS_RELIABLE_CONTROL_BIT,
};

typedef enum {
    DP_PROTOCOL_LITE_ROUTE_TELEMETRY = 1,
    DP_PROTOCOL_LITE_ROUTE_DIAGNOSTIC = 2,
    DP_PROTOCOL_LITE_ROUTE_CONTROL = 3,
    DP_PROTOCOL_LITE_ROUTE_BULK = 4,
} dp_protocol_lite_route_access_t;

typedef struct {
    uint32_t interface_id;
    uint16_t capability_id;
    dp_protocol_lite_route_access_t access;
    uint16_t encoded_size;
    uint16_t codec_profile_bit;
    uint16_t qos_bit;
} dp_protocol_lite_route_t;

static const dp_protocol_lite_route_t dp_protocol_lite_routes[] = {
    {4130u, 18u, DP_PROTOCOL_LITE_ROUTE_TELEMETRY, 87u, DP_PROTOCOL_LITE_CODEC_BOUNDED_TLV_BIT, DP_PROTOCOL_LITE_QOS_BEST_EFFORT_SENSOR_BIT},
    {4097u, 16u, DP_PROTOCOL_LITE_ROUTE_TELEMETRY, 16u, DP_PROTOCOL_LITE_CODEC_RAW_FIXED_BIT, DP_PROTOCOL_LITE_QOS_BEST_EFFORT_SENSOR_BIT},
    {4098u, 17u, DP_PROTOCOL_LITE_ROUTE_CONTROL, 8u, DP_PROTOCOL_LITE_CODEC_RAW_FIXED_BIT, DP_PROTOCOL_LITE_QOS_RELIABLE_CONTROL_BIT},
    {4112u, 15u, DP_PROTOCOL_LITE_ROUTE_DIAGNOSTIC, 44u, DP_PROTOCOL_LITE_CODEC_BOUNDED_TLV_BIT, DP_PROTOCOL_LITE_QOS_RELIABLE_CONTROL_BIT},
    {4113u, 15u, DP_PROTOCOL_LITE_ROUTE_DIAGNOSTIC, 72u, DP_PROTOCOL_LITE_CODEC_BOUNDED_TLV_BIT, DP_PROTOCOL_LITE_QOS_RELIABLE_CONTROL_BIT},
    {4114u, 12u, DP_PROTOCOL_LITE_ROUTE_BULK, 111u, DP_PROTOCOL_LITE_CODEC_BOUNDED_TLV_BIT, DP_PROTOCOL_LITE_QOS_RELIABLE_CONTROL_BIT},
    {4115u, 12u, DP_PROTOCOL_LITE_ROUTE_BULK, 64u, DP_PROTOCOL_LITE_CODEC_BOUNDED_TLV_BIT, DP_PROTOCOL_LITE_QOS_RELIABLE_CONTROL_BIT},
    {4116u, 12u, DP_PROTOCOL_LITE_ROUTE_BULK, 51u, DP_PROTOCOL_LITE_CODEC_BOUNDED_TLV_BIT, DP_PROTOCOL_LITE_QOS_RELIABLE_CONTROL_BIT},
    {4117u, 12u, DP_PROTOCOL_LITE_ROUTE_BULK, 124u, DP_PROTOCOL_LITE_CODEC_BOUNDED_TLV_BIT, DP_PROTOCOL_LITE_QOS_RELIABLE_CONTROL_BIT},
    {4118u, 12u, DP_PROTOCOL_LITE_ROUTE_BULK, 53u, DP_PROTOCOL_LITE_CODEC_BOUNDED_TLV_BIT, DP_PROTOCOL_LITE_QOS_RELIABLE_CONTROL_BIT},
    {4119u, 12u, DP_PROTOCOL_LITE_ROUTE_BULK, 42u, DP_PROTOCOL_LITE_CODEC_BOUNDED_TLV_BIT, DP_PROTOCOL_LITE_QOS_RELIABLE_CONTROL_BIT},
    {4120u, 12u, DP_PROTOCOL_LITE_ROUTE_BULK, 37u, DP_PROTOCOL_LITE_CODEC_BOUNDED_TLV_BIT, DP_PROTOCOL_LITE_QOS_RELIABLE_CONTROL_BIT},
    {4121u, 12u, DP_PROTOCOL_LITE_ROUTE_BULK, 42u, DP_PROTOCOL_LITE_CODEC_BOUNDED_TLV_BIT, DP_PROTOCOL_LITE_QOS_RELIABLE_CONTROL_BIT},
};

static inline const dp_protocol_lite_route_t *dp_protocol_lite_find_route(uint32_t interface_id) {
    size_t count = sizeof(dp_protocol_lite_routes) / sizeof(dp_protocol_lite_routes[0]);
    for (size_t i = 0u; i < count; ++i) {
        if (dp_protocol_lite_routes[i].interface_id == interface_id) return &dp_protocol_lite_routes[i];
    }
    return NULL;
}

static inline void dp_protocol_lite_put_le16(uint16_t value, uint8_t *output) {
    for (size_t i = 0u; i < 2u; ++i) output[i] = (uint8_t)(value >> (i * 8u));
}
static inline uint16_t dp_protocol_lite_get_le16(const uint8_t *input) {
    uint16_t value = 0u;
    for (size_t i = 0u; i < 2u; ++i) value |= (uint16_t)input[i] << (i * 8u);
    return value;
}

static inline void dp_protocol_lite_put_le32(uint32_t value, uint8_t *output) {
    for (size_t i = 0u; i < 4u; ++i) output[i] = (uint8_t)(value >> (i * 8u));
}
static inline uint32_t dp_protocol_lite_get_le32(const uint8_t *input) {
    uint32_t value = 0u;
    for (size_t i = 0u; i < 4u; ++i) value |= (uint32_t)input[i] << (i * 8u);
    return value;
}

#define DP_PROTOCOL_LITE_LINK_HELLO_V1_INTERFACE_ID 61441u
#define DP_PROTOCOL_LITE_LINK_HELLO_V1_ENCODED_SIZE 45u

typedef struct {
    uint8_t protocol_major;
    uint8_t protocol_minor;
    uint32_t product_id;
    uint32_t boot_counter;
    uint8_t device_id[16];
    uint8_t catalog_fingerprint[16];
    uint8_t security_profile;
    uint16_t max_payload_size;
} dp_protocol_lite_link_hello_v1_t;

static inline int dp_protocol_lite_link_hello_v1_encode(
    const dp_protocol_lite_link_hello_v1_t *message, uint8_t *output, size_t output_size) {
    if (message == NULL || output == NULL || output_size != DP_PROTOCOL_LITE_LINK_HELLO_V1_ENCODED_SIZE) return -1;
    output[0u] = (uint8_t)(message->protocol_major);
    output[1u] = (uint8_t)(message->protocol_minor);
    uint32_t bits_product_id = 0u;
    memcpy(&bits_product_id, &(message->product_id), 4u);
    dp_protocol_lite_put_le32(bits_product_id, &output[2u]);
    uint32_t bits_boot_counter = 0u;
    memcpy(&bits_boot_counter, &(message->boot_counter), 4u);
    dp_protocol_lite_put_le32(bits_boot_counter, &output[6u]);
    memcpy(&output[10u], message->device_id, 16u);
    memcpy(&output[26u], message->catalog_fingerprint, 16u);
    output[42u] = (uint8_t)(message->security_profile);
    uint16_t bits_max_payload_size = 0u;
    memcpy(&bits_max_payload_size, &(message->max_payload_size), 2u);
    dp_protocol_lite_put_le16(bits_max_payload_size, &output[43u]);
    return 0;
}

static inline int dp_protocol_lite_link_hello_v1_decode(
    const uint8_t *input, size_t input_size, dp_protocol_lite_link_hello_v1_t *message) {
    if (input == NULL || message == NULL || input_size != DP_PROTOCOL_LITE_LINK_HELLO_V1_ENCODED_SIZE) return -1;
    dp_protocol_lite_link_hello_v1_t decoded;
    memset(&decoded, 0, sizeof(decoded));
    decoded.protocol_major = (uint8_t)input[0u];
    decoded.protocol_minor = (uint8_t)input[1u];
    uint32_t bits_product_id = dp_protocol_lite_get_le32(&input[2u]);
    memcpy(&decoded.product_id, &bits_product_id, 4u);
    uint32_t bits_boot_counter = dp_protocol_lite_get_le32(&input[6u]);
    memcpy(&decoded.boot_counter, &bits_boot_counter, 4u);
    memcpy(decoded.device_id, &input[10u], 16u);
    memcpy(decoded.catalog_fingerprint, &input[26u], 16u);
    decoded.security_profile = (uint8_t)input[42u];
    uint16_t bits_max_payload_size = dp_protocol_lite_get_le16(&input[43u]);
    memcpy(&decoded.max_payload_size, &bits_max_payload_size, 2u);
    *message = decoded;
    return 0;
}

#define DP_PROTOCOL_LITE_CAPABILITY_BITMAP_V1_INTERFACE_ID 61442u
#define DP_PROTOCOL_LITE_CAPABILITY_BITMAP_V1_ENCODED_SIZE 41u

typedef struct {
    uint8_t catalog_fingerprint[16];
    uint16_t page_index;
    uint16_t page_count;
    uint8_t valid_bytes;
    uint8_t capability_bits[16];
    uint16_t codec_profile_bits;
    uint16_t qos_bits;
} dp_protocol_lite_capability_bitmap_v1_t;

static inline int dp_protocol_lite_capability_bitmap_v1_encode(
    const dp_protocol_lite_capability_bitmap_v1_t *message, uint8_t *output, size_t output_size) {
    if (message == NULL || output == NULL || output_size != DP_PROTOCOL_LITE_CAPABILITY_BITMAP_V1_ENCODED_SIZE) return -1;
    memcpy(&output[0u], message->catalog_fingerprint, 16u);
    uint16_t bits_page_index = 0u;
    memcpy(&bits_page_index, &(message->page_index), 2u);
    dp_protocol_lite_put_le16(bits_page_index, &output[16u]);
    uint16_t bits_page_count = 0u;
    memcpy(&bits_page_count, &(message->page_count), 2u);
    dp_protocol_lite_put_le16(bits_page_count, &output[18u]);
    output[20u] = (uint8_t)(message->valid_bytes);
    memcpy(&output[21u], message->capability_bits, 16u);
    uint16_t bits_codec_profile_bits = 0u;
    memcpy(&bits_codec_profile_bits, &(message->codec_profile_bits), 2u);
    dp_protocol_lite_put_le16(bits_codec_profile_bits, &output[37u]);
    uint16_t bits_qos_bits = 0u;
    memcpy(&bits_qos_bits, &(message->qos_bits), 2u);
    dp_protocol_lite_put_le16(bits_qos_bits, &output[39u]);
    return 0;
}

static inline int dp_protocol_lite_capability_bitmap_v1_decode(
    const uint8_t *input, size_t input_size, dp_protocol_lite_capability_bitmap_v1_t *message) {
    if (input == NULL || message == NULL || input_size != DP_PROTOCOL_LITE_CAPABILITY_BITMAP_V1_ENCODED_SIZE) return -1;
    dp_protocol_lite_capability_bitmap_v1_t decoded;
    memset(&decoded, 0, sizeof(decoded));
    memcpy(decoded.catalog_fingerprint, &input[0u], 16u);
    uint16_t bits_page_index = dp_protocol_lite_get_le16(&input[16u]);
    memcpy(&decoded.page_index, &bits_page_index, 2u);
    uint16_t bits_page_count = dp_protocol_lite_get_le16(&input[18u]);
    memcpy(&decoded.page_count, &bits_page_count, 2u);
    decoded.valid_bytes = (uint8_t)input[20u];
    memcpy(decoded.capability_bits, &input[21u], 16u);
    uint16_t bits_codec_profile_bits = dp_protocol_lite_get_le16(&input[37u]);
    memcpy(&decoded.codec_profile_bits, &bits_codec_profile_bits, 2u);
    uint16_t bits_qos_bits = dp_protocol_lite_get_le16(&input[39u]);
    memcpy(&decoded.qos_bits, &bits_qos_bits, 2u);
    *message = decoded;
    return 0;
}

#define DP_PROTOCOL_LITE_LINK_RESET_V1_INTERFACE_ID 61443u
#define DP_PROTOCOL_LITE_LINK_RESET_V1_ENCODED_SIZE 6u

typedef struct {
    uint32_t reset_generation;
    uint8_t reason;
    uint8_t mode;
} dp_protocol_lite_link_reset_v1_t;

static inline int dp_protocol_lite_link_reset_v1_encode(
    const dp_protocol_lite_link_reset_v1_t *message, uint8_t *output, size_t output_size) {
    if (message == NULL || output == NULL || output_size != DP_PROTOCOL_LITE_LINK_RESET_V1_ENCODED_SIZE) return -1;
    uint32_t bits_reset_generation = 0u;
    memcpy(&bits_reset_generation, &(message->reset_generation), 4u);
    dp_protocol_lite_put_le32(bits_reset_generation, &output[0u]);
    output[4u] = (uint8_t)(message->reason);
    output[5u] = (uint8_t)(message->mode);
    return 0;
}

static inline int dp_protocol_lite_link_reset_v1_decode(
    const uint8_t *input, size_t input_size, dp_protocol_lite_link_reset_v1_t *message) {
    if (input == NULL || message == NULL || input_size != DP_PROTOCOL_LITE_LINK_RESET_V1_ENCODED_SIZE) return -1;
    dp_protocol_lite_link_reset_v1_t decoded;
    memset(&decoded, 0, sizeof(decoded));
    uint32_t bits_reset_generation = dp_protocol_lite_get_le32(&input[0u]);
    memcpy(&decoded.reset_generation, &bits_reset_generation, 4u);
    decoded.reason = (uint8_t)input[4u];
    decoded.mode = (uint8_t)input[5u];
    *message = decoded;
    return 0;
}

#define DP_PROTOCOL_LITE_HEARTBEAT_V1_INTERFACE_ID 61444u
#define DP_PROTOCOL_LITE_HEARTBEAT_V1_ENCODED_SIZE 8u

typedef struct {
    uint32_t uptime_ms;
    uint32_t health_flags;
} dp_protocol_lite_heartbeat_v1_t;

static inline int dp_protocol_lite_heartbeat_v1_encode(
    const dp_protocol_lite_heartbeat_v1_t *message, uint8_t *output, size_t output_size) {
    if (message == NULL || output == NULL || output_size != DP_PROTOCOL_LITE_HEARTBEAT_V1_ENCODED_SIZE) return -1;
    uint32_t bits_uptime_ms = 0u;
    memcpy(&bits_uptime_ms, &(message->uptime_ms), 4u);
    dp_protocol_lite_put_le32(bits_uptime_ms, &output[0u]);
    uint32_t bits_health_flags = 0u;
    memcpy(&bits_health_flags, &(message->health_flags), 4u);
    dp_protocol_lite_put_le32(bits_health_flags, &output[4u]);
    return 0;
}

static inline int dp_protocol_lite_heartbeat_v1_decode(
    const uint8_t *input, size_t input_size, dp_protocol_lite_heartbeat_v1_t *message) {
    if (input == NULL || message == NULL || input_size != DP_PROTOCOL_LITE_HEARTBEAT_V1_ENCODED_SIZE) return -1;
    dp_protocol_lite_heartbeat_v1_t decoded;
    memset(&decoded, 0, sizeof(decoded));
    uint32_t bits_uptime_ms = dp_protocol_lite_get_le32(&input[0u]);
    memcpy(&decoded.uptime_ms, &bits_uptime_ms, 4u);
    uint32_t bits_health_flags = dp_protocol_lite_get_le32(&input[4u]);
    memcpy(&decoded.health_flags, &bits_health_flags, 4u);
    *message = decoded;
    return 0;
}

#define DP_PROTOCOL_LITE_KEY_ROTATE_V1_INTERFACE_ID 61445u
#define DP_PROTOCOL_LITE_KEY_ROTATE_V1_ENCODED_SIZE 13u

typedef struct {
    uint32_t key_epoch;
    uint8_t current_key_id;
    uint8_t next_key_id;
    uint8_t phase;
    uint16_t key_slot_id;
    uint32_t activate_after_ms;
} dp_protocol_lite_key_rotate_v1_t;

static inline int dp_protocol_lite_key_rotate_v1_encode(
    const dp_protocol_lite_key_rotate_v1_t *message, uint8_t *output, size_t output_size) {
    if (message == NULL || output == NULL || output_size != DP_PROTOCOL_LITE_KEY_ROTATE_V1_ENCODED_SIZE) return -1;
    uint32_t bits_key_epoch = 0u;
    memcpy(&bits_key_epoch, &(message->key_epoch), 4u);
    dp_protocol_lite_put_le32(bits_key_epoch, &output[0u]);
    output[4u] = (uint8_t)(message->current_key_id);
    output[5u] = (uint8_t)(message->next_key_id);
    output[6u] = (uint8_t)(message->phase);
    uint16_t bits_key_slot_id = 0u;
    memcpy(&bits_key_slot_id, &(message->key_slot_id), 2u);
    dp_protocol_lite_put_le16(bits_key_slot_id, &output[7u]);
    uint32_t bits_activate_after_ms = 0u;
    memcpy(&bits_activate_after_ms, &(message->activate_after_ms), 4u);
    dp_protocol_lite_put_le32(bits_activate_after_ms, &output[9u]);
    return 0;
}

static inline int dp_protocol_lite_key_rotate_v1_decode(
    const uint8_t *input, size_t input_size, dp_protocol_lite_key_rotate_v1_t *message) {
    if (input == NULL || message == NULL || input_size != DP_PROTOCOL_LITE_KEY_ROTATE_V1_ENCODED_SIZE) return -1;
    dp_protocol_lite_key_rotate_v1_t decoded;
    memset(&decoded, 0, sizeof(decoded));
    uint32_t bits_key_epoch = dp_protocol_lite_get_le32(&input[0u]);
    memcpy(&decoded.key_epoch, &bits_key_epoch, 4u);
    decoded.current_key_id = (uint8_t)input[4u];
    decoded.next_key_id = (uint8_t)input[5u];
    decoded.phase = (uint8_t)input[6u];
    uint16_t bits_key_slot_id = dp_protocol_lite_get_le16(&input[7u]);
    memcpy(&decoded.key_slot_id, &bits_key_slot_id, 2u);
    uint32_t bits_activate_after_ms = dp_protocol_lite_get_le32(&input[9u]);
    memcpy(&decoded.activate_after_ms, &bits_activate_after_ms, 4u);
    *message = decoded;
    return 0;
}
