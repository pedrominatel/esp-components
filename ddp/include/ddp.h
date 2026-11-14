/*
 * SPDX-FileCopyrightText: 2026 Pedro Minatel
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DDP_DEFAULT_PORT       4048U
#define DDP_MAX_PAYLOAD_LENGTH 1440U
#define DDP_DISPLAY_ID         1U
#define DDP_CONFIG_ID          250U
#define DDP_STATUS_ID          251U
#define DDP_ALL_ID             255U

#define DDP_TYPE_UNDEFINED 0x00U
#define DDP_TYPE_LEGACY_RGB 0x01U
#define DDP_TYPE_RGB24      0x0bU
#define DDP_TYPE_RGBW32     0x1bU

#define DDP_FLAG_VERSION_1 0x40U
#define DDP_FLAG_TIMECODE  0x10U
#define DDP_FLAG_STORAGE   0x08U
#define DDP_FLAG_REPLY     0x04U
#define DDP_FLAG_QUERY     0x02U
#define DDP_FLAG_PUSH      0x01U

typedef struct ddp_server *ddp_server_handle_t;

typedef enum {
    DDP_TRANSPORT_UDP = 1U << 0,
    DDP_TRANSPORT_TCP = 1U << 1,
} ddp_transport_t;

typedef struct {
    uint16_t port;
    uint8_t transports;
    uint8_t max_tcp_clients;
    uint16_t max_payload_length;
    uint16_t task_stack_size;
    UBaseType_t task_priority;
} ddp_server_config_t;

typedef struct {
    uint8_t flags;
    uint8_t sequence;
    uint8_t data_type;
    uint8_t destination_id;
    uint32_t offset;
    const uint8_t *data;
    uint16_t data_length;
    uint32_t source_ipv4;
    ddp_transport_t transport;
} ddp_packet_t;

typedef esp_err_t (*ddp_data_callback_t)(void *context, const ddp_packet_t *packet);
typedef esp_err_t (*ddp_push_callback_t)(void *context, const ddp_packet_t *packet);
typedef esp_err_t (*ddp_query_callback_t)(void *context, const ddp_packet_t *query,
                                          uint8_t *response, size_t response_capacity,
                                          size_t *response_length);

typedef struct {
    ddp_data_callback_t on_data;
    ddp_push_callback_t on_push;
    ddp_query_callback_t on_query;
} ddp_callbacks_t;

typedef struct {
    uint64_t received_packets;
    uint64_t data_packets;
    uint64_t push_packets;
    uint64_t query_packets;
    uint64_t reply_packets;
    uint64_t malformed_packets;
    uint64_t oversized_packets;
    uint64_t duplicate_packets;
    uint64_t sequence_gaps;
    uint64_t callback_errors;
    uint64_t tcp_connections;
    uint64_t tcp_disconnects;
} ddp_server_stats_t;

#define DDP_SERVER_CONFIG_DEFAULT() { \
    .port = DDP_DEFAULT_PORT, \
    .transports = DDP_TRANSPORT_UDP, \
    .max_tcp_clients = 2, \
    .max_payload_length = DDP_MAX_PAYLOAD_LENGTH, \
    .task_stack_size = 6144, \
    .task_priority = 6, \
}

esp_err_t ddp_server_start(const ddp_server_config_t *config,
                           const ddp_callbacks_t *callbacks,
                           void *context,
                           ddp_server_handle_t *out_handle);
esp_err_t ddp_server_stop(ddp_server_handle_t handle);
esp_err_t ddp_server_get_stats(ddp_server_handle_t handle, ddp_server_stats_t *stats);

#ifdef __cplusplus
}
#endif
