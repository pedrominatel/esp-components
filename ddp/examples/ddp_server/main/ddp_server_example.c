/*
 * SPDX-FileCopyrightText: 2026 Pedro Minatel
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>

#include "ddp.h"
#include "esp_log.h"

static const char *TAG = "ddp_example";

static esp_err_t on_data(void *context, const ddp_packet_t *packet)
{
    (void)context;
    ESP_LOGI(TAG, "Data: source=0x%08lx id=%u offset=%lu length=%u",
             (unsigned long)packet->source_ipv4, packet->destination_id,
             (unsigned long)packet->offset, packet->data_length);
    return ESP_OK;
}

static esp_err_t on_push(void *context, const ddp_packet_t *packet)
{
    (void)context;
    ESP_LOGI(TAG, "Push: id=%u sequence=%u", packet->destination_id, packet->sequence);
    return ESP_OK;
}

static esp_err_t on_query(void *context, const ddp_packet_t *query,
                          uint8_t *response, size_t response_capacity,
                          size_t *response_length)
{
    (void)context;
    int length = snprintf((char *)response, response_capacity,
                          "{\"name\":\"ddp-server-example\",\"id\":%u}",
                          query->destination_id);
    if (length < 0 || (size_t)length >= response_capacity) {
        return ESP_ERR_INVALID_SIZE;
    }
    *response_length = (size_t)length;
    return ESP_OK;
}

void app_main(void)
{
    ddp_server_config_t config = DDP_SERVER_CONFIG_DEFAULT();
    ddp_callbacks_t callbacks = {
        .on_data = on_data,
        .on_push = on_push,
        .on_query = on_query,
    };
    ddp_server_handle_t server = NULL;
    ESP_ERROR_CHECK(ddp_server_start(&config, &callbacks, NULL, &server));
    ESP_LOGI(TAG, "DDP server listening on UDP port %u", config.port);
}
