/*
 * SPDX-FileCopyrightText: 2026 Pedro Minatel
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdint.h>

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ultraled.h"

#define SELECTIVE_CHANNELS 8
#define SELECTIVE_LEDS 8
#define SELECTIVE_INTERVAL_MS 20

static const char *TAG = "ultraled_selective";

static const gpio_num_t s_gpios[SELECTIVE_CHANNELS] = {
    GPIO_NUM_22, GPIO_NUM_2, GPIO_NUM_3, GPIO_NUM_4,
    GPIO_NUM_5, GPIO_NUM_6, GPIO_NUM_32, GPIO_NUM_21,
};

void app_main(void)
{
    ultraled_config_t config = {
        .led_model = ULTRALED_MODEL_WS2812,
        .channel_count = SELECTIVE_CHANNELS,
    };
    for (size_t channel = 0; channel < SELECTIVE_CHANNELS; ++channel) {
        config.channels[channel] = (ultraled_channel_config_t) {
            .gpio_num = s_gpios[channel],
            .led_count = SELECTIVE_LEDS,
            .color_order = ULTRALED_COLOR_ORDER_GRB,
        };
    }

    ultraled_handle_t leds = NULL;
    ESP_ERROR_CHECK(ultraled_new(&config, &leds));
    ESP_ERROR_CHECK(ultraled_clear_all(leds));
    ESP_ERROR_CHECK(ultraled_show(leds, ULTRALED_WAIT_FOREVER));

    for (size_t channel = 0; channel < SELECTIVE_CHANNELS; ++channel) {
        ultraled_rgb_t color = {
            .red = (uint8_t)(0x11U * (channel + 1U)),
            .green = (uint8_t)(0xffU - 0x11U * channel),
            .blue = (uint8_t)(0x80U ^ (1U << channel)),
        };
        ESP_ERROR_CHECK(ultraled_fill(leds, channel, color));
    }

    ESP_LOGI(TAG, "Cycling one selected lane every %d ms", SELECTIVE_INTERVAL_MS);
    size_t channel = 0;
    while (true) {
        ESP_ERROR_CHECK(ultraled_show_mask(leds, (uint8_t)(1U << channel), ULTRALED_WAIT_FOREVER));
        channel = (channel + 1U) % SELECTIVE_CHANNELS;
        vTaskDelay(pdMS_TO_TICKS(SELECTIVE_INTERVAL_MS));
    }
}
