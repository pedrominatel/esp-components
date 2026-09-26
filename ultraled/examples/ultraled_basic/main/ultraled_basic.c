/*
 * SPDX-FileCopyrightText: 2026 Pedro Minatel
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdlib.h>

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ultraled.h"

static const char *TAG = "ultraled_basic";

void app_main(void)
{
    const ultraled_config_t config = {
        .led_model = ULTRALED_MODEL_WS2812B,
        .channel_count = CONFIG_ULTRALED_EXAMPLE_CHANNEL_COUNT,
        .channels = {
            { .gpio_num = CONFIG_ULTRALED_EXAMPLE_GPIO_0, .led_count = CONFIG_ULTRALED_EXAMPLE_LED_COUNT_0,
              .color_order = ULTRALED_COLOR_ORDER_GRB },
            { .gpio_num = CONFIG_ULTRALED_EXAMPLE_GPIO_1, .led_count = CONFIG_ULTRALED_EXAMPLE_LED_COUNT_1,
              .color_order = ULTRALED_COLOR_ORDER_BRG },
        },
    };

    ultraled_handle_t leds = NULL;
    ESP_ERROR_CHECK(ultraled_new(&config, &leds));
    ESP_ERROR_CHECK(ultraled_set_brightness(leds, 0, 96));
#if CONFIG_ULTRALED_EXAMPLE_CHANNEL_COUNT > 1
    ESP_ERROR_CHECK(ultraled_set_brightness(leds, 1, 48));
#endif

    ultraled_rgb_t *gradient = calloc(CONFIG_ULTRALED_EXAMPLE_LED_COUNT_0, sizeof(*gradient));
    ESP_ERROR_CHECK(gradient == NULL ? ESP_ERR_NO_MEM : ESP_OK);
    for (size_t i = 0; i < CONFIG_ULTRALED_EXAMPLE_LED_COUNT_0; ++i) {
        gradient[i] = (ultraled_rgb_t) {
            .red = (uint8_t)(255U * i / CONFIG_ULTRALED_EXAMPLE_LED_COUNT_0),
            .green = (uint8_t)(255U - 255U * i / CONFIG_ULTRALED_EXAMPLE_LED_COUNT_0),
            .blue = 24,
        };
    }
    ESP_ERROR_CHECK(ultraled_set_pixels(leds, 0, 0, gradient, CONFIG_ULTRALED_EXAMPLE_LED_COUNT_0));
    free(gradient);
#if CONFIG_ULTRALED_EXAMPLE_CHANNEL_COUNT > 1
    ESP_ERROR_CHECK(ultraled_fill(leds, 1, (ultraled_rgb_t) { .blue = 255 }));
#endif
    ESP_ERROR_CHECK(ultraled_show(leds, 1000));

#if CONFIG_ULTRALED_EXAMPLE_CHANNEL_COUNT > 1
    const size_t animation_channel = 1;
    const size_t animation_led_count = CONFIG_ULTRALED_EXAMPLE_LED_COUNT_1;
#else
    const size_t animation_channel = 0;
    const size_t animation_led_count = CONFIG_ULTRALED_EXAMPLE_LED_COUNT_0;
#endif
    size_t position = 0;
    while (true) {
        ESP_ERROR_CHECK(ultraled_clear(leds, animation_channel));
        ESP_ERROR_CHECK(ultraled_set_pixel(leds, animation_channel, position,
                                           (ultraled_rgb_t) { .green = 255 }));
        ESP_ERROR_CHECK(ultraled_show_mask(leds, (uint8_t)(1U << animation_channel), 1000));
        position = (position + 1) % animation_led_count;
        vTaskDelay(pdMS_TO_TICKS(80));
    }

    // If the animation loop is removed, blank and release the hardware with:
    ESP_ERROR_CHECK(ultraled_clear_all(leds));
    ESP_ERROR_CHECK(ultraled_show(leds, ULTRALED_WAIT_FOREVER));
    ESP_ERROR_CHECK(ultraled_del(leds));
    ESP_LOGI(TAG, "Ultraled stopped");
}
