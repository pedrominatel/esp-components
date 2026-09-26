/*
 * SPDX-FileCopyrightText: 2026 Pedro Minatel
 * SPDX-License-Identifier: Apache-2.0
 */

#include <assert.h>
#include <inttypes.h>
#include <stdlib.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ultraled.h"

#define PERF_CHANNELS 8
#define PERF_LEDS 480
#define PERF_SAMPLES 100
#define PERF_FRAMES 300
#define PERF_FRAME_PERIOD_US 33333
#define PERF_MEDIAN_LIMIT_US 2000
#define PERF_P99_LIMIT_US 3000

static const char *TAG = "ultraled_perf";
static const gpio_num_t s_gpios[PERF_CHANNELS] = {
    GPIO_NUM_22, GPIO_NUM_2, GPIO_NUM_3, GPIO_NUM_4,
    GPIO_NUM_5, GPIO_NUM_6, GPIO_NUM_32, GPIO_NUM_21,
};

static int compare_i64(const void *left, const void *right)
{
    int64_t a = *(const int64_t *)left;
    int64_t b = *(const int64_t *)right;
    return (a > b) - (a < b);
}

static ultraled_config_t make_config(size_t led_count)
{
    ultraled_config_t config = {
        .led_model = ULTRALED_MODEL_WS2812,
        .channel_count = PERF_CHANNELS,
    };
    for (size_t lane = 0; lane < PERF_CHANNELS; ++lane) {
        config.channels[lane] = (ultraled_channel_config_t) {
            .gpio_num = s_gpios[lane],
            .led_count = led_count,
            .color_order = ULTRALED_COLOR_ORDER_MODEL_DEFAULT,
        };
    }
    return config;
}

static void load_frame(ultraled_handle_t handle, ultraled_rgb_t *pixels, size_t led_count, uint32_t frame)
{
    for (size_t lane = 0; lane < PERF_CHANNELS; ++lane) {
        for (size_t pixel = 0; pixel < led_count; ++pixel) {
            pixels[pixel] = (ultraled_rgb_t) {
                .red = (uint8_t)(pixel + frame),
                .green = (uint8_t)(pixel * 3U + lane),
                .blue = (uint8_t)(frame * 7U + lane * 17U),
            };
        }
        ESP_ERROR_CHECK(ultraled_set_pixels(handle, lane, 0, pixels, led_count));
    }
}

static void benchmark_submit(ultraled_handle_t handle, const char *name)
{
    int64_t samples[PERF_SAMPLES];
    for (size_t iteration = 0; iteration < 10; ++iteration) {
        ESP_ERROR_CHECK(ultraled_show_async(handle));
        ESP_ERROR_CHECK(ultraled_wait_done(handle, ULTRALED_WAIT_FOREVER));
    }
    for (size_t iteration = 0; iteration < PERF_SAMPLES; ++iteration) {
        int64_t start = esp_timer_get_time();
        ESP_ERROR_CHECK(ultraled_show_async(handle));
        samples[iteration] = esp_timer_get_time() - start;
        ESP_ERROR_CHECK(ultraled_wait_done(handle, ULTRALED_WAIT_FOREVER));
    }
    qsort(samples, PERF_SAMPLES, sizeof(samples[0]), compare_i64);
    int64_t median = samples[PERF_SAMPLES / 2];
    int64_t p99 = samples[PERF_SAMPLES - 2];
    ESP_LOGI(TAG, "%s: median=%" PRId64 " us, p99=%" PRId64 " us", name, median, p99);
    assert(median < PERF_MEDIAN_LIMIT_US);
    assert(p99 < PERF_P99_LIMIT_US);
}

static void run_paced_frames(ultraled_handle_t handle, ultraled_rgb_t *pixels)
{
    size_t free_before = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    int64_t deadline = esp_timer_get_time();
    for (uint32_t frame = 0; frame < PERF_FRAMES; ++frame) {
        load_frame(handle, pixels, PERF_LEDS, frame);
        deadline += PERF_FRAME_PERIOD_US;
        int64_t remaining = deadline - esp_timer_get_time();
        if (remaining > 1000) {
            vTaskDelay(pdMS_TO_TICKS((remaining - 500) / 1000));
        }
        remaining = deadline - esp_timer_get_time();
        if (remaining > 0) {
            esp_rom_delay_us((uint32_t)remaining);
        }
        ESP_ERROR_CHECK(ultraled_show_async(handle));
    }
    ESP_ERROR_CHECK(ultraled_wait_done(handle, ULTRALED_WAIT_FOREVER));
    size_t free_after = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    ESP_LOGI(TAG, "%d paced frames complete; heap delta=%d bytes", PERF_FRAMES,
             (int)free_after - (int)free_before);
    assert(free_after == free_before);
}

void app_main(void)
{
    ultraled_rgb_t *pixels = malloc(960 * sizeof(*pixels));
    assert(pixels != NULL);

    ultraled_config_t config = make_config(PERF_LEDS);
    ultraled_handle_t handle = NULL;
    ESP_ERROR_CHECK(ultraled_new(&config, &handle));
    load_frame(handle, pixels, PERF_LEDS, 0);
    benchmark_submit(handle, "480x8 default/full brightness");

    for (size_t lane = 0; lane < PERF_CHANNELS; ++lane) {
        ESP_ERROR_CHECK(ultraled_set_color_order(handle, lane,
            (ultraled_color_order_t)(ULTRALED_COLOR_ORDER_RGB + lane % 6U)));
        ESP_ERROR_CHECK(ultraled_set_brightness(handle, lane, (uint8_t)(1U + lane * 36U)));
    }
    benchmark_submit(handle, "480x8 mixed order/brightness");
    run_paced_frames(handle, pixels);
    ESP_ERROR_CHECK(ultraled_del(handle));

    config = make_config(960);
    ESP_ERROR_CHECK(ultraled_new(&config, &handle));
    load_frame(handle, pixels, 960, 0);
    int64_t start = esp_timer_get_time();
    ESP_ERROR_CHECK(ultraled_show_async(handle));
    int64_t submit_us = esp_timer_get_time() - start;
    ESP_LOGI(TAG, "960x8 submit=%" PRId64 " us (informational)", submit_us);
    ESP_ERROR_CHECK(ultraled_wait_done(handle, ULTRALED_WAIT_FOREVER));
    ESP_ERROR_CHECK(ultraled_del(handle));
    free(pixels);

    ESP_LOGI(TAG, "PASS");
}
