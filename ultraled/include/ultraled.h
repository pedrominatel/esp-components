/*
 * SPDX-FileCopyrightText: 2026 Pedro Minatel
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "hal/gpio_types.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ULTRALED_MAX_CHANNELS 8
#define ULTRALED_WAIT_FOREVER (-1)

typedef struct ultraled_t *ultraled_handle_t;

typedef struct {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} ultraled_rgb_t;

typedef enum {
    ULTRALED_MODEL_WS2812 = 0,
    ULTRALED_MODEL_WS2812B = ULTRALED_MODEL_WS2812,
    ULTRALED_MODEL_SK6812_RGB,
    ULTRALED_MODEL_APA106,
    ULTRALED_MODEL_SM16703,
    ULTRALED_MODEL_MAX,
} ultraled_model_t;

typedef enum {
    ULTRALED_COLOR_ORDER_MODEL_DEFAULT = 0,
    ULTRALED_COLOR_ORDER_RGB,
    ULTRALED_COLOR_ORDER_RBG,
    ULTRALED_COLOR_ORDER_GRB,
    ULTRALED_COLOR_ORDER_GBR,
    ULTRALED_COLOR_ORDER_BRG,
    ULTRALED_COLOR_ORDER_BGR,
    ULTRALED_COLOR_ORDER_MAX,
} ultraled_color_order_t;

typedef struct {
    gpio_num_t gpio_num;
    size_t led_count;
    ultraled_color_order_t color_order;
} ultraled_channel_config_t;

typedef struct {
    /** Shared by every configured channel; mixed LED timing profiles are not supported. */
    ultraled_model_t led_model;
    size_t channel_count;
    ultraled_channel_config_t channels[ULTRALED_MAX_CHANNELS];
} ultraled_config_t;

/** Create and enable an Ultraled instance. Not callable from an ISR. */
esp_err_t ultraled_new(const ultraled_config_t *config, ultraled_handle_t *ret_handle);

/** Drain pending output and destroy an Ultraled instance. Not callable from an ISR. */
esp_err_t ultraled_del(ultraled_handle_t handle);

esp_err_t ultraled_set_pixel(ultraled_handle_t handle, size_t channel, size_t pixel, ultraled_rgb_t color);
esp_err_t ultraled_get_pixel(ultraled_handle_t handle, size_t channel, size_t pixel, ultraled_rgb_t *ret_color);
esp_err_t ultraled_set_pixels(ultraled_handle_t handle, size_t channel, size_t first_pixel,
                              const ultraled_rgb_t *colors, size_t color_count);
esp_err_t ultraled_fill(ultraled_handle_t handle, size_t channel, ultraled_rgb_t color);
esp_err_t ultraled_clear(ultraled_handle_t handle, size_t channel);
esp_err_t ultraled_clear_all(ultraled_handle_t handle);
esp_err_t ultraled_set_brightness(ultraled_handle_t handle, size_t channel, uint8_t brightness);
esp_err_t ultraled_get_brightness(ultraled_handle_t handle, size_t channel, uint8_t *ret_brightness);
esp_err_t ultraled_set_color_order(ultraled_handle_t handle, size_t channel, ultraled_color_order_t order);

/** Transmit every configured channel in lock-step. A timeout of -1 waits forever. */
esp_err_t ultraled_show(ultraled_handle_t handle, int timeout_ms);

/**
 * Transmit only the configured channels selected by channel_mask.
 * Bit N selects channel N. Unselected channel GPIOs remain low and their strips
 * retain the previously latched pixels. The mask must be nonzero and may not
 * select an unconfigured channel. A timeout of -1 waits forever.
 */
esp_err_t ultraled_show_mask(ultraled_handle_t handle, uint8_t channel_mask, int timeout_ms);

/**
 * Encode and queue every configured channel, returning before transmission completes.
 * Returns ESP_ERR_INVALID_STATE if the single staging buffer is still in use.
 * Not callable from an ISR.
 */
esp_err_t ultraled_show_async(ultraled_handle_t handle);

/**
 * Asynchronously transmit only the channels selected by channel_mask.
 * Mask validation and busy behavior match ultraled_show_mask().
 */
esp_err_t ultraled_show_mask_async(ultraled_handle_t handle, uint8_t channel_mask);

/** Wait for an asynchronous transmission. A timeout of -1 waits forever. Not callable from an ISR. */
esp_err_t ultraled_wait_done(ultraled_handle_t handle, int timeout_ms);

#ifdef __cplusplus
}
#endif
