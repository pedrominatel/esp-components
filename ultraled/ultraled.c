/*
 * SPDX-FileCopyrightText: 2026 Pedro Minatel
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ultraled.h"

#include <limits.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "driver/bitscrambler.h"
#include "driver/gpio.h"
#include "driver/parlio_bitscrambler.h"
#include "driver/parlio_tx.h"
#include "esp_chip_info.h"
#include "esp_heap_caps.h"
#include "esp_rom_gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "hal/gpio_types.h"
#include "hal/parlio_periph.h"
#include "soc/clk_tree_defs.h"
#include "soc/gpio_sig_map.h"
#include "ultraled_private.h"

#define ULTRALED_RESET_US 400U
#define ULTRALED_DMA_ALIGNMENT 32U
#define ULTRALED_MAGIC 0x554c4544U

BITSCRAMBLER_PROGRAM(ultraled_program_1h3h, "ws2812");
BITSCRAMBLER_PROGRAM(ultraled_program_1h2h, "sk6812");

typedef struct {
    uint32_t clock_hz;
    ultraled_color_order_t default_order;
    const void *program;
} ultraled_model_profile_t;

typedef struct {
    gpio_num_t gpio_num;
    size_t led_count;
    ultraled_color_order_t order;
    uint8_t component_indices[3];
    uint8_t brightness;
    ultraled_rgb_t *pixels;
} ultraled_channel_t;

struct ultraled_t {
    uint32_t magic;
    SemaphoreHandle_t frame_mutex;
    SemaphoreHandle_t tx_mutex;
    parlio_tx_unit_handle_t tx_unit;
    const ultraled_model_profile_t *profile;
    size_t channel_count;
    size_t longest_channel;
    ultraled_channel_t channels[ULTRALED_MAX_CHANNELS];
    uint8_t *staging;
    size_t staging_size;
    uint8_t *reset;
    size_t reset_size;
    bool enabled;
    bool decorated;
    bool in_flight;
    bool deleting;
    uint8_t routed_mask;
};

static const ultraled_model_profile_t s_profiles[] = {
    [ULTRALED_MODEL_WS2812] = {3200000, ULTRALED_COLOR_ORDER_GRB, ultraled_program_1h3h},
    [ULTRALED_MODEL_SK6812_RGB] = {3333333, ULTRALED_COLOR_ORDER_GRB, ultraled_program_1h2h},
    [ULTRALED_MODEL_APA106] = {2340000, ULTRALED_COLOR_ORDER_RGB, ultraled_program_1h3h},
    [ULTRALED_MODEL_SM16703] = {3333333, ULTRALED_COLOR_ORDER_RGB, ultraled_program_1h3h},
};

static bool valid_handle(ultraled_handle_t handle)
{
    return handle != NULL && handle->magic == ULTRALED_MAGIC;
}

static ultraled_color_order_t resolve_order(const ultraled_model_profile_t *profile, ultraled_color_order_t order)
{
    return order == ULTRALED_COLOR_ORDER_MODEL_DEFAULT ? profile->default_order : order;
}

static void set_channel_order(ultraled_channel_t *channel, ultraled_color_order_t order)
{
    static const uint8_t indices[][3] = {
        [ULTRALED_COLOR_ORDER_RGB] = {0, 1, 2},
        [ULTRALED_COLOR_ORDER_RBG] = {0, 2, 1},
        [ULTRALED_COLOR_ORDER_GRB] = {1, 0, 2},
        [ULTRALED_COLOR_ORDER_GBR] = {1, 2, 0},
        [ULTRALED_COLOR_ORDER_BRG] = {2, 0, 1},
        [ULTRALED_COLOR_ORDER_BGR] = {2, 1, 0},
    };
    channel->order = order;
    memcpy(channel->component_indices, indices[order], sizeof(channel->component_indices));
}

static esp_err_t validate_config(const ultraled_config_t *config, size_t *ret_longest)
{
    if (config == NULL || ret_longest == NULL || config->led_model < 0 || config->led_model >= ULTRALED_MODEL_MAX ||
        config->channel_count == 0 || config->channel_count > ULTRALED_MAX_CHANNELS) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t longest = 0;
    for (size_t i = 0; i < config->channel_count; ++i) {
        const ultraled_channel_config_t *channel = &config->channels[i];
        if (!GPIO_IS_VALID_OUTPUT_GPIO(channel->gpio_num) || channel->led_count == 0 ||
            channel->color_order < ULTRALED_COLOR_ORDER_MODEL_DEFAULT || channel->color_order >= ULTRALED_COLOR_ORDER_MAX) {
            return ESP_ERR_INVALID_ARG;
        }
        if (channel->led_count > SIZE_MAX / sizeof(ultraled_rgb_t)) {
            return ESP_ERR_INVALID_SIZE;
        }
        for (size_t previous = 0; previous < i; ++previous) {
            if (channel->gpio_num == config->channels[previous].gpio_num) {
                return ESP_ERR_INVALID_ARG;
            }
        }
        if (channel->led_count > longest) {
            longest = channel->led_count;
        }
    }
    if (longest > SIZE_MAX / 24U) {
        return ESP_ERR_INVALID_SIZE;
    }
    *ret_longest = longest;
    return ESP_OK;
}

static void remember_error(esp_err_t *first_error, esp_err_t error)
{
    if (*first_error == ESP_OK && error != ESP_OK) {
        *first_error = error;
    }
}

static esp_err_t destroy_resources(ultraled_handle_t handle)
{
    esp_err_t result = ESP_OK;
    if (handle->tx_unit != NULL && handle->in_flight) {
        esp_err_t error = parlio_tx_unit_wait_all_done(handle->tx_unit, ULTRALED_WAIT_FOREVER);
        remember_error(&result, error);
        if (error == ESP_OK) {
            handle->in_flight = false;
        }
    }
    if (handle->tx_unit != NULL && handle->enabled) {
        esp_err_t error = parlio_tx_unit_disable(handle->tx_unit);
        remember_error(&result, error);
        if (error == ESP_OK) {
            handle->enabled = false;
        }
    }
    if (handle->tx_unit != NULL && handle->decorated && !handle->enabled) {
        esp_err_t error = parlio_tx_unit_undecorate_bitscrambler(handle->tx_unit);
        remember_error(&result, error);
        if (error == ESP_OK) {
            handle->decorated = false;
        }
    }
    if (handle->tx_unit != NULL && !handle->enabled && !handle->decorated) {
        esp_err_t error = parlio_del_tx_unit(handle->tx_unit);
        remember_error(&result, error);
        if (error == ESP_OK) {
            handle->tx_unit = NULL;
        }
    }
    return result;
}

esp_err_t ultraled_new(const ultraled_config_t *config, ultraled_handle_t *ret_handle)
{
    if (ret_handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *ret_handle = NULL;

    esp_chip_info_t chip_info = {0};
    esp_chip_info(&chip_info);
    if (chip_info.revision < 300) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    size_t longest = 0;
    esp_err_t error = validate_config(config, &longest);
    if (error != ESP_OK) {
        return error;
    }

    ultraled_handle_t handle = calloc(1, sizeof(*handle));
    if (handle == NULL) {
        return ESP_ERR_NO_MEM;
    }
    handle->profile = &s_profiles[config->led_model];
    handle->channel_count = config->channel_count;
    handle->longest_channel = longest;
    handle->staging_size = longest * 24U;
    uint64_t reset_samples = ((uint64_t)handle->profile->clock_hz * ULTRALED_RESET_US + 999999U) / 1000000U;
    handle->reset_size = (size_t)((reset_samples + ULTRALED_DMA_ALIGNMENT - 1U) & ~(ULTRALED_DMA_ALIGNMENT - 1U));

    handle->frame_mutex = xSemaphoreCreateMutex();
    handle->tx_mutex = xSemaphoreCreateMutex();
    if (handle->frame_mutex == NULL || handle->tx_mutex == NULL) {
        error = ESP_ERR_NO_MEM;
        goto fail;
    }
    for (size_t i = 0; i < handle->channel_count; ++i) {
        handle->channels[i].gpio_num = config->channels[i].gpio_num;
        handle->channels[i].led_count = config->channels[i].led_count;
        set_channel_order(&handle->channels[i], resolve_order(handle->profile, config->channels[i].color_order));
        handle->channels[i].brightness = UINT8_MAX;
        handle->channels[i].pixels = calloc(config->channels[i].led_count, sizeof(ultraled_rgb_t));
        if (handle->channels[i].pixels == NULL) {
            error = ESP_ERR_NO_MEM;
            goto fail;
        }
    }
    handle->staging = heap_caps_aligned_calloc(ULTRALED_DMA_ALIGNMENT, 1, handle->staging_size,
                                                MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    handle->reset = heap_caps_aligned_calloc(ULTRALED_DMA_ALIGNMENT, 1, handle->reset_size,
                                              MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (handle->staging == NULL || handle->reset == NULL) {
        error = ESP_ERR_NO_MEM;
        goto fail;
    }

    parlio_tx_unit_config_t tx_config = {
        .clk_src = PARLIO_CLK_SRC_DEFAULT,
        .clk_in_gpio_num = GPIO_NUM_NC,
        .output_clk_freq_hz = handle->profile->clock_hz,
        .data_width = ULTRALED_MAX_CHANNELS,
        .clk_out_gpio_num = GPIO_NUM_NC,
        .valid_gpio_num = GPIO_NUM_NC,
        .trans_queue_depth = 2,
        .max_transfer_size = handle->staging_size > handle->reset_size ? handle->staging_size : handle->reset_size,
        .dma_burst_size = ULTRALED_DMA_ALIGNMENT,
        .sample_edge = PARLIO_SAMPLE_EDGE_POS,
    };
    for (size_t lane = 0; lane < ULTRALED_MAX_CHANNELS; ++lane) {
        tx_config.data_gpio_nums[lane] = lane < handle->channel_count ? handle->channels[lane].gpio_num : GPIO_NUM_NC;
    }
    error = parlio_new_tx_unit(&tx_config, &handle->tx_unit);
    if (error != ESP_OK) {
        goto fail;
    }
    error = parlio_tx_unit_decorate_bitscrambler(handle->tx_unit);
    if (error != ESP_OK) {
        goto fail;
    }
    handle->decorated = true;
    error = parlio_tx_unit_enable(handle->tx_unit);
    if (error != ESP_OK) {
        goto fail;
    }
    handle->enabled = true;
    handle->routed_mask = (uint8_t)((1U << handle->channel_count) - 1U);
    handle->magic = ULTRALED_MAGIC;
    *ret_handle = handle;
    return ESP_OK;

fail:
    destroy_resources(handle);
    for (size_t i = 0; i < ULTRALED_MAX_CHANNELS; ++i) {
        free(handle->channels[i].pixels);
    }
    heap_caps_free(handle->staging);
    heap_caps_free(handle->reset);
    if (handle->frame_mutex != NULL) {
        vSemaphoreDelete(handle->frame_mutex);
    }
    if (handle->tx_mutex != NULL) {
        vSemaphoreDelete(handle->tx_mutex);
    }
    free(handle);
    return error;
}

esp_err_t ultraled_del(ultraled_handle_t handle)
{
    if (!valid_handle(handle)) {
        return ESP_ERR_INVALID_ARG;
    }
    xSemaphoreTake(handle->tx_mutex, portMAX_DELAY);
    xSemaphoreTake(handle->frame_mutex, portMAX_DELAY);
    if (handle->deleting) {
        xSemaphoreGive(handle->frame_mutex);
        xSemaphoreGive(handle->tx_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    handle->deleting = true;
    esp_err_t result = destroy_resources(handle);
    if (result != ESP_OK) {
        handle->deleting = false;
        xSemaphoreGive(handle->frame_mutex);
        xSemaphoreGive(handle->tx_mutex);
        return result;
    }
    handle->magic = 0;
    for (size_t i = 0; i < handle->channel_count; ++i) {
        free(handle->channels[i].pixels);
    }
    heap_caps_free(handle->staging);
    heap_caps_free(handle->reset);
    xSemaphoreGive(handle->frame_mutex);
    xSemaphoreGive(handle->tx_mutex);
    vSemaphoreDelete(handle->frame_mutex);
    vSemaphoreDelete(handle->tx_mutex);
    free(handle);
    return ESP_OK;
}

static esp_err_t lock_channel(ultraled_handle_t handle, size_t channel)
{
    if (!valid_handle(handle)) {
        return ESP_ERR_INVALID_ARG;
    }
    xSemaphoreTake(handle->frame_mutex, portMAX_DELAY);
    if (handle->deleting) {
        xSemaphoreGive(handle->frame_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    if (channel >= handle->channel_count) {
        xSemaphoreGive(handle->frame_mutex);
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

esp_err_t ultraled_set_pixel(ultraled_handle_t handle, size_t channel, size_t pixel, ultraled_rgb_t color)
{
    esp_err_t error = lock_channel(handle, channel);
    if (error != ESP_OK) return error;
    if (pixel >= handle->channels[channel].led_count) {
        error = ESP_ERR_INVALID_ARG;
    } else {
        handle->channels[channel].pixels[pixel] = color;
    }
    xSemaphoreGive(handle->frame_mutex);
    return error;
}

esp_err_t ultraled_get_pixel(ultraled_handle_t handle, size_t channel, size_t pixel, ultraled_rgb_t *ret_color)
{
    if (ret_color == NULL) return ESP_ERR_INVALID_ARG;
    esp_err_t error = lock_channel(handle, channel);
    if (error != ESP_OK) return error;
    if (pixel >= handle->channels[channel].led_count) {
        error = ESP_ERR_INVALID_ARG;
    } else {
        *ret_color = handle->channels[channel].pixels[pixel];
    }
    xSemaphoreGive(handle->frame_mutex);
    return error;
}

esp_err_t ultraled_set_pixels(ultraled_handle_t handle, size_t channel, size_t first_pixel,
                              const ultraled_rgb_t *colors, size_t color_count)
{
    if (colors == NULL && color_count != 0) return ESP_ERR_INVALID_ARG;
    esp_err_t error = lock_channel(handle, channel);
    if (error != ESP_OK) return error;
    size_t count = handle->channels[channel].led_count;
    if (first_pixel > count || color_count > count - first_pixel) {
        error = ESP_ERR_INVALID_ARG;
    } else if (color_count != 0) {
        memcpy(&handle->channels[channel].pixels[first_pixel], colors, color_count * sizeof(*colors));
    }
    xSemaphoreGive(handle->frame_mutex);
    return error;
}

esp_err_t ultraled_fill(ultraled_handle_t handle, size_t channel, ultraled_rgb_t color)
{
    esp_err_t error = lock_channel(handle, channel);
    if (error != ESP_OK) return error;
    for (size_t i = 0; i < handle->channels[channel].led_count; ++i) {
        handle->channels[channel].pixels[i] = color;
    }
    xSemaphoreGive(handle->frame_mutex);
    return ESP_OK;
}

esp_err_t ultraled_clear(ultraled_handle_t handle, size_t channel)
{
    const ultraled_rgb_t black = {0};
    return ultraled_fill(handle, channel, black);
}

esp_err_t ultraled_clear_all(ultraled_handle_t handle)
{
    if (!valid_handle(handle)) return ESP_ERR_INVALID_ARG;
    xSemaphoreTake(handle->frame_mutex, portMAX_DELAY);
    if (handle->deleting) {
        xSemaphoreGive(handle->frame_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    for (size_t channel = 0; channel < handle->channel_count; ++channel) {
        memset(handle->channels[channel].pixels, 0, handle->channels[channel].led_count * sizeof(ultraled_rgb_t));
    }
    xSemaphoreGive(handle->frame_mutex);
    return ESP_OK;
}

esp_err_t ultraled_set_brightness(ultraled_handle_t handle, size_t channel, uint8_t brightness)
{
    esp_err_t error = lock_channel(handle, channel);
    if (error != ESP_OK) return error;
    handle->channels[channel].brightness = brightness;
    xSemaphoreGive(handle->frame_mutex);
    return ESP_OK;
}

esp_err_t ultraled_get_brightness(ultraled_handle_t handle, size_t channel, uint8_t *ret_brightness)
{
    if (ret_brightness == NULL) return ESP_ERR_INVALID_ARG;
    esp_err_t error = lock_channel(handle, channel);
    if (error != ESP_OK) return error;
    *ret_brightness = handle->channels[channel].brightness;
    xSemaphoreGive(handle->frame_mutex);
    return ESP_OK;
}

esp_err_t ultraled_set_color_order(ultraled_handle_t handle, size_t channel, ultraled_color_order_t order)
{
    if (order < ULTRALED_COLOR_ORDER_MODEL_DEFAULT || order >= ULTRALED_COLOR_ORDER_MAX) return ESP_ERR_INVALID_ARG;
    esp_err_t error = lock_channel(handle, channel);
    if (error != ESP_OK) return error;
    set_channel_order(&handle->channels[channel], resolve_order(handle->profile, order));
    xSemaphoreGive(handle->frame_mutex);
    return ESP_OK;
}

static int remaining_timeout(int timeout_ms, int64_t deadline_us)
{
    if (timeout_ms < 0) return ULTRALED_WAIT_FOREVER;
    int64_t remaining_us = deadline_us - esp_timer_get_time();
    if (remaining_us <= 0) return 0;
    int64_t remaining_ms = (remaining_us + 999) / 1000;
    return remaining_ms > INT_MAX ? INT_MAX : (int)remaining_ms;
}

static bool take_mutex_until(SemaphoreHandle_t mutex, int timeout_ms, int64_t deadline_us)
{
    if (timeout_ms < 0) {
        return xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE;
    }
    int remaining_ms = remaining_timeout(timeout_ms, deadline_us);
    TickType_t ticks = pdMS_TO_TICKS(remaining_ms);
    if (remaining_ms > 0 && ticks == 0) {
        ticks = 1;
    }
    return xSemaphoreTake(mutex, ticks) == pdTRUE;
}

static uint8_t configured_channel_mask(ultraled_handle_t handle)
{
    return (uint8_t)((1U << handle->channel_count) - 1U);
}

static bool valid_channel_mask(ultraled_handle_t handle, uint8_t channel_mask)
{
    uint8_t configured = configured_channel_mask(handle);
    return channel_mask != 0 && (channel_mask & (uint8_t)~configured) == 0;
}

static size_t encode_frame(ultraled_handle_t handle, uint8_t channel_mask)
{
    ultraled_encoder_channel_t channels[ULTRALED_MAX_CHANNELS];
    size_t longest_selected = 0;
    for (size_t i = 0; i < handle->channel_count; ++i) {
        bool selected = (channel_mask & (1U << i)) != 0;
        channels[i] = (ultraled_encoder_channel_t) {
            .pixels = handle->channels[i].pixels,
            .led_count = selected ? handle->channels[i].led_count : 0,
            .component_indices = handle->channels[i].component_indices,
            .brightness = handle->channels[i].brightness,
        };
        if (selected && handle->channels[i].led_count > longest_selected) {
            longest_selected = handle->channels[i].led_count;
        }
    }
    ultraled_encode(channels, handle->channel_count, longest_selected, handle->staging);
    return longest_selected * 24U;
}

static esp_err_t route_channels(ultraled_handle_t handle, uint8_t channel_mask)
{
    uint8_t changed = handle->routed_mask ^ channel_mask;
    for (size_t lane = 0; lane < handle->channel_count; ++lane) {
        uint8_t lane_bit = (uint8_t)(1U << lane);
        if ((changed & lane_bit) == 0) {
            continue;
        }
        gpio_num_t gpio_num = handle->channels[lane].gpio_num;
        if ((channel_mask & lane_bit) != 0) {
            esp_rom_gpio_connect_out_signal(gpio_num, soc_parlio_signals[0].tx_units[0].data_sigs[lane], false, false);
        } else {
            esp_err_t error = gpio_set_level(gpio_num, 0);
            if (error != ESP_OK) {
                return error;
            }
            esp_rom_gpio_connect_out_signal(gpio_num, SIG_GPIO_OUT_IDX, false, false);
            error = gpio_output_enable(gpio_num);
            if (error != ESP_OK) {
                return error;
            }
        }
    }
    handle->routed_mask = channel_mask;
    return ESP_OK;
}

static esp_err_t submit_frame(ultraled_handle_t handle, size_t encoded_size)
{
    const parlio_transmit_config_t data_config = {
        .idle_value = 0,
        .bitscrambler_program = handle->profile->program,
        .flags.queue_nonblocking = true,
    };
    const parlio_transmit_config_t reset_config = {
        .idle_value = 0,
        .bitscrambler_program = NULL,
        .flags.queue_nonblocking = true,
    };
    esp_err_t error = parlio_tx_unit_transmit(handle->tx_unit, handle->staging,
                                               encoded_size * 8U, &data_config);
    if (error == ESP_OK) {
        handle->in_flight = true;
        error = parlio_tx_unit_transmit(handle->tx_unit, handle->reset,
                                        handle->reset_size * 8U, &reset_config);
    }
    return error;
}

static esp_err_t wait_done_locked(ultraled_handle_t handle, int timeout_ms)
{
    if (!handle->in_flight) {
        return ESP_OK;
    }
    esp_err_t error = parlio_tx_unit_wait_all_done(handle->tx_unit, timeout_ms);
    if (error == ESP_OK) {
        handle->in_flight = false;
    }
    return error;
}

esp_err_t ultraled_show_mask_async(ultraled_handle_t handle, uint8_t channel_mask)
{
    if (!valid_handle(handle) || !valid_channel_mask(handle, channel_mask)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (xSemaphoreTake(handle->tx_mutex, 0) != pdTRUE) {
        return ESP_ERR_INVALID_STATE;
    }
    if (handle->deleting) {
        xSemaphoreGive(handle->tx_mutex);
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t error = wait_done_locked(handle, 0);
    if (error == ESP_ERR_TIMEOUT) {
        xSemaphoreGive(handle->tx_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    if (error != ESP_OK) {
        xSemaphoreGive(handle->tx_mutex);
        return error;
    }

    xSemaphoreTake(handle->frame_mutex, portMAX_DELAY);
    if (handle->deleting) {
        xSemaphoreGive(handle->frame_mutex);
        xSemaphoreGive(handle->tx_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    size_t encoded_size = encode_frame(handle, channel_mask);
    xSemaphoreGive(handle->frame_mutex);

    error = route_channels(handle, channel_mask);
    if (error == ESP_OK) {
        error = submit_frame(handle, encoded_size);
    }
    xSemaphoreGive(handle->tx_mutex);
    return error;
}

esp_err_t ultraled_show_async(ultraled_handle_t handle)
{
    if (!valid_handle(handle)) {
        return ESP_ERR_INVALID_ARG;
    }
    return ultraled_show_mask_async(handle, configured_channel_mask(handle));
}

esp_err_t ultraled_wait_done(ultraled_handle_t handle, int timeout_ms)
{
    if (!valid_handle(handle) || timeout_ms < ULTRALED_WAIT_FOREVER) {
        return ESP_ERR_INVALID_ARG;
    }
    int64_t deadline_us = timeout_ms < 0 ? 0 : esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    if (!take_mutex_until(handle->tx_mutex, timeout_ms, deadline_us)) {
        return ESP_ERR_TIMEOUT;
    }
    if (handle->deleting) {
        xSemaphoreGive(handle->tx_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t error = wait_done_locked(handle, remaining_timeout(timeout_ms, deadline_us));
    xSemaphoreGive(handle->tx_mutex);
    return error;
}

esp_err_t ultraled_show_mask(ultraled_handle_t handle, uint8_t channel_mask, int timeout_ms)
{
    if (!valid_handle(handle) || !valid_channel_mask(handle, channel_mask) ||
        timeout_ms < ULTRALED_WAIT_FOREVER) {
        return ESP_ERR_INVALID_ARG;
    }
    int64_t deadline_us = timeout_ms < 0 ? 0 : esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    if (!take_mutex_until(handle->tx_mutex, timeout_ms, deadline_us)) {
        return ESP_ERR_TIMEOUT;
    }
    if (handle->deleting) {
        xSemaphoreGive(handle->tx_mutex);
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t error = wait_done_locked(handle, remaining_timeout(timeout_ms, deadline_us));
    if (error != ESP_OK) {
        xSemaphoreGive(handle->tx_mutex);
        return error;
    }

    if (!take_mutex_until(handle->frame_mutex, timeout_ms, deadline_us)) {
        xSemaphoreGive(handle->tx_mutex);
        return ESP_ERR_TIMEOUT;
    }
    if (handle->deleting) {
        xSemaphoreGive(handle->frame_mutex);
        xSemaphoreGive(handle->tx_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    size_t encoded_size = encode_frame(handle, channel_mask);
    xSemaphoreGive(handle->frame_mutex);

    error = route_channels(handle, channel_mask);
    if (error == ESP_OK) {
        error = submit_frame(handle, encoded_size);
    }
    if (error == ESP_OK) {
        error = wait_done_locked(handle, remaining_timeout(timeout_ms, deadline_us));
    }
    xSemaphoreGive(handle->tx_mutex);
    return error;
}

esp_err_t ultraled_show(ultraled_handle_t handle, int timeout_ms)
{
    if (!valid_handle(handle)) {
        return ESP_ERR_INVALID_ARG;
    }
    return ultraled_show_mask(handle, configured_channel_mask(handle), timeout_ms);
}
