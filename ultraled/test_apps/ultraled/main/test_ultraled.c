/*
 * SPDX-FileCopyrightText: 2026 Pedro Minatel
 * SPDX-License-Identifier: Apache-2.0
 */

#include <limits.h>
#include <stdlib.h>
#include <string.h>

#include "driver/bitscrambler.h"
#include "driver/bitscrambler_loopback.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "unity.h"
#include "unity_test_runner.h"
#include "../../../ultraled_private.h"

BITSCRAMBLER_PROGRAM(program_1h3h, "ws2812");
BITSCRAMBLER_PROGRAM(program_1h2h, "sk6812");

static const uint8_t s_order_indices[][3] = {
    [ULTRALED_COLOR_ORDER_RGB] = {0, 1, 2},
    [ULTRALED_COLOR_ORDER_RBG] = {0, 2, 1},
    [ULTRALED_COLOR_ORDER_GRB] = {1, 0, 2},
    [ULTRALED_COLOR_ORDER_GBR] = {1, 2, 0},
    [ULTRALED_COLOR_ORDER_BRG] = {2, 0, 1},
    [ULTRALED_COLOR_ORDER_BGR] = {2, 1, 0},
};

static void expected_component(uint8_t component, uint8_t lane_mask, uint8_t *output)
{
    for (int bit = 7; bit >= 0; --bit) {
        *output++ = (component & (1U << bit)) ? lane_mask : 0;
    }
}

static uint8_t scale_reference(uint8_t value, uint8_t brightness)
{
    return (uint8_t)(((uint16_t)value * brightness + 127U) / 255U);
}

static void encode_reference(const ultraled_encoder_channel_t *channels, size_t channel_count,
                             size_t longest_channel, uint8_t *output)
{
    for (size_t pixel = 0; pixel < longest_channel; ++pixel) {
        for (size_t component = 0; component < 3; ++component) {
            for (int bit = 7; bit >= 0; --bit) {
                uint8_t lanes = 0;
                for (size_t lane = 0; lane < channel_count; ++lane) {
                    if (pixel < channels[lane].led_count) {
                        const uint8_t *rgb = (const uint8_t *)&channels[lane].pixels[pixel];
                        uint8_t value = scale_reference(rgb[channels[lane].component_indices[component]],
                                                        channels[lane].brightness);
                        lanes |= ((value >> bit) & 1U) << lane;
                    }
                }
                *output++ = lanes;
            }
        }
    }
}

TEST_CASE("encoder supports every color order", "[ultraled][encoder]")
{
    static const ultraled_color_order_t orders[] = {
        ULTRALED_COLOR_ORDER_RGB, ULTRALED_COLOR_ORDER_RBG, ULTRALED_COLOR_ORDER_GRB,
        ULTRALED_COLOR_ORDER_GBR, ULTRALED_COLOR_ORDER_BRG, ULTRALED_COLOR_ORDER_BGR,
    };
    const ultraled_rgb_t pixel = {.red = 0x81, .green = 0x42, .blue = 0x24};
    const uint8_t rgb[] = {pixel.red, pixel.green, pixel.blue};
    for (size_t order = 0; order < sizeof(orders) / sizeof(orders[0]); ++order) {
        ultraled_encoder_channel_t channel = {
            .pixels = &pixel,
            .led_count = 1,
            .component_indices = s_order_indices[orders[order]],
            .brightness = 255,
        };
        uint8_t actual[24] = {0};
        uint8_t expected[24] = {0};
        ultraled_encode(&channel, 1, 1, actual);
        for (size_t component = 0; component < 3; ++component) {
            expected_component(rgb[s_order_indices[orders[order]][component]], 1, &expected[component * 8]);
        }
        TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, actual, sizeof(actual));
    }
}

TEST_CASE("encoder handles brightness channel lengths and inactive lanes", "[ultraled][encoder]")
{
    const ultraled_rgb_t pixels0[] = {{.red = 255}, {.green = 255}};
    const ultraled_rgb_t pixels1[] = {{.red = 255}};
    const ultraled_rgb_t black[] = {{0}};
    ultraled_encoder_channel_t channels[ULTRALED_MAX_CHANNELS] = {
        {.pixels = pixels0, .led_count = 2, .component_indices = s_order_indices[ULTRALED_COLOR_ORDER_RGB], .brightness = 255},
        {.pixels = pixels1, .led_count = 1, .component_indices = s_order_indices[ULTRALED_COLOR_ORDER_RGB], .brightness = 0},
    };
    uint8_t output[48];
    memset(output, 0xa5, sizeof(output));
    ultraled_encode(channels, 2, 2, output);
    for (size_t i = 0; i < sizeof(output); ++i) {
        TEST_ASSERT_EQUAL_HEX8(0, output[i] & 0xfe); // lanes 1..7 stay low
    }
    TEST_ASSERT_EQUAL_HEX8(1, output[0]);
    TEST_ASSERT_EQUAL_HEX8(1, output[7]);
    TEST_ASSERT_EQUAL_HEX8(1, output[24 + 8]);
    channels[0] = (ultraled_encoder_channel_t) {
        .pixels = black, .led_count = 1,
        .component_indices = s_order_indices[ULTRALED_COLOR_ORDER_RGB], .brightness = 128,
    };
    ultraled_rgb_t half = {.red = 255};
    channels[0].pixels = &half;
    ultraled_encode(channels, 1, 1, output);
    TEST_ASSERT_EQUAL_HEX8(1, output[0]); // rounded 255 * 128 / 255 = 128
    for (size_t i = 1; i < 8; ++i) TEST_ASSERT_EQUAL_HEX8(0, output[i]);
}

TEST_CASE("encoder transposes all eight lanes", "[ultraled][encoder]")
{
    ultraled_rgb_t pixels[ULTRALED_MAX_CHANNELS];
    ultraled_encoder_channel_t channels[ULTRALED_MAX_CHANNELS];
    for (size_t lane = 0; lane < ULTRALED_MAX_CHANNELS; ++lane) {
        pixels[lane] = (ultraled_rgb_t) {.red = (uint8_t)(0x80U >> lane)};
        channels[lane] = (ultraled_encoder_channel_t) {
            .pixels = &pixels[lane], .led_count = 1,
            .component_indices = s_order_indices[ULTRALED_COLOR_ORDER_RGB], .brightness = 255,
        };
    }
    uint8_t output[24] = {0};
    ultraled_encode(channels, ULTRALED_MAX_CHANNELS, 1, output);
    for (size_t bit = 0; bit < 8; ++bit) {
        TEST_ASSERT_EQUAL_HEX8(1U << bit, output[bit]);
    }
    for (size_t i = 8; i < sizeof(output); ++i) TEST_ASSERT_EQUAL_HEX8(0, output[i]);
}

TEST_CASE("optimized encoder matches scalar reference for large mixed frames", "[ultraled][encoder]")
{
    static const size_t lengths[] = {1, 8, 480, 960};
    static const uint8_t brightness[] = {0, 1, 128, 254, 255};
    uint32_t random_state = 0x72a4c319U;

    for (size_t length_index = 0; length_index < sizeof(lengths) / sizeof(lengths[0]); ++length_index) {
        size_t longest = lengths[length_index];
        size_t output_size = longest * 24U;
        uint8_t *actual = malloc(output_size);
        uint8_t *expected = malloc(output_size);
        TEST_ASSERT_NOT_NULL(actual);
        TEST_ASSERT_NOT_NULL(expected);

        for (size_t channel_count = 1; channel_count <= ULTRALED_MAX_CHANNELS; ++channel_count) {
            ultraled_rgb_t *pixels[ULTRALED_MAX_CHANNELS] = {0};
            ultraled_encoder_channel_t channels[ULTRALED_MAX_CHANNELS] = {0};
            for (size_t lane = 0; lane < channel_count; ++lane) {
                size_t lane_length = longest > lane ? longest - lane : 1;
                pixels[lane] = malloc(lane_length * sizeof(*pixels[lane]));
                TEST_ASSERT_NOT_NULL(pixels[lane]);
                for (size_t pixel = 0; pixel < lane_length; ++pixel) {
                    random_state = random_state * 1664525U + 1013904223U;
                    pixels[lane][pixel] = (ultraled_rgb_t) {
                        .red = (uint8_t)random_state,
                        .green = (uint8_t)(random_state >> 8),
                        .blue = (uint8_t)(random_state >> 16),
                    };
                }
                ultraled_color_order_t order = (ultraled_color_order_t)(ULTRALED_COLOR_ORDER_RGB + lane % 6U);
                channels[lane] = (ultraled_encoder_channel_t) {
                    .pixels = pixels[lane],
                    .led_count = lane_length,
                    .component_indices = s_order_indices[order],
                    .brightness = brightness[lane % (sizeof(brightness) / sizeof(brightness[0]))],
                };
            }

            memset(actual, 0xa5, output_size);
            memset(expected, 0x5a, output_size);
            ultraled_encode(channels, channel_count, longest, actual);
            encode_reference(channels, channel_count, longest, expected);
            TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, actual, output_size);

            for (size_t lane = 0; lane < channel_count; ++lane) {
                free(pixels[lane]);
            }
        }
        free(actual);
        free(expected);
    }
}

static void check_program(const void *program, bool three_high)
{
    const uint8_t input[] = {0x00, 0xff, 0x01, 0x80, 0x55, 0xaa, 0x3c, 0xc3};
    const size_t output_capacity = 64;
    uint8_t *in = heap_caps_aligned_alloc(64, sizeof(input), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    uint8_t *out = heap_caps_aligned_calloc(64, 1, output_capacity, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    TEST_ASSERT_NOT_NULL(in);
    TEST_ASSERT_NOT_NULL(out);
    memcpy(in, input, sizeof(input));
    bitscrambler_handle_t bs = NULL;
    TEST_ESP_OK(bitscrambler_loopback_create(&bs, SOC_BITSCRAMBLER_ATTACH_I2S0, output_capacity));
    TEST_ESP_OK(bitscrambler_load_program(bs, program));
    size_t output_size = 0;
    TEST_ESP_OK(bitscrambler_loopback_run(bs, in, sizeof(input), out, output_capacity, &output_size));
    TEST_ASSERT_EQUAL(sizeof(input) * 4, output_size);
    for (size_t i = 0; i < sizeof(input); ++i) {
        TEST_ASSERT_EQUAL_HEX8(0xff, out[i * 4]);
        TEST_ASSERT_EQUAL_HEX8(input[i], out[i * 4 + 1]);
        TEST_ASSERT_EQUAL_HEX8(three_high ? input[i] : 0, out[i * 4 + 2]);
        TEST_ASSERT_EQUAL_HEX8(0, out[i * 4 + 3]);
    }
    bitscrambler_free(bs);
    free(in);
    free(out);
}

TEST_CASE("BitScrambler programs produce exact four-sample symbols", "[ultraled][bitscrambler]")
{
    check_program(program_1h3h, true);
    check_program(program_1h2h, false);
}

TEST_CASE("configuration rejects bad channels GPIOs and overflow", "[ultraled]")
{
    ultraled_handle_t handle = NULL;
    ultraled_config_t config = {
        .led_model = ULTRALED_MODEL_WS2812,
        .channel_count = 0,
    };
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ultraled_new(&config, &handle));
    config.channel_count = 2;
    config.channels[0] = (ultraled_channel_config_t) {.gpio_num = GPIO_NUM_10, .led_count = 1};
    config.channels[1] = (ultraled_channel_config_t) {.gpio_num = GPIO_NUM_10, .led_count = 1};
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ultraled_new(&config, &handle));
    config.channel_count = 1;
    config.channels[0].gpio_num = GPIO_NUM_NC;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ultraled_new(&config, &handle));
    config.channels[0].gpio_num = GPIO_NUM_10;
    config.channels[0].led_count = SIZE_MAX / 24U + 1U;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_SIZE, ultraled_new(&config, &handle));
}

TEST_CASE("public framebuffer bounds and brightness", "[ultraled][hardware]")
{
    const ultraled_config_t config = {
        .led_model = ULTRALED_MODEL_WS2812,
        .channel_count = 1,
        .channels = {{.gpio_num = GPIO_NUM_10, .led_count = 2, .color_order = ULTRALED_COLOR_ORDER_MODEL_DEFAULT}},
    };
    ultraled_handle_t handle = NULL;
    TEST_ESP_OK(ultraled_new(&config, &handle));
    const ultraled_rgb_t color = {.red = 1, .green = 2, .blue = 3};
    ultraled_rgb_t readback = {0};
    TEST_ESP_OK(ultraled_set_pixel(handle, 0, 1, color));
    TEST_ESP_OK(ultraled_get_pixel(handle, 0, 1, &readback));
    TEST_ASSERT_EQUAL_MEMORY(&color, &readback, sizeof(color));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ultraled_set_pixel(handle, 0, 2, color));
    TEST_ESP_OK(ultraled_set_brightness(handle, 0, 0));
    uint8_t brightness = 1;
    TEST_ESP_OK(ultraled_get_brightness(handle, 0, &brightness));
    TEST_ASSERT_EQUAL_UINT8(0, brightness);
    TEST_ESP_OK(ultraled_set_brightness(handle, 0, 255));
    TEST_ESP_OK(ultraled_show(handle, 1000));
    TEST_ASSERT_EQUAL(ESP_ERR_TIMEOUT, ultraled_show(handle, 0));
    TEST_ESP_OK(ultraled_set_pixel(handle, 0, 0, color));
    TEST_ESP_OK(ultraled_show(handle, 1000));
    TEST_ESP_OK(ultraled_del(handle));

    const ultraled_model_t other_models[] = {
        ULTRALED_MODEL_SK6812_RGB, ULTRALED_MODEL_APA106, ULTRALED_MODEL_SM16703,
    };
    for (size_t i = 0; i < sizeof(other_models) / sizeof(other_models[0]); ++i) {
        ultraled_config_t model_config = config;
        model_config.led_model = other_models[i];
        TEST_ESP_OK(ultraled_new(&model_config, &handle));
        TEST_ESP_OK(ultraled_show(handle, 1000));
        TEST_ESP_OK(ultraled_del(handle));
    }
}

TEST_CASE("asynchronous show permits framebuffer updates during DMA", "[ultraled][hardware][async]")
{
    const ultraled_config_t config = {
        .led_model = ULTRALED_MODEL_WS2812,
        .channel_count = 2,
        .channels = {
            {.gpio_num = GPIO_NUM_10, .led_count = 480, .color_order = ULTRALED_COLOR_ORDER_GRB},
            {.gpio_num = GPIO_NUM_11, .led_count = 8, .color_order = ULTRALED_COLOR_ORDER_GRB},
        },
    };
    ultraled_handle_t handle = NULL;
    TEST_ESP_OK(ultraled_new(&config, &handle));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ultraled_show_mask_async(handle, 0));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ultraled_show_mask(handle, 0x04, 1000));
    TEST_ESP_OK(ultraled_wait_done(handle, 0));
    TEST_ESP_OK(ultraled_show_mask_async(handle, 0x01));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, ultraled_show_mask_async(handle, 0x02));

    int64_t update_start = esp_timer_get_time();
    TEST_ESP_OK(ultraled_set_pixel(handle, 0, 0, (ultraled_rgb_t) {.green = 255}));
    TEST_ASSERT_LESS_THAN(5000, (int)(esp_timer_get_time() - update_start));
    TEST_ASSERT_EQUAL(ESP_ERR_TIMEOUT, ultraled_wait_done(handle, 0));
    TEST_ESP_OK(ultraled_wait_done(handle, 1000));
    TEST_ESP_OK(ultraled_wait_done(handle, 0));

    TEST_ESP_OK(ultraled_show_mask_async(handle, 0x02));
    TEST_ESP_OK(ultraled_wait_done(handle, 1000));
    TEST_ESP_OK(ultraled_show_mask(handle, 0x03, 1000));
    TEST_ESP_OK(ultraled_show_async(handle));
    TEST_ESP_OK(ultraled_show(handle, 1000));
    TEST_ESP_OK(ultraled_show_async(handle));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ultraled_wait_done(handle, -2));
    TEST_ESP_OK(ultraled_del(handle));
}

void app_main(void)
{
    unity_run_all_tests();
}
