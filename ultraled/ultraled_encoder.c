/*
 * SPDX-FileCopyrightText: 2026 Pedro Minatel
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ultraled_private.h"

#include <string.h>

_Static_assert(sizeof(ultraled_rgb_t) == 3, "ultraled_rgb_t must be packed RGB888");

_Static_assert(ULTRALED_MAX_CHANNELS == 16, "Encoder lane masks must be 16 bits");

static uint8_t scale_component(uint8_t value, uint8_t brightness)
{
    if (brightness == UINT8_MAX) {
        return value;
    }
    if (brightness == 0) {
        return 0;
    }
    return (uint8_t)(((uint16_t)value * brightness + 127U) / 255U);
}

static void ordered_components(ultraled_rgb_t color, const uint8_t component_indices[3], uint8_t out[3])
{
    const uint8_t rgb[] = {color.red, color.green, color.blue};
    out[0] = rgb[component_indices[0]];
    out[1] = rgb[component_indices[1]];
    out[2] = rgb[component_indices[2]];
}

void ultraled_encode(const ultraled_encoder_channel_t *channels, size_t channel_count,
                     size_t longest_channel, uint8_t *output)
{
    for (size_t pixel = 0; pixel < longest_channel; ++pixel) {
        uint8_t wire[ULTRALED_MAX_CHANNELS][3] = {0};
        for (size_t lane = 0; lane < channel_count; ++lane) {
            if (pixel < channels[lane].led_count) {
                ordered_components(channels[lane].pixels[pixel], channels[lane].component_indices, wire[lane]);
                for (size_t component = 0; component < 3; ++component) {
                    wire[lane][component] = scale_component(wire[lane][component], channels[lane].brightness);
                }
            }
        }
        for (size_t component = 0; component < 3; ++component) {
            for (int bit = 7; bit >= 0; --bit) {
                uint16_t lane_mask = 0;
                for (size_t lane = 0; lane < channel_count; ++lane) {
                    if ((wire[lane][component] & (1U << bit)) != 0) {
                        lane_mask |= (uint16_t)(1U << lane);
                    }
                }
                memcpy(output, &lane_mask, sizeof(lane_mask));
                output += sizeof(lane_mask);
            }
        }
    }
}
