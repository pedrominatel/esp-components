/*
 * SPDX-FileCopyrightText: 2026 Pedro Minatel
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ultraled_private.h"

#include <string.h>

_Static_assert(sizeof(ultraled_rgb_t) == 3, "ultraled_rgb_t must be packed RGB888");

/* Expand a nibble into the low bit of four consecutive output bytes, MSB first. */
static const uint32_t s_expand_nibble[16] = {
    0x00000000U, 0x01000000U, 0x00010000U, 0x01010000U,
    0x00000100U, 0x01000100U, 0x00010100U, 0x01010100U,
    0x00000001U, 0x01000001U, 0x00010001U, 0x01010001U,
    0x00000101U, 0x01000101U, 0x00010101U, 0x01010101U,
};

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
            uint32_t high_planes = 0;
            uint32_t low_planes = 0;
            for (size_t lane = 0; lane < channel_count; ++lane) {
                uint8_t value = wire[lane][component];
                high_planes |= s_expand_nibble[value >> 4] << lane;
                low_planes |= s_expand_nibble[value & 0x0fU] << lane;
            }
            memcpy(output, &high_planes, sizeof(high_planes));
            memcpy(output + sizeof(high_planes), &low_planes, sizeof(low_planes));
            output += 8;
        }
    }
}
