/*
 * SPDX-FileCopyrightText: 2026 Pedro Minatel
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "ultraled.h"

typedef struct {
    const ultraled_rgb_t *pixels;
    size_t led_count;
    const uint8_t *component_indices;
    uint8_t brightness;
} ultraled_encoder_channel_t;

void ultraled_encode(const ultraled_encoder_channel_t *channels, size_t channel_count,
                     size_t longest_channel, uint8_t *output);
