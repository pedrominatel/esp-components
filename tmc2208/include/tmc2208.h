/*
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief tmc2208 driver
 */

#pragma once

#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "driver/rmt.h"

typedef struct {
    rmt_channel_handle_t motor_chan;
    int step_pin;
    int dir_pin;
    int enable_pin;
    int ms1_pin;
    int ms2_pin;
} tmc2208_io_config_t;

typedef struct {
    uint8_t enable_level;
    uint8_t dir_clockwise;
    uint32_t resolution_hz;
    uint32_t start_freq_hz;
    uint32_t end_freq_hz;
    uint32_t accel_samples;
    uint32_t uniform_speed_hz;
    uint32_t decel_samples;
} tmc2208_motor_config_t;

typedef struct {
    tmc2208_io_config_t io_config;
    tmc2208_motor_config_t motor_config;
} tmc2208_config_t;

esp_err_t tmc2208_init(tmc2208_io_config_t *config);
esp_err_t tmc2208_move_steps(tmc2208_io_config_t *config, tmc2208_motor_config_t *motor_config, int32_t steps);
void tmc2208_test(tmc2208_io_config_t *config);

#ifdef __cplusplus
}
#endif
