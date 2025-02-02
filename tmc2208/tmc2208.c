/*
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"

#include "tmc2208.h"
#include "include/tmc2208.h"
#include "stepper_motor_encoder.h"

static const char *TAG = "TMC2208";

static esp_err_t create_uniform_motor_encoder(const tmc2208_motor_config_t *motor_config, rmt_encoder_handle_t *uniform_motor_encoder)
{
    stepper_motor_uniform_encoder_config_t uniform_encoder_config = {
        .resolution = motor_config->resolution_hz,
    };

    *uniform_motor_encoder = NULL;
    rmt_new_stepper_motor_uniform_encoder(&uniform_encoder_config, uniform_motor_encoder);
    
    return ESP_OK;
}

esp_err_t tmc2208_set_dir(tmc2208_io_config_t *config, tmc2208_motor_config_t *motor_config, tmc2208_dir_t dir)
{
    if (dir == TMC2208_DIR_CW) {
        gpio_set_level(config->dir_pin, motor_config->dir_clockwise);
    } else {
        gpio_set_level(config->dir_pin, !motor_config->dir_clockwise);
    }
    return ESP_OK;
}

static esp_err_t tmc2208_accelerate(tmc2208_io_config_t *config, tmc2208_motor_config_t *motor_config)
{
    stepper_motor_curve_encoder_config_t accel_encoder_config = {
        .resolution = motor_config->resolution_hz,
        .sample_points = motor_config->accel_samples,
        .start_freq_hz = motor_config->start_freq_hz,
        .end_freq_hz = motor_config->uniform_speed_hz,
    };

    rmt_encoder_handle_t accel_motor_encoder = NULL;
    ESP_ERROR_CHECK(rmt_new_stepper_motor_curve_encoder(&accel_encoder_config, &accel_motor_encoder));

    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
    };

    // acceleration phase
    tx_config.loop_count = 0;
    ESP_ERROR_CHECK(rmt_transmit(config->motor_chan, accel_motor_encoder, &motor_config->accel_samples, sizeof(motor_config->accel_samples), &tx_config));
    // wait all transactions finished
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(config->motor_chan, -1));

    return ESP_OK;
}

static esp_err_t tmc2208_decelerate(tmc2208_io_config_t *config, tmc2208_motor_config_t *motor_config)
{
    stepper_motor_curve_encoder_config_t decel_encoder_config = {
        .resolution = motor_config->resolution_hz,
        .sample_points = motor_config->decel_samples,
        .start_freq_hz = motor_config->uniform_speed_hz,
        .end_freq_hz = motor_config->end_freq_hz,
    };
    rmt_encoder_handle_t decel_motor_encoder = NULL;
    ESP_ERROR_CHECK(rmt_new_stepper_motor_curve_encoder(&decel_encoder_config, &decel_motor_encoder));

    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
    };

    // deceleration phase
    tx_config.loop_count = 0;
    ESP_ERROR_CHECK(rmt_transmit(config->motor_chan, decel_motor_encoder, &motor_config->decel_samples, sizeof(motor_config->decel_samples), &tx_config));
    // wait all transactions finished
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(config->motor_chan, -1));

    return ESP_OK;
}

esp_err_t tmc2208_uniform_move_steps(tmc2208_io_config_t *config, tmc2208_motor_config_t *motor_config, int32_t steps)
{

    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
    };
    
    // uniform phase
    tx_config.loop_count = steps;
    // set steps uniform
    ESP_ERROR_CHECK(rmt_transmit(config->motor_chan, motor_config->uniform_motor_encoder, &motor_config->uniform_speed_hz, sizeof(motor_config->uniform_speed_hz), &tx_config));
    // wait all transactions finished
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(config->motor_chan, -1));

    return ESP_OK;
}

esp_err_t tmc2208_move_steps(tmc2208_io_config_t *config, tmc2208_motor_config_t *motor_config, int32_t steps, bool accel, bool decel)
{

    // Enable motor
    tmc2208_enable(config, motor_config);

    // Set direction
    if (steps > 0) {
        tmc2208_set_dir(config, motor_config, TMC2208_DIR_CW);
    } else {
        tmc2208_set_dir(config, motor_config, TMC2208_DIR_CCW);
        steps = steps * -1;
    }
    
    if(accel) {
        tmc2208_accelerate(config, motor_config);
    }

    tmc2208_uniform_move_steps(config, motor_config, steps);

    if(decel) {
        tmc2208_decelerate(config, motor_config);
    }

    // Disable motor
    tmc2208_disable(config, motor_config);

    return ESP_OK;
}

esp_err_t tmc2208_init(tmc2208_io_config_t *config)
{
    
    ESP_LOGI(TAG, "Initialize GPIO");
    gpio_config_t en_dir_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pin_bit_mask = 1ULL << config->dir_pin | 1ULL << config->enable_pin | 1ULL << config->ms1_pin | 1ULL << config->ms2_pin,
    };
    ESP_ERROR_CHECK(gpio_config(&en_dir_gpio_config));

    ESP_LOGI(TAG, "Create RMT TX channel");

    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT, // select clock source
        .gpio_num = config->step_pin,
        .mem_block_symbols = 128,
        .resolution_hz = 1000000, // set counter clock frequency
        .trans_queue_depth = 10, // set the number of transactions that can be pending in the background
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &config->motor_chan));

    ESP_LOGI(TAG, "Driver initialization done");

    return ESP_OK;
}

esp_err_t tmc2208_enable(tmc2208_io_config_t *config, tmc2208_motor_config_t *motor_config)
{
    gpio_set_level(config->enable_pin, motor_config->enable_level);
    // set microstep
    gpio_set_level(config->ms1_pin, 0);
    gpio_set_level(config->ms2_pin, 0);

    // Set microsetp according to the motor_config
    // MS2, MS1: 00: 1/8, 01: 1/2, 10: 1/4 11: 1/16
    switch (motor_config->microstep) {
        case TMC2208_MICROSTEP_2:
            gpio_set_level(config->ms1_pin, 1);
            gpio_set_level(config->ms2_pin, 0);
            break;
        case TMC2208_MICROSTEP_4:
            gpio_set_level(config->ms1_pin, 0);
            gpio_set_level(config->ms2_pin, 1);
            break;
        case TMC2208_MICROSTEP_8:
            gpio_set_level(config->ms1_pin, 0);
            gpio_set_level(config->ms2_pin, 0);
            break;
        case TMC2208_MICROSTEP_16:
            gpio_set_level(config->ms1_pin, 1);
            gpio_set_level(config->ms2_pin, 1);
            break;
        default:
            break;
    }

    // Set direction
    gpio_set_level(config->dir_pin, motor_config->dir_clockwise);
    ESP_ERROR_CHECK(create_uniform_motor_encoder(motor_config, &motor_config->uniform_motor_encoder));
    ESP_ERROR_CHECK(rmt_enable(config->motor_chan));
    return ESP_OK;
}

esp_err_t tmc2208_disable(tmc2208_io_config_t *config, tmc2208_motor_config_t *motor_config)
{
    gpio_set_level(config->enable_pin, !motor_config->enable_level);
    ESP_ERROR_CHECK(rmt_disable(config->motor_chan));
    return ESP_OK;
}
