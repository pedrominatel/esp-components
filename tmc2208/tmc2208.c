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
#include "stepper_motor_encoder.h"

static const char *TAG = "TMC2208";

#define STEP_MOTOR_ENABLE_LEVEL  0
#define STEP_MOTOR_SPIN_DIR_CLOCKWISE 0
#define STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE !STEP_MOTOR_SPIN_DIR_CLOCKWISE

#define STEP_MOTOR_RESOLUTION_HZ 1000000 // 1MHz resolution

tmc2208_motor_config_t motor_config = {
    .enable_level = 0,
    .dir_clockwise = 0,
    .resolution_hz = 1000000,
    .start_freq_hz = 100,
    .end_freq_hz = 100,
    .accel_samples = 100,
    .uniform_speed_hz = 2000,
    .decel_samples = 100
};

void tmc2208_test(tmc2208_io_config_t *config)
{

    ESP_LOGI(TAG, "Set spin direction");
    gpio_set_level(config->dir_pin, STEP_MOTOR_SPIN_DIR_CLOCKWISE);
    ESP_LOGI(TAG, "Enable step motor");
    gpio_set_level(config->enable_pin, STEP_MOTOR_ENABLE_LEVEL);

    ESP_LOGI(TAG, "Create motor encoders");
    stepper_motor_curve_encoder_config_t accel_encoder_config = {
        .resolution = STEP_MOTOR_RESOLUTION_HZ,
        .sample_points = motor_config.accel_samples,
        .start_freq_hz = motor_config.start_freq_hz,
        .end_freq_hz = motor_config.uniform_speed_hz,
    };

    rmt_encoder_handle_t accel_motor_encoder = NULL;
    ESP_ERROR_CHECK(rmt_new_stepper_motor_curve_encoder(&accel_encoder_config, &accel_motor_encoder));

    stepper_motor_uniform_encoder_config_t uniform_encoder_config = {
        .resolution = STEP_MOTOR_RESOLUTION_HZ,
    };
    rmt_encoder_handle_t uniform_motor_encoder = NULL;
    ESP_ERROR_CHECK(rmt_new_stepper_motor_uniform_encoder(&uniform_encoder_config, &uniform_motor_encoder));

    stepper_motor_curve_encoder_config_t decel_encoder_config = {
        .resolution = STEP_MOTOR_RESOLUTION_HZ,
        .sample_points = motor_config.decel_samples,
        .start_freq_hz = motor_config.uniform_speed_hz,
        .end_freq_hz = motor_config.end_freq_hz,
    };
    rmt_encoder_handle_t decel_motor_encoder = NULL;
    ESP_ERROR_CHECK(rmt_new_stepper_motor_curve_encoder(&decel_encoder_config, &decel_motor_encoder));

    ESP_LOGI(TAG, "Enable RMT channel");
    ESP_ERROR_CHECK(rmt_enable(config->motor_chan));

    ESP_LOGI(TAG, "Spin motor for 6000 steps: 500 accel + 5000 uniform + 500 decel");
    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
    };

    uint8_t dir = 0;

    while (1) {
        // acceleration phase
        tx_config.loop_count = 0;
        ESP_ERROR_CHECK(rmt_transmit(config->motor_chan, accel_motor_encoder, &motor_config.accel_samples, sizeof(motor_config.accel_samples), &tx_config));

        if(dir == 0) {
            gpio_set_level(config->dir_pin, STEP_MOTOR_SPIN_DIR_CLOCKWISE);
            dir = 1;
        } else {
            gpio_set_level(config->dir_pin, !STEP_MOTOR_SPIN_DIR_CLOCKWISE);
            dir = 0;
        }

        // uniform phase
        tx_config.loop_count = 1600 - motor_config.accel_samples - motor_config.decel_samples;
        ESP_ERROR_CHECK(rmt_transmit(config->motor_chan, uniform_motor_encoder, &motor_config.uniform_speed_hz, sizeof(motor_config.uniform_speed_hz), &tx_config));

        // deceleration phase
        tx_config.loop_count = 0;
        ESP_ERROR_CHECK(rmt_transmit(config->motor_chan, decel_motor_encoder, &motor_config.decel_samples, sizeof(motor_config.decel_samples), &tx_config));
        // wait all transactions finished
        ESP_ERROR_CHECK(rmt_tx_wait_all_done(config->motor_chan, -1));

        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}


esp_err_t tmc2208_move_steps(tmc2208_io_config_t *config, tmc2208_motor_config_t *motor_config, int32_t steps)
{
    
    ESP_LOGI(TAG, "Enable step motor");
    gpio_set_level(config->enable_pin, motor_config->enable_level);

    // Set direction
    if (steps > 0) {
        gpio_set_level(config->dir_pin, motor_config->dir_clockwise);
    } else {
        gpio_set_level(config->dir_pin, !motor_config->dir_clockwise);
        steps = steps * -1;
    }
    
    ESP_LOGI(TAG, "Create motor encoders");
    stepper_motor_curve_encoder_config_t accel_encoder_config = {
        .resolution = STEP_MOTOR_RESOLUTION_HZ,
        .sample_points = motor_config->accel_samples,
        .start_freq_hz = motor_config->start_freq_hz,
        .end_freq_hz = motor_config->uniform_speed_hz,
    };

    rmt_encoder_handle_t accel_motor_encoder = NULL;
    ESP_ERROR_CHECK(rmt_new_stepper_motor_curve_encoder(&accel_encoder_config, &accel_motor_encoder));

    stepper_motor_uniform_encoder_config_t uniform_encoder_config = {
        .resolution = STEP_MOTOR_RESOLUTION_HZ,
    };
    rmt_encoder_handle_t uniform_motor_encoder = NULL;
    ESP_ERROR_CHECK(rmt_new_stepper_motor_uniform_encoder(&uniform_encoder_config, &uniform_motor_encoder));

    stepper_motor_curve_encoder_config_t decel_encoder_config = {
        .resolution = STEP_MOTOR_RESOLUTION_HZ,
        .sample_points = motor_config->decel_samples,
        .start_freq_hz = motor_config->uniform_speed_hz,
        .end_freq_hz = motor_config->end_freq_hz,
    };
    rmt_encoder_handle_t decel_motor_encoder = NULL;
    ESP_ERROR_CHECK(rmt_new_stepper_motor_curve_encoder(&decel_encoder_config, &decel_motor_encoder));

    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
    };

    ESP_LOGI(TAG, "Enable RMT channel");
    ESP_ERROR_CHECK(rmt_enable(config->motor_chan));

    // acceleration phase
    tx_config.loop_count = 0;
    ESP_ERROR_CHECK(rmt_transmit(config->motor_chan, accel_motor_encoder, &motor_config->accel_samples, sizeof(motor_config->accel_samples), &tx_config));

    // uniform phase
    tx_config.loop_count = steps - motor_config->accel_samples - motor_config->decel_samples;
    ESP_ERROR_CHECK(rmt_transmit(config->motor_chan, uniform_motor_encoder, &motor_config->uniform_speed_hz, sizeof(motor_config->uniform_speed_hz), &tx_config));

    // deceleration phase
    tx_config.loop_count = 0;
    ESP_ERROR_CHECK(rmt_transmit(config->motor_chan, decel_motor_encoder, &motor_config->decel_samples, sizeof(motor_config->decel_samples), &tx_config));
    // wait all transactions finished
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(config->motor_chan, -1));

    ESP_LOGI(TAG, "Enable RMT channel");
    ESP_ERROR_CHECK(rmt_disable(config->motor_chan));

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
        .mem_block_symbols = 64,
        .resolution_hz = STEP_MOTOR_RESOLUTION_HZ,
        .trans_queue_depth = 10, // set the number of transactions that can be pending in the background
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &config->motor_chan));

    ESP_LOGI(TAG, "Driver initialization done");

    return ESP_OK;
}
