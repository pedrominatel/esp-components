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

/**
 * @enum tmc2208_microstep_t
 * @brief Enumeration for TMC2208 microstepping resolution.
 *
 * This enumeration defines the possible microstepping resolutions for the TMC2208 stepper motor driver.
 *
 * @var TMC2208_MICROSTEP_2
 * Microstepping resolution of 2 steps.
 *
 * @var TMC2208_MICROSTEP_4
 * Microstepping resolution of 4 steps.
 *
 * @var TMC2208_MICROSTEP_8
 * Microstepping resolution of 8 steps.
 *
 * @var TMC2208_MICROSTEP_16
 * Microstepping resolution of 16 steps.
 */
typedef enum {
    TMC2208_MICROSTEP_2 = 0,
    TMC2208_MICROSTEP_4,
    TMC2208_MICROSTEP_8,
    TMC2208_MICROSTEP_16,
} tmc2208_microstep_t;

/**
 * @enum tmc2208_dir_t
 * @brief Enumeration for TMC2208 motor direction.
 *
 * This enumeration defines the possible directions for the TMC2208 stepper motor driver.
 *
 * @var TMC2208_DIR_CW
 * Clockwise direction.
 *
 * @var TMC2208_DIR_CCW
 * Counter-clockwise direction.
 */
typedef enum {
    TMC2208_DIR_CW = 0,
    TMC2208_DIR_CCW,
} tmc2208_dir_t;

/**
 * @enum tmc2208_curve_t
 * @brief Enumeration for TMC2208 curve types.
 *
 * This enumeration defines the types of curves that can be used with the TMC2208 stepper motor driver.
 *
 * @var TMC2208_CURVE_SMOOTH
 * Smooth curve type.
 *
 * @var TMC2208_CURVE_LINEAR
 * Linear curve type.
 */
typedef enum {
    TMC2208_CURVE_SMOOTH = 0,
    TMC2208_CURVE_LINEAR,
} tmc2208_curve_t;

/**
 * @brief Configuration structure for TMC2208 IO pins and RMT channels.
 *
 * This structure holds the configuration for the TMC2208 stepper motor driver,
 * including the RMT channel handle, RMT TX channel configuration, and the GPIO
 * pins used for step, direction, enable, and microstepping control.
 */
typedef struct {
    rmt_channel_handle_t motor_chan;
    rmt_tx_channel_config_t tx_chan_config;
    int step_pin;
    int dir_pin;
    int enable_pin;
    int ms1_pin;
    int ms2_pin;
} tmc2208_io_config_t;

/**
 * @brief Configuration structure for TMC2208 motor driver.
 *
 * This structure holds the configuration parameters for the TMC2208 motor driver.
 */
typedef struct {
    uint8_t enable_level;                /**< Level to enable the motor driver. */
    tmc2208_dir_t dir_clockwise;         /**< Direction of the motor rotation (clockwise). */
    tmc2208_curve_t smooth_curve;        /**< Smooth curve configuration for motor movement. */
    tmc2208_microstep_t microstep;       /**< Microstepping configuration. */
    rmt_encoder_handle_t uniform_motor_encoder; /**< Handle for the uniform motor encoder. */
    uint32_t resolution_hz;              /**< Resolution in Hz. */
    uint32_t start_freq_hz;              /**< Starting frequency in Hz. */
    uint32_t end_freq_hz;                /**< Ending frequency in Hz. */
    uint32_t accel_samples;              /**< Number of samples for acceleration. */
    uint32_t uniform_speed_hz;           /**< Uniform speed in Hz. */
    uint32_t decel_samples;              /**< Number of samples for deceleration. */
    uint32_t max_freq_hz;                /**< Maximum frequency in Hz. */
} tmc2208_motor_config_t;

/**
 * @brief Configuration structure for TMC2208 driver.
 * 
 * This structure holds the configuration settings for the TMC2208 driver,
 * including I/O configuration and motor configuration.
 */
typedef struct {
    tmc2208_io_config_t io_config;
    tmc2208_motor_config_t motor_config;
} tmc2208_config_t;

/**
 * @brief Initialize the TMC2208 driver with the specified configuration.
 *
 * This function sets up the TMC2208 driver using the provided I/O configuration.
 *
 * @param[in] config Pointer to the I/O configuration structure for the TMC2208 driver.
 *
 * @return
 *     - ESP_OK: Success
 *     - ESP_ERR_INVALID_ARG: Invalid argument
 *     - ESP_FAIL: Initialization failed
 */
esp_err_t tmc2208_init(tmc2208_io_config_t *config);

/**
 * @brief Enable the TMC2208 stepper motor driver.
 *
 * This function initializes and enables the TMC2208 stepper motor driver with the specified
 * I/O and motor configurations.
 *
 * @param[in] config Pointer to the I/O configuration structure for the TMC2208.
 * @param[in] motor_config Pointer to the motor configuration structure for the TMC2208.
 *
 * @return
 *     - ESP_OK: Success
 *     - ESP_ERR_INVALID_ARG: Invalid argument
 *     - ESP_FAIL: Other failure
 */
esp_err_t tmc2208_enable(tmc2208_io_config_t *config, tmc2208_motor_config_t *motor_config);

/**
 * @brief Disable the TMC2208 stepper motor driver.
 *
 * This function disables the TMC2208 stepper motor driver by configuring the 
 * necessary IO pins and motor settings.
 *
 * @param config Pointer to the IO configuration structure.
 * @param motor_config Pointer to the motor configuration structure.
 * 
 * @return 
 *     - ESP_OK: Success
 *     - ESP_ERR_INVALID_ARG: Invalid argument
 *     - ESP_FAIL: Other failures
 */
esp_err_t tmc2208_disable(tmc2208_io_config_t *config, tmc2208_motor_config_t *motor_config);

/**
 * @brief Set the direction of the TMC2208 stepper motor driver.
 *
 * This function configures the direction pin of the TMC2208 stepper motor driver
 * according to the specified direction.
 *
 * @param config Pointer to the TMC2208 I/O configuration structure.
 * @param motor_config Pointer to the TMC2208 motor configuration structure.
 * @param dir The desired direction to set (e.g., forward or reverse).
 *
 * @return
 *     - ESP_OK: Success
 *     - ESP_ERR_INVALID_ARG: Invalid argument
 *     - ESP_FAIL: General failure
 */
esp_err_t tmc2208_set_dir(tmc2208_io_config_t *config, tmc2208_motor_config_t *motor_config, tmc2208_dir_t dir);

/**
 * @brief Move the motor a specified number of steps.
 *
 * This function commands the TMC2208 stepper motor driver to move the motor
 * a given number of steps. The movement can be configured to include
 * acceleration and deceleration phases.
 *
 * @param config Pointer to the TMC2208 I/O configuration structure.
 * @param motor_config Pointer to the TMC2208 motor configuration structure.
 * @param steps Number of steps to move the motor. Positive values move the motor
 *              forward, while negative values move it backward.
 * @param accel Boolean flag to enable acceleration phase.
 * @param decel Boolean flag to enable deceleration phase.
 *
 * @return
 *     - ESP_OK: Success
 *     - ESP_ERR_INVALID_ARG: Invalid argument
 *     - ESP_FAIL: Other failure
 */
esp_err_t tmc2208_move_steps(tmc2208_io_config_t *config, tmc2208_motor_config_t *motor_config, int32_t steps, bool accel, bool decel);

/**
 * @brief Move the motor a specified number of steps uniformly.
 *
 * This function moves the motor connected to the TMC2208 driver a specified
 * number of steps in a uniform manner based on the provided configuration.
 *
 * @param config Pointer to the TMC2208 IO configuration structure.
 * @param motor_config Pointer to the TMC2208 motor configuration structure.
 * @param steps Number of steps to move the motor. Positive values move the motor
 *              forward, while negative values move it backward.
 *
 * @return
 *     - ESP_OK: Success
 *     - ESP_ERR_INVALID_ARG: Invalid argument
 *     - ESP_FAIL: Other failures
 */
esp_err_t tmc2208_uniform_move_steps(tmc2208_io_config_t *config, tmc2208_motor_config_t *motor_config, int32_t steps);

#ifdef __cplusplus
}
#endif
