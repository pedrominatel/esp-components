/*
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief sht4x driver
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/i2c_master.h"

#define SHT4X_I2C_ADDR_0                    0x44 // I2C address of SHT4X sensor
#define SHT4X_I2C_ADDR_1                    0x45 // I2C address of SHT4X sensor
#define SHT4X_I2C_ADDR_2                    0x46 // I2C address of SHT4X sensor
#define SHT4X_CMD_READ_SERIAL_NUMBER        0x89 // Read serial number command
#define SHT4X_CMD_SOFT_RESET                0x94 // Soft reset command

/**
 * @enum sht4x_measurement_mode
 * @brief Enumeration of SHT4x sensor commands.
 *
 * This enumeration defines the various commands that can be sent to the SHT4x sensor
 * to perform different operations such as reading measurements at different precision levels,
 * reading the serial number, and performing a soft reset.
 *
 * @var SHT4X_CMD_READ_MEASUREMENT_HIGH
 * Command to read measurement with high precision.
 *
 * @var SHT4X_CMD_READ_MEASUREMENT_MEDIUM
 * Command to read measurement with medium precision.
 *
 * @var SHT4X_CMD_READ_MEASUREMENT_LOW
 * Command to read measurement with low precision.
 *
 */
typedef enum {
    SHT4X_CMD_READ_MEASUREMENT_HIGH         = 0xFD, // Read measurement command
    SHT4X_CMD_READ_MEASUREMENT_MEDIUM       = 0xF6, // Read measurement command
    SHT4X_CMD_READ_MEASUREMENT_LOW          = 0xE0, // Read measurement command
} sht4x_measurement_mode;

/**
 * @brief Creates a handle for the SHT4x device on the specified I2C bus.
 *
 * This function initializes and returns a handle for the SHT4x sensor device
 * connected to the given I2C bus. The device address and communication speed
 * are specified as parameters.
 *
 * @param bus_handle Handle to the I2C bus to which the SHT4x device is connected.
 * @param dev_addr I2C address of the SHT4x device.
 * @param dev_speed Communication speed for the I2C bus in Hz.
 * @return Handle to the SHT4x device, or NULL if the creation fails.
 */
i2c_master_dev_handle_t sht4x_device_create(i2c_master_bus_handle_t bus_handle, const uint16_t dev_addr, const uint32_t dev_speed);

/**
 * @brief Deletes the SHT4x device instance.
 *
 * This function releases any resources associated with the SHT4x device
 * and invalidates the provided device handle.
 *
 * @param dev_handle The handle to the I2C master device instance.
 * 
 * @return
 *     - ESP_OK: Success
 *     - ESP_ERR_INVALID_ARG: Invalid argument
 *     - ESP_FAIL: General failure
 */
esp_err_t sht4x_device_delete(i2c_master_dev_handle_t dev_handle);

/**
 * @brief Start a measurement on the SHT4x sensor.
 *
 * This function initiates a measurement on the SHT4x sensor using the specified
 * measurement mode. The measurement results can be read after the appropriate
 * measurement duration has elapsed.
 *
 * @param[in] dev_handle Handle to the I2C master device.
 * @param[in] mode The measurement mode to use for the SHT4x sensor.
 *
 * @return
 *     - ESP_OK: Success
 *     - ESP_ERR_INVALID_ARG: Invalid argument
 *     - ESP_FAIL: Sending command failed
 */
esp_err_t sht4x_start_measurement(i2c_master_dev_handle_t dev_handle, sht4x_measurement_mode mode);

/**
 * @brief Reads the temperature and humidity measurement from the SHT4x sensor.
 *
 * This function communicates with the SHT4x sensor over I2C to retrieve the
 * current temperature and humidity values. The results are stored in the
 * provided pointers.
 *
 * @param[in] dev_handle The I2C master device handle.
 * @param[out] temperature Pointer to a float where the temperature value will be stored.
 * @param[out] humidity Pointer to a float where the humidity value will be stored.
 *
 * @return
 *     - ESP_OK: Success
 *     - ESP_ERR_INVALID_ARG: Invalid argument
 *     - ESP_FAIL: Communication with the sensor failed
 */
esp_err_t sht4x_read_measurement(i2c_master_dev_handle_t dev_handle, float *temperature, float *humidity);


#ifdef __cplusplus
}
#endif