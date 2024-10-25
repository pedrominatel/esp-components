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

#define SHT4X_CMD_READ_MEASUREMENT_HIGH     0xFD // Read measurement command
#define SHT4X_CMD_READ_MEASUREMENT_MEDIUM   0xF6 // Read measurement command
#define SHT4X_CMD_READ_MEASUREMENT_LOW      0xE0 // Read measurement command
#define SHT4X_CMD_READ_SERIAL_NUMBER        0x89 // Read serial number command
#define SHT4X_CMD_SOFT_RESET                0x94 // Soft reset command

i2c_master_dev_handle_t sht4x_device_create(i2c_master_bus_handle_t bus_handle, const uint16_t dev_addr, const uint32_t dev_speed);
esp_err_t sht4x_device_delete(i2c_master_dev_handle_t dev_handle);
esp_err_t sht4x_start_measurement(i2c_master_dev_handle_t dev_handle);
esp_err_t sht4x_read_measurement(i2c_master_dev_handle_t dev_handle, float *temperature, float *humidity);

#ifdef __cplusplus
}
#endif