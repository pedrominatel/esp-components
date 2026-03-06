/*
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief t9602 driver
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/i2c_master.h"
#include "driver/gpio.h"

#if __has_include("i2c_bus.h")
#include "i2c_bus.h"
#endif

#define T9602_I2C_ADDR_0    0x28    // I2C address of T9602 sensor

// T9602 register addresses
#define T9602_DATA_REG      0x00    // Data register

/**
 * @brief Opaque handle for T9602 device
 */
typedef struct t9602_dev_t *t9602_handle_t;

/**
 * @brief Creates a T9602 handle using the native IDF i2c_master driver.
 *
 * @param bus_handle Handle to the I2C master bus.
 * @param dev_addr   7-bit I2C address of the T9602 sensor.
 * @param dev_speed  SCL speed in Hz.
 * @return t9602_handle_t on success, NULL on failure.
 */
t9602_handle_t t9602_device_create(i2c_master_bus_handle_t bus_handle, const uint16_t dev_addr, const uint32_t dev_speed);

/**
 * @brief Deletes a T9602 device handle and releases resources.
 *
 * @param dev_handle Handle returned by t9602_device_create or t9602_device_create_i2cbus.
 * @return ESP_OK on success, or an error code.
 */
esp_err_t t9602_device_delete(t9602_handle_t dev_handle);

/**
 * @brief Reads temperature and humidity from the T9602 sensor.
 *
 * @param dev_handle  Handle to the T9602 device.
 * @param temperature Pointer to store the temperature value (°C).
 * @param humidity    Pointer to store the relative humidity value (%).
 */
void t9602_get_data(t9602_handle_t dev_handle, float *temperature, float *humidity);

#if __has_include("i2c_bus.h")
/**
 * @brief Creates a T9602 handle from an existing i2c_bus device handle.
 *
 * Use this function when the rest of your application already uses the
 * espressif/i2c_bus component to manage the I2C bus.
 *
 * @param bus_dev Handle to the i2c_bus device (already created by the caller).
 * @return t9602_handle_t on success, NULL on failure.
 */
t9602_handle_t t9602_device_create_i2cbus(i2c_bus_device_handle_t bus_dev);
#endif

#ifdef __cplusplus
}
#endif
