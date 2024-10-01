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

#define T9602_I2C_ADDR_0    0x28    // I2C address of T9602 sensor

// T9602 register addresses
#define T9602_DATA_REG      0x00    // Data register

i2c_master_dev_handle_t t9602_device_create(i2c_master_bus_handle_t bus_handle, const uint16_t dev_addr, const uint32_t dev_speed);
esp_err_t t9602_device_delete(i2c_master_dev_handle_t dev_handle);
void t9602_get_data(i2c_master_dev_handle_t dev_handle, float *temperature, float *humidity);

#ifdef __cplusplus
}
#endif
