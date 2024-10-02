/*
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief sht2x driver
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/i2c_master.h"
#include "driver/gpio.h"

#define SHT2X_I2C_ADDR                      0x80    // I2C address of SHT2X sensor

// SHT2X register addresses
#define SHT2X_T_DATA_HOLD_REG               0xE3    // Trigger T measurement hold master
#define SHT2X_RH_DATA_HOLD_REG              0xE5    // Trigger RH measurement hold master
#define SHT2X_T_DATA_REG                    0xF3    // Trigger RH measurement no hold master
#define SHT2X_RH_DATA_REG                   0xF5    // Trigger T measurement no hold master
#define SHT2X_WR_USER_REG                   0xE6    // Write user register
#define SHT2X_RD_USER_REG                   0xE7    // Read user register
#define SHT2X_SOFT_RESET                    0xFE    // Soft reset
// SHT2X user register bits
#define SHT2X_USER_REG_RESOLUTION_12_14BIT  0x00    // RH=12bit, T=14bit
#define SHT2X_USER_REG_RESOLUTION_8_12BIT   0x01    // RH= 8bit, T=12bit
#define SHT2X_USER_REG_RESOLUTION_10_13BIT  0x80    // RH=10bit, T=13bit
#define SHT2X_USER_REG_RESOLUTION_11_11BIT  0x81    // RH=11bit, T=11bit
#define SHT2X_USER_REG_RESOLUTION_MASK      0x81    // Mask for resolution bits

i2c_master_dev_handle_t sht2x_device_create(i2c_master_bus_handle_t bus_handle, const uint16_t dev_addr, const uint32_t dev_speed);
esp_err_t sht2x_device_delete(i2c_master_dev_handle_t dev_handle);
void sht2x_get_data(i2c_master_dev_handle_t dev_handle, float *temperature, float *humidity);
void sht2x_get_user_data(i2c_master_dev_handle_t dev_handle);

#ifdef __cplusplus
}
#endif
