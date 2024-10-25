
/*
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief lis3dh driver
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/i2c_master.h"

// LIS3DH I2C address
#define LIS3DH_I2C_H_ADDR          0x18 // I2C address of LIS3DH sensor
#define LIS3DH_I2C_L_ADDR          0x19 // I2C address of LIS3DH sensor
// LIS3DH register addresses
#define LIS3DH_STATUS_REG_AUX      0x07 // Auxiliary status register
#define LIS3DH_WHO_AM_I            0x0f // Device identification register

#define LIS3DH_CTRL_REG1           0x20 // Control register 1
#define LIS3DH_CTRL_REG2           0x21 // Control register 2
#define LIS3DH_CTRL_REG3           0x22 // Control register 3
#define LIS3DH_CTRL_REG4           0x23 // Control register 4
#define LIS3DH_CTRL_REG5           0x24 // Control register 5
#define LIS3DH_CTRL_REG6           0x25 // Control register 6
#define LIS3DH_REFERENCE           0x26 // Reference register

#define LIS3DH_STATUS_REG          0x27 // Status register
#define LIS3DH_OUT_X_L             0x28 // X-axis acceleration data (LSB)
#define LIS3DH_OUT_X_H             0x29 // X-axis acceleration data (MSB)
#define LIS3DH_OUT_Y_L             0x2a // Y-axis acceleration data (LSB)
#define LIS3DH_OUT_Y_H             0x2b // Y-axis acceleration data (MSB)
#define LIS3DH_OUT_Z_L             0x2c // Z-axis acceleration data (LSB)
#define LIS3DH_OUT_Z_H             0x2d // Z-axis acceleration data (MSB)

#define LIS3DH_TEMP_CFG_REG        0x1f // Temperature sensor configuration register

#define LIS3DH_FIFO_CTRL_REG       0x2e // FIFO control register
#define LIS3DH_FIFO_SRC_REG        0x2f // FIFO source register
#define LIS3DH_INT1_CFG            0x30 // Interrupt 1 configuration register
#define LIS3DH_INT1_SRC            0x31 // Interrupt 1 source register
#define LIS3DH_INT1_THS            0x32 // Interrupt 1 threshold register
#define LIS3DH_INT1_DURATION       0x33 // Interrupt 1 duration register
#define LIS3DH_INT2_CFG            0x34 // Interrupt 2 configuration register
#define LIS3DH_INT2_SRC            0x35 // Interrupt 2 source register
#define LIS3DH_INT2_THS            0x36 // Interrupt 2 threshold register
#define LIS3DH_INT2_DURATION       0x37 // Interrupt 2 duration register
#define LIS3DH_CLICK_CFG           0x38 // Click configuration register
#define LIS3DH_CLICK_SRC           0x39 // Click source register
#define LIS3DH_CLICK_THS           0x3a // Click threshold register

// Functions

i2c_master_dev_handle_t lis3dh_device_create(i2c_master_bus_handle_t bus_handle, const uint16_t dev_addr, const uint32_t dev_speed);
esp_err_t lis3dh_get_who_am_i(i2c_master_dev_handle_t dev_handle);
esp_err_t lis3dh_get_raw_data(i2c_master_dev_handle_t dev_handle, int16_t *raw_x, int16_t *raw_y, int16_t *raw_z);
esp_err_t lis3dh_reset(i2c_master_dev_handle_t dev_handle);
esp_err_t lis3dh_get_fifo_src(i2c_master_dev_handle_t dev_handle, uint8_t *fifo_src);
esp_err_t lis3dh_set_bypass_mode(i2c_master_dev_handle_t dev_handle);

#ifdef __cplusplus
}
#endif
