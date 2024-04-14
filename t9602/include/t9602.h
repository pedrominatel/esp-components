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

#include "driver/i2c.h"
#include "driver/gpio.h"

#define T9602_I2C_ADDR_0 0x28
#define I2C_MASTER_NUM I2C_NUM_0

// T9602 register addresses
#define T9602_DATA_REG 0x00

typedef void *t9602_handle_t;

t9602_handle_t t9602_create(i2c_port_t port, const uint16_t dev_addr);
t9602_handle_t t9602_get_data(t9602_handle_t sensor, float *temperature, float *humidity);

#ifdef __cplusplus
}
#endif
