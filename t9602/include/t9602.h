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

#define I2C_ADDR_0 0x4c
#define I2C_MASTER_NUM I2C_NUM_0

typedef void *t9602_handle_t;

#ifdef __cplusplus
}
#endif
