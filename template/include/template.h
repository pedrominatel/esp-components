/*
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief template driver
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/i2c.h"
#include "driver/gpio.h"

#define I2C_ADDR_0 0x4c // Address selection pin of the MC3479 to GND
#define I2C_ADDR_1 0x6c // Address selection pin of the MC3479 to VDD
#define I2C_MASTER_NUM I2C_NUM_0

typedef void *template_handle_t;

#ifdef __cplusplus
}
#endif
