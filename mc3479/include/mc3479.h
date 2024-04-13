/*
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief mc3479 driver
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/i2c.h"
#include "driver/gpio.h"

#define MC3479_I2C_ADDR_0 0x4c // Address selection pin of the MC3479 to GND
#define MC3479_I2C_ADDR_1 0x6c // Address selection pin of the MC3479 to VDD
#define MC3479_I2C_MASTER_NUM I2C_NUM_0

// MC3479 register addresses
// Reserved
#define MC3479_DEV_STATUS_REG           0x05 // Device status register
#define MC3479_INTR_CTRL                0x06 // Interrupt control
#define MC3479_MODE_CTRL                0x07 // Mode control
#define MC3479_SAMPLE_RATE              0x08 // Sample rate
#define MC3479_MOTION_CTRL              0x09 // Motion control
#define MC3479_FIFO_STATUS              0x0A // FIFO status
#define MC3479_FIFO_RD_P                0x0B // FIFO read pointer
#define MC3479_FIFO_WR_P                0x0C // FIFO write pointer
#define MC3479_XOUT_EX_L                0x0D // Continous X-axis LSB output
#define MC3479_XOUT_EX_H                0x0E // Continous X-axis MSB output
#define MC3479_YOUT_EX_L                0x0F // Continous Y-axis LSB output
#define MC3479_YOUT_EX_H                0x10 // Continous Y-axis MSB output
#define MC3479_ZOUT_EX_L                0x11 // Continous Z-axis LSB output
#define MC3479_ZOUT_EX_H                0x12 // Continous Z-axis MSB output
#define MC3479_STATUS_REG               0x13 // Status register
#define MC3479_INTR_STATUS              0x14 // Interrupt status
// Reserved
#define MC3479_CHIP_ID                  0x18 // Chip ID
#define MC3479_RANGE                    0x20 // Range Select
#define MC3479_XOFFL                    0x21 // X-axis offset LSB
#define MC3479_XOFFH                    0x22 // X-axis offset MSB
#define MC3479_YOFFL                    0x23 // Y-axis offset LSB
#define MC3479_YOFFH                    0x24 // Y-axis offset MSB
#define MC3479_ZOFFL                    0x25 // Z-axis offset LSB
#define MC3479_ZOFFH                    0x26 // Z-axis offset MSB
#define MC3479_XGAIN                    0x27 // X-axis gain
#define MC3479_YGAIN                    0x28 // Y-axis gain
#define MC3479_ZGAIN                    0x29 // Z-axis gain
// Reserved
#define MC3479_FIFO_CTRL                0x2D // FIFO control
#define MC3479_FIFO_THS                 0x2E // FIFO threshold
#define MC3479_FIFO_INTR                0x2F // FIFO interrupt
#define MC3479_FIFO_CTRL2_SR2           0x30 // FIFO control 2 and sample rate 2
#define MC3479_COMM_CTRL                0x31 // Communication control
// Reserved
#define MC3479_GPIO_CTRL                0x33 // GPIO control
// Reserved
#define MC3479_TF_THRESH_LSB            0x40 // Tilt and flip threshold
#define MC3479_TF_THRESH_MSB            0x41 // Tilt and flip threshold
#define MC3479_TF_DB                    0x42 // Tilt and flip debounce
#define MC3479_AM_THRESH_LSB            0x43 // Anymotion threshold
#define MC3479_AM_THRESH_MSB            0x44 // Anymotion threshold
#define MC3479_AM_DB                    0x45 // Anymotion debounce
#define MC3479_SHK_THRESH_LSB           0x46 // Shake threshold
#define MC3479_SHK_THRESH_MSB           0x47 // Shake threshold
#define MC3479_PK_P2P_DUR_TH_L          0x48 // Peak to peak duration threshold
#define MC3479_PK_P2P_DUR_TH_H          0x49 // Peak to peak duration threshold
#define MC3479_TIMER_CTRL               0x4A // Timer control
#define MC3479_RD_CNT                   0x4B // Read counter
// Reserved

#define MC3479_DEFAULT_CHIP_ID          0xa4 // Chip ID
#define MC3479_DEFAULT_MODE             MC3479_MODE_CWAKE // Default mode
#define MC3479_DEFAULT_RANGE            MC3479_RANGE_2G // Default range
#define MC3479_DEFAULT_SAMPLE_RATE      MC3479_SAMPLE_100Hz // Default sample rate

typedef enum
{
  MC3479_MODE_SLEEP     = 0b000,
  MC3479_MODE_CWAKE     = 0b001,
  MC3479_MODE_RESERVED  = 0b010,
  MC3479_MODE_STANDBY   = 0b011,
} mc3479_mode_t;

typedef enum
{
  MC3479_RANGE_2G    = 0b000,
  MC3479_RANGE_4G    = 0b001,
  MC3479_RANGE_8G    = 0b010,
  MC3479_RANGE_16G   = 0b011,
  MC3479_RANGE_12G   = 0b100,
}   mc3479_range_t;

typedef enum
{
  MC3479_SAMPLE_50Hz        = 0x10,
  MC3479_SAMPLE_100Hz       = 0x11,
  MC3479_SAMPLE_125Hz       = 0x12,
  MC3479_SAMPLE_200Hz       = 0x13,
  MC3479_SAMPLE_250Hz       = 0x14,
  MC3479_SAMPLE_500Hz       = 0x15,
  MC3479_SAMPLE_1000Hz      = 0x16,
  MC3479_SAMPLE_2000Hz      = 0x16,
}   mc3479_sample_t;

typedef struct
{
  bool TF;
  bool MOTION_LATCH;
  bool ANY_MOTION;
  bool SHAKE;
  bool TILT_35;
  bool Z_AXIS_ORT;
  bool RAW_PROC_STAT;
  bool MOTION_RESET;
}   mc3479_motion_t;

typedef struct
{
  bool TILT_INT;
  bool FLIP_INT;
  bool ANYM_INT;
  bool SHAKE_INT;
  bool TILT_35_INT;
  bool RESERVED;
  bool AUTO_CLR;
  bool ACQ_INT;
}   mc3479_motion_intr_t;

typedef struct
{
  bool TILT_INT;
  bool FLIP_INT;
  bool ANYM_INT;
  bool SHAKE_INT;
  bool TILT_35_INT;
  bool FIFO;
  bool RESERVED;
  bool ACQ_INT;
}   mc3479_motion_intr_status_t;

// Configuration structure
typedef struct
{
  uint8_t mode;
  uint8_t range;
  uint8_t sample_rate;
}   mc3479_config_t;

typedef struct
{
  bool RESERVED_0;
  bool RESERVED_1;
  bool GPIO1_INTN1_IAH;
  bool GPIO1_INTN1_IPP;
  bool RESERVED_4;
  bool RESERVED_5;
  bool GPIO2_INTN2_IAH;
  bool GPIO2_INTN2_IPP;
}   mc3479_gpio_intr_t;

typedef void *mc3479_handle_t;

esp_err_t mc3479_init();

mc3479_handle_t mc3479_create(i2c_port_t port, const uint16_t dev_addr);

void mc3479_delete(mc3479_handle_t sensor);
esp_err_t mc3479_get_chip_id(mc3479_handle_t sensor, uint8_t *chip_id);

esp_err_t mc3479_get_mode(mc3479_handle_t sensor, uint8_t *mode);
esp_err_t mc3479_set_mode(mc3479_handle_t sensor, uint8_t mode);

esp_err_t mc3479_get_acceleration(mc3479_handle_t sensor, int16_t *x, int16_t *y, int16_t *z);

esp_err_t mc3479_get_range(mc3479_handle_t sensor, uint8_t *range);
esp_err_t mc3479_set_range(mc3479_handle_t sensor, uint8_t range);

esp_err_t mc3479_get_sample_rate(mc3479_handle_t sensor, uint8_t *range);
esp_err_t mc3479_set_sample_rate(mc3479_handle_t sensor, uint8_t range);

esp_err_t mc3479_get_motion(mc3479_handle_t sensor, uint8_t *motion);
esp_err_t mc3479_set_motion(mc3479_handle_t sensor, mc3479_motion_t motion);

esp_err_t mc3479_get_motion_intr(mc3479_handle_t sensor, uint8_t *motion_intr);
esp_err_t mc3479_set_motion_intr(mc3479_handle_t sensor, mc3479_motion_intr_t motion_intr);

esp_err_t mc3479_set_gpio_intr(mc3479_handle_t sensor, mc3479_gpio_intr_t gpio_intr);

esp_err_t mc3479_get_status_reg(mc3479_handle_t sensor, uint8_t *status_reg);
esp_err_t mc3479_get_interrupt_status_reg(mc3479_handle_t sensor, uint8_t *intr_status_reg);
esp_err_t mc3479_clean_all_interrupts(mc3479_handle_t sensor);

#ifdef __cplusplus
}
#endif
