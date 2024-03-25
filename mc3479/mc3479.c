/*
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include "esp_system.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_types.h"
#include "mc3479.h"

/* MC3479 device structure
    * @param bus I2C bus
    * @param int_pin interrupt pin
    * @param dev_addr device address
    * @param counter counter
    * @param dt delay time between two measurements
    * @param timer timer
*/
typedef struct {
    i2c_port_t bus;
    gpio_num_t int_pin;
    uint16_t dev_addr;
    uint32_t counter;
    float dt;  /*!< delay time between two measurements, dt should be small (ms level) */
    struct timeval *timer;
} mc3479_dev_t;

/* Write data to the register of the sensor 
    * @param sensor object
    * @param reg_start_addr register address
    * @param data_buf data buffer
    * @param data_len data length
    * @return
    *     - ESP_OK Success
    *     - ESP_FAIL Fail
*/
static esp_err_t mc3479_write(mc3479_handle_t sensor, const uint8_t reg_start_addr, const uint8_t *const data_buf, const uint8_t data_len)
{
    mc3479_dev_t *mc3479 = (mc3479_dev_t *) sensor;
    esp_err_t ret = ESP_OK;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ret = i2c_master_start(cmd);
    assert(ESP_OK == ret);
    ret = i2c_master_write_byte(cmd, mc3479->dev_addr, true);
    assert(ESP_OK == ret);
    ret = i2c_master_write_byte(cmd, reg_start_addr, true);
    assert(ESP_OK == ret);
    ret = i2c_master_write(cmd, data_buf, data_len, true);
    assert(ESP_OK == ret);
    ret = i2c_master_stop(cmd);
    assert(ESP_OK == ret);
    ret = i2c_master_cmd_begin(mc3479->bus, cmd, 1000 / portTICK_PERIOD_MS);
    assert(ESP_OK == ret);
    i2c_cmd_link_delete(cmd);

    return ret;
}

/* Read data from the register of the sensor 
    * @param sensor object
    * @param reg_start_addr register address
    * @param data_buf data buffer
    * @param data_len data length
    * @return
    *     - ESP_OK Success
    *     - ESP_FAIL Fail
*/
static esp_err_t mc3479_read(mc3479_handle_t sensor, const uint8_t reg_start_addr, uint8_t *data_buf, const uint8_t data_len)
{
    
    mc3479_dev_t *mc3479 = (mc3479_dev_t *) sensor;
    esp_err_t ret = ESP_OK;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ret = i2c_master_start(cmd);
    assert(ESP_OK == ret);
    ret = i2c_master_write_byte(cmd, mc3479->dev_addr, true);
    assert(ESP_OK == ret);
    ret = i2c_master_write_byte(cmd, reg_start_addr, true);
    assert(ESP_OK == ret);
    ret = i2c_master_start(cmd);
    assert(ESP_OK == ret);
    ret = i2c_master_write_byte(cmd, mc3479->dev_addr | 1, true);
    assert(ESP_OK == ret);
    ret = i2c_master_read(cmd, data_buf, data_len, I2C_MASTER_LAST_NACK);
    assert(ESP_OK == ret);
    ret = i2c_master_stop(cmd);
    assert(ESP_OK == ret);
    ret = i2c_master_cmd_begin(mc3479->bus, cmd, 1000 / portTICK_PERIOD_MS);
    assert(ESP_OK == ret);
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

/* Delete the sensor object
    * @param sensor object
*/
void mc3479_delete(mc3479_handle_t sensor)
{
    mc3479_dev_t *mc3479 = (mc3479_dev_t *) sensor;
    free(sensor);
}

/* Create the sensor object
    * @param port I2C port
    * @param dev_addr device address
    * @return sensor object
*/
mc3479_handle_t mc3479_create(i2c_port_t port, const uint16_t dev_addr)
{
    mc3479_dev_t *sensor = (mc3479_dev_t *) calloc(1, sizeof(mc3479_dev_t));
    sensor->bus = port;
    sensor->dev_addr = dev_addr << 1;
    sensor->counter = 0;
    sensor->dt = 0;
    sensor->timer = (struct timeval *) malloc(sizeof(struct timeval));
    return (mc3479_handle_t) sensor;
}

/* Get the chip ID of the sensor
    * @param sensor object
    * @param deviceid chip ID
    * @return
    *     - ESP_OK Success
    *     - ESP_FAIL Fail
*/
esp_err_t mc3479_get_chip_id(mc3479_handle_t sensor, uint8_t *const deviceid)
{
    return mc3479_read(sensor, MC3479_CHIP_ID, deviceid, 1);
}

esp_err_t mc3479_get_mode(mc3479_handle_t sensor, uint8_t *mode)
{
    esp_err_t ret = mc3479_read(sensor, MC3479_MODE_CTRL, mode, 1);
    return ret;
}

esp_err_t mc3479_set_mode(mc3479_handle_t sensor, uint8_t mode)
{
    return mc3479_write(sensor, MC3479_MODE_CTRL, &mode, 1);
}

esp_err_t mc3479_get_range(mc3479_handle_t sensor, uint8_t *range)
{
    return mc3479_read(sensor, MC3479_RANGE, range, 1);
}

esp_err_t mc3479_set_range(mc3479_handle_t sensor, uint8_t range)
{
    esp_err_t ret;
    uint8_t value;

    // Set mode to standby before changing range
    ret = mc3479_set_mode(sensor, MC3479_MODE_SLEEP);
    // Read the current range value
    ret = mc3479_read(sensor, MC3479_RANGE, &value, 1);
    value &= 0b00000111;
    value |= (range << 4) & 0x70;
    // Write the new value to the range register
    ret = mc3479_write(sensor, MC3479_RANGE, &value, 1);

    return ret;
}

esp_err_t mc3479_get_sample_rate(mc3479_handle_t sensor, uint8_t *sr)
{
    return mc3479_read(sensor, MC3479_SAMPLE_RATE, sr, 1);
}

esp_err_t mc3479_set_sample_rate(mc3479_handle_t sensor, uint8_t sr)
{
    esp_err_t ret;
    uint8_t value;

    // Set mode to standby before changing range
    ret = mc3479_set_mode(sensor, MC3479_MODE_SLEEP);
    // Read the current rate value
    ret = mc3479_read(sensor, MC3479_SAMPLE_RATE, &value, 1);
    value &= 0b00000000;
    value |= sr;
    ret = mc3479_write(sensor, MC3479_SAMPLE_RATE, &value, 1);
    
    return ret;
}

esp_err_t mc3479_get_status(mc3479_handle_t sensor, uint8_t *status)
{
    return mc3479_read(sensor, MC3479_STATUS_REG, status, 1);
}

esp_err_t mc3479_get_interrupt_status(mc3479_handle_t sensor, uint8_t *status)
{
    return mc3479_read(sensor, MC3479_INTR_STATUS, status, 1);
}

esp_err_t mc3479_get_motion(mc3479_handle_t sensor, uint8_t *motion)
{
    return mc3479_read(sensor, MC3479_MOTION_CTRL, motion, 1);
}

esp_err_t mc3479_set_motion(mc3479_handle_t sensor, mc3479_motion_t motion)
{
    
    esp_err_t ret;
    uint8_t motion_register = 0;

    motion_register |= motion.MOTION_RESET << 7;
    motion_register |= motion.RAW_PROC_STAT << 6;
    motion_register |= motion.Z_AXIS_ORT << 5;
    motion_register |= motion.TILT_35 << 4;
    motion_register |= motion.SHAKE << 3;
    motion_register |= motion.ANY_MOTION << 2;
    motion_register |= motion.MOTION_LATCH << 1;
    motion_register |= motion.TF;

    ret = mc3479_write(sensor, MC3479_MOTION_CTRL, &motion_register, 1);
    
    return ret;
}

esp_err_t mc3479_get_acceleration(mc3479_handle_t sensor, int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t data[6];
    esp_err_t ret = mc3479_read(sensor, MC3479_XOUT_EX_L, data, 6);
    if (ret != ESP_OK) {
        return ret;
    }
    *x = (int16_t) ((data[1] << 8) | data[0]);
    *y = (int16_t) ((data[3] << 8) | data[2]);
    *z = (int16_t) ((data[5] << 8) | data[4]);
    return ESP_OK;
}
