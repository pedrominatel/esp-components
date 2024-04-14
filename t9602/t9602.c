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
#include "t9602.h"

/* t9602 device structure
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
} t9602_dev_t;

/* Write data to the register of the sensor 
    * @param sensor object
    * @param reg_start_addr register address
    * @param data_buf data buffer
    * @param data_len data length
    * @return
    *     - ESP_OK Success
    *     - ESP_FAIL Fail
*/
static esp_err_t t9602_write(t9602_handle_t sensor, const uint8_t reg_start_addr, const uint8_t *const data_buf, const uint8_t data_len)
{
    t9602_dev_t *t9602 = (t9602_dev_t *) sensor;
    esp_err_t ret = ESP_OK;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ret = i2c_master_start(cmd);
    assert(ESP_OK == ret);
    ret = i2c_master_write_byte(cmd, t9602->dev_addr, true);
    assert(ESP_OK == ret);
    ret = i2c_master_write_byte(cmd, reg_start_addr, true);
    assert(ESP_OK == ret);
    ret = i2c_master_write(cmd, data_buf, data_len, true);
    assert(ESP_OK == ret);
    ret = i2c_master_stop(cmd);
    assert(ESP_OK == ret);
    ret = i2c_master_cmd_begin(t9602->bus, cmd, 1000 / portTICK_PERIOD_MS);
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
static esp_err_t t9602_read(t9602_handle_t sensor, const uint8_t reg_start_addr, uint8_t *data_buf, const uint8_t data_len)
{
    
    t9602_dev_t *t9602 = (t9602_dev_t *) sensor;
    esp_err_t ret = ESP_OK;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ret = i2c_master_start(cmd);
    assert(ESP_OK == ret);
    ret = i2c_master_write_byte(cmd, t9602->dev_addr, true);
    assert(ESP_OK == ret);
    ret = i2c_master_write_byte(cmd, reg_start_addr, true);
    assert(ESP_OK == ret);
    ret = i2c_master_start(cmd);
    assert(ESP_OK == ret);
    ret = i2c_master_write_byte(cmd, t9602->dev_addr | 1, true);
    assert(ESP_OK == ret);
    ret = i2c_master_read(cmd, data_buf, data_len, I2C_MASTER_LAST_NACK);
    assert(ESP_OK == ret);
    ret = i2c_master_stop(cmd);
    assert(ESP_OK == ret);
    ret = i2c_master_cmd_begin(t9602->bus, cmd, 1000 / portTICK_PERIOD_MS);
    assert(ESP_OK == ret);
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

/* Delete the sensor object
    * @param sensor object
*/
void t9602_delete(t9602_handle_t sensor)
{
    t9602_dev_t *t9602 = (t9602_dev_t *) sensor;
    free(sensor);
}

/* Create the sensor object
    * @param port I2C port
    * @param dev_addr device address
    * @return sensor object
*/
t9602_handle_t t9602_create(i2c_port_t port, const uint16_t dev_addr)
{
    t9602_dev_t *sensor = (t9602_dev_t *) calloc(1, sizeof(t9602_dev_t));
    sensor->bus = port;
    sensor->dev_addr = dev_addr << 1;
    sensor->counter = 0;
    sensor->dt = 0;
    sensor->timer = (struct timeval *) malloc(sizeof(struct timeval));
    return (t9602_handle_t) sensor;
}

/* Get the sensor data
    * @param sensor object
    * @param temperature temperature value
    * @param humidity humidity value
    * @return sensor object
*/
t9602_handle_t t9602_get_data(t9602_handle_t sensor, float *temperature, float *humidity)
{

    esp_err_t ret = ESP_OK;
    uint8_t data[4] = {0};
    uint16_t raw_temperature = 0;
    uint16_t raw_humidity = 0;

    ret = t9602_read(sensor, T9602_DATA_REG, data, 4);
    assert(ESP_OK == ret);

    raw_temperature = (data[0] << 8) | data[1];
    raw_humidity = (data[3] << 8) | data[4];

    float temp = (float)((data[2] * 64) + (data[3] >> 2 )) / 16384.0 * 165.0 - 40.0;
    float hum = (float)(((data[0] & 0x3F ) << 8) + data[1]) / 16384.0 * 100.0;

    *temperature = temp;
    *humidity = hum;

    return sensor;
}