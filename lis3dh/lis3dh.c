/*
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "include/lis3dh.h"
#include "lis3dh.h"

static const char *TAG = "LIS3DH";

i2c_master_dev_handle_t lis3dh_device_create(i2c_master_bus_handle_t bus_handle, const uint16_t dev_addr, const uint32_t dev_speed)
{
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = dev_addr,
        .scl_speed_hz = dev_speed,
    };

    i2c_master_dev_handle_t dev_handle;

    // Add device to the I2C bus
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));

    return dev_handle;
}

esp_err_t lis3dh_device_delete(i2c_master_dev_handle_t dev_handle)
{
    return i2c_master_bus_rm_device(dev_handle);
}

esp_err_t lis3dh_get_who_am_i(i2c_master_dev_handle_t dev_handle)
{
    esp_err_t ret;
    uint8_t read_reg[1] = {LIS3DH_WHO_AM_I};
    uint8_t who_am_i[1];

    ret = i2c_master_transmit_receive(dev_handle, read_reg, 1, who_am_i, 1, -1);

    if(who_am_i[0] == 0x33) {
        ESP_LOGI(TAG, "Device found: LIS3DH");
    } else {
        ESP_LOGE(TAG, "Device found: 0x%02x (unknown)", who_am_i[0]);
    }
    
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to read WHO_AM_I register");

    return ret;
}

esp_err_t lis3dh_reset(i2c_master_dev_handle_t dev_handle)
{
    esp_err_t ret;
    uint8_t read_reg[8] = {LIS3DH_CTRL_REG1};
    read_reg[1] = 0x77;

    ret = i2c_master_transmit(dev_handle, read_reg, 2, -1);
    
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to reset LIS3DH sensor");

    return ret;
}

esp_err_t lis3dh_set_bypass_mode(i2c_master_dev_handle_t dev_handle)
{
    esp_err_t ret;
    uint8_t read_reg[2] = {LIS3DH_CTRL_REG5, 0x00};

    ret = i2c_master_transmit(dev_handle, read_reg, 2, -1);

    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to set bypass mode");

    return ret;
}

esp_err_t lis3dh_get_fifo_src(i2c_master_dev_handle_t dev_handle, uint8_t *fifo_src)
{
    esp_err_t ret;
    uint8_t read_reg[1] = {LIS3DH_CTRL_REG1};

    ret = i2c_master_transmit_receive(dev_handle, read_reg, 1, fifo_src, 1, -1);
    
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to read FIFO_SRC register");

    return ret;
}

esp_err_t lis3dh_get_raw_data(i2c_master_dev_handle_t dev_handle, int16_t *raw_x, int16_t *raw_y, int16_t *raw_z)
{
    esp_err_t ret = ESP_OK;
    uint8_t read_reg[1] = {LIS3DH_OUT_X_L};
    uint8_t raw_data[6] = {0};

    ret = i2c_master_transmit_receive(dev_handle, read_reg, 1, raw_data, 6, 100);

    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read data from the sensor");
        return ret;
    }

    ESP_LOGI(TAG, "Raw data: %02x %02x %02x %02x %02x %02x", raw_data[0], raw_data[1], raw_data[2], raw_data[3], raw_data[4], raw_data[5]);

    *raw_x = (int16_t)(raw_data[0] | (raw_data[1] << 8));
    *raw_y = (int16_t)(raw_data[2] | (raw_data[3] << 8));
    *raw_z = (int16_t)(raw_data[4] | (raw_data[5] << 8));

    return ret;

}
