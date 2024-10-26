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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "sht4x.h"

static const char *TAG = "SHT4X";

i2c_master_dev_handle_t sht4x_device_create(i2c_master_bus_handle_t bus_handle, const uint16_t dev_addr, const uint32_t dev_speed)
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

esp_err_t sht4x_start_measurement(i2c_master_dev_handle_t dev_handle, sht4x_measurement_mode mode)
{
    
    esp_err_t ret;
    uint8_t th_reg[1] = { SHT4X_CMD_READ_MEASUREMENT_HIGH };

    ret = i2c_master_transmit(dev_handle, th_reg, 1, -1);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write to the sensor");
        return ret;
    }
        
    return ESP_OK;
}

esp_err_t sht4x_read_measurement(i2c_master_dev_handle_t dev_handle, float *temperature, float *humidity)
{
    
    esp_err_t ret;
    uint8_t raw_values[6] = {0};

    ret = i2c_master_receive(dev_handle, raw_values, 6, -1);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read temperature and humidity");
        return ret;
    }
    
    // Convert raw values to temperature and humidity
    uint16_t raw_temp = (raw_values[0] << 8) | raw_values[1];
    uint16_t raw_hum = (raw_values[3] << 8) | raw_values[4];

    *temperature = 175.0f * (float)raw_temp / 65535.0f - 45.0f;
    *humidity = 100.0f * (float)raw_hum / 65535.0f;
    
    return ESP_OK;
}

esp_err_t sht4x_device_delete(i2c_master_dev_handle_t dev_handle)
{
    return i2c_master_bus_rm_device(dev_handle);
}
