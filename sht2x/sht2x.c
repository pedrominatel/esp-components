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
#include "include/sht2x.h"
#include "sht2x.h"

i2c_master_dev_handle_t sht2x_device_create(i2c_master_bus_handle_t bus_handle, const uint16_t dev_addr, const uint32_t dev_speed)
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

esp_err_t sht2x_device_delete(i2c_master_dev_handle_t dev_handle)
{
    return i2c_master_bus_rm_device(dev_handle);
}

void sht2x_get_user_data(i2c_master_dev_handle_t dev_handle)
{
    if (dev_handle == NULL) {
        return;
    }

    uint8_t data[1] = {0};
    uint8_t b_write[1] = { SHT2X_RD_USER_REG };

    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, b_write, 1, -1));

    // ESP_ERROR_CHECK(i2c_master_receive(dev_handle, data, 1, -1));

    printf("User register: 0x%02x\n", data[0]);

}


void sht2x_get_data(i2c_master_dev_handle_t dev_handle, float *temperature, float *humidity)
{

    // uint16_t raw_temperature, raw_humidity;
    // float temp, hum;

    if (dev_handle == NULL) {
        return;
    }

    uint8_t b_read[2] = {0};
    uint8_t b_write[1] = { SHT2X_RD_USER_REG };


    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, b_write, 1, -1));
    
    
    
    // ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, b_write, sizeof(b_write), b_read, 2, -1));





    // raw_temperature = (b_read[0] << 8) | b_read[1];
    // raw_humidity = (data[2] << 8) | data[3];

    // Convert raw temperature to Celsius
    // temp = -46.85 + 175.72 * (float)raw_temperature / 65536.0;
    // Convert raw humidity to percentage
    // hum = -6.0 + 125.0 * (float)raw_humidity / 65536.0;

    // *temperature = temp;
    // *humidity = hum;

}
