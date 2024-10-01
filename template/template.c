/*
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stddef.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include "esp_log.h"
#include "esp_err.h"
#include "template.h"


i2c_master_dev_handle_t template_device_create(i2c_master_bus_handle_t bus_handle, const uint16_t dev_addr, const uint32_t dev_speed)
{

    if (bus_handle == NULL) {
        return NULL;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = dev_addr,
        .scl_speed_hz = dev_speed,
    };

    i2c_master_dev_handle_t dev_handle = NULL;

    // Add device to the I2C bus
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));

    return dev_handle;
}

esp_err_t template_device_delete(i2c_master_dev_handle_t dev_handle)
{
    return i2c_master_bus_rm_device(dev_handle);
}



void template_read_data(i2c_master_dev_handle_t dev_handle, uint8_t *data, size_t len)
{
    if (dev_handle == NULL) {
        return;
    }

    // Read data from the sensor
    ESP_ERROR_CHECK(i2c_master_receive(dev_handle, data, len, -1));

}

void template_data_write(i2c_master_dev_handle_t dev_handle, uint8_t *data, size_t len)
{
    if (dev_handle == NULL) {
        return;
    }

    // Write data to the sensor
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, data, len, -1));
}

void template_data_write_read(i2c_master_dev_handle_t dev_handle, uint8_t *data, size_t len, uint8_t *data_rd, size_t len_rd)
{
    if (dev_handle == NULL) {
        return;
    }

    // Write data to the sensor
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, data, len, data_rd, len_rd, -1));

}