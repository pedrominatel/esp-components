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
#include "t9602.h"

/**
 * @brief Creates and configures an I2C device handle for the T9602 sensor.
 *
 * This function initializes an I2C device handle for the T9602 sensor by configuring
 * the device address, and speed. It then adds the device to the specified
 * I2C bus.
 *
 * @param bus_handle Handle to the I2C bus to which the device will be added.
 * @param dev_addr The 7-bit I2C address of the T9602 sensor.
 * @param dev_speed The speed of the I2C bus in Hz.
 *
 * @return Handle to the configured I2C device.
 */
i2c_master_dev_handle_t t9602_device_create(i2c_master_bus_handle_t bus_handle, const uint16_t dev_addr, const uint32_t dev_speed)
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

/**
 * @brief Deletes the T9602 device from the I2C master bus.
 *
 * This function removes the specified T9602 device from the I2C master bus
 * by using the provided device handle.
 *
 * @param[in] dev_handle Handle to the I2C master device to be removed.
 *
 * @return
 *     - ESP_OK: Success
 *     - ESP_ERR_INVALID_ARG: Invalid argument
 *     - ESP_FAIL: Other failures
 */
esp_err_t t9602_device_delete(i2c_master_dev_handle_t dev_handle)
{
    return i2c_master_bus_rm_device(dev_handle);
}

/**
 * @brief Retrieve temperature and humidity data from the T9602 sensor.
 *
 * This function reads 4 bytes of data from the T9602 sensor and calculates
 * the temperature and humidity values using the formula provided in the 
 * sensor's datasheet.
 *
 * @param dev_handle The I2C master device handle.
 * @param temperature Pointer to a float where the calculated temperature will be stored.
 * @param humidity Pointer to a float where the calculated humidity will be stored.
 */
void t9602_get_data(i2c_master_dev_handle_t dev_handle, float *temperature, float *humidity)
{

    uint8_t b_read[4] = {0};
    uint8_t b_write[1] = {T9602_DATA_REG};

    // Read 4 bytes of data from the sensor
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, b_write, sizeof(b_write), b_read, 4, -1));

    // Calculate temperature and humidity by using the formula provided in the datasheet
    *temperature = (float)((b_read[2] * 64) + (b_read[3] >> 2 )) / 16384.0 * 165.0 - 40.0;
    *humidity = (float)(((b_read[0] & 0x3F ) << 8) + b_read[1]) / 16384.0 * 100.0;

}
