/*
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include "esp_log.h"
#include "esp_err.h"
#include "t9602.h"

static const char *TAG = "T9602";

struct t9602_dev_t {
    i2c_master_dev_handle_t i2c_dev;
#if __has_include("i2c_bus.h")
    i2c_bus_device_handle_t bus_dev;
    bool use_bus;
#endif
};

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
t9602_handle_t t9602_device_create(i2c_master_bus_handle_t bus_handle, const uint16_t dev_addr, const uint32_t dev_speed)
{
    /* Probe device before adding to bus */
    esp_err_t ret = i2c_master_probe(bus_handle, dev_addr, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "T9602 device not found at address 0x%02X: %s", dev_addr, esp_err_to_name(ret));
        return NULL;
    }
    ESP_LOGI(TAG, "T9602 device found at address 0x%02X", dev_addr);

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = dev_addr,
        .scl_speed_hz = dev_speed,
    };

    struct t9602_dev_t *dev = calloc(1, sizeof(struct t9602_dev_t));
    if (!dev) {
        ESP_LOGE(TAG, "Failed to allocate T9602 device");
        return NULL;
    }

    // Add device to the I2C bus
    ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev->i2c_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add T9602 device to I2C bus: %s", esp_err_to_name(ret));
        free(dev);
        return NULL;
    }

    return dev;
}

#if __has_include("i2c_bus.h")
/**
 * @brief Creates a T9602 handle from an existing i2c_bus device handle.
 *
 * @param bus_dev Handle to the i2c_bus device (already created by the caller).
 * @return t9602_handle_t on success, NULL on failure.
 */
t9602_handle_t t9602_device_create_i2cbus(i2c_bus_device_handle_t bus_dev)
{
    if (!bus_dev) {
        ESP_LOGE(TAG, "Invalid i2c_bus device handle");
        return NULL;
    }

    struct t9602_dev_t *dev = calloc(1, sizeof(struct t9602_dev_t));
    if (!dev) {
        ESP_LOGE(TAG, "Failed to allocate T9602 device");
        return NULL;
    }

    dev->bus_dev = bus_dev;
    dev->use_bus = true;

    ESP_LOGI(TAG, "T9602 device created using i2c_bus");
    return dev;
}
#endif

/**
 * @brief Deletes the T9602 device and releases all resources.
 *
 * @param[in] dev_handle Handle to the T9602 device to be removed.
 *
 * @return
 *     - ESP_OK: Success
 *     - ESP_ERR_INVALID_ARG: Invalid argument
 *     - ESP_FAIL: Other failures
 */
esp_err_t t9602_device_delete(t9602_handle_t dev_handle)
{
    if (!dev_handle) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = ESP_OK;
#if __has_include("i2c_bus.h")
    if (dev_handle->use_bus) {
        ret = i2c_bus_device_delete(&dev_handle->bus_dev);
        free(dev_handle);
        return ret;
    }
#endif
    ret = i2c_master_bus_rm_device(dev_handle->i2c_dev);
    free(dev_handle);
    return ret;
}

/**
 * @brief Retrieve temperature and humidity data from the T9602 sensor.
 *
 * This function reads 4 bytes of data from the T9602 sensor and calculates
 * the temperature and humidity values using the formula provided in the 
 * sensor's datasheet.
 *
 * @param dev_handle The T9602 device handle.
 * @param temperature Pointer to a float where the calculated temperature will be stored.
 * @param humidity Pointer to a float where the calculated humidity will be stored.
 */
void t9602_get_data(t9602_handle_t dev_handle, float *temperature, float *humidity)
{
    uint8_t b_read[4] = {0};
    uint8_t b_write[1] = {T9602_DATA_REG};

#if __has_include("i2c_bus.h")
    if (dev_handle->use_bus) {
        // i2c_bus path: explicit write then read (no combined transmit_receive)
        ESP_ERROR_CHECK(i2c_bus_write_bytes(dev_handle->bus_dev, NULL_I2C_MEM_ADDR, sizeof(b_write), b_write));
        ESP_ERROR_CHECK(i2c_bus_read_bytes(dev_handle->bus_dev, NULL_I2C_MEM_ADDR, 4, b_read));
    } else {
#endif
        // Native i2c_master path
        ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle->i2c_dev, b_write, sizeof(b_write), b_read, 4, -1));
#if __has_include("i2c_bus.h")
    }
#endif

    // Calculate temperature and humidity by using the formula provided in the datasheet
    *temperature = (float)((b_read[2] * 64) + (b_read[3] >> 2 )) / 16384.0 * 165.0 - 40.0;
    *humidity = (float)(((b_read[0] & 0x3F ) << 8) + b_read[1]) / 16384.0 * 100.0;
}
