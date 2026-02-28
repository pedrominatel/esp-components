/*
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "nt3h2111.h"

static const char *TAG = "NT3H2111";

nt3h2111_handle_t nt3h2111_device_create(i2c_master_bus_handle_t bus_handle, const uint16_t dev_addr, const uint32_t dev_speed)
{
    /* Probe device before adding to bus */
    esp_err_t ret = i2c_master_probe(bus_handle, dev_addr, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NT3H2111 device not found at address 0x%02X: %s", dev_addr, esp_err_to_name(ret));
        return NULL;
    }
    ESP_LOGI(TAG, "NT3H2111 device found at address 0x%02X", dev_addr);

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = dev_addr,
        .scl_speed_hz = dev_speed,
    };

    i2c_master_dev_handle_t dev_handle = NULL;

    // Add device to the I2C bus
    ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add NT3H2111 device to I2C bus: %s", esp_err_to_name(ret));
        return NULL;
    }

    ESP_LOGI(TAG, "NT3H2111 device created successfully");
    return dev_handle;
}

esp_err_t nt3h2111_device_delete(nt3h2111_handle_t dev_handle)
{
    return i2c_master_bus_rm_device(dev_handle);
}

esp_err_t nt3h2111_read_block(nt3h2111_handle_t dev_handle, uint8_t block_addr, uint8_t *data)
{
    ESP_RETURN_ON_FALSE(data, ESP_ERR_INVALID_ARG, TAG, "Invalid pointer: data cannot be NULL");

    esp_err_t ret;

    // Write block address
    ret = i2c_master_transmit(dev_handle, &block_addr, 1, -1);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to write block address");

    // Read 16 bytes from the block
    ret = i2c_master_receive(dev_handle, data, NT3H2111_BLOCK_SIZE, -1);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to read block data");

    return ESP_OK;
}

esp_err_t nt3h2111_write_block(nt3h2111_handle_t dev_handle, uint8_t block_addr, const uint8_t *data)
{
    ESP_RETURN_ON_FALSE(data, ESP_ERR_INVALID_ARG, TAG, "Invalid pointer: data cannot be NULL");

    esp_err_t ret;
    uint8_t write_buf[17];  // 1 byte address + 16 bytes data

    // Prepare write buffer: address + data
    write_buf[0] = block_addr;
    memcpy(&write_buf[1], data, NT3H2111_BLOCK_SIZE);

    // Write block address + data
    ret = i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), -1);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to write block");

    return ESP_OK;
}

esp_err_t nt3h2111_read_session_regs(nt3h2111_handle_t dev_handle, uint8_t *data)
{
    return nt3h2111_read_block(dev_handle, NT3H2111_SESSION_REGS, data);
}

esp_err_t nt3h2111_read_config(nt3h2111_handle_t dev_handle, uint8_t *config)
{
    return nt3h2111_read_block(dev_handle, NT3H2111_CONFIG_BLOCK, config);
}

esp_err_t nt3h2111_write_config(nt3h2111_handle_t dev_handle, const uint8_t *config)
{
    return nt3h2111_write_block(dev_handle, NT3H2111_CONFIG_BLOCK, config);
}

esp_err_t nt3h2111_is_field_present(nt3h2111_handle_t dev_handle, bool *field_present)
{
    ESP_RETURN_ON_FALSE(field_present, ESP_ERR_INVALID_ARG, TAG, "Invalid pointer: field_present cannot be NULL");

    uint8_t session_regs[NT3H2111_BLOCK_SIZE];
    esp_err_t ret = nt3h2111_read_session_regs(dev_handle, session_regs);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to read session registers");

    // Byte 0, bit 0: Field detect (FD_ON)
    *field_present = (session_regs[0] & 0x01) ? true : false;

    return ESP_OK;
}

esp_err_t nt3h2111_init_field_detect_gpio(gpio_num_t fd_gpio, nt3h2111_fd_callback_t callback, void *callback_arg)
{
    ESP_RETURN_ON_FALSE(GPIO_IS_VALID_GPIO(fd_gpio), ESP_ERR_INVALID_ARG, TAG, "Invalid FD GPIO number");

    // Configure FD pin as input with pull-up (FD is active LOW)
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << fd_gpio),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,  // Interrupt on both edges
    };
    
    esp_err_t ret = gpio_config(&io_conf);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to configure FD GPIO");

    if (callback != NULL) {
        // Install GPIO ISR service if not already installed
        ret = gpio_install_isr_service(0);
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
            ESP_RETURN_ON_ERROR(ret, TAG, "Failed to install GPIO ISR service");
        }

        // Add ISR handler for this GPIO
        ret = gpio_isr_handler_add(fd_gpio, (gpio_isr_t)callback, callback_arg);
        ESP_RETURN_ON_ERROR(ret, TAG, "Failed to add ISR handler");
        
        ESP_LOGI(TAG, "Field detection GPIO%d initialized with interrupt", fd_gpio);
    } else {
        ESP_LOGI(TAG, "Field detection GPIO%d initialized (polling mode)", fd_gpio);
    }

    return ESP_OK;
}

esp_err_t nt3h2111_read_fd_pin(gpio_num_t fd_gpio, bool *field_present)
{
    ESP_RETURN_ON_FALSE(field_present, ESP_ERR_INVALID_ARG, TAG, "Invalid pointer: field_present cannot be NULL");
    ESP_RETURN_ON_FALSE(GPIO_IS_VALID_GPIO(fd_gpio), ESP_ERR_INVALID_ARG, TAG, "Invalid FD GPIO number");

    // FD pin is active LOW - 0 means field present
    int level = gpio_get_level(fd_gpio);
    *field_present = (level == 0);

    return ESP_OK;
}
