/*
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief nt3h2111 driver
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/i2c_master.h"
#include "driver/gpio.h"

#if __has_include("i2c_bus.h")
#include "i2c_bus.h"
#endif

// NT3H2111 I2C address
#define NT3H2111_I2C_ADDR           ((uint8_t)0x55)  // 7-bit I2C address

/**
 * @brief Callback function type for field detection events
 *
 * @param arg User argument passed during registration
 */
typedef void (*nt3h2111_fd_callback_t)(void *arg);

// NT3H2111 Memory blocks
#define NT3H2111_BLOCK_SIZE         16               // Each block is 16 bytes
#define NT3H2111_USER_MEMORY_BEGIN  0x01             // User memory starts at block 1
#define NT3H2111_USER_MEMORY_END    0x38             // User memory ends at block 56
#define NT3H2111_CONFIG_BLOCK       0x3A             // Configuration block
#define NT3H2111_SESSION_REGS       0xF8             // Session registers block
#define NT3H2111_SRAM_BEGIN         0xF8             // SRAM buffer start
#define NT3H2111_SRAM_END           0xFB             // SRAM buffer end (4 blocks)

// Configuration register bits
#define NT3H2111_NC_REG_NFCS_I2C_RST_ON_OFF  (1 << 0)  // NFC silent
#define NT3H2111_NC_REG_FD_OFF               (1 << 2)  // Field detect
#define NT3H2111_NC_REG_FD_ON                (1 << 3)
#define NT3H2111_NC_REG_SRAM_MIRROR_ON_OFF   (1 << 4)  // SRAM mirroring
#define NT3H2111_NC_REG_TRANSFER_DIR         (1 << 6)  // Transfer direction

/**
 * @brief NT3H2111 device handle (opaque pointer)
 */
typedef struct nt3h2111_dev_t *nt3h2111_handle_t;

/**
 * @brief Creates a handle for the NT3H2111 device on the specified I2C bus.
 *
 * This function initializes and returns a handle for the NT3H2111 NFC/I2C bridge
 * connected to the given I2C bus.
 *
 * @param bus_handle Handle to the I2C bus where the NT3H2111 device is connected.
 * @param dev_addr I2C address of the NT3H2111 device.
 * @param dev_speed Communication speed for the I2C device.
 * @return Handle to the NT3H2111 device, or NULL if creation failed.
 */
nt3h2111_handle_t nt3h2111_device_create(i2c_master_bus_handle_t bus_handle, const uint16_t dev_addr, const uint32_t dev_speed);

/**
 * @brief Deletes the NT3H2111 device instance.
 *
 * This function releases any resources associated with the NT3H2111 device instance.
 *
 * @param dev_handle The handle to the I2C master device associated with the NT3H2111.
 * 
 * @return
 *     - ESP_OK: Success
 *     - ESP_ERR_INVALID_ARG: Invalid argument
 *     - ESP_FAIL: Failed to delete the device
 */
esp_err_t nt3h2111_device_delete(nt3h2111_handle_t dev_handle);

#if __has_include("i2c_bus.h")
/**
 * @brief Creates an NT3H2111 handle from an existing i2c_bus device handle.
 *
 * Use this function when the rest of your application already uses the
 * espressif/i2c_bus component to manage the I2C bus.
 *
 * @param bus_dev Handle to the i2c_bus device (already created by the caller).
 * @return nt3h2111_handle_t on success, NULL on failure.
 */
nt3h2111_handle_t nt3h2111_device_create_i2cbus(i2c_bus_device_handle_t bus_dev);
#endif

/**
 * @brief Read a 16-byte block from NT3H2111 memory.
 *
 * This function reads one block (16 bytes) from the specified memory address.
 *
 * @param[in] dev_handle Handle to the I2C master device.
 * @param[in] block_addr Block address to read from (0x00-0xFB).
 * @param[out] data Pointer to a buffer where the 16 bytes will be stored.
 *
 * @return
 *     - ESP_OK: Success
 *     - ESP_ERR_INVALID_ARG: Invalid arguments
 *     - ESP_FAIL: Communication with the device failed
 */
esp_err_t nt3h2111_read_block(nt3h2111_handle_t dev_handle, uint8_t block_addr, uint8_t *data);

/**
 * @brief Write a 16-byte block to NT3H2111 memory.
 *
 * This function writes one block (16 bytes) to the specified memory address.
 *
 * @param[in] dev_handle Handle to the I2C master device.
 * @param[in] block_addr Block address to write to (0x00-0xFB).
 * @param[in] data Pointer to the 16 bytes to write.
 *
 * @return
 *     - ESP_OK: Success
 *     - ESP_ERR_INVALID_ARG: Invalid arguments
 *     - ESP_FAIL: Communication with the device failed
 */
esp_err_t nt3h2111_write_block(nt3h2111_handle_t dev_handle, uint8_t block_addr, const uint8_t *data);

/**
 * @brief Read from NT3H2111 session registers.
 *
 * This function reads the session registers which provide status information.
 *
 * @param dev_handle Handle to the I2C master device.
 * @param data Pointer to buffer for 16 bytes of session register data.
 * 
 * @return 
 *     - ESP_OK: Success
 *     - ESP_ERR_INVALID_ARG: Invalid arguments
 *     - ESP_FAIL: Communication failed
 */
esp_err_t nt3h2111_read_session_regs(nt3h2111_handle_t dev_handle, uint8_t *data);

/**
 * @brief Read configuration register.
 *
 * This function reads the configuration register from the NT3H2111.
 *
 * @param dev_handle Handle to the I2C master device.
 * @param config Pointer to buffer for configuration data (16 bytes).
 * 
 * @return 
 *     - ESP_OK: Success
 *     - ESP_ERR_INVALID_ARG: Invalid arguments
 *     - ESP_FAIL: Communication failed
 */
esp_err_t nt3h2111_read_config(nt3h2111_handle_t dev_handle, uint8_t *config);

/**
 * @brief Write configuration register.
 *
 * This function writes the configuration register to the NT3H2111.
 *
 * @param dev_handle Handle to the I2C master device.
 * @param config Pointer to configuration data (16 bytes) to write.
 * 
 * @return 
 *     - ESP_OK: Success
 *     - ESP_ERR_INVALID_ARG: Invalid arguments
 *     - ESP_FAIL: Communication failed
 */
esp_err_t nt3h2111_write_config(nt3h2111_handle_t dev_handle, const uint8_t *config);

/**
 * @brief Check if NFC field is present.
 *
 * This function checks the field detect bit in session registers.
 *
 * @param dev_handle Handle to the I2C master device.
 * @param field_present Pointer to bool that will be set to true if field detected.
 * 
 * @return 
 *     - ESP_OK: Success
 *     - ESP_ERR_INVALID_ARG: Invalid arguments
 *     - ESP_FAIL: Communication failed
 */
esp_err_t nt3h2111_is_field_present(nt3h2111_handle_t dev_handle, bool *field_present);

/**
 * @brief Initialize GPIO-based field detection.
 *
 * Configures the FD pin as input with interrupt on both edges.
 * The FD pin is active LOW when NFC field is present.
 *
 * @param fd_gpio GPIO number connected to NT3H2111 FD pin.
 * @param callback Function to call when field state changes.
 * @param callback_arg User argument passed to callback.
 * 
 * @return 
 *     - ESP_OK: Success
 *     - ESP_ERR_INVALID_ARG: Invalid GPIO number
 *     - ESP_FAIL: GPIO configuration failed
 */
esp_err_t nt3h2111_init_field_detect_gpio(gpio_num_t fd_gpio, nt3h2111_fd_callback_t callback, void *callback_arg);

/**
 * @brief Read current FD pin state directly from GPIO.
 *
 * @param fd_gpio GPIO number connected to NT3H2111 FD pin.
 * @param field_present Pointer to bool (true if field present, FD pin LOW).
 * 
 * @return 
 *     - ESP_OK: Success
 *     - ESP_ERR_INVALID_ARG: Invalid arguments
 */
esp_err_t nt3h2111_read_fd_pin(gpio_num_t fd_gpio, bool *field_present);

#ifdef __cplusplus
}
#endif
