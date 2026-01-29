#ifndef MAX17043_H
#define MAX17043_H

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/* I2C Address */
#define MAX17043_I2C_ADDR 0x36

/* Register Addresses */
#define MAX17043_REG_VCELL    0x02
#define MAX17043_REG_SOC      0x04
#define MAX17043_REG_MODE     0x06
#define MAX17043_REG_VERSION  0x08
#define MAX17043_REG_HIBRT    0x0A
#define MAX17043_REG_CONFIG   0x0C
#define MAX17043_REG_VALRT    0x14
#define MAX17043_REG_CRATE    0x16
#define MAX17043_REG_VRESET   0x18
#define MAX17043_REG_STATUS   0x1A
#define MAX17043_REG_TABLE    0x40
#define MAX17043_REG_CMD      0xFE

/* Device handle */
typedef void* max17043_handle_t;

/* Configuration structure */
typedef struct {
    i2c_master_bus_handle_t i2c_bus;
    uint8_t i2c_addr;
} max17043_config_t;

/**
 * @brief Create MAX17043 device handle
 * @param bus_handle I2C bus handle
 * @param i2c_addr I2C address
 * @return Device handle on success, NULL on failure
 */
max17043_handle_t max17043_create(i2c_master_bus_handle_t bus_handle, uint8_t i2c_addr);

/**
 * @brief Delete MAX17043 device handle
 * @param handle Device handle
 * @return ESP_OK on success
 */
esp_err_t max17043_delete(max17043_handle_t handle);

/**
 * @brief Read battery voltage
 * @param handle Device handle
 * @param voltage Pointer to store voltage in mV
 * @return ESP_OK on success
 */
esp_err_t max17043_read_voltage(max17043_handle_t handle, uint16_t *voltage);

/**
 * @brief Read battery state of charge
 * @param handle Device handle
 * @param soc Pointer to store SOC in percentage
 * @return ESP_OK on success
 */
esp_err_t max17043_read_soc(max17043_handle_t handle, uint8_t *soc);

/**
 * @brief Read battery version
 * @param handle Device handle
 * @param version Pointer to store version
 * @return ESP_OK on success
 */
esp_err_t max17043_read_version(max17043_handle_t handle, uint16_t *version);

/**
 * @brief Set alert threshold
 * @param handle Device handle
 * @param threshold Alert threshold in percentage (0-32%)
 * @return ESP_OK on success
 */
esp_err_t max17043_set_alert_threshold(max17043_handle_t handle, uint8_t threshold);

/**
 * @brief Get alert status
 * @param handle Device handle
 * @param alert Pointer to store alert status (1 = alert active, 0 = no alert)
 * @return ESP_OK on success
 */
esp_err_t max17043_get_alert_status(max17043_handle_t handle, uint8_t *alert);

/**
 * @brief Clear alert status
 * @param handle Device handle
 * @return ESP_OK on success
 */
esp_err_t max17043_clear_alert(max17043_handle_t handle);

/**
 * @brief Enter sleep mode
 * @param handle Device handle
 * @return ESP_OK on success
 */
esp_err_t max17043_sleep(max17043_handle_t handle);

/**
 * @brief Exit sleep mode
 * @param handle Device handle
 * @return ESP_OK on success
 */
esp_err_t max17043_wake(max17043_handle_t handle);

/**
 * @brief Quick start the fuel gauge
 * @param handle Device handle
 * @return ESP_OK on success
 */
esp_err_t max17043_quick_start(max17043_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif /* MAX17043_H */
