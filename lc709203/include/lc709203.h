#ifndef LC709203_H
#define LC709203_H

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/* I2C Address */
#define LC709203_I2C_ADDR 0x0B

/* Current Direction Values */
#define LC709203_CURRENT_DIR_DISCHARGE  0x0000
#define LC709203_CURRENT_DIR_CHARGE     0x0001
#define LC709203_CURRENT_DIR_AUTO       0xFFFF

/* Command Codes */
#define LC709203_CMD_CELL_TEMP       0x08
#define LC709203_CMD_CELL_VOLTAGE    0x09
#define LC709203_CMD_CURRENT_DIR     0x0A
#define LC709203_CMD_APA             0x0B
#define LC709203_CMD_APT             0x0C
#define LC709203_CMD_RSOC            0x0D
#define LC709203_CMD_ITE             0x0F
#define LC709203_CMD_IC_VERSION      0x11
#define LC709203_CMD_CHANGE_PARAM    0x12
#define LC709203_CMD_ALARM_LOW_RSOC  0x13
#define LC709203_CMD_ALARM_LOW_VOLT  0x14
#define LC709203_CMD_POWER_MODE      0x15
#define LC709203_CMD_STATUS_BIT      0x16
#define LC709203_CMD_INIT_RSOC       0x07

/* Battery Profile Codes */
#define LC709203_APA_100MAH          0x08
#define LC709203_APA_200MAH          0x0B
#define LC709203_APA_500MAH          0x10
#define LC709203_APA_1000MAH         0x19
#define LC709203_APA_2000MAH         0x2D
#define LC709203_APA_3000MAH         0x36

/* Power Modes */
typedef enum {
    LC709203_POWER_MODE_OPERATIONAL = 0x01,
    LC709203_POWER_MODE_SLEEP = 0x02
} lc709203_power_mode_t;

/* Device handle */
typedef void* lc709203_handle_t;

/* Configuration structure */
typedef struct {
    i2c_master_bus_handle_t i2c_bus;
    uint8_t i2c_addr;
} lc709203_config_t;

/**
 * @brief Create LC709203 device handle
 * @param bus_handle I2C bus handle
 * @param i2c_addr I2C address
 * @return Device handle on success, NULL on failure
 */
lc709203_handle_t lc709203_create(i2c_master_bus_handle_t bus_handle, uint8_t i2c_addr);

/**
 * @brief Delete LC709203 device handle
 * @param handle Device handle
 * @return ESP_OK on success
 */
esp_err_t lc709203_delete(lc709203_handle_t handle);

/**
 * @brief Read battery cell voltage
 * @param handle Device handle
 * @param voltage Pointer to store voltage in mV
 * @return ESP_OK on success
 */
esp_err_t lc709203_read_cell_voltage(lc709203_handle_t handle, uint16_t *voltage);

/**
 * @brief Read battery RSOC (Relative State of Charge)
 * @param handle Device handle
 * @param rsoc Pointer to store RSOC in 0.1% units
 * @return ESP_OK on success
 */
esp_err_t lc709203_read_rsoc(lc709203_handle_t handle, uint16_t *rsoc);

/**
 * @brief Read ITE (Indicator to Empty)
 * @param handle Device handle
 * @param ite Pointer to store ITE in 0.1% units
 * @return ESP_OK on success
 */
esp_err_t lc709203_read_ite(lc709203_handle_t handle, uint16_t *ite);

/**
 * @brief Read IC version
 * @param handle Device handle
 * @param version Pointer to store IC version
 * @return ESP_OK on success
 */
esp_err_t lc709203_read_ic_version(lc709203_handle_t handle, uint16_t *version);

/**
 * @brief Set APA (Adjustment Pack Application)
 * @param handle Device handle
 * @param apa APA value (battery capacity code)
 * @return ESP_OK on success
 */
esp_err_t lc709203_set_apa(lc709203_handle_t handle, uint8_t apa);

/**
 * @brief Set cell temperature
 * @param handle Device handle
 * @param temp_celsius Temperature in Celsius
 * @return ESP_OK on success
 */
esp_err_t lc709203_set_cell_temp(lc709203_handle_t handle, uint16_t temp_celsius);

/**
 * @brief Set battery profile
 * @param handle Device handle
 * @param profile Battery profile code (0 = default, 1 = UR18650)
 * @return ESP_OK on success
 */
esp_err_t lc709203_set_battery_profile(lc709203_handle_t handle, uint8_t profile);

/**
 * @brief Set RSOC alarm threshold
 * @param handle Device handle
 * @param alarm_percent Alarm threshold in percentage
 * @return ESP_OK on success
 */
esp_err_t lc709203_set_rsoc_alarm(lc709203_handle_t handle, uint8_t alarm_percent);

/**
 * @brief Set power mode
 * @param handle Device handle
 * @param mode Power mode (operational or sleep)
 * @return ESP_OK on success
 */
esp_err_t lc709203_set_power_mode(lc709203_handle_t handle, lc709203_power_mode_t mode);

/**
 * @brief Initialize RSOC algorithm
 * @param handle Device handle
 * @return ESP_OK on success
 */
esp_err_t lc709203_init_rsoc(lc709203_handle_t handle);

/**
 * @brief Read status register
 * @param handle Device handle
 * @param status Pointer to store status value
 * @return ESP_OK on success
 */
esp_err_t lc709203_read_status(lc709203_handle_t handle, uint16_t *status);

/**
 * @brief Set temperature mode (0x0000 = I2C mode, 0x0001 = Thermistor mode)
 * @param handle Device handle
 * @param mode Temperature mode (0 = I2C, 1 = Thermistor auto-detect)
 * @return ESP_OK on success
 */
esp_err_t lc709203_set_temperature_mode(lc709203_handle_t handle, uint8_t mode);

/**
 * @brief Read cell temperature from sensor
 * @param handle Device handle
 * @param temp_kelvin Pointer to store temperature in 0.1K units
 * @return ESP_OK on success
 */
esp_err_t lc709203_read_cell_temp(lc709203_handle_t handle, uint16_t *temp_kelvin);

/**
 * @brief Read current direction register
 * @param handle Device handle
 * @param direction Pointer to store direction (0x0000=discharge, 0x0001=charge, 0xFFFF=auto)
 * @return ESP_OK on success
 */
esp_err_t lc709203_read_current_direction(lc709203_handle_t handle, uint16_t *direction);

/**
 * @brief Set current direction (for RSOC calculation)
 * @param handle Device handle
 * @param direction Direction value (0x0000=discharge, 0x0001=charge, 0xFFFF=auto)
 * @return ESP_OK on success
 */
esp_err_t lc709203_set_current_direction(lc709203_handle_t handle, uint16_t direction);

/**
 * @brief Trigger sensor to update all measurements (reads temperature register)
 * @param handle Device handle
 * @return ESP_OK on success
 */
esp_err_t lc709203_trigger_update(lc709203_handle_t handle);

/**
 * @brief Detect if thermistor is connected and functioning
 * @param handle Device handle
 * @param thermistor_connected Pointer to store detection result (true if thermistor detected)
 * @return ESP_OK on success
 * @note This function temporarily switches to thermistor mode to test.
 *       If no thermistor is detected, it restores I2C mode with 25°C.
 */
esp_err_t lc709203_detect_thermistor(lc709203_handle_t handle, bool *thermistor_connected);

#ifdef __cplusplus
}
#endif

#endif /* LC709203_H */
