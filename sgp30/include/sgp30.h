#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"
#include "driver/i2c_master.h"

/* I2C address (fixed, not configurable) */
#define SGP30_DEFAULT_ADDRESS           0x58

/* SGP30 I2C commands (16-bit, sent MSB first) */
#define SGP30_CMD_INIT_AIR_QUALITY      0x2003
#define SGP30_CMD_MEASURE_AIR_QUALITY   0x2008
#define SGP30_CMD_GET_BASELINE          0x2015
#define SGP30_CMD_SET_BASELINE          0x201E
#define SGP30_CMD_SET_HUMIDITY          0x2061
#define SGP30_CMD_MEASURE_RAW           0x2050
#define SGP30_CMD_GET_SERIAL_ID         0x3682
#define SGP30_CMD_SOFT_RESET            0x0006

/* Measurement result */
typedef struct {
    uint16_t tvoc;   /*!< Total Volatile Organic Compounds in ppb */
    uint16_t eco2;   /*!< Equivalent CO2 concentration in ppm    */
} sgp30_measurement_t;

/* Baseline values used for calibration persistence */
typedef struct {
    uint16_t tvoc_baseline;  /*!< TVOC baseline value */
    uint16_t eco2_baseline;  /*!< eCO2 baseline value */
} sgp30_baseline_t;

/**
 * @brief  Initialize the SGP30 sensor.
 *
 * Adds the sensor to the given I2C master bus, probes for presence, and
 * sends the Init_air_quality command to start the on-chip algorithm.
 *
 * @param[in] bus_handle  Handle to an already-initialized I2C master bus.
 * @return
 *   - ESP_OK on success
 *   - ESP_FAIL if the sensor is not found or the command fails
 */
esp_err_t sgp30_init(i2c_master_bus_handle_t bus_handle);

/**
 * @brief  De-initialize the SGP30 sensor and release the device handle.
 */
void sgp30_denit(void);

/**
 * @brief  Perform an IAQ measurement.
 *
 * Sends Measure_air_quality and reads back TVOC (ppb) and eCO2 (ppm).
 * Must be called once per second for the on-chip algorithm to work correctly.
 *
 * @param[out] measurement  Pointer to struct that receives the results.
 * @return
 *   - ESP_OK on success
 *   - ESP_ERR_INVALID_ARG if measurement is NULL
 *   - ESP_FAIL on I2C error or CRC mismatch
 */
esp_err_t sgp30_measure(sgp30_measurement_t *measurement);

/**
 * @brief  Get the current baseline values from the sensor.
 *
 * @param[out] baseline  Pointer to struct that receives the baseline values.
 * @return  ESP_OK on success, ESP_FAIL on error.
 */
esp_err_t sgp30_get_baseline(sgp30_baseline_t *baseline);

/**
 * @brief  Restore previously saved baseline values into the sensor.
 *
 * Should be called after sgp30_init() when valid stored baselines are available.
 *
 * @param[in] baseline  Pointer to struct containing baseline values to restore.
 * @return  ESP_OK on success, ESP_FAIL on error.
 */
esp_err_t sgp30_set_baseline(const sgp30_baseline_t *baseline);

/**
 * @brief  Send a soft reset to the SGP30.
 *
 * @return  ESP_OK on success, ESP_FAIL on error.
 */
esp_err_t sgp30_soft_reset(void);

#ifdef __cplusplus
}
#endif
