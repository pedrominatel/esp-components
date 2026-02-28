#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "sgp30.h"

#define TAG "sgp30"

/* CRC-8 parameters per SGP30 datasheet */
#define SGP30_CRC8_POLYNOMIAL  0x31
#define SGP30_CRC8_INIT        0xFF

/* I2C device handle (module-level, single instance) */
static i2c_master_dev_handle_t sgp30_dev_handle = NULL;

/* ---------------------------------------------------------------------------
 * Internal helpers
 * --------------------------------------------------------------------------*/

/**
 * @brief Compute CRC-8 over a data buffer using the SGP30 polynomial.
 */
static uint8_t sgp30_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = SGP30_CRC8_INIT;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ SGP30_CRC8_POLYNOMIAL;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

/**
 * @brief Send a 16-bit command to the SGP30 (big-endian, 2 bytes).
 */
static esp_err_t sgp30_send_command(uint16_t cmd)
{
    uint8_t buf[2] = { (uint8_t)(cmd >> 8), (uint8_t)(cmd & 0xFF) };
    return i2c_master_transmit(sgp30_dev_handle, buf, sizeof(buf), 100);
}

/**
 * @brief Send a command that carries two 16-bit words with CRC (e.g. set_baseline).
 *        Frame: [cmd_msb][cmd_lsb][word0_msb][word0_lsb][crc0][word1_msb][word1_lsb][crc1]
 */
static esp_err_t sgp30_send_command_words(uint16_t cmd, uint16_t word0, uint16_t word1)
{
    uint8_t buf[8];
    buf[0] = (uint8_t)(cmd >> 8);
    buf[1] = (uint8_t)(cmd & 0xFF);
    buf[2] = (uint8_t)(word0 >> 8);
    buf[3] = (uint8_t)(word0 & 0xFF);
    buf[4] = sgp30_crc8(&buf[2], 2);
    buf[5] = (uint8_t)(word1 >> 8);
    buf[6] = (uint8_t)(word1 & 0xFF);
    buf[7] = sgp30_crc8(&buf[5], 2);
    return i2c_master_transmit(sgp30_dev_handle, buf, sizeof(buf), 100);
}

/**
 * @brief Read n_words 16-bit words from the sensor (each followed by a CRC byte).
 *        Validates all CRCs and unpacks the words into @p out.
 */
static esp_err_t sgp30_read_words(uint16_t *out, size_t n_words)
{
    size_t n_bytes = n_words * 3; /* 2 data bytes + 1 CRC per word */
    uint8_t buf[9];               /* max 3 words = 9 bytes */

    if (n_words == 0 || n_words > 3) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = i2c_master_receive(sgp30_dev_handle, buf, n_bytes, 100);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C read error: %s", esp_err_to_name(err));
        return ESP_FAIL;
    }

    for (size_t i = 0; i < n_words; i++) {
        uint8_t *w = &buf[i * 3];
        uint8_t expected_crc = sgp30_crc8(w, 2);
        if (w[2] != expected_crc) {
            ESP_LOGE(TAG, "CRC mismatch on word %u: got 0x%02X, expected 0x%02X",
                     (unsigned)i, w[2], expected_crc);
            return ESP_FAIL;
        }
        out[i] = ((uint16_t)w[0] << 8) | w[1];
    }

    return ESP_OK;
}

/* ---------------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------------*/

esp_err_t sgp30_init(i2c_master_bus_handle_t bus_handle)
{
    esp_err_t err;

    /* Probe the I2C bus to confirm the sensor is present */
    err = i2c_master_probe(bus_handle, SGP30_DEFAULT_ADDRESS, 200);
    if (err == ESP_ERR_NOT_FOUND || err == ESP_ERR_TIMEOUT) {
        ESP_LOGE(TAG, "SGP30 not found on I2C bus!");
        return ESP_FAIL;
    } else if (err == ESP_OK) {
        ESP_LOGI(TAG, "SGP30 found!");
    }

    /* Add the device to the master bus */
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = SGP30_DEFAULT_ADDRESS,
        .scl_speed_hz    = 100000,
    };

    err = i2c_master_bus_add_device(bus_handle, &dev_cfg, &sgp30_dev_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SGP30 device to I2C bus: %s", esp_err_to_name(err));
        return ESP_FAIL;
    }

    /* Send Init_air_quality — starts the on-chip baseline algorithm */
    err = sgp30_send_command(SGP30_CMD_INIT_AIR_QUALITY);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Init_air_quality command failed: %s", esp_err_to_name(err));
        return ESP_FAIL;
    }

    /* The sensor needs ~10 ms after init before the first measurement */
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_LOGI(TAG, "SGP30 initialized. Note: first 15 readings return fixed values (400 ppm eCO2 / 0 ppb TVOC) during sensor warm-up.");
    return ESP_OK;
}

void sgp30_denit(void)
{
    if (sgp30_dev_handle != NULL) {
        i2c_master_bus_rm_device(sgp30_dev_handle);
        sgp30_dev_handle = NULL;
    }
}

esp_err_t sgp30_measure(sgp30_measurement_t *measurement)
{
    if (measurement == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = sgp30_send_command(SGP30_CMD_MEASURE_AIR_QUALITY);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Measure_air_quality command failed: %s", esp_err_to_name(err));
        return ESP_FAIL;
    }

    /* Datasheet: measurement takes ≤12 ms */
    vTaskDelay(pdMS_TO_TICKS(12));

    uint16_t words[2] = {0};
    err = sgp30_read_words(words, 2);
    if (err != ESP_OK) {
        return ESP_FAIL;
    }

    measurement->eco2 = words[0];
    measurement->tvoc = words[1];

    ESP_LOGD(TAG, "eCO2: %u ppm, TVOC: %u ppb", measurement->eco2, measurement->tvoc);
    return ESP_OK;
}

esp_err_t sgp30_get_baseline(sgp30_baseline_t *baseline)
{
    if (baseline == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = sgp30_send_command(SGP30_CMD_GET_BASELINE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Get_baseline command failed: %s", esp_err_to_name(err));
        return ESP_FAIL;
    }

    vTaskDelay(pdMS_TO_TICKS(10));

    uint16_t words[2] = {0};
    err = sgp30_read_words(words, 2);
    if (err != ESP_OK) {
        return ESP_FAIL;
    }

    baseline->eco2_baseline = words[0];
    baseline->tvoc_baseline = words[1];

    ESP_LOGD(TAG, "Baseline — eCO2: 0x%04X, TVOC: 0x%04X",
             baseline->eco2_baseline, baseline->tvoc_baseline);
    return ESP_OK;
}

esp_err_t sgp30_set_baseline(const sgp30_baseline_t *baseline)
{
    if (baseline == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = sgp30_send_command_words(SGP30_CMD_SET_BASELINE,
                                             baseline->eco2_baseline,
                                             baseline->tvoc_baseline);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Set_baseline command failed: %s", esp_err_to_name(err));
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Baseline restored — eCO2: 0x%04X, TVOC: 0x%04X",
             baseline->eco2_baseline, baseline->tvoc_baseline);
    return ESP_OK;
}

esp_err_t sgp30_soft_reset(void)
{
    esp_err_t err = sgp30_send_command(SGP30_CMD_SOFT_RESET);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Soft reset failed: %s", esp_err_to_name(err));
        return ESP_FAIL;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
    return ESP_OK;
}
