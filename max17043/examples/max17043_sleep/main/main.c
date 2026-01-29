#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "max17043.h"
#include "max17043_config.h"

static const char *TAG = "MAX17043_SLEEP";

void app_main(void)
{
    ESP_LOGI(TAG, "Starting MAX17043 sleep/wake example");

    /* Initialize I2C master bus */
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = MAX17043_I2C_PORT,
        .scl_io_num = MAX17043_I2C_SCL_PIN,
        .sda_io_num = MAX17043_I2C_SDA_PIN,
        .glitch_ignore_cnt = 7,
        .flags = {
            .enable_internal_pullup = true,
        },
    };

    i2c_master_bus_handle_t bus_handle;
    esp_err_t ret = i2c_new_master_bus(&bus_config, &bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C bus: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "I2C bus initialized");

    /* Probe the sensor */
    ret = i2c_master_probe(bus_handle, MAX17043_I2C_ADDRESS, 200);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MAX17043 not found at address 0x%02X", MAX17043_I2C_ADDRESS);
        return;
    }

    ESP_LOGI(TAG, "MAX17043 detected on I2C bus");

    /* Create MAX17043 device handle */
    max17043_handle_t sensor_handle = max17043_create(bus_handle, MAX17043_I2C_ADDRESS);
    if (!sensor_handle) {
        ESP_LOGE(TAG, "Failed to create MAX17043 device");
        return;
    }

    ESP_LOGI(TAG, "MAX17043 device created successfully");

    /* Read initial battery data */
    uint16_t voltage;
    uint8_t soc;

    ret = max17043_read_voltage(sensor_handle, &voltage);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Initial Voltage: %u mV", voltage);
    }

    ret = max17043_read_soc(sensor_handle, &soc);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Initial SOC: %u %%", soc);
    }

    /* Demonstrate sleep/wake cycle */
    ESP_LOGI(TAG, "Entering sleep mode...");
    vTaskDelay(pdMS_TO_TICKS(1000));

    ret = max17043_sleep(sensor_handle);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✓ Device in sleep mode (low power)");
    } else {
        ESP_LOGE(TAG, "Failed to enter sleep mode: %s", esp_err_to_name(ret));
    }

    /* Sleep for 5 seconds */
    vTaskDelay(pdMS_TO_TICKS(5000));

    ESP_LOGI(TAG, "Waking up...");
    ret = max17043_wake(sensor_handle);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✓ Device awake");
    } else {
        ESP_LOGE(TAG, "Failed to wake device: %s", esp_err_to_name(ret));
    }

    /* Read battery data after waking */
    vTaskDelay(pdMS_TO_TICKS(1000));

    ret = max17043_read_voltage(sensor_handle, &voltage);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "After wake Voltage: %u mV", voltage);
    }

    ret = max17043_read_soc(sensor_handle, &soc);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "After wake SOC: %u %%", soc);
    }

    /* Perform quick start calibration */
    ESP_LOGI(TAG, "Performing quick start...");
    ret = max17043_quick_start(sensor_handle);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✓ Quick start complete");
    } else {
        ESP_LOGE(TAG, "Failed to quick start: %s", esp_err_to_name(ret));
    }

    /* Read final values */
    vTaskDelay(pdMS_TO_TICKS(1000));

    ret = max17043_read_voltage(sensor_handle, &voltage);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Final Voltage: %u mV", voltage);
    }

    ret = max17043_read_soc(sensor_handle, &soc);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Final SOC: %u %%", soc);
    }

    ESP_LOGI(TAG, "Example complete");

    max17043_delete(sensor_handle);
}
