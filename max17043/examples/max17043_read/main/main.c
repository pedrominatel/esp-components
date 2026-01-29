#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "max17043.h"
#include "max17043_config.h"

static const char *TAG = "MAX17043_EXAMPLE";

static void max17043_read_task(void *arg)
{
    max17043_handle_t sensor = (max17043_handle_t)arg;
    uint16_t voltage;
    uint8_t soc;

    while (1) {
        esp_err_t ret = max17043_read_voltage(sensor, &voltage);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Voltage: %u mV", voltage);
        } else {
            ESP_LOGE(TAG, "Failed to read voltage: %s", esp_err_to_name(ret));
        }

        ret = max17043_read_soc(sensor, &soc);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "SOC: %u %%", soc);
        } else {
            ESP_LOGE(TAG, "Failed to read SOC: %s", esp_err_to_name(ret));
        }

        vTaskDelay(pdMS_TO_TICKS(1000));  /* Read every 1 second */
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting MAX17043 fuel gauge example");

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

    /* Probe the sensor first to check if it is connected to the bus */
    esp_err_t err = i2c_master_probe(bus_handle, MAX17043_I2C_ADDRESS, 200);
    if (err != ESP_OK) {
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

    /* Read device version */
    uint16_t version;
    ret = max17043_read_version(sensor_handle, &version);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "MAX17043 Version: 0x%04X", version);
    } else {
        ESP_LOGE(TAG, "Failed to read version: %s", esp_err_to_name(ret));
    }

    /* Create sensor reader task */
    ret = xTaskCreate(
        max17043_read_task,
        "max17043_read_task",
        2048,
        sensor_handle,
        5,
        NULL
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create sensor reader task");
        max17043_delete(sensor_handle);
    }
}
