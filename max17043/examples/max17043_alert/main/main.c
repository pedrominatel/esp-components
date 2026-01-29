#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "max17043.h"
#include "max17043_config.h"

static const char *TAG = "MAX17043_ALERT";

static void max17043_alert_task(void *arg)
{
    max17043_handle_t sensor = (max17043_handle_t)arg;
    uint8_t soc, alert;

    while (1) {
        esp_err_t ret = max17043_read_soc(sensor, &soc);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "SOC: %u %%", soc);
        }

        ret = max17043_get_alert_status(sensor, &alert);
        if (ret == ESP_OK) {
            if (alert) {
                ESP_LOGW(TAG, "⚠️  ALERT: Battery level below threshold!");
                /* Clear the alert for next time */
                max17043_clear_alert(sensor);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting MAX17043 alert example");

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

    /* Set alert threshold to 10% */
    ret = max17043_set_alert_threshold(sensor_handle, 10);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Alert threshold set to 10%%");
    } else {
        ESP_LOGE(TAG, "Failed to set alert threshold: %s", esp_err_to_name(ret));
    }

    /* Create alert monitoring task */
    ret = xTaskCreate(
        max17043_alert_task,
        "max17043_alert_task",
        2048,
        sensor_handle,
        5,
        NULL
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create alert task");
        max17043_delete(sensor_handle);
    }
}
