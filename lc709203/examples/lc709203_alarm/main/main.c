#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "lc709203.h"
#include "lc709203_config.h"

static const char *TAG = "LC709203_ALARM";

#define LOW_BATTERY_THRESHOLD 15  // 15% RSOC threshold

static void lc709203_alarm_task(void *arg)
{
    lc709203_handle_t sensor = (lc709203_handle_t)arg;
    uint16_t rsoc;
    uint16_t voltage;

    while (1) {
        esp_err_t ret = lc709203_read_rsoc(sensor, &rsoc);
        if (ret == ESP_OK) {
            float rsoc_percent = rsoc / 10.0;
            ESP_LOGI(TAG, "RSOC: %.1f %%", rsoc_percent);

            if (rsoc <= LOW_BATTERY_THRESHOLD * 10) {
                ESP_LOGW(TAG, "⚠️  LOW BATTERY ALERT: Battery below %d%%!", LOW_BATTERY_THRESHOLD);
            }
        }

        ret = lc709203_read_cell_voltage(sensor, &voltage);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Cell Voltage: %u mV", voltage);
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting LC709203F low battery alarm example");

    /* Initialize I2C master bus */
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = LC709203_I2C_PORT,
        .scl_io_num = LC709203_I2C_SCL_PIN,
        .sda_io_num = LC709203_I2C_SDA_PIN,
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
    ret = i2c_master_probe(bus_handle, LC709203_I2C_ADDRESS, 200);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LC709203F not found at address 0x%02X", LC709203_I2C_ADDRESS);
        return;
    }

    ESP_LOGI(TAG, "LC709203F detected on I2C bus");

    /* Create LC709203 device handle */
    lc709203_handle_t sensor_handle = lc709203_create(bus_handle, LC709203_I2C_ADDRESS);
    if (!sensor_handle) {
        ESP_LOGE(TAG, "Failed to create LC709203 device");
        return;
    }

    ESP_LOGI(TAG, "LC709203F device created successfully");

    /* Configure battery capacity for 1000mAh battery */
    ret = lc709203_set_apa(sensor_handle, LC709203_APA_1000MAH);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Battery capacity configured for 1000mAh");
    }

    /* Set battery profile */
    ret = lc709203_set_battery_profile(sensor_handle, 0);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Battery profile set to default");
    }

    /* Set RSOC alarm threshold */
    ret = lc709203_set_rsoc_alarm(sensor_handle, LOW_BATTERY_THRESHOLD);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "RSOC alarm threshold set to %d%%", LOW_BATTERY_THRESHOLD);
    } else {
        ESP_LOGE(TAG, "Failed to set RSOC alarm: %s", esp_err_to_name(ret));
    }

    /* Initialize RSOC algorithm */
    ret = lc709203_init_rsoc(sensor_handle);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "RSOC initialized");
    }

    vTaskDelay(pdMS_TO_TICKS(500));

    /* Create alarm monitoring task */
    ret = xTaskCreate(
        lc709203_alarm_task,
        "lc709203_alarm_task",
        2048,
        sensor_handle,
        5,
        NULL
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create alarm task");
        lc709203_delete(sensor_handle);
    }
}
