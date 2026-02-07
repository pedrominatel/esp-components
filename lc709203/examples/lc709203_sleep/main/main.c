#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "lc709203.h"
#include "lc709203_config.h"

static const char *TAG = "LC709203_SLEEP";

#define SLEEP_DURATION_MS    10000  // 10 seconds in sleep mode
#define WAKE_DURATION_MS     5000   // 5 seconds in operational mode

static void lc709203_power_cycle_task(void *arg)
{
    lc709203_handle_t sensor = (lc709203_handle_t)arg;
    uint16_t voltage;
    uint16_t rsoc;

    while (1) {
        /* Wake up and enter operational mode */
        ESP_LOGI(TAG, "Entering operational mode...");
        esp_err_t ret = lc709203_set_power_mode(sensor, LC709203_POWER_MODE_OPERATIONAL);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to enter operational mode: %s", esp_err_to_name(ret));
        }

        /* Read battery data while in operational mode */
        for (int i = 0; i < 5; i++) {
            ret = lc709203_read_cell_voltage(sensor, &voltage);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Cell Voltage: %u mV", voltage);
            }

            ret = lc709203_read_rsoc(sensor, &rsoc);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "RSOC: %u.%u %%", rsoc / 10, rsoc % 10);
            }

            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        /* Enter sleep mode to save power */
        ESP_LOGI(TAG, "Entering sleep mode for %d seconds...", SLEEP_DURATION_MS / 1000);
        ret = lc709203_set_power_mode(sensor, LC709203_POWER_MODE_SLEEP);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to enter sleep mode: %s", esp_err_to_name(ret));
        }

        /* Wait while IC is in sleep mode */
        vTaskDelay(pdMS_TO_TICKS(SLEEP_DURATION_MS));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting LC709203F sleep mode example");
    ESP_LOGI(TAG, "This example demonstrates power mode control:");
    ESP_LOGI(TAG, "- Wake up and read battery data for %d seconds", WAKE_DURATION_MS / 1000);
    ESP_LOGI(TAG, "- Enter sleep mode for %d seconds", SLEEP_DURATION_MS / 1000);

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

    /* Read IC version */
    uint16_t version;
    ret = lc709203_read_ic_version(sensor_handle, &version);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "LC709203F IC Version: 0x%04X", version);
    }

    /* Configure battery capacity */
    ret = lc709203_set_apa(sensor_handle, LC709203_APA_1000MAH);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Battery capacity configured for 1000mAh");
    }

    /* Set battery profile */
    ret = lc709203_set_battery_profile(sensor_handle, 0);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Battery profile set to default");
    }

    /* Initialize RSOC */
    ret = lc709203_init_rsoc(sensor_handle);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "RSOC initialized");
    }

    vTaskDelay(pdMS_TO_TICKS(500));

    /* Create power cycling task */
    ret = xTaskCreate(
        lc709203_power_cycle_task,
        "lc709203_power_cycle_task",
        3072,
        sensor_handle,
        5,
        NULL
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create power cycle task");
        lc709203_delete(sensor_handle);
    }
}
