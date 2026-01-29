#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "lis3dh.h"
#include "lis3dh_config.h"

static const char *TAG = "APP_MAIN";

static void lis3dh_reader_task(void *arg)
{
    lis3dh_handle_t sensor = (lis3dh_handle_t)arg;
    lis3dh_accel_t accel;

    while (1) {
        esp_err_t ret = lis3dh_read_accel(sensor, &accel);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Accel - X: %6d, Y: %6d, Z: %6d", accel.x, accel.y, accel.z);
        } else {
            ESP_LOGE(TAG, "Failed to read accelerometer data: %s", esp_err_to_name(ret));
        }

        vTaskDelay(pdMS_TO_TICKS(100));  /* Read every 100ms */
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting LIS3DH sensor application");

    /* Initialize I2C master bus */
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = 0,
        .scl_io_num = LIS3DH_I2C_SCL_PIN,
        .sda_io_num = LIS3DH_I2C_SDA_PIN,
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

    /* Initialize LIS3DH sensor */
    lis3dh_config_t sensor_config = {
        .i2c_bus = bus_handle,
        .i2c_addr = LIS3DH_I2C_ADDRESS,
        .odr = LIS3DH_ODR_100_HZ,
        .fs = LIS3DH_FS_2G,
    };

    lis3dh_handle_t sensor_handle;
    ret = lis3dh_init(&sensor_config, &sensor_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize LIS3DH sensor: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "LIS3DH sensor initialized successfully");

    /* Create sensor reader task */
    ret = xTaskCreate(
        lis3dh_reader_task,
        "lis3dh_reader",
        2048,
        sensor_handle,
        5,
        NULL
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create sensor reader task");
        lis3dh_deinit(sensor_handle);
    }
}
