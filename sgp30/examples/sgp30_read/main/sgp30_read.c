#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_types.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_system.h"
#include "sdkconfig.h"
#include "sgp30.h"

#define SGP30_SDA_GPIO  CONFIG_SGP30_I2C_SDA
#define SGP30_SCL_GPIO  CONFIG_SGP30_I2C_SCL

static const char *TAG = "SGP30";

static i2c_master_bus_handle_t bus_handle = NULL;

static esp_err_t init_i2c(i2c_master_bus_handle_t *i2c_bus)
{
    if (!i2c_bus) return ESP_ERR_INVALID_ARG;

    const i2c_master_bus_config_t cfg = {
        .clk_source             = I2C_CLK_SRC_DEFAULT,
        .i2c_port               = CONFIG_SGP30_I2C_NUM,
        .scl_io_num             = SGP30_SCL_GPIO,
        .sda_io_num             = SGP30_SDA_GPIO,
        .flags.enable_internal_pullup = true,
        .glitch_ignore_cnt      = 7,
    };

    return i2c_new_master_bus(&cfg, i2c_bus);
}

static void read_sensor_task(void *pvParameters)
{
    sgp30_measurement_t measurement = {0};

    while (1) {
        if (sgp30_measure(&measurement) == ESP_OK) {
            ESP_LOGI(TAG, "eCO2: %u ppm  |  TVOC: %u ppb",
                     measurement.eco2, measurement.tvoc);
        } else {
            ESP_LOGE(TAG, "Measurement failed");
        }
        /* SGP30 requires exactly one measurement per second */
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "SGP30 sensor example");

    /* Initialize the I2C bus */
    ESP_ERROR_CHECK(init_i2c(&bus_handle));

    /* Initialize the SGP30 sensor */
    if (sgp30_init(bus_handle) == ESP_OK) {
        ESP_LOGI(TAG, "Sensor initialization ok!");
        /* Spawn the periodic measurement task */
        xTaskCreate(&read_sensor_task, "read_sensor_task", 2048, NULL, 2, NULL);
    } else {
        ESP_LOGE(TAG, "Sensor initialization error!");
    }
}
