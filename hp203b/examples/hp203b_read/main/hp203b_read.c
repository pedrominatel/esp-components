#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_types.h"
#include "esp_log.h"
#include "esp_system.h"
#include "sdkconfig.h"
#include "hp203b.h"

#include "driver/i2c_master.h"

#define HP203B_SDA_GPIO           CONFIG_HP203B_I2C_SDA  /*!< gpio number for I2C master data  */
#define HP203B_SCL_GPIO           CONFIG_HP203B_I2C_SCL  /*!< gpio number for I2C master clock */

static const char *TAG = "HP203B";

i2c_master_bus_handle_t bus_handle = NULL;
hp203b_handle_t sensor_handle = NULL;

esp_err_t init_i2c(i2c_master_bus_handle_t *i2c_bus)
{
    if (!i2c_bus) return ESP_ERR_INVALID_ARG;

    const i2c_master_bus_config_t cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port   = CONFIG_HP203B_I2C_NUM,     // must be I2C_NUM_0 on ESP32-C3
        .scl_io_num = HP203B_SCL_GPIO,
        .sda_io_num = HP203B_SDA_GPIO,
        .flags = {
            .enable_internal_pullup = true, // ok for dev bring-up; prefer external pull-ups in HW
        },
        .glitch_ignore_cnt = 7,
    };

    // Create & install the master bus driver; returns the handle via out parameter
    return i2c_new_master_bus(&cfg, i2c_bus);
}

void read_sensor_task(void *pvParameters)
{     

    while(1) {
        if(hp203b_read_press(sensor_handle) == ESP_OK)
        {
            uint32_t sample_pa = 0;
            sample_pa = hp203b_get_press(sensor_handle);
            ESP_LOGI(TAG, "Pressure: %lu Pa", sample_pa);
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{

    ESP_LOGI(TAG, "This is the HP203B sensor example");

    // Initialize the I2C bus
    ESP_ERROR_CHECK(init_i2c(&bus_handle));

    if(hp203b_init(bus_handle, &sensor_handle) == ESP_OK){
        ESP_LOGI(TAG, "Sensor initialization ok!");
        xTaskCreate(&read_sensor_task, "read_sensor_task", 2048, NULL, 2, NULL);
    } else {
        ESP_LOGE(TAG, "Sensor initialization error!");
    }

}
