#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_types.h"
#include "esp_log.h"
#include "esp_system.h"
#include "sdkconfig.h"
#include "shtc3.h"


#define SHTC3_SDA_GPIO           CONFIG_SHTC3_I2C_SDA  /*!< gpio number for I2C master data  */
#define SHTC3_SCL_GPIO           CONFIG_SHTC3_I2C_SCL  /*!< gpio number for I2C master clock */

static const char *TAG = "SHTC3";

i2c_master_dev_handle_t shtc3_handle;

// Task to read the sensor data
void shtc3_read_task(void *pvParameters)
{
    float temperature, humidity;
    esp_err_t err = ESP_OK;
    shtc3_register_t reg = SHTC3_REG_T_CSE_NM;

    while (1) {
        err = shtc3_get_th(shtc3_handle, reg, &temperature, &humidity);
        if(err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read data from SHTC3 sensor");
        } else {
            ESP_LOGI(TAG, "Temperature: %.2f C, Humidity: %.2f %%", temperature, humidity);
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

i2c_master_bus_handle_t i2c_bus_init(uint8_t sda_io, uint8_t scl_io)
{
    i2c_master_bus_config_t i2c_bus_config = {
        .i2c_port = CONFIG_SHTC3_I2C_NUM,
        .sda_io_num = sda_io,
        .scl_io_num = scl_io,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus_handle;

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));
    ESP_LOGI(TAG, "I2C master bus created");

    return bus_handle;
}

void app_main(void)
{
    i2c_master_bus_handle_t bus_handle = i2c_bus_init(SHTC3_SDA_GPIO, SHTC3_SCL_GPIO);
    shtc3_handle = shtc3_device_create(bus_handle, SHTC3_I2C_ADDR, CONFIG_SHTC3_I2C_CLK_SPEED_HZ);
    ESP_LOGI(TAG, "Sensor initialization success");

    // Probe the sensor to check if it is connected to the bus with a 10ms timeout
    esp_err_t err = i2c_master_probe(bus_handle, SHTC3_I2C_ADDR, 200);

    if(err == ESP_OK) {
        ESP_LOGI(TAG, "SHTC3 sensor found");
        err = shtc3_get_id(shtc3_handle);

        if(err == ESP_OK) {
            ESP_LOGI(TAG, "SHTC3 ID read successfully");
            xTaskCreate(shtc3_read_task, "shtc3_read_task", 4096, NULL, 5, NULL);
        } else {
            ESP_LOGE(TAG, "Failed to read SHTC3 ID");
        }

    } else {
        ESP_LOGE(TAG, "SHTC3 sensor not found");
        shtc3_device_delete(shtc3_handle);
    }
}
