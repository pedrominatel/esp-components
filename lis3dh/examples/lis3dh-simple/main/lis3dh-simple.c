#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_types.h"
#include "esp_log.h"
#include "esp_system.h"
#include "sdkconfig.h"
#include "lis3dh.h"

#define LIS3DH_SDA_GPIO           CONFIG_LIS3DH_I2C_SDA  /*!< gpio number for I2C master data  */
#define LIS3DH_SCL_GPIO           CONFIG_LIS3DH_I2C_SCL  /*!< gpio number for I2C master clock */

static const char *TAG = "LIS3DH";

i2c_master_dev_handle_t lis3dh_dev;

// Task to read the sensor data
void lis3dh_read_task(void *pvParameters)
{
    esp_err_t err = ESP_OK;
    int16_t raw_x, raw_y, raw_z;
    uint8_t fifo_src;

    lis3dh_get_fifo_src(lis3dh_dev, &fifo_src);

    ESP_LOGI(TAG, "FIFO_SRC: %d", fifo_src);

    while (1) {
        err = lis3dh_get_raw_data(lis3dh_dev, &raw_x, &raw_y, &raw_z);
        if(err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read data from the sensor");
        } else {
            ESP_LOGI(TAG, "X: %d, Y: %d, Z: %d", raw_x, raw_y, raw_z);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

i2c_master_bus_handle_t i2c_bus_init(uint8_t sda_io, uint8_t scl_io)
{
    i2c_master_bus_config_t i2c_bus_config = {
        .i2c_port = CONFIG_LIS3DH_I2C_NUM,
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

    uint8_t dev_addr = LIS3DH_I2C_L_ADDR;
    esp_err_t err = ESP_OK;

    i2c_master_bus_handle_t bus_handle = i2c_bus_init(LIS3DH_SDA_GPIO, LIS3DH_SCL_GPIO);
    
    err = i2c_master_probe(bus_handle, dev_addr, 200);
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LIS3DH sensor not found");
        return;
    }

    lis3dh_dev = lis3dh_device_create(bus_handle, dev_addr, CONFIG_LIS3DH_I2C_CLK_SPEED_HZ);

    if(lis3dh_dev == NULL) {
        ESP_LOGE(TAG, "Failed to create LIS3DH device");
        return;
    } else {
        ESP_LOGI(TAG, "LIS3DH device created");
    }

    err = lis3dh_get_who_am_i(lis3dh_dev);

    if(err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read data from the sensor");
    } else {
        lis3dh_reset(lis3dh_dev);
        // lis3dh_set_bypass_mode(lis3dh_dev);
        xTaskCreate(lis3dh_read_task, "lis3dh_read_task", 4096, NULL, 5, NULL);
    }
    
}

