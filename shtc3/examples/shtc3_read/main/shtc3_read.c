#include <stdio.h>
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

// Task to read the sensor data
void shtc3_read_task(void *pvParameters)
{
    float temperature, humidity;
    esp_err_t err = ESP_OK;
    shtc3_register_rw_t reg = SHTC3_REG_T_CSE_NM;
    i2c_master_dev_handle_t shtc3_handle = (i2c_master_dev_handle_t)pvParameters;

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
    ESP_LOGI(TAG, "Initializing I2C bus with SDA GPIO %d, SCL GPIO %d", sda_io, scl_io);
    
    i2c_master_bus_config_t i2c_bus_config = {
        .i2c_port = CONFIG_SHTC3_I2C_NUM,
        .sda_io_num = sda_io,
        .scl_io_num = scl_io,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus_handle = NULL;
    esp_err_t ret = i2c_new_master_bus(&i2c_bus_config, &bus_handle);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C master bus: %s", esp_err_to_name(ret));
        return NULL;
    }
    
    ESP_LOGI(TAG, "I2C master bus created successfully");
    return bus_handle;
}

void app_main(void)
{
    i2c_master_bus_handle_t bus_handle = i2c_bus_init(SHTC3_SDA_GPIO, SHTC3_SCL_GPIO);
    
    if (bus_handle == NULL) {
        ESP_LOGE(TAG, "Failed to initialize I2C bus, aborting");
        return;
    }
    
    // Probe the sensor first to check if it is connected to the bus
    esp_err_t err = i2c_master_probe(bus_handle, SHTC3_I2C_ADDR, 200);

    if(err == ESP_OK) {
        ESP_LOGI(TAG, "SHTC3 sensor found");
        
        // Create device handle
        i2c_master_dev_handle_t shtc3_handle = shtc3_device_create(bus_handle, SHTC3_I2C_ADDR, CONFIG_SHTC3_I2C_CLK_SPEED_HZ);
        if (shtc3_handle == NULL) {
            ESP_LOGE(TAG, "Failed to create SHTC3 device");
            return;
        }
        ESP_LOGI(TAG, "Sensor initialization success");
        
        // Read sensor ID
        uint8_t sensor_id[2];
        err = shtc3_get_id(shtc3_handle, sensor_id);
        if(err == ESP_OK) {
            ESP_LOGI(TAG, "Sensor ID: 0x%02x%02x", sensor_id[0], sensor_id[1]);
            ESP_LOGI(TAG, "SHTC3 ID read successfully");
            // Create task to continuously read sensor data, passing device handle as parameter
            xTaskCreate(shtc3_read_task, "shtc3_read_task", 4096, (void *)shtc3_handle, 5, NULL);
        } else {
            ESP_LOGE(TAG, "Failed to read SHTC3 ID");
            shtc3_device_delete(shtc3_handle);
            return;
        }
    } else {
        ESP_LOGE(TAG, "SHTC3 sensor not found");
        return;
    }
    
    // Keep the main task alive; sensor readings happen in shtc3_read_task
    while (1) {
        vTaskDelay(portMAX_DELAY);
    }
}
