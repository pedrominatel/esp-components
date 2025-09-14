#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "hp203b.h"

uint8_t hp203b_status = 0;
uint32_t hp203b_pressure = 0;
i2c_master_dev_handle_t hp203b_dev_handle = NULL;

#define TAG "hp203b"

esp_err_t hp203b_reset(void)
{    
    esp_err_t err = ESP_FAIL;
    uint8_t write_buffer[1] ={ 0x00 };
    write_buffer[0] = HP203B_CMD_SOFT_RESET;

    err = i2c_master_transmit(hp203b_dev_handle, write_buffer, sizeof(write_buffer), 120);



    if(err == ESP_FAIL){
        ESP_LOGE(TAG, "Sensor RESET error!");
        return ESP_FAIL;
    }
    return ESP_OK;
}

void hp203b_set_press(uint32_t press_value)
{
    hp203b_pressure = press_value;
}

int32_t hp203b_get_press(void)
{
    return hp203b_pressure;
}

void hp203b_denit(void)
{
    hp203b_status = 0;
}

esp_err_t hp203b_configure(void)
{
    esp_err_t err = ESP_FAIL;
    uint8_t write_buffer[1] ={ 0 };
    write_buffer[0] = 0x48; // OSR 1024

    err = i2c_master_transmit(hp203b_dev_handle, write_buffer, sizeof(write_buffer), 120);

    if(err == ESP_FAIL){
        ESP_LOGE(TAG, "Error to configure the sensor!");
    }

    return ESP_OK;
}

esp_err_t hp203b_read_press(void)
{
    esp_err_t err = ESP_OK;
    uint8_t read_bf[3] = { 0xff, 0xff, 0xff };
    uint8_t write_bf[1] = { HP203B_CMD_READ_P };
    uint32_t pressure = 0;

    if(hp203b_configure() == ESP_FAIL){
        return ESP_FAIL;
    }
    vTaskDelay(30 / portTICK_PERIOD_MS);

    ESP_ERROR_CHECK(i2c_master_transmit_receive(hp203b_dev_handle, write_bf, sizeof(write_bf), read_bf, sizeof(read_bf), -1));

    if(err != ESP_FAIL){
        pressure = (((read_bf[0] & 0x0F) * 65536) + (read_bf[1] * 256 + read_bf[2]));
        ESP_LOGD(TAG, "P: %lu", pressure);
        hp203b_set_press(pressure);
    } else {
        ESP_LOGE(TAG, "Error reading data from the sensor!");
    }

    return err;
}

esp_err_t hp203b_init(i2c_master_bus_handle_t bus_handle)
{

    esp_err_t err = ESP_FAIL;
    // Probe the sensor to check if it's connected
    err = i2c_master_probe(bus_handle, HP203B_DEFAULT_ADDRESS, 200);

    if(err == ESP_ERR_NOT_FOUND || err == ESP_ERR_TIMEOUT){
        ESP_LOGE(TAG, "Sensor not found!");
        return ESP_FAIL;
    } else if(err == ESP_OK){
        ESP_LOGI(TAG, "Sensor found!");
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = HP203B_DEFAULT_ADDRESS,
        .scl_speed_hz = 100000,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &hp203b_dev_handle));
    ESP_LOGI(TAG, "Sensor initialization...");

    if(hp203b_reset() == ESP_OK){
        ESP_LOGI(TAG, "Sensor RESET ok!");
        vTaskDelay(200 / portTICK_PERIOD_MS);
        // Send configuration to the sensor
        // Check if the sensor is responding
        err = hp203b_configure();
        if(err != ESP_FAIL){
            hp203b_status = 1;
            ESP_LOGI(TAG, "Sensor configured!");
            return ESP_OK;
        } else {
            ESP_LOGE(TAG, "Error configuring the sensor!");
            return ESP_FAIL;
        }
    } else {
        ESP_LOGE(TAG, "Error initializing sensor!");
        return ESP_FAIL;
    }
    return ESP_OK;
}
