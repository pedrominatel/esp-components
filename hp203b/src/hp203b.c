#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "hp203b.h"

#define TAG "hp203b"

typedef struct {
    i2c_master_dev_handle_t i2c_dev;
    uint32_t pressure;
    uint8_t status;
} hp203b_dev_t;

static esp_err_t hp203b_reset(hp203b_dev_t *dev)
{    
    esp_err_t err = ESP_FAIL;
    uint8_t write_buffer[1] = { HP203B_CMD_SOFT_RESET };

    err = i2c_master_transmit(dev->i2c_dev, write_buffer, sizeof(write_buffer), 120);

    if(err == ESP_FAIL){
        ESP_LOGE(TAG, "Sensor RESET error!");
        return ESP_FAIL;
    }
    return ESP_OK;
}

int32_t hp203b_get_press(hp203b_handle_t dev_handle)
{
    if (!dev_handle) {
        return 0;
    }
    hp203b_dev_t *dev = (hp203b_dev_t *)dev_handle;
    return dev->pressure;
}

void hp203b_deinit(hp203b_handle_t dev_handle)
{
    if (!dev_handle) {
        return;
    }
    hp203b_dev_t *dev = (hp203b_dev_t *)dev_handle;
    i2c_master_bus_rm_device(dev->i2c_dev);
    free(dev);
}

static esp_err_t hp203b_configure(hp203b_dev_t *dev)
{
    esp_err_t err = ESP_FAIL;
    uint8_t write_buffer[1] = { 0x48 }; // OSR 1024

    err = i2c_master_transmit(dev->i2c_dev, write_buffer, sizeof(write_buffer), 120);

    if(err == ESP_FAIL){
        ESP_LOGE(TAG, "Error to configure the sensor!");
    }

    return ESP_OK;
}

esp_err_t hp203b_read_press(hp203b_handle_t dev_handle)
{
    if (!dev_handle) {
        return ESP_ERR_INVALID_ARG;
    }

    hp203b_dev_t *dev = (hp203b_dev_t *)dev_handle;
    esp_err_t err = ESP_OK;
    uint8_t read_bf[3] = { 0xff, 0xff, 0xff };
    uint8_t write_bf[1] = { HP203B_CMD_READ_P };
    uint32_t pressure = 0;

    if(hp203b_configure(dev) == ESP_FAIL){
        return ESP_FAIL;
    }
    vTaskDelay(30 / portTICK_PERIOD_MS);

    err = i2c_master_transmit_receive(dev->i2c_dev, write_bf, sizeof(write_bf), read_bf, sizeof(read_bf), -1);

    if(err == ESP_OK){
        pressure = (((read_bf[0] & 0x0F) * 65536) + (read_bf[1] * 256 + read_bf[2]));
        ESP_LOGD(TAG, "P: %lu", pressure);
        dev->pressure = pressure;
    } else {
        ESP_LOGE(TAG, "Error reading data from the sensor!");
    }

    return err;
}

esp_err_t hp203b_init(i2c_master_bus_handle_t bus_handle, hp203b_handle_t *dev_handle)
{
    if (!bus_handle || !dev_handle) {
        ESP_LOGE(TAG, "Invalid arguments");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = ESP_FAIL;
    
    // Probe the sensor to check if it's connected
    err = i2c_master_probe(bus_handle, HP203B_DEFAULT_ADDRESS, 200);

    if(err == ESP_ERR_NOT_FOUND || err == ESP_ERR_TIMEOUT){
        ESP_LOGE(TAG, "Sensor not found!");
        return ESP_FAIL;
    } else if(err == ESP_OK){
        ESP_LOGI(TAG, "Sensor found!");
    }

    // Allocate device structure
    hp203b_dev_t *dev = (hp203b_dev_t *)malloc(sizeof(hp203b_dev_t));
    if (!dev) {
        ESP_LOGE(TAG, "Failed to allocate memory for device");
        return ESP_ERR_NO_MEM;
    }
    memset(dev, 0, sizeof(hp203b_dev_t));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = HP203B_DEFAULT_ADDRESS,
        .scl_speed_hz = 100000,
    };

    err = i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev->i2c_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add device to I2C bus: %s", esp_err_to_name(err));
        free(dev);
        return err;
    }
    ESP_LOGI(TAG, "Sensor initialization...");

    if(hp203b_reset(dev) == ESP_OK){
        ESP_LOGI(TAG, "Sensor RESET ok!");
        vTaskDelay(200 / portTICK_PERIOD_MS);
        // Send configuration to the sensor
        // Check if the sensor is responding
        err = hp203b_configure(dev);
        if(err != ESP_FAIL){
            dev->status = 1;
            ESP_LOGI(TAG, "Sensor configured!");
            *dev_handle = (hp203b_handle_t)dev;
            return ESP_OK;
        } else {
            ESP_LOGE(TAG, "Error configuring the sensor!");
            i2c_master_bus_rm_device(dev->i2c_dev);
            free(dev);
            return ESP_FAIL;
        }
    } else {
        ESP_LOGE(TAG, "Error initializing sensor!");
        i2c_master_bus_rm_device(dev->i2c_dev);
        free(dev);
        return ESP_FAIL;
    }
}
