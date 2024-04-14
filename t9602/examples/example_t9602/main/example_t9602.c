#include <stdio.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include "esp_system.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_system.h"
#include "t9602.h"
#include "driver/gpio.h"

#define I2C_MASTER_SDA_IO           17               /*!< gpio number for I2C master data  */
#define I2C_MASTER_SCL_IO           18               /*!< gpio number for I2C master clock */
#define I2C_MASTER_NUM              I2C_NUM_0        /*!< I2C port number for master dev */
#define I2C_MASTER_CLK_SPEED        100000           /*!< I2C master clock frequency */

static const char *TAG = "T9602 example";
t9602_handle_t sensor;

// task to read the sensor values
void t9602_task(void *pvParameters)
{
    while(1)
    {
        // Get raw temperature and humidity values
        float temperature, humidity = {0};
        t9602_get_data(sensor, &temperature, &humidity);
        ESP_LOGI(TAG, "T=%.2f, RH=%.2f", temperature, humidity);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

static void i2c_bus_init(void)
{
    
    esp_err_t err = ESP_OK;
    
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_CLK_SPEED,
    };

    err = i2c_param_config(I2C_MASTER_NUM, &conf);
    assert(ESP_OK == err);
    err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    
    if(err != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C bus initialization failed");
        return;
    }

}

static void t9602_sensor_init(void)
{
    i2c_bus_init();
    sensor = t9602_create(I2C_MASTER_NUM, T9602_I2C_ADDR_0);
    if (sensor == NULL) {
        ESP_LOGE(TAG, "Sensor initialization failed");
        return;
    }
    ESP_LOGI(TAG, "Sensor initialization success");
}

void app_main(void)
{
    t9602_sensor_init();
    xTaskCreate(t9602_task, "t9602_task", 2048, NULL, 10, NULL);
}
