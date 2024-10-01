#include <stdio.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include "esp_system.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_system.h"
#include "t9602.h"

#define T9602_SDA_GPIO           CONFIG_T9602_I2C_SDA  /*!< gpio number for I2C master data  */
#define T9602_SCL_GPIO           CONFIG_T9602_I2C_SCL  /*!< gpio number for I2C master clock */

static const char *TAG = "T9602 example";

i2c_master_dev_handle_t t9602_handle;

// task to read the sensor values
void t9602_task(void *pvParameters)
{
    while(1)
    {
        float temperature, humidity = {0};
        // Get raw temperature and humidity values
        t9602_get_data(t9602_handle, &temperature, &humidity);
        ESP_LOGI(TAG, "T=%.2f, RH=%.2f", temperature, humidity);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    t9602_device_delete(t9602_handle);
    vTaskDelete(NULL);
}

/**
 * @brief Initialize the I2C master bus.
 *
 * This function initializes the I2C master bus with the specified SDA and SCL
 * GPIO pins. It configures the I2C bus with default clock source and enables
 * internal pull-up resistors.
 *
 * @param sda_io GPIO number for the SDA (data) line.
 * @param scl_io GPIO number for the SCL (clock) line.
 * 
 * @return i2c_master_bus_handle_t Handle to the initialized I2C master bus.
 */
i2c_master_bus_handle_t i2c_bus_init(uint8_t sda_io, uint8_t scl_io)
{
    
    i2c_master_bus_config_t i2c_bus_config = {
        .i2c_port = CONFIG_T9602_I2C_NUM,
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

static void t9602_sensor_init(void)
{
    
    i2c_master_bus_handle_t bus_handle = i2c_bus_init(T9602_SDA_GPIO, T9602_SCL_GPIO);
    t9602_handle = t9602_device_create(bus_handle, T9602_I2C_ADDR_0, CONFIG_T9602_I2C_CLK_SPEED_HZ);
    ESP_LOGI(TAG, "Sensor initialization success");

}

void app_main(void)
{
    t9602_sensor_init();
    xTaskCreate(t9602_task, "t9602_task", 2048, NULL, 10, NULL);
}
