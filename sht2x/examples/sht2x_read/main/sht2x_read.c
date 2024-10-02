#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_types.h"
#include "esp_log.h"
#include "esp_system.h"
#include "sdkconfig.h"
#include "sht2x.h"


#define SHT2X_SDA_GPIO           CONFIG_SHT2X_I2C_SDA  /*!< gpio number for I2C master data  */
#define SHT2X_SCL_GPIO           CONFIG_SHT2X_I2C_SCL  /*!< gpio number for I2C master clock */

static const char *TAG = "SHT2x";

i2c_master_dev_handle_t sht2x_handle;

// Task to read the sensor data
void sht2x_read_task(void *pvParameters)
{
    float temperature, humidity;

    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        // Read user register
        sht2x_get_user_data(sht2x_handle);
        // sht2x_get_data(sht2x_handle, &temperature, &humidity);
        // ESP_LOGI(TAG, "Temperature: %.2f C, Humidity: %.2f %%", temperature, humidity);
    }
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
        .i2c_port = CONFIG_SHT2X_I2C_NUM,
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

    i2c_master_bus_handle_t bus_handle = i2c_bus_init(SHT2X_SDA_GPIO, SHT2X_SCL_GPIO);
    sht2x_handle = sht2x_device_create(bus_handle, SHT2X_I2C_ADDR, CONFIG_SHT2X_I2C_CLK_SPEED_HZ);
    ESP_LOGI(TAG, "Sensor initialization success");

    esp_err_t err = i2c_master_probe(bus_handle, 0x28, 100);

    if(err == ESP_OK) {
        ESP_LOGI(TAG, "SHT2x sensor found");
    } else {
        ESP_LOGE(TAG, "SHT2x sensor not found");
    }

    // Start task
    // xTaskCreate(sht2x_read_task, "sht2x_read_task", 4096, NULL, 5, NULL);

}
