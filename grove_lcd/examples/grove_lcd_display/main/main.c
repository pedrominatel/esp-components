#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "grove_lcd.h"
#include "grove_lcd_config.h"

static const char *TAG = "GROVE_LCD_DISPLAY";

void app_main(void)
{
    ESP_LOGI(TAG, "Starting Grove LCD display example");

    /* Initialize I2C master bus */
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = GROVE_LCD_I2C_PORT,
        .scl_io_num = GROVE_LCD_I2C_SCL_PIN,
        .sda_io_num = GROVE_LCD_I2C_SDA_PIN,
        .glitch_ignore_cnt = 7,
        .flags = {
            .enable_internal_pullup = true,
        },
    };

    i2c_master_bus_handle_t bus_handle;
    esp_err_t ret = i2c_new_master_bus(&bus_config, &bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C bus: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "I2C bus initialized");

    /* Create Grove LCD device */
    grove_lcd_handle_t lcd = grove_lcd_create(bus_handle, GROVE_LCD_I2C_ADDR, GROVE_LCD_RGB_ADDR);
    if (!lcd) {
        ESP_LOGE(TAG, "Failed to create Grove LCD device");
        return;
    }

    ESP_LOGI(TAG, "Grove LCD initialized successfully");

    /* Display welcome message */
    grove_lcd_print(lcd, "Hello ESP32!");
    grove_lcd_set_cursor(lcd, 1, 0);
    grove_lcd_print(lcd, "Grove LCD v4.0");
    
    vTaskDelay(pdMS_TO_TICKS(2000));

    /* Clear and display different content */
    grove_lcd_clear(lcd);
    grove_lcd_print(lcd, "Display Test");
    grove_lcd_set_cursor(lcd, 1, 0);
    grove_lcd_print(lcd, "Line 2 message");
    
    vTaskDelay(pdMS_TO_TICKS(2000));

    /* Change backlight colors */
    ESP_LOGI(TAG, "Setting backlight to RED");
    grove_lcd_set_rgb(lcd, 255, 0, 0);
    grove_lcd_clear(lcd);
    grove_lcd_print(lcd, "RED Backlight");
    vTaskDelay(pdMS_TO_TICKS(1500));

    ESP_LOGI(TAG, "Setting backlight to GREEN");
    grove_lcd_set_rgb(lcd, 0, 255, 0);
    grove_lcd_clear(lcd);
    grove_lcd_print(lcd, "GREEN Backlight");
    vTaskDelay(pdMS_TO_TICKS(1500));

    ESP_LOGI(TAG, "Setting backlight to BLUE");
    grove_lcd_set_rgb(lcd, 0, 0, 255);
    grove_lcd_clear(lcd);
    grove_lcd_print(lcd, "BLUE Backlight");
    vTaskDelay(pdMS_TO_TICKS(1500));

    ESP_LOGI(TAG, "Setting backlight to WHITE");
    grove_lcd_set_rgb(lcd, 255, 255, 255);
    grove_lcd_clear(lcd);
    grove_lcd_print(lcd, "WHITE Backlight");
    vTaskDelay(pdMS_TO_TICKS(1500));

    /* Cursor control demo */
    grove_lcd_clear(lcd);
    grove_lcd_print(lcd, "Cursor ON");
    grove_lcd_cursor(lcd, true, false);
    vTaskDelay(pdMS_TO_TICKS(2000));

    grove_lcd_cursor(lcd, true, true);
    grove_lcd_set_cursor(lcd, 1, 0);
    grove_lcd_print(lcd, "Blinking!");
    vTaskDelay(pdMS_TO_TICKS(2000));

    /* Continuous scroll */
    grove_lcd_clear(lcd);
    grove_lcd_cursor(lcd, false, false);
    grove_lcd_set_rgb(lcd, 0, 255, 255);  /* Cyan */
    
    grove_lcd_print(lcd, "Counter:");
    for (int i = 0; i < 100; i++) {
        grove_lcd_set_cursor(lcd, 1, 0);
        char buffer[17];
        snprintf(buffer, sizeof(buffer), "Value: %3d      ", i);
        grove_lcd_print(lcd, buffer);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    ESP_LOGI(TAG, "Example complete!");
    grove_lcd_delete(lcd);
}
