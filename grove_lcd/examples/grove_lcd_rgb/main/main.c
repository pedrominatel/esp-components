#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "grove_lcd.h"
#include "grove_lcd_config.h"

static const char *TAG = "GROVE_LCD_RGB";

void app_main(void)
{
    ESP_LOGI(TAG, "Starting Grove LCD RGB example");

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

    /* RGB color palette demo */
    const char *color_names[] = {"Red", "Green", "Blue", "Yellow", "Magenta", "Cyan"};
    const uint8_t colors[6][3] = {
        {255, 0, 0},      /* Red */
        {0, 255, 0},      /* Green */
        {0, 0, 255},      /* Blue */
        {255, 255, 0},    /* Yellow */
        {255, 0, 255},    /* Magenta */
        {0, 255, 255},    /* Cyan */
    };

    /* Display each color */
    for (int i = 0; i < 6; i++) {
        grove_lcd_clear(lcd);
        grove_lcd_print(lcd, "RGB Palette");
        grove_lcd_set_cursor(lcd, 1, 0);
        grove_lcd_print(lcd, color_names[i]);
        
        ESP_LOGI(TAG, "Color: %s", color_names[i]);
        grove_lcd_set_rgb(lcd, colors[i][0], colors[i][1], colors[i][2]);
        vTaskDelay(pdMS_TO_TICKS(1500));
    }

    /* Rainbow gradient effect */
    grove_lcd_clear(lcd);
    grove_lcd_print(lcd, "Rainbow Effect");
    grove_lcd_set_cursor(lcd, 1, 0);
    grove_lcd_print(lcd, "Transitioning...");

    for (int r = 0; r <= 255; r += 5) {
        grove_lcd_set_rgb(lcd, r, 0, 255 - r);
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    for (int g = 0; g <= 255; g += 5) {
        grove_lcd_set_rgb(lcd, 255 - g, g, 0);
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    for (int b = 0; b <= 255; b += 5) {
        grove_lcd_set_rgb(lcd, 0, 255 - b, b);
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    /* Intensity control */
    grove_lcd_clear(lcd);
    grove_lcd_print(lcd, "Intensity");
    grove_lcd_set_cursor(lcd, 1, 0);
    grove_lcd_print(lcd, "Dimming...");

    grove_lcd_set_rgb(lcd, 255, 0, 0);  /* Red */

    for (int intensity = 255; intensity >= 0; intensity -= 10) {
        grove_lcd_set_backlight(lcd, intensity);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    for (int intensity = 0; intensity <= 255; intensity += 10) {
        grove_lcd_set_backlight(lcd, intensity);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    /* Restore full brightness */
    grove_lcd_set_backlight(lcd, 255);

    /* Breathing effect with color */
    grove_lcd_clear(lcd);
    grove_lcd_print(lcd, "Breathing");
    grove_lcd_set_cursor(lcd, 1, 0);
    grove_lcd_print(lcd, "Blue Light");

    grove_lcd_set_rgb(lcd, 0, 0, 255);  /* Blue */

    for (int i = 0; i < 4; i++) {
        for (int intensity = 100; intensity <= 255; intensity += 10) {
            grove_lcd_set_backlight(lcd, intensity);
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        for (int intensity = 255; intensity >= 100; intensity -= 10) {
            grove_lcd_set_backlight(lcd, intensity);
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }

    grove_lcd_set_backlight(lcd, 255);

    /* Final state */
    grove_lcd_clear(lcd);
    grove_lcd_set_rgb(lcd, 0, 255, 0);  /* Green */
    grove_lcd_print(lcd, "Demo Complete!");
    grove_lcd_set_cursor(lcd, 1, 0);
    grove_lcd_print(lcd, "All tests passed");

    ESP_LOGI(TAG, "RGB example complete!");
    vTaskDelay(pdMS_TO_TICKS(3000));

    grove_lcd_delete(lcd);
}
