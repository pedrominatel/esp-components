#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "grove_lcd.h"
#include "esp_log.h"

static const char *TAG = "GROVE_LCD";

/* AIP31068L I2C LCD protocol constants */
#define LCD_CMD_MODE        0x80  /* Command mode */
#define LCD_DATA_MODE       0x40  /* Data mode */

/* HD44780 LCD Commands */
#define LCD_CLEAR           0x01
#define LCD_HOME            0x02
#define LCD_ENTRY_MODE      0x04
#define LCD_DISPLAY_CTRL    0x08
#define LCD_FUNCTION_SET    0x20
#define LCD_SET_DDRAM       0x80

/* Entry mode flags */
#define LCD_ENTRY_INC       0x02
#define LCD_ENTRY_SHIFT     0x01

/* Display control flags */
#define LCD_DISPLAY_ON      0x04
#define LCD_CURSOR_ON       0x02
#define LCD_BLINK_ON        0x01

/* Function set flags */
#define LCD_8BIT_MODE       0x10
#define LCD_2LINE           0x08
#define LCD_5x8DOTS         0x00

typedef struct {
    i2c_master_bus_handle_t i2c_bus;
    i2c_master_dev_handle_t lcd_dev;
    i2c_master_dev_handle_t rgb_dev;
    uint8_t lcd_addr;
    uint8_t rgb_addr;
} grove_lcd_dev_t;

/* Write command to AIP31068L */
static esp_err_t _write_command(grove_lcd_dev_t *dev, uint8_t cmd)
{
    uint8_t data[2] = {LCD_CMD_MODE, cmd};
    return i2c_master_transmit(dev->lcd_dev, data, 2, 1000);
}

/* Write data to AIP31068L */
static esp_err_t _write_data(grove_lcd_dev_t *dev, uint8_t value)
{
    uint8_t data[2] = {LCD_DATA_MODE, value};
    return i2c_master_transmit(dev->lcd_dev, data, 2, 1000);
}

grove_lcd_handle_t grove_lcd_create(i2c_master_bus_handle_t bus_handle, uint8_t lcd_addr, uint8_t rgb_addr)
{
    if (!bus_handle) {
        ESP_LOGE(TAG, "Invalid I2C bus handle");
        return NULL;
    }

    grove_lcd_dev_t *dev = (grove_lcd_dev_t *)malloc(sizeof(grove_lcd_dev_t));
    if (!dev) {
        ESP_LOGE(TAG, "Failed to allocate memory for device handle");
        return NULL;
    }

    dev->i2c_bus = bus_handle;
    dev->lcd_addr = lcd_addr;
    dev->rgb_addr = rgb_addr;

    /* Probe LCD device */
    esp_err_t ret = i2c_master_probe(bus_handle, lcd_addr, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LCD device not found at address 0x%02X: %s", lcd_addr, esp_err_to_name(ret));
        free(dev);
        return NULL;
    }
    ESP_LOGI(TAG, "LCD device found at address 0x%02X", lcd_addr);

    /* Probe RGB device */
    ret = i2c_master_probe(bus_handle, rgb_addr, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "RGB device not found at address 0x%02X: %s", rgb_addr, esp_err_to_name(ret));
        free(dev);
        return NULL;
    }
    ESP_LOGI(TAG, "RGB device found at address 0x%02X", rgb_addr);

    /* Create LCD I2C device */
    i2c_device_config_t lcd_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = lcd_addr,
        .scl_speed_hz = 100000,
    };

    ret = i2c_master_bus_add_device(bus_handle, &lcd_cfg, &dev->lcd_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add LCD I2C device at 0x%02X: %s", lcd_addr, esp_err_to_name(ret));
        free(dev);
        return NULL;
    }


    /* Create RGB I2C device */
    i2c_device_config_t rgb_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = rgb_addr,
        .scl_speed_hz = 100000,
    };

    ret = i2c_master_bus_add_device(bus_handle, &rgb_cfg, &dev->rgb_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add RGB I2C device at 0x%02X: %s", rgb_addr, esp_err_to_name(ret));
        i2c_master_bus_rm_device(dev->lcd_dev);
        free(dev);
        return NULL;
    }

    /* Initialize AIP31068L LCD controller */
    vTaskDelay(pdMS_TO_TICKS(100));  /* Allow time for power-up */
    
    /* Function set: 8-bit mode, 2 lines, 5x8 font */
    ret = _write_command(dev, LCD_FUNCTION_SET | LCD_8BIT_MODE | LCD_2LINE | LCD_5x8DOTS);
    if (ret != ESP_OK) {
        ESP_LOGD(TAG, "First command response: %s (may be expected during power-up)", esp_err_to_name(ret));
        vTaskDelay(pdMS_TO_TICKS(10));
        /* Retry once */
        _write_command(dev, LCD_FUNCTION_SET | LCD_8BIT_MODE | LCD_2LINE | LCD_5x8DOTS);
    }
    vTaskDelay(pdMS_TO_TICKS(5));

    /* Display control: display on, cursor off, blink off */
    _write_command(dev, LCD_DISPLAY_CTRL | LCD_DISPLAY_ON);
    vTaskDelay(pdMS_TO_TICKS(5));

    /* Entry mode: left to right, no shift */
    _write_command(dev, LCD_ENTRY_MODE | LCD_ENTRY_INC);
    vTaskDelay(pdMS_TO_TICKS(5));

    /* Clear display */
    _write_command(dev, LCD_CLEAR);
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Initialize RGB backlight (SGM31323) */
    uint8_t rgb_cmd[2];
    
    vTaskDelay(pdMS_TO_TICKS(10));
    
    /* MODE1: Wake up from sleep, enable internal oscillator */
    rgb_cmd[0] = 0x00;
    rgb_cmd[1] = 0x01;  /* Normal mode, oscillator on */
    ret = i2c_master_transmit(dev->rgb_dev, rgb_cmd, 2, 1000);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set MODE1: %s", esp_err_to_name(ret));
    }
    vTaskDelay(pdMS_TO_TICKS(1));
    
    /* MODE2: DMBLNK to 1, output change on STOP */
    rgb_cmd[0] = 0x01;
    rgb_cmd[1] = 0x00;
    ret = i2c_master_transmit(dev->rgb_dev, rgb_cmd, 2, 1000);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set MODE2: %s", esp_err_to_name(ret));
    }
    vTaskDelay(pdMS_TO_TICKS(1));
    
    /* Set all PWM outputs to 0 initially */
    rgb_cmd[0] = 0x02; rgb_cmd[1] = 0x00;
    i2c_master_transmit(dev->rgb_dev, rgb_cmd, 2, 1000);
    rgb_cmd[0] = 0x03; rgb_cmd[1] = 0x00;
    i2c_master_transmit(dev->rgb_dev, rgb_cmd, 2, 1000);
    rgb_cmd[0] = 0x04; rgb_cmd[1] = 0x00;
    i2c_master_transmit(dev->rgb_dev, rgb_cmd, 2, 1000);
    
    /* Enable LED outputs */
    rgb_cmd[0] = 0x08;  /* LEDOUT register */
    rgb_cmd[1] = 0xFF;  /* All LEDs on, PWM controlled */
    ret = i2c_master_transmit(dev->rgb_dev, rgb_cmd, 2, 1000);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to enable LED outputs: %s", esp_err_to_name(ret));
    }
    
    /* Default to white backlight */
    grove_lcd_set_rgb((grove_lcd_handle_t)dev, 255, 255, 255);

    ESP_LOGI(TAG, "Grove LCD RGB initialized successfully");
    return (grove_lcd_handle_t)dev;
}

esp_err_t grove_lcd_delete(grove_lcd_handle_t handle)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    grove_lcd_dev_t *dev = (grove_lcd_dev_t *)handle;

    esp_err_t ret = i2c_master_bus_rm_device(dev->lcd_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to remove LCD device: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = i2c_master_bus_rm_device(dev->rgb_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to remove RGB device: %s", esp_err_to_name(ret));
        return ret;
    }

    free(dev);
    return ESP_OK;
}

esp_err_t grove_lcd_clear(grove_lcd_handle_t handle)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    grove_lcd_dev_t *dev = (grove_lcd_dev_t *)handle;
    _write_command(dev, LCD_CLEAR);
    vTaskDelay(pdMS_TO_TICKS(2));
    return ESP_OK;
}

esp_err_t grove_lcd_home(grove_lcd_handle_t handle)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    grove_lcd_dev_t *dev = (grove_lcd_dev_t *)handle;
    _write_command(dev, LCD_HOME);
    vTaskDelay(pdMS_TO_TICKS(2));
    return ESP_OK;
}

esp_err_t grove_lcd_set_cursor(grove_lcd_handle_t handle, uint8_t row, uint8_t col)
{
    if (!handle || row > 1 || col > 15) {
        return ESP_ERR_INVALID_ARG;
    }

    grove_lcd_dev_t *dev = (grove_lcd_dev_t *)handle;
    uint8_t row_offset[] = {0x00, 0x40};
    uint8_t cmd = LCD_SET_DDRAM | (col + row_offset[row]);
    _write_command(dev, cmd);
    return ESP_OK;
}

esp_err_t grove_lcd_print(grove_lcd_handle_t handle, const char *text)
{
    if (!handle || !text) {
        return ESP_ERR_INVALID_ARG;
    }

    grove_lcd_dev_t *dev = (grove_lcd_dev_t *)handle;

    for (int i = 0; text[i] != '\0' && i < 16; i++) {
        _write_data(dev, text[i]);
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    return ESP_OK;
}

esp_err_t grove_lcd_cursor(grove_lcd_handle_t handle, bool cursor, bool blink)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    grove_lcd_dev_t *dev = (grove_lcd_dev_t *)handle;
    uint8_t cmd = LCD_DISPLAY_CTRL | LCD_DISPLAY_ON;
    
    if (cursor) cmd |= LCD_CURSOR_ON;
    if (blink) cmd |= LCD_BLINK_ON;

    _write_command(dev, cmd);
    return ESP_OK;
}

esp_err_t grove_lcd_display(grove_lcd_handle_t handle, bool enable)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    grove_lcd_dev_t *dev = (grove_lcd_dev_t *)handle;
    uint8_t cmd = LCD_DISPLAY_CTRL;
    
    if (enable) cmd |= LCD_DISPLAY_ON;

    _write_command(dev, cmd);
    return ESP_OK;
}

esp_err_t grove_lcd_set_rgb(grove_lcd_handle_t handle, uint8_t red, uint8_t green, uint8_t blue)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    grove_lcd_dev_t *dev = (grove_lcd_dev_t *)handle;
    uint8_t cmd[2];
    esp_err_t ret;

    /* Set red (PWM2 - register 0x04) */
    cmd[0] = RGB_REG_RED;
    cmd[1] = red;
    ret = i2c_master_transmit(dev->rgb_dev, cmd, 2, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set red at 0x%02X: %s", RGB_REG_RED, esp_err_to_name(ret));
        return ret;
    }

    /* Set green (PWM1 - register 0x03) */
    cmd[0] = RGB_REG_GREEN;
    cmd[1] = green;
    ret = i2c_master_transmit(dev->rgb_dev, cmd, 2, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set green at 0x%02X: %s", RGB_REG_GREEN, esp_err_to_name(ret));
        return ret;
    }

    /* Set blue (PWM0 - register 0x02) */
    cmd[0] = RGB_REG_BLUE;
    cmd[1] = blue;
    ret = i2c_master_transmit(dev->rgb_dev, cmd, 2, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set blue at 0x%02X: %s", RGB_REG_BLUE, esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

esp_err_t grove_lcd_set_backlight(grove_lcd_handle_t handle, uint8_t intensity)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    grove_lcd_dev_t *dev = (grove_lcd_dev_t *)handle;
    
    /* Set all RGB channels to same intensity for white backlight */
    return grove_lcd_set_rgb(handle, intensity, intensity, intensity);
}
