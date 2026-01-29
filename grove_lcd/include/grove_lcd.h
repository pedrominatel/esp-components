#ifndef GROVE_LCD_H
#define GROVE_LCD_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/* I2C Addresses */
#define GROVE_LCD_ADDR      0x3E  /* AIP31068L LCD controller */
#define GROVE_RGB_ADDR      0x62  /* SGM31323 RGB backlight */

/* RGB Backlight Registers (SGM31323) */
#define RGB_REG_MODE1       0x00
#define RGB_REG_MODE2       0x01
#define RGB_REG_PWM0        0x02  /* Blue channel */
#define RGB_REG_PWM1        0x03  /* Green channel */
#define RGB_REG_PWM2        0x04  /* Red channel */
#define RGB_REG_RED         RGB_REG_PWM2
#define RGB_REG_GREEN       RGB_REG_PWM1
#define RGB_REG_BLUE        RGB_REG_PWM0

/* Device handle */
typedef void* grove_lcd_handle_t;

/**
 * @brief Create Grove LCD device handle
 * @param bus_handle I2C bus handle
 * @param lcd_addr I2C address of LCD
 * @param rgb_addr I2C address of RGB backlight
 * @return Device handle on success, NULL on failure
 */
grove_lcd_handle_t grove_lcd_create(i2c_master_bus_handle_t bus_handle, uint8_t lcd_addr, uint8_t rgb_addr);

/**
 * @brief Delete Grove LCD device handle
 * @param handle Device handle
 * @return ESP_OK on success
 */
esp_err_t grove_lcd_delete(grove_lcd_handle_t handle);

/**
 * @brief Clear LCD display
 * @param handle Device handle
 * @return ESP_OK on success
 */
esp_err_t grove_lcd_clear(grove_lcd_handle_t handle);

/**
 * @brief Move cursor to home position (0, 0)
 * @param handle Device handle
 * @return ESP_OK on success
 */
esp_err_t grove_lcd_home(grove_lcd_handle_t handle);

/**
 * @brief Set cursor position
 * @param handle Device handle
 * @param row Row (0 or 1)
 * @param col Column (0-15)
 * @return ESP_OK on success
 */
esp_err_t grove_lcd_set_cursor(grove_lcd_handle_t handle, uint8_t row, uint8_t col);

/**
 * @brief Print text to LCD
 * @param handle Device handle
 * @param text Text to print (up to 16 characters per line)
 * @return ESP_OK on success
 */
esp_err_t grove_lcd_print(grove_lcd_handle_t handle, const char *text);

/**
 * @brief Set cursor visibility and blink
 * @param handle Device handle
 * @param cursor Show cursor (true/false)
 * @param blink Cursor blink (true/false)
 * @return ESP_OK on success
 */
esp_err_t grove_lcd_cursor(grove_lcd_handle_t handle, bool cursor, bool blink);

/**
 * @brief Enable/disable display
 * @param handle Device handle
 * @param enable Display on/off
 * @return ESP_OK on success
 */
esp_err_t grove_lcd_display(grove_lcd_handle_t handle, bool enable);

/**
 * @brief Set RGB backlight color
 * @param handle Device handle
 * @param red Red value (0-255)
 * @param green Green value (0-255)
 * @param blue Blue value (0-255)
 * @return ESP_OK on success
 */
esp_err_t grove_lcd_set_rgb(grove_lcd_handle_t handle, uint8_t red, uint8_t green, uint8_t blue);

/**
 * @brief Set backlight intensity (PWM mode)
 * @param handle Device handle
 * @param intensity Intensity value (0-255)
 * @return ESP_OK on success
 */
esp_err_t grove_lcd_set_backlight(grove_lcd_handle_t handle, uint8_t intensity);

#ifdef __cplusplus
}
#endif

#endif /* GROVE_LCD_H */
