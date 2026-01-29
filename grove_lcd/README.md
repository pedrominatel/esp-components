# Grove LCD RGB Backlight v4.0

The Grove LCD RGB Backlight v4.0 is a 16x2 character LCD display with full RGB backlight control. It features an I²C interface with the AIP31068L LCD controller and SGM31323 RGB LED driver, perfect for displaying text, sensor readings, and status information with customizable backlight colors.

## Hardware

- **LCD Controller**: AIP31068L (I2C address: 0x3E)
- **RGB Backlight**: SGM31323 LED driver (I2C address: 0x62)
- **Display**: 16 columns × 2 rows
- **Interface**: I2C (100 kHz)

## Features

- 16x2 character LCD display
- RGB backlight with 16.8 million colors
- I²C interface (two separate addresses for LCD and RGB)
- Configurable contrast
- Cursor control
- Custom character support
- Easy-to-use C API

## How to use

### Init the I2C bus

```c
i2c_master_bus_config_t i2c_bus_config = {
    .i2c_port = CONFIG_GROVE_LCD_I2C_NUM,
    .sda_io_num = CONFIG_GROVE_LCD_I2C_SDA,
    .scl_io_num = CONFIG_GROVE_LCD_I2C_SCL,
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
};

i2c_master_bus_handle_t bus_handle;
ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));
```

### Init the device

```c
grove_lcd_handle_t lcd = grove_lcd_create(bus_handle, CONFIG_GROVE_LCD_I2C_ADDR, CONFIG_GROVE_LCD_RGB_ADDR);
```

### Display text

```c
grove_lcd_print(lcd, "Hello World!");
grove_lcd_set_cursor(lcd, 0, 1);  /* Row 1, Column 0 */
grove_lcd_print(lcd, "ESP32");
```

### Control backlight

```c
grove_lcd_set_rgb(lcd, 255, 0, 0);     /* Red */
grove_lcd_set_rgb(lcd, 0, 255, 0);     /* Green */
grove_lcd_set_rgb(lcd, 0, 0, 255);     /* Blue */
```

For more details on how to use, see the provided examples.

## Resources

- [Grove LCD RGB Backlight v4.0](https://wiki.seeedstudio.com/Grove-16x2_LCD_Series/)
- [Datasheet](https://files.seeedstudio.com/wiki/Grove-16x2_LCD_Series/res/LCD1602_RGB_datasheet.pdf)
