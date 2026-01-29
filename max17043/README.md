# I2C Driver Component for the MAX17043 Fuel Gauge

The MAX17043 is a compact, ultra-low power fuel gauge IC for lithium ion (Li+) batteries used in portable equipment. It accurately predicts the battery capacity and provides an alert when battery is depleted.

## Features

- Simple I2C interface
- Fuel gauge functionality
- Battery voltage monitoring
- Low power consumption
- Programmable alert threshold

## How to use

### Init the I2C bus

```c
i2c_master_bus_config_t i2c_bus_config = {
    .i2c_port = CONFIG_MAX17043_I2C_NUM,
    .sda_io_num = CONFIG_MAX17043_I2C_SDA,
    .scl_io_num = CONFIG_MAX17043_I2C_SCL,
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
};

i2c_master_bus_handle_t bus_handle;
ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));
```

### Init the device

```c
max17043_handle_t handle = max17043_create(bus_handle, MAX17043_I2C_ADDR);
```

### Read battery data

```c
uint16_t voltage;
uint8_t soc;
max17043_read_voltage(handle, &voltage);
max17043_read_soc(handle, &soc);
```

For more details on how to use, see the provided example.

## Resources

- [MAX17043 Product Page](https://www.maxim-ic.com/products/power-management/battery-management/MAX17043)
- [MAX17043 Datasheet](https://datasheets.maximintegrated.com/en/ds/MAX17043.pdf)
