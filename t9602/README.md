# I2C Driver Component for the T9602 Temperature and Humidity Sensor

[![Component Registry](https://components.espressif.com/components/pedrominatel/t9602/badge.svg)](https://components.espressif.com/components/pedrominatel/t9602)

## Features

* Get Temperature in celsius
* Get Humidity in %RH

## Get Started

To get the component, use the command `add-dependency` inside your project folder.

```bash
idf.py add-dependency "pedrominatel/t9602"
```

### Create the I2C bus

Include the `t9602.h` into your code.

```c
#include "t9602.h"
```

Initialize the I2C bus.

```c
i2c_master_bus_handle_t i2c_bus_init(uint8_t sda_io, uint8_t scl_io)
{
    
    i2c_master_bus_config_t i2c_bus_config = {
        .i2c_port = -1,
        .sda_io_num = sda_io,
        .scl_io_num = scl_io,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus_handle;

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));

    return bus_handle;

}
```

### Initialize the sensor

```c

#define T9602_SDA_GPIO           17               /*!< gpio number for I2C master data  */
#define T9602_SCL_GPIO           18               /*!< gpio number for I2C master clock */

i2c_master_dev_handle_t t9602_handle;

static void t9602_sensor_init(void)
{
    
    i2c_master_bus_handle_t bus_handle = i2c_bus_init(T9602_SDA_GPIO, T9602_SCL_GPIO);
    t9602_handle = t9602_device_create(bus_handle, T9602_I2C_ADDR_0, 100000);
    ESP_LOGI(TAG, "Sensor initialization success");

}
```

### Read the values

```c
float temperature, humidity = {0};

t9602_get_data(t9602_handle, &temperature, &humidity);
```

## Examples

Please check the example **tab** or the **folder** to see on how to use the component.

## Resources

- [T9602 Datasheet](https://f.hubspotusercontent40.net/hubfs/9035299/Documents/AAS-920-638H-Telaire-T9602-041318-web.pdf)
- [T9602 Application Guide](https://f.hubspotusercontent40.net/hubfs/9035299/Documents/AAS-916-127J-Telaire-ChipCap2-022118-web.pdf)
