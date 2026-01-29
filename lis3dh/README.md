# I2C Driver Component for the LIS3DH 3-Axis Accelerometer

The LIS3DH is an ultra-low-power high-performance three-axis linear accelerometer with digital I2C interface. It features ultra-low-power operating modes, embedded self-test, and programmable interrupt generators.

## Features

- Read 3-axis acceleration data (X, Y, Z)
- Selectable full scale ranges: ±2g, ±4g, ±8g, ±16g
- Configurable output data rates from 1 Hz to 400 Hz
- WHO_AM_I register verification
- High resolution mode support
- Ultra-low power consumption

## How to use

### Init the I2C bus

```c
i2c_master_bus_config_t bus_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = 0,
    .scl_io_num = LIS3DH_I2C_SCL_PIN,
    .sda_io_num = LIS3DH_I2C_SDA_PIN,
    .glitch_ignore_cnt = 7,
    .flags = {
        .enable_internal_pullup = true,
    },
};

i2c_master_bus_handle_t bus_handle;
ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));
ESP_LOGI(TAG, "I2C master bus created");
```

### Init the device

```c
lis3dh_config_t sensor_config = {
    .i2c_bus = bus_handle,
    .i2c_addr = LIS3DH_ADDR_LOW,  /* or LIS3DH_ADDR_HIGH depending on SA0 pin */
    .odr = LIS3DH_ODR_100_HZ,
    .fs = LIS3DH_FS_2G,
};

lis3dh_handle_t sensor_handle;
ESP_ERROR_CHECK(lis3dh_init(&sensor_config, &sensor_handle));
```

### Read acceleration values

```c
lis3dh_accel_t accel;
ESP_ERROR_CHECK(lis3dh_read_accel(sensor_handle, &accel));
ESP_LOGI(TAG, "Accel - X: %d, Y: %d, Z: %d", accel.x, accel.y, accel.z);
```

For more details on how to use, see the provided example.

## Resources

- [LIS3DH Product Page](https://www.st.com/en/mems-and-sensors/lis3dh.html)
- [LIS3DH Datasheet](https://www.st.com/resource/en/datasheet/lis3dh.pdf)
- [LIS3DH Application Notes](https://www.st.com/resource/en/application_note/cd00290365.pdf)
