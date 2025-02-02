# Stepper motor driver for the TMC2208 driver

This driver for the [TMC22xx](https://www.analog.com/media/en/technical-documentation/data-sheets/TMC2202_TMC2208_TMC2224_datasheet_rev1.14.pdf) is compatible with the TMC2202, TMC2208, TMC2224 Step/Dir Drivers for Two-Phase Bipolar Stepper Motors and it was developed using the [RMT example](https://github.com/espressif/esp-idf/tree/master/examples/peripherals/rmt/stepper_motor) from the ESP-IDF.

> This driver is currently only compatible with the STEP/DIR Interface.

## Prerequisites

To use this driver, the RMT peripheral is required. Ensure that your ESP32 board supports the RMT peripheral and that it is enabled in your ESP-IDF project configuration.

## Features

- Steps Mode
- Continuous mode
- Microstep

The UART interface will be supported soon.

## How to use

To use this driver, you will need to connect the following IOs.

### GPIO Configuration

- **STEP Pin**: Connect to the STEP input of the TMC2208.
- **DIR Pin**: Connect to the DIR input of the TMC2208.
- **EN Pin**: Connect to the EN input of the TMC2208.
- **MS1 Pin**: Connect to the MS1 input of the TMC2208 for the microstep configuration.
- **MS2 Pin**: Connect to the MS2 input of the TMC2208 for the microstep configuration.

Make sure to configure these pins in your ESP-IDF project according to your specific GPIO assignments.

Set the IO and the motor configuration.

```c
    tmc2208_io_config_t config_io_motor1 = {
        .step_pin = 5,
        .dir_pin = 4,
        .enable_pin = 10,
        .ms1_pin = 6,
        .ms2_pin = 7,
    };

    ESP_ERROR_CHECK(tmc2208_init(&config_io_motor1));
    ESP_LOGI(TAG, "Driver initialization done");

    tmc2208_motor_config_t motor1_config = {
        .enable_level = 0,
        .dir_clockwise = 0,
        .resolution_hz = 500000,
        .start_freq_hz = 100,
        .end_freq_hz = 100,
        .accel_samples = 250,
        .uniform_speed_hz = 500,
        .decel_samples = 250,
        .microstep = TMC2208_MICROSTEP_4
    };
```

To move in steps mode:

```c
int32_t steps = 1600;
tmc2208_move_steps(&config_io_motor1, &motor1_config, steps, false, false);
```

For the continuous mode:

```c
   // Test the uniform continous move
    // Enable motor with the encoder
    tmc2208_enable(&config_io_motor1, &motor1_config);
    // Set the speed
    motor1_config.uniform_speed_hz = 10;
    // Set the direction
    tmc2208_set_dir(&config_io_motor1, &motor1_config, TMC2208_DIR_CCW);
    // Move 400 steps
    uint32_t steps = 400;
    while (1) {
        tmc2208_uniform_move_steps(&config_io_motor1, &motor1_config, 10);
        steps -= 10;
        if(steps == 0) {
            break;
        }
    }
    // Disable motor
    tmc2208_disable(&config_io_motor1, &motor1_config);
```

## Resources

- Datasheet [TMC22xx](https://www.analog.com/media/en/technical-documentation/data-sheets/TMC2202_TMC2208_TMC2224_datasheet_rev1.14.pdf)