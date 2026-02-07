# LC709203F Sleep Mode Example

Example demonstrating how to use the LC709203F power modes to minimize power consumption.

## Description

This example demonstrates the LC709203F power mode control:

1. **Operational Mode**: Device wakes up and reads battery data for 5 seconds
2. **Sleep Mode**: Device enters low-power sleep mode for 10 seconds
3. The cycle repeats continuously

This is useful for battery-powered applications where you want to minimize power consumption by putting the fuel gauge to sleep when not actively monitoring.

## Features Demonstrated

- Entering and exiting sleep mode
- Power mode management
- Battery monitoring during wake periods
- Low-power operation

## Hardware Required

- ESP32 development board
- LC709203F fuel gauge IC connected via I2C
- Li-ion battery connected to the LC709203F

## How to Use

Configure the I2C pins in menuconfig under `LC709203F Configuration`.

You can adjust `SLEEP_DURATION_MS` and `WAKE_DURATION_MS` in main.c to change the timing.

Build and flash the example:

```bash
idf.py build flash monitor
```

## Expected Output

```
I (xxx) LC709203_SLEEP: Starting LC709203F sleep mode example
I (xxx) LC709203_SLEEP: This example demonstrates power mode control:
I (xxx) LC709203_SLEEP: - Wake up and read battery data for 5 seconds
I (xxx) LC709203_SLEEP: - Enter sleep mode for 10 seconds
I (xxx) LC709203_SLEEP: I2C bus initialized
I (xxx) LC709203_SLEEP: LC709203F detected on I2C bus
I (xxx) LC709203_SLEEP: LC709203F device created successfully
I (xxx) LC709203_SLEEP: LC709203F IC Version: 0x2003
I (xxx) LC709203_SLEEP: Battery capacity configured for 1000mAh
I (xxx) LC709203_SLEEP: Battery profile set to default
I (xxx) LC709203_SLEEP: RSOC initialized
I (xxx) LC709203_SLEEP: Entering operational mode...
I (xxx) LC709203: LC709203 entering operational mode
I (xxx) LC709203_SLEEP: Cell Voltage: 3850 mV
I (xxx) LC709203_SLEEP: RSOC: 75.2 %
I (xxx) LC709203_SLEEP: Cell Voltage: 3848 mV
I (xxx) LC709203_SLEEP: RSOC: 75.1 %
...
I (xxx) LC709203_SLEEP: Entering sleep mode for 10 seconds...
I (xxx) LC709203: LC709203 entering sleep mode
```

## Power Savings

When in sleep mode, the LC709203F consumes significantly less current (typically < 1µA), making it ideal for battery-powered applications.
