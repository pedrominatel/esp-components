# LC709203F Low Battery Alarm Example

Example demonstrating how to use the LC709203F fuel gauge to monitor battery level and trigger a low battery alarm.

## Description

This example monitors the battery RSOC (Relative State of Charge) and triggers an alarm when the battery level falls below a configured threshold (15% by default).

The example demonstrates:
- Setting up RSOC alarm threshold
- Continuous battery monitoring
- Low battery alert detection
- Battery voltage and RSOC reporting

## Hardware Required

- ESP32 development board
- LC709203F fuel gauge IC connected via I2C
- Li-ion battery connected to the LC709203F

## How to Use

Configure the I2C pins in menuconfig under `LC709203F Configuration`.

You can adjust the `LOW_BATTERY_THRESHOLD` in main.c to change the alarm threshold.

Build and flash the example:

```bash
idf.py build flash monitor
```

## Expected Output

```
I (xxx) LC709203_ALARM: Starting LC709203F low battery alarm example
I (xxx) LC709203_ALARM: I2C bus initialized
I (xxx) LC709203_ALARM: LC709203F detected on I2C bus
I (xxx) LC709203_ALARM: LC709203F device created successfully
I (xxx) LC709203_ALARM: Battery capacity configured for 1000mAh
I (xxx) LC709203_ALARM: Battery profile set to default
I (xxx) LC709203_ALARM: RSOC alarm threshold set to 15%
I (xxx) LC709203_ALARM: RSOC initialized
I (xxx) LC709203_ALARM: RSOC: 75.2 %
I (xxx) LC709203_ALARM: Cell Voltage: 3850 mV
```

When battery drops below threshold:

```
W (xxx) LC709203_ALARM: ⚠️  LOW BATTERY ALERT: Battery below 15%!
```
