# Example for the LC709203F

Example to read battery voltage, RSOC (Relative State of Charge), and ITE (Indicator to Empty) from the LC709203F fuel gauge.

## Description

This example will continuously read and print the battery cell voltage (in mV), RSOC (in 0.1%), and ITE (in 0.1%) to the console. The readings are updated every 2 seconds.

The example also demonstrates how to:
- Initialize the LC709203F device
- Configure battery capacity (APA setting)
- Set battery profile
- Set cell temperature for improved accuracy
- Initialize the RSOC algorithm

## Hardware Required

- ESP32 development board
- LC709203F fuel gauge IC connected via I2C
- Li-ion battery connected to the LC709203F

## How to Use

Configure the I2C pins in menuconfig under `LC709203F Configuration`.

Build and flash the example:

```bash
idf.py build flash monitor
```

## Expected Output

```
I (xxx) LC709203_EXAMPLE: Starting LC709203F fuel gauge example
I (xxx) LC709203_EXAMPLE: I2C bus initialized
I (xxx) LC709203_EXAMPLE: LC709203F detected on I2C bus
I (xxx) LC709203_EXAMPLE: LC709203F device created successfully
I (xxx) LC709203_EXAMPLE: LC709203F IC Version: 0x2003
I (xxx) LC709203_EXAMPLE: Battery capacity configured for 1000mAh
I (xxx) LC709203_EXAMPLE: Battery profile set to default
I (xxx) LC709203_EXAMPLE: Cell temperature set to 25°C
I (xxx) LC709203_EXAMPLE: RSOC initialized
I (xxx) LC709203_EXAMPLE: Cell Voltage: 3850 mV
I (xxx) LC709203_EXAMPLE: RSOC: 75.2 %
I (xxx) LC709203_EXAMPLE: ITE: 75.1 %
I (xxx) LC709203_EXAMPLE: ---
```
