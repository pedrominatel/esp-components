# SGP30 Read Example

This example reads TVOC (ppb) and eCO2 (ppm) from the SGP30 sensor once per second and logs the values over UART.

## Wiring

| SGP30 Pin | ESP32 Default |
|-----------|---------------|
| VCC       | 3.3 V         |
| GND       | GND           |
| SDA       | GPIO 2        |
| SCL       | GPIO 1        |

## Build and Flash

```bash
cd examples/sgp30_read
idf.py set-target esp32c3   # adjust for your target
idf.py menuconfig           # optional: change I2C pins under "Driver SGP30 Sensor"
idf.py build flash monitor
```

## Expected Output

```
I (xxx) SGP30: SGP30 sensor example
I (xxx) SGP30: SGP30 found!
I (xxx) SGP30: SGP30 initialized. Note: first 15 readings return fixed values ...
I (xxx) SGP30: Sensor initialization ok!
I (xxx) SGP30: eCO2: 400 ppm  |  TVOC: 0 ppb    <- warm-up readings
...
I (xxx) SGP30: eCO2: 412 ppm  |  TVOC: 3 ppb    <- live readings after ~15 s
```
