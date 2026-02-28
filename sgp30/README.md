# SGP30 Component

ESP-IDF component driver for the **Sensirion SGP30** multi-gas sensor (TVOC + eCO2).

> **Note:** The SGP30 is end-of-life. Sensirion's recommended successor is the SGP40/SGP41. This driver is provided for existing hardware designs.

## Features

- Measures Total Volatile Organic Compounds (TVOC) in ppb
- Measures equivalent CO2 (eCO2) in ppm
- On-chip humidity compensation (requires calling `Set_humidity`, optional)
- Baseline persistence — save and restore calibration state across power cycles
- CRC-8 validation on every I2C response word
- Uses the new ESP-IDF `i2c_master` API (IDF ≥ 5.3)

## Hardware

| SGP30 Pin | ESP32 Default |
|-----------|---------------|
| VCC       | 3.3 V         |
| GND       | GND           |
| SDA       | GPIO 2        |
| SCL       | GPIO 1        |

The SGP30 has a fixed I2C address of `0x58`.

## Configuration

Use `idf.py menuconfig` → **Driver SGP30 Sensor** → **I2C** to configure:

- `SGP30_I2C_NUM` — I2C peripheral index (`-1` for auto)
- `SGP30_I2C_SDA` — SDA GPIO (default 2)
- `SGP30_I2C_SCL` — SCL GPIO (default 1)
- `SGP30_I2C_CLK_SPEED_HZ` — I2C clock speed (default 100 000 Hz)

## API

```c
// Initialize the sensor (call once after creating the I2C master bus)
esp_err_t sgp30_init(i2c_master_bus_handle_t bus_handle);

// Release the device handle
void sgp30_denit(void);

// Read TVOC (ppb) and eCO2 (ppm) — call once per second
esp_err_t sgp30_measure(sgp30_measurement_t *measurement);

// Save baseline for persistence
esp_err_t sgp30_get_baseline(sgp30_baseline_t *baseline);

// Restore a previously saved baseline (call after sgp30_init)
esp_err_t sgp30_set_baseline(const sgp30_baseline_t *baseline);

// Soft reset
esp_err_t sgp30_soft_reset(void);
```

## Important: Sensor Warm-Up

The SGP30 must be called **once per second** with `sgp30_measure()`. For the first 15 seconds after `sgp30_init()`, the sensor returns placeholder values (400 ppm eCO2, 0 ppb TVOC) while the on-chip algorithm initialises. Valid readings begin after the warm-up period.

## Examples

- [`sgp30_read`](examples/sgp30_read) — reads TVOC and eCO2 every second and logs the values.

## License

Apache-2.0 — see [license.txt](license.txt).
