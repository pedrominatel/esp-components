# I2C Driver Component for the LC709203F Fuel Gauge

The LC709203F is an ultra-compact, highly accurate battery fuel gauge IC for single-cell lithium-ion batteries. It provides battery voltage, RSOC (Relative State of Charge), ITE (Indicator to Empty), and low battery alert notifications.

## Features

- **SMBus/I2C interface with CRC-8 error detection**
- Highly accurate battery fuel gauge with coulomb counting
- Battery voltage monitoring in 1mV resolution
- RSOC (Relative State of Charge) in 0.1% resolution
- ITE (Indicator to Empty) in 0.1% resolution
- Adjustable battery capacity settings (100mAh to 3000mAh)
- **Automatic thermistor detection** (auto-configures I2C or thermistor mode)
- Temperature compensation for accurate measurements
- **Current direction detection** (charge/discharge/auto)
- Low battery alarm function
- Ultra-low power consumption
- Sleep mode support
- Status register monitoring
- IC version readback

## Hardware Requirements

- ESP32 series microcontroller with I2C support
- LC709203F fuel gauge IC
- Single-cell lithium-ion/polymer battery
- Optional: 10kΩ NTC thermistor for temperature sensing

## How to use

### Initialize the I2C bus

```c
#include "driver/i2c_master.h"
#include "lc709203.h"

i2c_master_bus_config_t bus_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = I2C_NUM_0,
    .scl_io_num = GPIO_NUM_22,
    .sda_io_num = GPIO_NUM_21,
    .glitch_ignore_cnt = 7,
    .flags = {
        .enable_internal_pullup = true,
    },
};

i2c_master_bus_handle_t bus_handle;
ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));
```

### Create and configure the device

```c
// Create device handle
lc709203_handle_t sensor = lc709203_create(bus_handle, LC709203_I2C_ADDR);

// Check IC version
uint16_t version;
lc709203_read_ic_version(sensor, &version);
ESP_LOGI(TAG, "IC Version: 0x%04X", version);

// Set battery capacity (e.g., 1000mAh)
lc709203_set_apa(sensor, LC709203_APA_1000MAH);

// Set battery profile (0 = default)
lc709203_set_battery_profile(sensor, 0);

// Detect and configure temperature mode automatically
bool thermistor_available = false;
lc709203_detect_thermistor(sensor, &thermistor_available);
if (thermistor_available) {
    ESP_LOGI(TAG, "Using thermistor for temperature sensing");
} else {
    ESP_LOGI(TAG, "Using I2C mode with fixed temperature");
}

// Set current direction (auto-detect, charge, or discharge)
lc709203_set_current_direction(sensor, LC709203_CURRENT_DIR_AUTO);

// Initialize RSOC algorithm
lc709203_init_rsoc(sensor);
```

### Read battery data

```c
// Trigger sensor update (refreshes all measurements)
lc709203_trigger_update(sensor);

// Read cell voltage in mV
uint16_t voltage;
lc709203_read_cell_voltage(sensor, &voltage);

// Read RSOC in 0.1% units
uint16_t rsoc;
lc709203_read_rsoc(sensor, &rsoc);

// Read ITE (Indicator to Empty) in 0.1% units
uint16_t ite;
lc709203_read_ite(sensor, &ite);

ESP_LOGI(TAG, "Voltage: %u mV", voltage);
ESP_LOGI(TAG, "RSOC: %u.%u %%", rsoc / 10, rsoc % 10);
ESP_LOGI(TAG, "ITE: %u.%u %%", ite / 10, ite % 10);
```

### Read temperature

```c
uint16_t temp_kelvin;
lc709203_read_cell_temp(sensor, &temp_kelvin);

// Convert from 0.1K to Celsius
float temp_celsius = (temp_kelvin / 10.0f) - 273.15f;
ESP_LOGI(TAG, "Temperature: %.1f °C", temp_celsius);
```

### Read current direction

```c
uint16_t direction;
lc709203_read_current_direction(sensor, &direction);

if (direction == LC709203_CURRENT_DIR_DISCHARGE) {
    ESP_LOGI(TAG, "Battery is discharging");
} else if (direction == LC709203_CURRENT_DIR_CHARGE) {
    ESP_LOGI(TAG, "Battery is charging");
} else {
    ESP_LOGI(TAG, "Current direction: Auto-detect");
}
```

### Manual temperature setting (I2C mode)

```c
// Set temperature manually when thermistor is not available
lc709203_set_temperature_mode(sensor, 0);  // 0 = I2C mode
lc709203_set_cell_temp(sensor, 25);        // 25°C
```

### Configure low battery alarm

```c
// Set alarm threshold at 10% RSOC
lc709203_set_rsoc_alarm(sensor, 10);
```

### Power management

```c
// Enter sleep mode to save power
lc709203_set_power_mode(sensor, LC709203_POWER_MODE_SLEEP);

// Wake up and return to operational mode
lc709203_set_power_mode(sensor, LC709203_POWER_MODE_OPERATIONAL);
```

### Read status register

```c
uint16_t status;
lc709203_read_status(sensor, &status);
// 0x0000 = I2C temperature mode
// 0x0001 = Thermistor mode
```

### Clean up

```c
lc709203_delete(sensor);
```

## API Reference

### Battery Capacity Constants

| Constant | Capacity |
|----------|----------|
| `LC709203_APA_100MAH` | 100 mAh |
| `LC709203_APA_200MAH` | 200 mAh |
| `LC709203_APA_500MAH` | 500 mAh |
| `LC709203_APA_1000MAH` | 1000 mAh |
| `LC709203_APA_2000MAH` | 2000 mAh |
| `LC709203_APA_3000MAH` | 3000 mAh |

### Current Direction Constants

| Constant | Description |
|----------|-------------|
| `LC709203_CURRENT_DIR_DISCHARGE` | 0x0000 - Discharging |
| `LC709203_CURRENT_DIR_CHARGE` | 0x0001 - Charging |
| `LC709203_CURRENT_DIR_AUTO` | 0xFFFF - Auto-detect |

### Key Functions

- `lc709203_create()` - Create device handle
- `lc709203_delete()` - Delete device handle
- `lc709203_detect_thermistor()` - Auto-detect thermistor availability
- `lc709203_trigger_update()` - Force sensor to refresh measurements
- `lc709203_read_cell_voltage()` - Read battery voltage
- `lc709203_read_rsoc()` - Read state of charge
- `lc709203_read_ite()` - Read indicator to empty
- `lc709203_read_cell_temp()` - Read temperature
- `lc709203_read_current_direction()` - Read charge/discharge state
- `lc709203_read_status()` - Read status register
- `lc709203_read_ic_version()` - Read IC version
- `lc709203_set_apa()` - Set battery capacity
- `lc709203_set_battery_profile()` - Set battery chemistry profile
- `lc709203_set_cell_temp()` - Set temperature manually
- `lc709203_set_temperature_mode()` - Set I2C or thermistor mode
- `lc709203_set_current_direction()` - Set charge/discharge mode
- `lc709203_set_rsoc_alarm()` - Configure low battery alarm
- `lc709203_set_power_mode()` - Enter sleep or operational mode
- `lc709203_init_rsoc()` - Initialize/reset RSOC algorithm

## Important Notes

### CRC-8 Protocol
This driver implements the full SMBus protocol with CRC-8 error checking as required by the LC709203F. All I2C transactions include automatic CRC calculation and verification.

### Thermistor Detection
The driver automatically detects if a thermistor is connected:
- **Thermistor mode**: Sensor continuously monitors temperature (best accuracy)
- **I2C mode**: Temperature set manually via software (fallback when no thermistor)

Valid temperature range for detection: -20°C to +60°C

### RSOC Updates
RSOC (State of Charge) updates slowly because it uses coulomb counting:
- Requires accurate current direction setting (charge/discharge)
- Updates faster during active charge/discharge
- May take several minutes to reflect changes during idle state
- Periodic RSOC reinitialization helps maintain accuracy

### Current Direction
For accurate RSOC calculation:
- Set to `LC709203_CURRENT_DIR_CHARGE` when charging
- Set to `LC709203_CURRENT_DIR_DISCHARGE` when discharging
- Use `LC709203_CURRENT_DIR_AUTO` only if hardware auto-detect is wired
- The example includes voltage-based trend detection for automatic mode selection

## Example

See the `examples/lc709203_read` directory for a complete working example that demonstrates:
- Automatic thermistor detection
- Continuous battery monitoring
- Voltage-based charge/discharge detection
- Periodic measurement updates
- Temperature reading and display

## Resources

- [LC709203F Product Page](https://www.onsemi.com/products/power-management/battery-management/battery-fuel-gauges/lc709203f)
- [LC709203F Datasheet](https://www.onsemi.com/pdf/datasheet/lc709203f-d.pdf)
- [Application Note: Using LC709203F](https://www.onsemi.com/pub/Collateral/AND9379-D.PDF)

## License

Apache License 2.0
