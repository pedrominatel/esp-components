# ESP-IDF Components for Sensors & Peripherals

This repository provides **ESP-IDF components** for a collection of common sensors and peripherals, ready to integrate into your ESP32 projects.

Each component is written in C for the ESP-IDF build system and follows a consistent structure:  
- Easy initialization with configurable I²C/SPI pins and frequency  
- Minimal, clean API with `esp_err_t` return codes  
- Multiple working examples for quick start  
- Kconfig integration for easy customization  
- Apache 2.0 licensed

---

## Included Components

| Component | Manufacturer | Interface | Key Features | Use Cases |
|-----------|--------------|-----------|--------------|-----------|
| **SHTC3** | Sensirion | I²C | Temp/Humidity, 1.62–3.6V, ±2% RH / ±0.2°C, Sleep mode | Wearables, IoT, handheld devices |
| **SHT4x** | Sensirion | I²C | Temp/Humidity, 2.4–5.5V, ±1.5% RH / ±0.1°C, Multi-mode | Smart home, precision monitoring |
| **T9602** | Amphenol | I²C | Temp/Humidity, 2.7–5.5V, ±2% RH / ±0.5°C, Calibrated | HVAC, weather stations, industry |
| **HP203B** | Hope Micro | I²C | Pressure, 1.7–3.6V, 300–1100 hPa, Altitude calc | Weather, altitude detection, drones |
| **MC3479** | mCube | I²C | 3-Axis accel, ±2–16g, Low-power modes, Interrupts | Motion detection, step counting, tilt |
| **LIS3DH** | STMicro | I²C/SPI | 3-Axis accel, ±2–16g, 2µA low-power, Self-test | Gesture recognition, activity monitor |
| **LC709203F** | ON Semi | I²C/SMBus | Fuel gauge, RSOC/ITE, CRC-8, Thermistor, Auto-detect | Battery monitoring, IoT, portable devices |
| **MAX17043** | Maxim | I²C | Fuel gauge, SOC/Voltage, Alert, Sleep mode, Compact | Battery devices, IoT, wearables |
| **NT3H2111** | NXP | I²C/NFC | NFC Tag, 1KB EEPROM, Dual access, NDEF, WiFi provisioning | NFC tags, contactless data, WiFi setup |
| **Grove LCD RGB** | Seeed Studio | I²C | 16x2 LCD, RGB backlight, 16.8M colors, Custom chars | Display text, status, sensor readings |
| **TMC2208** | Trinamic | UART/SPI | Stepper driver, 256µsteps, Stealthchop, Standalone | 3D printers, CNC, robotics |
| **SGP30** | Sensirion | I²C | TVOC (ppb) + eCO2 (ppm), Baseline persistence, CRC-8 | Air quality monitoring, IAQ, ventilation |

---

## Detailed Component Information

### Temperature & Humidity Sensors

#### SHTC3 – Digital Humidity & Temperature Sensor
- **Interface:** I²C  
- **Manufacturer:** Sensirion  
- **Features:**  
  - Supply range: 1.62–3.6 V  
  - Accuracy: ±2% RH, ±0.2 °C  
  - Sleep and low-power modes  
  - Ultra-low power consumption  
- **API:** Read temperature, humidity, sleep/wake control  
- **Use cases:** Wearables, IoT nodes, handheld devices, battery-powered applications

---

#### SHT4x – Next-Gen Digital Humidity & Temperature Sensor
- **Interface:** I²C  
- **Manufacturer:** Sensirion  
- **Features:**  
  - Improved accuracy: up to ±1.5% RH, ±0.1 °C  
  - Multiple measurement modes (high/medium/low power)  
  - Fully calibrated digital output  
  - Supply range: 2.4–5.5 V  
- **API:** Read temperature, humidity with selectable precision  
- **Use cases:** Smart home devices, precision environmental monitoring, weather stations

---

#### T9602 – Humidity & Temperature Sensor
- **Interface:** I²C  
- **Manufacturer:** Amphenol  
- **Features:**  
  - Fully calibrated and compensated output  
  - Accuracy: ±2% RH, ±0.5 °C  
  - Factory calibrated, no external components required  
  - Supply range: 2.7–5.5 V  
- **API:** Read temperature, humidity, status  
- **Use cases:** HVAC systems, weather stations, industrial monitoring, environmental control

---

#### HP203B – Digital Barometric Pressure Sensor
- **Interface:** I²C  
- **Manufacturer:** Hope Microelectronics  
- **Features:**  
  - Barometric pressure measurement (300–1100 hPa)  
  - Temperature compensation  
  - Altitude calculation capability  
  - Supply range: 1.7–3.6 V  
  - Ultra-low power consumption  
- **API:** Read pressure, temperature, altitude  
- **Use cases:** Weather stations, altitude detection, drone applications, altimeters

---

### Motion & Acceleration Sensors

#### MC3479 – 3-Axis Accelerometer
- **Interface:** I²C (up to 1 MHz)  
- **Manufacturer:** mCube  
- **Features:**  
  - Selectable ranges: ±2g to ±16g  
  - Multiple low-power modes  
  - Configurable output data rates  
  - Programmable interrupt generators  
- **API:** Read acceleration (X, Y, Z), set ranges, data rates, interrupts  
- **Use cases:** Motion detection, step counting, tilt sensing, activity monitoring

---

#### LIS3DH – 3-Axis Accelerometer
- **Interface:** I²C/SPI (up to 10 MHz)  
- **Manufacturer:** STMicroelectronics  
- **Features:**  
  - Selectable ranges: ±2g to ±16g  
  - Ultra-low power: down to 2 µA in low-power mode  
  - Embedded self-test  
  - Multiple interrupt generators  
  - High resolution mode support  
- **API:** Read acceleration, set ODR, full-scale range, WHO_AM_I verification  
- **Use cases:** Gesture recognition, activity monitoring, freefall detection, gaming peripherals

---

### Power Management & Fuel Gauge

#### LC709203F – Ultra-Accurate Battery Fuel Gauge with CRC
- **Interface:** I²C/SMBus with CRC-8  
- **Manufacturer:** ON Semiconductor  
- **Features:**  
  - SMBus protocol with automatic CRC-8 error detection  
  - Voltage measurement in 1mV resolution  
  - RSOC (Relative State of Charge) in 0.1% resolution  
  - ITE (Indicator to Empty) in 0.1% resolution  
  - Automatic thermistor detection (I2C or thermistor mode)  
  - Current direction control (charge/discharge/auto)  
  - Configurable battery capacity (100mAh–3000mAh)  
  - Temperature compensation for accuracy  
  - Sleep mode with ultra-low power consumption  
  - Status register and IC version readback  
- **API:** Read voltage, RSOC, ITE, temperature, current direction, auto thermistor detect, trigger updates, set capacity, alarms  
- **Use cases:** Portable devices, IoT nodes, battery-powered sensors, wearables, power banks

---

#### MAX17043 – Ultra-Compact Fuel Gauge
- **Interface:** I²C  
- **Manufacturer:** Maxim Integrated  
- **Features:**  
  - Simple state-of-charge (SOC) monitoring  
  - Battery voltage measurement  
  - Programmable alert threshold (0–32%)  
  - Sleep mode for ultra-low power  
  - Quick start calibration  
- **API:** Read voltage, SOC, set alerts, sleep/wake, quick start  
- **Use cases:** Battery-powered devices, IoT nodes, portable electronics, wearables

---
NFC & RFID

#### NT3H2111 – NFC Forum Type 2 Tag with I²C Interface
- **Interface:** I²C (up to 400 kHz) + NFC RF (13.56 MHz)  
- **Manufacturer:** NXP Semiconductors  
- **Features:**  
  - Dual interface: I²C and NFC RF simultaneous access  
  - 1KB EEPROM user memory (888 bytes usable)  
  - SRAM buffer for fast data exchange (64 bytes)  
  - NFC Forum Type 2 Tag compliant  
  - NDEF message support with WiFi Simple Configuration (WSC)  
  - Field detection pin with interrupt support  
  - Session registers for RF status monitoring  
  - Password protection and memory locking  
  - Energy harvesting from NFC field  
- **API:** Read/Write blocks, field detection, NDEF parsing, WiFi credential extraction, session register monitoring  
- **Use cases:** NFC tags, contactless data exchange, WiFi provisioning, secure pairing, asset tracking, smart posters

---

### Air Quality Sensors

#### SGP30 – Multi-Gas (TVOC + eCO2) Sensor
- **Interface:** I²C (fixed address 0x58)  
- **Manufacturer:** Sensirion  
- **Features:**  
  - TVOC measurement: 0–60,000 ppb  
  - eCO2 equivalent: 400–60,000 ppm  
  - On-chip humidity compensation  
  - Baseline persistence for calibration across power cycles  
  - CRC-8 error detection on every response word  
  - Compatible with both the raw `i2c_master` API and the `espressif/i2c_bus` component  
  - Supply range: 1.62–1.98 V  
  > **Note:** SGP30 is end-of-life. Sensirion recommends the SGP40/SGP41 for new designs.  
- **API:** `sgp30_init`, `sgp30_measure`, `sgp30_get_baseline`, `sgp30_set_baseline`, `sgp30_soft_reset`, `sgp30_init_i2cbus`  
- **Use cases:** Indoor air quality monitoring, demand-controlled ventilation, smart home, IAQ logging

---

### Display Modules

#### Grove LCD RGB Backlight v4.0 – 16x2 Character LCD Display
- **Interface:** I²C (dual addresses)  
- **Manufacturer:** Seeed Studio  
- **Features:**  
  - 16 columns × 2 rows character display  
  - Full RGB backlight control (16.8 million colors)  
  - AIP31068L LCD controller (0x3E)  
  - SGM31323 RGB LED driver (0x62)  
  - Custom character support  
  - Cursor control and positioning  
- **API:** Print text, set cursor, control RGB backlight, clear display, custom characters  
- **Use cases:** Status displays, sensor readouts, menu systems, user interfaces

---

### Motion Control

#### TMC2208 – Stepper Motor Driver
- **Interface:** UART/SPI  
- **Manufacturer:** Trinamic  
- **Features:**  
  - Integrated microstepping (up to 256 microsteps)  
  - Stealthchop for quiet operation  
  - Current regulation  
  - UART interface for easy control  
  - Standalone operation without microcontroller  
- **API:** Motor control, speed adjustment, direction, microstepping  
- **Use cases:** 3D printers, CNC machines, robotics, stepper motor applications

---

## Installation & Usage

### Via Espressif Component Service

All components are published at the [Espressif Component Registry](https://components.espressif.com/components?q=namespace%3Apedrominatel). Add to your project with:

```bash
idf.py add-dependency "pedrominatel/<component-name>"
```

### Via Git

Clone this repository and reference the components:

```bash
git clone https://github.com/pedrominatel/esp-components.git
```

Then in your `idf_component.yml`:

```yaml
dependencies:
  pedrominatel/<component-name>:
    version: "*"
    path: ../esp-components/<component-name>
```

---

## Component Structure

Each component follows this standard structure:

```
component-name/
├── CMakeLists.txt           # Build configuration
├── Kconfig                  # Configuration options
├── idf_component.yml        # Component metadata
├── README.md                # Component documentation
├── license.txt              # Apache 2.0 License
├── include/
│   ├── component.h          # Main API header
│   └── component_config.h   # Configuration macros
├── component.c              # Driver implementation
└── examples/
    ├── example_basic/       # Basic usage
    ├── example_advanced/    # Advanced features
    └── ...
```

---

## Features

✅ **Simple API** - Minimal, intuitive functions with standard ESP-IDF error handling  
✅ **I²C/SPI Support** - Most components use standard ESP-IDF drivers  
✅ **Kconfig Integration** - Easy hardware pin configuration via `idf.py menuconfig`  
✅ **Examples** - Multiple working examples for each component  
✅ **Well Documented** - API documentation, examples, and use cases  
✅ **Active Maintenance** - Regular updates and improvements  

---

## Building Examples

To build an example:

```bash
cd esp-components/<component-name>/examples/<example-name>
idf.py build
idf.py flash
idf.py monitor
```

---

## CI/CD

- **Build Testing:** Examples are automatically built and tested against:
  - ESP-IDF versions: v5.3, v5.4, v5.5, and latest
  - Target chips: ESP32, ESP32-S3, ESP32-C3, ESP32-C6
  - All examples in component folders and the examples directory
- **Component Publishing:** Components are automatically published to the registry on push to main

---

## Contributing

Contributions are welcome! Please ensure:
- Code follows the existing style
- Components include examples and documentation
- README.md is updated with component information

---

## License

All components are licensed under the **Apache License 2.0** - see individual `license.txt` files for details.

---

## Resources

- [Espressif IoT Development Framework (ESP-IDF)](https://github.com/espressif/esp-idf)
- [Component Registry](https://components.espressif.com)
- [ESP32 Documentation](https://docs.espressif.com)
