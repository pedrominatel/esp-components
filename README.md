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
| **MAX17043** | Maxim | I²C | Fuel gauge, SOC/Voltage, Alert, Sleep mode, Compact | Battery devices, IoT, wearables |
| **MAX17055** | Maxim | I²C | Advanced fuel gauge, Current, Time-to-empty | Smartphones, tablets, UPS systems |
| **TMC2208** | Trinamic | UART/SPI | Stepper driver, 256µsteps, Stealthchop, Standalone | 3D printers, CNC, robotics |

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

#### MAX17055 – Advanced Fuel Gauge with ModelGauge m5
- **Interface:** I²C  
- **Manufacturer:** Maxim Integrated  
- **Features:**  
  - Advanced fuel gauge with better accuracy than MAX17043  
  - Current measurement (charge/discharge rates)  
  - Time-to-empty calculation  
  - Configurable alert thresholds  
  - Register-based configuration  
- **API:** Read SOC, voltage, current, time-to-empty, configuration  
- **Use cases:** Smartphones, tablets, high-accuracy battery monitoring, UPS systems

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
  - Target chips: ESP32, ESP32-S3, ESP32-C3
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
