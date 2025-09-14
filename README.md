# ESP-IDF Components for Sensors

This repository provides **ESP-IDF components** for a set of common sensors, ready to integrate into your ESP32 projects.

Each component is written in C for the ESP-IDF build system and follows a similar structure:  
- Easy initialization with configurable I²C pins and frequency  
- Minimal, clean API with `esp_err_t` return codes  
- Examples for quick start

---

## Included Components

### MC3479 – 3-Axis Accelerometer
- **Interface:** I²C (up to 1 MHz)  
- **Function:** Measures acceleration on X, Y, Z axes  
- **Features:**  
  - ±2g to ±16g selectable ranges  
  - Low-power modes for battery applications  
  - Configurable data rates and interrupts  
- **Use cases:** Motion detection, step counting, tilt sensing

---

### T9602 – Humidity & Temperature Sensor
- **Interface:** I²C  
- **Function:** Provides relative humidity (%) and temperature (°C)  
- **Features:**  
  - Fully calibrated and compensated output  
  - Typical accuracy: ±2% RH, ±0.5 °C  
  - Factory calibrated, no external components required  
- **Use cases:** HVAC, weather stations, industrial monitoring

---

### SHTC3 – Humidity & Temperature Sensor
- **Interface:** I²C  
- **Function:** Measures relative humidity (%) and temperature (°C)  
- **Features:**  
  - Wide supply range (1.62–3.6 V)  
  - Typical accuracy: ±2% RH, ±0.2 °C  
  - Sleep and low-power modes  
- **Use cases:** Wearables, IoT nodes, handheld devices

---

### SHT4x – Next-Gen Humidity & Temperature Sensor
- **Interface:** I²C  
- **Function:** Provides high-accuracy humidity and temperature readings  
- **Features:**  
  - Improved accuracy: up to ±1.5% RH, ±0.1 °C  
  - Multiple measurement modes (trade-off speed vs power)  
  - Fully calibrated digital output  
- **Use cases:** Smart home devices, precision environmental monitoring

---

## Installation

Visit the Components published at [Registry](https://components.espressif.com/components?q=namespace%3Apedrominatel).