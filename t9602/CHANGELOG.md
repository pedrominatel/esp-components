# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.3.0] - 2026-02-28

### Changed
- Improved error handling by replacing ESP_ERROR_CHECK() with proper error returns in t9602_device_create()
- Better error messages with esp_err_to_name()

### Added
- I2C device probing in t9602_device_create() for better device detection
- CHANGELOG.md file to track version history

## [0.2.0] - Previous Release

### Added
- T9602 temperature and humidity sensor driver with ESP-IDF 5.x I2C master bus API
- Temperature reading in Celsius
- Humidity reading in %RH
- Simple I2C interface
