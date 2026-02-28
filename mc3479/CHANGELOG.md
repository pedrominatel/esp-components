# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.2.0] - 2026-02-28

### Added
- Kconfig file for I2C configuration options (I2C_NUM, SDA, SCL, CLK_SPEED, ADDRESS)
- Support for both possible I2C addresses (0x4C and 0x6C)
- CHANGELOG.md file to track version history

## [0.1.0] - Previous Release

### Added
- MC3479 3-axis accelerometer driver with ESP-IDF 5.x I2C master bus API
- Basic motion detection (anymotion)
- Chip ID verification
- Configurable accelerometer range and resolution
