# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.1.0] - 2026-02-28

### Added
- I2C device probing in nt3h2111_device_create() for better device detection
- CHANGELOG.md file to track version history

### Changed
- Improved error handling with proper error messages

## [1.0.0] - Previous Release

### Added
- NT3H2111 NFC/I2C bridge driver with ESP-IDF 5.x I2C master bus API
- NDEF message reading and writing
- Field detection functionality
- WiFi provisioning via NFC
- Memory block access
