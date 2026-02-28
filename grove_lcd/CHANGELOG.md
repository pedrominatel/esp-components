# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.2.0] - 2026-02-28

### Added
- I2C device probing in grove_lcd_create() for better device detection
- Probe both LCD and RGB devices before adding to bus
- CHANGELOG.md file to track version history

## [1.1.0] - Previous Release

### Added
- Grove LCD RGB Backlight driver with ESP-IDF I2C master bus API
- Support for RGB backlight control
- Text display functionality
- Cursor positioning functions
