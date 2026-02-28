# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.3.0] - 2026-02-28

### Changed
- **BREAKING**: Refactored to handle-based architecture for thread safety
- hp203b_init() now takes a device handle output parameter
- All functions now require hp203b_handle_t parameter
- Renamed hp203b_denit() to hp203b_deinit() for consistency

### Fixed
- Removed global state variables to support multiple device instances
- Fixed thread-safety issues
- Improved error handling in init function

### Added
- CHANGELOG.md file to track version history
- MIGRATION.md with detailed migration guide from v0.2.0
- Proper memory cleanup in error paths

**⚠️ See [MIGRATION.md](MIGRATION.md) for complete migration instructions.**

## [0.2.0] - Previous Release

### Added
- HP203B digital barometric pressure sensor driver with ESP-IDF 5.x I2C master bus API
- Pressure reading functionality
- Device probing support
