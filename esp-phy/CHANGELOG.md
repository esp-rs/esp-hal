# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added


### Changed


### Fixed


### Removed


## [v0.3.0] - 2026-09-15

### Added

- Support for ESP32-S31 (#6296)
- set_wifi_rx_enabled to toggle the default Wi-Fi RX state on combo modules. (#5776)

### Changed

- Updated radio blobs to ESP-IDF `6.1-255-g4dbfecac7e3`. (#6271)
- updated defmt to 1.1 (#5752)

## [v0.2.0] - 2026-04-16

### Added

- `last_calibration_result` to get the result of the last calibration (#4479)
- Support for ESP32-C5 (#5003)
- Support for ESP32-C61 (#5255)

### Changed

- Use drivers from ESP-IDF v5.5.3 (#5226)

### Fixed

- Fix PHY clock reference counter leak in `disable_phy` (#5322)

### Removed

- The `PhyController` extension traits on the radio peripherals got removed in favor of a free standing `enable_phy` function (#5205)

## [v0.1.1] - 2025-10-30

## [v0.1.0] - 2025-10-13

### Added

- Initial release (#3892)

[v0.1.0]: https://github.com/esp-rs/esp-hal/releases/tag/esp-phy-v0.1.0
[v0.1.1]: https://github.com/esp-rs/esp-hal/compare/esp-phy-v0.1.0...esp-phy-v0.1.1
[v0.2.0]: https://github.com/esp-rs/esp-hal/compare/esp-phy-v0.1.1...esp-phy-v0.2.0
[v0.3.0]: https://github.com/esp-rs/esp-hal/compare/esp-phy-v0.2.0...esp-phy-v0.3.0
[Unreleased]: https://github.com/esp-rs/esp-hal/compare/esp-phy-v0.3.0...HEAD
