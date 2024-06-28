# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- Add the `esp32c6-lp-hal` package (#714)
- Add GPIO (output) and delay functionality to `esp32c6-lp-hal` (#715)
- Add GPIO input support and implement additional `embedded-hal` output traits for the C6's LP core [#720]
- Add the `ulp-riscv-hal` package (#840)
- Add LP_UART basic driver (#1113)
- Added basic `LP-I2C` driver for C6 (#1185)
- Add remaining GPIO pins for ESP32-S2/S3 (#1695)
- Add `wake_hp_core` for ESP32-C6 (#1723)

### Changed

- Renamed to `esp-ulp-riscv-hal` (#916)
- Remove 2nd level generics from GPIO pin (#1526)

### Fixed

### Removed

[Unreleased]: https://github.com/esp-rs/esp-hal/commits/main/esp-lp-hal
