# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## Unreleased

### Added

### Changed

- Bump MSRV to 1.84 (#2951)
- Fix gpio `input_state` and `output_state` for the ESP32-S3

### Fixed

### Removed

- Remove embedded-hal 0.2.x impls and dependency from esp-lp-hal package (#2609)

## 0.1.0 - 2024-07-15

### Added

- Add the `esp32c6-lp-hal` package (#714)
- Add GPIO (output) and delay functionality to `esp32c6-lp-hal` (#715)
- Add GPIO input support and implement additional `embedded-hal` output traits for the C6's LP core [#720]
- Add the `ulp-riscv-hal` package (#840)
- Add LP_UART basic driver (#1113)
- Added basic `LP-I2C` driver for C6 (#1185)
- Add remaining GPIO pins for ESP32-S2/S3 (#1695)
- Add `wake_hp_core` for ESP32-C6 (#1723)
- Implement `embedded-hal@1.x.x` traits by default instead of `embedded-hal@0.2.x` (#1754)
- Implement `embedded-hal-nb` and `embedded-io` traits for UART driver (#1754)
- Add `Delay.delay_millis` function (#1789)
- Make some UART functions public, allowing it to be used without `embedded-hal`/`embedded-io` traits (#1789)

### Changed

- Renamed to `esp-ulp-riscv-hal` (#916)
- Remove 2nd level generics from GPIO pin (#1526)
- GPIO Input/Output types have been converted to unit structs (#1754)
