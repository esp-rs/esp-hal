# Changelog

All notable changes to this project will be documented in this file.

Please note that only changes to the `esp-hal-common` package are tracked in this CHANGELOG.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.9.0] - 2023-05-02

### Added

- Add bare-bones PSRAM support for ESP32-S2 (#493)
- Add `DEBUG_ASSIST` functionality (#484)
- Add RSA peripheral support (#467)
- Add PeripheralClockControl argument to `timg`, `wdt`, `sha`, `usb-serial-jtag` and `uart` constructors (#463)
- Added API to raise and reset software interrupts (#426)
- Implement `embedded_hal_nb::serial::*` traits for `UsbSerialJtag` (#498)

### Fixed

- Fix `get_wakeup_cause` comparison error (#472)
- Use 192 as mclk_multiple for 24-bit I2S (#471)
- Fix `CpuControl::start_app_core` signature (#466)
- Move `rwtext` after other RAM data sections (#464)
- ESP32-C3: Disable `usb_pad_enable` when setting GPIO18/19 to input/output (#461)
- Fix 802.15.4 clock enabling (ESP32-C6) (#458)

### Changed

- Update `embedded-hal-async` and `embassy-*` dependencies (#488)
- Update to `embedded-hal@1.0.0-alpha.10` and `embedded-hal-nb@1.0.0-alpha.2` (#487)
- Let users configure the LEDC output pin as open-drain (#474)
- Use bitflags to decode wakeup cause (#473)
- Minor linker script additions (#470)
- Minor documentation improvements (#460)

### Removed

- Remove unnecessary generic from `UsbSerialJtag` driver (#492)
- Remove `#[doc(inline)]` from esp-hal-common re-exports (#490)

## [0.8.0] - 2023-03-27

## [0.7.1] - 2023-02-22

## [0.7.0] - 2023-02-21

## [0.5.0] - 2023-01-26

## [0.4.0] - 2022-12-12

## [0.3.0] - 2022-11-17

## [0.2.0] - 2022-09-13

## [0.1.0] - 2022-08-05

[unreleased]: https://github.com/esp-rs/esp-hal/compare/v0.9.0...HEAD
[0.9.0]: https://github.com/esp-rs/esp-hal/compare/v0.8.0...v0.9.0
[0.8.0]: https://github.com/esp-rs/esp-hal/compare/v0.7.1...v0.8.0
[0.7.1]: https://github.com/esp-rs/esp-hal/compare/v0.7.0...v0.7.1
[0.7.0]: https://github.com/esp-rs/esp-hal/compare/v0.5.0...v0.7.0
[0.5.0]: https://github.com/esp-rs/esp-hal/compare/v0.4.0...v0.5.0
[0.4.0]: https://github.com/esp-rs/esp-hal/compare/v0.3.0...v0.4.0
[0.3.0]: https://github.com/esp-rs/esp-hal/compare/v0.2.0...v0.3.0
[0.2.0]: https://github.com/esp-rs/esp-hal/compare/v0.1.0...v0.2.0
[0.1.0]: https://github.com/esp-rs/esp-hal/releases/tag/v0.1.0
