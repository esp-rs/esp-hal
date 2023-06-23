# Changelog

All notable changes to this project will be documented in this file.

Please note that only changes to the `esp-hal-common` package are tracked in this CHANGELOG.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- Add bare-bones PSRAM support for ESP32 (#506)
- Add initial support for the ESP32-H2 (#513, #526, #527, #528, #530, #538, #544, #548, #551, #556, #560, #566, #549, #564, #569, #576, #577, #589, #591, #597)
- Add bare-bones PSRAM support for ESP32-S3 (#517)
- Add async support to the I2C driver (#519)
- Implement Copy and Eq for EspTwaiError (#540)
- Add LEDC hardware fade support (#475)
- Added support for multicore async GPIO (#542)
- Add a fn to poll DMA transfers (#559)
- Simplify the `Delay` driver, derive `Clone` and `Copy` (#568)
- Fix Async GPIO not disabling interupts on chips with multiple banks (#572)
- Add unified field-based efuse access (#567)
- Move `esp-riscv-rt` into esp-hal (#578)
- Add CRC functions from ESP ROM (#587)
- Add a `debug` feature to enable the PACs' `impl-register-debug` feature (#596)
- Add initial support for `I2S` in ESP32-H2 (#597)
- Fix rom::crc docs
- Add octal PSRAM support for ESP32-S3 (#610)

### Changed

- Move core interrupt handling from Flash to RAM for RISC-V chips (ESP32-H2, ESP32-C2, ESP32-C3, ESP32-C6) (#541)
- Change LED pin to GPIO2 in ESP32 blinky example (#581)
- Update ESP32-H2 and ESP32-C6 clocks and remove `i2c_clock` for all chips but ESP32 (#592)
- Use both timers in `TIMG0` for embassy time driver when able (#609)

### Fixed

- DMA is supported for SPI3 on ESP32-S3 (#507)
- `change_bus_frequency` is now available on `SpiDma` (#529)
- Fixed a bug where a GPIO interrupt could erroneously fire again causing the next `await` on that pin to instantly return `Poll::Ok` (#537)
- Set `vecbase` on core 1 (ESP32, ESP32-S3) (#536)
- ESP32-S3: Move PSRAM related function to RAM (#546)
- ADC driver will now apply attenuation values to the correct ADC's channels. (#554)
- Sometimes half-duplex non-DMA SPI reads were reading garbage in non-release mode (#552)
- ESP32-C3: Fix GPIO5 ADC channel id (#562)
- ESP32-H2: Fix direct-boot feature (#570)
- ESP32-C6: Support FOSC CLK calibration for ECO1+ chip revisions (#593)
- Fixed CI by pinning the log crate to 0.4.18 (#600)
- ESP32-S3: Fix calculation of PSRAM start address
- Fixed wrong variable access (FOSC CLK calibration for ESP32-C6 #593)
- Fixed [trap location in ram](https://github.com/esp-rs/esp-hal/pull/605#issuecomment-1604039683) (#605)
### Changed

- Improve examples documentation (#533)
- esp32h2-hal: added README (#585)

### Breaking

- Significantly simplified user-facing GPIO pin types. (#553)
- No longer re-export the `soc` moduleand the contents of the `interrupt` module at the package level (#607)

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
