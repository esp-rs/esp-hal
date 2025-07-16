# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added


### Changed


### Fixed


### Removed


## [v0.8.0] - 2025-07-16

### Added

- Added `Config::generate_metadata` to generate code for firmware crates. (#3604)

### Removed

- Removed the firmware-side component of the crate. (#3604)

## [v0.7.0] - 2025-06-03

### Added

- Add ability to define memory regions, have DRAM defined there (#3300)
- Provide macros to get the start/end of a memory region, make it possible to use the macros in a no-std project (#3300)

### Changed

- Bump Rust edition to 2024, bump MSRV to 1.86. (#3391, #3560)

## [0.6.0] - 2025-02-24

### Added

- Introduced the `adc1` and `adc2` symbols (#3082)

### Removed

- Removed the `adc` symbol (#3082)

## 0.5.0 - 2025-01-15

### Added

- Introduced the `wifi6` symbol (#2612)
- Introduced the `gpio_bank_1` symbol (#2625)

### Changed

- Bump MSRV to 1.84 (#2951)

## 0.4.0 - 2024-10-10

## 0.3.0 - 2024-08-29

## 0.2.0 - 2024-07-15

## 0.1.1 - 2024-06-04

## 0.1.0 - 2024-04-17

### Added

- Initial release (#2518)

[0.6.0]: https://github.com/esp-rs/esp-hal/releases/tag/esp-metadata-v0.6.0
[v0.7.0]: https://github.com/esp-rs/esp-hal/compare/esp-metadata-v0.6.0...esp-metadata-v0.7.0
[v0.8.0]: https://github.com/esp-rs/esp-hal/compare/esp-metadata-v0.7.0...esp-metadata-v0.8.0
[Unreleased]: https://github.com/esp-rs/esp-hal/compare/esp-metadata-v0.8.0...HEAD
