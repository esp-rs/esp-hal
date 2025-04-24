# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## Unreleased

### Added

- Added new `_esp_println_timestamp()` hook, gated by the `timestamp` feature to provide timestamp for logging (#3194)
- Added metadata for espflash to help setting log format (#3276)

### Changed

- Bump Rust edition to 2024, bump MSRV to 1.85. (#3391)
- Update `defmt` to 1.0 (#3416)

### Fixed

- Manually setting a log level now correctly ignores `ESP_LOG`. (#3240)
- Fixed logging rules being order-dependent. (#3240)

### Removed

## [0.13.1] - 2025-02-24

### Fixed

- Fix build failure when `critical-section` feature is disabled

## 0.13.0 - 2025-01-15

### Changed

- Bump MSRV to 1.84 (#2951)

## 0.12.0 - 2024-10-10

### Changed

- Replace environment variables `ESP_LOGLEVEL` and `ESP_LOGFILTER` with just one environment variable: `ESP_LOG` (#2291)

## 0.11.0 - 2024-08-29

### Added

- Made `esp_println::Printer::write_bytes` public (#1812)

## 0.10.0 - 2024-07-15

### Added

- Add `auto` feature to auto-detect Serial-JTAG/UART communication (#1658)

### Changed

- `auto` is the default communication method (#1658)
- Add the `links` field to Cargo.toml so that only one version of the package can be included (#1761)

## 0.9.1 - 2024-03-11

### Changed

- Un-pinned the defmt package's version number

## 0.9.0 - 2024-02-07

### Added

- Add support for ESP32-P4

### Removed

- Remove ESP 8266 support

## 0.8.0 - 2023-12-21

### Removed

- Remove RTT and defmt-raw support

[0.13.1]: https://github.com/esp-rs/esp-hal/releases/tag/esp-println-v0.13.1
