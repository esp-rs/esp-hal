# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

### Changed

### Fixed

### Removed

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

[Unreleased]: https://github.com/esp-rs/esp-hal/commits/main/esp-println?since=2024-10-10
