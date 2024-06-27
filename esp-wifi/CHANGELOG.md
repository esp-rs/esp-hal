# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

### Fixed

### Changed

- `esp_wifi::initialize` no longer requires running maximum CPU clock, instead check it runs above 80MHz. (#1688)

### Removed

## 0.6.0 - 2024-06-04

### Removed

- Removed embedded-hal v0.2 dependency

## 0.5.1 - 2024-04-22

Patch release to fix docs.rs build

## 0.5.0 - 2024-04-19

### Fixed

- Fix compile error when using smoltcp `DNS_MAX_RESULT_COUNT` values other than 1

## 0.4.0 - 2024-03-12

### Changed

- Users don't need embedded-svc to control wifi anymore. The wifi trait is optionally implemented now. (#429)
- Better network performance by forced yielding of the task when buffers are full / empty. (#430)
- Depend on esp-hal 0.16.1, update other dependencies

## 0.3.0 - 2024-01-29

### Added

- Include coex in list of enabled features for docs.rs (#405)

### Fixed

- Small correction to coex warning message (#404)
- Use a random local port when initializing the wifi stack. (#414)

### Changed

- Update driver blobs (#410)
- Update dependencies to fit `embedded-hal` `1.0`

### Removed

## 0.2.0 - 2024-01-05

Initial release supporting WiFi on ESP32, ESP32-S2, ESP32-S3, ESP32-C3, ESP32-C2, ESP32-C6, supporting BLE on WiFi on ESP32, ESP32-S3, ESP32-C3, ESP32-C2, ESP32-C6, ESP32-H2

## 0.1.0 - 2023-11-27

Initial release supporting WiFi on ESP32, ESP32-S2, ESP32-S3, ESP32-C3, ESP32-C2, ESP32-C6, supporting BLE on WiFi on ESP32, ESP32-S3, ESP32-C3, ESP32-C2, ESP32-C6

[Unreleased]: https://github.com/esp-rs/esp-hal/commits/main/esp-wifi?since=2024-06-05
