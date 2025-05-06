# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## Unreleased

### Added

- Re-added the `multiple-integrated` timer queue flavour (#3166)

### Changed

- Bump Rust edition to 2024, bump MSRV to 1.85. (#3391)
- Update `defmt` to 1.0 (#3416)
- The `IntoAnyTimer` trait has been removed (#3444)
- The `TimerCollection` trait has been sealed and renamed to `TimeBase`. Former `IntoAnyTimer` functionality has been merged into `TimeBase`. (#3444)
- `esp_hal_embassy::init` will panic if called multiple times (#3444)
- The `log` feature has been replaced by `log-04`. (#3425)

### Fixed

- Fixed a panic on very long wakeup times (#3433)

### Removed

## [0.7.0] - 2025-02-24

### Fixed

- Fixed an issue where the `ESP_HAL_EMBASSY_CONFIG_LOW_POWER_WAIT` option was not possible to disable (#2975)

### Removed

- The `multiple-integrated` timer queue flavour has been temporarily removed (#3159)

## 0.6.0 - 2025-01-15

### Added

- Added `ESP_HAL_EMBASSY_CONFIG_TIMER_QUEUE` (#2701)
- Added `ESP_HAL_EMBASSY_CONFIG_GENERIC_QUEUE_SIZE` instead of using `embassy-time/generic-queue-*` (#2701)

### Changed

- Bump MSRV to 1.83 (#2615)
- Updated embassy-time to v0.4 (#2701)
- Config: Crate prefixes and configuration keys are now separated by `_CONFIG_` (#2848)
- Bump MSRV to 1.84 (#2951)

### Fixed

- Fixed an issue with using thread-mode executors on both cores (#2924)

### Removed

- The `integrated-timers` option has been replaced by configuration options. (#2701)

## 0.5.0 - 2024-11-20

### Added

- `ESP_HAL_EMBASSY_LOW_POWER_WAIT` configuration option. (#2329)

### Changed

- Reduce memory footprint by 4 bytes on multi-core MCUs.
- The time driver no longer uses cross-core critical sections. (#2559)

### Fixed

- Alarm interrupts are now handled on the core that allocated them. (For executors created on the second core after calling `esp_hal_embassy::init`) (#2451)

### Removed

## 0.4.0 - 2024-10-10

### Changed

- MSRV bump to 1.79 (#2156)

### Removed

- Removed the `clocks` parameter from `esp_hal_embassy::init`. (#1999)

## 0.3.0 - 2024-08-29

### Added

- This package now re-exports the `esp_hal_procmacros::main` macro (#1828)

### Changed

- Updated to latest release (`0.6.0`) for `embassy-executor` (#1942)
- Changed `init` to accept timers of multiple types (#1957, #2012)

### Fixed

- Fixed a bug where the timeout was huge whenever the timestamp at the time of scheduling was already in the past (#1875)
- Fixed interrupt executors looping endlessly when `integrated-timers` is used. (#1936)

## 0.2.0 - 2024-07-15

### Changed

- Removed the TIMG and SYSTIMER time drivers, replaced by a generic time driver taking `OneShotTimer<ErasedTimer>` (#1753)

## 0.1.0 - 2024-06-04

[0.7.0]: https://github.com/esp-rs/esp-hal/releases/tag/esp-hal-embassy-v0.7.0
