# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## Unreleased

### Added

### Changed

- MSRV bump to 1.79 (#2156)

### Fixed

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
