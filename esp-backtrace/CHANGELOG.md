# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## Unreleased

### Added

### Changed

### Fixed

- Stack traces no longer stop at recursive functions (#3270)
- ESP32/S2/S3: Fixed an issue where the backtrace wasn't correctly captured in some cases (#3272)

### Removed

## [0.15.1] - 2025-02-24

### Fixed

- `PanicInfo` is now printed natively by `defmt` (#3112)

## 0.15.0 - 2025-01-15

### Changed

- Bump MSRV to 1.84 (#2951)

## 0.14.2 - 2024-10-10

### Fixed

- Fix build when not using `panic-handler` (#2257)

## 0.14.1 - 2024-09-06

### Added

### Changed

- Print a more helpful message in case of a `Cp0Disabled` exception (#2061)

### Fixed

### Removed

## 0.14.0 - 2024-08-29

### Added

- Add custom-pre-backtrace feature (#1822)

### Changed

- Improve panic message printing (#1823)

## 0.13.0 - 2024-07-16

No changes - published to avoid conflicts with `esp-println`

## 0.12.2 - 2024-07-15

### Changed

- Remove build script check for `nightly-2024-06-12` (#1788)

## 0.12.1 - 2024-06-19

### Fixed

- Fix compilation for nightly after 2024-06-12. (#1681)
- Only prints float registers on targets which have them. (#1690)

[0.15.1]: https://github.com/esp-rs/esp-hal/releases/tag/esp-backtrace-v0.15.1
