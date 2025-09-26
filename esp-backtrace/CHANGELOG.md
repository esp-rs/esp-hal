# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added


### Changed

- `exception-handler` now panics. (#3838)
- Only halt cores in panics when `halt-cores` feature is enabled. (#4010)
- It is no longer possible to select multiple halt method features (`halt-cores`, `custom-halt`, `semihosting`) (#4012)
- RISC-V: If stack-frames are not enabled the panic-handler will now emit a stack dump (#4189)

### Fixed


### Removed

- the `exception-handler` feature got removed (#3887)

## [v0.17.0] - 2025-07-16

### Removed

- Removed support for ESP32-P4 (#3754)

## [v0.16.0] - 2025-06-03

### Added

- The length of the stack trace can now be configured using `ESP_BACKTRACE_CONFIG_BACKTRACE_FRAMES` (#3271)
- `Backtrace` and `BacktraceFrame` types. (#3280)

### Changed

- The `arch::backtrace` function now returns a `Backtrace` struct (#3280)
- Bump Rust edition to 2024, bump MSRV to 1.86. (#3391, #3560)
- Update `defmt` to 1.0 (#3416)

### Fixed

- Stack traces no longer stop at recursive functions (#3270)
- ESP32/S2/S3: Fixed an issue where the backtrace wasn't correctly captured in some cases (#3272)

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

### Changed

- Print a more helpful message in case of a `Cp0Disabled` exception (#2061)

## 0.14.0 - 2024-08-29

### Added

- Add custom-pre-backtrace feature (#1822)

### Changed

- Improve panic message printing (#1823)

## 0.13.0 - 2024-07-16

## 0.12.2 - 2024-07-15

### Changed

- Remove build script check for `nightly-2024-06-12` (#1788)

## 0.12.1 - 2024-06-19

### Fixed

- Fix compilation for nightly after 2024-06-12. (#1681)
- Only prints float registers on targets which have them. (#1690)

[0.15.1]: https://github.com/esp-rs/esp-hal/releases/tag/esp-backtrace-v0.15.1
[v0.16.0]: https://github.com/esp-rs/esp-hal/compare/esp-backtrace-v0.15.1...esp-backtrace-v0.16.0
[v0.17.0]: https://github.com/esp-rs/esp-hal/compare/esp-backtrace-v0.16.0...esp-backtrace-v0.17.0
[Unreleased]: https://github.com/esp-rs/esp-hal/compare/esp-backtrace-v0.17.0...HEAD
