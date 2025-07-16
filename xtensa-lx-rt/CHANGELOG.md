# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added


### Changed


### Fixed


### Removed


## [v0.20.0] - 2025-07-16

### Changed

- MSRV is now 1.88.0 (#3742)

### Removed

- The `esp32`, `esp32s2` and `esp32s3` features have been removed. (#3598)

## [v0.19.0] - 2025-06-03

### Changed

- Bump Rust edition to 2024, bump MSRV to 1.85. (#3391)

## [0.18.0] - 2025-01-15

### Changed

- Bump MSRV to 1.84 (#2951)

## 0.17.2 - 2024-11-20

### Fixed

- Fixed saving the state of the FPU co-processor. (#2311)

## 0.17.1 - 2024-09-02

### Added

- Better diagnostics when using floats inside an interrupt handler when using the default hard fault handler (#2044)

### Fixed

- Store state of FP coprocessor in stack memory (#2057)

## Initial releases

[0.18.0]: https://github.com/esp-rs/esp-hal/releases/tag/xtensa-lx-rt-v0.18.0
[v0.19.0]: https://github.com/esp-rs/esp-hal/compare/xtensa-lx-rt-v0.18.0...xtensa-lx-rt-v0.19.0
[v0.20.0]: https://github.com/esp-rs/esp-hal/compare/xtensa-lx-rt-v0.19.0...xtensa-lx-rt-v0.20.0
[Unreleased]: https://github.com/esp-rs/esp-hal/compare/xtensa-lx-rt-v0.20.0...HEAD
