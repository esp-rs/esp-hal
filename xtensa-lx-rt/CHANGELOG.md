# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

### Changed

- Bump Rust edition to 2024, bump MSRV to 1.85. (#3391)

### Fixed

### Removed

## 0.18.0 - 2025-01-15

### Changed

- Bump MSRV to 1.84 (#2951)

## 0.17.2 - 2024-11-20

### Fixed

- Fixed saving the state of the FPU co-processor. (#2311)

### Removed

## 0.17.1 - 2024-09-02

### Added

- Better diagnostics when using floats inside an interrupt handler when using the default hard fault handler (#2044)

### Fixed

- Store state of FP coprocessor in stack memory (#2057)

## Initial releases

[Unreleased]: https://github.com/esp-rs/esp-hal/commits/main/xtensa-lx?since=2025-01-15
