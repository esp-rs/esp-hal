# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added


### Changed


### Fixed


### Removed


## [v0.11.0] - 2025-06-03

### Changed

- Bump Rust edition to 2024, bump MSRV to 1.85. (#3391)

## [0.10.0] - 2025-01-15

### Fixed

- Fixed `interrupt:free` incorrectly providing `CriticalSection` (#2537)

### Changed

- The `singleton` macro has been updated to match the cortex-m counterpart (#2537)
- Bump MSRV to 1.84 (#2951)

### Removed

- The `spin` feature and `mutex` module has been removed. (#2537)
- The `InterruptNumber` trait has been removed. (#2537)

## 0.9.0 - 2024-02-21

## 0.8.0 - 2023-02-23

## 0.7.0 - 2022-04-20

## 0.6.0 - 2022-01-16

## 0.5.0 - 2022-01-15

## 0.4.0 - 2021-08-11

## 0.3.0 - 2020-09-19

[0.10.0]: https://github.com/esp-rs/esp-hal/releases/tag/xtensa-lx-v0.10.0
[v0.11.0]: https://github.com/esp-rs/esp-hal/compare/xtensa-lx-v0.10.0...xtensa-lx-v0.11.0
[Unreleased]: https://github.com/esp-rs/esp-hal/compare/xtensa-lx-v0.11.0...HEAD
