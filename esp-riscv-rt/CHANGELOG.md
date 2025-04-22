# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## Unreleased

### Added

### Changed

- Bump Rust edition to 2024, bump MSRV to 1.85. (#3391)

### Fixed

### Removed

## [0.10.0] - 2025-02-24

### Changed

- Bump MSRV to 1.84 (#2951)
- Removed the `fix-sp` feature (check is always enabled) (#3001)

## 0.9.1 - 2024-11-20

## Fixed

- Fix interrupt stack alignment (#2425)

## 0.9.0 - 2024-07-15

### Added

- `rtc-ram` feature used by `esp-hal` to control rtc ram initialization (#1677)

### Removed

- All existing features controlling ram initialization. Most (`init-data`, `init-rw-text`,
  `init-rtc-fast-data`, and `init-rtc-fast-text`) were only used for the (already removed) direct
  boot support. `zero-bss` is now enabled unconditionally. `zero-rtc-fast-bss` was merged into the
  new `rtc-ram` feature. (#1677)

## 0.8.0 - 2024-04-18

### Fixed

- Ensure we don't strongly define cpu int handlers (#1324)
- Discard interrupt symbols from LTO so that LTO doesn't end up rebinding them (#1327)

### Changed

- Remove the `direct-vectoring` & `interrupt-preemption` features and enable them by default (#1310)

## 0.7.0 - 2024-03-08

### Changed

- `start_rust` calls `hal_main` instead of calling user's `main` directly (#1135)

## 0.6.1 - 2024-01-19

### Changed

- Updated to latest version of `riscv` and `riscv-rt-macros` dependencies

## 0.6.0 - 2023-12-12

### Fixed

- Fix overwriting `rtc-uninit-data` when there is no rtc-bss data (#952)
- Fix RISC-V stack allocation (#988)
- ESP32-C6/ESP32-H2: Add `fix-sp` feature to support `flip-link` in `esp-hal-common` (#1008)

### Removed

## 0.5.0 - 2023-09-05

### Changed

- Use all remaining memory for the stack (#716)

### Fixed

- Fix RISCV stack-start (#721)

## 0.4.0 - 2023-08-10

## 0.3.0 - 2023-03-20

## 0.2.0 - 2023-03-14

## 0.1.0 - 2023-01-26

[0.10.0]: https://github.com/esp-rs/esp-hal/releases/tag/esp-riscv-rt-v0.10.0
