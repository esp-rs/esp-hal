# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- Provide implementation for the `_getreent` syscall when the `alloc` feature is enabled (#4473)
- Provide implementation for the `_malloc_r` and `_free_r` syscalls when the `alloc` feature is enabled (#4484)

### Changed

- `esp_rtos::start` now takes `SoftwareInterrupt<'static, 0>` for all CPUs (#4459)
- `esp_rtos::start_second_core` no longer takes `SoftwareInterrupt<'static, 0>` (#4459)
- `esp-alloc` dependency no longer enables default features (#4721)

### Fixed

- No longer enables the default feature of `esp-hal` (#4433)
- Time driver should no longer generate an interrupt-storm when the wakeup time does not fit 52 bits (#4444)
- Fixed an issue causing incorrectly re-queueing timers (#4444)
- Fixed an issue on ESP32 that prevented completing some interrupt handlers (#4459)
- Fixed a possible deadlock on multi-core chips (#4478)
- Fixed a memory leak of 48 bytes when deleting esp-radio timers (#4541)
- Fixed a rare crash on Xtensa MCUs (#4580, #4591)

### Removed

- `Semaphore` has been removed (#4559)

## [v0.2.0] - 2025-10-30

### Added

- Interrupt-safe semaphore operations (#4396)

### Changed

- `start_second_core` will now panic if the scheduler does not start on the second core. (#4353)

### Fixed

- Fixed a bug causing timers to stop working in certain cases. (#4393)
- Place more RTOS code in IRAM to improve performance. (#4394)

## [v0.1.1] - 2025-10-14

### Fixed

- Fixed a bug causing a crash when deleting a task (#4338)

## [v0.1.0] - 2025-10-13

### Added

- Initial release (#3855)
- The `esp-hal-embassy` crate has been merged into `esp-rtos`. (#4172)

[v0.1.0]: https://github.com/esp-rs/esp-hal/releases/tag/esp-rtos-v0.1.0
[v0.1.1]: https://github.com/esp-rs/esp-hal/compare/esp-rtos-v0.1.0...esp-rtos-v0.1.1
[v0.2.0]: https://github.com/esp-rs/esp-hal/compare/esp-rtos-v0.1.1...esp-rtos-v0.2.0
[Unreleased]: https://github.com/esp-rs/esp-hal/compare/esp-rtos-v0.2.0...HEAD
