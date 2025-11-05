# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added


### Changed


### Fixed

- No longer enables the default feature of `esp-hal` (#4433).

### Removed


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
