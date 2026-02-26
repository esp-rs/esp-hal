# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- `usleep_until` and `Scheduler::usleep_until` to put the current task to sleep until the specified timestamp. (#4555)
- `SemaphoreHandle::take_with_deadline` to take a semaphore with a deadline. (#4555)
- `QueueHandle::{send_to_front_with_deadline, send_to_back_with_deadline, receive_with_deadline}` for queue operations with a deadline. (#4555)
- `timer::CompatTimer` to simplify OS integration. (#4555)
- `semaphore::CompatSemaphore` to simplify OS integration. (#4559)
- `wait_queue::WaitQueueImplementation` as an alternative way to integrate with an OS. (#4559)
- `ThreadPtr` (#4559)
- The `ipc-implementations` feature, which enables the `CompatQueue`, `CompatSemaphore` and `CompatTimer` types (#4559)
- Chip-specific features required for the `ipc-implementations` feature (#4559)
- Support for ESP32-C5 (#5003)

### Changed

- Renamed `Scheduler` to `SchedulerImplementation` and `scheduler_impl!` to `register_scheduler_implementation!` (#4559)
- `current_task`, `task_create` and `schedule_task_deletion` functions now work with `ThreadPtr` instead of `*mut c_void` (#4559)
- `schedule_task_deletion` now takes an `Option` (#4559)
- `SemaphoreHandle` is now `Send` (#4761)

### Fixed

- `register_queue_implementation` no longer requires `QueuePtr` to be in scope (#4559)
- `task_create` now saturates the priority to the maximum supported by the OS (#5074)

### Removed


## [v0.2.0] - 2025-10-30

### Added

- `queue::CompatQueue` to simplify OS integration. (#4371)

## [v0.1.0] - 2025-10-13

### Added

- Initial release (#3855)

[v0.1.0]: https://github.com/esp-rs/esp-hal/releases/tag/esp-radio-rtos-driver-v0.1.0
[v0.2.0]: https://github.com/esp-rs/esp-hal/compare/esp-radio-rtos-driver-v0.1.0...esp-radio-rtos-driver-v0.2.0
[Unreleased]: https://github.com/esp-rs/esp-hal/compare/esp-radio-rtos-driver-v0.2.0...HEAD
