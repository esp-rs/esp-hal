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
- `ThreadPtr` (#?)
- The `ipc-implementations` feature, which enables the `CompatQueue` type (#?)

### Changed

- Renamed `Scheduler` to `SchedulerImplementation` and `scheduler_impl!` to `register_scheduler_implementation!` (#?)
- `current_task`, `task_create` and `schedule_task_deletion` functions now work with `ThreadPtr` instead of `*mut c_void` (#?)
- `schedule_task_deletion` now takes an `Option` (#?)

### Fixed

- `register_queue_implementation` no longer requires `QueuePtr` to be in scope (#?)

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
