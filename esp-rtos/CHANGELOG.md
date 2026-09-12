# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added


### Changed


### Fixed


### Removed


## [v0.4.0] - 2026-08-25

### Added

- Initial support for ESP32-P4 (#5523, #5535)
- Add ESP32-S31 support (#5922)
- `start_on_second_core_only` on multi-core chips, for applications where it is simler to keep the first core bare metal. (#6044)
- Configuration for the esp-radio timer thread priority. (#5908)
- Sleep: enabled auto-lightsleep when esp-rtos runs on multiple cores (#5825)
- `rearm_alarm` to manually adjust the time base after waking up from light sleep (#5778)
- ESP32-C6, ESP32-H2: `auto_light_sleep`, an idle-hook factory that puts the system into light sleep when idle long enough, restoring timekeeping and re-arming the scheduler timer on wake. (#5756)
- `ESP_RTOS_CONFIG_LIGHT_SLEEP_MIN_US` config option setting the minimum residency below which light sleep is skipped. (#5756)

### Changed

- `start` functions and `InterruptExecutor::new` now take `FROM_CPU_INTRn` singletons instead of `SoftwareInterrupt` structs. (#6142)
- `DeepSleep::deep_sleep` no longer takes a list of wakeup sources. (#6060)
- The automatic light sleep hook owns one wakeup source, the timer deadline, and leaves every (#6060)
- `auto_light_sleep` has been removed. Instead, the `sleep` module now has a `configure` function that returns `Sleep`. `Sleep` holds the idle hook function, as well as a deep sleep handle. (#5839)
- updated defmt to 1.1 (#5752)

### Fixed

- the main task can no longer be deleted (on either core) (#6032)
- deleting a task no longer allows reallocating its stack memory while the stack may be in use. (#6032)
- a task deleting itself no longer blocks going to sleep (#6032)
- deleting the other core's current task now triggers switching away from that task (#6032)
- main task stack sizes are correctly tracked (#6027)
- the idle tasks now perform stack overflow checking (#6027)
- fixed a potential crash on RISC-V devices (#5641)

## [v0.3.0] - 2026-04-16

### Added

- Provide implementation for the `_getreent` syscall when the `alloc` feature is enabled (#4473)
- Provide implementation for the `_malloc_r` and `_free_r` syscalls when the `alloc` feature is enabled (#4484)
- Support for ESP32-C5 (#4884)
- `ESP_RTOS_CONFIG_STACK_POINTER_RANGE_CHECK` to enable another way of stack-overflow detection. Enabled by default (#5227)
- Support for ESP32-C61 (#5240)

### Changed

- `esp_rtos::start` now takes `SoftwareInterrupt<'static, 0>` for all CPUs (#4459)
- `esp_rtos::start` can no longer be called from an interrupt handler (#4766)
- `esp_rtos::start_second_core` no longer takes `SoftwareInterrupt<'static, 0>` (#4459)
- `esp-alloc` dependency no longer enables default features (#4721)
- Place the pointer to the current thread in the thread pointer registers (#4766)

### Fixed

- No longer enables the default feature of `esp-hal` (#4433)
- Time driver should no longer generate an interrupt-storm when the wakeup time does not fit 52 bits (#4444)
- Fixed an issue causing incorrectly re-queueing timers (#4444)
- Fixed an issue on ESP32 that prevented completing some interrupt handlers (#4459)
- Fixed a possible deadlock on multi-core chips (#4478)
- Fixed a memory leak of 48 bytes when deleting esp-radio timers (#4541)
- Fixed a rare crash on Xtensa MCUs (#4580, #4591)
- RISC-V: the idle hook no longer prevents a debugger from reading memory (#4782)
- Fixed a bug causing core 1 to not be able to wake up core 0 when started using `CpuControl::start_app_core` (#4890)

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
[v0.3.0]: https://github.com/esp-rs/esp-hal/compare/esp-rtos-v0.2.0...esp-rtos-v0.3.0
[v0.4.0]: https://github.com/esp-rs/esp-hal/compare/esp-rtos-v0.3.0...esp-rtos-v0.4.0
[Unreleased]: https://github.com/esp-rs/esp-hal/compare/esp-rtos-v0.4.0...HEAD
