# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## Unreleased

### Added

- It's possible to use partial RF calibration, it's possible to use the None-calibration-schema after deep-sleep (#3383)

### Changed

- The scheduler now runs at interrupt priority 1 on Xtensa chips, too. (#3164)

- `esp-now` and `sniffer` are available via `Interfaces` (#3283)

- Remove the `heapless` dependency (including from the public API) (#3317)

- Bump Rust edition to 2024, bump MSRV to 1.85. (#3391)
- Update `defmt` to 1.0 (#3416)

### Fixed

- Update bt-hci version to fix serialization/deserialization of byte slices (#3340)

- Allow `Configuration::None`, set country early, changed default power-save-mode to None (#3364)

- Enterprise WPA fixed for ESP32-S2 (#3406)
- COEX on ESP32 is now working (#3403)

### Removed

## [0.13.0] - 2025-02-24

### Added

- Added support for using an external scheduler (#3115)

### Changed

- `esp_wifi::init` now takes an `impl Peripheral` for RNG source (#2992)
- `set_power_saving` is now also available when the `coex` feature is activated (#3081)
- Network interfaces and the controller are now more separated (#3027)

### Fixed

- Fixed a problem using BLE on ESP32-C6 when connected via Serial-JTAG (#2981)
- Fix a possible dead-lock when the rx-queue is overrun (#3015)

## 0.12.0 - 2025-01-15

### Changed

- Bump smoltcp to 0.12.0 (#2849)
- `csi_enabled` option converted to feature (#2945)
- Bump MSRV to 1.84 (#2951)

### Fixed

- Fixed triggering a debug-assertion during scan (#2612)
- Fix WPA2-ENTERPRISE functionality (#2896)
- Make sure to de-allocate memory used by timers on removal (#2936)

## 0.11.0 - 2024-11-20

### Added

- Added `serde` support through the `serde` feature (#2346)
- Added `PowerSaveMode` and `set_power_saving` methods on `EspNowManager` & `WifiController` (#2446)
- Added CSI support (#2422)
- Enable setting event handlers for wifi events (#2453)

### Changed

- `esp_wifi::init` no longer requires `EspWifiInitFor`, and now returns `EspWifiController`, see the migration guide for more details (#2301)
- No need to add `rom_functions.x` manually anymore (#2374)
- esp-now: Data is now private in `ReceivedData` - use `data()`(#2396)
- Changed the async APIs to have a `_async` postfix to avoid name collisions (#2446)
- `phy_enable_usb` is enabled by default (#2446)
- Removed `get_` prefixes from functions (#2528)
- Opting out of `esp-alloc` now requires implementing `esp_wifi_deallocate_internal_ram` (#3320)

- Config: Crate prefixes and configuration keys are now separated by `_CONFIG_` (#2848)

### Fixed

- Fixed a possible crash when parsing results from a radius server (#2380)
- Fixed `async fn WifiController::disconnect` hanging forever when awaited if not connected when called (#2392).
- Fixed building esp-wifi without either `ble` or `wifi` enabled (#3336)

### Removed

- Feature `have-strchr` is removed (#2462)
- Features `async`, `embassy-net` have been removed (#2446)
- Features `phy-enable-usb` & `dump-packets` have been turned into configuration options `phy_enable_usb` & `dump_packets` (#2446)
- Features `ps-min-modem` & `ps-max-modem` have been removed in favour of a runtime config (#2446)
- The blocking networking stack is removed (#2488)

## 0.10.1 - 2024-10-10

### Changed

- Bumped esp-wifi-sys to `v0.6.0`

## 0.10.0 - 2024-10-10 - YANKED

### Added

- Added `have-strchr` feature to disable including `strchr` (#2096)
- Adding a way to deinitialize the WiFi stack (#2187)

### Changed

- esp-wifi now allocates memory from the global allocator provided by `esp-alloc` (#2099)
- Renamed the `wifi-logs` feature to `sys-logs` for consistency (#2183)
- Updated drivers to v5.3.1 (#2239)
- Rename `initialize` to `init` (#2295)
- `esp-wifi` no longer enables features on `esp-hal-embassy` (like `esp-hal-embassy/esp32c6`) (#2306)

### Fixed

- Feature `sys-logs` doesn't break the build anymore (#2117)
- Fixed a panic when overflow-checks are enabled (#2164)
- Create mutexes in heap memory, fixes running out of mutexes when connecting and disconnecting to a WPA2-ENTERPRISE ap multiple times (#2202)

### Removed

- Removed the `clocks` parameter from `esp_wifi::initialize` (#1999)
- `cfg_toml` configuration system has been removed in favour of [esp-config](https://docs.rs/esp-config) (#2156)
- Removed the `embedded-svc` traits and feature (#2235)
- Removed the `log` feature from default features (#2253)
- Removed the `enumset` feature (#2297)
- Removed `esp_wifi::current_millis` (#2304)

## 0.9.1 - 2024-09-03

### Added

### Changed

### Fixed

- Builds on stable, again (#2067)

### Removed

## 0.9.0 - 2024-09-03

### Added

- Added support for WPA2-ENTERPRISE (#2004)

### Changed

### Fixed

### Removed

## 0.8.0 - 2024-08-29

### Added

- Implement `embedded_io::{ReadReady, WriteReady}` traits for `WifiStack` (#1882)
- Implement `queue_msg_waiting` on the os_adapter (#1925)
- Added API for promiscuous mode (#1935)
- Implement `bt_hci::transport::Transport` traits for BLE (#1933)

### Changed

- Changed `init` to accept timers of multiple types (#1957)

### Fixed

- Increased NPL event queue size to prevent overflow (#1891)

## 0.7.1 - 2024-07-17

### Changed

- Check no password is set when using `AuthMethod::None`(#1806)

### Fixed

- Downgrade `embedded-svc` to 0.27.1 (#1820)

## 0.7.0 - 2024-07-15

### Added

- Add support for `Protocol::P802D11BGNAX` (#1742)

### Fixed

- Fixed `set_mode` functionality (#1742)

### Changed

- `esp_wifi::initialize` no longer requires running maximum CPU clock, instead check it runs above 80MHz. (#1688)
- Rename `set_mode` to `set_protocol`, also available in esp-now API (#1742)
- `esp_wifi::initialize` now takes a `PeriodicTimer<ErasedTimer>` (#1753)

## 0.6.0 - 2024-06-04

### Removed

- Removed embedded-hal v0.2 dependency

## 0.5.1 - 2024-04-22

Patch release to fix docs.rs build

## 0.5.0 - 2024-04-19

### Fixed

- Fix compile error when using smoltcp `DNS_MAX_RESULT_COUNT` values other than 1

## 0.4.0 - 2024-03-12

### Changed

- Users don't need embedded-svc to control wifi anymore. The wifi trait is optionally implemented now. (#429)
- Better network performance by forced yielding of the task when buffers are full / empty. (#430)
- Depend on esp-hal 0.16.1, update other dependencies

## 0.3.0 - 2024-01-29

### Added

- Include coex in list of enabled features for docs.rs (#405)

### Fixed

- Small correction to coex warning message (#404)
- Use a random local port when initializing the wifi stack. (#414)

### Changed

- Update driver blobs (#410)
- Update dependencies to fit `embedded-hal` `1.0`

### Removed

## 0.2.0 - 2024-01-05

Initial release supporting WiFi on ESP32, ESP32-S2, ESP32-S3, ESP32-C3, ESP32-C2, ESP32-C6, supporting BLE on WiFi on ESP32, ESP32-S3, ESP32-C3, ESP32-C2, ESP32-C6, ESP32-H2

## 0.1.0 - 2023-11-27

Initial release supporting WiFi on ESP32, ESP32-S2, ESP32-S3, ESP32-C3, ESP32-C2, ESP32-C6, supporting BLE on WiFi on ESP32, ESP32-S3, ESP32-C3, ESP32-C2, ESP32-C6

[0.13.0]: https://github.com/esp-rs/esp-hal/releases/tag/esp-wifi-v0.13.0
