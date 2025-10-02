# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- `AccessPointInfo::country` to access the Country Code from the Wi-Fi scan results (#3837)
- `unstable` feature to opt into `ble`, `esp-now`, `csi`, `sniffer`, `esp-ieee802154` and `smoltcp` APIs (#3865)
- Added unstable `wifi-eap` feature (#3924)
- Optional `max` field in `ScanConfig` to allow limiting the number of returned results (#3963)
- `set_phy_calibration_data` and `phy_calibration_data` (#4001)
- common traits for `Protocol`, `Country`,  (#4017)
- `BuilderLite` pattern to `AccessPointConfig`, `ClientConfig`, and `EapClientConfig` (#4017, #4115)
- lifetime to `Sniffer` (#4017)
- `dtim_period` parameter for `PowerSaveMode` (#4040)
- `WifiConfig`, `CountryInfo` and `OperatingClass` (#4121)
- Configuration options for `BleController` (#4223)

### Changed

- `esp_wifi::init` no longer needs an `RNG` driver. (#3829)
- The `builtin-scheduler` feature has been removed. Your project will have to specify a task scheduler. (#3855)
- `esp-wifi` has been renamed to `esp-radio`. (#3858)
- Removed `EspWifi` prefix from structs in the package codebase. (#3869)
- Rename `esp-wifi` to `esp-radio`. (#3858)
- `esp-ieee802154` package has been folded into `esp-radio`, it's now alloc. (#3861, #3890)
- `ble`, `esp-now`, `csi`, `sniffer`, `esp-ieee802154` and `smoltcp` features and APIs marked as unstable (#3865)
- Update bt-hci version to add additional HCI commands (#3920)
- A number of enums/structs have been marked as `#[non_exhaustive]` (#3981, #4017)
  - `AuthMethod`, `Protocol`, `AccessPointInfo`, `AccessPointConfiguration`, `ClientConfiguration`, `Capability`, `Configuration`, `WifiEvent`, `InternalWifiError`, `ScanTypeConfig`, `WifiState`, and `WifiMode`
- The `Configuration`, `ClientConfiguration`, `AccessPointConfiguration`, and `EapClientConfiguration` enums have been renamed to `Config`, `ClientConfig`, `AccessPointConfig`, and `EapClientConfig` (#3994)
  - Error types implements `core::error:Error`
- Use `esp-phy` internally for PHY initialization (#3892)
- `ap_state()` and `sta_state()` marked as stable (#4017)
- `wifi_state()` marked as unstable (#4017)
- `ap_mac` and `sta_mac` returns `[u8; 6]` instead of taking an `[u8; 6]` argument (#4017)
- `RxControlInfo` hidden behind `esp-now` feature (#4017)
- `set_configuration()` to `set_config() (#4017)
- `WifiState` split into `WifiStaState` and `WifiApState` (#4046)
- `Mixed` has been renamed to `ApSta` in `Config` and `Capability` (#4040)
- The memory allocation functions expected by `esp_radio` have been renamed and extended (#3890, #4043)
- Updated radio related drivers to ESP-IDF 5.5.1 (#4113)
- Event handlers are now passed the event by reference (#4113)
- Some build-time configuration options have been replaced by runtime options in `WifiConfig` (#4121)
- Update to bt-hci version with flash usage improvements (#4146, #4165)
- `scan_mode`, `(ap_)beacon_timeout`, `listen_interval` and `failure_retry_cnt` config options have been replaced by runtime options in `AccessPointConfig`, `ClientConfig` and `EapClientConfig` (#4224)
- The `ieee802154_rx_queue_size` config option has been replaced by a runtime option in `esp_radio::ieee802154::Config` (#4224)
- The default value of `wifi_max_burst_size` has been changed to 3 (#4231)
- Set `ble_ll_sync_cnt` to 0 on C6, C2 and H2 as in esp-idf Kconfig default (#4241)

### Fixed

- Fixed a BLE panic caused by unimplemented functions (#3762)
- Fixed the BLE stack crashing in certain cases (#3854)
- `ADC2` now cannot be used simultaneously with `radio` on ESP32 (#3876)
- Fixed names of some Wi-Fi events: ApStaConnected, ApStaDisconnected, ApProbeReqReceived (#4065)
- BLE on ESP32-C2 with 26MHz xtal (#4062)

### Removed

- `scan_with_config_sync_max`, `scan_with_config_sync_max`, `scan_n`, and `scan_n_async` functions (#3963)
- `EnumSetType` from `Protocol`, `Country` enums (#4017)
- `AtomicWifiState` and `WifiDeviceMode` are not available anymore (#4029)
- `wifi_state()` and `WifiState` are not available anymore (#4046)
- `config` module (#4040)
- Remove `as_client_conf_ref`, `as_ap_conf_ref`, `as_ap_conf_mut`, `as_client_conf_mut` and `as_mixed_conf_mut` from `Config` (#4060)

## [v0.15.0] - 2025-07-16

### Changed

- MSRV is now 1.88.0 (#3742)
- Removed `esp_wifi::deinit_unchecked` and `esp_wifi::EspWifiController::deinit` - you can just drop `EspWifiController` instead (#3553)
- `defmt` and `log-04` can no longer be selected at the same time (#3675)
- `esp_wifi::init` no longer requires the `RADIO_CLK` peripheral (#3687)

## [v0.14.1] - 2025-06-05

### Added

- It's now possible to obtain the RSSI of the currently connected AP, by using `WifiController::rssi(&self)` (#3593)

### Fixed

- Fix a compilation error for ESP32 + coex + ble + defmt (#3596)

## [v0.14.0] - 2025-06-03

### Added

- It's possible to use partial RF calibration, it's possible to use the None-calibration-schema after deep-sleep (#3383)

### Changed

- The scheduler now runs at interrupt priority 1 on Xtensa chips, too. (#3164)
- `esp-now` and `sniffer` are available via `Interfaces` (#3283)
- Remove the `heapless` dependency (including from the public API) (#3317)
- Bump Rust edition to 2024, bump MSRV to 1.86. (#3391, #3560)
- Update `defmt` to 1.0 (#3416)
- The `log` feature has been replaced by `log-04`. (#3425)
- Removed `esp_wifi::deinit_unchecked` and `esp_wifi::EspWifiController::deinit` - you can just drop `EspWifiController` instead (#3553)

### Fixed

- Update bt-hci version to fix serialization/deserialization of byte slices (#3340)
- Allow `Configuration::None`, set country early, changed default power-save-mode to None (#3364)
- Enterprise WPA fixed for ESP32-S2 (#3406)
- COEX on ESP32 is now working (#3403)
- Correctly de-init wifi if the WifiController is dropped (#3550)

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
- esp-now: Data is now private in `ReceivedData` - use `data()` (#2396)
- Changed the async APIs to have a `_async` postfix to avoid name collisions (#2446)
- `phy_enable_usb` is enabled by default (#2446)
- Removed `get_` prefixes from functions (#2528)
- Opting out of `esp-alloc` now requires implementing `esp_wifi_deallocate_internal_ram` (#3320)
- Config: Crate prefixes and configuration keys are now separated by `_CONFIG_` (#2848)

### Fixed

- Fixed a possible crash when parsing results from a radius server (#2380)
- Fixed `async fn WifiController::disconnect` hanging forever when awaited if not connected when called (#2392)
- Fixed building esp-wifi without either `ble` or `wifi` enabled (#3336)

### Removed

- Feature `have-strchr` is removed (#2462)
- Features `async`, `embassy-net` have been removed (#2446)
- Features `phy-enable-usb` & `dump-packets` have been turned into configuration options `phy_enable_usb` & `dump_packets` (#2446)
- Features `ps-min-modem` & `ps-max-modem` have been removed in favour of a runtime config (#2446)
- The blocking networking stack is removed (#2488)

## 0.10.1 - 2024-10-10

### Changed

- Bumped esp-wifi-sys to `v0.6.0` (#2328)

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

### Fixed

- Builds on stable, again (#2067)

## 0.9.0 - 2024-09-03

### Added

- Added support for WPA2-ENTERPRISE (#2004)

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

- Check no password is set when using `AuthMethod::None` (#1806)

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

- Removed embedded-hal v0.2 dependency (#1582)

## 0.5.1 - 2024-04-22

### Fixed

- Patch release to fix docs.rs build (#1582)

## 0.5.0 - 2024-04-19

### Fixed

- Fix compile error when using smoltcp `DNS_MAX_RESULT_COUNT` values other than 1 (#1654)

## 0.4.0 - 2024-03-12

### Changed

- Users don't need embedded-svc to control wifi anymore. The wifi trait is optionally implemented now. (#429)
- Better network performance by forced yielding of the task when buffers are full / empty. (#430)
- Depend on esp-hal 0.16.1, update other dependencies (#1582)

## 0.3.0 - 2024-01-29

### Added

- Include coex in list of enabled features for docs.rs (#405)

### Fixed

- Small correction to coex warning message (#404)
- Use a random local port when initializing the wifi stack. (#414)

### Changed

- Update driver blobs (#410)
- Update dependencies to fit `embedded-hal` `1.0` (#1582)

## 0.2.0 - 2024-01-05

### Added

- Initial release supporting WiFi on ESP32, ESP32-S2, ESP32-S3, ESP32-C3, ESP32-C2, ESP32-C6, supporting BLE on WiFi on ESP32, ESP32-S3, ESP32-C3, ESP32-C2, ESP32-C6, ESP32-H2 (#1582)

## 0.1.0 - 2023-11-27

### Added

- Initial release supporting WiFi on ESP32, ESP32-S2, ESP32-S3, ESP32-C3, ESP32-C2, ESP32-C6, supporting BLE on WiFi on ESP32, ESP32-S3, ESP32-C3, ESP32-C2, ESP32-C6 (#1582)

[0.13.0]: https://github.com/esp-rs/esp-hal/releases/tag/esp-wifi-v0.13.0
[v0.14.0]: https://github.com/esp-rs/esp-hal/compare/esp-wifi-v0.13.0...esp-wifi-v0.14.0
[v0.14.1]: https://github.com/esp-rs/esp-hal/compare/esp-wifi-v0.14.0...esp-wifi-v0.14.1
[v0.15.0]: https://github.com/esp-rs/esp-hal/compare/esp-wifi-v0.14.1...esp-wifi-v0.15.0
[Unreleased]: https://github.com/esp-rs/esp-hal/compare/esp-wifi-v0.15.0...HEAD
