# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added


### Changed


### Fixed


### Removed


## [v0.8.0] - 2025-07-16

### Changed

- `esp_ieee802154::Ieee802154::new` no longer requires the `RADIO_CLK` peripheral (#3687)
- MSRV is now 1.88.0 (#3742)

## [v0.7.0] - 2025-06-03

### Changed

- Bump Rust edition to 2024, bump MSRV to 1.86. (#3391, #3560)
- Update `defmt` to 1.0 (#3416)
- The `log` feature has been replaced by `log-04`. (#3425)

### Fixed

- Fixed compatibility with `defmt` (#3374)

## [0.6.0] - 2025-02-24

## 0.5.0 - 2025-01-15

### Changed

- Config: Crate prefixes and configuration keys are now separated by `_CONFIG_` (#2848)
- Bump MSRV to 1.84 (#2951)

## 0.4.0 - 2024-11-20

### Removed

- Removed `get_` prefixes from functions (#2528)

## 0.3.1 - 2024-10-10

### Changed

- Bumped esp-wifi-sys to `v0.6.0` (#2328)

## 0.3.0 - 2024-10-10 - YANKED

### Added

- Added board-specific consts for c6 and h2 when caluclating transmit power conversion (#2114)
- Added `defmt` and `log` features (#2183)
- Make RX queue size configurable using esp-config (#2324)

### Changed

- Modified CCA threshold value to default of -60 (#2114)
- The driver now take `RADIO_CLK` by value to avoid a collision with esp-wifi's usage (#2183)
- `binary-logs` feature renamed to `sys-logs` (#2183)
- Updated PHY driver to v5.3.1 (#2239)

### Fixed

- Fixed possible integer underflow in array access (#2114)
- Fixed compile error when building sys-logs feature (#2114)

## 0.2.0 - 2024-08-29

### Added

- Added additional checks to prevent various array access panics while processing frames (#1923)
- Added range check to avoid panic when indexing into RX_BUFFER slice (#1682)

## 0.1.0 - 2024-07-15

### Added

- Initial release (#1800)

[0.6.0]: https://github.com/esp-rs/esp-hal/releases/tag/esp-ieee802154-v0.6.0
[v0.7.0]: https://github.com/esp-rs/esp-hal/compare/esp-ieee802154-v0.6.0...esp-ieee802154-v0.7.0
[v0.8.0]: https://github.com/esp-rs/esp-hal/compare/esp-ieee802154-v0.7.0...esp-ieee802154-v0.8.0
[Unreleased]: https://github.com/esp-rs/esp-hal/compare/esp-ieee802154-v0.8.0...HEAD
