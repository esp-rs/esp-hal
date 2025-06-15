# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added


### Changed


### Fixed


### Removed


## [v0.4.0] - 2025-06-03

### Added

- `ConfigOption` struct (#3362)
- `Stabiliy` to specify unstable options, and the version in which they became stable (#3365)

### Changed

- `generate_config` now takes a slice of `ConfigOption`s instead of tuples. (#3362)
- Bump Rust edition to 2024, bump MSRV to 1.86. (#3391, #3560)
- `ConfigOption` favors `String` over `&str` (#3455)
- Removed the `Custom` validator (#3455)

## [0.3.1] - 2025-02-24

### Added

- `Enumeration` validator added (#3172)

## 0.3.0 - 2025-01-15

### Fixed

- Users no longer have to manually import `esp_config_int_parse`. (#2630)

### Changed

- Crate prefixes and configuration keys are now separated by `_CONFIG_` (#2848)
- Bump MSRV to 1.84 (#2951)

## 0.2.0 - 2024-11-20

### Added

- Add configuration validation (#2475)

## 0.1.0 - 2024-10-10

### Added

- Initial release (#2518)

[0.3.1]: https://github.com/esp-rs/esp-hal/releases/tag/esp-config-v0.3.1
[v0.4.0]: https://github.com/esp-rs/esp-hal/compare/esp-config-v0.3.1...esp-config-v0.4.0
[Unreleased]: https://github.com/esp-rs/esp-hal/compare/esp-config-v0.4.0...HEAD
