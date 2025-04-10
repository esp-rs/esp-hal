# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## Unreleased

### Added

- `ConfigOption` struct (#3362)
- `Stabiliy` to specify unstable options, and the version in which they became stable (#3365)

### Changed

- `generate_config` now takes a slice of `ConfigOption`s instead of tuples. (#3362)

### Fixed

### Removed

## [0.3.1] - 2025-02-24

- `Enumeration` validator added

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

- Initial release

[0.3.1]: https://github.com/esp-rs/esp-hal/releases/tag/esp-config-v0.3.1
