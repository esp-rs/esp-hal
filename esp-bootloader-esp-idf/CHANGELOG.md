# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added


### Changed


### Fixed

- Fixed a problem with calculating the otadata checksum (#3629)

### Removed


## [v0.1.0] - 2025-06-03

### Added

- Support ESP-IDF app descriptor (#3281)
- Support reading partition tables and conveniently read/write partition content (#3316)
- OTA-DATA partition support (#3354)

### Changed

- Bump Rust edition to 2024, bump MSRV to 1.86. (#3391, #3560)
- Update `defmt` to 1.0 (#3416)

[v0.1.0]: https://github.com/esp-rs/esp-hal/releases/tag/esp-bootloader-esp-idf-v0.1.0
[Unreleased]: https://github.com/esp-rs/esp-hal/compare/esp-bootloader-esp-idf-v0.1.0...HEAD
