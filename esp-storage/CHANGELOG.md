# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added


### Changed


### Fixed


### Removed


## [v0.6.0] - 2025-06-03

### Changed

- Bump Rust edition to 2024, bump MSRV to 1.86. (#3391, #3560)
- The `low-level` feature has been removed, the gated API is always available (#3425)

### Removed

- The `storage` and `nor-flash` features have been removed, the related functionality is now always available. (#3431)

## [0.5.0] - 2025-02-24

### Changed

- Bump MSRV to 1.84 (#2951)
- Add support for 32MB flash (#3163)

## 0.4.0 - 2024-11-20

### Added

- Added trait MultiwriteNorFlash to FlashStorage (#2478)

## 0.3.1 - 2024-10-10

## 0.3.0 - 2023-08-16

## 0.2.0 - 2023-07-05

## 0.1.0 - 2022-09-26

[0.5.0]: https://github.com/esp-rs/esp-hal/releases/tag/esp-storage-v0.5.0
[v0.6.0]: https://github.com/esp-rs/esp-hal/compare/esp-storage-v0.5.0...esp-storage-v0.6.0
[Unreleased]: https://github.com/esp-rs/esp-hal/compare/esp-storage-v0.6.0...HEAD
