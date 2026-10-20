# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added


### Changed


### Fixed


### Removed


## [v0.10.0] - 2026-08-25

### Added

- Add ESP32-S31 support (#5922)
- `FlashStorage::read_encrypted` and `FlashStorage::write_encrypted` for transparent flash encryption read/write (#5810)
- `flash_encryption()` helper to query whether flash encryption is enabled in eFuses (#5810)
- Support for ESP32-P4 (#5751)
- Inherent flash I/O methods on `FlashStorage`: `read`, `write`, `capacity`, `read_nor`, `write_nor`, and `erase` (#5739)

### Changed

- `FlashStorage::read_encrypted` now follows the multi-core strategy, so it can return `OtherCoreRunning`. (#6120)
- esp-storage can now be built with any opt-level (#5911)
- updated defmt to 1.1 (#5752)
- `embedded-storage` trait implementations are now an opt-in feature (non-default) (#5739)

### Fixed

- On ESP32, reading and writing encrypted flash no longer hangs. The flash MMU and the cache are now changed with the cache turned off, as the hardware requires. (#6120)
- `FlashStorage::erase` rejects reversed ranges (`to < from`) (#6045)

## [v0.9.0] - 2026-04-16

### Added

- Support for ESP32-C5 (#4998)
- Support for ESP32-C61 (#5240)

### Changed

- Speed up erase by erasing entire blocks when possible (#5079)

## [v0.8.1] - 2025-10-30

## [v0.8.0] - 2025-10-13

### Added

- `defmt` feature and `FlashStorage`, `FlashStorageError` now implements `Defmt` (#4127)
- `Drop` impl for `FlashStorage` (#4132)
- `FlashStorage::new()` now takes a `Flash` argument (#4173)

### Removed

- `Default` impl for `FlashStorage` (#4132)

## v0.8.0 - 2025-09-10

### Added

- Added strategies for dealing with multi-core systems (#4082)

## [v0.7.0] - 2025-07-16

### Fixed

- Fix incorrect usage of MaybeUninit (#3677)
- Use `libesp_rom.a` from ESP-IDF on ESP32 as a replacement for our incomplete and less accurate port (#3688)

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
[v0.7.0]: https://github.com/esp-rs/esp-hal/compare/esp-storage-v0.6.0...esp-storage-v0.7.0
[v0.8.0]: https://github.com/esp-rs/esp-hal/compare/esp-storage-v0.7.0...esp-storage-v0.8.0
[v0.8.1]: https://github.com/esp-rs/esp-hal/compare/esp-storage-v0.8.0...esp-storage-v0.8.1
[v0.9.0]: https://github.com/esp-rs/esp-hal/compare/esp-storage-v0.8.1...esp-storage-v0.9.0
[v0.10.0]: https://github.com/esp-rs/esp-hal/compare/esp-storage-v0.9.0...esp-storage-v0.10.0
[Unreleased]: https://github.com/esp-rs/esp-hal/compare/esp-storage-v0.10.0...HEAD
