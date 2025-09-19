# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- `FlashRegion::partition_size` (#3902)
- `PartitionTable::booted_partition`(#3979)
- A new high-level OTA update helper, `OtaUpdater`, has been introduced. This simplifies the process of performing an OTA update by validating the partition table, finding the next available update slot, and handling the activation of the new image. (#4150)
- Support for partition tables with more than two OTA application partitions (up to 16). The OTA logic now correctly cycles through all available `ota_X` slots. (#4150)
- `PartitionTable::booted_partition()` function to determine which application partition is currently running by inspecting MMU registers. (#4150)
- `PartitionTable::iter()` to iterate over available partitions. (#4150)

### Changed

- The `ota` module's API has been updated to support a variable number of OTA partitions. `Ota::new` now requires the number of available OTA partitions. (#4150)
- `ota::Ota::current_slot()` and `set_current_slot()` have been replaced by the more explicit `current_app_partition()` and `set_current_app_partition()`, which now operate on `AppPartitionSubType`. (#4150)
- The `ota/update` example has been updated to use the new high-level `OtaUpdater`, demonstrating a simpler and safer update workflow. (#4150)

### Fixed

- FlashRegion: The `capacity` methods implemented for `embedded_storage::ReadStorage` and `embedded_storage::nor_flash::ReadNorFlash` now return the same value (#3902)
- Don't fail the build on long project names (#3905)
- FlashRegion: Fix off-by-one bug when bounds checking (#3977)
- Correctly set the `ESP_IDF_COMPATIBLE_VERSION` in the application descriptor. It was previously using the `MMU_PAGE_SIZE` configuration value by mistake. (#4150)

### Removed


## [v0.2.0] - 2025-07-16

### Changed

- The `log` feature has been renamed to `log-04` (#3675)
- `defmt` and `log-04` can no longer be selected at the same time (#3675)
- A chip feature (e.g. `esp32`) is now required (#3688)
- Use ROM functions for MD5 (#3758)

### Fixed

- Fixed a problem with calculating the otadata checksum (#3629)

## [v0.1.0] - 2025-06-03

### Added

- Support ESP-IDF app descriptor (#3281)
- Support reading partition tables and conveniently read/write partition content (#3316)
- OTA-DATA partition support (#3354)

### Changed

- Bump Rust edition to 2024, bump MSRV to 1.86. (#3391, #3560)
- Update `defmt` to 1.0 (#3416)

[v0.1.0]: https://github.com/esp-rs/esp-hal/releases/tag/esp-bootloader-esp-idf-v0.1.0
[v0.2.0]: https://github.com/esp-rs/esp-hal/compare/esp-bootloader-esp-idf-v0.1.0...esp-bootloader-esp-idf-v0.2.0
[Unreleased]: https://github.com/esp-rs/esp-hal/compare/esp-bootloader-esp-idf-v0.2.0...HEAD
