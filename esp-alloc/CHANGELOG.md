# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## Unreleased

### Added

- `allocator_api2` to support allocator APIs on stable Rust. (#3318)
- `AnyMemory`, `InternalMemory`, `ExternalMemory` allocators. (#3318)

### Changed

### Fixed

### Removed

## [0.7.0] - 2025-02-24

### Added

- `esp_alloc::heap_allocator!` now accepts attributes, e.g., `esp_alloc::heap_allocator!(#[link_section = ".dram2_uninit"] size: 64000)` (#3133)

### Changed

- `esp_alloc::heap_allocator!` syntax has been changed to `esp_alloc::heap_allocator!(size: 64000)` (#3135)

## 0.6.0 - 2025-01-15

### Added

- `esp_alloc::HEAP.stats()` can now be used to get heap usage informations (#2137)

### Changed

- Bump MSRV to 1.84 (#2951)

## 0.5.0 - 2024-10-10

### Changed

- a global allocator is created in esp-alloc, now you need to add individual memory regions (up to 3) to the allocator (#2099)

## 0.4.0 - 2024-06-04

## 0.3.0 - 2023-04-25

## 0.2.1 - 2023-04-21

## 0.2.0 - 2023-02-22

## 0.1.0 - 2022-07-25

[0.7.0]: https://github.com/esp-rs/esp-hal/releases/tag/esp-alloc-v0.7.0
