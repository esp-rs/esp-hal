# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## Unreleased

### Added

- This package now re-exports the `esp_hal_procmacros::main` macro (#1828)

### Changed

### Fixed

### Removed

## 0.2.0 - 2024-07-15

### Changed

- Removed the TIMG and SYSTIMER time drivers, replaced by a generic time driver taking `OneShotTimer<ErasedTimer>` (#1753)

## 0.1.0 - 2024-06-04
