# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## Unreleased

### Added

- Added board-specific consts for c6 and h2 when caluclating transmit power conversion

### Changed

- Modified CCA threshold value to default of -60
- The driver now take `RADIO_CLK` by value to avoid a collision with esp-wifi's usage (#2183)

### Fixed

- Fixed possible integer underflow in array access
- Fixed compile error when building binary-logs feature

### Removed

## 0.2.0 - 2024-08-29

### Added

- Added additional checks to prevent various array access panics while processing frames (#1923)
- Added range check to avoid panic when indexing into RX_BUFFER slice (#1682)

## 0.1.0 - 2024-07-15

### Added

- Initial release
