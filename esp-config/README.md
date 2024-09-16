# esp-config

[![Crates.io](https://img.shields.io/crates/v/esp-config?labelColor=1C2C2E&color=C96329&logo=Rust&style=flat-square)](https://crates.io/crates/esp-config)
[![docs.rs](https://img.shields.io/docsrs/esp-config?labelColor=1C2C2E&color=C96329&logo=rust&style=flat-square)](https://docs.rs/esp-config)
![MSRV](https://img.shields.io/badge/MSRV-1.79-blue?labelColor=1C2C2E&style=flat-square)
![Crates.io](https://img.shields.io/crates/l/esp-config?labelColor=1C2C2E&style=flat-square)
[![Matrix](https://img.shields.io/matrix/esp-rs:matrix.org?label=join%20matrix&labelColor=1C2C2E&color=BEC5C9&logo=matrix&style=flat-square)](https://matrix.to/#/#esp-rs:matrix.org)

## [Documentation](https://docs.rs/crate/esp-config)

## Usage

`esp-config` takes a prefix (usually the crate name) and a set of configuration keys and default values to produce a configuration system that supports:

- Emitting rustc cfg's for boolean keys
- Emitting environment variables for numbers
  - Along with decimal parsing, it supports Hex, Octal and Binary with the respective `0x`, `0o` and `0b` prefixes.
- Emitting environment variables string values

### Viewing the configuration

The possible configuration values are output as a markdown table in the crates `OUT_DIR` with the format `{prefix}_config_table.md`, this can then be included into the crates top level documentation. Here is an example of the output:


| Name | Description | Default value |
|------|-------------|---------------|
|**ESP_HAL_PLACE_SPI_DRIVER_IN_RAM**|Places the SPI driver in RAM for better performance|false|

### Setting configuration options

For any available configuration option, the environment variable or cfg is _always_ set based on the default value specified in the table. Users can override this by setting environment variables locally in their shell _or_ the preferred option is to utilize cargo's [`env` section](https://doc.rust-lang.org/cargo/reference/config.html#env).

It's important to note that due to a [bug in cargo](https://github.com/rust-lang/cargo/issues/10358), any modifications to the environment, local or otherwise will only get picked up on a full clean build of the project.

To see the final selected configuration another table is output to the `OUT_DIR` with the format `{prefix}_selected_config.md`.

## Minimum Supported Rust Version (MSRV)

This crate is guaranteed to compile on stable Rust 1.79 and up. It _might_
compile with older versions but that may change in any new patch release.

## License

Licensed under either of:

- Apache License, Version 2.0 ([LICENSE-APACHE](../LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](../LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in
the work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without
any additional terms or conditions.
