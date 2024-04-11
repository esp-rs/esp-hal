# esp-lp-hal

[![Crates.io](https://img.shields.io/crates/v/esp-lp-hal?labelColor=1C2C2E&color=C96329&logo=Rust&style=flat-square)](https://crates.io/crates/esp-lp-hal)
[![docs.rs](https://img.shields.io/docsrs/esp-lp-hal?labelColor=1C2C2E&color=C96329&logo=rust&style=flat-square)](https://docs.rs/esp-lp-hal)
![MSRV](https://img.shields.io/badge/MSRV-1.76-blue?labelColor=1C2C2E&style=flat-square)
![Crates.io](https://img.shields.io/crates/l/esp-lp-hal?labelColor=1C2C2E&style=flat-square)
[![Matrix](https://img.shields.io/matrix/esp-rs:matrix.org?label=join%20matrix&labelColor=1C2C2E&color=BEC5C9&logo=matrix&style=flat-square)](https://matrix.to/#/#esp-rs:matrix.org)

Bare-metal (`no_std`) hardware abstraction layer for the low-power RISC-V coprocessors found in the ESP32-C6, ESP32-S2, and ESP32-S3 from Espressif.

Implements a number of blocking and, where applicable, async traits from the various packages in the [embedded-hal] repository.

For help getting started with this HAL, please refer to [The Rust on ESP Book] and the [documentation].

[embedded-hal]: https://github.com/rust-embedded/embedded-hal
[the rust on esp book]: https://docs.esp-rs.org/book/

## [Documentation]

[documentation]: https://docs.rs/esp-lp-hal/

## Supported Devices

|   Chip   |        Datasheet         | Technical Reference Manual |             Target             |
| :------: | :----------------------: | :------------------------: | :----------------------------: |
| ESP32-C6 | [ESP32-C6][c6-datasheet] |     [ESP32-C6][c6-trm]     | `riscv32imac-unknown-none-elf` |
| ESP32-S2 | [ESP32-S2][s2-datasheet] |     [ESP32-S2][s2-trm]     | `riscv32imc-unknown-none-elf`  |
| ESP32-S3 | [ESP32-S3][s3-datasheet] |     [ESP32-S3][s3-trm]     | `riscv32imc-unknown-none-elf`  |

[c6-datasheet]: https://www.espressif.com/sites/default/files/documentation/esp32-c6_datasheet_en.pdf
[s2-datasheet]: https://www.espressif.com/sites/default/files/documentation/esp32-s2_datasheet_en.pdf
[s3-datasheet]: https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_en.pdf
[c6-trm]: https://www.espressif.com/sites/default/files/documentation/esp32-c6_technical_reference_manual_en.pdf
[s2-trm]: https://www.espressif.com/sites/default/files/documentation/esp32-s2_technical_reference_manual_en.pdf
[s3-trm]: https://www.espressif.com/sites/default/files/documentation/esp32-s3_technical_reference_manual_en.pdf

## Minimum Supported Rust Version (MSRV)

This crate is guaranteed to compile on stable Rust 1.76 and up. It _might_
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
