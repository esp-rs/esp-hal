# esp-hal

[![Crates.io](https://img.shields.io/crates/v/esp-hal?labelColor=1C2C2E&color=C96329&logo=Rust&style=flat-square)](https://crates.io/crates/esp-hal)
[![docs.rs](https://img.shields.io/docsrs/esp-hal?labelColor=1C2C2E&color=C96329&logo=rust&style=flat-square)](https://docs.espressif.com/projects/rust/esp-hal/latest/)
![MSRV](https://img.shields.io/badge/MSRV-1.88.0-blue?labelColor=1C2C2E&style=flat-square)
![Crates.io](https://img.shields.io/crates/l/esp-hal?labelColor=1C2C2E&style=flat-square)
[![Matrix](https://img.shields.io/matrix/esp-rs:matrix.org?label=join%20matrix&labelColor=1C2C2E&color=BEC5C9&logo=matrix&style=flat-square)](https://matrix.to/#/#esp-rs:matrix.org)

Bare-metal (`no_std`) hardware abstraction layer for Espressif devices.

Implements a number of blocking and, where applicable, async traits from the various packages in the [embedded-hal] repository.

For help getting started with this HAL, please refer to [The Rust on ESP Book] and the [documentation].

[embedded-hal]: https://github.com/rust-embedded/embedded-hal
[the rust on esp book]: https://docs.espressif.com/projects/rust/book/

## [Documentation]

[documentation]: https://docs.espressif.com/projects/rust/

## Supported Devices

|   Chip   |         Datasheet          | Technical Reference Manual |             Target             |
| :------: | :------------------------: | :------------------------: | :----------------------------: |
|  ESP32   |    [ESP32][32-datasheet]   |      [ESP32][32-trm]       |    `xtensa-esp32-none-elf`     |
| ESP32-C2 |  [ESP32-C2][c2-datasheet]  |     [ESP32-C2][c2-trm]     | `riscv32imc-unknown-none-elf`  |
| ESP32-C3 |  [ESP32-C3][c3-datasheet]  |     [ESP32-C3][c3-trm]     | `riscv32imc-unknown-none-elf`  |
| ESP32-C5 |  [ESP32-C5][c5-datasheet]  |     [ESP32-C5][c5-trm]     | `riscv32imac-unknown-none-elf` |
| ESP32-C6 |  [ESP32-C6][c6-datasheet]  |     [ESP32-C6][c6-trm]     | `riscv32imac-unknown-none-elf` |
| ESP32-C61| [ESP32-C61][c61-datasheet] |     [ESP32-C61][c61-trm]   | `riscv32imac-unknown-none-elf` |
| ESP32-H2 |  [ESP32-H2][h2-datasheet]  |     [ESP32-H2][h2-trm]     | `riscv32imac-unknown-none-elf` |
| ESP32-P4 |  [ESP32-P4][p4-datasheet]  |     [ESP32-P4][p4-trm]     | `riscv32imafc-unknown-none-elf`|
| ESP32-S2 |  [ESP32-S2][s2-datasheet]  |     [ESP32-S2][s2-trm]     |   `xtensa-esp32s2-none-elf`    |
| ESP32-S3 |  [ESP32-S3][s3-datasheet]  |     [ESP32-S3][s3-trm]     |   `xtensa-esp32s3-none-elf`    |

[32-datasheet]: https://www.espressif.com/sites/default/files/documentation/esp32_datasheet_en.pdf
[c2-datasheet]: https://www.espressif.com/sites/default/files/documentation/esp8684_datasheet_en.pdf
[c3-datasheet]: https://www.espressif.com/sites/default/files/documentation/esp32-c3_datasheet_en.pdf
[c5-datasheet]: https://www.espressif.com/sites/default/files/documentation/esp32-c5_datasheet_en.pdf
[c6-datasheet]: https://www.espressif.com/sites/default/files/documentation/esp32-c6_datasheet_en.pdf
[c61-datasheet]: https://www.espressif.com/sites/default/files/documentation/esp32-c61_datasheet_en.pdf
[h2-datasheet]: https://www.espressif.com/sites/default/files/documentation/esp32-h2_datasheet_en.pdf
[p4-datasheet]: https://www.espressif.com/sites/default/files/documentation/esp32-p4_datasheet_en.pdf
[s2-datasheet]: https://www.espressif.com/sites/default/files/documentation/esp32-s2_datasheet_en.pdf
[s3-datasheet]: https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_en.pdf
[32-trm]: https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf
[c2-trm]: https://www.espressif.com/sites/default/files/documentation/esp8684_technical_reference_manual_en.pdf
[c3-trm]: https://www.espressif.com/sites/default/files/documentation/esp32-c3_technical_reference_manual_en.pdf
[c5-trm]: https://www.espressif.com/sites/default/files/documentation/esp32-c5_technical_reference_manual_en.pdf
[c6-trm]: https://www.espressif.com/sites/default/files/documentation/esp32-c6_technical_reference_manual_en.pdf
[c61-trm]: https://www.espressif.com/sites/default/files/documentation/esp32-c61_technical_reference_manual_en.pdf
[h2-trm]: https://www.espressif.com/sites/default/files/documentation/esp32-h2_technical_reference_manual_en.pdf
[p4-trm]: https://www.espressif.com/sites/default/files/documentation/esp32-p4_technical_reference_manual_en.pdf
[s2-trm]: https://www.espressif.com/sites/default/files/documentation/esp32-s2_technical_reference_manual_en.pdf
[s3-trm]: https://www.espressif.com/sites/default/files/documentation/esp32-s3_technical_reference_manual_en.pdf

## Peripheral support

<!-- The following table is machine generated. Do not edit the comments and the table by hand! -->
<!-- start chip support table -->
| Driver                    | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C5 | ESP32-C6 | ESP32-C61 | ESP32-H2 | ESP32-P4 | ESP32-S2 | ESP32-S3 |
| ------------------------- |:-----:|:--------:|:--------:|:--------:|:--------:|:---------:|:--------:|:--------:|:--------:|:--------:|
| ADC                       | ⚒️   | ⚒️      | ⚒️      | ⚒️      | ⚒️      | [❌][5422] [^1] | ⚒️      | ❌       | ⚒️      | ⚒️      |
| AES                       | ⚒️   |          | ⚒️      | ⚒️      | ⚒️      |           | ⚒️      | ❌       | ⚒️      | ⚒️      |
| ASSIST_DEBUG              |       | ⚒️      | ⚒️      | ⚒️      | ⚒️      | ⚒️       | ⚒️      | ❌       |          | ⚒️      |
| Analog Voltage Comparator |       |          |          | [❌][5168] [^1] |          | [❌][5423] [^1] |          | ❌       |          |          |
| Bit Scrambler             |       |          |          | [❌][5170] [^1] |          |           |          |          |          |          |
| Bluetooth                 | ⚒️   | ⚒️      | ⚒️      | ⚒️      | ⚒️      | ⚒️       | ⚒️      |          |          | ⚒️      |
| Camera interface          | ❌    |          |          |          |          |           |          | ❌       | ❌       | ⚒️      |
| DAC                       | ⚒️   |          |          |          |          |           |          |          | ⚒️      |          |
| Dedicated GPIO            |       | ⚒️      | ⚒️      | ⚒️      | ⚒️      | ⚒️       | ⚒️      |          | ⚒️      | ⚒️      |
| DMA                       | ⚒️   | ⚒️      | ⚒️      | ⚒️      | ⚒️      | ⚒️       | ⚒️      | ❌       | ⚒️      | ⚒️      |
| DS                        |       |          | [❌][884] [^1] | [❌][884] [^1] | [❌][884] [^1] |           | [❌][884] [^1] | ❌       | [❌][884] [^1] | [❌][884] [^1] |
| ECDSA                     |       |          |          | [❌][5444] [^1] |          | [❌][5444] [^1] | [❌][5444] [^1] | ❌       |          |          |
| ECC                       |       | ⚒️      |          | ⚒️      | ⚒️      | ⚒️       | ⚒️      | ⚒️      |          |          |
| Ethernet                  | ❌    |          |          |          |          |           |          | ❌       |          |          |
| ETM                       |       |          |          | [❌][5167] [^1] | ⚒️      | [❌][5419] [^1] | ⚒️      | ❌       |          |          |
| GPIO                      | ✔️   | ✔️      | ✔️      | ⚒️      | ✔️      | ⚒️       | ✔️      | ⚒️      | ✔️      | ✔️      |
| HMAC                      |       |          | ⚒️      | [❌][5166] [^1] | ⚒️      |           | ⚒️      | ❌       | ⚒️      | ⚒️      |
| I2C master                | ✔️   | ✔️      | ✔️      | ✔️      | ✔️      | ✔️       | ✔️      | ⚒️      | ✔️      | ✔️      |
| I2C slave                 | [❌][1909] [^1] |          | [❌][1909] [^1] | [❌][1909] [^1] | [❌][1909] [^1] | [❌][1909] [^1] | [❌][1909] [^1] | ❌       | [❌][1909] [^1] | [❌][1909] [^1] |
| I2S                       | ⚒️   |          | ⚒️      | ⚒️      | ⚒️      | ⚒️       | ⚒️      | ❌       | ⚒️      | ⚒️      |
| IEEE 802.15.4             |       |          |          | ⚒️      | ⚒️      |           | ⚒️      |          |          |          |
| Interrupts                | ⚒️   | ⚒️      | ⚒️      | ⚒️      | ⚒️      | ⚒️       | ⚒️      | ⚒️      | ⚒️      | ⚒️      |
| IOMUX                     | ⚒️   | ⚒️      | ⚒️      | ⚒️      | ⚒️      | ⚒️       | ⚒️      |          | ⚒️      | ⚒️      |
| Key Manager               |       |          |          | [❌][5171] [^1] |          |           |          |          |          |          |
| LEDC                      | ⚒️   | ⚒️      | ⚒️      | [❌][5161] [^1] | ⚒️      | [❌][5418] [^1] | ⚒️      | ❌       | ⚒️      | ⚒️      |
| LP I2C master             |       |          |          | ⚒️      | ⚒️      |           |          | ❌       |          |          |
| LP UART                   |       |          |          | [❌][5155] [^1] | ⚒️      |           |          | ❌       |          |          |
| MCPWM                     | ⚒️   |          |          | [❌][5154] [^1] | ⚒️      |           | ⚒️      | ❌       |          | ⚒️      |
| PARL_IO                   |       |          |          | ⚒️      | ⚒️      |           | ⚒️      |          |          |          |
| PCNT                      | ⚒️   |          |          | ⚒️      | ⚒️      |           | ⚒️      | ❌       | ⚒️      | ⚒️      |
| PHY                       | ⚒️   | ⚒️      | ⚒️      | ⚒️      | ⚒️      | ⚒️       | ⚒️      |          | ⚒️      | ⚒️      |
| PSRAM                     | ⚒️   |          |          | ⚒️      |          | ⚒️       |          | ⚒️      | ⚒️      | ⚒️      |
| RGB display               | ⚒️   |          |          |          |          |           |          | ❌       | ❌       | ⚒️      |
| RMT                       | ⚒️   |          | ⚒️      | ⚒️      | ⚒️      |           | ⚒️      |          | ⚒️      | ⚒️      |
| RNG                       | ⚒️   | ⚒️      | ⚒️      | ⚒️      | ⚒️      | ⚒️       | ⚒️      | ⚒️      | ⚒️      | ⚒️      |
| RSA                       | ⚒️   |          | ⚒️      | ⚒️      | ⚒️      |           | ⚒️      | ❌       | ⚒️      | ⚒️      |
| RTC Timekeeping           | ⚒️   | ⚒️      | ⚒️      | ⚒️      | ⚒️      | ⚒️       | ⚒️      | ❌       | ⚒️      | ⚒️      |
| SDIO host                 | ⚒️   |          |          |          |          |           |          | ❌       |          | ⚒️      |
| SDIO slave                | ⚒️   |          |          | [❌][5169] [^1] | ⚒️      | [❌][5417] [^1] |          | ❌       |          |          |
| SHA                       | ⚒️   | ⚒️      | ⚒️      | ⚒️      | ⚒️      | ⚒️       | ⚒️      | ❌       | ⚒️      | ⚒️      |
| Light/deep sleep          | ⚒️   | ⚒️      | ⚒️      | [❌][5165] [^1] | ⚒️      | [❌][5424] [^1] | ⚒️      | ❌       | ⚒️      | ⚒️      |
| SPI master                | ✔️   | ✔️      | ✔️      | ✔️      | ✔️      | ✔️       | ✔️      | ⚒️      | ✔️      | ✔️      |
| SPI slave                 | ⚒️   | ⚒️      | ⚒️      | ⚒️      | ⚒️      | ⚒️       | ⚒️      | ❌       | ⚒️      | ⚒️      |
| SYSTIMER                  |       | ⚒️      | ⚒️      | ⚒️      | ⚒️      | ⚒️       | ⚒️      | ⚒️      | ⚒️      | ⚒️      |
| Temperature sensor        | ⚒️   | ⚒️      | ⚒️      | [❌][5153] [^1] | ⚒️      | [❌][5421] [^1] | ⚒️      | ❌       | ⚒️      | ⚒️      |
| Timers                    | ⚒️   | ⚒️      | ⚒️      | ⚒️      | ⚒️      | ⚒️       | ⚒️      | ⚒️      | ⚒️      | ⚒️      |
| Touch                     | ⚒️   |          |          | [❌][5164] [^1] |          |           |          | ❌       | [❌][1905] [^1] | [❌][1905] [^1] |
| TWAI / CAN / CANFD        | ⚒️   |          | ⚒️      | [❌][5163] [^1] | ⚒️      |           | ⚒️      | ❌       | ⚒️      | ⚒️      |
| UART                      | ✔️   | ✔️      | ✔️      | ✔️      | ✔️      | ✔️       | ✔️      | ⚒️      | ✔️      | ✔️      |
| UHCI                      | ❌    |          | ⚒️      | ⚒️      | ⚒️      |           | ⚒️      | ❌       | ❌       | ⚒️      |
| ULP (FSM)                 | ⚒️   |          |          |          |          |           |          |          | ⚒️      | ⚒️      |
| ULP (RISC-V)              |       |          |          | [❌][5160] [^1] | ⚒️      |           |          | ❌       | ⚒️      | ⚒️      |
| USB OTG FS                |       |          |          |          |          |           |          |          | ⚒️      | ⚒️      |
| USB Serial/JTAG           |       |          | ⚒️      | ⚒️      | ⚒️      | ⚒️       | ⚒️      | ⚒️      |          | ⚒️      |
| WIFI                      | ⚒️   | ⚒️      | ⚒️      | ⚒️      | ⚒️      | ⚒️       |          |          | ⚒️      | ⚒️      |

 * Empty cell: Not available
 * ❌: Not supported
 * ⚒️: Partial support
 * ✔️: Supported

[^1]: This cell is clickable and will open the peripheral's issue on GitHub

[884]: https://github.com/esp-rs/esp-hal/issues/884
[1905]: https://github.com/esp-rs/esp-hal/issues/1905
[1909]: https://github.com/esp-rs/esp-hal/issues/1909
[5153]: https://github.com/esp-rs/esp-hal/issues/5153
[5154]: https://github.com/esp-rs/esp-hal/issues/5154
[5155]: https://github.com/esp-rs/esp-hal/issues/5155
[5160]: https://github.com/esp-rs/esp-hal/issues/5160
[5161]: https://github.com/esp-rs/esp-hal/issues/5161
[5163]: https://github.com/esp-rs/esp-hal/issues/5163
[5164]: https://github.com/esp-rs/esp-hal/issues/5164
[5165]: https://github.com/esp-rs/esp-hal/issues/5165
[5166]: https://github.com/esp-rs/esp-hal/issues/5166
[5167]: https://github.com/esp-rs/esp-hal/issues/5167
[5168]: https://github.com/esp-rs/esp-hal/issues/5168
[5169]: https://github.com/esp-rs/esp-hal/issues/5169
[5170]: https://github.com/esp-rs/esp-hal/issues/5170
[5171]: https://github.com/esp-rs/esp-hal/issues/5171
[5417]: https://github.com/esp-rs/esp-hal/issues/5417
[5418]: https://github.com/esp-rs/esp-hal/issues/5418
[5419]: https://github.com/esp-rs/esp-hal/issues/5419
[5421]: https://github.com/esp-rs/esp-hal/issues/5421
[5422]: https://github.com/esp-rs/esp-hal/issues/5422
[5423]: https://github.com/esp-rs/esp-hal/issues/5423
[5424]: https://github.com/esp-rs/esp-hal/issues/5424
[5444]: https://github.com/esp-rs/esp-hal/issues/5444
<!-- end chip support table -->

## `unstable` feature

The stable feature set is designed to remain consistent and reliable. Other parts guarded by the `unstable` feature, however, are still under active development and may undergo breaking changes and are disabled by default.

## Minimum Supported Rust Version (MSRV)

This crate is guaranteed to compile when using the latest stable Rust version at the time of the crate's release. It _might_ compile with older versions, but that may change in any new release, including patches.

## License

Licensed under either of:

- Apache License, Version 2.0 ([LICENSE-APACHE](../LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](../LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in
the work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without
any additional terms or conditions.
