# esp-hal

[![Crates.io](https://img.shields.io/crates/v/esp-hal?labelColor=1C2C2E&color=C96329&logo=Rust&style=flat-square)](https://crates.io/crates/esp-hal)
[![docs.rs](https://img.shields.io/docsrs/esp-hal?labelColor=1C2C2E&color=C96329&logo=rust&style=flat-square)](https://docs.espressif.com/projects/rust/esp-hal/latest/)
![MSRV](https://img.shields.io/badge/MSRV-1.95.0-blue?labelColor=1C2C2E&style=flat-square)
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

<!-- The following table is machine generated. Do not edit the comments and the table by hand! -->
<!-- start supported devices table -->
| Chip  | Documentation | Target | Note  |
| :---: | :-----------: | :----: | :---: |
| ESP32 | [Datasheet][esp32-datasheet] [TRM][esp32-trm] | `xtensa-esp32-none-elf` | Revisions below v3.0 are not supported. |
| ESP32-C2 | [Datasheet][esp32c2-datasheet] [TRM][esp32c2-trm] | `riscv32imc-unknown-none-elf` |  |
| ESP32-C3 | [Datasheet][esp32c3-datasheet] [TRM][esp32c3-trm] | `riscv32imc-unknown-none-elf` |  |
| ESP32-C5 | [Datasheet][esp32c5-datasheet] [TRM][esp32c5-trm] | `riscv32imac-unknown-none-elf` |  |
| ESP32-C6 | [Datasheet][esp32c6-datasheet] [TRM][esp32c6-trm] | `riscv32imac-unknown-none-elf` |  |
| ESP32-C61 | [Datasheet][esp32c61-datasheet] [TRM][esp32c61-trm] | `riscv32imac-unknown-none-elf` |  |
| ESP32-H2 | [Datasheet][esp32h2-datasheet] [TRM][esp32h2-trm] | `riscv32imac-unknown-none-elf` |  |
| ESP32-H4 | [Datasheet][esp32h4-datasheet] [TRM][esp32h4-trm] | `riscv32imafc-unknown-none-elf` | In early stages of development. |
| ESP32-P4 | [Datasheet][esp32p4-datasheet] [TRM][esp32p4-trm] | `riscv32imafc-unknown-none-elf` | Revisions below v3.0 are not supported. |
| ESP32-S2 | [Datasheet][esp32s2-datasheet] [TRM][esp32s2-trm] | `xtensa-esp32s2-none-elf` |  |
| ESP32-S3 | [Datasheet][esp32s3-datasheet] [TRM][esp32s3-trm] | `xtensa-esp32s3-none-elf` |  |
| ESP32-S31 | [Datasheet][esp32s31-datasheet] [TRM][esp32s31-trm] | `riscv32imafc-unknown-none-elf` | In early stages of development. |

[esp32-datasheet]: https://www.espressif.com/sites/default/files/documentation/esp32_datasheet_en.pdf
[esp32-trm]: https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf
[esp32c2-datasheet]: https://www.espressif.com/sites/default/files/documentation/esp8684_datasheet_en.pdf
[esp32c2-trm]: https://www.espressif.com/sites/default/files/documentation/esp8684_technical_reference_manual_en.pdf
[esp32c3-datasheet]: https://www.espressif.com/sites/default/files/documentation/esp32-c3_datasheet_en.pdf
[esp32c3-trm]: https://www.espressif.com/sites/default/files/documentation/esp32-c3_technical_reference_manual_en.pdf
[esp32c5-datasheet]: https://www.espressif.com/sites/default/files/documentation/esp32-c5_datasheet_en.pdf
[esp32c5-trm]: https://www.espressif.com/sites/default/files/documentation/esp32-c5_technical_reference_manual_en.pdf
[esp32c6-datasheet]: https://www.espressif.com/sites/default/files/documentation/esp32-c6_datasheet_en.pdf
[esp32c6-trm]: https://www.espressif.com/sites/default/files/documentation/esp32-c6_technical_reference_manual_en.pdf
[esp32c61-datasheet]: https://www.espressif.com/sites/default/files/documentation/esp32-c61_datasheet_en.pdf
[esp32c61-trm]: https://www.espressif.com/sites/default/files/documentation/esp32-c61_technical_reference_manual_en.pdf
[esp32h2-datasheet]: https://www.espressif.com/sites/default/files/documentation/esp32-h2_datasheet_en.pdf
[esp32h2-trm]: https://www.espressif.com/sites/default/files/documentation/esp32-h2_technical_reference_manual_en.pdf
[esp32h4-datasheet]: https://www.espressif.com/sites/default/files/documentation/esp32-h4_datasheet_en.pdf
[esp32h4-trm]: https://www.espressif.com/sites/default/files/documentation/esp32-h4_technical_reference_manual_en.pdf
[esp32p4-datasheet]: https://www.espressif.com/sites/default/files/documentation/esp32-p4_datasheet_en.pdf
[esp32p4-trm]: https://www.espressif.com/sites/default/files/documentation/esp32-p4_technical_reference_manual_en.pdf
[esp32s2-datasheet]: https://www.espressif.com/sites/default/files/documentation/esp32-s2_datasheet_en.pdf
[esp32s2-trm]: https://www.espressif.com/sites/default/files/documentation/esp32-s2_technical_reference_manual_en.pdf
[esp32s3-datasheet]: https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_en.pdf
[esp32s3-trm]: https://www.espressif.com/sites/default/files/documentation/esp32-s3_technical_reference_manual_en.pdf
[esp32s31-datasheet]: https://www.espressif.com/sites/default/files/documentation/esp32-s31_datasheet_en.pdf
[esp32s31-trm]: https://www.espressif.com/sites/default/files/documentation/esp32-s31_technical_reference_manual_en.pdf
<!-- end supported devices table -->

## Peripheral support

<!-- The following table is machine generated. Do not edit the comments and the table by hand! -->
<!-- start chip support table -->
For a detailed per-chip support matrix, see the [documentation index](https://docs.espressif.com/projects/rust/esp-hal/latest/index.html#peripheral-support).

 * ❌: Not supported
 * ⚠️: Planned overhaul (unstable, expect major breaking changes)
 * 🚧: Unstable (requires the `unstable` feature)
 * ✔️: Stable

### [GPIO](https://docs.espressif.com/projects/rust/esp-hal/latest/index.html#gpio)

| Driver         | Status |
| -------------- |:------:|
| GPIO           | ✔️    |
| Dedicated GPIO | 🚧     |
| IOMUX          | 🚧     |
| LP IO          | 🚧     |

### [Serial interfaces](https://docs.espressif.com/projects/rust/esp-hal/latest/index.html#serial-interfaces)

| Driver          | Status |
| --------------- |:------:|
| UART            | ✔️    |
| LP UART         | 🚧     |
| UHCI            | 🚧     |
| I2C master      | ✔️    |
| LP I2C master   | 🚧     |
| I2C slave       | 🚧     |
| I3C master      | ❌     |
| I3C slave       | ❌     |
| SPI master      | ✔️    |
| SPI slave       | 🚧     |
| LP SPI master   | ❌     |
| LP SPI slave    | ❌     |
| I2S             | 🚧     |
| LP I2S          | ❌     |
| PARL_IO         | 🚧     |
| RMT             | 🚧     |
| SDMMC/SDIO host | 🚧     |
| SDIO slave      | ❌     |
| TWAI            | 🚧     |
| CANFD           | 🚧     |
| USB OTG FS      | 🚧     |
| USB OTG HS      | 🚧     |
| USB Serial/JTAG | 🚧     |

### [Wireless and networking](https://docs.espressif.com/projects/rust/esp-hal/latest/index.html#wireless-and-networking)

| Driver        | Status |
| ------------- |:------:|
| Bluetooth     | 🚧     |
| WIFI          | 🚧     |
| IEEE 802.15.4 | 🚧     |
| Ethernet      | 🚧     |

### [Display and camera](https://docs.espressif.com/projects/rust/esp-hal/latest/index.html#display-and-camera)

| Driver            | Status |
| ----------------- |:------:|
| Camera interface  | 🚧     |
| MIPI-CSI          | ❌     |
| MIPI-DSI          | 🚧     |
| RGB display       | 🚧     |
| SPI LCD interface | [❌][5374] [^1] |

### [Signal processing](https://docs.espressif.com/projects/rust/esp-hal/latest/index.html#signal-processing)

| Driver                       | Status |
| ---------------------------- |:------:|
| Bit Scrambler                | ❌     |
| Image Signal Processor       | ❌     |
| Pixel Processing Accelerator | ❌     |
| JPEG Codec                   | ❌     |
| H.264 encoder                | ❌     |
| Audio Sample Rate Converter  | ❌     |
| CORDIC accelerator           | ❌     |
| Voice Activity Detection     | ❌     |

### [Analog](https://docs.espressif.com/projects/rust/esp-hal/latest/index.html#analog)

| Driver                    | Status |
| ------------------------- |:------:|
| ADC                       | 🚧     |
| Analog Voltage Comparator | ❌     |
| DAC                       | 🚧     |
| Temperature sensor        | 🚧     |
| Touch                     | 🚧     |

### [Timers and motor control](https://docs.espressif.com/projects/rust/esp-hal/latest/index.html#timers-and-motor-control)

| Driver          | Status |
| --------------- |:------:|
| LEDC            | [⚠️][2286] [^1] |
| MCPWM           | 🚧     |
| PCNT            | 🚧     |
| RTC Timekeeping | 🚧     |
| SDM             | 🚧     |
| SYSTIMER        | 🚧     |
| Timers          | 🚧     |

### [Cryptography and security](https://docs.espressif.com/projects/rust/esp-hal/latest/index.html#cryptography-and-security)

| Driver                  | Status |
| ----------------------- |:------:|
| AES                     | 🚧     |
| RSA Digital Signature   | ❌     |
| ECDSA Digital Signature | ❌     |
| ECC                     | 🚧     |
| HMAC                    | 🚧     |
| Key Manager             | ❌     |
| RNG                     | ✔️    |
| RSA                     | 🚧     |
| SHA                     | 🚧     |

### [Low power](https://docs.espressif.com/projects/rust/esp-hal/latest/index.html#low-power)

| Driver           | Status |
| ---------------- |:------:|
| Light/deep sleep | 🚧     |
| ULP (FSM)        | [❌][376] [^1] |
| ULP (RISC-V)     | 🚧     |

### [System](https://docs.espressif.com/projects/rust/esp-hal/latest/index.html#system)

| Driver       | Status |
| ------------ |:------:|
| ASSIST_DEBUG | 🚧     |
| DMA          | 🚧     |
| ETM          | 🚧     |
| Interrupts   | 🚧     |
| Flash        | 🚧     |
| PSRAM        | 🚧     |

[^1]: This cell is clickable and will open the peripheral's issue on GitHub

[376]: https://github.com/esp-rs/esp-hal/issues/376
[2286]: https://github.com/esp-rs/esp-hal/issues/2286
[5374]: https://github.com/esp-rs/esp-hal/issues/5374
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
