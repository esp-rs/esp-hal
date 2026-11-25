<p align="center">
  <img src="./resources/esp-rs.svg" alt="esp-rs logo" width="100px" />
</p>

<h1 align="center">esp-hal</h1>

<p align="center">
  <img src="https://img.shields.io/github/actions/workflow/status/esp-rs/esp-hal/ci.yml?labelColor=1C2C2E&label=CI&logo=github&style=flat-square" alt="GitHub Actions Workflow Status" />
  <img src="https://img.shields.io/github/actions/workflow/status/esp-rs/esp-hal/hil.yml?labelColor=1C2C2E&label=HIL&logo=github&style=flat-square&event=merge_group" alt="GitHub Actions Workflow Status" />
  <img src="https://img.shields.io/badge/license-MIT%2FApache--2.0-blue?labelColor=1C2C2E&style=flat-square" alt="MIT/Apache-2.0 licensed" />
  <a href="https://matrix.to/#/#esp-rs:matrix.org">
    <img src="https://img.shields.io/matrix/esp-rs:matrix.org?labelColor=1C2C2E&label=join%20matrix&color=BEC5C9&logo=matrix&style=flat-square" alt="Matrix" />
  </a>
</p>

Bare-metal (`no_std`) hardware abstraction layer for Espressif devices. Currently supports the following devices:

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
| ESP32-H4 | [Datasheet][esp32h4-datasheet] [TRM][esp32h4-trm] | `riscv32imafc-unknown-none-elf` |  |
| ESP32-P4 | [Datasheet][esp32p4-datasheet] [TRM][esp32p4-trm] | `riscv32imafc-unknown-none-elf` | Revisions below v3.0 are not supported. |
| ESP32-S2 | [Datasheet][esp32s2-datasheet] [TRM][esp32s2-trm] | `xtensa-esp32s2-none-elf` |  |
| ESP32-S3 | [Datasheet][esp32s3-datasheet] [TRM][esp32s3-trm] | `xtensa-esp32s3-none-elf` |  |
| ESP32-S31 | [Datasheet][esp32s31-datasheet] [TRM][esp32s31-trm] | `riscv32imafc-unknown-none-elf` |  |

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

Additionally provides support for programming the low-power RISC-V cores found on the _ESP32-C6_, _ESP32-S2_, and _ESP32-S3_ via the [esp-lp-hal] package.

For additional information regarding any of the crates in this repository, please refer to the relevant crate's `README.md` file. If you have any questions, comments, or concerns, please [open an issue], or join us on [Matrix].

If you are currently using (or considering using) `esp-hal` in a production environment and have any feedback or require support, please feel free to contact us at <rust.support@espressif.com>.

> [!NOTE]
>
> This repository includes crates that are at various stages of maturity and stability. While many functionalities have already been implemented and are usable for most tasks, certain advanced or less common features may still be under development. Each crate may offer different levels of functionality and guarantees.

[esp-lp-hal]: https://github.com/esp-rs/esp-hal/tree/main/esp-lp-hal
[esp-idf-svc]: https://github.com/esp-rs/esp-idf-svc
[open an issue]: https://github.com/esp-rs/esp-hal/issues/new
[matrix]: https://matrix.to/#/#esp-rs:matrix.org

## Getting Started

For information relating to the development of Rust applications on ESP devices, please first read [The Rust on ESP Book].

For information about the HAL and how to use it in your own projects, please refer to the [documentation].

When browsing the examples, we recommend viewing the tag for the `esp-hal` release you are using to ensure compatibility, e.g. [esp-hal-v1.0.0], as the `main` branch is used for development and APIs may have changed in the meantime.

[The Rust on ESP Book]: https://docs.espressif.com/projects/rust/book/
[documentation]: https://docs.espressif.com/projects/rust/
[esp-hal-v1.0.0]: https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0/examples

## Peripheral support

<!-- The following table is machine generated. Do not edit the comments and the table by hand! -->
<!-- start chip support table -->
For a detailed per-chip support matrix, see the [documentation index](https://docs.espressif.com/projects/rust/esp-hal/main/index.html#peripheral-support).

 * ❌: Not supported
 * ⚠️: Planned overhaul (unstable, expect major breaking changes)
 * 🚧: Unstable (requires the `unstable` feature)
 * ✔️: Stable

### [GPIO](https://docs.espressif.com/projects/rust/esp-hal/main/index.html#gpio)

| Driver         | Status |
| -------------- |:------:|
| GPIO           | ✔️    |
| Dedicated GPIO | 🚧     |
| IOMUX          | 🚧     |
| LP IO          | 🚧     |

### [Serial interfaces](https://docs.espressif.com/projects/rust/esp-hal/main/index.html#serial-interfaces)

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

### [Wireless and networking](https://docs.espressif.com/projects/rust/esp-hal/main/index.html#wireless-and-networking)

| Driver        | Status |
| ------------- |:------:|
| Bluetooth     | 🚧     |
| WIFI          | 🚧     |
| IEEE 802.15.4 | 🚧     |
| Ethernet      | 🚧     |

### [Display and camera](https://docs.espressif.com/projects/rust/esp-hal/main/index.html#display-and-camera)

| Driver            | Status |
| ----------------- |:------:|
| Camera interface  | 🚧     |
| MIPI-CSI          | ❌     |
| MIPI-DSI          | 🚧     |
| RGB display       | 🚧     |
| SPI LCD interface | [❌][5374] [^1] |

### [Signal processing](https://docs.espressif.com/projects/rust/esp-hal/main/index.html#signal-processing)

| Driver                       | Status |
| ---------------------------- |:------:|
| Bit Scrambler                | ❌     |
| Image Signal Processor       | ❌     |
| Pixel Processing Accelerator | 🚧     |
| JPEG Codec                   | 🚧     |
| H.264 encoder                | ❌     |
| Audio Sample Rate Converter  | ❌     |
| CORDIC accelerator           | 🚧     |
| Voice Activity Detection     | ❌     |

### [Analog](https://docs.espressif.com/projects/rust/esp-hal/main/index.html#analog)

| Driver                    | Status |
| ------------------------- |:------:|
| ADC                       | 🚧     |
| Analog Voltage Comparator | ❌     |
| DAC                       | 🚧     |
| Temperature sensor        | 🚧     |
| Touch                     | 🚧     |

### [Timers and motor control](https://docs.espressif.com/projects/rust/esp-hal/main/index.html#timers-and-motor-control)

| Driver          | Status |
| --------------- |:------:|
| LEDC            | [⚠️][2286] [^1] |
| 2D DMA          | 🚧     |
| MCPWM           | 🚧     |
| PCNT            | 🚧     |
| RTC Timekeeping | 🚧     |
| SDM             | 🚧     |
| SYSTIMER        | 🚧     |
| Timers          | 🚧     |

### [Cryptography and security](https://docs.espressif.com/projects/rust/esp-hal/main/index.html#cryptography-and-security)

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

### [Low power](https://docs.espressif.com/projects/rust/esp-hal/main/index.html#low-power)

| Driver           | Status |
| ---------------- |:------:|
| Light/deep sleep | 🚧     |
| ULP (FSM)        | [❌][376] [^1] |
| ULP (RISC-V)     | 🚧     |

### [System](https://docs.espressif.com/projects/rust/esp-hal/main/index.html#system)

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

## Resources

- [The Rust Programming Language](https://doc.rust-lang.org/book/)
- [The Embedded Rust Book](https://docs.rust-embedded.org/book/index.html)
- [The Embedonomicon](https://docs.rust-embedded.org/embedonomicon/)
- [The Rust on ESP Book](https://docs.espressif.com/projects/rust/esp-hal/latest/)
- [Embedded Rust (no_std) on Espressif](https://docs.espressif.com/projects/rust/no_std-training/)

## Support policy

All active development will occur on `main`.

We will only backport fixes to the _latest_ minor release in a major version. For example, this means we will apply patches (bug fixes) to `1.1.x` until `1.2.0` is released, at which point all patches are only backported to the `1.2.x` series of releases.

If you are a user of `unstable` APIs, we will never push breaking changes in a patch release. However, `unstable` changes _will_ make their way into minor releases. This means that as an `unstable` user updating from `1.1.x` to `1.2.x` _may_ introduce breaking changes. If you depend on `unstable`, we recommend defining your esp-hal dependency as follows:

```toml
esp-hal = { version = "~1.1" }
```

Using the [`~` operator](https://doc.rust-lang.org/cargo/reference/specifying-dependencies.html#tilde-requirements) will prevent cargo auto updating to minor versions, allowing you to use `cargo update` without the possibility of breaking your project.

## Release policy

For how releases are prepared, staged as release candidates, and published, see [RELEASING.md](https://github.com/esp-rs/esp-hal/blob/main/documentation/RELEASING.md).

## AI Contribution Policy

We follow the same policy as the official Rust Embedded working group, please review [the policy](https://github.com/rust-embedded/wg/blob/HEAD/CODE_OF_CONDUCT.md#ai-tool-use-policy) before contributing with AI tools.

## Contributing

We have a number of living documents to aid contributing to the project, please give these a read before modifying code:

- [DEVELOPER-GUIDELINES](https://github.com/esp-rs/esp-hal/blob/main/documentation/DEVELOPER-GUIDELINES.md)
- [CONTRIBUTING-GUIDE](https://github.com/esp-rs/esp-hal/blob/main/documentation/CONTRIBUTING.md)

## License

All packages within this repository are licensed under either of:

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

### Contribution notice

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in
the work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without
any additional terms or conditions.
