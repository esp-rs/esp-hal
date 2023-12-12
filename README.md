# esp-hal

![GitHub Workflow Status](https://img.shields.io/github/actions/workflow/status/esp-rs/esp-hal/ci.yml?label=CI&logo=github&style=flat-square)
![MIT/Apache-2.0 licensed](https://img.shields.io/badge/license-MIT%2FApache--2.0-blue?style=flat-square)
[![Matrix](https://img.shields.io/matrix/esp-rs:matrix.org?label=join%20matrix&color=BEC5C9&logo=matrix&style=flat-square)](https://matrix.to/#/#esp-rs:matrix.org)

**H**ardware **A**bstraction **L**ayer crates for the **ESP32**, **ESP32-C2/C3/C6**, **ESP32-H2**, and **ESP32-S2/S3** from Espressif. Additionally provides support for programming the low-power RISC-V cores found on the **ESP32-C6** and **ESP32-S2/S3**.

These HALs are `no_std`; if you are looking for `std` support, please use [esp-idf-hal] instead.

If you have any questions, comments, or concerns, please [open an issue], [start a new discussion], or join us on [Matrix]. For additional information regarding any of the crates in this repository, please refer to the relevant crate's README.

> [!NOTE]
>
> This project is still in the relatively early stages of development, and as such there should be no expectation of API stability. A significant number of peripherals currently have drivers implemented but have varying levels of functionality. For most basic tasks, this should be usable already, however some more advanced or uncommon features may not yet be implemented.

[esp-idf-hal]: https://github.com/esp-rs/esp-idf-hal
[open an issue]: https://github.com/esp-rs/esp-hal/issues/new
[start a new discussion]: https://github.com/esp-rs/esp-hal/discussions/new
[matrix]: https://matrix.to/#/#esp-rs:matrix.org

## Getting Started

For information relating to the development of Rust applications on ESP devices, please first read [The Rust on ESP Book].

For information about the HAL and how to use it in your own projects, please refer to the documentation on [docs.rs] for the relevant chip.

[The Rust on ESP Book]: https://esp-rs.github.io/book/
[docs.rs]: https://docs.rs

## Resources

- [The Rust Programming Language](https://doc.rust-lang.org/book/)
- [The Embedded Rust Book](https://docs.rust-embedded.org/book/index.html)
- [The Embedonomicon](https://docs.rust-embedded.org/embedonomicon/)
- [The Rust on ESP Book](https://esp-rs.github.io/book/)
- [Embedded Rust (no_std) on Espressif](https://esp-rs.github.io/no_std-training/)

## HAL Crates

### High-Power Cores

|     Crate     |                   Documentation                    | Technical Reference Manual |             Target             |
| :-----------: | :------------------------------------------------: | :------------------------: | :----------------------------: |
|  [esp32-hal]  |   [![esp32-hal-docs]](https://docs.rs/esp32-hal)   |          [ESP32]           |    `xtensa-esp32-none-elf`     |
| [esp32c2-hal] | [![esp32c2-hal-docs]](https://docs.rs/esp32c2-hal) |         [ESP32-C2]         | `riscv32imc-unknown-none-elf`  |
| [esp32c3-hal] | [![esp32c3-hal-docs]](https://docs.rs/esp32c3-hal) |         [ESP32-C3]         | `riscv32imc-unknown-none-elf`  |
| [esp32c6-hal] | [![esp32c6-hal-docs]](https://docs.rs/esp32c6-hal) |         [ESP32-C6]         | `riscv32imac-unknown-none-elf` |
| [esp32h2-hal] | [![esp32h2-hal-docs]](https://docs.rs/esp32h2-hal) |         [ESP32-H2]         | `riscv32imac-unknown-none-elf` |
| [esp32s2-hal] | [![esp32s2-hal-docs]](https://docs.rs/esp32s2-hal) |         [ESP32-S2]         |   `xtensa-esp32s2-none-elf`    |
| [esp32s3-hal] | [![esp32s3-hal-docs]](https://docs.rs/esp32s3-hal) |         [ESP32-S3]         |   `xtensa-esp32s3-none-elf`    |

[esp32-hal]: https://github.com/esp-rs/esp-hal/tree/main/esp32-hal
[esp32c2-hal]: https://github.com/esp-rs/esp-hal/tree/main/esp32c2-hal
[esp32c3-hal]: https://github.com/esp-rs/esp-hal/tree/main/esp32c3-hal
[esp32c6-hal]: https://github.com/esp-rs/esp-hal/tree/main/esp32c6-hal
[esp32h2-hal]: https://github.com/esp-rs/esp-hal/tree/main/esp32h2-hal
[esp32s2-hal]: https://github.com/esp-rs/esp-hal/tree/main/esp32s2-hal
[esp32s3-hal]: https://github.com/esp-rs/esp-hal/tree/main/esp32s3-hal
[esp32-hal-docs]: https://img.shields.io/docsrs/esp32-hal?color=C96329&logo=rust&style=flat-square
[esp32c2-hal-docs]: https://img.shields.io/docsrs/esp32c2-hal?color=C96329&logo=rust&style=flat-square
[esp32c3-hal-docs]: https://img.shields.io/docsrs/esp32c3-hal?color=C96329&logo=rust&style=flat-square
[esp32c6-hal-docs]: https://img.shields.io/docsrs/esp32c6-hal?color=C96329&logo=rust&style=flat-square
[esp32h2-hal-docs]: https://img.shields.io/docsrs/esp32h2-hal?color=C96329&logo=rust&style=flat-square
[esp32s2-hal-docs]: https://img.shields.io/docsrs/esp32s2-hal?color=C96329&logo=rust&style=flat-square
[esp32s3-hal-docs]: https://img.shields.io/docsrs/esp32s3-hal?color=C96329&logo=rust&style=flat-square
[esp32]: https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf
[esp32-c2]: https://www.espressif.com/sites/default/files/documentation/esp8684_technical_reference_manual_en.pdf
[esp32-c3]: https://www.espressif.com/sites/default/files/documentation/esp32-c3_technical_reference_manual_en.pdf
[esp32-c6]: https://www.espressif.com/sites/default/files/documentation/esp32-c6_technical_reference_manual_en.pdf
[esp32-h2]: https://www.espressif.com/sites/default/files/documentation/esp32-h2_technical_reference_manual_en.pdf
[esp32-s2]: https://www.espressif.com/sites/default/files/documentation/esp32-s2_technical_reference_manual_en.pdf
[esp32-s3]: https://www.espressif.com/sites/default/files/documentation/esp32-s3_technical_reference_manual_en.pdf

### Low-Power Cores

|        Crate        |       Documentation        |             Target             |
| :-----------------: | :------------------------: | :----------------------------: |
| [esp-ulp-riscv-hal] | N/A (_Not yet ppublished_) | `riscv32imc-unknown-none-elf`  |
|  [esp32c6-lp-hal]   | N/A (_Not yet ppublished_) | `riscv32imac-unknown-none-elf` |

[esp-ulp-riscv-hal]: https://github.com/esp-rs/esp-hal/tree/main/esp-ulp-riscv-hal
[esp32c6-lp-hal]: https://github.com/esp-rs/esp-hal/tree/main/esp32c6-lp-hal

## Ancillary Crates

There are a number of other crates within the [esp-rs organization] which can be used in conjunction with `esp-hal`:

|      Crate       |                                  Description                                   |
| :--------------: | :----------------------------------------------------------------------------: |
|   [esp-alloc]    |                        A simple `no_std` heap allocator                        |
| [esp-backtrace]  |                 Backtrace support for bare-metal applications                  |
| [esp-ieee802154] |          Low-level IEEE802.15.4 driver for the ESP32-C6 and ESP32-H2           |
| [esp-openthread] |           A bare-metal Thread implementation using `esp-ieee802154`            |
|  [esp-println]   |                Provides `print!` and `println!` implementations                |
|  [esp-storage]   | Implementation of [embedded-storage] traits to access unencrypted flash memory |
|    [esp-wifi]    |                       `no_std` Wi-Fi/BLE/ESP-NOW support                       |

[esp-rs organization]: https://github.com/esp-rs
[esp-alloc]: https://github.com/esp-rs/esp-alloc
[esp-backtrace]: https://github.com/esp-rs/esp-backtrace
[esp-ieee802154]: https://github.com/esp-rs/esp-ieee802154
[esp-openthread]: https://github.com/esp-rs/esp-openthread
[esp-println]: https://github.com/esp-rs/esp-println
[esp-storage]: https://github.com/esp-rs/esp-storage
[embedded-storage]: https://github.com/rust-embedded-community/embedded-storage
[esp-wifi]: https://github.com/esp-rs/esp-wifi

## Git Hooks

We provide a simple `pre-commit` hook to verify the formatting of each package prior to committing changes. We _strongly_ encourage use of this git hook.

The hook can be enabled by copying it in to the `.git/hooks/` directory:

```bash
cp pre-commit .git/hooks/pre-commit
```

When using this hook, you can choose to ignore its failure on a per-commit basis by committing with the `--no-verify` flag; however, you will need to be sure that all packages are formatted when submitting a pull request.

## MSRV

The **M**inimum **S**upported **R**ust **V**ersion is `1.67.0` for all packages.

RISC-V is officially supported by the official Rust compiler, however, it should be noted that targeting the Xtensa ISA currently requires the use of the [esp-rs/rust] compiler fork. Our recommend method of installation is [espup].

When targetting the RISC-V architecture _and_ using a `stable` Rust release, it is necessary to set `RUSTC_BOOTSTRAP=1` in order to build successfully; this is not required when using a `nightly` release or when targeting Xtensa.

[esp-rs/rust]: https://github.com/esp-rs/rust
[espup]: https://github.com/esp-rs/espup

## License

Licensed under either of:

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in
the work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without
any additional terms or conditions.
