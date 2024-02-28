# esp-hal

![GitHub Workflow Status](https://img.shields.io/github/actions/workflow/status/esp-rs/esp-hal/ci.yml?label=CI&logo=github&style=flat-square)
![MIT/Apache-2.0 licensed](https://img.shields.io/badge/license-MIT%2FApache--2.0-blue?style=flat-square)
[![Matrix](https://img.shields.io/matrix/esp-rs:matrix.org?label=join%20matrix&color=BEC5C9&logo=matrix&style=flat-square)](https://matrix.to/#/#esp-rs:matrix.org)

**H**ardware **A**bstraction **L**ayer crates for the **ESP32**, **ESP32-C2/C3/C6**, **ESP32-H2**, **ESP32-P4**, and **ESP32-S2/S3** from Espressif. Additionally provides support for programming the low-power RISC-V cores found on the **ESP32-C6** and **ESP32-S2/S3**.

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

## Crates

### [esp-hal]

Implements number of the traits defined in [embedded-hal](https://github.com/rust-embedded/embedded-hal) for various ESP devices. Full list of currently supported devices with basic information is in the table below.

|   Feature     | Technical Reference Manual |             Target              |    MSRV    |
| :-----------: | :------------------------: | :-----------------------------: | :--------: |
| `esp32`       |           [ESP32]          |     `xtensa-esp32-none-elf`     |   ![esp]   |
| `esp32c2`     |         [ESP32-C2]         |  `riscv32imc-unknown-none-elf`  | ![stable] |
| `esp32c3`     |         [ESP32-C3]         |  `riscv32imc-unknown-none-elf`  | ![stable] |
| `esp32c6`     |         [ESP32-C6]         | `riscv32imac-unknown-none-elf`  | ![stable] |
| `esp32h2`     |         [ESP32-H2]         | `riscv32imac-unknown-none-elf`  | ![stable] |
| `esp32p4`     |         [ESP32-P4]         | `riscv32imafc-unknown-none-elf` | ![stable] |
| `esp32s2`     |         [ESP32-S2]         |    `xtensa-esp32s2-none-elf`    |   ![esp]   |
| `esp32s3`     |         [ESP32-S3]         |    `xtensa-esp32s3-none-elf`    |   ![esp]   |

[esp-hal]: https://github.com/esp-rs/esp-hal/tree/main/esp-hal
[esp]: https://img.shields.io/badge/rustc-esp%201.74+-red.svg
[stable]: https://img.shields.io/badge/rustc-stable%201.76+-red.svg
[esp32]: https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf
[esp32-c2]: https://www.espressif.com/sites/default/files/documentation/esp8684_technical_reference_manual_en.pdf
[esp32-c3]: https://www.espressif.com/sites/default/files/documentation/esp32-c3_technical_reference_manual_en.pdf
[esp32-c6]: https://www.espressif.com/sites/default/files/documentation/esp32-c6_technical_reference_manual_en.pdf
[esp32-h2]: https://www.espressif.com/sites/default/files/documentation/esp32-h2_technical_reference_manual_en.pdf
[esp32-p4]: https://www.espressif.com/sites/default/files/documentation/esp32-p4_technical_reference_manual_en.pdf
[esp32-s2]: https://www.espressif.com/sites/default/files/documentation/esp32-s2_technical_reference_manual_en.pdf
[esp32-s3]: https://www.espressif.com/sites/default/files/documentation/esp32-s3_technical_reference_manual_en.pdf

### [esp-lp-hal]

Implements a number of the traits defined in [embedded-hal](https://github.com/rust-embedded/embedded-hal) for the low-power (lp) RISC-V coprocessors found on the ESP32-C6, ESP32-S2, and ESP32-S3 from Espressif. The main idea is to have code running on lp core and putting the main core into sleep, making it more power-efficient.


|    Crate     |       Documentation       |                            Targets                            |
| :----------: | :-----------------------: | :-----------------------------------------------------------: |
| [esp-lp-hal] | N/A (_Not yet published_) | `riscv32imc-unknown-none-elf`, `riscv32imac-unknown-none-elf` |


[esp-lp-hal]: https://github.com/esp-rs/esp-hal/tree/main/esp-lp-hal

### [esp-hal-procmacros]

Procedural macros for use with the esp-hal family of HAL packages.

[esp-hal-procmacros]: https://github.com/esp-rs/esp-hal/tree/main/esp-hal-procmacros

### [esp-riscv-rt]

Minimal runtime / startup for RISC-V CPUs from Espressif.

[esp-riscv-rt]: https://github.com/esp-rs/esp-hal/tree/main/esp-riscv-rt

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

## License

Licensed under either of:

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in
the work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without
any additional terms or conditions.
