# esp-hal

![GitHub Workflow Status](https://img.shields.io/github/actions/workflow/status/esp-rs/esp-hal/ci.yml?label=CI&logo=github&style=flat-square)
![MIT/Apache-2.0 licensed](https://img.shields.io/badge/license-MIT%2FApache--2.0-blue?style=flat-square)
[![Matrix](https://img.shields.io/matrix/esp-rs:matrix.org?label=join%20matrix&color=BEC5C9&logo=matrix&style=flat-square)](https://matrix.to/#/#esp-rs:matrix.org)

**H**ardware **A**bstraction **L**ayer crates for the **ESP32**, **ESP32-C2/C3/C6**, **ESP32-H2**, and **ESP32-S2/S3** from Espressif.

These HALs are `no_std`; if you are looking for `std` support, please use [esp-idf-hal] instead.

This project is still in the early stages of development, and as such there should be no expectation of API stability. A significant number of peripherals currently have drivers implemented (you can see a full list [here]) but have varying levels of functionality. For most basic tasks, this should be usable already.

If you have any questions, comments, or concerns, please [open an issue], [start a new discussion], or join us on [Matrix]. For additional information regarding any of the crates in this repository, please refer to the crate's README.

|      Crate       |             Target             | Technical Reference Manual |
| :--------------: | :----------------------------: | :------------------------: |
|   [esp32-hal]    |    `xtensa-esp32-none-elf`     |          [ESP32]           |
|  [esp32c2-hal]   | `riscv32imc-unknown-none-elf`  |         [ESP32-C2]         |
|  [esp32c3-hal]   | `riscv32imc-unknown-none-elf`  |         [ESP32-C3]         |
|  [esp32c6-hal]   | `riscv32imac-unknown-none-elf` |         [ESP32-C6]         |
| [esp32c6-lp-hal] | `riscv32imac-unknown-none-elf` |            N/A             |
|  [esp32h2-hal]   | `riscv32imac-unknown-none-elf` |         [ESP32-H2]         |
|  [esp32s2-hal]   |   `xtensa-esp32s2-none-elf`    |         [ESP32-S2]         |
|  [esp32s3-hal]   |   `xtensa-esp32s3-none-elf`    |         [ESP32-S3]         |

[here]: https://github.com/esp-rs/esp-hal/issues/19
[esp-idf-hal]: https://github.com/esp-rs/esp-idf-hal
[open an issue]: https://github.com/esp-rs/esp-hal/issues/new
[start a new discussion]: https://github.com/esp-rs/esp-hal/discussions/new
[matrix]: https://matrix.to/#/#esp-rs:matrix.org
[esp32-hal]: https://github.com/esp-rs/esp-hal/tree/main/esp32-hal
[esp32c2-hal]: https://github.com/esp-rs/esp-hal/tree/main/esp32c2-hal
[esp32c3-hal]: https://github.com/esp-rs/esp-hal/tree/main/esp32c3-hal
[esp32c6-hal]: https://github.com/esp-rs/esp-hal/tree/main/esp32c6-hal
[esp32c6-lp-hal]: https://github.com/esp-rs/esp-hal/tree/main/esp32c6-lp-hal
[esp32h2-hal]: https://github.com/esp-rs/esp-hal/tree/main/esp32h2-hal
[esp32s2-hal]: https://github.com/esp-rs/esp-hal/tree/main/esp32s2-hal
[esp32s3-hal]: https://github.com/esp-rs/esp-hal/tree/main/esp32s3-hal
[esp32]: https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf
[esp32-c2]: https://www.espressif.com/sites/default/files/documentation/esp8684_technical_reference_manual_en.pdf
[esp32-c3]: https://www.espressif.com/sites/default/files/documentation/esp32-c3_technical_reference_manual_en.pdf
[esp32-c6]: https://www.espressif.com/sites/default/files/documentation/esp32-c6_technical_reference_manual_en.pdf
[esp32-h2]: https://www.espressif.com/sites/default/files/documentation/esp32-h2_technical_reference_manual_en.pdf
[esp32-s2]: https://www.espressif.com/sites/default/files/documentation/esp32-s2_technical_reference_manual_en.pdf
[esp32-s3]: https://www.espressif.com/sites/default/files/documentation/esp32-s3_technical_reference_manual_en.pdf

## Quickstart

If you have not already, we strongly encourage you to read [The Rust on ESP Book] prior to starting.

We recommend using [cargo-generate] and [esp-template] in order to generate a new project. This will allow you to generate a minimal project skeleton with all the required dependencies and configuration ready to go.

You can install [cargo-generate] and generate a new project by running:

```bash
$ cargo install cargo-generate
$ cargo generate -a esp-rs/esp-template
```

For more information on using this template, please refer to [its README].

[the rust on esp book]: https://esp-rs.github.io/book/
[cargo-generate]: https://github.com/cargo-generate/cargo-generate
[esp-template]: https://github.com/esp-rs/esp-template
[its readme]: https://github.com/esp-rs/esp-template/blob/main/README.md

## Ancillary Crates

There are a number of other crates within the [esp-rs organization] which can be used in conjunction with `esp-hal`:

|      Crate       |                                  Description                                   |
| :--------------: | :----------------------------------------------------------------------------: |
|   [esp-alloc]    |                        A simple `no_std` heap allocator                        |
| [esp-backtrace]  |                 Backtrace support for bare-metal applications                  |
| [esp-ieee802154] |          Low-level IEEE802.15.4 driver for the ESP32-C6 and ESP32-H2           |
|  [esp-println]   |                Provides `print!` and `println!` implementations                |
|  [esp-storage]   | Implementation of [embedded-storage] traits to access unencrypted flash memory |
|    [esp-wifi]    |                      `no_std` Wi-Fi/Bluetooth LE support                       |

[esp-rs organization]: https://github.com/esp-rs
[esp-alloc]: https://github.com/esp-rs/esp-alloc
[esp-backtrace]: https://github.com/esp-rs/esp-backtrace
[esp-ieee802154]: https://github.com/esp-rs/esp-ieee802154
[esp-println]: https://github.com/esp-rs/esp-println
[esp-storage]: https://github.com/esp-rs/esp-storage
[embedded-storage]: https://github.com/rust-embedded-community/embedded-storage
[esp-wifi]: https://github.com/esp-rs/esp-wifi

## MSRV

The **M**inimum **S**upported **R**ust **V**ersions are:

- `nightly-2022-09-16` for RISC-V devices (**ESP32-C2**, **ESP32-C3**, **ESP32-C6**, **ESP32-H2**)
  - This corresponds to the date that the `1.65.0` release was branched from `master`
- `1.65.0` for Xtensa devices (**ESP32**, **ESP32-S2**, **ESP32-S3**)
- `1.67.0` for all `async` examples (`embassy_hello_world`, `embassy_wait`, etc.)

Note that targeting the Xtensa ISA currently requires the use of the [esp-rs/rust] compiler fork. Our recommend method of installation is [espup].

RISC-V is officially supported by the official Rust compiler.

[esp-rs/rust]: https://github.com/esp-rs/rust
[espup]: https://github.com/esp-rs/espup

## Git Hooks

We provide a simple `pre-commit` hook to verify the formatting of each package prior to committing changes. We strongly encourage use of this git hook.

The hook can be enabled by placing it in the `.git/hooks/` directory:

```bash
$ cp pre-commit .git/hooks/pre-commit
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
