# esp32c6-hal

[![Crates.io](https://img.shields.io/crates/v/esp32c6-hal?labelColor=1C2C2E&color=C96329&logo=Rust&style=flat-square)](https://crates.io/crates/esp32c6-hal)
[![docs.rs](https://img.shields.io/docsrs/esp32c6-hal?labelColor=1C2C2E&color=C96329&logo=rust&style=flat-square)](https://docs.rs/esp32c6-hal)
![Crates.io](https://img.shields.io/crates/l/esp32c6-hal?labelColor=1C2C2E&style=flat-square)
[![Matrix](https://img.shields.io/matrix/esp-rs:matrix.org?label=join%20matrix&labelColor=1C2C2E&color=BEC5C9&logo=matrix&style=flat-square)](https://matrix.to/#/#esp-rs:matrix.org)

`no_std` HAL for the ESP32-C6 from Espressif. Implements a number of the traits defined by [embedded-hal](https://github.com/rust-embedded/embedded-hal).

This device uses the RISC-V ISA, which is officially supported by the Rust compiler via the `riscv32imac-unknown-none-elf` target. Refer to the [Getting Started](#getting-started) section below for more information.

## [Documentation]

[documentation]: https://docs.rs/esp32c6-hal/

## Getting Started

### Installing the Rust Compiler Target

The compilation target for this device is officially supported via the `stable` release channel and can be installed via [rustup](https://rustup.rs/):

```shell
$ rustup target add riscv32imac-unknown-none-elf
```

### Supported boot methods

#### IDF Bootloader

The [IDF second stage bootloader](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c6/api-guides/startup.html#second-stage-bootloader) is the default bootloader solution.

By default, [espflash](https://github.com/esp-rs/espflash) fetches the required binaries (Bootloader and Partition Table) and flashes them onto the target device together with the Rust-based application firmware image.

#### Direct Boot

[Direct Boot](https://github.com/espressif/esp32c3-direct-boot-example#direct-boot-in-esp32-c3) allows an application stored in the External Flash to be executed directly, without being copied into Internal RAM.

##### Booting the Hello World example using Direct Boot

Build the Hello World example with support for Direct Boot:

```shell
cargo build --release --example hello_world --features direct-boot
```

Then proceed to generating the application binary and flashing it onto the target device:

```shell
cargo espflash --release --format direct-boot --features direct-boot --example hello_world --monitor
```

The ROM Bootloader will identify the firmware image built with Direct Boot support and load it appropriately from the External Flash:

```shell
ESP-ROM:esp32c6-20220919
Build:Sep 19 2022
rst:0x1 (POWERON),boot:0x6e (SPI_FAST_FLASH_BOOT)
Hello world!
Hello world!
Hello world!
```

## License

Licensed under either of:

- Apache License, Version 2.0 ([LICENSE-APACHE](../LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](../LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in
the work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without
any additional terms or conditions.
