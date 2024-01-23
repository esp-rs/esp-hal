# esp32p4-hal

[![Crates.io](https://img.shields.io/crates/v/esp32p4-hal?labelColor=1C2C2E&color=C96329&logo=Rust&style=flat-square)](https://crates.io/crates/esp32p4-hal)
[![docs.rs](https://img.shields.io/docsrs/esp32p4-hal?labelColor=1C2C2E&color=C96329&logo=rust&style=flat-square)](https://docs.rs/esp32p4-hal)
![Crates.io](https://img.shields.io/crates/l/esp32p4-hal?labelColor=1C2C2E&style=flat-square)
[![Matrix](https://img.shields.io/matrix/esp-rs:matrix.org?label=join%20matrix&labelColor=1C2C2E&color=BEC5C9&logo=matrix&style=flat-square)](https://matrix.to/#/#esp-rs:matrix.org)

`no_std` HAL for the ESP32-P4 from Espressif.

Implements a number of the traits defined in [embedded-hal](https://github.com/rust-embedded/embedded-hal).

This device uses the RISC-V ISA, which is officially supported by the Rust compiler via the `riscv32imafc-unknown-none-elf` target.

Please refer to the documentation for more information.

## [Documentation]

[documentation]: https://docs.rs/esp32p4-hal/

## Resources

- [Datasheet](https://www.espressif.com/sites/default/files/documentation/esp32-p4_datasheet_en.pdf)
- [Technical Reference Manual](https://www.espressif.com/sites/default/files/documentation/esp32-p4_technical_reference_manual_en.pdf)
- [The Rust Programming Language](https://doc.rust-lang.org/book/)
- [The Embedded Rust Book](https://docs.rust-embedded.org/book/index.html)
- [The Rust on ESP Book](https://esp-rs.github.io/book/)

## Getting Started

### Installing the Rust Compiler Target

The compilation target for this device is officially supported by the mainline Rust compiler and can be installed using [rustup](https://rustup.rs/):

```shell
rustup target add riscv32imafc-unknown-none-elf
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
