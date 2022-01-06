# esp32c3-hal

[![Crates.io](https://img.shields.io/crates/v/esp32c3-hal.svg)](https://crates.io/crates/esp32c3-hal)
[![Docs](https://docs.rs/esp32c3-hal/badge.svg)](https://docs.rs/esp32c3-hal/)
![Crates.io](https://img.shields.io/crates/l/esp32c3-hal)

`no_std` HAL for the ESP32-C3 from Espressif. Implements a number of the traits defined by [embedded-hal](https://github.com/rust-embedded/embedded-hal).

This device uses the RISC-V ISA, which is officially supported by the Rust compiler via the `riscv32imc-unknown-none-elf` target. Refer to the [Getting Stared](#getting-started) section below for more information.

## [Documentation]

[documentation]: https://docs.rs/esp32c3-hal/

## Getting Started

### Installing the Rust Compiler Target

The compilation target for this device is officially supported via the `stable` release channel and can be installed via [rustup](https://rustup.rs/):

```shell
$ rustup target add riscv32imc-unknown-none-elf
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
