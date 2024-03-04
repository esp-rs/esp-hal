# esp-hal

[![Crates.io](https://img.shields.io/crates/v/esp-hal?labelColor=1C2C2E&color=C96329&logo=Rust&style=flat-square)](https://crates.io/crates/esp-hal)
[![docs.rs](https://img.shields.io/docsrs/esp-hal?labelColor=1C2C2E&color=C96329&logo=rust&style=flat-square)](https://docs.rs/esp-hal)
![Crates.io](https://img.shields.io/crates/l/esp-hal?labelColor=1C2C2E&style=flat-square)
[![Matrix](https://img.shields.io/matrix/esp-rs:matrix.org?label=join%20matrix&labelColor=1C2C2E&color=BEC5C9&logo=matrix&style=flat-square)](https://matrix.to/#/#esp-rs:matrix.org)

### Quickstart

[esp-hal] encompasses the hardware abstraction layer across multiple Espressif devices atop peripheral access crate. Implements a number of the traits defined by [embedded-hal](https://github.com/rust-embedded/embedded-hal) for both, `blocking` and `async` APIs.
To select a concrete chip, correct chip feature has to be used. The current list of chip features is `esp32`, `esp32c2`, `esp32c3`, `esp32c6`, `esp32h2`, `esp32p4`, `esp32s2`, and `esp32s3`.

By default, the code uses `blocking` API. If you want to use [Embassy] `async` API, the `async` feature has to be used.

`esp-hal` is closely associated with different crates as well. [esp-lp-hal] is HAL for the low-power RISC-V coprocessors found on the ESP32-C6, ESP32-S2, and ESP32-S3 and is part of the [ESP-HAL] package. Another crates, supporting not only WiFi and BLE that are not part of the [ESP-HAL] package could be found [here](https://github.com/esp-rs/esp-hal?tab=readme-ov-file#ancillary-crates).

The package tree is as follows:
- [devices] holds the information about which `peripherals` are supported by the chip HW, `symbols` created by the maintainers to simplify writing drivers for multiple chips, and `efuse` description.
- [ld] contains `linker-scripts` defining the layout of the chip's memory sections, specify the location of various symbols (such as ROM functions, for example).
- [src] is a place, where _all_ peripheral drivers for main (high-power) core are placed. Each peripheral is separated into its own source file(s).

## Usage

Before using this `esp-hal`, be sure you [configured] the environment and [tooling] correctly.
We **highly recommend** to use our [esp-template] that does all the basic configuration for you. If you want to try to run examples, the [xtask] should be used:

`cargo xtask run-example examples esp32c3 hello_world`

> [!NOTE]
>
> Please note, not all chips support the same set of peripherals.

[Embassy]: https://github.com/embassy-rs/embassy
[esp-lp-hal]: https://github.com/esp-rs/esp-hal/tree/main/esp-lp-hal
[ESP-HAL]: https://github.com/esp-rs/esp-hal
[configured]: https://esp-rs.github.io/book/installation/index.html
[tooling]: https://esp-rs.github.io/book/tooling/espflash.html
[examples]: https://github.com/esp-rs/esp-hal/tree/main/examples
[esp-template]: https://github.com/esp-rs/esp-template
[template]: https://esp-rs.github.io/book/writing-your-own-application/generate-project/esp-template.html
[esp-hal]: https://github.com/esp-rs/esp-hal/tree/main/esp-hal
[devices]: https://github.com/esp-rs/esp-hal/tree/main/esp-hal/devices
[ld]: https://github.com/esp-rs/esp-hal/tree/main/esp-hal/ld
[src]: https://github.com/esp-rs/esp-hal/tree/main/esp-hal/src
[xtask]: https://github.com/esp-rs/esp-hal/tree/main/xtask

## [Documentation]

[documentation]: https://docs.rs/esp-hal/

## License

Licensed under either of:

- Apache License, Version 2.0 ([LICENSE-APACHE](../LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](../LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in
the work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without
any additional terms or conditions.
