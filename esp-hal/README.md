# esp-hal

[![Crates.io](https://img.shields.io/crates/v/esp-hal?labelColor=1C2C2E&color=C96329&logo=Rust&style=flat-square)](https://crates.io/crates/esp-hal)
[![docs.rs](https://img.shields.io/docsrs/esp-hal?labelColor=1C2C2E&color=C96329&logo=rust&style=flat-square)](https://docs.rs/esp-hal)
![Crates.io](https://img.shields.io/crates/l/esp-hal?labelColor=1C2C2E&style=flat-square)
[![Matrix](https://img.shields.io/matrix/esp-rs:matrix.org?label=join%20matrix&labelColor=1C2C2E&color=BEC5C9&logo=matrix&style=flat-square)](https://matrix.to/#/#esp-rs:matrix.org)

`no_std` HAL implementations for the peripherals which are common among Espressif devices. Implements a number of the traits defined by [embedded-hal](https://github.com/rust-embedded/embedded-hal).

[esp-hal] encompasses the hardware abstraction layer across multiple Espressif devices atop peripheral access crate. To select a concrete chip, correct chip feature has to be used. The current list of chip features is `esp32, esp32c2, esp32c3, esp32c6, esp32h2, esp32p4, esp32s2, esp32s3`.

The package tree is as follows:
- [devices] holds the information about which `peripherals` are supported by the chip HW, `symbols` created by the maintainers to simplify writing drivers for multiple chips, and `efuse` description.
- [ld] contains `linker-scripts` defining the layout of the chip's memory sections, specify the location of various symbols (such as ROM functions, for example).
- [src] is a place, where _all_ peripheral drivers for main (high-power) core are placed. Each peripheral is separated into it's own source file(s).

### Quickstart

After [configuring] the environment and [tooling] you can build and run our [examples]
- make sure, you are in [examples] directory.
- use `cargo +$TOOLCHAIN $CHIP_FEATURE --bin=$EXAMPLE_NAME`, for example, to run a `hello_world` example for `ESP32-C3` using `nightly` toolchain: `cargo +nightly esp32c3 --bin=hello_world`

> [!NOTE]
>
> Please note, that some peripherals miss on some chip HW. Always double-check the [devices] of the chip you want to use or description of example.

If you want to use `esp-hal` as a dependency, using our [template] is highly recommended.

[configuring]: https://esp-rs.github.io/book/installation/index.html
[tooling]: https://esp-rs.github.io/book/tooling/espflash.html
[examples]: https://github.com/esp-rs/esp-hal/tree/main/examples
[template]: https://esp-rs.github.io/book/writing-your-own-application/generate-project/esp-template.html
[esp-hal]: https://github.com/esp-rs/esp-hal/tree/main/esp-hal
[devices]: https://github.com/esp-rs/esp-hal/tree/main/esp-hal/devices
[ld]: https://github.com/esp-rs/esp-hal/tree/main/esp-hal/ld
[src]: https://github.com/esp-rs/esp-hal/tree/main/esp-hal/src

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
