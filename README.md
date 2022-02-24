# esp-hal

![GitHub Workflow Status](https://img.shields.io/github/workflow/status/esp-rs/esp-hal/CI?label=CI&logo=github&style=flat-square)
![MIT/Apache-2.0 licensed](https://img.shields.io/badge/license-MIT%2FApache--2.0-blue?style=flat-square)
[![Matrix](https://img.shields.io/matrix/esp-rs:matrix.org?label=join%20matrix&color=BEC5C9&logo=matrix&style=flat-square)](https://matrix.to/#/#esp-rs:matrix.org)

An _extremely_ experimental attempt at writing a HAL which targets the **ESP32**, **ESP32-C3**, **ESP32-S2**, and **ESP32-S3**.

If you are interested in `no_std` development for ESP devices, we encourage you to contribute whatever you can to this repository!

### IMPORTANT:

> **These crates should not be used for anything other than experimentation at this point in time, this is merely a proof-of-concept.**
>
> **The `esp32-hal` package in this repository is NOT the same as the one published on [crates.io]. The published crate can be found in the [esp-rs/esp32-hal] repository. Once feature parity has been reached, the hope is for the version in this repository to supersede the old one.**
>
> **Until the first releases are published, there should be no expectation of API stability.**

Please make sure you understand all the points above, and use these packages at your own risk. When the packages in this repository are ready for general use this README will be updated to indicate such.

For additional updates, please follow along in the [esp-rs channel] on Matrix.

[crates.io]: https://crates.io/crates/esp32-hal
[esp-rs/esp32-hal]: https://github.com/esp-rs/esp32-hal
[esp-rs channel]: https://matrix.to/#/#esp-rs:matrix.org

## What is working?

Presently the `GPIO`, `TIMG`, and `UART` peripherals have drivers implemented for all supported chips. In addition the `DelayUs` and `DelayMs` traits from [embedded-hal] have been implemented, using the `SYSTIMER` peripheral for the **ESP32-C3** and the internal Xtensa timers for the remaining chips.

For a complete list of which peripheral drivers are implemented for which chips, please refer to [this issue].

[embedded-hal]: https://github.com/rust-embedded/embedded-hal
[this issue]: https://github.com/esp-rs/esp-hal/issues/19

## License

Licensed under either of:

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in
the work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without
any additional terms or conditions.
