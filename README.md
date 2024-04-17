# esp-hal

![GitHub Actions Workflow Status](https://img.shields.io/github/actions/workflow/status/esp-rs/esp-hal/ci.yml?labelColor=1C2C2E&label=CI&logo=github&style=flat-square)
![GitHub Actions Workflow Status](https://img.shields.io/github/actions/workflow/status/esp-rs/esp-hal/hil.yml?labelColor=1C2C2E&label=HIL&logo=github&style=flat-square&event=merge_group)
![MIT/Apache-2.0 licensed](https://img.shields.io/badge/license-MIT%2FApache--2.0-blue?labelColor=1C2C2E&style=flat-square)
[![Matrix](https://img.shields.io/matrix/esp-rs:matrix.org?labelColor=1C2C2E&label=join%20matrix&color=BEC5C9&logo=matrix&style=flat-square)](https://matrix.to/#/#esp-rs:matrix.org)

Bare-metal (`no_std`) hardware abstraction layer for Espressif devices. Currently supports, to varying degrees, the following devices:

- ESP32 Series: _ESP32_
- ESP32-C Series: _ESP32-C2 (aka ESP8684), ESP32-C3 (aka ESP8685), ESP32-C6_
- ESP32-H Series: _ESP32-H2_
- ESP32-S Series: _ESP32-S2, ESP32-S3_

Additionally provides limited support for programming the low-power RISC-V cores found on the _ESP32-C6_, _ESP32-S2_, and _ESP32-S3_ via the [esp-lp-hal] package.

These packages are all `no_std`; if you are looking for `std` support, please use [esp-idf-svc] instead.

If you have any questions, comments, or concerns, please [open an issue], [start a new discussion], or join us on [Matrix]. For additional information regarding any of the crates in this repository, please refer to the relevant crate's README.

> [!NOTE]
>
> This project is still in the relatively early stages of development, and as such there should be no expectation of API stability. A significant number of peripherals currently have drivers implemented but have varying levels of functionality. For most basic tasks, this should be usable already, however some more advanced or uncommon features may not yet be implemented.

[esp-lp-hal]: https://github.com/esp-rs/esp-hal/tree/main/esp-lp-hal
[esp-idf-svc]: https://github.com/esp-rs/esp-idf-svc
[open an issue]: https://github.com/esp-rs/esp-hal/issues/new
[start a new discussion]: https://github.com/esp-rs/esp-hal/discussions/new
[matrix]: https://matrix.to/#/#esp-rs:matrix.org

## Getting Started

For information relating to the development of Rust applications on ESP devices, please first read [The Rust on ESP Book].

For information about the HAL and how to use it in your own projects, please refer to the [documentation].

[The Rust on ESP Book]: https://esp-rs.github.io/book/
[documentation]: https://docs.esp-rs.org/esp-hal/

## Resources

- [The Rust Programming Language](https://doc.rust-lang.org/book/)
- [The Embedded Rust Book](https://docs.rust-embedded.org/book/index.html)
- [The Embedonomicon](https://docs.rust-embedded.org/embedonomicon/)
- [The Rust on ESP Book](https://esp-rs.github.io/book/)
- [Embedded Rust (no_std) on Espressif](https://esp-rs.github.io/no_std-training/)

## Crates

This repository is home to a number of different packages; for more information regarding a particular package, please refer to its `README.md` and/or documentation.

## Ancillary Crates

There are a number of other crates within the [esp-rs organization] which can be used in conjunction with `esp-hal`:

|      Crate       |                                  Description                                   |
| :--------------: | :----------------------------------------------------------------------------: |
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

## License

Licensed under either of:

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in
the work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without
any additional terms or conditions.
