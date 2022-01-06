# esp-hal-common

[![Crates.io](https://img.shields.io/crates/v/esp-hal-common.svg)](https://crates.io/crates/esp-hal-common)
[![Docs](https://docs.rs/esp-hal-common/badge.svg)](https://docs.rs/esp-hal-common/)
![Crates.io](https://img.shields.io/crates/l/esp-hal-common)

`no_std` HAL implementations for the peripherals which are common among Espressif devices. Implements a number of the traits defined by [embedded-hal](https://github.com/rust-embedded/embedded-hal).

This crate should not be used directly; you should use one of the device-specific HAL crates instead:

- [esp32-hal](../esp32-hal/README.md)
- [esp32c3-hal](../esp32c3-hal/README.md)
- [esp32s2-hal](../esp32s2-hal/README.md)
- [esp32s3-hal](../esp32s3-hal/README.md)

## [Documentation]

[documentation]: https://docs.rs/esp-hal-common/

## License

Licensed under either of:

- Apache License, Version 2.0 ([LICENSE-APACHE](../LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](../LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in
the work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without
any additional terms or conditions.
