# esp-hal-procmacros

[![Crates.io](https://img.shields.io/crates/v/esp-hal-procmacros?labelColor=1C2C2E&color=C96329&logo=Rust&style=flat-square)](https://crates.io/crates/esp-hal-procmacros)
[![docs.rs](https://img.shields.io/docsrs/esp-hal-procmacros?labelColor=1C2C2E&color=C96329&logo=rust&style=flat-square)](https://docs.rs/esp-hal-procmacros)
![Crates.io](https://img.shields.io/crates/l/esp-hal-procmacros?labelColor=1C2C2E&style=flat-square)
[![Matrix](https://img.shields.io/matrix/esp-rs:matrix.org?label=join%20matrix&labelColor=1C2C2E&color=BEC5C9&logo=matrix&style=flat-square)](https://matrix.to/#/#esp-rs:matrix.org)

Procedural macros for use with the `esp-hal` family of HAL packages.

Provides macros for:

- Placing statics and functions into RAM
- Marking interrupt handlers
- Automatically creating an `embassy` executor instance and spawning the defined entry point

## [Documentation]

[documentation]: https://docs.rs/esp-hal-procmacros/

## License

Licensed under either of:

- Apache License, Version 2.0 ([LICENSE-APACHE](../LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](../LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in
the work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without
any additional terms or conditions.
