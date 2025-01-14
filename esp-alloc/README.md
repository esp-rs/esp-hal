# esp-alloc

[![Crates.io](https://img.shields.io/crates/v/esp-alloc?labelColor=1C2C2E&color=C96329&logo=Rust&style=flat-square)](https://crates.io/crates/esp-alloc)
[![docs.rs](https://img.shields.io/docsrs/esp-alloc?labelColor=1C2C2E&color=C96329&logo=rust&style=flat-square)](https://docs.rs/esp-alloc)
![MSRV](https://img.shields.io/badge/MSRV-1.84-blue?labelColor=1C2C2E&style=flat-square)
![Crates.io](https://img.shields.io/crates/l/esp-alloc?labelColor=1C2C2E&style=flat-square)
[![Matrix](https://img.shields.io/matrix/esp-rs:matrix.org?label=join%20matrix&labelColor=1C2C2E&color=BEC5C9&logo=matrix&style=flat-square)](https://matrix.to/#/#esp-rs:matrix.org)

A simple `no_std` heap allocator for RISC-V and Xtensa processors from Espressif. Supports all currently available ESP32 devices.

**NOTE:** using this as your global allocator requires using Rust 1.68 or greater, or the `nightly` release channel.

## License

Licensed under either of:

- Apache License, Version 2.0 ([LICENSE-APACHE](../LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](../LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in
the work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without
any additional terms or conditions.
