# esp-alloc

![GitHub Workflow Status](https://img.shields.io/github/actions/workflow/status/esp-rs/esp-alloc/ci.yml?label=CI&logo=github&style=flat-square)
[![Crates.io](https://img.shields.io/crates/v/esp-alloc?color=C96329&logo=Rust&style=flat-square)](https://crates.io/crates/esp-alloc)
[![docs.rs](https://img.shields.io/docsrs/esp-alloc?color=C96329&logo=rust&style=flat-square)](https://docs.rs/esp-alloc)
![MSRV](https://img.shields.io/badge/MSRV-1.68-blue?style=flat-square)
![Crates.io](https://img.shields.io/crates/l/esp-alloc?style=flat-square)

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
