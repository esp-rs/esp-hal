# `xtensa-lx-rt`

[![Crates.io](https://img.shields.io/crates/v/xtensa-lx-rt?labelColor=1C2C2E&color=C96329&logo=Rust&style=flat-square)](https://crates.io/crates/xtensa-lx-rt)
[![docs.rs](https://img.shields.io/docsrs/xtensa-lx-rt?labelColor=1C2C2E&color=C96329&logo=rust&style=flat-square)](https://docs.rs/xtensa-lx-rt)
![MSRV](https://img.shields.io/badge/MSRV-1.84-blue?labelColor=1C2C2E&style=flat-square)
![Crates.io](https://img.shields.io/crates/l/xtensa-lx-rt?labelColor=1C2C2E&style=flat-square)
[![Matrix](https://img.shields.io/matrix/esp-rs:matrix.org?label=join%20matrix&labelColor=1C2C2E&color=BEC5C9&logo=matrix&style=flat-square)](https://matrix.to/#/#esp-rs:matrix.org)

Minimal runtime/startup for Xtensa LX processors. This crate currently supports the following CPU's:

| Feature   | Supported CPUs   |
| --------- | ---------------- |
| `esp32`   | ESP32 (_LX6_)    |
| `esp32s2` | ESP32-S2 (_LX7_) |
| `esp32s3` | ESP32-S3 (_LX7_) |

## I get linker errors when I build for debug

Xtensa only provides a small code space for exceptions to fit inside, when building an unoptimized build the code size of a exception handler may exceed that size, causing a linker error. To fix this, you should always optimize this crate, even in debug builds. Adding the following to your projects `Cargo.toml` should do the trick.

```toml
[profile.dev.package.xtensa-lx-rt]
opt-level = 'z'
```

## Minimum Supported Rust Version (MSRV)

This crate is guaranteed to compile when using the latest stable Rust version at the time of the crate's release. It _might_ compile with older versions, but that may change in any new release, including patches.

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](../LICENSE-APACHE) or
  http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](../LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in the
work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any
additional terms or conditions.
