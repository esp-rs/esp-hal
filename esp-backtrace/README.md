# esp-backtrace - backtrace for ESP32 bare-metal

[![Crates.io](https://img.shields.io/crates/v/esp-backtrace?labelColor=1C2C2E&color=C96329&logo=Rust&style=flat-square)](https://crates.io/crates/esp-backtrace)
[![docs.rs](https://img.shields.io/docsrs/esp-backtrace?labelColor=1C2C2E&color=C96329&logo=rust&style=flat-square)](https://docs.rs/esp-backtrace)
![MSRV](https://img.shields.io/badge/MSRV-1.84-blue?labelColor=1C2C2E&style=flat-square)
![Crates.io](https://img.shields.io/crates/l/esp-backtrace?labelColor=1C2C2E&style=flat-square)
[![Matrix](https://img.shields.io/matrix/esp-rs:matrix.org?label=join%20matrix&labelColor=1C2C2E&color=BEC5C9&logo=matrix&style=flat-square)](https://matrix.to/#/#esp-rs:matrix.org)

Supports the ESP32, ESP32-C2/C3/C6, ESP32-H2, ESP32-P4, and ESP32-S2/S3. Optional exception and panic handlers are included, both of which can be enabled via their respective features.

Please note that when targeting a RISC-V device, you **need** to force frame pointers (i.e. `"-C", "force-frame-pointers",` in your `.cargo/config.toml`); this is **not** required for Xtensa.

You can get an array of backtrace addresses (currently limited to 10) via `arch::backtrace()` if
you want to create a backtrace yourself (i.e. not using the panic or exception handler).

When using the panic and/or exception handler make sure to include `use esp_backtrace as _;`.

## Features

| Feature              | Description                                                                                                        |
|----------------------|--------------------------------------------------------------------------------------------------------------------|
| esp32                | Target ESP32                                                                                                       |
| esp32c2              | Target ESP32-C2                                                                                                    |
| esp32c3              | Target ESP32-C3                                                                                                    |
| esp32c6              | Target ESP32-C6                                                                                                    |
| esp32h2              | Target ESP32-H2                                                                                                    |
| esp32p4              | Target ESP32-P4                                                                                                    |
| esp32s2              | Target ESP32-S2                                                                                                    |
| esp32s3              | Target ESP32-S3                                                                                                    |
| panic-handler        | Include a panic handler, will add `esp-println` as a dependency                                                    |
| exception-handler    | Include an exception handler, will add `esp-println` as a dependency                                               |
| println              | Use `esp-println` to print messages                                                                                |
| defmt                | Use `defmt` logging to print messages\* (check [example](https://github.com/playfulFence/backtrace-defmt-example)) |
| colors               | Print messages in red\*                                                                                            |
| halt-cores           | Halt both CPUs on ESP32 / ESP32-S3 instead of doing a `loop {}` in case of a panic or exception                    |
| semihosting          | Call `semihosting::process::abort()` on panic.                                                                     |
| custom-halt          | Invoke the extern function `custom_halt()` instead of doing a `loop {}` in case of a panic or exception            |
| custom-pre-backtrace | Invoke the extern function `custom_pre_backtrace()` before handling a panic or exception                           |

\* _only used for panic and exception handlers_

### `defmt` Feature

Please note that `defmt` does _not_ provide MSRV guarantees with releases, and as such we are not able to make any MSRV guarantees when this feature is enabled. For more information refer to the MSRV section of `defmt`'s README:
https://github.com/knurling-rs/defmt?tab=readme-ov-file#msrv

## Minimum Supported Rust Version (MSRV)

This crate is guaranteed to compile when using the latest stable Rust version at the time of the crate's release. It _might_ compile with older versions, but that may change in any new release, including patches.

## License

Licensed under either of:

- Apache License, Version 2.0 ([LICENSE-APACHE](../LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](../LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in
the work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without
any additional terms or conditions.
