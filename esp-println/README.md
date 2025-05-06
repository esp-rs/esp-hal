# esp-println

[![Crates.io](https://img.shields.io/crates/v/esp-println?labelColor=1C2C2E&color=C96329&logo=Rust&style=flat-square)](https://crates.io/crates/esp-println)
[![docs.rs](https://img.shields.io/docsrs/esp-println?labelColor=1C2C2E&color=C96329&logo=rust&style=flat-square)](https://docs.espressif.com/projects/rust/esp-println/latest/)
![MSRV](https://img.shields.io/badge/MSRV-1.84-blue?labelColor=1C2C2E&style=flat-square)
![Crates.io](https://img.shields.io/crates/l/esp-println?labelColor=1C2C2E&style=flat-square)
[![Matrix](https://img.shields.io/matrix/esp-rs:matrix.org?label=join%20matrix&labelColor=1C2C2E&color=BEC5C9&logo=matrix&style=flat-square)](https://matrix.to/#/#esp-rs:matrix.org)

A library that provides `print!`, `println!`, `dbg!` implementations and
logging capabilities for Espressif devices.

- Supports all Espressif ESP32 family devices.
- Supports different communication methods:
  - UART (Default)
  - JTAG-Serial (Only available in ESP32-C3, ESP32-C6, ESP32-H2, ESP32-S3)
  - No-op: Turns printing into a no-op
- Supports [`defmt`] backend

# Usage

```toml
esp-println = { version = "0.11.0", features = ["esp32c2"] }
```

or `cargo add esp-println --features esp32c2`
It's important to specify your target device as feature.

Then in your program:

```rust
use esp_println::println;
```

You can now `println!("Hello world")` as usual.

## Logging

With the feature `log-04` activated, and version 0.4 of the `log` crate added to your dependencies,
you can initialize a simple logger like this:

```rust
init_logger(log::LevelFilter::Info);
```

There is a default feature `colors` which enables colored log output.

Additionally, you can use

```rust
init_logger_from_env();
```

In this case the following environment variables are used:

- `ESP_LOG` log messages you want to show, similar to `RUST_LOG`. RegEx is not supported. e.g. `warn,test::foo=info,test::foo::bar=debug`

If this simple logger implementation isn't sufficient for your needs, you can implement your own logger on top of `esp-println`. See [Implementing a Logger section log documentaion]

## `defmt`

Using the `defmt-espflash` feature, `esp-println` will install a `defmt` global logger. The logger will
output to the same data stream as `println!()`, and adds framing bytes so it can be used even with
other, non-`defmt` output. Using the `defmt-espflash` feature automatically uses the [rzCOBS] encoding and does
not allow changing the encoding.

Follow the [`defmt` book's setup instructions] on how to
set up `defmt`. Remember, the global logger is already installed for you by `esp-println`!

Please note that `defmt` does _not_ provide MSRV guarantees with releases, and as such we are not able to make any MSRV guarantees when this feature is enabled. For more information refer to the MSRV section of `defmt`'s README:
https://github.com/knurling-rs/defmt?tab=readme-ov-file#msrv

[`defmt`]: https://github.com/knurling-rs/defmt
[`log` crate]: https://github.com/rust-lang/log
[rzCOBS]: https://github.com/Dirbaio/rzcobs
[`espflash`]: https://github.com/esp-rs/espflash
[Implementing a Logger section log documentaion]: https://docs.rs/log/0.4.17/log/#implementing-a-logger
[`defmt` book's setup instructions]: https://defmt.ferrous-systems.com/setup

# Troubleshooting linker errors

If you experience linker errors, make sure you have _some_ reference to `esp_println` in your code.
If you don't use `esp_println` directly, you'll need to add e.g. `use esp_println as _;` to your
import statements. This ensures that the global logger will not be removed by the compiler.

## Minimum Supported Rust Version (MSRV)

This crate is guaranteed to compile when using the latest stable Rust version at the time of the crate's release. It _might_ compile with older versions, but that may change in any new release, including patches.

# License

Licensed under either of:

- Apache License, Version 2.0 ([LICENSE-APACHE](../LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](../LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

# Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in
the work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without
any additional terms or conditions.
