# esp-log

![MSRV](https://img.shields.io/badge/MSRV-1.100.0-blue?labelColor=1C2C2E&style=flat-square)
[![Matrix](https://img.shields.io/matrix/esp-rs:matrix.org?label=join%20matrix&labelColor=1C2C2E&color=BEC5C9&logo=matrix&style=flat-square)](https://matrix.to/#/#esp-rs:matrix.org)

A [`log`] and [`defmt`] backend for Espressif chips. It replaces `esp-println`.

The logger writes to an esp-hal driver that you create after `esp_hal::init`. The `init_uart!` and
`init_usb!` macros create UART0 or USB-Serial-JTAG on the default pins of the chip:

```rust,ignore
let peripherals = esp_hal::init(esp_hal::Config::default());
esp_log::init_uart!(peripherals);
log::info!("Hello");
```

`init_uart_split!` and `init_usb_split!` also return the receiver of the port. `esp_log::init` takes
any other [`embedded_io::Write`], for example a UART on other pins.

Enable the chip feature of this crate, and esp-hal's `unstable` feature: the macros use the unstable
UART and USB-Serial-JTAG drivers.

Enable exactly one of the `log-04` and `defmt` features. With `log-04`, `init` filters records using
the `ESP_LOG` environment variable at build time, for example `ESP_LOG=info,my_crate::net=debug`.
`init_with_level` uses a single maximum level instead.

The crate records the output format in the `.espressif.metadata` section, so espflash picks the
right decoder. defmt output uses rzCOBS framing and can only be decoded by espflash.

Nothing is printed before the logger is installed. Use RTT if you need output before that.

[`log`]: https://github.com/rust-lang/log
[`defmt`]: https://github.com/knurling-rs/defmt
[`embedded_io::Write`]: https://docs.rs/embedded-io/0.7/embedded_io/trait.Write.html

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
