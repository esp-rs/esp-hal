# embedded-log

[![Crates.io](https://img.shields.io/crates/v/embedded-log?labelColor=1C2C2E&color=C96329&logo=Rust&style=flat-square)](https://crates.io/crates/embedded-log)
[![docs.rs](https://img.shields.io/docsrs/embedded-log?labelColor=1C2C2E&color=C96329&logo=rust&style=flat-square)](https://docs.espressif.com/projects/rust/embedded-log/latest/)
![MSRV](https://img.shields.io/badge/MSRV-1.95.0-blue?labelColor=1C2C2E&style=flat-square)
![Crates.io](https://img.shields.io/crates/l/embedded-log?labelColor=1C2C2E&style=flat-square)
[![Matrix](https://img.shields.io/matrix/esp-rs:matrix.org?label=join%20matrix&labelColor=1C2C2E&color=BEC5C9&logo=matrix&style=flat-square)](https://matrix.to/#/#esp-rs:matrix.org)

A [`log`] and [`defmt`] backend that writes to any [`embedded_io::Write`]. It replaces
`esp-println`.

The crate does not depend on a HAL. You construct the transport, for example esp-hal's `Uart` or
`UsbSerialJtag`, and hand it to the logger:

```rust,ignore
let peripherals = esp_hal::init(esp_hal::Config::default());
let usj = UsbSerialJtag::new(peripherals.USB_DEVICE, Default::default()).unwrap();
static WRITER: StaticCell<UsbSerialJtag<'static, Blocking>> = StaticCell::new();
embedded_log::init(WRITER.init(usj));
log::info!("Hello");
```

Using esp-hal's `Uart` or `UsbSerialJtag` as the transport requires esp-hal's `unstable` feature.

Enable exactly one of the `log-04` and `defmt` features. With `log-04`, `init` filters records using
the `ESP_LOG` environment variable at build time, for example `ESP_LOG=info,my_crate::net=debug`.
`init_with_level` uses a single maximum level instead.

The crate records the output format in the `.espressif.metadata` section, so espflash picks the
right decoder. defmt output uses rzCOBS framing and can only be decoded by espflash.

Nothing is printed before `init`. Use RTT if you need output before the transport exists.

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
