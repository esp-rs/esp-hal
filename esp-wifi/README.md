# esp-wifi

A WiFi, BLE and ESP-NOW driver for Espressif microcontrollers.

## Current support

If a cell contains am em dash (&mdash;) this means that the particular feature is not present for a chip. A check mark (✓) means that some driver implementation exists. A Tilde (&tilde;) means its implemented but buggy. An empty cell means that the feature is present in the chip but not implemented yet.

|          | [Wifi](https://github.com/esp-rs/esp-wifi/issues/94) | [BLE](https://github.com/esp-rs/esp-wifi/issues/93) | [Coex](https://github.com/esp-rs/esp-wifi/issues/92) | ESP-NOW |
| :------: | :--------------------------------------------------: | :-------------------------------------------------: | :--------------------------------------------------: | :-----: |
|  ESP32   |                          ✓                           |                          ✓                          |                       &tilde;                        |    ✓    |
| ESP32-S2 |                          ✓                           |                       &mdash;                       |                       &mdash;                        |    ✓    |
| ESP32-S3 |                          ✓                           |                          ✓                          |                          ✓                           |    ✓    |
| ESP32-C3 |                          ✓                           |                          ✓                          |                          ✓                           |    ✓    |
| ESP32-C2 |                          ✓                           |                          ✓                          |                          ✓                           |    ✓    |
| ESP32-C6 |                          ✓                           |                          ✓                          |                          ✓                           |    ✓    |

Minimum supported Rust compiler version: 1.72.0.0

## Usage

### Importing

Ensure that the right features are enabled for your chip. See [Examples] for more examples.

```toml
[dependencies.esp-wifi]
# A supported chip needs to be specified, as well as specific use-case features
features = ["esp32s3", "wifi", "esp-now"]
```

### Link configuration

Make sure to include the rom functions for your target:

```toml
# .cargo/config.toml
rustflags = [
    "-C", "link-arg=-Tlinkall.x",
    "-C", "link-arg=-Trom_functions.x",
]
```
At time of writing, you will already have the linkall flag if you used `cargo generate`. Generating from a template does not include the `rom_functions` flag.


### Optimization Level

It is necessary to build with optimization level 2 or 3 since otherwise it might not even be able to connect or advertise.

To make it work also for your debug builds add this to your `Cargo.toml`

```toml
[profile.dev.package.esp-wifi]
opt-level = 3
```

### Xtensa considerations

Within this crate, `CCOMPARE0` CPU timer is used for timing, ensure that in your application you are not using this CPU timer.

## USB-SERIAL-JTAG

When using USB-SERIAL-JTAG you have to activate the feature `phy-enable-usb`.

Don't use this feature if your are _not_ using USB-SERIAL-JTAG since it might reduce WiFi performance.

## Features

| Feature        | Meaning                                                                                              |
| -------------- | ---------------------------------------------------------------------------------------------------- |
| wifi-logs      | logs the WiFi logs from the driver at log level info                                                 |
| dump-packets   | dumps packet info at log level info                                                                  |
| smoltcp        | Provide implementations of `smoltcp` traits                                                          |
| utils          | Provide utilities for smoltcp initialization; adds `smoltcp` dependency                              |
| ble            | Enable BLE support                                                                                   |
| wifi           | Enable WiFi support                                                                                  |
| esp-now        | Enable esp-now support                                                                               |
| coex           | Enable coex support                                                                                  |
| ipv4           | IPv4 support: includes `utils` feature                                                               |
| ipv6           | IPv6 support: includes `utils` feature                                                               |
| tcp            | TCP socket support: includes `ipv4` feature                                                          |
| udp            | UDP socket support: includes `ipv4` feature                                                          |
| igmp           | IGMP (multicast) support: includes `ipv4` feature                                                    |
| dns            | DNS support: includes `udp` feature                                                                  |
| dhcpv4         | DHCPv4 support, both creating sockets and autoconfiguring network settings: includes `utils` feature |
| phy-enable-usb | See _USB-SERIAL-JTAG_ below                                                                          |
| ps-min-modem   | Enable minimum modem sleep. Only for STA mode                                                        |
| ps-max-modem   | Enable maximum modem sleep. Only for STA mode                                                        |
| log            | Route log output to the `log` crate                                                                  |
| defmt          | Add `defmt::Format` implementation                                                                   |

When using the `dump-packets` feature you can use the extcap in `extras/esp-wifishark` to analyze the frames in Wireshark.
For more information see [extras/esp-wifishark/README.md](../extras/esp-wifishark/README.md)

## Tuning

The defaults used by `esp-wifi` and the examples are rather conservative. It is possible to change a few of the important settings.

See [Tuning](https://github.com/esp-rs/esp-wifi/blob/main/esp-wifi/docs/tuning.md) for details

## Examples

See [Examples] for details.

[Examples]: https://github.com/esp-rs/esp-wifi/blob/main/esp-wifi/docs/examples.md

## Missing / To be done

- Make CoEx work on ESP32 (it kind of works when commenting out setting the country in wifi_start, probably some mis-compilation since it then crashes in a totally different code path)
- Support for non-open SoftAP

## Directory Structure

- `src/timer/`: systimer code used for timing and task switching
- `src/preemt/`: a bare minimum RISCV and Xtensa round-robin task scheduler
- `src/compat/`: code needed to emulate enough of an (RT)OS to use the driver
  - `common.rs`: basics like semaphores and recursive mutexes
  - `timer_compat.rs`: code to emulate timer related functionality
- `examples/*.rs`: examples

## Driver version

This uses the WiFi drivers from https://github.com/esp-rs/esp-wireless-drivers-3rdparty

v5.1-rc2-4-gc570f67461 commit c570f674610479fc5e070c8db6d181b73ddf60a8

https://github.com/esp-rs/esp-wireless-drivers-3rdparty/ (commit 976e9cc6c0725e8325a7e3a362d113559238c45c)

## License

Licensed under either of:

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in
the work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without
any additional terms or conditions.
