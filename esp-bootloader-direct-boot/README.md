# esp-bootloader-direct-boot

[![Crates.io](https://img.shields.io/crates/v/esp-bootloader-direct-boot?labelColor=1C2C2E&color=C96329&logo=Rust&style=flat-square)](https://crates.io/crates/esp-bootloader-direct-boot)
[![docs.rs](https://img.shields.io/docsrs/esp-bootloader-direct-boot?labelColor=1C2C2E&color=C96329&logo=rust&style=flat-square)](https://docs.espressif.com/projects/rust/esp-bootloader-direct-boot/latest/)
![MSRV](https://img.shields.io/badge/MSRV-1.95.0-blue?labelColor=1C2C2E&style=flat-square)
![Crates.io](https://img.shields.io/crates/l/esp-bootloader-direct-boot?labelColor=1C2C2E&style=flat-square)
[![Matrix](https://img.shields.io/matrix/esp-rs:matrix.org?label=join%20matrix&labelColor=1C2C2E&color=BEC5C9&logo=matrix&style=flat-square)](https://matrix.to/#/#esp-rs:matrix.org)

Boots esp-hal applications directly from the ROM, without the ESP-IDF 2nd-stage bootloader, a
partition table or an app image header.

The application uses this crate instead of `esp-bootloader-esp-idf`, enables `esp-hal/unstable`,
links with `-Tlinkall.x` and invokes `esp_bootloader_direct_boot::direct_boot!();` once.

## Chip support

| Chip      | Magic offset |
| --------- | ------------ |
| ESP32-C2  | `0x0`        |
| ESP32-C3  | `0x0`        |
| ESP32-C5  | `0x0`        |
| ESP32-C6  | `0x0`        |
| ESP32-C61 | `0x0`        |
| ESP32-H2  | `0x0`        |
| ESP32-H4  | `0x0`        |
| ESP32-P4  | `0x0`        |
| ESP32-S3  | `0x400`      |
| ESP32-S31 | `0x0`        |

The ESP32 and the ESP32-S2 do not support direct boot. On the ESP32-C3 direct boot needs chip
revision v0.3 or later.

## Flashing

`espflash flash` and `espflash save-image` create an ESP-IDF image, which the ROM won't start
directly. Write the ELF file's loadable sections at their load addresses instead, which are flash
offsets:

```text
rust-objcopy -O binary --gap-fill 0xff target/<target>/release/app app.bin
espflash write-bin 0x0 app.bin
espflash monitor --elf target/<target>/release/app
```

`rust-objcopy` comes with `cargo install cargo-binutils` and `rustup component add llvm-tools`.
`objcopy` starts the image at the lowest load address, so on the ESP32-S3 write it to `0x400`
instead of `0x0`.

## Caveats

- There is no partition table, OTA, app descriptor or secure boot.
- `#[ram(reclaimed)]` is not available. The memory the 2nd-stage bootloader would occupy is
  regular RAM instead.
- Direct boot can be disabled permanently via the `DIS_DIRECT_BOOT` eFuse. It must not be burned.
- When switching from an ESP-IDF image, run `espflash erase-flash` once, so a stale bootloader or
  partition table can't get in the way.
- The image must fit into the flash the linker scripts map: 4 MB on the ESP32-C2 and ESP32-S3 (the
  ESP32-S3 ROM maps no more for direct boot), 8 MB on the ESP32-C3, 16 MB on the ESP32-C5, -C6,
  -C61, -H2 and -H4, and 64 MB on the ESP32-P4 and ESP32-S31.
- On the ESP32-P4, `ESP_HAL_CONFIG_L2_CACHE_SIZE` must not be `0KB`.

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
