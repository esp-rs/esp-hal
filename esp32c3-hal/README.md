# esp32c3-hal

[![Crates.io](https://img.shields.io/crates/v/esp32c3-hal?labelColor=1C2C2E&color=C96329&logo=Rust&style=flat-square)](https://crates.io/crates/esp32c3-hal)
[![docs.rs](https://img.shields.io/docsrs/esp32c3-hal?labelColor=1C2C2E&color=C96329&logo=rust&style=flat-square)](https://docs.rs/esp32c3-hal)
![Crates.io](https://img.shields.io/crates/l/esp32c3-hal?labelColor=1C2C2E&style=flat-square)
[![Matrix](https://img.shields.io/matrix/esp-rs:matrix.org?label=join%20matrix&labelColor=1C2C2E&color=BEC5C9&logo=matrix&style=flat-square)](https://matrix.to/#/#esp-rs:matrix.org)

`no_std` HAL for the ESP32-C3 from Espressif. Implements a number of the traits defined by [embedded-hal](https://github.com/rust-embedded/embedded-hal).

This device uses the RISC-V ISA, which is officially supported by the Rust compiler via the `riscv32imc-unknown-none-elf` target. Refer to the [Getting Started](#getting-started) section below for more information.

## [Documentation]

[documentation]: https://docs.rs/esp32c3-hal/

## Getting Started

### Installing the Rust Compiler Target

The compilation target for this device is officially supported via the `stable` release channel and can be installed via [rustup](https://rustup.rs/):

```shell
$ rustup target add riscv32imc-unknown-none-elf
```

### Supported boot methods

#### IDF Bootloader

The [IDF second stage bootloader](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/api-guides/startup.html#second-stage-bootloader) is the default bootloader solution.

By default, [espflash](https://github.com/esp-rs/espflash) fetches the required binaries (Bootloader and Partition Table) and flashes them onto the target device together with the Rust-based application firmware image.

#### MCUboot Secure Bootloader

[MCUboot](https://github.com/mcu-tools/mcuboot) is a secure bootloader solution feature-wise equivalent to the [IDF Bootloader](#idf-bootloader).
You may find more information on the documentation pages for MCUboot and the Espressif port:
- https://docs.mcuboot.com/
- https://docs.mcuboot.com/readme-espressif.html

##### Requirements

Booting from MCUboot secure bootloader requires the Rust application image to be built in a [MCUboot-specific image format](https://docs.mcuboot.com/design.html#image-format). You need to install the following dependencies:

```shell
# Required for generating the object file in Intel HEX format
cargo install cargo-binutils
rustup component add llvm-tools-preview

# MCUboot's tool for image signing and key management
pip install imgtool
```

Currently, MCUboot is still not supported as a booting option in [espflash](https://github.com/esp-rs/espflash/issues/267), so you'll need to use the [esptool](https://github.com/espressif/esptool) utility for flashing both the MCUboot bootloader and the Rust application binaries:

```shell
# Serial flasher utility for Espressif chips
pip install esptool
```

Download a prebuilt MCUboot bootloader image for the target device:

```shell
# Prebuilt MCUboot bootloader binary
curl -LO https://github.com/espressif/esp-nuttx-bootloader/releases/download/latest/mcuboot-esp32c3.bin
```

##### Booting the Hello World example from MCUboot

Build the Hello World example with MCUboot support:

```shell
cargo build --release --example hello_world --features mcu-boot
```
Then proceed to generating the application binary and flashing it onto the target device:

```shell
# Generate the object file in Intel HEX format
rust-objcopy -O ihex target/riscv32imc-unknown-none-elf/release/examples/hello_world app.hex

# Generate the application firmware image binary file in MCUboot-format
imgtool sign --pad --align 4 -v 0 -s auto -H 32 --pad-header -S 0x100000 app.hex app.bin

# Flash the application firmware image binary onto the target device
esptool.py -c esp32c3 -p /dev/ttyUSB0 -b 921600 --after no_reset write_flash -fs 4MB -fm dio -ff 40m 0x0 ./mcuboot-esp32c3.bin 0x110000 ./app.bin
```
Once the device is flashed, you may monitor the serial interface (e.g. with `picocom`):

```shell
picocom -b 115200 /dev/ttyUSB0 --imap lfcrlf
```

Reset the board and MCUboot should load the Hello World example:
```shell
ESP-ROM:esp32c3-api1-20210207
Build:Feb  7 2021
rst:0x1 (POWERON),boot:0xc (SPI_FAST_FLASH_BOOT)
SPIWP:0xee
mode:DIO, clock div:2
load:0x3fcd8598,len:0x10cc
load:0x403c8000,len:0x2b90
load:0x403d0000,len:0x1364
entry 0x403c804a
[esp32c3] [INF] Enabling RNG early entropy source...
[esp32c3] [INF] *** Booting MCUboot build v1.8.0-86-g14763b1 ***
[esp32c3] [INF] Primary image: magic=good, swap_type=0x2, copy_done=0x1, image_ok=0x3
[esp32c3] [INF] Scratch: magic=unset, swap_type=0x1, copy_done=0x3, image_ok=0x3
[esp32c3] [INF] Boot source: none
[esp32c3] [INF] Swap type: test
[esp32c3] [INF] Disabling RNG early entropy source...
[esp32c3] [INF] br_image_off = 0x10000
[esp32c3] [INF] ih_hdr_size = 0x20
[esp32c3] [INF] DRAM segment: start=0x3fcd0000, size=0x0, vaddr=0x3fcd0000
[esp32c3] [INF] IRAM segment: start=0x1d00, size=0x170c, vaddr=0x40380000
[esp32c3] [INF] start=0x40380004
Hello world!
Hello world!
Hello world!
```

#### Direct Boot

[Direct Boot](https://github.com/espressif/esp32c3-direct-boot-example#direct-boot-in-esp32-c3) allows an application stored in the External Flash to be executed directly, without being copied into Internal RAM.

##### Booting the Hello World example using Direct Boot

Build the Hello World example with support for Direct Boot:

```shell
cargo build --release --example hello_world --features direct-boot
```

Then proceed to generating the application binary and flashing it onto the target device:

```shell
cargo espflash --release --format direct-boot --features direct-boot --example hello_world --monitor
```

The ROM Bootloader will identify the firmware image built with Direct Boot support and load it appropriately from the External Flash:

```shell
ESP-ROM:esp32c3-api1-20210207
Build:Feb  7 2021
rst:0x1 (POWERON),boot:0xc (SPI_FAST_FLASH_BOOT)
Hello world!
Hello world!
Hello world!
```

## License

Licensed under either of:

- Apache License, Version 2.0 ([LICENSE-APACHE](../LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](../LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in
the work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without
any additional terms or conditions.
