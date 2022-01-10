# esp-hal

[![CI](https://github.com/jessebraham/esp-hal/actions/workflows/ci.yml/badge.svg)](https://github.com/jessebraham/esp-hal/actions/workflows/ci.yml)

An _extremely_ experimental attempt at writing a HAL which targets the **ESP32**, **ESP32-C3**, **ESP32-S2**, and **ESP32-S3**.

**This should not be used for anything other than experimentation at this point in time, this is merely a proof-of-concept.**

The various packages in this repository may or may not build at any given time. Until the first releases are published there should be no expectation of API stability.

## What is working?

For the **ESP32** and **ESP32-C3**, the `GPIO`, `TIMG` and `UART` peripherals have (probably incomplete) implementations which are nonetheless functional. Both of the aforementioned chips also have implmented the `DelayUs` and `DelayMs` traits from `embedded-hal`. These packages include examples to demonstrate these peripherals.

## What is NOT working?

Everything else.

### Notes on the ESP32-S2 and ESP32-S3

At this time, there are two major issues blocking progress on the **ESP32-S2** and **ESP32-S3**:

- The lack of runtime support via [riscv-rt](https://github.com/rust-embedded/riscv-rt)
- The omission of linker scripts, required for actually building the binaries

Once these issues have been resolved, progress can resume on the `esp32s2-hal` and `esp32s3-hal` crates.

## License

Licensed under either of:

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in
the work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without
any additional terms or conditions.
