# esp-hal

An _extremely_ experimental attempt at writing a HAL which targets the **ESP32**, **ESP32-C3**, **ESP32-S2**, and **ESP32-S3**.

**This should not be used for anything at this point in time. This is merely a proof-of-concept.**

The various packages in this repository may or may not build at any given time.

## What is working?

For the **ESP32** and **ESP32-C3**, the `TIMG` and `UART` peripherals have (probably incomplete) implementations which are functional. These packages include examples to demonstrate this.

The **ESP32-C3** has functioning `GPIO` as well, though the implementation may not be complete or correct.

## What is NOT working?

Everything else.

## License

Licensed under either of:

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in
the work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without
any additional terms or conditions.
