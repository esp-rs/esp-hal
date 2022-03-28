# esp-hal

![GitHub Workflow Status](https://img.shields.io/github/workflow/status/esp-rs/esp-hal/CI?label=CI&logo=github&style=flat-square)
![MIT/Apache-2.0 licensed](https://img.shields.io/badge/license-MIT%2FApache--2.0-blue?style=flat-square)
[![Matrix](https://img.shields.io/matrix/esp-rs:matrix.org?label=join%20matrix&color=BEC5C9&logo=matrix&style=flat-square)](https://matrix.to/#/#esp-rs:matrix.org)

**H**ardware **A**bstraction **L**ayer crates for the **ESP32**, **ESP32-C3**, **ESP32-S2**, and **ESP32-S3** from Espressif.

This project is still in the early stages of development, and as such there should be no expectation of API stability. Only a small number of peripherals currently have drivers implemented (you can see a full list [here]) and out of those most are still incomplete, albeit functional. These HALs are `no_std`; if you are looking for `std` support please use [esp-idf-hal] instead.

If you have any questions, comments, or concerns please join us on [Matrix]. For additional information regarding any of the crates in the monorepo, please refer to the crate's README.

|     Crate     |                               Target                                | Technical Reference Manual |
| :-----------: | :-----------------------------------------------------------------: | :------------------------: |
|  [esp32-hal]  |                       `xtensa-esp32-none-elf`                       |          [ESP32]           |
| [esp32c3-hal] | `riscv32imc-unknown-none-elf`<br />`riscv32imac-unknown-none-elf`\* |         [ESP32-C3]         |
| [esp32s2-hal] |                      `xtensa-esp32s2-none-elf`                      |         [ESP32-S2]         |
| [esp32s3-hal] |                      `xtensa-esp32s3-none-elf`                      |         [ESP32-S3]         |

_\* via [atomic emulation]_

[here]: https://github.com/esp-rs/esp-hal/issues/19
[esp-idf-hal]: https://github.com/esp-rs/esp-idf-hal
[matrix]: https://matrix.to/#/#esp-rs:matrix.org
[esp32-hal]: https://github.com/esp-rs/esp-hal/tree/main/esp32-hal
[esp32c3-hal]: https://github.com/esp-rs/esp-hal/tree/main/esp32c3-hal
[esp32s2-hal]: https://github.com/esp-rs/esp-hal/tree/main/esp32s2-hal
[esp32s3-hal]: https://github.com/esp-rs/esp-hal/tree/main/esp32s3-hal
[esp32]: https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf
[esp32-c3]: https://www.espressif.com/sites/default/files/documentation/esp32-c3_technical_reference_manual_en.pdf
[esp32-s2]: https://www.espressif.com/sites/default/files/documentation/esp32-s2_technical_reference_manual_en.pdf
[esp32-s3]: https://www.espressif.com/sites/default/files/documentation/esp32-s3_technical_reference_manual_en.pdf
[atomic emulation]: https://github.com/esp-rs/riscv-atomic-emulation-trap

## MSRV

The **M**inimum **S**upported **R**ust **V**ersions are:

- `1.56.0` for RISC-V devices (**ESP32-C3**)
- `stable - 1` for Xtensa devices (**ESP32**, **ESP32-S2**, **ESP32-S3**)

Note that targeting the Xtensa ISA requires the use of the [esp-rs/rust] compiler fork, whereas RISC-V is officially supported by the official Rust compiler.

[esp-rs/rust]: https://github.com/esp-rs/rust

## License

Licensed under either of:

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in
the work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without
any additional terms or conditions.
