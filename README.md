<p align="center">
  <img src="./resources/esp-rs.svg" alt="esp-rs logo" width="100px" />
</p>

<h1 align="center">esp-hal</h1>

<p align="center">
  <img src="https://img.shields.io/github/actions/workflow/status/esp-rs/esp-hal/ci.yml?labelColor=1C2C2E&label=CI&logo=github&style=flat-square" alt="GitHub Actions Workflow Status" />
  <img src="https://img.shields.io/github/actions/workflow/status/esp-rs/esp-hal/hil.yml?labelColor=1C2C2E&label=HIL&logo=github&style=flat-square&event=merge_group" alt="GitHub Actions Workflow Status" />
  <img src="https://img.shields.io/badge/license-MIT%2FApache--2.0-blue?labelColor=1C2C2E&style=flat-square" alt="MIT/Apache-2.0 licensed" />
  <a href="https://matrix.to/#/#esp-rs:matrix.org">
    <img src="https://img.shields.io/matrix/esp-rs:matrix.org?labelColor=1C2C2E&label=join%20matrix&color=BEC5C9&logo=matrix&style=flat-square" alt="Matrix" />
  </a>
</p>

Bare-metal (`no_std`) hardware abstraction layer for Espressif devices. Currently supports the following devices:

- ESP32 Series: _ESP32_
- ESP32-C Series: _ESP32-C2, ESP32-C3, ESP32-C6_
- ESP32-H Series: _ESP32-H2_
- ESP32-S Series: _ESP32-S2, ESP32-S3_

Additionally provides support for programming the low-power RISC-V cores found on the _ESP32-C6_, _ESP32-S2_, and _ESP32-S3_ via the [esp-lp-hal] package.

For additional information regarding any of the crates in this repository, please refer to the relevant crate's `README.md` file. If you have any questions, comments, or concerns, please [open an issue], or join us on [Matrix].

If you are currently using (or considering using) `esp-hal` in a production environment and have any feedback or require support, please feel free to contact us at <rust.support@espressif.com>.

> [!NOTE]
>
> This repository includes crates that are at various stages of maturity and stability. While many functionalities have already been implemented and are usable for most tasks, certain advanced or less common features may still be under development. Each crate may offer different levels of functionality and guarantees.

[esp-lp-hal]: https://github.com/esp-rs/esp-hal/tree/main/esp-lp-hal
[esp-idf-svc]: https://github.com/esp-rs/esp-idf-svc
[open an issue]: https://github.com/esp-rs/esp-hal/issues/new
[matrix]: https://matrix.to/#/#esp-rs:matrix.org

## Getting Started

For information relating to the development of Rust applications on ESP devices, please first read [The Rust on ESP Book].

For information about the HAL and how to use it in your own projects, please refer to the [documentation].

When browsing the examples, we recommend viewing the tag for the `esp-hal` release you are using to ensure compatibility, e.g. [esp-hal-v1.0.0], as the `main` branch is used for development and APIs may have changed in the meantime.

[The Rust on ESP Book]: https://docs.espressif.com/projects/rust/book/
[documentation]: https://docs.espressif.com/projects/rust/
[esp-hal-v1.0.0]: https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0/examples

## Resources

- [The Rust Programming Language](https://doc.rust-lang.org/book/)
- [The Embedded Rust Book](https://docs.rust-embedded.org/book/index.html)
- [The Embedonomicon](https://docs.rust-embedded.org/embedonomicon/)
- [The Rust on ESP Book](https://docs.espressif.com/projects/rust/esp-hal/latest/)
- [Embedded Rust (no_std) on Espressif](https://docs.espressif.com/projects/rust/no_std-training/)

## Support policy

All active development will occur on `main`.

We will only backport fixes to the _latest_ minor release in a major version. For example, this means we will apply patches (bug fixes) to `1.1.x` until `1.2.0` is released, at which point all patches are only backported to the `1.2.x` series of releases.

If you are a user of `unstable` APIs, we will never push breaking changes in a patch release. However, `unstable` changes _will_ make their way into minor releases. This means that as an `unstable` user updating from `1.1.x` to `1.2.x` _may_ introduce breaking changes. If you depend on `unstable`, we recommend defining your esp-hal dependency as follows:

```toml
esp-hal = { version = "~1.1" }
```

Using the [`~` operator](https://doc.rust-lang.org/cargo/reference/specifying-dependencies.html#tilde-requirements) will prevent cargo auto updating to minor versions, allowing you to use `cargo update` without the possibility of breaking your project.

## AI Contribution Policy

We follow the same policy as the official Rust Embedded working group, please review [the policy](https://github.com/rust-embedded/wg/blob/HEAD/CODE_OF_CONDUCT.md#ai-tool-use-policy) before contributing with AI tools.

## Contributing

We have a number of living documents to aid contributing to the project, please give these a read before modifying code:

- [DEVELOPER-GUIDELINES](https://github.com/esp-rs/esp-hal/blob/main/documentation/DEVELOPER-GUIDELINES.md)
- [CONTRIBUTING-GUIDE](https://github.com/esp-rs/esp-hal/blob/main/documentation/CONTRIBUTING.md)

## License

All packages within this repository are licensed under either of:

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

### Contribution notice

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in
the work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without
any additional terms or conditions.
