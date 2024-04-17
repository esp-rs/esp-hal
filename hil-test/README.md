# hil-test

Hardware-in-loop testing for `esp-hal`.

For assistance with this package please [open an issue] or [start a discussion].

[open an issue]: https://github.com/esp-rs/esp-hal/issues/new
[start a discussion]: https://github.com/esp-rs/esp-hal/discussions/new/choose

## Quickstart

We use [embedded-test] as our testing framework, which relies on [defmt] internally. This allows us to write unit and integration tests much in the same way you would for a normal Rust project, when the standard library is available, and to execute them using Cargo's built-in test runner.

[embedded-test]: https://github.com/probe-rs/embedded-test
[defmt]: https://github.com/knurling-rs/defmt

### Running Tests Locally

We use [probe-rs] for flashing and running the tests on a target device, however, this **MUST** be installed from the correct revision, and with the correct features enabled:

```text
cargo install probe-rs \
  --git=https://github.com/probe-rs/probe-rs \
  --rev=ddd59fa \
  --features=cli \
  --bin=probe-rs
```

Target device **MUST** connected via its USB-Serial-JTAG port, or if unavailable (eg. ESP32, ESP32-C2, ESP32-S2) then you must connect a compatible debug probe such as an [ESP-Prog].

You can run all tests for a given device by running the following command from the `xtask` folder:

```shell
cargo xtask run-tests $CHIP
```

For running a single test on a target, from the `xtask` folder run:

```shell
# Run GPIO tests for ESP32-C6
cargo xtask run-tests esp32c6 --test gpio
```

Another alternative way of running a single test is, from the `hil-tests` folder:
```shell
# Run GPIO tests for ESP32-C6
CARGO_BUILD_TARGET=riscv32imac-unknown-none-elf \
PROBE_RS_CHIP=esp32c6 \
  cargo +nightly test --features=esp32c6 --test=gpio
```
- If the `--test` argument is omitted, then all tests will be run, independently if the tests are supported for that target, for this reason, we encourage using the `xtask` approach.
- The build target **MUST** be specified via the `CARGO_BUILD_TARGET` environment variable or as an argument (`--target`).
- The chip **MUST** be specified via the `PROBE_RS_CHIP` environment variable or as an argument of `probe-rs` (`--chip`).

Some tests will require physical connections, please see the current [configuration in our runners](#running-tests-remotes-ie-on-self-hosted-runners).

### Running Tests Remotes (ie. On Self-Hosted Runners)
The [`hil.yml`] workflow builds the test suite for all our available targets and executes them.

Our Virtual Machines have the following setup:
- ESP32-C3 (`rustboard`):
  - Devkit: `ESP32-C3-DevKit-RUST-1` connected via USB-Serial-JTAG.
    - `GPIO2` and `GPIO4` are connected.
  - VM: Ubuntu 20.04.5 configured with the following [setup](#vm-setup)
- ESP32-C6 (`esp32c6-usb`):
  - Devkit: `ESP32-C6-DevKitC-1 V1.2` connected via USB-Serial-JTAG (`USB` port).
    - `GPIO2` and `GPIO4` are connected.
  - VM: Ubuntu 20.04.5 configured with the following [setup](#vm-setup)
- ESP32-H2 (`esp32h2-usb`):
  - Devkit: `ESP32-H2-DevKitM-1` connected via USB-Serial-JTAG (`USB` port).
    - `GPIO2` and `GPIO4` are connected.
  - VM: Ubuntu 20.04.5 configured with the following [setup](#vm-setup)
- ESP32-S3 (`esp32s3-usb`):
  - Devkit: `ESP32-S3-DevKitC-1` connected via USB-Serial-JTAG.
    - `GPIO2` and `GPIO4` are connected.
  - VM: Ubuntu 22.04.4 configured with the following [setup](#vm-setup)

[`hil.yml`]: https://github.com/esp-rs/esp-hal/blob/main/.github/workflows/hil.yml

#### VM Setup
```bash
# Install Rust:
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- --default-toolchain stable -y --profile minimal
# Source the current shell:
source "$HOME/.cargo/env"
# Install dependencies
sudo apt install -y pkg-config libudev-dev
# Install probe-rs
cargo install probe-rs --git=https://github.com/probe-rs/probe-rs --rev=ddd59fa --features=cli --bin=probe-rs --locked --force
# Add the udev rules
wget -O - https://probe.rs/files/69-probe-rs.rules | sudo tee /etc/udev/rules.d/69-probe-rs.rules > /dev/null
# Add the user to plugdev group
sudo usermod -a -G plugdev $USER
# Reboot the VM
sudo reboot
```

## Adding New Tests

1. Create a new integration test file (`tests/$PERIPHERAL.rs`)
2. Add a corresponding `[[test]]` entry to `Cargol.toml` (**MUST** set `harness = false`)
3. Write the tests
4. Document any necessary physical connections on boards connected to self-hosted runners
5. Add a header in the test stating which targets support the given tests. Eg:
```rust
//! AES Test

//% CHIPS: esp32 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
```
If the test is supported by all the targets, you can omit the header.
6. Write some documentation at the top of the `tests/$PERIPHERAL.rs` file with the pins being used and the required connections, if applicable.
