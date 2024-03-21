# hil-test

Hardware-in-loop testing for `esp-hal`.

For assistance with this package please [open an issue] or [start a discussion].

[open an issue]: https://github.com/esp-rs/esp-hal/issues/new
[start a discussion]: https://github.com/esp-rs/esp-hal/discussions/new/choose

## Quickstart

We use [embedded-test] as our testing framework, which relies on [defmt] internally. This allows us to write unit and integration tests much in the same way you would for a normal Rust project when the standard library is available, and to execute them using Cargo's built-in test runner.

[embedded-test]: https://github.com/probe-rs/embedded-test
[defmt]: https://github.com/knurling-rs/defmt

### Running Tests Locally

We use [probe-rs] for flashing and running the tests on a target device, however this **MUST** be installed from the correct branch, and with the correct features enabled:

```text
cargo install probe-rs \
  --git=https://github.com/probe-rs/probe-rs \
  --branch=feature/testing-rebased \
  --features=cli \
  --bin=probe-rs
```

You **MUST** have the target device connected via its USB-Serial-JTAG port, or if unavailable (eg. ESP32, ESP32-C2, ESP32-S2) then you must connect a compatible debug probe such as an ESP-Prog.

Additionally:

- The build target **MUST** be specified via the `CARGO_BUILD_TARGET` environment variable.
- The chip **MUST** be specified via the `PROBE_RS_CHIP` environment variable.

Generalized, tests can be run locally via:

```shell
CARGO_BUILD_TARGET=$TARGET_TRIPLE \
PROBE_RS_CHIP=$CHIP \
  cargo +$TOOLCHAIN test --features=$CHIP
```

For example, testing GPIO on the ESP32-C6:

```shell
CARGO_BUILD_TARGET=riscv32imac-unknown-none-elf \
PROBE_RS_CHIP=esp32c6 \
  cargo +nightly test --features=esp32c6 --test=gpio
```

The `--test` argument is optional, and if omitted then all tests will be run.

Also, there is an alias to run all tests for a target with:

```shell
cargo +$TOOLCHAIN $CHIP
```

For example, to run all tests on the ESP32-S3:

```shell
cargo +esp esp32s3
```

[probe-rs]: https://github.com/probe-rs/probe-rs/

### Running Tests Remotes (ie. On Self-Hosted Runners)
The `hil.yml` workflow will build the test suite for all our available targets and run the tests on them.

Currently, here are the Virtual Machines set up for HIL testing:
- ESP32-C3:
  - Has an `ESP32-C3-DevKit-RUST-1` connected via USB-JTAG-SERIAL.
    - Pins 2 and 4 are connected for `spi_full_duplex` and `uart` tests.
  - VM has the following [setup](#vm-setup)
- ESP32-C6:
  - Has an `ESP32-C6-DevKitC-1 V1.2` connected via USB-JTAG-SERIAL.
    - Pins 2 and 4 are connected for `spi_full_duplex` and `uart` tests.
  - VM has the following [setup](#vm-setup)
- ESP32-H2
  - Has an `ESP32-H2-DevKitM-1` connected via USB-JTAG-SERIAL.
    - Pins 2 and 4 are connected for `spi_full_duplex` and `uart` tests.
  - VM has the following [setup](#vm-setup)

#### VM Setup
```bash
# Install Rust:
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- --default-toolchain stable -y --profile minimal
# Source the current shell:
source "$HOME/.cargo/env"
# Install dependencies
sudo apt install -y pkg-config libudev-dev
# Install probe-rs
cargo install probe-rs --git=https://github.com/probe-rs/probe-rs --rev=b431b24 --features=cli --bin=probe-rs --locked --force
# Add the udev rules
wget -O - https://probe.rs/files/69-probe-rs.rules | sudo tee /etc/udev/rules.d/69-probe-rs.rules > /dev/null
# Add the user to plugdev group
sudo usermod -a -G plugdev $USER
# Reboot the VM
```

## Adding New Tests

- Create a new integration test file (`tests/$PERIPHERAL.rs`)
- Add a corresponding `[[test]]` entry to `Cargol.toml` (**MUST** set `harness = false`)
- Write the tests
- Document any necessary physical connections on boards connected to self-hosted runners
  - Write some documentation at the top of the `tests/$PERIPHERAL.rs` file with the pins being used and the required connections, if applicable.


