# `hil-test`

Hardware-in-loop testing for `esp-hal`.

For assistance with this package please [open an issue] or [start a discussion].

[open an issue]: https://github.com/esp-rs/esp-hal/issues/new
[start a discussion]: https://github.com/esp-rs/esp-hal/discussions/new/choose

## Quickstart

We use [embedded-test] as our testing framework. This allows us to write unit and integration tests much in the same way you would for a normal Rust project, when the standard library is available, and to execute them using Cargo's built-in test runner.

[embedded-test]: https://github.com/probe-rs/embedded-test

### Running Tests Locally

We use [probe-rs] for flashing and running the tests on a target device, however, this **MUST** be installed from the correct revision:

```text
cargo install probe-rs-tools \
  --git https://github.com/probe-rs/probe-rs \
  --rev 9bde591 --force --locked
```

Target device **MUST** connected via its USB-Serial-JTAG port, or if unavailable (eg. ESP32, ESP32-C2, ESP32-S2) then you must connect a compatible debug probe such as an [ESP-Prog].

You can run all tests for a given device by running the following command from the workspace root:

```shell
cargo xtask run-tests $CHIP
```

For running a single test on a target, from the `xtask` folder run:

```shell
# Run GPIO tests for ESP32-C6
cargo xtask run-tests esp32c6 --test gpio
```

If you want to run a test multiple times:

```shell
# Run GPIO tests for ESP32-C6
cargo xtask run-tests esp32c6 --test gpio --repeat 10
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

Some tests will require physical connections, please see the current [configuration in our runners].

[probe-rs]: https://probe.rs
[ESP-Prog]: https://docs.espressif.com/projects/esp-dev-kits/en/latest/other/esp-prog/user_guide.html
[configuration in our runners]: #running-tests-remotes-ie-on-self-hosted-runners

### Running Tests Remotes (ie. on Self-Hosted Runners)
The [`hil.yml`] workflow builds the test suite for all our available targets and executes them.

Our self-hosted runners have the following setup:
- ESP32-C2 (`esp32c2-jtag`):
  - Devkit: `ESP8684-DevKitM-1` connected via UART.
    - `GPIO2` and `GPIO3` are connected.
  - Probe: `ESP-Prog` connected with the [following connections][connection_c2]
  - RPi: Raspbian 12 configured with the following [setup]
- ESP32-C3 (`rustboard`):
  - Devkit: `ESP32-C3-DevKit-RUST-1` connected via USB-Serial-JTAG.
    - `GPIO2` and `GPIO3` are connected.
    - `GPIO5` and `GPIO6` are connected.
  - RPi: Raspbian 12 configured with the following [setup]
- ESP32-C6 (`esp32c6-usb`):
  - Devkit: `ESP32-C6-DevKitC-1 V1.2` connected via USB-Serial-JTAG (`USB` port).
    - `GPIO2` and `GPIO3` are connected.
    - `GPIO5` and `GPIO6` are connected.
  - RPi: Raspbian 12 configured with the following [setup]
- ESP32-H2 (`esp32h2-usb`):
  - Devkit: `ESP32-H2-DevKitM-1` connected via USB-Serial-JTAG (`USB` port).
    - `GPIO2` and `GPIO3` are connected.
    - `GPIO5` and `GPIO8` are connected.
  - RPi: Raspbian 12 configured with the following [setup]
- ESP32-S2 (`esp32s2-jtag`):
  - Devkit: `ESP32-S2-Saola-1` connected via UART.
    - `GPIO2` and `GPIO3` are connected.
    - `GPIO5` and `GPIO6` are connected.
  - Probe: `ESP-Prog` connected with the [following connections][connection_s2]
  - RPi: Raspbian 12 configured with the following [setup]
- ESP32-S3 (`esp32s3-usb`):
  - Devkit: `ESP32-S3-DevKitC-1` connected via USB-Serial-JTAG.
    - `GPIO2` and `GPIO3` are connected.
    - `GPIO5` and `GPIO6` are connected.
    - `GPIO1` and `GPIO21` are connected.
    - `GPIO43 (TX)` and `GPIO45` are connected.
  - RPi: Raspbian 12 configured with the following [setup]

[connection_c2]: https://docs.espressif.com/projects/esp-idf/en/stable/esp32c2/api-guides/jtag-debugging/configure-other-jtag.html#configure-hardware
[connection_s2]: https://docs.espressif.com/projects/esp-idf/en/stable/esp32s2/api-guides/jtag-debugging/configure-other-jtag.html#configure-hardware
[`hil.yml`]: https://github.com/esp-rs/esp-hal/blob/main/.github/workflows/hil.yml
[setup]: #rpi-setup

#### RPi Setup
```bash
# Install Rust:
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- --default-toolchain stable -y --profile minimal
# Source the current shell:
. "$HOME/.cargo/env"
# Install dependencies
sudo apt install -y pkg-config libudev-dev uhubctl
# Install probe-rs
cargo install probe-rs-tools --git https://github.com/probe-rs/probe-rs --rev 9bde591 --force
# Add the udev rules
wget -O - https://probe.rs/files/69-probe-rs.rules | sudo tee /etc/udev/rules.d/69-probe-rs.rules > /dev/null
# Add the user to plugdev group
sudo usermod -a -G plugdev $USER
# Install espflash
ARCH=$($HOME/.cargo/bin/rustup show | grep "Default host" | sed -e 's/.* //')
curl -L "https://github.com/esp-rs/espflash/releases/latest/download/espflash-${ARCH}.zip" -o "${HOME}/.cargo/bin/espflash.zip"
unzip "${HOME}/.cargo/bin/espflash.zip" -d "${HOME}/.cargo/bin/"
rm "${HOME}/.cargo/bin/espflash.zip"
chmod u+x "${HOME}/.cargo/bin/espflash"
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

## Logging in tests

The tests can use [defmt] to print logs. To enable log output, add the `defmt` feature to the test
you want to run. Eg:

```rust
//! AES Test

//% CHIPS: esp32 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: defmt
```

Make sure to remove this addition before you commit any modifications.

> NOTE: log output is disabled by default. Enabling it can introduce some timing issues, which
makes some tests fail randomly. This issue affects all Xtensa devices, as well as ESP32-C2 and
ESP32-C3 currently.

[defmt]: https://github.com/knurling-rs/defmt
