# hil-test

Hardware-in-loop testing for `esp-hal`.

For assistance with this package please [open an issue] or [start a discussion], or contact @jessebraham on Matrix.

[open an issue]: https://github.com/esp-rs/esp-hal/issues/new
[start a discussion]: https://github.com/esp-rs/esp-hal/discussions/new/choose

## Quickstart

We use [embedded-test] as our testing framework, which relies on [defmt] internally. This allows us to write unit and integration tests much in the same way you would for a normal Rust project when the standard library is available, and to execute them using Cargo's built-in test runner.

[embedded-test]: https://github.com/probe-rs/embedded-test
[defmt]: https://github.com/knurling-rs/defmt

### Running Tests Locally

We use [probe-rs] for flashing and runnings the tests on a target device, however this **MUST** be installed from the correct branch, and with the correct features enabled:

```text
cargo install probe-rs \
  --git=https://github.com/probe-rs/probe-rs \
  --branch=feature/testing \
  --features=cli,ftdi \
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
  cargo test --features=$CHIP
```

For example, testing GPIO on the ESP32-C6:

```shell
CARGO_BUILD_TARGET=riscv32imac-unknown-none-elf \
PROBE_RS_CHIP=esp32c6 \
  cargo test --features=esp32c6 --test=gpio
```

The `--test` argument is optional, and if omitted then all tests will be run.

[probe-rs]: https://github.com/probe-rs/probe-rs/

### Running Tests Remotes (ie. On Self-Hosted Runners)

- ???

## Adding New Tests

- Create a new integration test file (`tests/$PERIPHERAL.rs`)
- Add a corresponding `[[test]]` entry to `Cargol.toml` (**MUST** set `harness = false`)
- Write the tests (elaborate)
- Make any necessary physical connections on boards connected to self-hosted runners
  - We need a way to document this that is accessible by the public
  - Maybe I can whip up a simple web app to host on `esp-rs.org` or something?
