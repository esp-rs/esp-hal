# `hil-test-radio`

Hardware-in-loop testing for [`esp-radio`](../esp-radio).

Unlike [`hil-test`](../hil-test), these tests run over-the-air across **two** boards of the same chip:

1. **Harness board**: Runs support firmware (`src/bin/support/`). The runner starts the DUT after the first `[HARNESS-READY]` semihosting line from `hil_test::signal_harness_ready()`.
2. **DUT board**: Runs the [`embedded-test`] suite (`src/bin/tests/`) against the harness board.

[`embedded-test`]: https://github.com/probe-rs/embedded-test

Wi-Fi and BLE run on ESP32-C6 and ESP32-S3. IEEE 802.15.4 runs on ESP32-C6 only.

## Running Tests Locally

`cargo xtask test esp32c6` runs `hil-test` only. Name this package to run radio tests.

Two boards of the target chip **MUST** be connected. The runner reads `probe-rs list` order: the first probe is the harness, the second is the DUT.

```shell
# Run all radio tests for a chip
cargo xtask test hil-test-radio $CHIP

# Run Wi-Fi tests for ESP32-C6
cargo xtask test hil-test-radio esp32c6 wifi

# Repeat a test
cargo xtask test hil-test-radio esp32c6 wifi --repeat 10
```

[probe-rs]: https://probe.rs

## Running Tests Remotely

The [`hil.yml`] workflow runs this package on dual-board runners (`esp32c6-radio` for Wi-Fi, BLE, and IEEE 802.15.4; `esp32s3-radio` for Wi-Fi and BLE). On PRs, `radio` is the CI alias for this package (see [`documentation/HIL-GUIDE.md`](../documentation/HIL-GUIDE.md)):

```text
/test c6 radio
/test s3 radio wifi
```

[`hil.yml`]: https://github.com/esp-rs/esp-hal/blob/main/.github/workflows/hil.yml

## Adding New Tests

Register each binary in `Cargo.toml` with `harness = false` and `required-features = ["esp-alloc"]`.

1. **Support firmware** (`src/bin/support/`): Add `//% SUPPORT-FIRMWARE: true` and call `hil_test::signal_harness_ready()` when the peer is on-air. The runner waits for the first line.
2. **Test binary** (`src/bin/tests/`): Point `//% HARNESS-FIRMWARE` at that support binary. Enable only the radio features this test uses:
```rust
//% CHIP_FILTER: esp32c6 || esp32s3
//% HARNESS-FIRMWARE: wifi_ap_support
//% FEATURES: unstable esp-alloc embassy esp-radio/wifi esp-radio esp-radio-unstable
```

## Logging

Add `defmt` to `//% FEATURES:` to enable [defmt] logs. Remove it before committing. Logging can change test timing.

[defmt]: https://github.com/knurling-rs/defmt