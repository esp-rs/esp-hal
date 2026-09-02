# Examples

This directory contains a number of binary applications demonstrating the use of various hardware peripherals found within the ESP32 family of devices from Espressif.

Each device has its own unique set of peripherals, and as such not every example will run on every device. We recommend building and flashing the examples using the `xtask` method shown below (no need to install any additional external tools), which will greatly simplify the process.

To check if a device is compatible with a given example, check the features in the `Cargo.toml` file for the example application, which will include a feature for each supported device.

For more information regarding the examples, refer to the `README.md` file in any of the subdirectories within the `examples/` directory.

## Building Examples

Build all examples for a given device:

```shell
cargo xtask build examples all --chip esp32
```

Or build a single example:

```shell
cargo xtask build hello_world --chip esp32c6
```

## Running Examples

With a target device connected, chip is inferred when possible:

```shell
cargo xtask run embassy_hello_world
```

Pass `--chip` when inference is unavailable.

QA binaries live in `examples/qa` and run the same way (`cargo xtask run sleep_timer`).

## Adding Examples

If you are contributing to `esp-hal` and would like to add an example, the process is generally the same as any other project. The `Cargo.toml` file should include a feature for each supported chip, which itself should enable any dependency's features required for the given chip.

Another thing to be aware of is the GPIO pins being used. We have tried to use pins available on the DevKit-C boards from Espressif, however this is being done on a best-effort basis.

In general, the following GPIO are recommended for use, though be conscious of whether certain pins are used for UART, strapping pins, etc. on some devices:

- GPIO0
- GPIO1
- GPIO2
- GPIO3
- GPIO4
- GPIO5
- GPIO8
- GPIO9
- GPIO10
