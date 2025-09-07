# Examples

This directory contains a number of binary applications demonstrating the use of various hardware peripherals found within the ESP32 family of devices from Espressif.

Each device has its own unique set of peripherals, and as such not every example will run on every device. We recommend building and flashing the examples using the `xtask` method shown below (no need to install any additional external tools), which will greatly simplify the process.

To check if a device is compatible with a given example, check the features in the `Cargo.toml` file for the example application, which will include a feature for each supported device.

For more information regarding the examples, refer to the `README.md` file in any of the subdirectories within the `examples/` directory.

## Building Examples

You can build all examples for a given device using the `build examples` subcommand:

```shell
cargo xtask build examples esp-hal --chip esp32 all
```

Note that we must specify which package to build the examples for, since this repository contains multiple packages. Specifying `esp-hal` will build the examples in the `examples/` directory instead.

## Running Examples

You can also build and then subsequently flash and run an example using the `run example` subcommand. With a target device connected to your host system, run:

```shell
cargo xtask run example embassy_hello_world --chip=esp32c6
```

Again, note that we must specify which package to build the example from, plus which example to build and flash to the target device.

## Adding Examples

If you are contributing to `esp-hal` and would like to add an example, the process is generally the same as any other project. The `Cargo.toml` file should include a feature for each supported chip, which itself should enable any dependency's features required for the given chip.

Another thing to be aware of is the GPIO pins being used. We have tried to use pins available the DevKit-C boards from Espressif, however this is being done on a best-effort basis.

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
