# Examples

This directory contains a number of binary applications demonstrating the use of various hardware peripherals found within the ESP32 family of devices from Espressif.

Each device has its own unique set of peripherals, and as such not every example will run on every device. We recommend building and flashing the examples using the `xtask` method shown below (no need to install any additional external tools), which will greatly simplify the process.

To check if a device is compatible with a given example, check the features in the `Cargo.toml` file for the example application, which will include a feature for each supported device.

For more information regarding the examples, refer to the `README.md` file in any of the subdirectories within the `examples/` directory.

## Building Examples

You can build all examples for a given device using the `build examples` subcommand:

```shell
cargo xtask build examples --chip esp32 all
```

Or build a single example with: 

```shell
cargo xtask build examples --chip esp32c6 hello_world
```

## Running Examples

You can also build and then subsequently flash and run an example using the `run example` subcommand. With a target device connected to your host system, run:

```shell
cargo xtask run example embassy_hello_world --chip=esp32c6
```

Again, note that we must specify which package to build the example from, plus which example to build and flash to the target device.

## Wi-Fi Networking

The Wi-Fi examples use `esp-radio` for the IEEE 802.11 link and `embassy-net`
for DHCP, DNS, TCP, and UDP. Both ESP32-C3 and ESP32-S3 run the same example
source. The `--chip` argument selects the target and chip-specific Cargo
features.

Start with `embassy_udp`. It connects in Station mode, obtains an IPv4 address
through DHCP, creates an `embassy_net::udp::UdpSocket` directly, and sends
`b"banana"` to a configurable address. On a Linux PC connected to the same
network, start a UDP listener:

```shell
nc -u -l 5000
```

Find that PC's local IP address, then set it along with the access point
credentials as build-time environment variables:

```shell
SSID='your-network' PASSWORD='your-password' \
  UDP_DESTINATION='192.168.1.100:5000' \
  cargo xtask run example embassy_udp --chip=esp32c3
```

For ESP32-S3, change only the chip argument:

```shell
SSID='your-network' PASSWORD='your-password' \
  UDP_DESTINATION='192.168.1.100:5000' \
  cargo xtask run example embassy_udp --chip=esp32s3
```

`SSID`, `PASSWORD`, and `UDP_DESTINATION` are compiled into the example
firmware. Do not commit real credentials to a source file or repository. When
`banana` appears in `nc`, type a reply and press Enter; the example receives it
with `recv_from()` and prints the bytes to the serial console.

Use the example that matches the networking task:

| Task | Example | Application protocol |
| --- | --- | --- |
| Send and receive arbitrary bytes | `embassy_udp` | DHCP and UDP |
| Connect to a router and make a web request | `embassy_dhcp` | DNS, TCP, and HTTP |
| Send and receive UDP datagrams | `embassy_sntp` | DNS, UDP, and SNTP |
| Create a Wi-Fi network and serve a page | `embassy_access_point` | TCP and HTTP |
| Run Access Point and Station modes together | `embassy_access_point_with_sta` | DNS, TCP, and HTTP |
| Run Wi-Fi and Bluetooth Low Energy together | `embassy_coex` | DNS, TCP, HTTP, and BLE |

For custom application data, start from `embassy_udp`: change the payload and
destination passed to `send_to()`. Create sockets only after
`stack.wait_config_up().await` completes.

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
