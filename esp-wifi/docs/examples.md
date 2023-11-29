## Examples

 > The following instructions assume you have the [Xtensa toolchain](https://esp-rs.github.io/book/installation/riscv-and-xtensa.html) set up. In case you are building for a RISC-V target and would use
the mainline Rust compiler, you'll need to edit or remove the `rust-toolchain.toml` file.

To build these ensure you are in the `esp-wifi` directory (the inner one, which has `examples` inside) and you are using the right chip name invocation. For example, to build for the `esp32c3`, please run 

```
cargo esp32c3 --release ...
```

### dhcp

- set SSID and PASSWORD env variable
- gets an ip address via DHCP
- performs an HTTP get request to some "random" server

`cargo $CHIP --example dhcp --release --features "wifi"`

### static_ip

- set SSID and PASSWORD env variable
- set STATIC_IP and GATEWAY_IP env variable (e.g. "192.168.2.191" / "192.168.2.1")
- might be necessary to configure your WiFi access point accordingly
- uses the given static IP
- responds with some HTML content when connecting to port 8080

`cargo $CHIP --example static_ip --release --features "wifi"`

### ble

- starts Bluetooth advertising
- offers one service with three characteristics (one is read/write, one is write only, one is read/write/notify)
- pressing the boot-button on a dev-board will send a notification if it is subscribed
- this uses a toy level BLE stack - might not work with every BLE central device (tested with Android and Windows Bluetooth LE Explorer)

`cargo $CHIP --example ble --release --features "ble"`

**NOTE:** ESP32-S2 doesn't support bluetooth

### embassy_ble

- same as `ble` but async

`cargo $CHIP --example embassy_ble --release --features "async,ble"`

**NOTE:** ESP32-S2 doesn't support bluetooth

### coex

- set SSID and PASSWORD env variable
- gets an ip address via DHCP
- performs an HTTP get request to some "random" server
- does BLE advertising
- coex support is still somewhat flaky

`cargo $CHIP --example coex --release --features "wifi,ble"`

**NOTE:** Not currently available for the ESP32, ESP32-C2, ESP32-C6 or ESP32-S2

### esp_now

- broadcasts, receives and sends messages via esp-now

`cargo $CHIP --example esp_now --release --features "esp-now"`

### embassy_esp_now

- broadcasts, receives and sends messages via esp-now in an async way

`cargo $CHIP --example embassy_esp_now --release --features "async,esp-now"`

### embassy_esp_now_duplex

- asynchronously broadcasts, receives and sends messages via esp-now in multiple embassy tasks

`cargo $CHIP --example embassy_esp_now_duplex --release --features "async,esp-now"`

### embassy_dhcp

- Read and Write to sockets over WiFi asyncronously using embassy-executor.

`cargo $CHIP --example embassy_dhcp --release --features "async,wifi,embassy-net"`

### access_point

- creates an open access-point with SSID `esp-wifi`
- you can connect to it using a static IP in range 192.168.2.2 .. 192.168.2.255, gateway 192.168.2.1
- open http://192.168.2.1:8080/ in your browser
- on Android you might need to choose _Keep Accesspoint_ when it tells you the WiFi has no internet connection, Chrome might not want to load the URL - you can use a shell and try `curl` and `ping`

`cargo $CHIP --example access_point --release --features "wifi"`

### access_point_with_sta

- set SSID and PASSWORD env variable
- gets an ip address via DHCP
- creates an open access-point with SSID `esp-wifi`
- you can connect to it using a static IP in range 192.168.2.2 .. 192.168.2.255, gateway 192.168.2.1
- open http://192.168.2.1:8080/ in your browser - the example will perform an HTTP get request to some "random" server
- on Android you might need to choose _Keep Accesspoint_ when it tells you the WiFi has no internet connection, Chrome might not want to load the URL - you can use a shell and try `curl` and `ping`

`cargo $CHIP --example access_point_with_sta --release --features "wifi"`

### embassy_access_point

- creates an open access-point with SSID `esp-wifi`
- you can connect to it using a static IP in range 192.168.2.2 .. 192.168.2.255, gateway 192.168.2.1
- open http://192.168.2.1:8080/ in your browser
- on Android you might need to choose _Keep Accesspoint_ when it tells you the WiFi has no internet connection, Chrome might not want to load the URL - you can use a shell and try `curl` and `ping`

`cargo $CHIP --example embassy_access_point --release --features "async,wifi,embassy-net"`

### embassy_access_point_with_sta

- set SSID and PASSWORD env variable
- gets an ip address via DHCP
- creates an open access-point with SSID `esp-wifi`
- you can connect to it using a static IP in range 192.168.2.2 .. 192.168.2.255, gateway 192.168.2.1
- open http://192.168.2.1:8080/ in your browser - the example will perform an HTTP get request to some "random" server
- on Android you might need to choose _Keep Accesspoint_ when it tells you the WiFi has no internet connection, Chrome might not want to load the URL - you can use a shell and try `curl` and `ping`

`cargo $CHIP --example embassy_access_point_with_sta --release --features "async,wifi,embassy-net"`

## Benchmarking

A prerequisite to running the benchmark examples is to run the benchmark server on your local machine. Simply run the following commands to do so.

```
cd extras/bench-server
cargo run --release
```

### bench

- Run a test of download, upload and download+upload in a blocking fashion.
- Ensure you have set the IP of your local machine in the `HOST_IP` env variable. E.g `HOST_IP="192.168.0.24"`

`cargo $CHIP --example bench --release --features "wifi"`

### embassy_bench

- Run a test of download, upload and download+upload in a async fashion.

`cargo $CHIP --example embassy_bench --release --features "wifi,embassy-net"`