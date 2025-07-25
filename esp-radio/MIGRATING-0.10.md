# Migration Guide from 0.10.x to v0.11.x

## Initialization changes

`EspWifiInitFor` has been removed, individual drivers such as `Wifi` and `BleConnector` handle the initialization and de-initialization of the radio stack.

`EspWifiInit` has been removed in favour of `EspWifiController`, each radio driver takes reference to this object. If no driver is borrowing `EspWifiController`,
you can safely call `EspWifiController::deinit()` to completely deinit the stack and return the peripherals used in `esp_wifi::init`.

`esp-wifi::init` now takes all peripherals using the `PeripheralRef` pattern, with the exception of the rng source.

`esp_wifi::init` now accepts `esp_hal::rng::Rng` or `esp_hal::rng::Trng`.

The following error enum variants have been removed from `InitializationError`:

- `Timer(hal::timer::Error)`
- `TimerUnavailable`
- `RadioClockUnavailable`

```diff
- let init: EspWifiInitialization = esp_wifi::init(EspWifiInitFor::Wifi, timg0.timer0, rng.clone(), peripherals.RADIO_CLK).unwrap();
+ let init: EspWifiController = esp_wifi::init(timg0.timer0, rng.clone(), peripherals.RADIO_CLK).unwrap();
```

## No need to include `rom_functions.x` manually

Don't include `rom_functions.x` from esp-wifi

```diff
rustflags = [
    "-C", "link-arg=-Tlinkall.x",
-    "-C", "link-arg=-Trom_functions.x",
]
```

## ESP-NOW: Use `data` to access the received payload

Previously `data` and `len` were public - use the previously already existing `data()` function.

Accessing `data` or `len` was never encouraged.

## Async features have been removed and async functionality is always available

The cost of this is that we need to rename the various `async` methods on `WifiController`.

```diff
- controller.start().await.unwrap();
+ controller.start_async().await.unwrap();
```

## The blocking networking stack was removed

The blocking networking stack is not included anymore. You can use e.g. `smoltcp-nal` or even use `smoltcp` directly.

For an easy migration path there is https://github.com/bjoernQ/blocking-network-stack.git which is basically the previously included networking stack as it's
own crate.

The `create_network_interface` function doesn't take `&mut SocketSet[..]` anymore.

```diff
+use blocking_network_stack::Stack;
use esp_wifi::{
-    wifi_interface::WifiStack,
};

+    let mut rng = Rng::new(peripherals.RNG);
-    let init = init(timg0.timer0, rng, peripherals.RADIO_CLK).unwrap();
+    let init = init(timg0.timer0, rng.clone(), peripherals.RADIO_CLK).unwrap();

     let mut wifi = peripherals.WIFI;
     let mut socket_set_entries: [SocketStorage; 3] = Default::default();
+    let (iface, device, mut controller) =
+        create_network_interface(&init, &mut wifi, WifiStaDevice).unwrap();
+
-    let (iface, device, mut controller, sockets) =
-        create_network_interface(&init, &mut wifi, WifiStaDevice, &mut socket_set_entries).unwrap();
+    let socket_set = SocketSet::new(&mut socket_set_entries[..]);
     let now = || time::now().duration_since_epoch().to_millis();
-    let wifi_stack = WifiStack::new(iface, device, sockets, now);
+    let wifi_stack = Stack::new(iface, device, socket_set, now, rng.random());
```

The related features are removed from `esp-wifi`: wifi-default, ipv6, ipv4, tcp, udp, icmp, igmp, dns, dhcpv4

## `get_` prefixes have been removed from functions

In order to better comply with the Rust API Guidelines [getter names convention], we have removed the `get_` prefixes from all functions which previously had it. Due to the number of changes it's not practical to list all changes here, however if a function previous began with `get_`, you can simply remove this prefix.

[getter names convention]: https://rust-lang.github.io/api-guidelines/naming.html#c-getter

## Removal of features

Along with the features removed regarding the blocking network stack, the following features have been removed:

- `async` - Enabled by default
- `phy-enable-usb` - Turned into a configuration option
- `dump-packets` - Turned into a configuration option
- `ps-min-modem` & `ps-max-modem` - Turned into a runtime configuration, see `set_power_saving`
