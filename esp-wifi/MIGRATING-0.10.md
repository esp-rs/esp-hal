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

-    let init = init(
-        timg0.timer0,
-        Rng::new(peripherals.RNG),
-        peripherals.RADIO_CLK,
-    )
-    .unwrap();
+    let mut rng = Rng::new(peripherals.RNG);
+    let init = init(timg0.timer0, rng.clone(), peripherals.RADIO_CLK).unwrap();
 
     let mut wifi = peripherals.WIFI;
+    let (iface, device, mut controller) =
+        create_network_interface(&init, &mut wifi, WifiStaDevice).unwrap();
+
-    let (iface, device, mut controller, sockets) =
-        create_network_interface(&init, &mut wifi, WifiStaDevice, &mut socket_set_entries).unwrap();
+    let mut socket_set_entries: [SocketStorage; 3] = Default::default();
+    let socket_set = SocketSet::new(&mut socket_set_entries[..]);
     let now = || time::now().duration_since_epoch().to_millis();
-    let wifi_stack = WifiStack::new(iface, device, sockets, now);
+    let wifi_stack = Stack::new(iface, device, socket_set, now, rng.random());
```

The related features are removed from `esp-wifi`: wifi-default, ipv6, ipv4, tcp, udp, icmp, igmp, dns, dhcpv4
