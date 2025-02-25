# Migration Guide from 0.11.x to v0.12.x

## Crate configuration changes

To prevent ambiguity between configurations, we had to change the naming format of configuration
keys. Before, we used `{prefix}_{key}`, which meant that esp-hal and esp-hal-* configuration keys
were impossible to tell apart. To fix this issue, we are changing the separator from one to two
underscore characters. This also means that users will have to change their `config.toml`
configurations to match the new format.

```diff
 [env]
-ESP_WIFI_RX_QUEUE_SIZE = "16"
-ESP_WIFI_STATIC_RX_BUF_NUM = "32"
-ESP_WIFI_DYNAMIC_RX_BUF_NUM = "16"
+ESP_WIFI_CONFIG_RX_QUEUE_SIZE = "16"
+ESP_WIFI_CONFIG_STATIC_RX_BUF_NUM = "32"
+ESP_WIFI_CONFIG_DYNAMIC_RX_BUF_NUM = "16"
```

## `csi_enabled` option converted to feature

As part of limiting public API changes due to config options, the `csi_enabled` option has been changed to feature. The feature must now be activated in `esp-wifi` crate to activate the corresponding functionality. 

```diff
# In `Cargo.toml`:
-esp-wifi = { version = "0.12.0", features = ["wifi"] }
+esp-wifi = { version = "0.12.0", features = ["wifi", "csi"] }
```

## Changed the way to get the WiFi controller and interfaces

The network interfaces and the controller are now more separated. This way you can change between STA, AP and AP_STA mode easily without reconstructing the networking stacks.

There is no convenience utility to create a `smoltcp` interface needed by blocking networking stacks anymore. You need your own implementation.

Please note that networking stacks _might_ need to be reset when connecting to a different network interface (i.e. get a new IP address and routings) - `embassy-net` should manage to do that automatically.

```diff
-    let (iface, device, mut controller) =
-        create_network_interface(&init, peripherals.WIFI, WifiStaDevice).unwrap();
+    let (mut controller, interfaces) =
+        esp_wifi::wifi::new(&init, peripherals.WIFI).unwrap();
+    let mut device = interfaces.sta;
+    let iface = create_interface(&mut device);
    ...
+ fn timestamp() -> smoltcp::time::Instant {
+     smoltcp::time::Instant::from_micros(
+         esp_hal::time::Instant::now()
+             .duration_since_epoch()
+             .as_micros() as i64,
+     )
+ }
+ 
+ pub fn create_interface(device: &mut esp_wifi::wifi::WifiDevice) -> smoltcp::iface::Interface {
+     // users could create multiple instances but since they only have one WifiDevice
+     // they probably can't do anything bad with that
+     smoltcp::iface::Interface::new(
+         smoltcp::iface::Config::new(smoltcp::wire::HardwareAddress::Ethernet(
+             smoltcp::wire::EthernetAddress::from_bytes(&device.mac_address()),
+         )),
+         device,
+         timestamp(),
+     )
+ }
```

If you have been using `esp_wifi::wifi::new_with_mode` before:

- You will have to replace `new_with_mode` with `new`, which does not need a device kind parameter.
- You will have to configure the returned controller into the correct mode.

```diff
-let (ap_device, controller) = esp_wifi::wifi::new_with_mode(
+let (controller, interfaces) = esp_wifi::wifi::new(
     &init,
     wifi,
-    WifiApDevice
 ).unwrap();

+let ap_device = interfaces.ap;
+controller.set_configuration(&Configuration::AccessPoint(AccessPointConfiguration {
+    ssid: "MyNetwork".try_into().unwrap(),
+    ..Default::default()
+}));
```

If you are using both the AP and STA interfaces, you will need to apply `Configuration::Mixed`,
and you will need to make sure you don't accidentally reapply a non-`Mixed` configuration.

## Other API changes

`esp_wifi::wifi::WifiDevice` is no longer generic over the interface mode:

```diff
-WifiDevice<'static, WifiApDevice>
+WifiDevice<'static>
```

`WifiController` no longer stores the applied configuration. The `configuration` getter has been
removed. If you need to access the currently applied settings, you will need to store them yourself
in your firmware.
