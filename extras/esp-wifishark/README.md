# esp-wifishark

This is an extcap to be used with esp-wifi and the `dump_packets` config option.

To use it build via `cargo build --release` and copy the resulting executable to the Wireshark's `extcap` folder.

Then you should see two new capture interfaces in Wireshark
- esp-wifi HCI capture (for Bluetooth HCI)
- esp-wifi Ethernet capture (for WiFi traffic)

If you are running an application using esp-wifi's `dump_packets` config and logging at INFO level active these capture interfaces can connect via serialport to give you insights on what is going on.

By default it tries to identify exactly one serialport. If that doesn't work for you, you can configure the serialport via the Wireshark UI.
