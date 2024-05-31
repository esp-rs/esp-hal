# IEEE 802.15.4 Sniffer

This is an extcap to be used with the `ieee802154_sniffer` example. (Make sure the ESP32-C6/ESP32-H2 is connected via UART-bridge, not JTAG-Serial)

To use it, build via `cargo build --release` and copy the resulting executable to the Wireshark's `extcap` folder.

Then you should see a new capture interface in Wireshark

If you are running the ieee802154_sniffer example this capture interface can connect via serialport to give you insights on what is going on.

By default it tries to identify exactly one serialport. If that doesn't work for you, you can configure the serialport via the Wireshark UI.

In Wireshark use `ITU-T-CRC-16` as `FCS format`
