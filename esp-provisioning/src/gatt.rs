//! Provisioning GATT service, built with `trouble-host`'s `gatt_server` derive.

use trouble_host::prelude::*;

/// Espressif provisioning GATT server (proto-ver, session, config, scan, ctrl).
#[gatt_server(attribute_table_size = 64)]
pub(crate) struct ProvServer {
    /// Provisioning service `021a9004-0382-4aea-bff4-6b3f1c5adfb4`.
    pub service: ProvService,
}

/// Characteristics use the service UUID with the 16-bit endpoint id in the time-low field.
#[gatt_service(uuid = "021a9004-0382-4aea-bff4-6b3f1c5adfb4")]
pub(crate) struct ProvService {
    /// Encrypted `wifi_ctrl` protobuf (`CtrlReset` / `CtrlReprov`).
    #[descriptor(uuid = "2901", read, value = "prov-ctrl")]
    #[characteristic(uuid = "021aff4f-0382-4aea-bff4-6b3f1c5adfb4", read, write, notify)]
    pub ctrl: heapless::Vec<u8, 64>,

    /// Encrypted `wifi_scan` protobuf.
    #[descriptor(uuid = "2901", read, value = "prov-scan")]
    #[characteristic(uuid = "021aff50-0382-4aea-bff4-6b3f1c5adfb4", read, write, notify)]
    pub scan: heapless::Vec<u8, 512>,

    /// sec1 handshake.
    #[descriptor(uuid = "2901", read, value = "prov-session")]
    #[characteristic(uuid = "021aff51-0382-4aea-bff4-6b3f1c5adfb4", read, write, notify)]
    pub session: heapless::Vec<u8, 256>,

    /// Encrypted `wifi_config` protobuf.
    #[descriptor(uuid = "2901", read, value = "prov-config")]
    #[characteristic(uuid = "021aff52-0382-4aea-bff4-6b3f1c5adfb4", read, write, notify)]
    pub config: heapless::Vec<u8, 256>,

    /// `proto-ver` JSON.
    #[descriptor(uuid = "2901", read, value = "proto-ver")]
    #[characteristic(uuid = "021aff53-0382-4aea-bff4-6b3f1c5adfb4", read, write, notify)]
    pub proto_ver: heapless::Vec<u8, 128>,
}
