//! Crate-private protocol constants.

#![allow(dead_code)]

/// JSON body for the `proto-ver` endpoint (sec1 + wifi_scan capability).
pub(crate) const PROTO_VER_JSON: &[u8] =
    b"{\"prov\":{\"ver\":\"v1.1\",\"sec_ver\":1,\"sec_patch_ver\":0,\"cap\":[\"wifi_scan\"]}}";

/// 128-bit BLE provisioning service UUID (little-endian) used by the Espressif
/// provisioning apps.
///
/// Decoded: `021a9004-0382-4aea-bff4-6b3f1c5adfb4`
pub(crate) const PROV_SERVICE_UUID_128: [u8; 16] = [
    0xb4, 0xdf, 0x5a, 0x1c, 0x3f, 0x6b, 0xf4, 0xbf, 0xea, 0x4a, 0x82, 0x03, 0x04, 0x90, 0x1a, 0x02,
];
