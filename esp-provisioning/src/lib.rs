//! ESP-IDF Wi-Fi provisioning for bare-metal Rust.
//!
//! The crate speaks protocomm security 1 and the ESP-IDF v5.4.2
//! `wifi_provisioning` protobufs (`prov-config`, `prov-scan`, and `prov-ctrl`;
//! not `network_provisioning`). The application owns the Wi-Fi controller,
//! `embassy-net` stacks, BLE connector, and join/verify loop:
//!
//! - `softap::serve` — HTTP on an access-point `embassy_net::Stack` the application already brought
//!   up, plus a scan-result slice the application owns.
//! - `ble::serve` — GATT write/read on an `esp_radio::ble::controller::BleConnector` the
//!   application owns (so Bluetooth deinit stays with the caller).
//! - `Session` — shared state coordinating the transports with the application's verify loop via
//!   `Session::credentials` and `Session::report`.
//!
//! Dropping a serve future cancels that transport. Credentials stay in RAM for
//! the session; the crate does not write flash and never logs passphrases.
//!
//! Protobuf sources are vendored from ESP-IDF v5.4.2 `wifi_provisioning` and
//! `protocomm` (not `network_provisioning`). Generated code is committed so
//! firmware builds do not need `protoc`.
//!
//! # Examples
//!
//! `examples/wifi/embassy_provisioning` brings up SoftAP and BLE, pre-scans
//! nearby networks, joins the station with the provisioned credentials, and
//! reports `StaState::Connected` or `StaState::Failed` back to `Session`
//! so the Espressif phone app receives real verification status and can retry
//! via `prov-ctrl`.
//!
//! ## Feature Flags
#![doc = document_features::document_features!(feature_label = r#"<span class="stab portability"><code>{feature}</code></span>"#)]
#![doc(html_logo_url = "https://docs.espressif.com/projects/rust/esp-rs-grey-bg.svg")]
#![deny(missing_docs, rust_2018_idioms, rustdoc::all)]
#![cfg_attr(
    semver_checks,
    allow(rustdoc::private_intra_doc_links, rustdoc::broken_intra_doc_links)
)]
#![no_std]

// MUST be the first module
mod fmt;

#[allow(
    missing_docs,
    clippy::all,
    non_camel_case_types,
    non_snake_case,
    non_upper_case_globals,
    dead_code,
    unused
)]
mod proto_gen;

#[cfg_attr(not(any(feature = "softap", feature = "ble", test)), allow(dead_code))]
mod proto;
#[cfg_attr(not(any(feature = "softap", feature = "ble", test)), allow(dead_code))]
mod protocomm;
#[cfg_attr(not(any(feature = "softap", feature = "ble", test)), allow(dead_code))]
mod sec1;
#[cfg_attr(not(any(feature = "softap", feature = "ble", test)), allow(dead_code))]
mod state;
mod types;

#[cfg(any(feature = "softap", feature = "ble"))]
mod handlers;

#[cfg(feature = "softap")]
pub mod softap;

#[cfg(feature = "ble")]
mod gatt;

#[cfg(feature = "ble")]
pub mod ble;

#[cfg(any(feature = "softap", feature = "ble"))]
mod session;

#[cfg(any(feature = "softap", feature = "ble"))]
#[instability::unstable]
pub use session::{Config, ProvError, Session, StaFailReason, StaState, WifiCredentials};
