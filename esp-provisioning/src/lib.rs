//! ESP-IDF Wi-Fi provisioning for bare-metal Rust.
//!
//! Protocol core (always available):
//! - [`sec1`] — Security 1 (Curve25519 + PoP + AES-256-CTR session)
//! - [`protocomm`] — plaintext `prov-config` / `prov-scan` handlers
//!
//! With the default `softap` and `ble` features, this crate also runs combined
//! provisioning sessions via [`run`]: SoftAP HTTP and BLE GATT race until the
//! first transport delivers Wi-Fi credentials. Credential storage, STA bring-up,
//! and reboot remain the application’s responsibility.
//!
//! # Examples
//!
//! A menu-driven SoftAP + BLE shell lives at
//! `examples/wifi/embassy_provisioning`. It calls [`run`], stores credentials,
//! tests STA join, and can cancel with Esc or hold-C.
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

mod pb;
mod types;

#[cfg(any(feature = "softap", feature = "ble"))]
mod handlers;

#[cfg(feature = "softap")]
mod scan;

#[cfg(feature = "softap")]
pub mod softap;

#[cfg(feature = "ble")]
mod gatt;

#[cfg(feature = "ble")]
pub mod ble;

#[cfg(any(feature = "softap", feature = "ble"))]
mod session;

pub mod protocomm;
pub mod sec1;

#[cfg(feature = "softap")]
#[instability::unstable]
pub use scan::cache_scan_results;
#[cfg(any(feature = "softap", feature = "ble"))]
#[instability::unstable]
pub use session::{ProvConfig, ProvError, WifiCredentials, cancel, cancel_isr, run};
#[instability::unstable]
pub use types::{
    CachedAp,
    CryptoRngWrap,
    DEFAULT_AP_GATEWAY,
    DEFAULT_POP,
    DEFAULT_PROV_NAME,
    DEFAULT_TIMEOUT_SECS,
    MAX_CACHED_APS,
    PROTO_VER_JSON,
    PROV_SERVICE_UUID_128,
    SESSION_RESPONSE_MIN_LEN,
    SESSION_RESPONSE_SEC0,
    cached_aps,
    set_cached_aps,
};
