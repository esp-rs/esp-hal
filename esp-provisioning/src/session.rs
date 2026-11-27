//! Provisioning session shared by SoftAP and BLE.
//!
//! The application owns the Wi-Fi controller, network stacks, BLE connector,
//! and verify loop. [`Session::credentials`] yields submitted credentials for
//! each attempt and [`Session::report`] publishes the real STA outcome so
//! `GetStatus` reflects association + DHCP rather than an optimistic guess.
//! Dropping a serve future cancels that transport.

use core::{
    cell::RefCell,
    error::Error,
    fmt::{self, Debug, Display},
    net::Ipv4Addr,
};

use embassy_sync::{
    blocking_mutex::{Mutex, raw::CriticalSectionRawMutex},
    signal::Signal,
    watch::Watch,
};
use esp_radio::wifi::{Ssid, sta::ConnectedInfo};
use heapless::String;
use procmacros::BuilderLite;

pub use crate::state::StaFailReason;
use crate::state::{Phase, SessionCore};

/// Station outcome the application reports after it tries the provisioned network.
#[derive(Debug, Clone, PartialEq, Eq)]
#[instability::unstable]
pub enum StaState {
    /// Join is still in progress.
    Connecting,
    /// STA associated and acquired an IPv4 address via DHCP.
    Connected {
        /// IPv4 address obtained by the station interface.
        ipv4: Ipv4Addr,
        /// Access point metadata returned by `WifiController::connect_async`.
        info: ConnectedInfo,
    },
    /// Join or DHCP failed.
    ///
    /// Transports stay alive so the phone can send `prov-ctrl` `CtrlReset` and
    /// submit corrected credentials on the same link.
    Failed(StaFailReason),
}

#[cfg(feature = "defmt")]
impl defmt::Format for StaState {
    fn format(&self, fmt: defmt::Formatter<'_>) {
        match self {
            Self::Connecting => defmt::write!(fmt, "StaState::Connecting"),
            Self::Connected { ipv4, info } => {
                let [a, b, c, d] = ipv4.octets();
                defmt::write!(
                    fmt,
                    "StaState::Connected {{ ipv4: {}.{}.{}.{}, info: {} }}",
                    a,
                    b,
                    c,
                    d,
                    info
                );
            }
            Self::Failed(reason) => defmt::write!(fmt, "StaState::Failed({})", reason),
        }
    }
}

/// Snapshot of a connected station stored in [`Session`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct ConnectedSnapshot {
    pub ipv4: Ipv4Addr,
    pub info: ConnectedInfo,
}

/// Snapshot of [`Session`] observed by `GetStatus`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) enum SessionStatus {
    Idle,
    Connecting,
    Connected(ConnectedSnapshot),
    Failed(StaFailReason),
}

/// Wi-Fi credentials from `SetConfig` / `ApplyConfig`.
///
/// [`Ssid`] can be copied into STA config. The passphrase stays a [`String`]
/// because `esp_radio::wifi::Password` does not expose its bytes; convert with
/// `Password::try_from(password.as_str())` when joining.
#[derive(Clone)]
#[instability::unstable]
pub struct WifiCredentials {
    /// Network name.
    pub ssid: Ssid,
    /// WPA passphrase. Empty for an open network.
    pub password: String<64>,
}

impl Debug for WifiCredentials {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("WifiCredentials")
            .field("ssid", &self.ssid)
            .field("password", &"**REDACTED**")
            .finish()
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for WifiCredentials {
    fn format(&self, fmt: defmt::Formatter<'_>) {
        defmt::write!(
            fmt,
            "WifiCredentials {{ ssid: {}, password: **REDACTED** }}",
            self.ssid
        );
    }
}

/// Why `softap::serve` or `ble::serve` returned an error.
/// Dropping a serve future is the cancel.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub enum ProvError {
    /// The handshake failed on the wire.
    ///
    /// HTTP answers 400 with an empty body. BLE rejects the ATT write.
    /// `serve` stays up for the next attempt.
    Handshake,
    /// The HTTP listener or the BLE runner stopped.
    Transport,
    /// A protobuf body could not be decoded or encoded.
    ///
    /// HTTP answers 400 with an empty body. BLE rejects the ATT write.
    /// `serve` stays up for the next attempt.
    Protocol,
}

impl Display for ProvError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::Handshake => "The sec1 handshake failed.",
            Self::Transport => "The provisioning transport stopped.",
            Self::Protocol => "The provisioning protobuf could not be decoded or encoded.",
        })
    }
}

impl Error for ProvError {}

/// Proof-of-possession, SoftAP name, BLE name, and gateway.
///
/// The radio, embassy stack, BLE connector, and RNG are arguments of the serve
/// functions. Empty strings are not sentinels.
#[derive(Clone, PartialEq, Eq, Hash, BuilderLite)]
#[instability::unstable]
pub struct Config {
    /// Proof-of-possession mixed into the sec1 shared secret.
    #[builder_lite(reference)]
    pop: String<64>,
    /// SoftAP SSID. Already length-checked by [`Ssid`].
    softap_ssid: Ssid,
    /// BLE complete local name.
    #[builder_lite(reference)]
    ble_name: String<29>,
    /// IPv4 address of the SoftAP and the captive-portal DNS answer.
    gateway: Ipv4Addr,
}

impl Debug for Config {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Config")
            .field("pop", &"**REDACTED**")
            .field("softap_ssid", &self.softap_ssid)
            .field("ble_name", &self.ble_name)
            .field("gateway", &self.gateway)
            .finish()
    }
}

impl Default for Config {
    fn default() -> Self {
        let mut pop = String::new();
        unwrap!(pop.push_str("abcd1234"));
        let mut ble_name = String::new();
        unwrap!(ble_name.push_str("PROV_esp"));
        Self {
            pop,
            softap_ssid: unwrap!(Ssid::try_from("PROV_esp")),
            ble_name,
            gateway: Ipv4Addr::new(192, 168, 4, 1),
        }
    }
}

/// Shared provisioning state for one or both transports.
///
/// The proof of possession lives on [`Config`]. `serve` reads it from there.
#[instability::unstable]
pub struct Session {
    core: Mutex<CriticalSectionRawMutex, RefCell<SessionCore<WifiCredentials, ConnectedSnapshot>>>,
    creds_ready: Signal<CriticalSectionRawMutex, ()>,
    /// Both transports and the verify loop can wait on the same finish signal.
    finished: Watch<CriticalSectionRawMutex, (), 3>,
}

impl Default for Session {
    fn default() -> Self {
        Self::new()
    }
}

impl Session {
    /// Creates a new idle session.
    ///
    /// Credentials arrive from the phone via `softap::serve` or `ble::serve`.
    /// The application awaits [`Self::credentials`] and reports each outcome
    /// with [`Self::report`].
    #[instability::unstable]
    pub fn new() -> Self {
        Self {
            core: Mutex::new(RefCell::new(SessionCore::new())),
            creds_ready: Signal::new(),
            finished: Watch::new(),
        }
    }

    /// Waits until the phone applies credentials, then transitions the session to connecting.
    ///
    /// If the previous attempt ended with [`StaState::Failed`], calling this again waits
    /// until the phone sends `prov-ctrl` `CtrlReset` and submits a new `ApplyConfig`.
    /// The serve futures stay alive so a later [`Self::report`] can answer `GetStatus`.
    ///
    /// # Cancellation Safety
    ///
    /// Dropping this future while waiting leaves the session unchanged. The next call
    /// observes the same pending credentials.
    #[instability::unstable]
    pub async fn credentials(&self) -> WifiCredentials {
        loop {
            if let Some(creds) = self.core.lock(|core| core.borrow_mut().take_pending()) {
                return creds;
            }
            self.creds_ready.wait().await;
        }
    }

    /// Records the STA join outcome for subsequent `GetStatus` polls.
    ///
    /// Reporting [`StaState::Failed`] keeps the transports alive for `prov-ctrl`
    /// `CtrlReset` and retry. Reporting [`StaState::Connected`] lets the transport
    /// deliver `Connected` + `WifiConnectedState` before finishing.
    #[instability::unstable]
    pub fn report(&self, state: StaState) {
        self.core.lock(|core| {
            let mut core = core.borrow_mut();
            match state {
                StaState::Connecting => core.report_connecting(),
                StaState::Connected { ipv4, info } => {
                    core.report_connected(ConnectedSnapshot { ipv4, info });
                }
                StaState::Failed(reason) => core.report_failed(reason),
            }
        });
    }

    pub(crate) fn submit(&self, creds: WifiCredentials) -> bool {
        let fresh = self.core.lock(|core| core.borrow_mut().submit(creds));
        if fresh {
            self.creds_ready.signal(());
        }
        fresh
    }

    pub(crate) fn reset_on_failure(&self) -> bool {
        self.core.lock(|core| core.borrow_mut().reset_on_failure())
    }

    pub(crate) fn status(&self) -> SessionStatus {
        self.core.lock(|core| match core.borrow().phase() {
            Phase::Idle => SessionStatus::Idle,
            Phase::Pending(_) | Phase::Connecting => SessionStatus::Connecting,
            Phase::Failed(reason) => SessionStatus::Failed(*reason),
            Phase::Connected(info) => SessionStatus::Connected(info.clone()),
        })
    }

    pub(crate) fn finish(&self) {
        self.finished.sender().send(());
    }

    pub(crate) async fn wait_finished(&self) {
        let mut receiver = self
            .finished
            .receiver()
            .expect("too many provisioning finish waiters");
        receiver.changed().await;
    }
}
