use core::sync::atomic::Ordering;

use portable_atomic_enum::atomic_enum;

use super::WifiEvent;

/// Wifi interface state
#[atomic_enum]
#[derive(PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum WifiState {
    StaStarted,
    StaConnected,
    StaDisconnected,
    StaStopped,

    ApStarted,
    ApStopped,

    Invalid,
}

impl From<WifiEvent> for WifiState {
    fn from(event: WifiEvent) -> WifiState {
        match event {
            WifiEvent::StaStart => WifiState::StaStarted,
            WifiEvent::StaConnected => WifiState::StaConnected,
            WifiEvent::StaDisconnected => WifiState::StaDisconnected,
            WifiEvent::StaStop => WifiState::StaStopped,
            WifiEvent::ApStart => WifiState::ApStarted,
            WifiEvent::ApStop => WifiState::ApStopped,
            _ => WifiState::Invalid,
        }
    }
}

pub(crate) static STA_STATE: AtomicWifiState = AtomicWifiState::new(WifiState::Invalid);
pub(crate) static AP_STATE: AtomicWifiState = AtomicWifiState::new(WifiState::Invalid);

/// Get the current state of the AP
pub fn get_ap_state() -> WifiState {
    AP_STATE.load(Ordering::Relaxed)
}

/// Get the current state of the STA
pub fn get_sta_state() -> WifiState {
    STA_STATE.load(Ordering::Relaxed)
}

pub(crate) fn update_state(event: WifiEvent) {
    match event {
        WifiEvent::StaConnected
        | WifiEvent::StaDisconnected
        | WifiEvent::StaStart
        | WifiEvent::StaStop => STA_STATE.store(WifiState::from(event), Ordering::Relaxed),

        WifiEvent::ApStart | WifiEvent::ApStop => {
            AP_STATE.store(WifiState::from(event), Ordering::Relaxed)
        }

        other => debug!("Unhandled event: {:?}", other),
    }
}

pub(crate) fn reset_ap_state() {
    AP_STATE.store(WifiState::Invalid, Ordering::Relaxed)
}

pub(crate) fn reset_sta_state() {
    STA_STATE.store(WifiState::Invalid, Ordering::Relaxed)
}

/// Returns the current state of the WiFi stack.
///
/// This does not support AP-STA mode. Use one of `get_sta_state` or
/// `get_ap_state` instead.
pub fn get_wifi_state() -> WifiState {
    use super::WifiMode;
    match WifiMode::current() {
        Ok(WifiMode::Sta) => get_sta_state(),
        Ok(WifiMode::Ap) => get_ap_state(),
        _ => WifiState::Invalid,
    }
}
