use core::sync::atomic::Ordering;

use portable_atomic_enum::atomic_enum;

use super::WifiEvent;

/// Wi-Fi interface state.
#[atomic_enum]
#[derive(PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum WifiState {
    /// Station started.
    StaStarted,
    /// Station connected.
    StaConnected,
    /// Station disconnected.
    StaDisconnected,
    /// Station stopped
    StaStopped,

    /// Access point started.
    ApStarted,
    /// Access point stopped.
    ApStopped,

    /// Invalid Wi-Fi state.
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

/// Get the current state of the AP.
pub fn ap_state() -> WifiState {
    AP_STATE.load(Ordering::Relaxed)
}

/// Get the current state of the STA.
pub fn sta_state() -> WifiState {
    STA_STATE.load(Ordering::Relaxed)
}

pub(crate) fn update_state(event: WifiEvent, handled: bool) {
    match event {
        WifiEvent::StaConnected
        | WifiEvent::StaDisconnected
        | WifiEvent::StaStart
        | WifiEvent::StaStop => STA_STATE.store(WifiState::from(event), Ordering::Relaxed),

        WifiEvent::ApStart | WifiEvent::ApStop => {
            AP_STATE.store(WifiState::from(event), Ordering::Relaxed)
        }

        other => {
            if !handled {
                debug!("Unhandled event: {:?}", other)
            }
        }
    }
}

pub(crate) fn reset_ap_state() {
    AP_STATE.store(WifiState::Invalid, Ordering::Relaxed)
}

pub(crate) fn reset_sta_state() {
    STA_STATE.store(WifiState::Invalid, Ordering::Relaxed)
}

/// Returns the current state of the Wi-Fi stack.
///
/// This does not support AP-STA mode. Use one of `sta_state` or
/// `ap_state` instead.
#[instability::unstable]
pub fn wifi_state() -> WifiState {
    use super::WifiMode;
    match WifiMode::current() {
        Ok(WifiMode::Sta) => sta_state(),
        Ok(WifiMode::Ap) => ap_state(),
        _ => WifiState::Invalid,
    }
}
