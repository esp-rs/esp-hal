use core::sync::atomic::Ordering;

use private::{AtomicWifiApState, AtomicWifiStaState};
pub use private::{WifiApState, WifiStaState};

use super::WifiEvent;

mod private {
    use portable_atomic_enum::atomic_enum;

    /// Wi-Fi interface for station state.
    #[atomic_enum]
    #[derive(PartialEq, Debug)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    #[non_exhaustive]
    pub enum WifiStaState {
        /// Station started.
        Started,
        /// Station connected.
        Connected,
        /// Station disconnected.
        Disconnected,
        /// Station stopped
        Stopped,
        /// Invalid state.
        Invalid,
    }

    /// Wi-Fi interface for access point state.
    #[atomic_enum]
    #[derive(PartialEq, Debug)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    #[non_exhaustive]
    pub enum WifiApState {
        /// Access point started.
        Started,
        /// Access point stopped.
        Stopped,
        /// Invalid state.
        Invalid,
    }
}

impl From<WifiEvent> for WifiStaState {
    fn from(event: WifiEvent) -> WifiStaState {
        match event {
            WifiEvent::StaStart => WifiStaState::Started,
            WifiEvent::StaConnected => WifiStaState::Connected,
            WifiEvent::StaDisconnected => WifiStaState::Disconnected,
            WifiEvent::StaStop => WifiStaState::Stopped,
            _ => WifiStaState::Invalid,
        }
    }
}

impl From<WifiEvent> for WifiApState {
    fn from(event: WifiEvent) -> WifiApState {
        match event {
            WifiEvent::ApStart => WifiApState::Started,
            WifiEvent::ApStop => WifiApState::Stopped,
            _ => WifiApState::Invalid,
        }
    }
}

pub(crate) static STA_STATE: AtomicWifiStaState = AtomicWifiStaState::new(WifiStaState::Invalid);
pub(crate) static AP_STATE: AtomicWifiApState = AtomicWifiApState::new(WifiApState::Invalid);

/// Get the current state of the AP.
pub fn ap_state() -> WifiApState {
    AP_STATE.load(Ordering::Relaxed)
}

/// Get the current state of the STA.
pub fn sta_state() -> WifiStaState {
    STA_STATE.load(Ordering::Relaxed)
}

pub(crate) fn update_state(event: WifiEvent, handled: bool) {
    match event {
        WifiEvent::StaConnected
        | WifiEvent::StaDisconnected
        | WifiEvent::StaStart
        | WifiEvent::StaStop => STA_STATE.store(WifiStaState::from(event), Ordering::Relaxed),

        WifiEvent::ApStart | WifiEvent::ApStop => {
            AP_STATE.store(WifiApState::from(event), Ordering::Relaxed)
        }

        other => {
            if !handled {
                debug!("Unhandled event: {:?}", other)
            }
        }
    }
}

pub(crate) fn reset_ap_state() {
    AP_STATE.store(WifiApState::Invalid, Ordering::Relaxed)
}

pub(crate) fn reset_sta_state() {
    STA_STATE.store(WifiStaState::Invalid, Ordering::Relaxed)
}
