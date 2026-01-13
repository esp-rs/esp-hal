use core::sync::atomic::Ordering;

use private::{AtomicWifiAccessPointState, AtomicWifiStationState};
pub(crate) use private::{WifiAccessPointState, WifiStationState};

use super::WifiEvent;

mod private {
    use portable_atomic_enum::atomic_enum;

    /// Wi-Fi interface for station state.
    #[atomic_enum]
    #[derive(PartialEq, Debug, Clone, Copy, Hash)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    #[non_exhaustive]
    pub enum WifiStationState {
        /// Start initiated.
        Starting,
        /// Station started.
        Started,
        /// Connect initiated.
        Connecting,
        /// Station connected.
        Connected,
        /// Disconnect initiated.
        Disconnecting,
        /// Station disconnected.
        Disconnected,
        /// Stop initiated.
        Stopping,
        /// Station stopped
        Stopped,
        /// Uninitialized state.
        Uninitialized,
    }

    /// Wi-Fi interface for access point state.
    #[atomic_enum]
    #[derive(PartialEq, Debug, Clone, Copy, Hash)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    #[non_exhaustive]
    pub enum WifiAccessPointState {
        /// Start initiated.
        Starting,
        /// Access point started.
        Started,
        /// Stop initiated.
        Stopping,
        /// Access point stopped.
        Stopped,
        /// Uninitialized state.
        Uninitialized,
    }
}

impl From<WifiEvent> for WifiStationState {
    fn from(event: WifiEvent) -> WifiStationState {
        match event {
            WifiEvent::StationStart => WifiStationState::Started,
            WifiEvent::StationConnected => WifiStationState::Connected,
            WifiEvent::StationDisconnected => WifiStationState::Disconnected,
            WifiEvent::StationStop => WifiStationState::Stopped,
            _ => WifiStationState::Uninitialized,
        }
    }
}

impl From<WifiEvent> for WifiAccessPointState {
    fn from(event: WifiEvent) -> WifiAccessPointState {
        match event {
            WifiEvent::AccessPointStart => WifiAccessPointState::Started,
            WifiEvent::AccessPointStop => WifiAccessPointState::Stopped,
            _ => WifiAccessPointState::Uninitialized,
        }
    }
}

pub(crate) static STATION_STATE: AtomicWifiStationState =
    AtomicWifiStationState::new(WifiStationState::Uninitialized);
pub(crate) static ACCESS_POINT_STATE: AtomicWifiAccessPointState =
    AtomicWifiAccessPointState::new(WifiAccessPointState::Uninitialized);

/// Get the current state of the access point.
pub(crate) fn access_point_state() -> WifiAccessPointState {
    ACCESS_POINT_STATE.load(Ordering::Relaxed)
}

/// Get the current state of the Station.
pub(crate) fn station_state() -> WifiStationState {
    STATION_STATE.load(Ordering::Relaxed)
}

pub(crate) fn update_state(event: WifiEvent, handled: bool) {
    match event {
        WifiEvent::StationConnected
        | WifiEvent::StationDisconnected
        | WifiEvent::StationStart
        | WifiEvent::StationStop => {
            STATION_STATE.store(WifiStationState::from(event), Ordering::Relaxed)
        }

        WifiEvent::AccessPointStart | WifiEvent::AccessPointStop => {
            ACCESS_POINT_STATE.store(WifiAccessPointState::from(event), Ordering::Relaxed)
        }

        other => {
            if !handled {
                debug!("Unhandled event: {:?}", other)
            }
        }
    }
}

pub(crate) fn set_access_point_state(state: WifiAccessPointState) {
    ACCESS_POINT_STATE.store(state, Ordering::Relaxed)
}

pub(crate) fn set_station_state(state: WifiStationState) {
    STATION_STATE.store(state, Ordering::Relaxed)
}
