use core::sync::atomic::Ordering;

use esp_radio_rtos_driver::semaphore::{SemaphoreHandle, SemaphoreKind};
use esp_sync::NonReentrantMutex;
use portable_atomic_enum::atomic_enum;

use super::event::WifiEvent;

/// Wi-Fi interface for station state.
#[atomic_enum]
#[derive(PartialEq, Debug, Clone, Copy, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub(crate) enum WifiStationState {
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
pub(crate) enum WifiAccessPointState {
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

static STATION_STATE: AtomicWifiStationState =
    AtomicWifiStationState::new(WifiStationState::Uninitialized);
static ACCESS_POINT_STATE: AtomicWifiAccessPointState =
    AtomicWifiAccessPointState::new(WifiAccessPointState::Uninitialized);

/// Get the current state of the access point.
pub(crate) fn access_point_state() -> WifiAccessPointState {
    ACCESS_POINT_STATE.load(Ordering::Relaxed)
}

/// Get the current state of the Station.
pub(crate) fn station_state() -> WifiStationState {
    STATION_STATE.load(Ordering::Relaxed)
}

pub(crate) fn update_state(event: WifiEvent) {
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

        _ => (),
    }
}

pub(crate) fn set_access_point_state(state: WifiAccessPointState) {
    ACCESS_POINT_STATE.store(state, Ordering::Relaxed)
}

pub(crate) fn set_station_state(state: WifiStationState) {
    STATION_STATE.store(state, Ordering::Relaxed)
}

pub(crate) fn locked<R>(f: impl FnOnce() -> R) -> R {
    static LOCK: NonReentrantMutex<Option<SemaphoreHandle>> = NonReentrantMutex::new(None);

    LOCK.with(|sem| {
        sem.get_or_insert_with(|| SemaphoreHandle::new(SemaphoreKind::Mutex))
            .take(None)
    });

    let res: R = f();

    LOCK.with(|sem| {
        sem.get_or_insert_with(|| SemaphoreHandle::new(SemaphoreKind::Mutex))
            .give()
    });

    res
}
