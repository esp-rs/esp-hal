//! Crate-private event handling implementation.

use alloc::boxed::Box;

use esp_hal::sync::Locked;

use crate::wifi::WifiEvent;

pub(crate) mod sealed {
    use super::*;
    pub trait Event {
        fn handler() -> &'static Locked<Option<Box<Handler<Self>>>>;
        unsafe fn from_raw_event_data(ptr: *mut crate::binary::c_types::c_void) -> Self;
    }
}

pub(crate) type Handler<T> = dyn FnMut(&T) + Sync + Send;

fn default_handler<Event: 'static>() -> Box<Handler<Event>> {
    fn drop_ref<T>(_: &T) {}
    Box::new(drop_ref)
}

// This extension trait provides the internal handler management.
pub(crate) trait EventExt: sealed::Event + Sized + 'static {
    fn take_handler() -> Box<Handler<Self>> {
        Self::handler().with(|handler| handler.take().unwrap_or_else(default_handler::<Self>))
    }
    fn replace_handler(f: impl FnMut(&Self) + Sync + Send + 'static) -> Box<Handler<Self>> {
        Self::handler().with(|handler| {
            handler
                .replace(Box::new(f))
                .unwrap_or_else(default_handler::<Self>)
        })
    }
    fn update_handler(mut f: impl FnMut(&Self) + Sync + Send + 'static) {
        Self::handler().with(|handler| {
            let mut prev = handler.take().unwrap_or_else(default_handler::<Self>);
            handler.replace(Box::new(move |event| {
                prev(event);
                f(event)
            }));
        })
    }
}
impl<T: sealed::Event + 'static> EventExt for T {}

// Macros to generate the internal wrapper structs around C types.
macro_rules! impl_wifi_event_accessors {
    (ApStaconnected, wifi_event_ap_staconnected_t) => {
        pub fn mac(&self) -> &[u8; 6] {
            &self.0.mac
        }
        pub fn aid(&self) -> u8 {
            self.0.aid
        }
    };
    (ApStadisconnected, wifi_event_ap_stadisconnected_t) => {
        pub fn mac(&self) -> &[u8; 6] {
            &self.0.mac
        }
        pub fn reason(&self) -> u16 {
            self.0.reason
        }
    };
    ($_newtype:ident, $_data:ident) => {};
}

macro_rules! impl_wifi_event {
    ($newtype:ident) => {
        #[derive(Copy, Clone)]
        pub(crate) struct $newtype;
        impl sealed::Event for $newtype {
            unsafe fn from_raw_event_data(_: *mut crate::binary::c_types::c_void) -> Self {
                Self
            }
            fn handler() -> &'static Locked<Option<Box<Handler<Self>>>> {
                static HANDLE: Locked<Option<Box<Handler<$newtype>>>> = Locked::new(None);
                &HANDLE
            }
        }
    };
    ($newtype:ident, $data:ident) => {
        pub(crate) use esp_wifi_sys::include::$data;
        #[derive(Copy, Clone)]
        #[allow(unused)]
        pub(crate) struct $newtype(pub(crate) $data);
        impl sealed::Event for $newtype {
            unsafe fn from_raw_event_data(ptr: *mut crate::binary::c_types::c_void) -> Self {
                Self(unsafe { *ptr.cast() })
            }
            fn handler() -> &'static Locked<Option<Box<Handler<Self>>>> {
                static HANDLE: Locked<Option<Box<Handler<$newtype>>>> = Locked::new(None);
                &HANDLE
            }
        }
        impl $newtype {
            impl_wifi_event_accessors!($newtype, $data);
        }
    };
}

// Define all the internal event structs.
impl_wifi_event!(WifiReady);
impl_wifi_event!(ScanDone, wifi_event_sta_scan_done_t);
impl_wifi_event!(StaStart);
impl_wifi_event!(StaStop);
impl_wifi_event!(StaConnected, wifi_event_sta_connected_t);
impl_wifi_event!(StaDisconnected, wifi_event_sta_disconnected_t);
impl_wifi_event!(StaAuthmodeChange, wifi_event_sta_authmode_change_t);
impl_wifi_event!(StaWpsErSuccess, wifi_event_sta_wps_er_success_t);
impl_wifi_event!(StaWpsErFailed);
impl_wifi_event!(StaWpsErTimeout);
impl_wifi_event!(StaWpsErPin, wifi_event_sta_wps_er_pin_t);
impl_wifi_event!(StaWpsErPbcOverlap);
impl_wifi_event!(ApStart);
impl_wifi_event!(ApStop);
impl_wifi_event!(ApStaconnected, wifi_event_ap_staconnected_t);
impl_wifi_event!(ApStadisconnected, wifi_event_ap_stadisconnected_t);
impl_wifi_event!(ApProbereqrecved, wifi_event_ap_probe_req_rx_t);
impl_wifi_event!(FtmReport, wifi_event_ftm_report_t);
impl_wifi_event!(StaBssRssiLow, wifi_event_bss_rssi_low_t);
impl_wifi_event!(ActionTxStatus, wifi_event_action_tx_status_t);
impl_wifi_event!(RocDone, wifi_event_roc_done_t);
impl_wifi_event!(StaBeaconTimeout);
impl_wifi_event!(ConnectionlessModuleWakeIntervalStart);
impl_wifi_event!(ApWpsRgSuccess, wifi_event_ap_wps_rg_success_t);
impl_wifi_event!(ApWpsRgFailed, wifi_event_ap_wps_rg_fail_reason_t);
impl_wifi_event!(ApWpsRgTimeout);
impl_wifi_event!(ApWpsRgPin, wifi_event_ap_wps_rg_pin_t);
impl_wifi_event!(ApWpsRgPbcOverlap);
cfg_if::cfg_if! {
    if #[cfg(wifi_has_wifi6)] {
        impl_wifi_event!(ItwtSetup, wifi_event_sta_itwt_setup_t);
        impl_wifi_event!(ItwtTeardown, wifi_event_sta_itwt_teardown_t);
        impl_wifi_event!(ItwtProbe, wifi_event_sta_itwt_probe_t);
        impl_wifi_event!(ItwtSuspend, wifi_event_sta_itwt_suspend_t);
        impl_wifi_event!(TwtWakeup);
        impl_wifi_event!(BtwtSetup, wifi_event_sta_btwt_setup_t);
        impl_wifi_event!(BtwtTeardown, wifi_event_sta_btwt_teardown_t);
    } else {
        impl_wifi_event!(ItwtSetup);
        impl_wifi_event!(ItwtTeardown);
        impl_wifi_event!(ItwtProbe);
        impl_wifi_event!(ItwtSuspend);
        impl_wifi_event!(TwtWakeup);
        impl_wifi_event!(BtwtSetup);
        impl_wifi_event!(BtwtTeardown);
    }
}
impl_wifi_event!(NanStarted);
impl_wifi_event!(NanStopped);
impl_wifi_event!(NanSvcMatch, wifi_event_nan_svc_match_t);
impl_wifi_event!(NanReplied, wifi_event_nan_replied_t);
impl_wifi_event!(NanReceive, wifi_event_nan_receive_t);
impl_wifi_event!(NdpIndication, wifi_event_ndp_indication_t);
impl_wifi_event!(NdpConfirm, wifi_event_ndp_confirm_t);
impl_wifi_event!(NdpTerminated, wifi_event_ndp_terminated_t);
impl_wifi_event!(HomeChannelChange, wifi_event_home_channel_change_t);
impl_wifi_event!(StaNeighborRep, wifi_event_neighbor_report_t);

unsafe fn handle_raw<E: EventExt>(event_data: *mut crate::binary::c_types::c_void) -> bool {
    let event = unsafe { E::from_raw_event_data(event_data) };
    E::handler().with(|handler| {
        if let Some(h) = handler.as_mut() {
            h(&event);
            true
        } else {
            false
        }
    })
}

macro_rules! impl_dispatch {
    ($($event_type:ident => $internal_struct:ident),+ $(,)?) => {
        pub(crate) unsafe fn dispatch_event_handler(
            event: WifiEvent,
            event_data: *mut crate::binary::c_types::c_void,
        ) -> bool {
            unsafe {
                match event {
                    $(
                        WifiEvent::$event_type => handle_raw::<$internal_struct>(event_data),
                    )+
                }
            }
        }
    }
}

// Map the enum variant to the internal struct type.
impl_dispatch! {
    WifiReady => WifiReady,
    ScanDone => ScanDone,
    StaStart => StaStart,
    StaStop => StaStop,
    StaConnected => StaConnected,
    StaDisconnected => StaDisconnected,
    StaAuthmodeChange => StaAuthmodeChange,
    StaWpsErSuccess => StaWpsErSuccess,
    StaWpsErFailed => StaWpsErFailed,
    StaWpsErTimeout => StaWpsErTimeout,
    StaWpsErPin => StaWpsErPin,
    StaWpsErPbcOverlap => StaWpsErPbcOverlap,
    ApStart => ApStart,
    ApStop => ApStop,
    ApStaconnected => ApStaconnected,
    ApStadisconnected => ApStadisconnected,
    ApProbereqrecved => ApProbereqrecved,
    FtmReport => FtmReport,
    StaBssRssiLow => StaBssRssiLow,
    ActionTxStatus => ActionTxStatus,
    RocDone => RocDone,
    StaBeaconTimeout => StaBeaconTimeout,
    ConnectionlessModuleWakeIntervalStart => ConnectionlessModuleWakeIntervalStart,
    ApWpsRgSuccess => ApWpsRgSuccess,
    ApWpsRgFailed => ApWpsRgFailed,
    ApWpsRgTimeout => ApWpsRgTimeout,
    ApWpsRgPin => ApWpsRgPin,
    ApWpsRgPbcOverlap => ApWpsRgPbcOverlap,
    ItwtSetup => ItwtSetup,
    ItwtTeardown => ItwtTeardown,
    ItwtProbe => ItwtProbe,
    ItwtSuspend => ItwtSuspend,
    TwtWakeup => TwtWakeup,
    BtwtSetup => BtwtSetup,
    BtwtTeardown => BtwtTeardown,
    NanStarted => NanStarted,
    NanStopped => NanStopped,
    NanSvcMatch => NanSvcMatch,
    NanReplied => NanReplied,
    NanReceive => NanReceive,
    NdpIndication => NdpIndication,
    NdpConfirm => NdpConfirm,
    NdpTerminated => NdpTerminated,
    HomeChannelChange => HomeChannelChange,
    StaNeighborRep => StaNeighborRep,
}
