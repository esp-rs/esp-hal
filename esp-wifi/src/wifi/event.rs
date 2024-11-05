use alloc::boxed::Box;
use core::cell::RefCell;

use critical_section::Mutex;

use super::WifiEvent;

pub(crate) mod sealed {
    use super::*;

    pub trait Event {
        /// Get the static reference to the handler for this event.
        fn get_handler() -> &'static Mutex<RefCell<Option<Box<Handler<Self>>>>>;
        /// # Safety
        /// `ptr` must be a valid for casting to this event's inner event data.
        unsafe fn from_raw_event_data(ptr: *mut crate::binary::c_types::c_void) -> Self;
    }
}
/// The type of handlers of events.
pub type Handler<T> = dyn FnMut(&T) + Sync + Send;

fn default_handler<Event: 'static>() -> Box<Handler<Event>> {
    fn drop_ref<T>(_: &T) {}
    Box::new(drop_ref)
}

/// Extension trait for setting handlers for an event.
///
/// Register a new event handler like:
/// ```
/// # use esp_wifi::wifi::event::{self, *};
/// # fn new_handler(_: &ApStaconnected) {}
/// event::ApStaconnected::update_handler(|prev, event| {
///     prev(event);
///     new_handler(event);
/// })
/// ```
// Implemented like this instead of free functions because the syntax would be
// ```
// event::update_handler::<event::ApStaconnected, _>(...)
// ```
pub trait EventExt: sealed::Event + Sized + 'static {
    /// Get the handler for this event, replacing it with the default handler.
    fn take_handler() -> Box<Handler<Self>> {
        critical_section::with(|cs| {
            Self::get_handler()
                .borrow_ref_mut(cs)
                .take()
                .unwrap_or(default_handler::<Self>())
        })
    }
    /// Set the handler for this event, returning the old handler.
    fn replace_handler<F: FnMut(&Self) + Sync + Send + 'static>(f: F) -> Box<Handler<Self>> {
        critical_section::with(|cs| {
            Self::get_handler()
                .borrow_ref_mut(cs)
                .replace(Box::new(f))
                .unwrap_or(default_handler::<Self>())
        })
    }
    /// Atomic combination of [`take_handler`] and [`replace_handler`]. Use this
    /// to add a new handler while preserving the previous.
    fn update_handler<F: FnMut(&mut Handler<Self>, &Self) + Sync + Send + 'static>(mut f: F) {
        critical_section::with(move |cs| {
            let mut handler: Box<Handler<Self>> = Self::take_handler();
            Self::get_handler()
                .borrow_ref_mut(cs)
                .replace(Box::new(move |event| f(&mut handler, event)));
        });
    }
}
impl<T: sealed::Event + 'static> EventExt for T {}

macro_rules! impl_wifi_event {
    // no data
    ($newtype:ident) => {
        /// See [`WifiEvent`].
        #[derive(Copy, Clone)]
        pub struct $newtype;
        impl sealed::Event for $newtype {
            unsafe fn from_raw_event_data(_: *mut crate::binary::c_types::c_void) -> Self {
                Self
            }
            fn get_handler() -> &'static Mutex<RefCell<Option<Box<Handler<Self>>>>> {
                static HANDLE: Mutex<RefCell<Option<Box<Handler<$newtype>>>>> =
                    Mutex::new(RefCell::new(None));
                &HANDLE
            }
        }
    };
    // data
    ($newtype:ident, $data:ident) => {
        pub use esp_wifi_sys::include::$data;
        /// See [`WifiEvent`].
        #[derive(Copy, Clone)]
        pub struct $newtype(pub $data);
        impl sealed::Event for $newtype {
            unsafe fn from_raw_event_data(ptr: *mut crate::binary::c_types::c_void) -> Self {
                Self(unsafe { *ptr.cast() })
            }
            fn get_handler() -> &'static Mutex<RefCell<Option<Box<Handler<Self>>>>> {
                static HANDLE: Mutex<RefCell<Option<Box<Handler<$newtype>>>>> =
                    Mutex::new(RefCell::new(None));
                &HANDLE
            }
        }
    };
}

impl_wifi_event!(WifiReady);
impl_wifi_event!(ScanDone);
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
impl_wifi_event!(ItwtSetup);
impl_wifi_event!(ItwtTeardown);
impl_wifi_event!(ItwtProbe);
impl_wifi_event!(ItwtSuspend);
impl_wifi_event!(TwtWakeup);
impl_wifi_event!(BtwtSetup);
impl_wifi_event!(BtwtTeardown);
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

/// Handle the given event using the registered event handlers.
pub fn handle<Event: EventExt>(event_data: &Event) {
    critical_section::with(|cs| {
        if let Some(handler) = &mut *Event::get_handler().borrow_ref_mut(cs) {
            handler(event_data)
        }
    });
}

/// Handle an event given the raw pointers.
/// # Safety
/// The pointer should be valid to cast to `Event`'s inner type (if it has one)
pub(crate) unsafe fn handle_raw<Event: EventExt>(
    event_data: *mut crate::binary::c_types::c_void,
    event_data_size: usize,
) {
    debug_assert_eq!(
        event_data_size,
        core::mem::size_of::<Event>(),
        "wrong size event data"
    );
    handle::<Event>(unsafe { &Event::from_raw_event_data(event_data) })
}

/// Handle event regardless of its type.
/// # Safety
/// Arguments should be self-consistent.
pub(crate) unsafe fn dispatch_event_handler(
    event: WifiEvent,
    event_data: *mut crate::binary::c_types::c_void,
    event_data_size: usize,
) {
    match event {
        WifiEvent::WifiReady => {
            handle_raw::<WifiReady>(event_data, event_data_size);
        }
        WifiEvent::ScanDone => {
            handle_raw::<ScanDone>(event_data, event_data_size);
        }
        WifiEvent::StaStart => {
            handle_raw::<StaStart>(event_data, event_data_size);
        }
        WifiEvent::StaStop => {
            handle_raw::<StaStop>(event_data, event_data_size);
        }
        WifiEvent::StaConnected => {
            handle_raw::<StaConnected>(event_data, event_data_size);
        }
        WifiEvent::StaDisconnected => {
            handle_raw::<StaDisconnected>(event_data, event_data_size);
        }
        WifiEvent::StaAuthmodeChange => {
            handle_raw::<StaAuthmodeChange>(event_data, event_data_size);
        }
        WifiEvent::StaWpsErSuccess => {
            handle_raw::<StaWpsErSuccess>(event_data, event_data_size);
        }
        WifiEvent::StaWpsErFailed => {
            handle_raw::<StaWpsErFailed>(event_data, event_data_size);
        }
        WifiEvent::StaWpsErTimeout => {
            handle_raw::<StaWpsErTimeout>(event_data, event_data_size);
        }
        WifiEvent::StaWpsErPin => {
            handle_raw::<StaWpsErPin>(event_data, event_data_size);
        }
        WifiEvent::StaWpsErPbcOverlap => {
            handle_raw::<StaWpsErPbcOverlap>(event_data, event_data_size);
        }
        WifiEvent::ApStart => {
            handle_raw::<ApStart>(event_data, event_data_size);
        }
        WifiEvent::ApStop => {
            handle_raw::<ApStop>(event_data, event_data_size);
        }
        WifiEvent::ApStaconnected => {
            handle_raw::<ApStaconnected>(event_data, event_data_size);
        }
        WifiEvent::ApStadisconnected => {
            handle_raw::<ApStadisconnected>(event_data, event_data_size);
        }
        WifiEvent::ApProbereqrecved => {
            handle_raw::<ApProbereqrecved>(event_data, event_data_size);
        }
        WifiEvent::FtmReport => {
            handle_raw::<FtmReport>(event_data, event_data_size);
        }
        WifiEvent::StaBssRssiLow => {
            handle_raw::<StaBssRssiLow>(event_data, event_data_size);
        }
        WifiEvent::ActionTxStatus => {
            handle_raw::<ActionTxStatus>(event_data, event_data_size);
        }
        WifiEvent::RocDone => {
            handle_raw::<RocDone>(event_data, event_data_size);
        }
        WifiEvent::StaBeaconTimeout => {
            handle_raw::<StaBeaconTimeout>(event_data, event_data_size);
        }
        WifiEvent::ConnectionlessModuleWakeIntervalStart => {
            handle_raw::<ConnectionlessModuleWakeIntervalStart>(event_data, event_data_size);
        }
        WifiEvent::ApWpsRgSuccess => {
            handle_raw::<ApWpsRgSuccess>(event_data, event_data_size);
        }
        WifiEvent::ApWpsRgFailed => {
            handle_raw::<ApWpsRgFailed>(event_data, event_data_size);
        }
        WifiEvent::ApWpsRgTimeout => {
            handle_raw::<ApWpsRgTimeout>(event_data, event_data_size);
        }
        WifiEvent::ApWpsRgPin => {
            handle_raw::<ApWpsRgPin>(event_data, event_data_size);
        }
        WifiEvent::ApWpsRgPbcOverlap => {
            handle_raw::<ApWpsRgPbcOverlap>(event_data, event_data_size);
        }
        WifiEvent::ItwtSetup => {
            handle_raw::<ItwtSetup>(event_data, event_data_size);
        }
        WifiEvent::ItwtTeardown => {
            handle_raw::<ItwtTeardown>(event_data, event_data_size);
        }
        WifiEvent::ItwtProbe => {
            handle_raw::<ItwtProbe>(event_data, event_data_size);
        }
        WifiEvent::ItwtSuspend => {
            handle_raw::<ItwtSuspend>(event_data, event_data_size);
        }
        WifiEvent::TwtWakeup => {
            handle_raw::<TwtWakeup>(event_data, event_data_size);
        }
        WifiEvent::BtwtSetup => {
            handle_raw::<BtwtSetup>(event_data, event_data_size);
        }
        WifiEvent::BtwtTeardown => {
            handle_raw::<BtwtTeardown>(event_data, event_data_size);
        }
        WifiEvent::NanStarted => {
            handle_raw::<NanStarted>(event_data, event_data_size);
        }
        WifiEvent::NanStopped => {
            handle_raw::<NanStopped>(event_data, event_data_size);
        }
        WifiEvent::NanSvcMatch => {
            handle_raw::<NanSvcMatch>(event_data, event_data_size);
        }
        WifiEvent::NanReplied => {
            handle_raw::<NanReplied>(event_data, event_data_size);
        }
        WifiEvent::NanReceive => {
            handle_raw::<NanReceive>(event_data, event_data_size);
        }
        WifiEvent::NdpIndication => {
            handle_raw::<NdpIndication>(event_data, event_data_size);
        }
        WifiEvent::NdpConfirm => {
            handle_raw::<NdpConfirm>(event_data, event_data_size);
        }
        WifiEvent::NdpTerminated => {
            handle_raw::<NdpTerminated>(event_data, event_data_size);
        }
        WifiEvent::HomeChannelChange => {
            handle_raw::<HomeChannelChange>(event_data, event_data_size);
        }
        WifiEvent::StaNeighborRep => {
            handle_raw::<StaNeighborRep>(event_data, event_data_size);
        }
    }
}
