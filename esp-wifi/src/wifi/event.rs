use alloc::boxed::Box;
use core::cell::RefCell;

use critical_section::Mutex;

use super::WifiEvent;

pub(crate) mod sealed {
    use super::*;

    pub trait Event {
        /// Get the static reference to the handler for this event.
        fn get_handler() -> &'static Mutex<RefCell<Option<Box<Handler<Self>>>>>;
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
/// # fn new_handler(_: &wifi_event_ap_stadisconnected_t) {}
/// event::wifi_event_ap_stadisconnected_t::update_handler(|prev, event| {
///     prev(event);
///     new_handler(event);
/// })
/// ```
// Implemented like this instead of free functions because the syntax would be
// ```
// event::update_handler::<event::wifi_event_ap_stadisconnected_t, _>(...)
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

macro_rules! impl_wifi_event_data {
    ($ty:path) => {
        pub use $ty;
        impl sealed::Event for $ty {
            fn get_handler() -> &'static Mutex<RefCell<Option<Box<Handler<Self>>>>> {
                static HANDLE: Mutex<RefCell<Option<Box<Handler<$ty>>>>> =
                    Mutex::new(RefCell::new(None));
                &HANDLE
            }
        }
    };
}

impl_wifi_event_data!(esp_wifi_sys::include::wifi_event_action_tx_status_t);
impl_wifi_event_data!(esp_wifi_sys::include::wifi_event_ap_probe_req_rx_t);
impl_wifi_event_data!(esp_wifi_sys::include::wifi_event_ap_staconnected_t);
impl_wifi_event_data!(esp_wifi_sys::include::wifi_event_ap_stadisconnected_t);
impl_wifi_event_data!(esp_wifi_sys::include::wifi_event_ap_wps_rg_fail_reason_t);
impl_wifi_event_data!(esp_wifi_sys::include::wifi_event_ap_wps_rg_pin_t);
impl_wifi_event_data!(esp_wifi_sys::include::wifi_event_ap_wps_rg_success_t);
impl_wifi_event_data!(esp_wifi_sys::include::wifi_event_bss_rssi_low_t);
impl_wifi_event_data!(esp_wifi_sys::include::wifi_event_ftm_report_t);
impl_wifi_event_data!(esp_wifi_sys::include::wifi_event_home_channel_change_t);
impl_wifi_event_data!(esp_wifi_sys::include::wifi_event_nan_receive_t);
impl_wifi_event_data!(esp_wifi_sys::include::wifi_event_nan_replied_t);
impl_wifi_event_data!(esp_wifi_sys::include::wifi_event_nan_svc_match_t);
impl_wifi_event_data!(esp_wifi_sys::include::wifi_event_ndp_confirm_t);
impl_wifi_event_data!(esp_wifi_sys::include::wifi_event_ndp_indication_t);
impl_wifi_event_data!(esp_wifi_sys::include::wifi_event_ndp_terminated_t);
impl_wifi_event_data!(esp_wifi_sys::include::wifi_event_neighbor_report_t);
impl_wifi_event_data!(esp_wifi_sys::include::wifi_event_roc_done_t);
impl_wifi_event_data!(esp_wifi_sys::include::wifi_event_sta_authmode_change_t);
impl_wifi_event_data!(esp_wifi_sys::include::wifi_event_sta_connected_t);
impl_wifi_event_data!(esp_wifi_sys::include::wifi_event_sta_disconnected_t);
impl_wifi_event_data!(esp_wifi_sys::include::wifi_event_sta_wps_er_pin_t);
impl_wifi_event_data!(esp_wifi_sys::include::wifi_event_sta_wps_er_success_t);

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
/// The pointer should be valid to cast to `Event`
pub(crate) unsafe fn handle_raw<Event: EventExt>(
    event_data: *mut crate::binary::c_types::c_void,
    event_data_size: usize,
) {
    debug_assert_eq!(
        event_data_size,
        core::mem::size_of::<Event>(),
        "wrong size event data"
    );
    handle::<Event>(unsafe { &*event_data.cast() })
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
        WifiEvent::WifiReady => {}
        WifiEvent::ScanDone => {}
        WifiEvent::StaStart => {}
        WifiEvent::StaStop => {}
        WifiEvent::StaConnected => unsafe {
            handle_raw::<wifi_event_sta_connected_t>(event_data, event_data_size)
        },
        WifiEvent::StaDisconnected => unsafe {
            handle_raw::<wifi_event_sta_disconnected_t>(event_data, event_data_size)
        },
        WifiEvent::StaAuthmodeChange => unsafe {
            handle_raw::<wifi_event_sta_authmode_change_t>(event_data, event_data_size)
        },
        WifiEvent::StaWpsErSuccess => unsafe {
            handle_raw::<wifi_event_sta_wps_er_success_t>(event_data, event_data_size)
        },
        WifiEvent::StaWpsErFailed => {}
        WifiEvent::StaWpsErTimeout => {}
        WifiEvent::StaWpsErPin => unsafe {
            handle_raw::<wifi_event_sta_wps_er_pin_t>(event_data, event_data_size)
        },
        WifiEvent::StaWpsErPbcOverlap => {}
        WifiEvent::ApStart => {}
        WifiEvent::ApStop => {}
        WifiEvent::ApStaconnected => unsafe {
            handle_raw::<wifi_event_ap_staconnected_t>(event_data, event_data_size)
        },
        WifiEvent::ApStadisconnected => unsafe {
            handle_raw::<wifi_event_ap_stadisconnected_t>(event_data, event_data_size)
        },
        WifiEvent::ApProbereqrecved => unsafe {
            handle_raw::<wifi_event_ap_probe_req_rx_t>(event_data, event_data_size)
        },
        WifiEvent::FtmReport => unsafe {
            handle_raw::<wifi_event_ftm_report_t>(event_data, event_data_size)
        },
        WifiEvent::StaBssRssiLow => unsafe {
            handle_raw::<wifi_event_bss_rssi_low_t>(event_data, event_data_size)
        },
        WifiEvent::ActionTxStatus => unsafe {
            handle_raw::<wifi_event_action_tx_status_t>(event_data, event_data_size)
        },
        WifiEvent::RocDone => unsafe {
            handle_raw::<wifi_event_roc_done_t>(event_data, event_data_size)
        },
        WifiEvent::StaBeaconTimeout => {}
        WifiEvent::ConnectionlessModuleWakeIntervalStart => {}
        WifiEvent::ApWpsRgSuccess => unsafe {
            handle_raw::<wifi_event_ap_wps_rg_success_t>(event_data, event_data_size)
        },
        WifiEvent::ApWpsRgFailed => unsafe {
            handle_raw::<wifi_event_ap_wps_rg_fail_reason_t>(event_data, event_data_size);
        },
        WifiEvent::ApWpsRgTimeout => {}
        WifiEvent::ApWpsRgPin => unsafe {
            handle_raw::<wifi_event_ap_wps_rg_pin_t>(event_data, event_data_size)
        },
        WifiEvent::ApWpsRgPbcOverlap => {}
        WifiEvent::ItwtSetup => {}
        WifiEvent::ItwtTeardown => {}
        WifiEvent::ItwtProbe => {}
        WifiEvent::ItwtSuspend => {}
        WifiEvent::TwtWakeup => {}
        WifiEvent::BtwtSetup => {}
        WifiEvent::BtwtTeardown => {}
        WifiEvent::NanStarted => {}
        WifiEvent::NanStopped => {}
        WifiEvent::NanSvcMatch => unsafe {
            handle_raw::<wifi_event_nan_svc_match_t>(event_data, event_data_size)
        },
        WifiEvent::NanReplied => unsafe {
            handle_raw::<wifi_event_nan_replied_t>(event_data, event_data_size)
        },
        WifiEvent::NanReceive => unsafe {
            handle_raw::<wifi_event_nan_receive_t>(event_data, event_data_size)
        },
        WifiEvent::NdpIndication => unsafe {
            handle_raw::<wifi_event_ndp_indication_t>(event_data, event_data_size)
        },
        WifiEvent::NdpConfirm => unsafe {
            handle_raw::<wifi_event_ndp_confirm_t>(event_data, event_data_size)
        },
        WifiEvent::NdpTerminated => unsafe {
            handle_raw::<wifi_event_ndp_terminated_t>(event_data, event_data_size)
        },
        WifiEvent::HomeChannelChange => unsafe {
            handle_raw::<wifi_event_home_channel_change_t>(event_data, event_data_size)
        },
        WifiEvent::StaNeighborRep => unsafe {
            handle_raw::<wifi_event_neighbor_report_t>(event_data, event_data_size)
        },
    }
}
