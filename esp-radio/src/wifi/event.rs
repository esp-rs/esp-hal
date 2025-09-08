//! Wi-Fi event handlers.

use alloc::boxed::Box;

use esp_sync::NonReentrantMutex;

use super::WifiEvent;
use crate::wifi::include::{
    wifi_event_sta_wps_er_success_t__bindgen_ty_1,
    wifi_ftm_report_entry_t,
};

pub(crate) mod sealed {
    use super::*;

    pub trait Event {
        /// Get the static reference to the handler for this event.
        fn handler() -> &'static NonReentrantMutex<Option<Box<Handler<Self>>>>;
        /// # Safety
        /// `ptr` must be a valid for casting to this event's inner event data.
        unsafe fn from_raw_event_data(ptr: *mut crate::binary::c_types::c_void) -> Self;
    }
}

/// The type of handlers of events.
#[instability::unstable]
pub type Handler<T> = dyn FnMut(&T) + Sync + Send;

fn default_handler<Event: 'static>() -> Box<Handler<Event>> {
    fn drop_ref<T>(_: &T) {}
    // perf: `drop_ref` is a ZST [function item](https://doc.rust-lang.org/reference/types/function-item.html)
    // so this doesn't actually allocate.
    Box::new(drop_ref)
}

/// Extension trait for setting handlers for an event.
///
/// Register a new event handler like:
///
/// ```rust, no_run
/// # use esp_radio::wifi::event::{self, *};
/// # fn new_handler(_: &ApStaConnected) {}
/// event::ApStaConnected::update_handler(|_cs, event| {
///     new_handler(event);
/// })
/// ```
// Implemented like this instead of free functions because the syntax would be
// ```
// event::update_handler::<event::ApStaConnected, _>(...)
// ```
#[instability::unstable]
pub trait EventExt: sealed::Event + Sized + 'static {
    /// Get the handler for this event, replacing it with the default handler.
    fn take_handler() -> Box<Handler<Self>> {
        Self::handler().with(|handler| handler.take().unwrap_or_else(default_handler::<Self>))
    }

    /// Set the handler for this event, returning the old handler.
    fn replace_handler<F: FnMut(&Self) + Sync + Send + 'static>(f: F) -> Box<Handler<Self>> {
        Self::handler().with(|handler| {
            handler
                .replace(Box::new(f))
                .unwrap_or_else(default_handler::<Self>)
        })
    }

    /// Atomic combination of [`Self::take_handler`] and
    /// [`Self::replace_handler`]. Use this to add a new handler which runs
    /// after the previously registered handlers.
    fn update_handler<F: FnMut(&Self) + Sync + Send + 'static>(mut f: F) {
        Self::handler().with(|handler| {
            let mut prev: Box<Handler<Self>> =
                handler.take().unwrap_or_else(default_handler::<Self>);
            handler.replace(Box::new(move |event| {
                prev(event);
                f(event)
            }));
        })
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
            fn handler() -> &'static NonReentrantMutex<Option<Box<Handler<Self>>>> {
                static HANDLE: NonReentrantMutex<Option<Box<Handler<$newtype>>>> =
                    NonReentrantMutex::new(None);
                &HANDLE
            }
        }
    };

    ($newtype:ident, $data:ident) => {
        use esp_wifi_sys::include::$data;
        /// See [`WifiEvent`].
        #[derive(Copy, Clone)]
        pub struct $newtype($data);
        impl sealed::Event for $newtype {
            unsafe fn from_raw_event_data(ptr: *mut crate::binary::c_types::c_void) -> Self {
                Self(unsafe { *ptr.cast() })
            }
            fn handler() -> &'static NonReentrantMutex<Option<Box<Handler<Self>>>> {
                static HANDLE: NonReentrantMutex<Option<Box<Handler<$newtype>>>> =
                    NonReentrantMutex::new(None);
                &HANDLE
            }
        }
    };
}

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
impl_wifi_event!(ApStaConnected, wifi_event_ap_staconnected_t);
impl_wifi_event!(ApStaDisconnected, wifi_event_ap_stadisconnected_t);
impl_wifi_event!(ApProbeReqReceived, wifi_event_ap_probe_req_rx_t);
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

impl ApStaConnected {
    /// Get the MAC address of the connected station.
    pub fn mac(&self) -> &[u8] {
        &self.0.mac
    }

    /// Get the AID (Association Identifier) of the connected station.
    pub fn aid(&self) -> u8 {
        self.0.aid
    }
}

impl ApStaDisconnected {
    /// Get the MAC address of the disconnected station.
    pub fn mac(&self) -> &[u8] {
        &self.0.mac
    }

    /// Get the reason for the disconnection.
    pub fn reason(&self) -> u16 {
        self.0.reason
    }
}

impl ScanDone {
    /// Get the status of the scan operation.
    pub fn status(&self) -> u32 {
        self.0.status
    }

    /// Get the number of found APs.
    pub fn number(&self) -> u8 {
        self.0.number
    }

    /// Get the scan ID associated with this scan operation.
    pub fn id(&self) -> u8 {
        self.0.scan_id
    }
}

impl StaConnected {
    /// Get the SSID of the connected station.
    pub fn ssid(&self) -> &[u8] {
        &self.0.ssid
    }

    /// Get the length of the SSID.
    pub fn ssid_len(&self) -> u8 {
        self.0.ssid_len
    }

    /// Get the BSSID (MAC address) of the connected station.
    pub fn bssid(&self) -> &[u8] {
        &self.0.bssid
    }

    /// Get the channel on which the station is connected.
    pub fn channel(&self) -> u8 {
        self.0.channel
    }

    /// Get the authentication mode used for the connection.
    pub fn authmode(&self) -> u32 {
        self.0.authmode
    }

    /// Get the AID (Association Identifier) of the connected station.
    pub fn aid(&self) -> u16 {
        self.0.aid
    }
}

impl StaDisconnected {
    /// Get the SSID of the disconnected station.
    pub fn ssid(&self) -> &[u8] {
        &self.0.ssid
    }

    /// Get the length of the SSID.
    pub fn ssid_len(&self) -> u8 {
        self.0.ssid_len
    }

    /// Get the BSSID (MAC address) of the disconnected station.
    pub fn bssid(&self) -> &[u8] {
        &self.0.bssid
    }

    /// Get the reason for the disconnection.
    pub fn reason(&self) -> u8 {
        self.0.reason
    }

    /// Get the authentication mode used for the disconnection.
    pub fn rssi(&self) -> i8 {
        self.0.rssi
    }
}

impl StaAuthmodeChange {
    /// Get the old authentication mode.
    pub fn old_mode(&self) -> u32 {
        self.0.old_mode
    }

    /// Get the new authentication mode.
    pub fn new_mode(&self) -> u32 {
        self.0.new_mode
    }
}

impl StaWpsErSuccess {
    /// Get number of AP credentials received.
    pub fn ap_cred_cnt(&self) -> u8 {
        self.0.ap_cred_cnt
    }

    /// Get all AP credentials received.
    pub fn ap_cred(&self) -> &[wifi_event_sta_wps_er_success_t__bindgen_ty_1] {
        &self.0.ap_cred
    }
}

impl StaWpsErPin {
    /// Get the PIN code received from the WPS.
    pub fn pin(&self) -> &[u8] {
        &self.0.pin_code
    }
}

impl FtmReport {
    /// Get the MAC address of the FTM peer.
    pub fn peer_mac(&self) -> &[u8] {
        &self.0.peer_mac
    }

    /// Get the status of the FTM operation.
    pub fn status(&self) -> u32 {
        self.0.status
    }

    /// Get the raw round-trip time (RTT) in nanoseconds.
    pub fn rtt_raw(&self) -> u32 {
        self.0.rtt_raw
    }

    /// Get the estimated round-trip time (RTT) in nanoseconds.
    pub fn rtt_est(&self) -> u32 {
        self.0.rtt_est
    }

    /// Get the distance estimate in centimeters.
    pub fn dist_est(&self) -> u32 {
        self.0.dist_est
    }

    /// Get Pointer to FTM Report, should be freed after use.
    pub fn report_data(&self) -> *mut wifi_ftm_report_entry_t {
        self.0.ftm_report_data
    }

    /// Get the number of entries in the FTM report data.
    pub fn report_num_entries(&self) -> u8 {
        self.0.ftm_report_num_entries
    }
}

impl ApProbeReqReceived {
    /// Get received probe request SSID.
    pub fn rssi(&self) -> i32 {
        self.0.rssi
    }

    /// Get the MAC address of the station which send probe request.
    pub fn mac(&self) -> &[u8] {
        &self.0.mac
    }
}

impl StaBssRssiLow {
    /// Get received probe request SSID of bss.
    pub fn rssi(&self) -> i32 {
        self.0.rssi
    }
}

impl ActionTxStatus {
    /// Get Wi-Fi interface to send request to.
    pub fn ifx(&self) -> u32 {
        self.0.ifx
    }

    /// Get context to identify the request.
    pub fn context(&self) -> u32 {
        self.0.context
    }

    /// Get destination MAC address.
    pub fn da(&self) -> &[u8] {
        &self.0.da
    }

    /// Get the status of the operation.
    pub fn status(&self) -> u8 {
        self.0.status
    }
}

impl RocDone {
    /// Get context to identify the request.
    pub fn context(&self) -> u32 {
        self.0.context
    }
}

impl ApWpsRgSuccess {
    /// Get enrollee mac address.
    pub fn peer_mac(&self) -> &[u8] {
        &self.0.peer_macaddr
    }
}

impl ApWpsRgFailed {
    /// Get WPS failure reason.
    pub fn reason(&self) -> u32 {
        self.0.reason
    }

    /// Get enrollee mac address.
    pub fn peer_macaddr(&self) -> &[u8; 6] {
        &self.0.peer_macaddr
    }
}

impl ApWpsRgPin {
    /// Get the PIN code of station in enrollee mode.
    pub fn pin_code(&self) -> &[u8] {
        &self.0.pin_code
    }
}

cfg_if::cfg_if! {
    if #[cfg(wifi_has_wifi6)] {
        use crate::wifi::include::wifi_twt_setup_config_t;

        impl ItwtSetup {
            /// Get the ITWT setup config, this value is determined by the AP.
            pub fn config(&self) -> &wifi_twt_setup_config_t {
                &self.0.config
            }

            /// Get the ITWT setup status, 1 indicates success, others indicate setup failure.
            pub fn status(&self) -> i32 {
                self.0.status
            }

            /// Get the ITWT setup frame tx fail reason.
            pub fn reason(&self) -> u8 {
                self.0.reason
            }

            /// Get TWT SP start time.
            pub fn target_wake_time(&self) -> u64 {
                self.0.target_wake_time
            }
        }

        impl ItwtTeardown {
            /// Get flow ID.
            pub fn flow_id(&self) -> u8 {
                self.0.flow_id
            }

            /// Get ITWT teardown status.
            pub fn status(&self) -> u32 {
                self.0.status
            }
        }

        impl ItwtProbe {
            /// Get probe status.
            pub fn status(&self) -> u32 {
                self.0.status
            }

            /// Get failure reason.
            pub fn reason(&self) -> u8 {
                self.0.reason
            }
        }

        impl ItwtSuspend {
            /// Get suspend status.
            pub fn status(&self) -> i32 {
                self.0.status
            }

            /// Get bitmap of the suspended flow ID.
            pub fn flow_id_bitmap(&self) -> u8 {
                self.0.flow_id_bitmap
            }

            /// Get the actual suspend time for each flow ID in milliseconds.
            pub fn actual_suspend_time_ms(&self) -> &[u32] {
                &self.0.actual_suspend_time_ms
            }
        }

        impl BtwtSetup {
            /// Get the BTWT setup status.
            pub fn status(&self) -> u32 {
                self.0.status
            }

            /// Get the type of TWT command.
            pub fn cmd(&self) -> u32 {
                self.0.setup_cmd
            }

            /// Get the TWT id.
            pub fn btwt_id(&self) -> u8 {
                self.0.btwt_id
            }

            /// Get the Nominal Minimum Wake Duration, indicates the minimum amount of time,
            /// in units of 256 µs, that the TWT requesting STA expects that it needs to be awake.
            pub fn min_wake_dura(&self) -> u8 {
                self.0.min_wake_dura
            }

            /// Get the TWT Wake Interval Exponent.
            pub fn wake_invl_expn(&self) -> u8 {
                self.0.wake_invl_expn
            }

            /// Get the TWT Wake Interval Mantissa.
            pub fn wake_invl_mant(&self) -> u16 {
                self.0.wake_invl_mant
            }

            /// Get whether this is a trigger-enabled TWT (true) or a non-trigger-enabled TWT (false).
            pub fn trigger(&self) -> bool {
                self.0.trigger
            }

            /// Get the TWT flow type, an announced TWT (true) or an unannounced TWT (false).
            pub fn flow_type(&self) -> u8 {
                self.0.flow_type
            }

            /// Get the BTWT setup frame tx fail reason.
            pub fn reason(&self) -> u8 {
                self.0.reason
            }

            /// Get the TWT SP start time.
            pub fn target_wake_time(&self) -> u64 {
                self.0.target_wake_time
            }
        }

        impl BtwtTeardown {
            /// Get the TWT ID.
            pub fn id(&self) -> u8 {
                self.0.btwt_id
            }

            /// Get the btwt teardown status.
            pub fn status(&self) -> u32 {
                self.0.status
            }
        }
    }
}

impl NanSvcMatch {
    /// Get the Subscribe Service ID.
    pub fn subscribe_id(&self) -> u8 {
        self.0.subscribe_id
    }

    /// Get the Publish Service ID.
    pub fn publish_id(&self) -> u8 {
        self.0.publish_id
    }

    /// Get the NAN Interface MAC of the Publisher.
    pub fn pub_if_mac(&self) -> &[u8] {
        &self.0.pub_if_mac
    }

    /// Indicates whether publisher’s service ID needs to be updated.
    pub fn update_pub_id(&self) -> bool {
        self.0.update_pub_id
    }
}

impl NanReplied {
    /// Get the Subscribe Service ID.
    pub fn subscribe_id(&self) -> u8 {
        self.0.subscribe_id
    }

    /// Get the Publish Service ID.
    pub fn publish_id(&self) -> u8 {
        self.0.publish_id
    }

    /// Get the NAN Interface MAC of the Subscriber.
    pub fn sub_if_mac(&self) -> &[u8] {
        &self.0.sub_if_mac
    }
}

impl NanReceive {
    /// Get Our Service Identifier.
    pub fn inst_id(&self) -> u8 {
        self.0.inst_id
    }

    /// Get Peer's Service Identifier.
    pub fn peer_inst_id(&self) -> u8 {
        self.0.peer_inst_id
    }

    /// Get Peer’s NAN Interface MAC
    pub fn peer_if_mac(&self) -> &[u8; 6] {
        &self.0.peer_if_mac
    }

    /// Get Peer Service Info.
    pub fn peer_svc_info(&self) -> &[u8; 64] {
        &self.0.peer_svc_info
    }
}

impl NdpIndication {
    /// Get Publish ID for NAN Service.
    pub fn publish_id(&self) -> u8 {
        self.0.publish_id
    }

    /// Get NDF instance ID.
    pub fn ndp_id(&self) -> u8 {
        self.0.ndp_id
    }

    /// Get Peer’s NAN Interface MAC.
    pub fn peer_nmi(&self) -> &[u8; 6] {
        &self.0.peer_nmi
    }

    /// Get Peer’s NAN Data Interface MAC.
    pub fn peer_ndi(&self) -> &[u8; 6] {
        &self.0.peer_ndi
    }

    /// Get Service Specific Info.
    pub fn svc_info(&self) -> &[u8; 64] {
        &self.0.svc_info
    }
}

impl NdpConfirm {
    /// Get NDP status code.
    pub fn status(&self) -> u8 {
        self.0.status
    }

    /// Get NDP instance ID.
    pub fn id(&self) -> u8 {
        self.0.ndp_id
    }

    /// Get Peer’s NAN Management Interface MAC.
    pub fn peer_nmi(&self) -> &[u8; 6] {
        &self.0.peer_nmi
    }

    /// Get Peer’s NAN Data Interface MAC.
    pub fn peer_ndi(&self) -> &[u8; 6] {
        &self.0.peer_ndi
    }

    /// Get Own NAN Data Interface MAC.
    pub fn own_ndi(&self) -> &[u8; 6] {
        &self.0.own_ndi
    }

    /// Get Service Specific Info.
    pub fn svc_info(&self) -> &[u8; 64] {
        &self.0.svc_info
    }
}

impl NdpTerminated {
    /// Get termination reason code.
    pub fn reason(&self) -> u8 {
        self.0.reason
    }

    /// Get NDP instance ID.
    pub fn id(&self) -> u8 {
        self.0.ndp_id
    }

    /// Get Initiator’s NAN Data Interface MAC
    pub fn init_ndi(&self) -> &[u8; 6] {
        &self.0.init_ndi
    }
}

impl HomeChannelChange {
    /// Get the old home channel of the device.
    pub fn old_chan(&self) -> u8 {
        self.0.old_chan
    }

    /// Get the old second channel of the device.
    pub fn old_snd(&self) -> u32 {
        self.0.old_snd
    }

    /// Get the new home channel of the device.
    pub fn new_chan(&self) -> u8 {
        self.0.new_chan
    }

    /// Get the new second channel of the device.
    pub fn new_snd(&self) -> u32 {
        self.0.new_snd
    }
}

impl StaNeighborRep {
    /// Get the Neighbor Report received from the AP.
    pub fn report(&self) -> &[u8] {
        &self.0.report[..self.0.report_len as usize]
    }

    /// Get the length of report.
    pub fn report_len(&self) -> u16 {
        self.0.report_len
    }
}

/// Handle the given event using the registered event handlers.
#[instability::unstable]
pub fn handle<Event: EventExt>(event_data: &Event) -> bool {
    Event::handler().with(|handler| {
        if let Some(handler) = handler {
            handler(event_data);
            true
        } else {
            false
        }
    })
}

/// Handle an event given the raw pointers.
///
/// # Safety
/// The pointer should be valid to cast to `Event`'s inner type (if it has one)
pub(crate) unsafe fn handle_raw<Event: EventExt>(
    event_data: *mut crate::binary::c_types::c_void,
    event_data_size: usize,
) -> bool {
    debug_assert_eq!(
        event_data_size,
        core::mem::size_of::<Event>(),
        "wrong size event data"
    );

    let event = unsafe { Event::from_raw_event_data(event_data) };
    handle::<Event>(&event)
}

/// Handle event regardless of its type.
///
/// # Safety
/// Arguments should be self-consistent.
#[rustfmt::skip]
pub(crate) unsafe fn dispatch_event_handler(
    event: WifiEvent,
    event_data: *mut crate::binary::c_types::c_void,
    event_data_size: usize,
) -> bool { unsafe {
    match event {
        WifiEvent::WifiReady => {
            handle_raw::<WifiReady>(event_data, event_data_size)
        }
        WifiEvent::ScanDone => {
            handle_raw::<ScanDone>(event_data, event_data_size)
        }
        WifiEvent::StaStart => {
            handle_raw::<StaStart>(event_data, event_data_size)
        }
        WifiEvent::StaStop => {
            handle_raw::<StaStop>(event_data, event_data_size)
        }
        WifiEvent::StaConnected => {
            handle_raw::<StaConnected>(event_data, event_data_size)
        }
        WifiEvent::StaDisconnected => {
            handle_raw::<StaDisconnected>(event_data, event_data_size)
        }
        WifiEvent::StaAuthmodeChange => {
            handle_raw::<StaAuthmodeChange>(event_data, event_data_size)
        }
        WifiEvent::StaWpsErSuccess => {
            handle_raw::<StaWpsErSuccess>(event_data, event_data_size)
        }
        WifiEvent::StaWpsErFailed => {
            handle_raw::<StaWpsErFailed>(event_data, event_data_size)
        }
        WifiEvent::StaWpsErTimeout => {
            handle_raw::<StaWpsErTimeout>(event_data, event_data_size)
        }
        WifiEvent::StaWpsErPin => {
            handle_raw::<StaWpsErPin>(event_data, event_data_size)
        }
        WifiEvent::StaWpsErPbcOverlap => {
            handle_raw::<StaWpsErPbcOverlap>(event_data, event_data_size)
        }
        WifiEvent::ApStart => {
            handle_raw::<ApStart>(event_data, event_data_size)
        }
        WifiEvent::ApStop => {
            handle_raw::<ApStop>(event_data, event_data_size)
        }
        WifiEvent::ApStaConnected => {
            handle_raw::<ApStaConnected>(event_data, event_data_size)
        }
        WifiEvent::ApStaDisconnected => {
            handle_raw::<ApStaDisconnected>(event_data, event_data_size)
        }
        WifiEvent::ApProbeReqReceived => {
            handle_raw::<ApProbeReqReceived>(event_data, event_data_size)
        }
        WifiEvent::FtmReport => {
            handle_raw::<FtmReport>(event_data, event_data_size)
        }
        WifiEvent::StaBssRssiLow => {
            handle_raw::<StaBssRssiLow>(event_data, event_data_size)
        }
        WifiEvent::ActionTxStatus => {
            handle_raw::<ActionTxStatus>(event_data, event_data_size)
        }
        WifiEvent::RocDone => {
            handle_raw::<RocDone>(event_data, event_data_size)
        }
        WifiEvent::StaBeaconTimeout => {
            handle_raw::<StaBeaconTimeout>(event_data, event_data_size)
        }
        WifiEvent::ConnectionlessModuleWakeIntervalStart => {
            handle_raw::<ConnectionlessModuleWakeIntervalStart>(event_data, event_data_size)
        }
        WifiEvent::ApWpsRgSuccess => {
            handle_raw::<ApWpsRgSuccess>(event_data, event_data_size)
        }
        WifiEvent::ApWpsRgFailed => {
            handle_raw::<ApWpsRgFailed>(event_data, event_data_size)
        }
        WifiEvent::ApWpsRgTimeout => {
            handle_raw::<ApWpsRgTimeout>(event_data, event_data_size)
        }
        WifiEvent::ApWpsRgPin => {
            handle_raw::<ApWpsRgPin>(event_data, event_data_size)
        }
        WifiEvent::ApWpsRgPbcOverlap => {
            handle_raw::<ApWpsRgPbcOverlap>(event_data, event_data_size)
        }
        WifiEvent::ItwtSetup => {
            handle_raw::<ItwtSetup>(event_data, event_data_size)
        }
        WifiEvent::ItwtTeardown => {
            handle_raw::<ItwtTeardown>(event_data, event_data_size)
        }
        WifiEvent::ItwtProbe => {
            handle_raw::<ItwtProbe>(event_data, event_data_size)
        }
        WifiEvent::ItwtSuspend => {
            handle_raw::<ItwtSuspend>(event_data, event_data_size)
        }
        WifiEvent::TwtWakeup => {
            handle_raw::<TwtWakeup>(event_data, event_data_size)
        }
        WifiEvent::BtwtSetup => {
            handle_raw::<BtwtSetup>(event_data, event_data_size)
        }
        WifiEvent::BtwtTeardown => {
            handle_raw::<BtwtTeardown>(event_data, event_data_size)
        }
        WifiEvent::NanStarted => {
            handle_raw::<NanStarted>(event_data, event_data_size)
        }
        WifiEvent::NanStopped => {
            handle_raw::<NanStopped>(event_data, event_data_size)
        }
        WifiEvent::NanSvcMatch => {
            handle_raw::<NanSvcMatch>(event_data, event_data_size)
        }
        WifiEvent::NanReplied => {
            handle_raw::<NanReplied>(event_data, event_data_size)
        }
        WifiEvent::NanReceive => {
            handle_raw::<NanReceive>(event_data, event_data_size)
        }
        WifiEvent::NdpIndication => {
            handle_raw::<NdpIndication>(event_data, event_data_size)
        }
        WifiEvent::NdpConfirm => {
            handle_raw::<NdpConfirm>(event_data, event_data_size)
        }
        WifiEvent::NdpTerminated => {
            handle_raw::<NdpTerminated>(event_data, event_data_size)
        }
        WifiEvent::HomeChannelChange => {
            handle_raw::<HomeChannelChange>(event_data, event_data_size)
        }
        WifiEvent::StaNeighborRep => {
            handle_raw::<StaNeighborRep>(event_data, event_data_size)
        }
    }
}}
