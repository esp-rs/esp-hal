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
        unsafe fn from_raw_event_data(ptr: *mut crate::sys::c_types::c_void) -> Self;
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

#[procmacros::doc_replace]
/// Extension trait for setting handlers for an event.
///
/// Register a new event handler like:
///
/// ```rust, no_run
/// # {before_snippet}
/// use esp_radio::wifi::event::{self, EventExt};
/// event::AccessPointStationConnected::update_handler(|event| {
///     // ... handle AccessPointStationConnected event here
/// });
/// # {after_snippet}
/// ```
// Implemented like this instead of free functions because the syntax would be
// ```
// event::update_handler::<event::AccessPointStationConnected, _>(...)
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
            unsafe fn from_raw_event_data(_: *mut crate::sys::c_types::c_void) -> Self {
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
        use crate::sys::include::$data;
        /// See [`WifiEvent`].
        #[derive(Copy, Clone)]
        pub struct $newtype<'a>(&'a $data);

        impl sealed::Event for $newtype<'_> {
            unsafe fn from_raw_event_data(ptr: *mut crate::sys::c_types::c_void) -> Self {
                Self(unsafe { &*ptr.cast() })
            }

            fn handler() -> &'static NonReentrantMutex<Option<Box<Handler<Self>>>> {
                static HANDLE: NonReentrantMutex<Option<Box<Handler<$newtype<'_>>>>> =
                    NonReentrantMutex::new(None);
                &HANDLE
            }
        }
    };
}

impl_wifi_event!(WifiReady);
impl_wifi_event!(ScanDone, wifi_event_sta_scan_done_t);
impl_wifi_event!(StationStart);
impl_wifi_event!(StationStop);
impl_wifi_event!(StationConnected, wifi_event_sta_connected_t);
impl_wifi_event!(StationDisconnected, wifi_event_sta_disconnected_t);
impl_wifi_event!(
    StationAuthenticationModeChange,
    wifi_event_sta_authmode_change_t
);
impl_wifi_event!(
    StationWifiProtectedStatusEnrolleeSuccess,
    wifi_event_sta_wps_er_success_t
);
impl_wifi_event!(StationWifiProtectedStatusEnrolleeFailed);
impl_wifi_event!(StationWifiProtectedStatusEnrolleeTimeout);
impl_wifi_event!(
    StationWifiProtectedStatusEnrolleePin,
    wifi_event_sta_wps_er_pin_t
);
impl_wifi_event!(StationWifiProtectedStatusEnrolleePushButtonConfigurationOverlap);
impl_wifi_event!(AccessPointStart);
impl_wifi_event!(AccessPointStop);
impl_wifi_event!(AccessPointStationConnected, wifi_event_ap_staconnected_t);
impl_wifi_event!(
    AccessPointStationDisconnected,
    wifi_event_ap_stadisconnected_t
);
impl_wifi_event!(
    AccessPointProbeRequestReceived,
    wifi_event_ap_probe_req_rx_t
);
impl_wifi_event!(FineTimingMeasurementReport, wifi_event_ftm_report_t);
impl_wifi_event!(
    StationBasicServiceSetReceivedSignalStrengthIndicatorLow,
    wifi_event_bss_rssi_low_t
);
impl_wifi_event!(ActionTransmissionStatus, wifi_event_action_tx_status_t);
impl_wifi_event!(RemainOnChannelDone, wifi_event_roc_done_t);
impl_wifi_event!(StationBeaconTimeout);
impl_wifi_event!(ConnectionlessModuleWakeIntervalStart);
impl_wifi_event!(
    AccessPointWifiProtectedStatusRegistrarSuccess,
    wifi_event_ap_wps_rg_success_t
);
impl_wifi_event!(
    AccessPointWifiProtectedStatusRegistrarFailed,
    wifi_event_ap_wps_rg_fail_reason_t
);
impl_wifi_event!(AccessPointWifiProtectedStatusRegistrarTimeout);
impl_wifi_event!(
    AccessPointWifiProtectedStatusRegistrarPin,
    wifi_event_ap_wps_rg_pin_t
);
impl_wifi_event!(AccessPointWifiProtectedStatusRegistrarPushButtonConfigurationOverlap);
impl_wifi_event!(IndividualTargetWakeTimeSetup);
impl_wifi_event!(IndividualTargetWakeTimeTeardown);
impl_wifi_event!(IndividualTargetWakeTimeProbe);
impl_wifi_event!(IndividualTargetWakeTimeSuspend);
impl_wifi_event!(TargetWakeTimeWakeup);
impl_wifi_event!(BroadcastTargetWakeTimeSetup);
impl_wifi_event!(BroadcastTargetWakeTimeTeardown);
impl_wifi_event!(NeighborAwarenessNetworkingStarted);
impl_wifi_event!(NeighborAwarenessNetworkingStopped);
impl_wifi_event!(
    NeighborAwarenessNetworkingServiceMatch,
    wifi_event_nan_svc_match_t
);
impl_wifi_event!(NeighborAwarenessNetworkingReplied, wifi_event_nan_replied_t);
impl_wifi_event!(NeighborAwarenessNetworkingReceive, wifi_event_nan_receive_t);
impl_wifi_event!(
    NeighborDiscoveryProtocolIndication,
    wifi_event_ndp_indication_t
);
impl_wifi_event!(
    NeighborDiscoveryProtocolConfirmation,
    wifi_event_ndp_confirm_t
);
impl_wifi_event!(
    NeighborDiscoveryProtocolTerminated,
    wifi_event_ndp_terminated_t
);
impl_wifi_event!(HomeChannelChange, wifi_event_home_channel_change_t);
impl_wifi_event!(StationNeighborRep, wifi_event_neighbor_report_t);

impl AccessPointStationConnected<'_> {
    /// Get the MAC address of the connected station.
    pub fn mac(&self) -> &[u8] {
        &self.0.mac
    }

    /// Get the AID (Association Identifier) of the connected station.
    pub fn aid(&self) -> u8 {
        self.0.aid
    }

    /// Flag indicating whether the connected station is a mesh child.
    pub fn is_mesh_child(&self) -> bool {
        self.0.is_mesh_child
    }
}

impl AccessPointStationDisconnected<'_> {
    /// Get the MAC address of the disconnected station.
    pub fn mac(&self) -> &[u8] {
        &self.0.mac
    }

    /// Get the reason for the disconnection.
    pub fn reason(&self) -> u16 {
        self.0.reason
    }

    /// AID that the Soft-AccessPoint assigned to the disconnected station.
    pub fn aid(&self) -> u8 {
        self.0.aid
    }

    /// Flag indicating whether the connected station is a mesh child.
    pub fn is_mesh_child(&self) -> bool {
        self.0.is_mesh_child
    }
}

impl ScanDone<'_> {
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

impl StationConnected<'_> {
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

impl StationDisconnected<'_> {
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

/// All access point credentials received from WPS handshake.
#[repr(transparent)]
pub struct AccessPointCredential(wifi_event_sta_wps_er_success_t__bindgen_ty_1);

impl AccessPointCredential {
    /// Get the SSID of an access point.
    pub fn ssid(&self) -> &[u8] {
        &self.0.ssid
    }

    /// Get passphrase for the access point.
    pub fn passphrase(&self) -> &[u8] {
        &self.0.passphrase
    }
}

impl StationAuthenticationModeChange<'_> {
    /// Get the old authentication mode.
    pub fn old_mode(&self) -> u32 {
        self.0.old_mode
    }

    /// Get the new authentication mode.
    pub fn new_mode(&self) -> u32 {
        self.0.new_mode
    }
}

impl StationWifiProtectedStatusEnrolleeSuccess<'_> {
    /// Get number of access point credentials received.
    pub fn access_point_cred_cnt(&self) -> u8 {
        self.0.ap_cred_cnt
    }

    /// Get all access point credentials received.
    pub fn access_point_cred(&self) -> &[AccessPointCredential] {
        let array_ref: &[AccessPointCredential; 3] =
            // cast reference of fixed-size array to wrapper type
            unsafe { &*(&self.0.ap_cred as *const _ as *const [AccessPointCredential; 3]) };

        &array_ref[..]
    }
}

impl StationWifiProtectedStatusEnrolleePin<'_> {
    /// Get the PIN code received from the WPS.
    pub fn pin(&self) -> &[u8] {
        &self.0.pin_code
    }
}

/// A safe, read-only wrapper for a single FTM report entry.
#[repr(transparent)]
pub struct FineTimingMeasurementReportEntry<'a>(&'a wifi_ftm_report_entry_t);

impl FineTimingMeasurementReportEntry<'_> {
    /// Gets the Dialog Token of the FTM frame.
    pub fn dialog_token(&self) -> u8 {
        self.0.dlog_token
    }

    /// Gets the Received Signal Strength Indicator (RSSI) of the FTM frame.
    pub fn rssi(&self) -> i8 {
        self.0.rssi
    }

    /// Gets the Round Trip Time (RTT) in picoseconds.
    pub fn rtt(&self) -> u32 {
        self.0.rtt
    }

    /// Gets T1: Time of departure of the FTM frame from the Responder (in picoseconds).
    pub fn t1(&self) -> u64 {
        self.0.t1
    }

    /// Gets T2: Time of arrival of the FTM frame at the Initiator (in picoseconds).
    pub fn t2(&self) -> u64 {
        self.0.t2
    }

    /// Gets T3: Time of departure of the ACK from the Initiator (in picoseconds).
    pub fn t3(&self) -> u64 {
        self.0.t3
    }

    /// Gets T4: Time of arrival of the ACK at the Responder (in picoseconds).
    pub fn t4(&self) -> u64 {
        self.0.t4
    }
}

impl FineTimingMeasurementReport<'_> {
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

    /// Get the number of entries in the FTM report data.
    pub fn report_num_entries(&self) -> u8 {
        self.0.ftm_report_num_entries
    }

    /// Returns an iterator over the detailed FTM report entries.
    pub fn entries(&self) -> impl Iterator<Item = FineTimingMeasurementReportEntry<'_>> + '_ {
        let ptr = self.0.ftm_report_data;
        let len = self.0.ftm_report_num_entries as usize;

        // Return an empty slice when there are no entries.
        let entries_slice = if ptr.is_null() || len == 0 {
            &[]
        } else {
            // Otherwise, it's the slice from the data.
            // Can we trust the C API to provide a valid pointer and length?
            unsafe { core::slice::from_raw_parts(ptr, len) }
        };

        entries_slice.iter().map(FineTimingMeasurementReportEntry)
    }
}

impl AccessPointProbeRequestReceived<'_> {
    /// Get received probe request SSID.
    pub fn rssi(&self) -> i32 {
        self.0.rssi
    }

    /// Get the MAC address of the station which send probe request.
    pub fn mac(&self) -> &[u8] {
        &self.0.mac
    }
}

impl StationBasicServiceSetReceivedSignalStrengthIndicatorLow<'_> {
    /// Get received probe request SSID of bss.
    pub fn rssi(&self) -> i32 {
        self.0.rssi
    }
}

impl ActionTransmissionStatus<'_> {
    /// Get Wi-Fi interface to send request to.
    pub fn ifx(&self) -> u32 {
        self.0.ifx
    }

    /// Get context to identify the request.
    pub fn context(&self) -> u32 {
        self.0.context
    }

    /// ID of the corresponding operation that was provided during action tx request.
    pub fn op_id(&self) -> u8 {
        self.0.op_id
    }

    /// Channel provided in tx request.
    pub fn channel(&self) -> u8 {
        self.0.channel
    }

    /// Get the status of the operation.
    pub fn status(&self) -> u32 {
        self.0.status
    }
}

impl RemainOnChannelDone<'_> {
    /// Get context to identify the request.
    pub fn context(&self) -> u32 {
        self.0.context
    }
}

impl AccessPointWifiProtectedStatusRegistrarSuccess<'_> {
    /// Get enrollee mac address.
    pub fn peer_mac(&self) -> &[u8] {
        &self.0.peer_macaddr
    }
}

impl AccessPointWifiProtectedStatusRegistrarFailed<'_> {
    /// Get WPS failure reason.
    pub fn reason(&self) -> u32 {
        self.0.reason
    }

    /// Get enrollee mac address.
    pub fn peer_macaddr(&self) -> &[u8; 6] {
        &self.0.peer_macaddr
    }
}

impl AccessPointWifiProtectedStatusRegistrarPin<'_> {
    /// Get the PIN code of station in enrollee mode.
    pub fn pin_code(&self) -> &[u8] {
        &self.0.pin_code
    }
}

impl NeighborAwarenessNetworkingServiceMatch<'_> {
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

impl NeighborAwarenessNetworkingReplied<'_> {
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

impl NeighborAwarenessNetworkingReceive<'_> {
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

impl NeighborDiscoveryProtocolIndication<'_> {
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

impl NeighborDiscoveryProtocolConfirmation<'_> {
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

impl NeighborDiscoveryProtocolTerminated<'_> {
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

impl HomeChannelChange<'_> {
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

impl StationNeighborRep<'_> {
    /// Get the Neighbor Report received from the access point.
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
    event_data: *mut crate::sys::c_types::c_void,
    _event_data_size: usize,
) -> bool {
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
    event_data: *mut crate::sys::c_types::c_void,
    event_data_size: usize,
) -> bool { unsafe {
    match event {
        WifiEvent::WifiReady => {
            handle_raw::<WifiReady>(event_data, event_data_size)
        }
        WifiEvent::ScanDone => {
            handle_raw::<ScanDone<'_>>(event_data, event_data_size)
        }
        WifiEvent::StationStart => {
            handle_raw::<StationStart>(event_data, event_data_size)
        }
        WifiEvent::StationStop => {
            handle_raw::<StationStop>(event_data, event_data_size)
        }
        WifiEvent::StationConnected => {
            handle_raw::<StationConnected<'_>>(event_data, event_data_size)
        }
        WifiEvent::StationDisconnected => {
            handle_raw::<StationDisconnected<'_>>(event_data, event_data_size)
        }
        WifiEvent::StationAuthenticationModeChange => {
            handle_raw::<StationAuthenticationModeChange<'_>>(event_data, event_data_size)
        }
        WifiEvent::StationWifiProtectedStatusEnrolleeSuccess => {
            handle_raw::<StationWifiProtectedStatusEnrolleeSuccess<'_>>(event_data, event_data_size)
        }
        WifiEvent::StationWifiProtectedStatusEnrolleeFailed => {
            handle_raw::<StationWifiProtectedStatusEnrolleeFailed>(event_data, event_data_size)
        }
        WifiEvent::StationWifiProtectedStatusEnrolleeTimeout => {
            handle_raw::<StationWifiProtectedStatusEnrolleeTimeout>(event_data, event_data_size)
        }
        WifiEvent::StationWifiProtectedStatusEnrolleePin => {
            handle_raw::<StationWifiProtectedStatusEnrolleePin<'_>>(event_data, event_data_size)
        }
        WifiEvent::StationWifiProtectedStatusEnrolleePushButtonConfigurationOverlap => {
            handle_raw::<StationWifiProtectedStatusEnrolleePushButtonConfigurationOverlap>(event_data, event_data_size)
        }
        WifiEvent::AccessPointStart => {
            handle_raw::<AccessPointStart>(event_data, event_data_size)
        }
        WifiEvent::AccessPointStop => {
            handle_raw::<AccessPointStop>(event_data, event_data_size)
        }
        WifiEvent::AccessPointStationConnected => {
            handle_raw::<AccessPointStationConnected<'_>>(event_data, event_data_size)
        }
        WifiEvent::AccessPointStationDisconnected => {
            handle_raw::<AccessPointStationDisconnected<'_>>(event_data, event_data_size)
        }
        WifiEvent::AccessPointProbeRequestReceived => {
            handle_raw::<AccessPointProbeRequestReceived<'_>>(event_data, event_data_size)
        }
        WifiEvent::FineTimingMeasurementReport => {
            handle_raw::<FineTimingMeasurementReport<'_>>(event_data, event_data_size)
        }
        WifiEvent::StationBasicServiceSetReceivedSignalStrengthIndicatorLow => {
            handle_raw::<StationBasicServiceSetReceivedSignalStrengthIndicatorLow<'_>>(event_data, event_data_size)
        }
        WifiEvent::ActionTransmissionStatus => {
            handle_raw::<ActionTransmissionStatus<'_>>(event_data, event_data_size)
        }
        WifiEvent::RemainOnChannelDone => {
            handle_raw::<RemainOnChannelDone<'_>>(event_data, event_data_size)
        }
        WifiEvent::StationBeaconTimeout => {
            handle_raw::<StationBeaconTimeout>(event_data, event_data_size)
        }
        WifiEvent::ConnectionlessModuleWakeIntervalStart => {
            handle_raw::<ConnectionlessModuleWakeIntervalStart>(event_data, event_data_size)
        }
        WifiEvent::AccessPointWifiProtectedStatusRegistrarSuccess => {
            handle_raw::<AccessPointWifiProtectedStatusRegistrarSuccess<'_>>(event_data, event_data_size)
        }
        WifiEvent::AccessPointWifiProtectedStatusRegistrarFailed => {
            handle_raw::<AccessPointWifiProtectedStatusRegistrarFailed<'_>>(event_data, event_data_size)
        }
        WifiEvent::AccessPointWifiProtectedStatusRegistrarTimeout => {
            handle_raw::<AccessPointWifiProtectedStatusRegistrarTimeout>(event_data, event_data_size)
        }
        WifiEvent::AccessPointWifiProtectedStatusRegistrarPin => {
            handle_raw::<AccessPointWifiProtectedStatusRegistrarPin<'_>>(event_data, event_data_size)
        }
        WifiEvent::AccessPointWifiProtectedStatusRegistrarPushButtonConfigurationOverlap => {
            handle_raw::<AccessPointWifiProtectedStatusRegistrarPushButtonConfigurationOverlap>(event_data, event_data_size)
        }
        WifiEvent::IndividualTargetWakeTimeSetup => {
            handle_raw::<IndividualTargetWakeTimeSetup>(event_data, event_data_size)
        }
        WifiEvent::IndividualTargetWakeTimeTeardown => {
            handle_raw::<IndividualTargetWakeTimeTeardown>(event_data, event_data_size)
        }
        WifiEvent::IndividualTargetWakeTimeProbe => {
            handle_raw::<IndividualTargetWakeTimeProbe>(event_data, event_data_size)
        }
        WifiEvent::IndividualTargetWakeTimeSuspend => {
            handle_raw::<IndividualTargetWakeTimeSuspend>(event_data, event_data_size)
        }
        WifiEvent::TargetWakeTimeWakeup => {
            handle_raw::<TargetWakeTimeWakeup>(event_data, event_data_size)
        }
        WifiEvent::BroadcastTargetWakeTimeSetup => {
            handle_raw::<BroadcastTargetWakeTimeSetup>(event_data, event_data_size)
        }
        WifiEvent::BroadcastTargetWakeTimeTeardown => {
            handle_raw::<BroadcastTargetWakeTimeTeardown>(event_data, event_data_size)
        }
        WifiEvent::NeighborAwarenessNetworkingStarted => {
            handle_raw::<NeighborAwarenessNetworkingStarted>(event_data, event_data_size)
        }
        WifiEvent::NeighborAwarenessNetworkingStopped => {
            handle_raw::<NeighborAwarenessNetworkingStopped>(event_data, event_data_size)
        }
        WifiEvent::NeighborAwarenessNetworkingServiceMatch => {
            handle_raw::<NeighborAwarenessNetworkingServiceMatch<'_>>(event_data, event_data_size)
        }
        WifiEvent::NeighborAwarenessNetworkingReplied => {
            handle_raw::<NeighborAwarenessNetworkingReplied<'_>>(event_data, event_data_size)
        }
        WifiEvent::NeighborAwarenessNetworkingReceive => {
            handle_raw::<NeighborAwarenessNetworkingReceive<'_>>(event_data, event_data_size)
        }
        WifiEvent::NeighborDiscoveryProtocolIndication => {
            handle_raw::<NeighborDiscoveryProtocolIndication<'_>>(event_data, event_data_size)
        }
        WifiEvent::NeighborDiscoveryProtocolConfirmation => {
            handle_raw::<NeighborDiscoveryProtocolConfirmation<'_>>(event_data, event_data_size)
        }
        WifiEvent::NeighborDiscoveryProtocolTerminated => {
            handle_raw::<NeighborDiscoveryProtocolTerminated<'_>>(event_data, event_data_size)
        }
        WifiEvent::HomeChannelChange => {
            handle_raw::<HomeChannelChange<'_>>(event_data, event_data_size)
        }
        WifiEvent::StationNeighborRep => {
            handle_raw::<StationNeighborRep<'_>>(event_data, event_data_size)
        }
    }
}}
