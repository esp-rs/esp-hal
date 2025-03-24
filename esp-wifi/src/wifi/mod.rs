//! WiFi

pub mod config;
pub mod event;
pub(crate) mod internal;
pub(crate) mod os_adapter;
pub(crate) mod state;
use alloc::collections::vec_deque::VecDeque;
use core::{fmt::Debug, marker::PhantomData, mem::MaybeUninit, ptr::addr_of, task::Poll};

use enumset::EnumSet;
use esp_hal::{asynch::AtomicWaker, sync::Locked};
use esp_wifi_sys::include::{
    esp_eap_client_clear_ca_cert,
    esp_eap_client_clear_certificate_and_key,
    esp_eap_client_clear_identity,
    esp_eap_client_clear_new_password,
    esp_eap_client_clear_password,
    esp_eap_client_clear_username,
    esp_eap_client_set_ca_cert,
    esp_eap_client_set_certificate_and_key,
    esp_eap_client_set_disable_time_check,
    esp_eap_client_set_fast_params,
    esp_eap_client_set_identity,
    esp_eap_client_set_new_password,
    esp_eap_client_set_pac_file,
    esp_eap_client_set_password,
    esp_eap_client_set_ttls_phase2_method,
    esp_eap_client_set_username,
    esp_eap_fast_config,
    esp_wifi_sta_enterprise_enable,
    wifi_pkt_rx_ctrl_t,
    wifi_scan_channel_bitmap_t,
    WIFI_PROTOCOL_11AX,
    WIFI_PROTOCOL_11B,
    WIFI_PROTOCOL_11G,
    WIFI_PROTOCOL_11N,
    WIFI_PROTOCOL_LR,
};
#[cfg(feature = "sniffer")]
use esp_wifi_sys::include::{
    esp_wifi_80211_tx,
    esp_wifi_set_promiscuous,
    esp_wifi_set_promiscuous_rx_cb,
    wifi_promiscuous_pkt_t,
    wifi_promiscuous_pkt_type_t,
};
use event::*;
pub(crate) use internal::*;
use num_derive::FromPrimitive;
#[doc(hidden)]
pub(crate) use os_adapter::*;
use portable_atomic::{AtomicUsize, Ordering};
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};
#[cfg(feature = "smoltcp")]
use smoltcp::phy::{Device, DeviceCapabilities, RxToken, TxToken};
pub use state::*;

#[cfg(feature = "csi")]
pub use crate::binary::include::wifi_csi_info_t;
use crate::{
    common_adapter::*,
    config::PowerSaveMode,
    esp_wifi_result,
    hal::ram,
    wifi::{config::AuthMethodExt, private::EspWifiPacketBuffer},
    EspWifiController,
};

const MTU: usize = crate::CONFIG.mtu;

use crate::binary::{
    c_types,
    include::{
        self,
        __BindgenBitfieldUnit,
        esp_err_t,
        esp_interface_t_ESP_IF_WIFI_AP,
        esp_interface_t_ESP_IF_WIFI_STA,
        esp_supplicant_deinit,
        esp_supplicant_init,
        esp_wifi_connect,
        esp_wifi_deinit_internal,
        esp_wifi_disconnect,
        esp_wifi_init_internal,
        esp_wifi_internal_free_rx_buffer,
        esp_wifi_internal_reg_rxcb,
        esp_wifi_internal_tx,
        esp_wifi_scan_start,
        esp_wifi_set_config,
        esp_wifi_set_country,
        esp_wifi_set_mode,
        esp_wifi_set_protocol,
        esp_wifi_set_tx_done_cb,
        esp_wifi_start,
        esp_wifi_stop,
        g_wifi_default_wpa_crypto_funcs,
        wifi_active_scan_time_t,
        wifi_ap_config_t,
        wifi_cipher_type_t_WIFI_CIPHER_TYPE_CCMP,
        wifi_config_t,
        wifi_country_policy_t_WIFI_COUNTRY_POLICY_MANUAL,
        wifi_country_t,
        wifi_interface_t,
        wifi_interface_t_WIFI_IF_AP,
        wifi_interface_t_WIFI_IF_STA,
        wifi_mode_t_WIFI_MODE_AP,
        wifi_mode_t_WIFI_MODE_APSTA,
        wifi_mode_t_WIFI_MODE_NULL,
        wifi_mode_t_WIFI_MODE_STA,
        wifi_pmf_config_t,
        wifi_scan_config_t,
        wifi_scan_threshold_t,
        wifi_scan_time_t,
        wifi_scan_type_t_WIFI_SCAN_TYPE_ACTIVE,
        wifi_scan_type_t_WIFI_SCAN_TYPE_PASSIVE,
        wifi_sort_method_t_WIFI_CONNECT_AP_BY_SIGNAL,
        wifi_sta_config_t,
    },
};

#[cfg(feature = "csi")]
pub(crate) trait CsiCallback: FnMut(crate::binary::include::wifi_csi_info_t) {}

#[cfg(feature = "csi")]
impl<T> CsiCallback for T where T: FnMut(crate::binary::include::wifi_csi_info_t) {}

#[cfg(feature = "csi")]
unsafe extern "C" fn csi_rx_cb<C: CsiCallback>(
    ctx: *mut crate::wifi::c_types::c_void,
    data: *mut crate::binary::include::wifi_csi_info_t,
) {
    let csi_callback = unsafe { &mut *(ctx as *mut C) };
    csi_callback(*data);
}

const RX_QUEUE_SIZE: usize = crate::CONFIG.rx_queue_size;
const TX_QUEUE_SIZE: usize = crate::CONFIG.tx_queue_size;

pub(crate) static DATA_QUEUE_RX_AP: Locked<VecDeque<EspWifiPacketBuffer>> =
    Locked::new(VecDeque::new());

pub(crate) static DATA_QUEUE_RX_STA: Locked<VecDeque<EspWifiPacketBuffer>> =
    Locked::new(VecDeque::new());

/// Common errors.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum WifiError {
    /// Wi-Fi module is not initialized or not initialized for `Wi-Fi`
    /// operations.
    NotInitialized,

    /// Internal Wi-Fi error.
    InternalError(InternalWifiError),

    /// The device disconnected from the network or failed to connect to it.
    Disconnected,

    /// Unknown Wi-Fi mode (not Sta/Ap/ApSta).
    UnknownWifiMode,

    /// Unsupported operation or mode.
    Unsupported,
}

/// Error originating from the underlying drivers
#[repr(i32)]
#[derive(Copy, Clone, Debug, PartialEq, Eq, FromPrimitive)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(clippy::enum_variant_names)] // FIXME remove prefix
pub enum InternalWifiError {
    /// Out of memory
    EspErrNoMem          = 0x101,

    /// Invalid argument
    EspErrInvalidArg     = 0x102,

    /// WiFi driver was not installed by esp_wifi_init
    EspErrWifiNotInit    = 0x3001,

    /// WiFi driver was not started by esp_wifi_start
    EspErrWifiNotStarted = 0x3002,

    /// WiFi driver was not stopped by esp_wifi_stop
    EspErrWifiNotStopped = 0x3003,

    /// WiFi interface error
    EspErrWifiIf         = 0x3004,

    /// WiFi mode error
    EspErrWifiMode       = 0x3005,

    /// WiFi internal state error
    EspErrWifiState      = 0x3006,

    /// WiFi internal control block of station or soft-AP error
    EspErrWifiConn       = 0x3007,

    /// WiFi internal NVS module error
    EspErrWifiNvs        = 0x3008,

    /// MAC address is invalid
    EspErrWifiMac        = 0x3009,

    /// SSID is invalid
    EspErrWifiSsid       = 0x300A,

    /// Password is invalid
    EspErrWifiPassword   = 0x300B,

    /// Timeout error
    EspErrWifiTimeout    = 0x300C,

    /// WiFi is in sleep state(RF closed) and wakeup fail
    EspErrWifiWakeFail   = 0x300D,

    /// The caller would block
    EspErrWifiWouldBlock = 0x300E,

    /// Station still in disconnect status
    EspErrWifiNotConnect = 0x300F,

    /// Failed to post the event to WiFi task
    EspErrWifiPost       = 0x3012,

    /// Invalid WiFi state when init/deinit is called
    EspErrWifiInitState  = 0x3013,

    /// Returned when WiFi is stopping
    EspErrWifiStopState  = 0x3014,

    /// The WiFi connection is not associated
    EspErrWifiNotAssoc   = 0x3015,

    /// The WiFi TX is disallowed
    EspErrWifiTxDisallow = 0x3016,
}

/// Get the STA MAC address
pub fn sta_mac(mac: &mut [u8; 6]) {
    unsafe {
        read_mac(mac as *mut u8, 0);
    }
}

/// Get the AP MAC address
pub fn ap_mac(mac: &mut [u8; 6]) {
    unsafe {
        read_mac(mac as *mut u8, 1);
    }
}

pub(crate) fn wifi_init() -> Result<(), WifiError> {
    unsafe {
        G_CONFIG.wpa_crypto_funcs = g_wifi_default_wpa_crypto_funcs;
        G_CONFIG.feature_caps = g_wifi_feature_caps;

        #[cfg(coex)]
        esp_wifi_result!(coex_init())?;

        esp_wifi_result!(esp_wifi_init_internal(addr_of!(G_CONFIG)))?;
        esp_wifi_result!(esp_wifi_set_mode(wifi_mode_t_WIFI_MODE_NULL))?;

        esp_wifi_result!(esp_supplicant_init())?;

        esp_wifi_result!(esp_wifi_set_tx_done_cb(Some(esp_wifi_tx_done_cb)))?;

        esp_wifi_result!(esp_wifi_internal_reg_rxcb(
            esp_interface_t_ESP_IF_WIFI_STA,
            Some(recv_cb_sta)
        ))?;

        // until we support APSTA we just register the same callback for AP and STA
        esp_wifi_result!(esp_wifi_internal_reg_rxcb(
            esp_interface_t_ESP_IF_WIFI_AP,
            Some(recv_cb_ap)
        ))?;

        #[cfg(any(esp32, esp32s3))]
        {
            static mut NVS_STRUCT: [u32; 12] = [0; 12];
            chip_specific::g_misc_nvs = addr_of!(NVS_STRUCT) as u32;
        }

        crate::flags::WIFI.fetch_add(1, Ordering::SeqCst);

        Ok(())
    }
}

pub(crate) fn wifi_deinit() -> Result<(), crate::InitializationError> {
    esp_wifi_result!(unsafe { esp_wifi_stop() })?;
    esp_wifi_result!(unsafe { esp_wifi_deinit_internal() })?;
    esp_wifi_result!(unsafe { esp_supplicant_deinit() })?;
    Ok(())
}

unsafe extern "C" fn recv_cb_sta(
    buffer: *mut c_types::c_void,
    len: u16,
    eb: *mut c_types::c_void,
) -> esp_err_t {
    let packet = EspWifiPacketBuffer { buffer, len, eb };
    // We must handle the result outside of the lock because
    // EspWifiPacketBuffer::drop must not be called in a critical section.
    // Dropping an EspWifiPacketBuffer will call `esp_wifi_internal_free_rx_buffer`
    // which will try to lock an internal mutex. If the mutex is already taken,
    // the function will try to trigger a context switch, which will fail if we
    // are in an interrupt-free context.
    if let Ok(()) = DATA_QUEUE_RX_STA.with(|queue| {
        if queue.len() < RX_QUEUE_SIZE {
            queue.push_back(packet);
            Ok(())
        } else {
            Err(packet)
        }
    }) {
        embassy::STA_RECEIVE_WAKER.wake();
        include::ESP_OK as esp_err_t
    } else {
        debug!("RX QUEUE FULL");
        include::ESP_ERR_NO_MEM as esp_err_t
    }
}

unsafe extern "C" fn recv_cb_ap(
    buffer: *mut c_types::c_void,
    len: u16,
    eb: *mut c_types::c_void,
) -> esp_err_t {
    let packet = EspWifiPacketBuffer { buffer, len, eb };
    // We must handle the result outside of the critical section because
    // EspWifiPacketBuffer::drop must not be called in a critical section.
    // Dropping an EspWifiPacketBuffer will call `esp_wifi_internal_free_rx_buffer`
    // which will try to lock an internal mutex. If the mutex is already taken,
    // the function will try to trigger a context switch, which will fail if we
    // are in an interrupt-free context.
    if let Ok(()) = DATA_QUEUE_RX_AP.with(|queue| {
        if queue.len() < RX_QUEUE_SIZE {
            queue.push_back(packet);
            Ok(())
        } else {
            Err(packet)
        }
    }) {
        embassy::AP_RECEIVE_WAKER.wake();
        include::ESP_OK as esp_err_t
    } else {
        debug!("RX QUEUE FULL");
        include::ESP_ERR_NO_MEM as esp_err_t
    }
}

pub(crate) static WIFI_TX_INFLIGHT: AtomicUsize = AtomicUsize::new(0);

fn decrement_inflight_counter() {
    unwrap!(
        WIFI_TX_INFLIGHT.fetch_update(Ordering::SeqCst, Ordering::SeqCst, |x| {
            Some(x.saturating_sub(1))
        })
    );
}

#[ram]
unsafe extern "C" fn esp_wifi_tx_done_cb(
    _ifidx: u8,
    _data: *mut u8,
    _data_len: *mut u16,
    _tx_status: bool,
) {
    trace!("esp_wifi_tx_done_cb");

    decrement_inflight_counter();

    embassy::TRANSMIT_WAKER.wake();
}

pub(crate) fn wifi_start() -> Result<(), WifiError> {
    unsafe {
        esp_wifi_result!(esp_wifi_start())?;

        let mode = config::WifiMode::current()?;

        // This is not an if-else because in AP-STA mode, both are true
        if mode.is_ap() {
            esp_wifi_result!(include::esp_wifi_set_inactive_time(
                wifi_interface_t_WIFI_IF_AP,
                crate::CONFIG.ap_beacon_timeout
            ))?;
        }
        if mode.is_sta() {
            esp_wifi_result!(include::esp_wifi_set_inactive_time(
                wifi_interface_t_WIFI_IF_STA,
                crate::CONFIG.beacon_timeout
            ))?;
        };

        let mut cntry_code = [0u8; 3];
        cntry_code[..crate::CONFIG.country_code.len()]
            .copy_from_slice(crate::CONFIG.country_code.as_bytes());
        cntry_code[2] = crate::CONFIG.country_code_operating_class;

        #[allow(clippy::useless_transmute)]
        let country = wifi_country_t {
            // FIXME once we bumped the MSRV accordingly (see https://github.com/esp-rs/esp-hal/pull/3027#discussion_r1944718266)
            #[allow(clippy::useless_transmute)]
            cc: core::mem::transmute::<[u8; 3], [core::ffi::c_char; 3]>(cntry_code),
            schan: 1,
            nchan: 13,
            max_tx_power: 20,
            policy: wifi_country_policy_t_WIFI_COUNTRY_POLICY_MANUAL,
        };
        esp_wifi_result!(esp_wifi_set_country(&country))?;
    }

    Ok(())
}

pub(crate) fn wifi_start_scan(
    block: bool,
    config::ScanConfig {
        ssid,
        mut bssid,
        channel,
        show_hidden,
        scan_type,
    }: config::ScanConfig<'_>,
) -> i32 {
    scan_type.validate();
    let (scan_time, scan_type) = match scan_type {
        config::ScanTypeConfig::Active { min, max } => (
            wifi_scan_time_t {
                active: wifi_active_scan_time_t {
                    min: min.as_millis() as u32,
                    max: max.as_millis() as u32,
                },
                passive: 0,
            },
            wifi_scan_type_t_WIFI_SCAN_TYPE_ACTIVE,
        ),
        config::ScanTypeConfig::Passive(dur) => (
            wifi_scan_time_t {
                active: wifi_active_scan_time_t { min: 0, max: 0 },
                passive: dur.as_millis() as u32,
            },
            wifi_scan_type_t_WIFI_SCAN_TYPE_PASSIVE,
        ),
    };

    let mut ssid_buf = ssid.map(|m| {
        let mut buf = heapless::Vec::<u8, 33>::from_iter(m.bytes());
        unwrap!(buf.push(b'\0').ok());
        buf
    });

    let ssid = ssid_buf
        .as_mut()
        .map(|e| e.as_mut_ptr())
        .unwrap_or_else(core::ptr::null_mut);
    let bssid = bssid
        .as_mut()
        .map(|e| e.as_mut_ptr())
        .unwrap_or_else(core::ptr::null_mut);

    let scan_config = wifi_scan_config_t {
        ssid,
        bssid,
        channel: channel.unwrap_or(0),
        show_hidden,
        scan_type,
        scan_time,
        home_chan_dwell_time: 0,
        channel_bitmap: wifi_scan_channel_bitmap_t {
            ghz_2_channels: 0,
            ghz_5_channels: 0,
        },
    };

    unsafe { esp_wifi_scan_start(&scan_config, block) }
}

mod private {
    use super::*;

    #[derive(Debug)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    /// Take care not to drop this while in a critical section.
    ///
    /// Dropping an EspWifiPacketBuffer will call
    /// `esp_wifi_internal_free_rx_buffer` which will try to lock an
    /// internal mutex. If the mutex is already taken, the function will try
    /// to trigger a context switch, which will fail if we are in a critical
    /// section.
    pub struct EspWifiPacketBuffer {
        pub(crate) buffer: *mut c_types::c_void,
        pub(crate) len: u16,
        pub(crate) eb: *mut c_types::c_void,
    }

    unsafe impl Send for EspWifiPacketBuffer {}

    impl Drop for EspWifiPacketBuffer {
        fn drop(&mut self) {
            trace!("Dropping EspWifiPacketBuffer, freeing memory");
            unsafe { esp_wifi_internal_free_rx_buffer(self.eb) };
        }
    }

    impl EspWifiPacketBuffer {
        pub fn as_slice_mut(&mut self) -> &mut [u8] {
            unsafe { core::slice::from_raw_parts_mut(self.buffer as *mut u8, self.len as usize) }
        }
    }
}

/// Provides methods for retrieving the Wi-Fi mode and MAC address.
#[derive(Debug, Clone, Copy)]
pub enum WifiDeviceMode {
    Sta,
    Ap,
}

impl WifiDeviceMode {
    fn mac_address(&self) -> [u8; 6] {
        match self {
            WifiDeviceMode::Sta => {
                let mut mac = [0; 6];
                sta_mac(&mut mac);
                mac
            }
            WifiDeviceMode::Ap => {
                let mut mac = [0; 6];
                ap_mac(&mut mac);
                mac
            }
        }
    }

    fn data_queue_rx(&self) -> &'static Locked<VecDeque<EspWifiPacketBuffer>> {
        match self {
            WifiDeviceMode::Sta => &DATA_QUEUE_RX_STA,
            WifiDeviceMode::Ap => &DATA_QUEUE_RX_AP,
        }
    }

    fn can_send(&self) -> bool {
        WIFI_TX_INFLIGHT.load(Ordering::SeqCst) < TX_QUEUE_SIZE
    }

    fn increase_in_flight_counter(&self) {
        WIFI_TX_INFLIGHT.fetch_add(1, Ordering::SeqCst);
    }

    fn tx_token(&self) -> Option<WifiTxToken> {
        if !self.can_send() {
            crate::preempt::yield_task();
        }

        if self.can_send() {
            Some(WifiTxToken { mode: *self })
        } else {
            None
        }
    }

    fn rx_token(&self) -> Option<(WifiRxToken, WifiTxToken)> {
        let is_empty = self.data_queue_rx().with(|q| q.is_empty());
        if is_empty || !self.can_send() {
            crate::preempt::yield_task();
        }

        let is_empty = is_empty && self.data_queue_rx().with(|q| q.is_empty());

        if !is_empty {
            self.tx_token().map(|tx| (WifiRxToken { mode: *self }, tx))
        } else {
            None
        }
    }

    fn interface(&self) -> wifi_interface_t {
        match self {
            WifiDeviceMode::Sta => wifi_interface_t_WIFI_IF_STA,
            WifiDeviceMode::Ap => wifi_interface_t_WIFI_IF_AP,
        }
    }

    fn register_transmit_waker(&self, cx: &mut core::task::Context<'_>) {
        embassy::TRANSMIT_WAKER.register(cx.waker())
    }

    fn register_receive_waker(&self, cx: &mut core::task::Context<'_>) {
        match self {
            WifiDeviceMode::Sta => embassy::STA_RECEIVE_WAKER.register(cx.waker()),
            WifiDeviceMode::Ap => embassy::AP_RECEIVE_WAKER.register(cx.waker()),
        }
    }

    fn register_link_state_waker(&self, cx: &mut core::task::Context<'_>) {
        match self {
            WifiDeviceMode::Sta => embassy::STA_LINK_STATE_WAKER.register(cx.waker()),
            WifiDeviceMode::Ap => embassy::AP_LINK_STATE_WAKER.register(cx.waker()),
        }
    }

    fn link_state(&self) -> embassy_net_driver::LinkState {
        match self {
            WifiDeviceMode::Sta => {
                if matches!(sta_state(), WifiState::StaConnected) {
                    embassy_net_driver::LinkState::Up
                } else {
                    embassy_net_driver::LinkState::Down
                }
            }
            WifiDeviceMode::Ap => {
                if matches!(ap_state(), WifiState::ApStarted) {
                    embassy_net_driver::LinkState::Up
                } else {
                    embassy_net_driver::LinkState::Down
                }
            }
        }
    }
}

/// A wifi device implementing smoltcp's Device trait.
pub struct WifiDevice<'d> {
    _phantom: PhantomData<&'d ()>,
    mode: WifiDeviceMode,
}

impl WifiDevice<'_> {
    /// Retrieves the MAC address of the Wi-Fi device.
    pub fn mac_address(&self) -> [u8; 6] {
        self.mode.mac_address()
    }

    /// Receives data from the Wi-Fi device (only when `smoltcp` feature is
    /// disabled).
    #[cfg(not(feature = "smoltcp"))]
    pub fn receive(&mut self) -> Option<(WifiRxToken, WifiTxToken)> {
        self.mode.rx_token()
    }

    /// Transmits data through the Wi-Fi device (only when `smoltcp` feature is
    /// disabled).
    #[cfg(not(feature = "smoltcp"))]
    pub fn transmit(&mut self) -> Option<WifiTxToken> {
        self.mode.tx_token()
    }
}

fn convert_ap_info(record: &include::wifi_ap_record_t) -> config::AccessPointInfo {
    let str_len = record
        .ssid
        .iter()
        .position(|&c| c == 0)
        .unwrap_or(record.ssid.len());
    let ssid_ref = unsafe { core::str::from_utf8_unchecked(&record.ssid[..str_len]) };

    let mut ssid = heapless::String::<32>::new();
    unwrap!(ssid.push_str(ssid_ref));

    config::AccessPointInfo {
        ssid,
        bssid: record.bssid,
        channel: record.primary,
        secondary_channel: match record.second {
            include::wifi_second_chan_t_WIFI_SECOND_CHAN_NONE => config::SecondaryChannel::None,
            include::wifi_second_chan_t_WIFI_SECOND_CHAN_ABOVE => config::SecondaryChannel::Above,
            include::wifi_second_chan_t_WIFI_SECOND_CHAN_BELOW => config::SecondaryChannel::Below,
            _ => panic!(),
        },
        signal_strength: record.rssi,
        protocols: EnumSet::empty(), // TODO
        auth_method: Some(config::AuthMethod::from_raw(record.authmode)),
    }
}

/// The radio metadata header of the received packet, which is the common header
/// at the beginning of all RX callback buffers in promiscuous mode.
#[cfg(not(any(esp32c6)))]
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RxControlInfo {
    /// Received Signal Strength Indicator (RSSI) of the packet, in dBm.
    pub rssi: i32,
    /// PHY rate encoding of the packet. Only valid for non-HT (802.11b/g)
    /// packets.
    pub rate: u32,
    /// Protocol of the received packet: 0 for non-HT (11bg), 1 for HT (11n), 3
    /// for VHT (11ac).
    pub sig_mode: u32,
    /// Modulation and Coding Scheme (MCS). Indicates modulation for HT (11n)
    /// packets.
    pub mcs: u32,
    /// Channel bandwidth of the packet: 0 for 20MHz, 1 for 40MHz.
    pub cwb: u32,
    /// Channel estimate smoothing: 1 recommends smoothing; 0 recommends
    /// per-carrier-independent estimate.
    pub smoothing: u32,
    /// Sounding indicator: 0 for sounding PPDU (used for channel estimation); 1
    /// for non-sounding PPDU.
    pub not_sounding: u32,
    /// Aggregation status: 0 for MPDU packet, 1 for AMPDU packet.
    pub aggregation: u32,
    /// Space-Time Block Coding (STBC) status: 0 for non-STBC packet, 1 for STBC
    /// packet.
    pub stbc: u32,
    /// Forward Error Correction (FEC) status: indicates if LDPC coding is used
    /// for 11n packets.
    pub fec_coding: u32,
    /// Short Guard Interval (SGI): 0 for long guard interval, 1 for short guard
    /// interval.
    pub sgi: u32,
    /// Number of subframes aggregated in an AMPDU packet.
    pub ampdu_cnt: u32,
    /// Primary channel on which the packet is received.
    pub channel: u32,
    /// Secondary channel on which the packet is received: 0 for none, 1 for
    /// above, 2 for below.
    pub secondary_channel: u32,
    /// Timestamp of when the packet is received, in microseconds. Precise only
    /// if modem sleep or light sleep is not enabled.
    pub timestamp: u32,
    /// Noise floor of the Radio Frequency module, in dBm.
    pub noise_floor: i32,
    /// Antenna number from which the packet is received: 0 for antenna 0, 1 for
    /// antenna 1.
    pub ant: u32,
    /// Length of the packet including the Frame Check Sequence (FCS).
    pub sig_len: u32,
    /// State of the packet: 0 for no error, other values indicate error codes.
    pub rx_state: u32,
}

/// The radio metadata header of the received packet, which is the common header
/// at the beginning of all RX callback buffers in promiscuous mode.
#[cfg(esp32c6)]
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RxControlInfo {
    /// Received Signal Strength Indicator (RSSI) of the packet, in dBm.
    pub rssi: i32,
    /// PHY rate encoding of the packet. Only valid for non-HT (802.11b/g)
    /// packets.
    pub rate: u32,
    /// Length of the received packet including the Frame Check Sequence (FCS).
    pub sig_len: u32,
    /// Reception state of the packet: 0 for no error, others indicate error
    /// codes.
    pub rx_state: u32,
    /// Length of the dump buffer.
    pub dump_len: u32,
    /// Length of HE-SIG-B field (802.11ax).
    pub he_sigb_len: u32,
    /// Indicates if this is a single MPDU.
    pub cur_single_mpdu: u32,
    /// Current baseband format.
    pub cur_bb_format: u32,
    /// Channel estimation validity.
    pub rx_channel_estimate_info_vld: u32,
    /// Length of the channel estimation.
    pub rx_channel_estimate_len: u32,
    /// Timing information in seconds.
    pub second: u32,
    /// Primary channel on which the packet is received.
    pub channel: u32,
    /// Noise floor of the Radio Frequency module, in dBm.
    pub noise_floor: i32,
    /// Indicates if this is a group-addressed frame.
    pub is_group: u32,
    /// End state of the packet reception.
    pub rxend_state: u32,
    /// Indicate whether the reception frame is from interface 3.
    pub rxmatch3: u32,
    /// Indicate whether the reception frame is from interface 2.
    pub rxmatch2: u32,
    /// Indicate whether the reception frame is from interface 1.
    pub rxmatch1: u32,
    /// Indicate whether the reception frame is from interface 0.
    pub rxmatch0: u32,
}
impl RxControlInfo {
    /// Create an instance from a raw pointer to [wifi_pkt_rx_ctrl_t].
    ///
    /// # Safety
    /// When calling this, you must ensure, that `rx_cntl` points to a valid
    /// instance of [wifi_pkt_rx_ctrl_t].
    pub unsafe fn from_raw(rx_cntl: *const wifi_pkt_rx_ctrl_t) -> Self {
        #[cfg(not(esp32c6))]
        let rx_control_info = RxControlInfo {
            rssi: (*rx_cntl).rssi(),
            rate: (*rx_cntl).rate(),
            sig_mode: (*rx_cntl).sig_mode(),
            mcs: (*rx_cntl).mcs(),
            cwb: (*rx_cntl).cwb(),
            smoothing: (*rx_cntl).smoothing(),
            not_sounding: (*rx_cntl).not_sounding(),
            aggregation: (*rx_cntl).aggregation(),
            stbc: (*rx_cntl).stbc(),
            fec_coding: (*rx_cntl).fec_coding(),
            sgi: (*rx_cntl).sgi(),
            ampdu_cnt: (*rx_cntl).ampdu_cnt(),
            channel: (*rx_cntl).channel(),
            secondary_channel: (*rx_cntl).secondary_channel(),
            timestamp: (*rx_cntl).timestamp(),
            noise_floor: (*rx_cntl).noise_floor(),
            ant: (*rx_cntl).ant(),
            sig_len: (*rx_cntl).sig_len(),
            rx_state: (*rx_cntl).rx_state(),
        };
        #[cfg(esp32c6)]
        let rx_control_info = RxControlInfo {
            rssi: (*rx_cntl).rssi(),
            rate: (*rx_cntl).rate(),
            sig_len: (*rx_cntl).sig_len(),
            rx_state: (*rx_cntl).rx_state(),
            dump_len: (*rx_cntl).dump_len(),
            he_sigb_len: (*rx_cntl).he_sigb_len(),
            cur_single_mpdu: (*rx_cntl).cur_single_mpdu(),
            cur_bb_format: (*rx_cntl).cur_bb_format(),
            rx_channel_estimate_info_vld: (*rx_cntl).rx_channel_estimate_info_vld(),
            rx_channel_estimate_len: (*rx_cntl).rx_channel_estimate_len(),
            second: (*rx_cntl).second(),
            channel: (*rx_cntl).channel(),
            noise_floor: (*rx_cntl).noise_floor(),
            is_group: (*rx_cntl).is_group(),
            rxend_state: (*rx_cntl).rxend_state(),
            rxmatch3: (*rx_cntl).rxmatch3(),
            rxmatch2: (*rx_cntl).rxmatch2(),
            rxmatch1: (*rx_cntl).rxmatch1(),
            rxmatch0: (*rx_cntl).rxmatch0(),
        };
        rx_control_info
    }
}
/// Represents a Wi-Fi packet in promiscuous mode.
#[cfg(feature = "sniffer")]
pub struct PromiscuousPkt<'a> {
    /// Control information related to packet reception.
    pub rx_cntl: RxControlInfo,
    /// Frame type of the received packet.
    pub frame_type: wifi_promiscuous_pkt_type_t,
    /// Length of the received packet.
    pub len: usize,
    /// Data contained in the received packet.
    pub data: &'a [u8],
}
#[cfg(feature = "sniffer")]
impl PromiscuousPkt<'_> {
    /// # Safety
    /// When calling this, you have to ensure, that `buf` points to a valid
    /// [wifi_promiscuous_pkt_t].
    pub(crate) unsafe fn from_raw(
        buf: *const wifi_promiscuous_pkt_t,
        frame_type: wifi_promiscuous_pkt_type_t,
    ) -> Self {
        let rx_cntl = RxControlInfo::from_raw(&(*buf).rx_ctrl);
        let len = rx_cntl.sig_len as usize;
        PromiscuousPkt {
            rx_cntl,
            frame_type,
            len,
            data: core::slice::from_raw_parts(
                (buf as *const u8).add(core::mem::size_of::<wifi_pkt_rx_ctrl_t>()),
                len,
            ),
        }
    }
}

#[cfg(feature = "sniffer")]
static SNIFFER_CB: Locked<Option<fn(PromiscuousPkt<'_>)>> = Locked::new(None);

#[cfg(feature = "sniffer")]
unsafe extern "C" fn promiscuous_rx_cb(buf: *mut core::ffi::c_void, frame_type: u32) {
    if let Some(sniffer_callback) = SNIFFER_CB.with(|callback| *callback) {
        let promiscuous_pkt = PromiscuousPkt::from_raw(buf as *const _, frame_type);
        sniffer_callback(promiscuous_pkt);
    }
}

#[cfg(feature = "sniffer")]
/// A wifi sniffer.
#[non_exhaustive]
pub struct Sniffer {}

#[cfg(feature = "sniffer")]
impl Sniffer {
    pub(crate) fn new() -> Self {
        // This shouldn't fail, since the way this is created, means that wifi will
        // always be initialized.
        unwrap!(esp_wifi_result!(unsafe {
            esp_wifi_set_promiscuous_rx_cb(Some(promiscuous_rx_cb))
        }));
        Self {}
    }
    /// Set promiscuous mode enabled or disabled.
    pub fn set_promiscuous_mode(&self, enabled: bool) -> Result<(), WifiError> {
        esp_wifi_result!(unsafe { esp_wifi_set_promiscuous(enabled) })?;
        Ok(())
    }
    /// Transmit a raw frame.
    pub fn send_raw_frame(
        &mut self,
        use_sta_interface: bool,
        buffer: &[u8],
        use_internal_seq_num: bool,
    ) -> Result<(), WifiError> {
        esp_wifi_result!(unsafe {
            esp_wifi_80211_tx(
                if use_sta_interface {
                    wifi_interface_t_WIFI_IF_STA
                } else {
                    wifi_interface_t_WIFI_IF_AP
                } as wifi_interface_t,
                buffer.as_ptr() as *const _,
                buffer.len() as i32,
                use_internal_seq_num,
            )
        })
    }
    /// Set the callback for receiving a packet.
    pub fn set_receive_cb(&mut self, cb: fn(PromiscuousPkt<'_>)) {
        SNIFFER_CB.with(|callback| *callback = Some(cb));
    }
}

// see https://docs.rs/smoltcp/0.7.1/smoltcp/phy/index.html
#[cfg(feature = "smoltcp")]
impl Device for WifiDevice<'_> {
    type RxToken<'a>
        = WifiRxToken
    where
        Self: 'a;
    type TxToken<'a>
        = WifiTxToken
    where
        Self: 'a;

    fn receive(
        &mut self,
        _instant: smoltcp::time::Instant,
    ) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        self.mode.rx_token()
    }

    fn transmit(&mut self, _instant: smoltcp::time::Instant) -> Option<Self::TxToken<'_>> {
        self.mode.tx_token()
    }

    fn capabilities(&self) -> smoltcp::phy::DeviceCapabilities {
        let mut caps = DeviceCapabilities::default();
        caps.max_transmission_unit = MTU;
        caps.max_burst_size = if crate::CONFIG.max_burst_size == 0 {
            None
        } else {
            Some(crate::CONFIG.max_burst_size)
        };
        caps
    }
}

#[doc(hidden)]
#[derive(Debug)]
pub struct WifiRxToken {
    mode: WifiDeviceMode,
}

impl WifiRxToken {
    /// Consumes the RX token and applies the callback function to the received
    /// data buffer.
    pub fn consume_token<R, F>(self, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        let mut data = self.mode.data_queue_rx().with(|queue| {
            unwrap!(
                queue.pop_front(),
                "unreachable: transmit()/receive() ensures there is a packet to process"
            )
        });

        // We handle the received data outside of the lock because
        // EspWifiPacketBuffer::drop must not be called in a critical section.
        // Dropping an EspWifiPacketBuffer will call `esp_wifi_internal_free_rx_buffer`
        // which will try to lock an internal mutex. If the mutex is already
        // taken, the function will try to trigger a context switch, which will
        // fail if we are in an interrupt-free context.
        let buffer = data.as_slice_mut();
        dump_packet_info(buffer);

        f(buffer)
    }
}

#[cfg(feature = "smoltcp")]
impl RxToken for WifiRxToken {
    fn consume<R, F>(self, f: F) -> R
    where
        F: FnOnce(&[u8]) -> R,
    {
        self.consume_token(|t| f(t))
    }
}

#[doc(hidden)]
#[derive(Debug)]
pub struct WifiTxToken {
    mode: WifiDeviceMode,
}

impl WifiTxToken {
    /// Consumes the TX token and applies the callback function to the received
    /// data buffer.
    pub fn consume_token<R, F>(self, len: usize, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        self.mode.increase_in_flight_counter();

        // (safety): creation of multiple WiFi devices with the same mode is impossible
        // in safe Rust, therefore only smoltcp _or_ embassy-net can be used at
        // one time
        static mut BUFFER: [u8; MTU] = [0u8; MTU];

        let buffer = unsafe { &mut BUFFER[..len] };

        let res = f(buffer);

        esp_wifi_send_data(self.mode.interface(), buffer);

        res
    }
}

#[cfg(feature = "smoltcp")]
impl TxToken for WifiTxToken {
    fn consume<R, F>(self, len: usize, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        self.consume_token(len, f)
    }
}

// FIXME data here has to be &mut because of `esp_wifi_internal_tx` signature,
// requiring a *mut ptr to the buffer Casting const to mut is instant UB, even
// though in reality `esp_wifi_internal_tx` copies the buffer into its own
// memory and does not modify
pub(crate) fn esp_wifi_send_data(interface: wifi_interface_t, data: &mut [u8]) {
    trace!("sending... {} bytes", data.len());
    dump_packet_info(data);

    let len = data.len() as u16;
    let ptr = data.as_mut_ptr().cast();

    let res = unsafe { esp_wifi_internal_tx(interface, ptr, len) };

    if res != 0 {
        warn!("esp_wifi_internal_tx {}", res);
        decrement_inflight_counter();
    } else {
        trace!("esp_wifi_internal_tx ok");
    }
}

fn apply_ap_config(config: &config::AccessPointConfiguration) -> Result<(), WifiError> {
    let mut cfg = wifi_config_t {
        ap: wifi_ap_config_t {
            ssid: [0; 32],
            password: [0; 64],
            ssid_len: 0,
            channel: config.channel,
            authmode: config.auth_method.to_raw(),
            ssid_hidden: if config.ssid_hidden { 1 } else { 0 },
            max_connection: config.max_connections as u8,
            beacon_interval: 100,
            pairwise_cipher: wifi_cipher_type_t_WIFI_CIPHER_TYPE_CCMP,
            ftm_responder: false,
            pmf_cfg: wifi_pmf_config_t {
                capable: true,
                required: false,
            },
            sae_pwe_h2e: 0,
            csa_count: 3,
            dtim_period: 2,
        },
    };

    if config.auth_method == config::AuthMethod::None && !config.password.is_empty() {
        return Err(WifiError::InternalError(
            InternalWifiError::EspErrInvalidArg,
        ));
    }

    unsafe {
        cfg.ap.ssid[0..(config.ssid.len())].copy_from_slice(config.ssid.as_bytes());
        cfg.ap.ssid_len = config.ssid.len() as u8;
        cfg.ap.password[0..(config.password.len())].copy_from_slice(config.password.as_bytes());

        esp_wifi_result!(esp_wifi_set_config(wifi_interface_t_WIFI_IF_AP, &mut cfg))
    }
}

fn apply_sta_config(config: &config::ClientConfiguration) -> Result<(), WifiError> {
    let mut cfg = wifi_config_t {
        sta: wifi_sta_config_t {
            ssid: [0; 32],
            password: [0; 64],
            scan_method: crate::CONFIG.scan_method,
            bssid_set: config.bssid.is_some(),
            bssid: config.bssid.unwrap_or_default(),
            channel: config.channel.unwrap_or(0),
            listen_interval: crate::CONFIG.listen_interval,
            sort_method: wifi_sort_method_t_WIFI_CONNECT_AP_BY_SIGNAL,
            threshold: wifi_scan_threshold_t {
                rssi: -99,
                authmode: config.auth_method.to_raw(),
            },
            pmf_cfg: wifi_pmf_config_t {
                capable: true,
                required: false,
            },
            sae_pwe_h2e: 3,
            _bitfield_align_1: [0; 0],
            _bitfield_1: __BindgenBitfieldUnit::new([0; 4]),
            failure_retry_cnt: crate::CONFIG.failure_retry_cnt,
            _bitfield_align_2: [0; 0],
            _bitfield_2: __BindgenBitfieldUnit::new([0; 4]),
            sae_pk_mode: 0, // ??
            sae_h2e_identifier: [0; 32],
        },
    };

    if config.auth_method == config::AuthMethod::None && !config.password.is_empty() {
        return Err(WifiError::InternalError(
            InternalWifiError::EspErrInvalidArg,
        ));
    }

    unsafe {
        cfg.sta.ssid[0..(config.ssid.len())].copy_from_slice(config.ssid.as_bytes());
        cfg.sta.password[0..(config.password.len())].copy_from_slice(config.password.as_bytes());

        esp_wifi_result!(esp_wifi_set_config(wifi_interface_t_WIFI_IF_STA, &mut cfg))
    }
}

fn apply_sta_eap_config(config: &config::EapClientConfiguration) -> Result<(), WifiError> {
    let mut cfg = wifi_config_t {
        sta: wifi_sta_config_t {
            ssid: [0; 32],
            password: [0; 64],
            scan_method: crate::CONFIG.scan_method,
            bssid_set: config.bssid.is_some(),
            bssid: config.bssid.unwrap_or_default(),
            channel: config.channel.unwrap_or(0),
            listen_interval: crate::CONFIG.listen_interval,
            sort_method: wifi_sort_method_t_WIFI_CONNECT_AP_BY_SIGNAL,
            threshold: wifi_scan_threshold_t {
                rssi: -99,
                authmode: config.auth_method.to_raw(),
            },
            pmf_cfg: wifi_pmf_config_t {
                capable: true,
                required: false,
            },
            sae_pwe_h2e: 3,
            _bitfield_align_1: [0; 0],
            _bitfield_1: __BindgenBitfieldUnit::new([0; 4]),
            failure_retry_cnt: crate::CONFIG.failure_retry_cnt,
            _bitfield_align_2: [0; 0],
            _bitfield_2: __BindgenBitfieldUnit::new([0; 4]),
            sae_pk_mode: 0, // ??
            sae_h2e_identifier: [0; 32],
        },
    };

    unsafe {
        cfg.sta.ssid[0..(config.ssid.len())].copy_from_slice(config.ssid.as_bytes());
        esp_wifi_result!(esp_wifi_set_config(wifi_interface_t_WIFI_IF_STA, &mut cfg))?;

        if let Some(identity) = &config.identity {
            esp_wifi_result!(esp_eap_client_set_identity(
                identity.as_str().as_ptr(),
                identity.len() as i32
            ))?;
        } else {
            esp_eap_client_clear_identity();
        }

        if let Some(username) = &config.username {
            esp_wifi_result!(esp_eap_client_set_username(
                username.as_str().as_ptr(),
                username.len() as i32
            ))?;
        } else {
            esp_eap_client_clear_username();
        }

        if let Some(password) = &config.password {
            esp_wifi_result!(esp_eap_client_set_password(
                password.as_str().as_ptr(),
                password.len() as i32
            ))?;
        } else {
            esp_eap_client_clear_password();
        }

        if let Some(new_password) = &config.new_password {
            esp_wifi_result!(esp_eap_client_set_new_password(
                new_password.as_str().as_ptr(),
                new_password.len() as i32
            ))?;
        } else {
            esp_eap_client_clear_new_password();
        }

        if let Some(pac_file) = &config.pac_file {
            esp_wifi_result!(esp_eap_client_set_pac_file(
                pac_file.as_ptr(),
                pac_file.len() as i32
            ))?;
        }

        if let Some(phase2_method) = &config.ttls_phase2_method {
            esp_wifi_result!(esp_eap_client_set_ttls_phase2_method(
                phase2_method.to_raw()
            ))?;
        }

        if let Some(ca_cert) = config.ca_cert {
            esp_wifi_result!(esp_eap_client_set_ca_cert(
                ca_cert.as_ptr(),
                ca_cert.len() as i32
            ))?;
        } else {
            esp_eap_client_clear_ca_cert();
        }

        if let Some((cert, key, password)) = config.certificate_and_key {
            let (pwd, pwd_len) = if let Some(pwd) = password {
                (pwd.as_ptr(), pwd.len() as i32)
            } else {
                (core::ptr::null(), 0)
            };

            esp_wifi_result!(esp_eap_client_set_certificate_and_key(
                cert.as_ptr(),
                cert.len() as i32,
                key.as_ptr(),
                key.len() as i32,
                pwd,
                pwd_len,
            ))?;
        } else {
            esp_eap_client_clear_certificate_and_key();
        }

        if let Some(cfg) = &config.eap_fast_config {
            let params = esp_eap_fast_config {
                fast_provisioning: cfg.fast_provisioning as i32,
                fast_max_pac_list_len: cfg.fast_max_pac_list_len as i32,
                fast_pac_format_binary: cfg.fast_pac_format_binary,
            };
            esp_wifi_result!(esp_eap_client_set_fast_params(params))?;
        }

        esp_wifi_result!(esp_eap_client_set_disable_time_check(!&config.time_check))?;

        // esp_eap_client_set_suiteb_192bit_certification unsupported because we build
        // without MBEDTLS

        // esp_eap_client_use_default_cert_bundle unsupported because we build without
        // MBEDTLS

        esp_wifi_result!(esp_wifi_sta_enterprise_enable())?;

        Ok(())
    }
}

fn dump_packet_info(_buffer: &mut [u8]) {
    #[cfg(dump_packets)]
    {
        info!("@WIFIFRAME {:?}", _buffer);
    }
}

#[doc(hidden)]
#[macro_export]
macro_rules! esp_wifi_result {
    ($value:expr) => {{
        use num_traits::FromPrimitive;
        let result = $value;
        if result != esp_wifi_sys::include::ESP_OK as i32 {
            warn!("{} returned an error: {}", stringify!($value), result);
            Err(WifiError::InternalError(unwrap!(FromPrimitive::from_i32(
                result
            ))))
        } else {
            Ok::<(), WifiError>(())
        }
    }};
}

pub(crate) mod embassy {
    use embassy_net_driver::{Capabilities, Driver, HardwareAddress, RxToken, TxToken};
    use esp_hal::asynch::AtomicWaker;

    use super::*;

    // We can get away with a single tx waker because the transmit queue is shared
    // between interfaces.
    pub(crate) static TRANSMIT_WAKER: AtomicWaker = AtomicWaker::new();

    pub(crate) static AP_RECEIVE_WAKER: AtomicWaker = AtomicWaker::new();
    pub(crate) static AP_LINK_STATE_WAKER: AtomicWaker = AtomicWaker::new();

    pub(crate) static STA_RECEIVE_WAKER: AtomicWaker = AtomicWaker::new();
    pub(crate) static STA_LINK_STATE_WAKER: AtomicWaker = AtomicWaker::new();

    impl RxToken for WifiRxToken {
        fn consume<R, F>(self, f: F) -> R
        where
            F: FnOnce(&mut [u8]) -> R,
        {
            self.consume_token(f)
        }
    }

    impl TxToken for WifiTxToken {
        fn consume<R, F>(self, len: usize, f: F) -> R
        where
            F: FnOnce(&mut [u8]) -> R,
        {
            self.consume_token(len, f)
        }
    }

    impl Driver for WifiDevice<'_> {
        type RxToken<'a>
            = WifiRxToken
        where
            Self: 'a;
        type TxToken<'a>
            = WifiTxToken
        where
            Self: 'a;

        fn receive(
            &mut self,
            cx: &mut core::task::Context<'_>,
        ) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
            self.mode.register_receive_waker(cx);
            self.mode.register_transmit_waker(cx);
            self.mode.rx_token()
        }

        fn transmit(&mut self, cx: &mut core::task::Context<'_>) -> Option<Self::TxToken<'_>> {
            self.mode.register_transmit_waker(cx);
            self.mode.tx_token()
        }

        fn link_state(
            &mut self,
            cx: &mut core::task::Context<'_>,
        ) -> embassy_net_driver::LinkState {
            self.mode.register_link_state_waker(cx);
            self.mode.link_state()
        }

        fn capabilities(&self) -> Capabilities {
            let mut caps = Capabilities::default();
            caps.max_transmission_unit = MTU;
            caps.max_burst_size = if crate::CONFIG.max_burst_size == 0 {
                None
            } else {
                Some(crate::CONFIG.max_burst_size)
            };
            caps
        }

        fn hardware_address(&self) -> HardwareAddress {
            HardwareAddress::Ethernet(self.mac_address())
        }
    }
}

pub(crate) fn apply_power_saving(ps: PowerSaveMode) -> Result<(), WifiError> {
    esp_wifi_result!(unsafe { esp_wifi_sys::include::esp_wifi_set_ps(ps.into()) })?;
    Ok(())
}

struct FreeApListOnDrop;
impl FreeApListOnDrop {
    pub fn defuse(self) {
        core::mem::forget(self);
    }
}

impl Drop for FreeApListOnDrop {
    fn drop(&mut self) {
        unsafe {
            include::esp_wifi_clear_ap_list();
        }
    }
}

#[non_exhaustive]
pub struct Interfaces<'d> {
    pub sta: WifiDevice<'d>,
    pub ap: WifiDevice<'d>,
    #[cfg(feature = "esp-now")]
    pub esp_now: crate::esp_now::EspNow<'d>,
    #[cfg(feature = "sniffer")]
    pub sniffer: Sniffer,
}

/// Create a WiFi controller and it's associated interfaces.
///
/// Dropping the controller will deinitialize / stop WiFi.
pub fn new<'d>(
    inited: &'d EspWifiController<'d>,
    _device: impl crate::hal::peripheral::Peripheral<P = crate::hal::peripherals::WIFI> + 'd,
) -> Result<(WifiController<'d>, Interfaces<'d>), WifiError> {
    if !inited.wifi() {
        crate::wifi::wifi_init()?;
    }
    Ok((
        WifiController {
            _phantom: Default::default(),
        },
        Interfaces {
            sta: WifiDevice {
                _phantom: Default::default(),
                mode: WifiDeviceMode::Sta,
            },
            ap: WifiDevice {
                _phantom: Default::default(),
                mode: WifiDeviceMode::Ap,
            },
            #[cfg(feature = "esp-now")]
            esp_now: crate::esp_now::EspNow::new_internal(),
            #[cfg(feature = "sniffer")]
            sniffer: Sniffer::new(),
        },
    ))
}

#[non_exhaustive]
pub struct WifiController<'d> {
    _phantom: PhantomData<&'d ()>,
}

impl Drop for WifiController<'_> {
    fn drop(&mut self) {
        if unwrap!(
            crate::flags::WIFI.fetch_update(Ordering::SeqCst, Ordering::SeqCst, |x| {
                Some(x.saturating_sub(1))
            })
        ) == 0
        {
            if let Err(e) = crate::wifi::wifi_deinit() {
                warn!("Failed to cleanly deinit wifi: {:?}", e);
            }
        }
    }
}

impl WifiController<'_> {
    /// Set CSI configuration and register the receiving callback.
    #[cfg(feature = "csi")]
    pub fn set_csi(
        &mut self,
        mut csi: config::CsiConfig,
        cb: impl FnMut(crate::binary::include::wifi_csi_info_t) + Send,
    ) -> Result<(), WifiError> {
        csi.apply_config()?;
        csi.set_receive_cb(cb)?;
        csi.set_csi(true)?;

        Ok(())
    }

    /// Set the wifi protocol.
    ///
    /// This will set the wifi protocol to the desired protocol, the default for
    /// this is: `WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N`
    ///
    /// # Arguments:
    ///
    /// * `protocols` - The desired protocols
    ///
    /// # Example:
    ///
    /// ```
    /// wifi_controller.set_protocol(config::Protocol::P802D11BGNLR.into());
    /// ```
    pub fn set_protocol(&mut self, protocols: EnumSet<config::Protocol>) -> Result<(), WifiError> {
        let protocol = protocols
            .into_iter()
            .map(|v| match v {
                config::Protocol::P802D11B => WIFI_PROTOCOL_11B,
                config::Protocol::P802D11BG => WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G,
                config::Protocol::P802D11BGN => {
                    WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N
                }
                config::Protocol::P802D11BGNLR => {
                    WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR
                }
                config::Protocol::P802D11LR => WIFI_PROTOCOL_LR,
                config::Protocol::P802D11BGNAX => {
                    WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_11AX
                }
            })
            .fold(0, |combined, protocol| combined | protocol) as u8;

        let mode = self.mode()?;
        if mode.is_sta() {
            esp_wifi_result!(unsafe {
                esp_wifi_set_protocol(wifi_interface_t_WIFI_IF_STA, protocol)
            })?;
        }
        if mode.is_ap() {
            esp_wifi_result!(unsafe {
                esp_wifi_set_protocol(wifi_interface_t_WIFI_IF_AP, protocol)
            })?;
        }

        Ok(())
    }

    /// Configures modem power saving
    pub fn set_power_saving(&mut self, ps: PowerSaveMode) -> Result<(), WifiError> {
        apply_power_saving(ps)
    }

    /// A blocking wifi network scan with caller-provided scanning options.
    pub fn scan_with_config_sync<const N: usize>(
        &mut self,
        config: config::ScanConfig<'_>,
    ) -> Result<(heapless::Vec<config::AccessPointInfo, N>, usize), WifiError> {
        esp_wifi_result!(crate::wifi::wifi_start_scan(true, config))?;

        let count = self.scan_result_count()?;
        let result = self.scan_results()?;

        Ok((result, count))
    }

    fn scan_result_count(&mut self) -> Result<usize, WifiError> {
        let mut bss_total: u16 = 0;

        // Prevents memory leak on error
        let guard = FreeApListOnDrop;

        unsafe { esp_wifi_result!(include::esp_wifi_scan_get_ap_num(&mut bss_total))? };

        guard.defuse();

        Ok(bss_total as usize)
    }

    fn scan_results<const N: usize>(
        &mut self,
    ) -> Result<heapless::Vec<config::AccessPointInfo, N>, WifiError> {
        let mut scanned = heapless::Vec::<config::AccessPointInfo, N>::new();
        let mut bss_total: u16 = N as u16;

        let mut records: [MaybeUninit<include::wifi_ap_record_t>; N] = [MaybeUninit::uninit(); N];

        // Prevents memory leak on error
        let guard = FreeApListOnDrop;

        unsafe {
            esp_wifi_result!(include::esp_wifi_scan_get_ap_records(
                &mut bss_total,
                records[0].as_mut_ptr(),
            ))?
        };

        guard.defuse();

        for i in 0..bss_total {
            let record = unsafe { MaybeUninit::assume_init_ref(&records[i as usize]) };
            let ap_info = convert_ap_info(record);

            scanned.push(ap_info).ok();
        }

        Ok(scanned)
    }

    /// A blocking wifi network scan with default scanning options.
    pub fn scan_n<const N: usize>(
        &mut self,
    ) -> Result<(heapless::Vec<config::AccessPointInfo, N>, usize), WifiError> {
        self.scan_with_config_sync(Default::default())
    }

    /// Starts the WiFi controller.
    pub fn start(&mut self) -> Result<(), WifiError> {
        crate::wifi::wifi_start()
    }

    /// Stops the WiFi controller.
    pub fn stop(&mut self) -> Result<(), WifiError> {
        self.stop_impl()
    }

    /// Connect WiFi station to the AP.
    ///
    /// - If station is connected , call [Self::disconnect] to disconnect.
    /// - Scanning will not be effective until connection between device and the
    ///   AP is established.
    /// - If device is scanning and connecting at the same time, it will abort
    ///   scanning and return a warning message and error
    pub fn connect(&mut self) -> Result<(), WifiError> {
        self.connect_impl()
    }

    /// Disconnect WiFi station from the AP.
    pub fn disconnect(&mut self) -> Result<(), WifiError> {
        self.disconnect_impl()
    }

    /// Get the supported capabilities of the controller.
    pub fn capabilities(&self) -> Result<EnumSet<crate::wifi::config::Capability>, WifiError> {
        let caps = enumset::enum_set! { config::Capability::Client | config::Capability::AccessPoint | config::Capability::Mixed };
        Ok(caps)
    }

    /// Set the configuration.
    ///
    /// This will set the mode accordingly.
    /// You need to use Wifi::connect() for connecting to an AP.
    pub fn set_configuration(&mut self, conf: &config::Configuration) -> Result<(), WifiError> {
        let mode = match conf {
            config::Configuration::None => {
                return Err(WifiError::InternalError(
                    InternalWifiError::EspErrInvalidArg,
                ));
            }
            config::Configuration::Client(_) => wifi_mode_t_WIFI_MODE_STA,
            config::Configuration::AccessPoint(_) => wifi_mode_t_WIFI_MODE_AP,
            config::Configuration::Mixed(_, _) => wifi_mode_t_WIFI_MODE_APSTA,
            config::Configuration::EapClient(_) => wifi_mode_t_WIFI_MODE_STA,
        };

        esp_wifi_result!(unsafe { esp_wifi_set_mode(mode) })?;

        match conf {
            config::Configuration::None => {
                return Err(WifiError::InternalError(
                    InternalWifiError::EspErrInvalidArg,
                ));
            }
            config::Configuration::Client(config) => {
                apply_sta_config(config)?;
            }
            config::Configuration::AccessPoint(config) => {
                apply_ap_config(config)?;
            }
            config::Configuration::Mixed(sta_config, ap_config) => {
                apply_ap_config(ap_config)?;
                apply_sta_config(sta_config)?;
            }
            config::Configuration::EapClient(config) => {
                apply_sta_eap_config(config)?;
            }
        };

        Ok(())
    }

    /// Set the WiFi mode.
    ///
    /// This will override the mode inferred by [Self::set_configuration].
    pub fn set_mode(&mut self, mode: config::WifiMode) -> Result<(), WifiError> {
        esp_wifi_result!(unsafe { esp_wifi_set_mode(mode.into()) })?;
        Ok(())
    }

    fn stop_impl(&mut self) -> Result<(), WifiError> {
        esp_wifi_result!(unsafe { esp_wifi_stop() })
    }

    fn connect_impl(&mut self) -> Result<(), WifiError> {
        esp_wifi_result!(unsafe { esp_wifi_connect() })
    }

    fn disconnect_impl(&mut self) -> Result<(), WifiError> {
        esp_wifi_result!(unsafe { esp_wifi_disconnect() })
    }

    /// Checks if the WiFi controller has started.
    ///
    /// This function should be called after the `start` method to verify if the
    /// WiFi has started successfully.
    pub fn is_started(&self) -> Result<bool, WifiError> {
        if matches!(
            crate::wifi::sta_state(),
            WifiState::StaStarted | WifiState::StaConnected | WifiState::StaDisconnected
        ) {
            return Ok(true);
        }
        if matches!(crate::wifi::ap_state(), WifiState::ApStarted) {
            return Ok(true);
        }
        Ok(false)
    }

    /// Checks if the WiFi controller is connected to an AP.
    ///
    /// This function should be called after the `connect` method to verify if
    /// the connection was successful.
    pub fn is_connected(&self) -> Result<bool, WifiError> {
        match crate::wifi::sta_state() {
            crate::wifi::WifiState::StaConnected => Ok(true),
            crate::wifi::WifiState::StaDisconnected => Err(WifiError::Disconnected),
            // FIXME: Should any other enum value trigger an error instead of returning false?
            _ => Ok(false),
        }
    }

    fn mode(&self) -> Result<config::WifiMode, WifiError> {
        config::WifiMode::current()
    }

    /// Async version of [`crate::wifi::WifiController`]'s `scan_n` method
    pub async fn scan_n_async<const N: usize>(
        &mut self,
    ) -> Result<(heapless::Vec<config::AccessPointInfo, N>, usize), WifiError> {
        self.scan_with_config_async(Default::default()).await
    }

    /// An async wifi network scan with caller-provided scanning options.
    pub async fn scan_with_config_async<const N: usize>(
        &mut self,
        config: config::ScanConfig<'_>,
    ) -> Result<(heapless::Vec<config::AccessPointInfo, N>, usize), WifiError> {
        Self::clear_events(WifiEvent::ScanDone);
        esp_wifi_result!(wifi_start_scan(false, config))?;

        // Prevents memory leak if `scan_n`'s future is dropped.
        let guard = FreeApListOnDrop;
        WifiEventFuture::new(WifiEvent::ScanDone).await;

        guard.defuse();

        let count = self.scan_result_count()?;
        let result = self.scan_results()?;

        Ok((result, count))
    }

    /// Async version of [`crate::wifi::WifiController`]'s `start` method
    pub async fn start_async(&mut self) -> Result<(), WifiError> {
        let mut events = enumset::enum_set! {};

        let mode = self.mode()?;
        if mode.is_ap() {
            events |= WifiEvent::ApStart;
        }
        if mode.is_sta() {
            events |= WifiEvent::StaStart;
        }

        Self::clear_events(events);

        wifi_start()?;

        self.wait_for_all_events(events, false).await;

        Ok(())
    }

    /// Async version of [`crate::wifi::WifiController`]'s `stop` method
    pub async fn stop_async(&mut self) -> Result<(), WifiError> {
        let mut events = enumset::enum_set! {};

        let mode = self.mode()?;
        if mode.is_ap() {
            events |= WifiEvent::ApStop;
        }
        if mode.is_sta() {
            events |= WifiEvent::StaStop;
        }

        Self::clear_events(events);

        crate::wifi::WifiController::stop_impl(self)?;

        self.wait_for_all_events(events, false).await;

        reset_ap_state();
        reset_sta_state();

        Ok(())
    }

    /// Async version of [`crate::wifi::WifiController`]'s `connect` method
    pub async fn connect_async(&mut self) -> Result<(), WifiError> {
        Self::clear_events(WifiEvent::StaConnected | WifiEvent::StaDisconnected);

        let err = crate::wifi::WifiController::connect_impl(self).err();

        if MultiWifiEventFuture::new(WifiEvent::StaConnected | WifiEvent::StaDisconnected)
            .await
            .contains(WifiEvent::StaDisconnected)
        {
            Err(err.unwrap_or(WifiError::Disconnected))
        } else {
            Ok(())
        }
    }

    /// Async version of [`crate::wifi::WifiController`]'s `Disconnect`
    /// method
    pub async fn disconnect_async(&mut self) -> Result<(), WifiError> {
        // If not connected, this will do nothing.
        // It will also wait forever for a `StaDisconnected` event that will never come.
        // Return early instead of hanging.
        if !matches!(self.is_connected(), Ok(true)) {
            return Ok(());
        }

        Self::clear_events(WifiEvent::StaDisconnected);
        crate::wifi::WifiController::disconnect_impl(self)?;
        WifiEventFuture::new(WifiEvent::StaDisconnected).await;

        Ok(())
    }

    fn clear_events(events: impl Into<EnumSet<WifiEvent>>) {
        WIFI_EVENTS.with(|evts| evts.get_mut().remove_all(events.into()));
    }

    /// Wait for one [`WifiEvent`].
    pub async fn wait_for_event(&mut self, event: WifiEvent) {
        Self::clear_events(event);
        WifiEventFuture::new(event).await
    }

    /// Wait for one of multiple [`WifiEvent`]s. Returns the events that
    /// occurred while waiting.
    pub async fn wait_for_events(
        &mut self,
        events: EnumSet<WifiEvent>,
        clear_pending: bool,
    ) -> EnumSet<WifiEvent> {
        if clear_pending {
            Self::clear_events(events);
        }
        MultiWifiEventFuture::new(events).await
    }

    /// Wait for multiple [`WifiEvent`]s.
    pub async fn wait_for_all_events(
        &mut self,
        mut events: EnumSet<WifiEvent>,
        clear_pending: bool,
    ) {
        if clear_pending {
            Self::clear_events(events);
        }

        while !events.is_empty() {
            let fired = MultiWifiEventFuture::new(events).await;
            events -= fired;
        }
    }
}

impl WifiEvent {
    pub(crate) fn waker(&self) -> &'static AtomicWaker {
        // for now use only one waker for all events
        // if that ever becomes a problem we might want to pick some events to use their
        // own
        static WAKER: AtomicWaker = AtomicWaker::new();
        &WAKER
    }
}

#[must_use = "futures do nothing unless you `.await` or poll them"]
pub(crate) struct WifiEventFuture {
    event: WifiEvent,
}

impl WifiEventFuture {
    /// Creates a new `Future` for the specified WiFi event.
    pub fn new(event: WifiEvent) -> Self {
        Self { event }
    }
}

impl core::future::Future for WifiEventFuture {
    type Output = ();

    fn poll(
        self: core::pin::Pin<&mut Self>,
        cx: &mut core::task::Context<'_>,
    ) -> Poll<Self::Output> {
        self.event.waker().register(cx.waker());
        if WIFI_EVENTS.with(|events| events.get_mut().remove(self.event)) {
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    }
}

#[must_use = "futures do nothing unless you `.await` or poll them"]
pub(crate) struct MultiWifiEventFuture {
    event: EnumSet<WifiEvent>,
}

impl MultiWifiEventFuture {
    /// Creates a new `Future` for the specified set of WiFi events.
    pub fn new(event: EnumSet<WifiEvent>) -> Self {
        Self { event }
    }
}

impl core::future::Future for MultiWifiEventFuture {
    type Output = EnumSet<WifiEvent>;

    fn poll(
        self: core::pin::Pin<&mut Self>,
        cx: &mut core::task::Context<'_>,
    ) -> Poll<Self::Output> {
        let output = WIFI_EVENTS.with(|events| {
            let events = events.get_mut();
            let active = events.intersection(self.event);
            events.remove_all(active);
            active
        });
        if output.is_empty() {
            for event in self.event.iter() {
                event.waker().register(cx.waker());
            }

            Poll::Pending
        } else {
            Poll::Ready(output)
        }
    }
}
