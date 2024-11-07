//! ESP-NOW is a kind of connectionless Wi-Fi communication protocol that is
//! defined by Espressif.
//!
//! In ESP-NOW, application data is encapsulated in a vendor-specific action
//! frame and then transmitted from one Wi-Fi device to another without
//! connection. CTR with CBC-MAC Protocol(CCMP) is used to protect the action
//! frame for security. ESP-NOW is widely used in smart light, remote
//! controlling, sensor, etc.
//!
//! For more information see https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_now.html

use alloc::{boxed::Box, collections::vec_deque::VecDeque};
use core::{cell::RefCell, fmt::Debug, marker::PhantomData};

use critical_section::Mutex;
use enumset::EnumSet;
use portable_atomic::{AtomicBool, AtomicU8, Ordering};

#[cfg(not(coex))]
use crate::config::PowerSaveMode;
use crate::{
    binary::include::*,
    hal::peripheral::{Peripheral, PeripheralRef},
    wifi::{Protocol, RxControlInfo, WifiError},
    EspWifiController,
};

const RECEIVE_QUEUE_SIZE: usize = 10;

/// Maximum payload length
pub const ESP_NOW_MAX_DATA_LEN: usize = 250;

/// Broadcast address
pub const BROADCAST_ADDRESS: [u8; 6] = [0xffu8, 0xffu8, 0xffu8, 0xffu8, 0xffu8, 0xffu8];

// Stores received packets until dequeued by the user
static RECEIVE_QUEUE: Mutex<RefCell<VecDeque<ReceivedData>>> =
    Mutex::new(RefCell::new(VecDeque::new()));

/// This atomic behaves like a guard, so we need strict memory ordering when
/// operating it.
///
/// This flag indicates whether the send callback has been called after a
/// sending.
static ESP_NOW_SEND_CB_INVOKED: AtomicBool = AtomicBool::new(false);
/// Status of esp now send, true for success, false for failure
static ESP_NOW_SEND_STATUS: AtomicBool = AtomicBool::new(true);

macro_rules! check_error {
    ($block:block) => {
        match unsafe { $block } {
            0 => Ok(()),
            res => Err(EspNowError::Error(Error::from_code(res as u32))),
        }
    };
}

/// Internal errors that can occur with ESP-NOW.
#[repr(u32)]
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// ESP-NOW is not initialized.
    NotInitialized  = 12389,

    /// Invalid argument.
    InvalidArgument = 12390,

    /// Indicates that there was insufficient memory to complete the operation.
    OutOfMemory     = 12391,

    /// ESP-NOW peer list is full.
    PeerListFull    = 12392,

    /// ESP-NOW peer is not found.
    NotFound        = 12393,

    /// Internal error.
    InternalError   = 12394,

    /// ESP-NOW peer already exists.
    PeerExists      = 12395,

    /// Interface error.
    InterfaceError  = 12396,

    /// Represents any other error not covered by the above variants, with an
    /// associated error code.
    Other(u32),
}

impl Error {
    pub fn from_code(code: u32) -> Error {
        match code {
            12389 => Error::NotInitialized,
            12390 => Error::InvalidArgument,
            12391 => Error::OutOfMemory,
            12392 => Error::PeerListFull,
            12393 => Error::NotFound,
            12394 => Error::InternalError,
            12395 => Error::PeerExists,
            12396 => Error::InterfaceError,
            _ => Error::Other(code),
        }
    }
}

/// Common errors that can occur while using ESP-NOW driver.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum EspNowError {
    /// Internal Error.
    Error(Error),
    /// Failed to send an ESP-NOW message.
    SendFailed,
    /// Attempt to create `EspNow` instance twice.
    DuplicateInstance,
    /// Initialization error
    Initialization(WifiError),
}

impl From<WifiError> for EspNowError {
    fn from(f: WifiError) -> Self {
        Self::Initialization(f)
    }
}

/// Holds the count of peers in an ESP-NOW communication context.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PeerCount {
    /// The total number of peers.
    pub total_count: i32,

    /// The number of encrypted peers.
    pub encrypted_count: i32,
}

/// ESP-NOW rate of specified interface.
#[repr(u32)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum WifiPhyRate {
    /// < 1 Mbps with long preamble
    Rate1mL = 0,
    /// < 2 Mbps with long preamble
    Rate2m,
    /// < 5.5 Mbps with long preamble
    Rate5mL,
    /// < 11 Mbps with long preamble
    Rate11mL,
    /// < 2 Mbps with short preamble
    Rate2mS,
    /// < 5.5 Mbps with short preamble
    Rate5mS,
    /// < 11 Mbps with short preamble
    Rate11mS,
    /// < 48 Mbps
    Rate48m,
    /// < 24 Mbps
    Rate24m,
    /// < 12 Mbps
    Rate12m,
    /// < 6 Mbps
    Rate6m,
    /// < 54 Mbps
    Rate54m,
    /// < 36 Mbps
    Rate36m,
    /// < 18 Mbps
    Rate18m,
    /// < 9 Mbps
    Rate9m,
    /// < MCS0 with long GI, 6.5 Mbps for 20MHz, 13.5 Mbps for 40MHz
    RateMcs0Lgi,
    /// < MCS1 with long GI, 13 Mbps for 20MHz, 27 Mbps for 40MHz
    RateMcs1Lgi,
    /// < MCS2 with long GI, 19.5 Mbps for 20MHz, 40.5 Mbps for 40MHz
    RateMcs2Lgi,
    /// < MCS3 with long GI, 26 Mbps for 20MHz, 54 Mbps for 40MHz
    RateMcs3Lgi,
    /// < MCS4 with long GI, 39 Mbps for 20MHz, 81 Mbps for 40MHz
    RateMcs4Lgi,
    /// < MCS5 with long GI, 52 Mbps for 20MHz, 108 Mbps for 40MHz
    RateMcs5Lgi,
    /// < MCS6 with long GI, 58.5 Mbps for 20MHz, 121.5 Mbps for 40MHz
    RateMcs6Lgi,
    /// < MCS7 with long GI, 65 Mbps for 20MHz, 135 Mbps for 40MHz
    RateMcs7Lgi,
    /// < MCS0 with short GI, 7.2 Mbps for 20MHz, 15 Mbps for 40MHz
    RateMcs0Sgi,
    /// < MCS1 with short GI, 14.4 Mbps for 20MHz, 30 Mbps for 40MHz
    RateMcs1Sgi,
    /// < MCS2 with short GI, 21.7 Mbps for 20MHz, 45 Mbps for 40MHz
    RateMcs2Sgi,
    /// < MCS3 with short GI, 28.9 Mbps for 20MHz, 60 Mbps for 40MHz
    RateMcs3Sgi,
    /// < MCS4 with short GI, 43.3 Mbps for 20MHz, 90 Mbps for 40MHz
    RateMcs4Sgi,
    /// < MCS5 with short GI, 57.8 Mbps for 20MHz, 120 Mbps for 40MHz
    RateMcs5Sgi,
    /// < MCS6 with short GI, 65 Mbps for 20MHz, 135 Mbps for 40MHz
    RateMcs6Sgi,
    /// < MCS7 with short GI, 72.2 Mbps for 20MHz, 150 Mbps for 40MHz
    RateMcs7Sgi,
    /// < 250 Kbps
    RateLora250k,
    /// < 500 Kbps
    RateLora500k,
    /// Max
    RateMax,
}

/// ESP-NOW peer information parameters.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PeerInfo {
    /// ESP-NOW peer MAC address that is also the MAC address of station or
    /// softap.
    pub peer_address: [u8; 6],

    /// ESP-NOW peer local master key that is used to encrypt data.
    pub lmk: Option<[u8; 16]>,

    /// Wi-Fi channel that peer uses to send/receive ESP-NOW data.
    pub channel: Option<u8>,

    /// Whether the data sent/received by this peer is encrypted.
    pub encrypt: bool,
    // we always use STA for now
}

/// Information about a received packet.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ReceiveInfo {
    /// The source address of the received packet.
    pub src_address: [u8; 6],

    /// The destination address of the received packet.
    pub dst_address: [u8; 6],

    /// Rx control info of ESP-NOW packet.
    pub rx_control: RxControlInfo,
}

/// Stores information about the received data, including the packet content and
/// associated information.
#[derive(Clone)]
pub struct ReceivedData {
    data: Box<[u8]>,
    pub info: ReceiveInfo,
}

impl ReceivedData {
    /// Returns the received payload.
    pub fn data(&self) -> &[u8] {
        &self.data
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for ReceivedData {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "ReceivedData {}, Info {}", &self.data[..], &self.info,)
    }
}

impl Debug for ReceivedData {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("ReceivedData")
            .field("data", &self.data())
            .field("info", &self.info)
            .finish()
    }
}

/// A token used to create an `EspNow` instance while Wi-Fi is enabled.
pub struct EspNowWithWifiCreateToken {
    _private: (),
}

/// Enables ESP-NOW while keeping Wi-Fi active.
pub fn enable_esp_now_with_wifi(
    device: crate::hal::peripherals::WIFI,
) -> (crate::hal::peripherals::WIFI, EspNowWithWifiCreateToken) {
    (device, EspNowWithWifiCreateToken { _private: () })
}

/// Manages the `EspNow` instance lifecycle while ensuring it remains active.
pub struct EspNowManager<'d> {
    _rc: EspNowRc<'d>,
}

impl EspNowManager<'_> {
    /// Set the wifi protocol.
    ///
    /// This will set the wifi protocol to the desired protocol
    ///
    /// # Arguments:
    ///
    /// * `protocols` - The desired protocols
    pub fn set_protocol(&self, protocols: EnumSet<Protocol>) -> Result<(), EspNowError> {
        let mut protocol = 0u8;

        protocols.into_iter().for_each(|v| match v {
            Protocol::P802D11B => protocol |= WIFI_PROTOCOL_11B as u8,
            Protocol::P802D11BG => protocol |= WIFI_PROTOCOL_11B as u8 | WIFI_PROTOCOL_11G as u8,
            Protocol::P802D11BGN => {
                protocol |=
                    WIFI_PROTOCOL_11B as u8 | WIFI_PROTOCOL_11G as u8 | WIFI_PROTOCOL_11N as u8
            }
            Protocol::P802D11BGNLR => {
                protocol |= WIFI_PROTOCOL_11B as u8
                    | WIFI_PROTOCOL_11G as u8
                    | WIFI_PROTOCOL_11N as u8
                    | WIFI_PROTOCOL_LR as u8
            }
            Protocol::P802D11LR => protocol |= WIFI_PROTOCOL_LR as u8,
            Protocol::P802D11BGNAX => {
                protocol |= WIFI_PROTOCOL_11B as u8
                    | WIFI_PROTOCOL_11G as u8
                    | WIFI_PROTOCOL_11N as u8
                    | WIFI_PROTOCOL_11AX as u8
            }
        });

        let mut mode = wifi_mode_t_WIFI_MODE_NULL;
        check_error!({ esp_wifi_get_mode(&mut mode) })?;

        if mode == wifi_mode_t_WIFI_MODE_STA || mode == wifi_mode_t_WIFI_MODE_APSTA {
            check_error!({ esp_wifi_set_protocol(wifi_interface_t_WIFI_IF_STA, protocol) })?;
        }
        if mode == wifi_mode_t_WIFI_MODE_AP || mode == wifi_mode_t_WIFI_MODE_APSTA {
            check_error!({ esp_wifi_set_protocol(wifi_interface_t_WIFI_IF_AP, protocol) })?;
        }

        Ok(())
    }

    #[cfg(not(coex))]
    /// Configures modem power saving
    pub fn set_power_saving(&self, ps: PowerSaveMode) -> Result<(), WifiError> {
        crate::wifi::apply_power_saving(ps)
    }

    /// Set primary WiFi channel.
    /// Should only be used when using ESP-NOW without AP or STA.
    pub fn set_channel(&self, channel: u8) -> Result<(), EspNowError> {
        check_error!({ esp_wifi_set_channel(channel, 0) })
    }

    /// Get the version of ESP-NOW.
    pub fn get_version(&self) -> Result<u32, EspNowError> {
        let mut version = 0u32;
        check_error!({ esp_now_get_version(&mut version as *mut u32) })?;
        Ok(version)
    }

    /// Add a peer to the list of known peers.
    pub fn add_peer(&self, peer: PeerInfo) -> Result<(), EspNowError> {
        let raw_peer = esp_now_peer_info_t {
            peer_addr: peer.peer_address,
            lmk: peer.lmk.unwrap_or([0u8; 16]),
            channel: peer.channel.unwrap_or(0),
            ifidx: wifi_interface_t_WIFI_IF_STA,
            encrypt: peer.encrypt,
            priv_: core::ptr::null_mut(),
        };
        check_error!({ esp_now_add_peer(&raw_peer as *const _) })
    }

    /// Remove the given peer.
    pub fn remove_peer(&self, peer_address: &[u8; 6]) -> Result<(), EspNowError> {
        check_error!({ esp_now_del_peer(peer_address.as_ptr()) })
    }

    /// Modify a peer information.
    pub fn modify_peer(&self, peer: PeerInfo) -> Result<(), EspNowError> {
        let raw_peer = esp_now_peer_info_t {
            peer_addr: peer.peer_address,
            lmk: peer.lmk.unwrap_or([0u8; 16]),
            channel: peer.channel.unwrap_or(0),
            ifidx: wifi_interface_t_WIFI_IF_STA,
            encrypt: peer.encrypt,
            priv_: core::ptr::null_mut(),
        };
        check_error!({ esp_now_mod_peer(&raw_peer as *const _) })
    }

    /// Get peer by MAC address.
    pub fn get_peer(&self, peer_address: &[u8; 6]) -> Result<PeerInfo, EspNowError> {
        let mut raw_peer = esp_now_peer_info_t {
            peer_addr: [0u8; 6],
            lmk: [0u8; 16],
            channel: 0,
            ifidx: 0,
            encrypt: false,
            priv_: core::ptr::null_mut(),
        };
        check_error!({ esp_now_get_peer(peer_address.as_ptr(), &mut raw_peer as *mut _) })?;

        Ok(PeerInfo {
            peer_address: raw_peer.peer_addr,
            lmk: if raw_peer.lmk.is_empty() {
                None
            } else {
                Some(raw_peer.lmk)
            },
            channel: if raw_peer.channel != 0 {
                Some(raw_peer.channel)
            } else {
                None
            },
            encrypt: raw_peer.encrypt,
        })
    }

    /// Fetch a peer from peer list.
    ///
    /// Only returns peers which address is unicast, for multicast/broadcast
    /// addresses, the function will skip the entry and find the next in the
    /// peer list.
    pub fn fetch_peer(&self, from_head: bool) -> Result<PeerInfo, EspNowError> {
        let mut raw_peer = esp_now_peer_info_t {
            peer_addr: [0u8; 6],
            lmk: [0u8; 16],
            channel: 0,
            ifidx: 0,
            encrypt: false,
            priv_: core::ptr::null_mut(),
        };
        check_error!({ esp_now_fetch_peer(from_head, &mut raw_peer as *mut _) })?;

        Ok(PeerInfo {
            peer_address: raw_peer.peer_addr,
            lmk: if raw_peer.lmk.is_empty() {
                None
            } else {
                Some(raw_peer.lmk)
            },
            channel: if raw_peer.channel != 0 {
                Some(raw_peer.channel)
            } else {
                None
            },
            encrypt: raw_peer.encrypt,
        })
    }

    /// Check is peer is known.
    pub fn peer_exists(&self, peer_address: &[u8; 6]) -> bool {
        unsafe { esp_now_is_peer_exist(peer_address.as_ptr()) }
    }

    /// Get the number of peers.
    pub fn peer_count(&self) -> Result<PeerCount, EspNowError> {
        let mut peer_num = esp_now_peer_num_t {
            total_num: 0,
            encrypt_num: 0,
        };
        check_error!({ esp_now_get_peer_num(&mut peer_num as *mut _) })?;

        Ok(PeerCount {
            total_count: peer_num.total_num,
            encrypted_count: peer_num.encrypt_num,
        })
    }

    /// Set the primary master key.
    pub fn set_pmk(&self, pmk: &[u8; 16]) -> Result<(), EspNowError> {
        check_error!({ esp_now_set_pmk(pmk.as_ptr()) })
    }

    /// Set wake window for esp_now to wake up in interval unit.
    ///
    /// Window is milliseconds the chip keep waked each interval, from 0 to
    /// 65535.
    pub fn set_wake_window(&self, wake_window: u16) -> Result<(), EspNowError> {
        check_error!({ esp_now_set_wake_window(wake_window) })
    }

    /// Configure ESP-NOW rate.
    pub fn set_rate(&self, rate: WifiPhyRate) -> Result<(), EspNowError> {
        check_error!({ esp_wifi_config_espnow_rate(wifi_interface_t_WIFI_IF_STA, rate as u32,) })
    }
}

impl<'d> Drop for EspNowManager<'d> {
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

/// This is the sender part of ESP-NOW. You can get this sender by splitting
/// a `EspNow` instance.
///
/// You need a lock when using this sender in multiple tasks.
/// **DO NOT USE** a lock implementation that disables interrupts since the
/// completion of a sending requires waiting for a callback invoked in an
/// interrupt.
pub struct EspNowSender<'d> {
    _rc: EspNowRc<'d>,
}

impl EspNowSender<'_> {
    /// Send data to peer
    ///
    /// The peer needs to be added to the peer list first.
    pub fn send<'s>(
        &'s mut self,
        dst_addr: &[u8; 6],
        data: &[u8],
    ) -> Result<SendWaiter<'s>, EspNowError> {
        ESP_NOW_SEND_CB_INVOKED.store(false, Ordering::Release);
        check_error!({ esp_now_send(dst_addr.as_ptr(), data.as_ptr(), data.len()) })?;
        Ok(SendWaiter(PhantomData))
    }
}

#[allow(unknown_lints)]
#[allow(clippy::too_long_first_doc_paragraph)]
/// This struct is returned by a sync esp now send. Invoking `wait` method of
/// this struct will block current task until the callback function of esp now
/// send is called and return the status of previous sending.
///
/// This waiter borrows the sender, so when used in multiple tasks, the lock
/// will only be released when the waiter is dropped or consumed via `wait`.
///
/// When using a lock that disables interrupts, the waiter will block forever
/// since the callback which signals the completion of sending will never be
/// invoked.
#[must_use]
pub struct SendWaiter<'s>(PhantomData<&'s mut EspNowSender<'s>>);

impl SendWaiter<'_> {
    /// Wait for the previous sending to complete, i.e. the send callback is
    /// invoked with status of the sending.
    pub fn wait(self) -> Result<(), EspNowError> {
        // prevent redundant waiting since we waits for the callback in the Drop
        // implementation
        core::mem::forget(self);
        while !ESP_NOW_SEND_CB_INVOKED.load(Ordering::Acquire) {}

        if ESP_NOW_SEND_STATUS.load(Ordering::Relaxed) {
            Ok(())
        } else {
            Err(EspNowError::SendFailed)
        }
    }
}

impl Drop for SendWaiter<'_> {
    /// wait for the send to complete to prevent the lock on `EspNowSender` get
    /// unlocked before a callback is invoked.
    fn drop(&mut self) {
        while !ESP_NOW_SEND_CB_INVOKED.load(Ordering::Acquire) {}
    }
}

/// This is the receiver part of ESP-NOW. You can get this receiver by splitting
/// an `EspNow` instance.
pub struct EspNowReceiver<'d> {
    _rc: EspNowRc<'d>,
}

impl EspNowReceiver<'_> {
    /// Receives data from the ESP-NOW queue.
    pub fn receive(&self) -> Option<ReceivedData> {
        critical_section::with(|cs| {
            let mut queue = RECEIVE_QUEUE.borrow_ref_mut(cs);
            queue.pop_front()
        })
    }
}

/// The reference counter for properly deinit espnow after all parts are
/// dropped.
struct EspNowRc<'d> {
    rc: &'static AtomicU8,
    inner: PhantomData<EspNow<'d>>,
}

impl EspNowRc<'_> {
    fn new() -> Result<Self, EspNowError> {
        static ESP_NOW_RC: AtomicU8 = AtomicU8::new(0);
        // The reference counter is not 0, which means there is another instance of
        // EspNow, which is not allowed
        if ESP_NOW_RC.fetch_add(1, Ordering::AcqRel) != 0 {
            return Err(EspNowError::DuplicateInstance);
        }

        Ok(Self {
            rc: &ESP_NOW_RC,
            inner: PhantomData,
        })
    }
}

impl Clone for EspNowRc<'_> {
    fn clone(&self) -> Self {
        self.rc.fetch_add(1, Ordering::Release);
        Self {
            rc: self.rc,
            inner: PhantomData,
        }
    }
}

impl Drop for EspNowRc<'_> {
    fn drop(&mut self) {
        if self.rc.fetch_sub(1, Ordering::AcqRel) == 1 {
            unsafe {
                esp_now_unregister_recv_cb();
                esp_now_deinit();
            }
        }
    }
}

#[allow(unknown_lints)]
#[allow(clippy::too_long_first_doc_paragraph)]
/// ESP-NOW is a kind of connectionless Wi-Fi communication protocol that is
/// defined by Espressif. In ESP-NOW, application data is encapsulated in a
/// vendor-specific action frame and then transmitted from one Wi-Fi device to
/// another without connection. CTR with CBC-MAC Protocol(CCMP) is used to
/// protect the action frame for security. ESP-NOW is widely used in smart
/// light, remote controlling, sensor, etc.
///
/// Currently this implementation (when used together with traditional Wi-Fi)
/// ONLY support STA mode.
pub struct EspNow<'d> {
    manager: EspNowManager<'d>,
    sender: EspNowSender<'d>,
    receiver: EspNowReceiver<'d>,
    _phantom: PhantomData<&'d ()>,
}

impl<'d> EspNow<'d> {
    /// Creates an `EspNow` instance.
    pub fn new(
        inited: &'d EspWifiController<'d>,
        device: impl Peripheral<P = crate::hal::peripherals::WIFI> + 'd,
    ) -> Result<EspNow<'d>, EspNowError> {
        EspNow::new_internal(inited, Some(device.into_ref()))
    }

    /// Creates an `EspNow` instance with support for Wi-Fi coexistence.
    pub fn new_with_wifi(
        inited: &'d EspWifiController<'d>,
        _token: EspNowWithWifiCreateToken,
    ) -> Result<EspNow<'d>, EspNowError> {
        EspNow::new_internal(
            inited,
            None::<PeripheralRef<'d, crate::hal::peripherals::WIFI>>,
        )
    }

    fn new_internal(
        inited: &'d EspWifiController<'d>,
        device: Option<PeripheralRef<'d, crate::hal::peripherals::WIFI>>,
    ) -> Result<EspNow<'d>, EspNowError> {
        if !inited.wifi() {
            // if wifi isn't already enabled, and we try to coexist - panic
            assert!(device.is_some());
            crate::wifi::wifi_init()?;
        }

        let espnow_rc = EspNowRc::new()?;
        let esp_now = EspNow {
            manager: EspNowManager {
                _rc: espnow_rc.clone(),
            },
            sender: EspNowSender {
                _rc: espnow_rc.clone(),
            },
            receiver: EspNowReceiver { _rc: espnow_rc },
            _phantom: PhantomData,
        };
        check_error!({ esp_wifi_set_mode(wifi_mode_t_WIFI_MODE_STA) })?;
        check_error!({ esp_wifi_start() })?;
        check_error!({
            esp_wifi_set_inactive_time(wifi_interface_t_WIFI_IF_STA, crate::CONFIG.beacon_timeout)
        })?;
        check_error!({ esp_now_init() })?;
        check_error!({ esp_now_register_recv_cb(Some(rcv_cb)) })?;
        check_error!({ esp_now_register_send_cb(Some(send_cb)) })?;

        esp_now.add_peer(PeerInfo {
            peer_address: BROADCAST_ADDRESS,
            lmk: None,
            channel: None,
            encrypt: false,
        })?;

        Ok(esp_now)
    }

    /// Splits the `EspNow` instance into its manager, sender, and receiver
    /// components.
    pub fn split(self) -> (EspNowManager<'d>, EspNowSender<'d>, EspNowReceiver<'d>) {
        (self.manager, self.sender, self.receiver)
    }

    /// Set the wifi protocol.
    ///
    /// This will set the wifi protocol to the desired protocol
    ///
    /// # Arguments:
    ///
    /// * `protocols` - The desired protocols
    pub fn set_protocol(&self, protocols: EnumSet<Protocol>) -> Result<(), EspNowError> {
        self.manager.set_protocol(protocols)
    }

    /// Set primary WiFi channel.
    /// Should only be used when using ESP-NOW without AP or STA.
    pub fn set_channel(&self, channel: u8) -> Result<(), EspNowError> {
        self.manager.set_channel(channel)
    }

    /// Get the version of ESP-NOW.
    pub fn get_version(&self) -> Result<u32, EspNowError> {
        self.manager.get_version()
    }

    /// Add a peer to the list of known peers.
    pub fn add_peer(&self, peer: PeerInfo) -> Result<(), EspNowError> {
        self.manager.add_peer(peer)
    }

    /// Remove the given peer.
    pub fn remove_peer(&self, peer_address: &[u8; 6]) -> Result<(), EspNowError> {
        self.manager.remove_peer(peer_address)
    }

    /// Modify a peer information.
    pub fn modify_peer(&self, peer: PeerInfo) -> Result<(), EspNowError> {
        self.manager.modify_peer(peer)
    }

    /// Get peer by MAC address.
    pub fn get_peer(&self, peer_address: &[u8; 6]) -> Result<PeerInfo, EspNowError> {
        self.manager.get_peer(peer_address)
    }

    /// Fetch a peer from peer list.
    ///
    /// Only returns peers which address is unicast, for multicast/broadcast
    /// addresses, the function will skip the entry and find the next in the
    /// peer list.
    pub fn fetch_peer(&self, from_head: bool) -> Result<PeerInfo, EspNowError> {
        self.manager.fetch_peer(from_head)
    }

    /// Check is peer is known.
    pub fn peer_exists(&self, peer_address: &[u8; 6]) -> bool {
        self.manager.peer_exists(peer_address)
    }

    /// Get the number of peers.
    pub fn peer_count(&self) -> Result<PeerCount, EspNowError> {
        self.manager.peer_count()
    }

    /// Set the primary master key.
    pub fn set_pmk(&self, pmk: &[u8; 16]) -> Result<(), EspNowError> {
        self.manager.set_pmk(pmk)
    }

    /// Set wake window for esp_now to wake up in interval unit.
    ///
    /// Window is milliseconds the chip keep waked each interval, from 0 to
    /// 65535.
    pub fn set_wake_window(&self, wake_window: u16) -> Result<(), EspNowError> {
        self.manager.set_wake_window(wake_window)
    }

    /// Configure ESP-NOW rate.
    pub fn set_rate(&self, rate: WifiPhyRate) -> Result<(), EspNowError> {
        self.manager.set_rate(rate)
    }

    /// Send data to peer.
    ///
    /// The peer needs to be added to the peer list first.
    pub fn send<'s>(
        &'s mut self,
        dst_addr: &[u8; 6],
        data: &[u8],
    ) -> Result<SendWaiter<'s>, EspNowError> {
        self.sender.send(dst_addr, data)
    }

    /// Receive data.
    pub fn receive(&self) -> Option<ReceivedData> {
        self.receiver.receive()
    }
}

unsafe extern "C" fn send_cb(_mac_addr: *const u8, status: esp_now_send_status_t) {
    critical_section::with(|_| {
        let is_success = status == esp_now_send_status_t_ESP_NOW_SEND_SUCCESS;
        ESP_NOW_SEND_STATUS.store(is_success, Ordering::Relaxed);

        ESP_NOW_SEND_CB_INVOKED.store(true, Ordering::Release);

        asynch::ESP_NOW_TX_WAKER.wake();
    })
}

unsafe extern "C" fn rcv_cb(
    esp_now_info: *const esp_now_recv_info_t,
    data: *const u8,
    data_len: i32,
) {
    let src = [
        (*esp_now_info).src_addr.offset(0).read(),
        (*esp_now_info).src_addr.offset(1).read(),
        (*esp_now_info).src_addr.offset(2).read(),
        (*esp_now_info).src_addr.offset(3).read(),
        (*esp_now_info).src_addr.offset(4).read(),
        (*esp_now_info).src_addr.offset(5).read(),
    ];

    let dst = [
        (*esp_now_info).des_addr.offset(0).read(),
        (*esp_now_info).des_addr.offset(1).read(),
        (*esp_now_info).des_addr.offset(2).read(),
        (*esp_now_info).des_addr.offset(3).read(),
        (*esp_now_info).des_addr.offset(4).read(),
        (*esp_now_info).des_addr.offset(5).read(),
    ];

    let rx_cntl = (*esp_now_info).rx_ctrl;
    let rx_control = RxControlInfo::from_raw(rx_cntl);

    let info = ReceiveInfo {
        src_address: src,
        dst_address: dst,
        rx_control,
    };
    let slice = core::slice::from_raw_parts(data, data_len as usize);
    critical_section::with(|cs| {
        let mut queue = RECEIVE_QUEUE.borrow_ref_mut(cs);
        let data = Box::from(slice);

        if queue.len() >= RECEIVE_QUEUE_SIZE {
            queue.pop_front();
        }

        queue.push_back(ReceivedData { data, info });

        asynch::ESP_NOW_RX_WAKER.wake();
    });
}

pub use asynch::SendFuture;

mod asynch {
    use core::task::{Context, Poll};

    use embassy_sync::waitqueue::AtomicWaker;

    use super::*;

    pub(super) static ESP_NOW_TX_WAKER: AtomicWaker = AtomicWaker::new();
    pub(super) static ESP_NOW_RX_WAKER: AtomicWaker = AtomicWaker::new();

    impl EspNowReceiver<'_> {
        /// This function takes mutable reference to self because the
        /// implementation of `ReceiveFuture` is not logically thread
        /// safe.
        pub fn receive_async(&mut self) -> ReceiveFuture<'_> {
            ReceiveFuture(PhantomData)
        }
    }

    impl EspNowSender<'_> {
        /// Sends data asynchronously to a peer (using its MAC) using ESP-NOW.
        pub fn send_async<'s, 'r>(
            &'s mut self,
            addr: &'r [u8; 6],
            data: &'r [u8],
        ) -> SendFuture<'s, 'r> {
            SendFuture {
                _sender: PhantomData,
                addr,
                data,
                sent: false,
            }
        }
    }

    impl EspNow<'_> {
        /// This function takes mutable reference to self because the
        /// implementation of `ReceiveFuture` is not logically thread
        /// safe.
        pub fn receive_async(&mut self) -> ReceiveFuture<'_> {
            self.receiver.receive_async()
        }

        /// The returned future must not be dropped before it's ready to avoid
        /// getting wrong status for sendings.
        pub fn send_async<'s, 'r>(
            &'s mut self,
            dst_addr: &'r [u8; 6],
            data: &'r [u8],
        ) -> SendFuture<'s, 'r> {
            self.sender.send_async(dst_addr, data)
        }
    }

    /// A `future` representing the result of an asynchronous ESP-NOW send
    /// operation.
    #[must_use = "futures do nothing unless you `.await` or poll them"]
    pub struct SendFuture<'s, 'r> {
        _sender: PhantomData<&'s mut EspNowSender<'s>>,
        addr: &'r [u8; 6],
        data: &'r [u8],
        sent: bool,
    }

    impl core::future::Future for SendFuture<'_, '_> {
        type Output = Result<(), EspNowError>;

        fn poll(mut self: core::pin::Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
            if !self.sent {
                ESP_NOW_TX_WAKER.register(cx.waker());
                ESP_NOW_SEND_CB_INVOKED.store(false, Ordering::Release);
                if let Err(e) = check_error!({
                    esp_now_send(self.addr.as_ptr(), self.data.as_ptr(), self.data.len())
                }) {
                    return Poll::Ready(Err(e));
                }
                self.sent = true;
            }

            if !ESP_NOW_SEND_CB_INVOKED.load(Ordering::Acquire) {
                Poll::Pending
            } else {
                Poll::Ready(if ESP_NOW_SEND_STATUS.load(Ordering::Relaxed) {
                    Ok(())
                } else {
                    Err(EspNowError::SendFailed)
                })
            }
        }
    }

    /// It's not logically safe to poll multiple instances of `ReceiveFuture`
    /// simultaneously since the callback can only wake one future, leaving
    /// the rest of them unwakable.
    #[must_use = "futures do nothing unless you `.await` or poll them"]
    pub struct ReceiveFuture<'r>(PhantomData<&'r mut EspNowReceiver<'r>>);

    impl core::future::Future for ReceiveFuture<'_> {
        type Output = ReceivedData;

        fn poll(self: core::pin::Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
            ESP_NOW_RX_WAKER.register(cx.waker());

            if let Some(data) = critical_section::with(|cs| {
                let mut queue = RECEIVE_QUEUE.borrow_ref_mut(cs);
                queue.pop_front()
            }) {
                Poll::Ready(data)
            } else {
                Poll::Pending
            }
        }
    }
}
