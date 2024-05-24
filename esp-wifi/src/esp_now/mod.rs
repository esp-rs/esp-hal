//! ESP-NOW is a kind of connectionless Wi-Fi communication protocol that is defined by Espressif.
//!
//! In ESP-NOW, application data is encapsulated in a vendor-specific action frame and then transmitted from
//! one Wi-Fi device to another without connection. CTR with CBC-MAC Protocol(CCMP) is used to protect the action
//! frame for security. ESP-NOW is widely used in smart light, remote controlling, sensor, etc.
//!
//! For more information see https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_now.html

use core::{cell::RefCell, fmt::Debug};

use critical_section::Mutex;
use esp_hal_common::peripheral::{Peripheral, PeripheralRef};

use crate::compat::queue::SimpleQueue;
use crate::EspWifiInitialization;

use crate::binary::include::*;

/// Maximum payload length
pub const ESP_NOW_MAX_DATA_LEN: usize = 250;

/// Broadcast address
pub const BROADCAST_ADDRESS: [u8; 6] = [0xffu8, 0xffu8, 0xffu8, 0xffu8, 0xffu8, 0xffu8];

static RECEIVE_QUEUE: Mutex<RefCell<SimpleQueue<ReceivedData, 10>>> =
    Mutex::new(RefCell::new(SimpleQueue::new()));

macro_rules! check_error {
    ($block:block) => {
        let res = unsafe { $block };
        if res != 0 {
            return Err(EspNowError::Error(Error::from_code(res as u32)));
        }
    };
}

#[repr(u32)]
#[derive(Debug)]
pub enum Error {
    NotInitialized = 12389,
    InvalidArgument = 12390,
    OutOfMemory = 12391,
    PeerListFull = 12392,
    UnknownPeer = 12393,
    NotFound = 12394,
    PeerExists = 12395,
    InterfaceError = 12396,
    Other(u32),
}

impl Error {
    pub fn from_code(code: u32) -> Error {
        match code {
            12389 => Error::NotInitialized,
            12390 => Error::InvalidArgument,
            12391 => Error::OutOfMemory,
            12392 => Error::PeerListFull,
            12393 => Error::UnknownPeer,
            12394 => Error::NotFound,
            12395 => Error::PeerExists,
            12396 => Error::InterfaceError,
            _ => Error::Other(code),
        }
    }
}

#[derive(Debug)]
pub enum EspNowError {
    Error(Error),
}

#[derive(Debug)]
pub struct PeerCount {
    pub total_count: i32,
    pub encrypted_count: i32,
}

#[repr(u32)]
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

#[derive(Debug, Clone, Copy)]
pub struct PeerInfo {
    pub peer_address: [u8; 6],
    pub lmk: Option<[u8; 16]>,
    pub channel: Option<u8>,
    pub encrypt: bool,
    // we always use STA for now
}

#[cfg(not(any(esp32c6)))]
#[derive(Debug, Clone, Copy)]
pub struct RxControlInfo {
    pub rssi: i32,
    pub rate: u32,
    pub sig_mode: u32,
    pub mcs: u32,
    pub cwb: u32,
    pub smoothing: u32,
    pub not_sounding: u32,
    pub aggregation: u32,
    pub stbc: u32,
    pub fec_coding: u32,
    pub sgi: u32,
    pub ampdu_cnt: u32,
    pub channel: u32,
    pub secondary_channel: u32,
    pub timestamp: u32,
    pub noise_floor: i32,
    pub ant: u32,
    pub sig_len: u32,
    pub rx_state: u32,
}

#[cfg(any(esp32c6))]
#[derive(Debug, Clone, Copy)]
pub struct RxControlInfo {
    pub rssi: i32,
    pub rate: u32,
    pub sig_len: u32,
    pub rx_state: u32,
    pub dump_len: u32,
    pub he_sigb_len: u32,
    pub cur_single_mpdu: u32,
    pub cur_bb_format: u32,
    pub rx_channel_estimate_info_vld: u32,
    pub rx_channel_estimate_len: u32,
    pub second: u32,
    pub channel: u32,
    pub data_rssi: i32,
    pub noise_floor: u32,
    pub is_group: u32,
    pub rxend_state: u32,
    pub rxmatch3: u32,
    pub rxmatch2: u32,
    pub rxmatch1: u32,
    pub rxmatch0: u32,
}

#[derive(Debug, Clone, Copy)]
pub struct ReceiveInfo {
    pub src_address: [u8; 6],
    pub dst_address: [u8; 6],
    pub rx_control: RxControlInfo,
}

#[derive(Clone, Copy)]
pub struct ReceivedData {
    pub len: u8,
    pub data: [u8; 256],
    pub info: ReceiveInfo,
}

impl ReceivedData {
    pub fn get_data(&self) -> &[u8] {
        &self.data[..self.len as usize]
    }
}

impl Debug for ReceivedData {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("ReceivedData")
            .field("data", &self.get_data())
            .field("info", &self.info)
            .finish()
    }
}

pub struct EspNowWithWifiCreateToken {
    _private: (),
}

pub fn enable_esp_now_with_wifi(
    device: esp_hal_common::radio::Wifi,
) -> (esp_hal_common::radio::Wifi, EspNowWithWifiCreateToken) {
    (device, EspNowWithWifiCreateToken { _private: () })
}

/// ESP-NOW is a kind of connectionless Wi-Fi communication protocol that is defined by Espressif.
/// In ESP-NOW, application data is encapsulated in a vendor-specific action frame and then transmitted from
/// one Wi-Fi device to another without connection. CTR with CBC-MAC Protocol(CCMP) is used to protect the
/// action frame for security. ESP-NOW is widely used in smart light, remote controlling, sensor, etc.
///
/// Currently this implementation (when used together with traditional Wi-Fi) ONLY support STA mode.
///
pub struct EspNow<'d> {
    _device: Option<PeripheralRef<'d, esp_hal_common::radio::Wifi>>,
}

impl<'d> EspNow<'d> {
    pub fn new(
        inited: &EspWifiInitialization,
        device: impl Peripheral<P = esp_hal_common::radio::Wifi> + 'd,
    ) -> Result<EspNow<'d>, EspNowError> {
        EspNow::new_internal(inited, Some(device.into_ref()))
    }

    pub fn new_with_wifi(
        inited: &EspWifiInitialization,
        _token: EspNowWithWifiCreateToken,
    ) -> Result<EspNow<'d>, EspNowError> {
        EspNow::new_internal(
            inited,
            None::<PeripheralRef<'d, esp_hal_common::radio::Wifi>>,
        )
    }

    fn new_internal(
        inited: &EspWifiInitialization,
        device: Option<PeripheralRef<'d, esp_hal_common::radio::Wifi>>,
    ) -> Result<EspNow<'d>, EspNowError> {
        if !inited.is_wifi() {
            panic!("Not initialized for Wifi use");
        }

        let mut esp_now = EspNow { _device: device };
        check_error!({ esp_wifi_set_mode(wifi_mode_t_WIFI_MODE_STA) });
        check_error!({ esp_wifi_start() });
        check_error!({ esp_now_init() });
        check_error!({ esp_now_register_recv_cb(Some(rcv_cb)) });

        esp_now.add_peer(PeerInfo {
            peer_address: BROADCAST_ADDRESS,
            lmk: None,
            channel: None,
            encrypt: false,
        })?;

        Ok(esp_now)
    }

    /// Set primary WiFi channel
    /// Should only be used when using ESP-NOW without AP or STA
    pub fn set_channel(&mut self, channel: u8) -> Result<(), EspNowError> {
        check_error!({ esp_wifi_set_channel(channel, 0) });
        Ok(())
    }

    /// Get the version of ESPNOW
    pub fn get_version(&self) -> Result<u32, EspNowError> {
        let mut version = 0u32;
        check_error!({ esp_now_get_version(&mut version as *mut u32) });
        Ok(version)
    }

    /// Add a peer to the list of known peers
    pub fn add_peer(&mut self, peer: PeerInfo) -> Result<(), EspNowError> {
        let raw_peer = esp_now_peer_info_t {
            peer_addr: peer.peer_address,
            lmk: peer.lmk.unwrap_or_else(|| [0u8; 16]),
            channel: peer.channel.unwrap_or_else(|| 0),
            ifidx: wifi_interface_t_WIFI_IF_STA,
            encrypt: peer.encrypt,
            priv_: core::ptr::null_mut(),
        };
        check_error!({ esp_now_add_peer(&raw_peer as *const _) });

        Ok(())
    }

    /// Remove the given peer
    pub fn remove_peer(&mut self, peer_address: &[u8; 6]) -> Result<(), EspNowError> {
        check_error!({ esp_now_del_peer(peer_address.as_ptr()) });
        Ok(())
    }

    /// Modify a peer information
    pub fn modify_peer(&mut self, peer: PeerInfo) -> Result<(), EspNowError> {
        let raw_peer = esp_now_peer_info_t {
            peer_addr: peer.peer_address,
            lmk: peer.lmk.unwrap_or_else(|| [0u8; 16]),
            channel: peer.channel.unwrap_or_else(|| 0),
            ifidx: wifi_interface_t_WIFI_IF_STA,
            encrypt: peer.encrypt,
            priv_: core::ptr::null_mut(),
        };
        check_error!({ esp_now_mod_peer(&raw_peer as *const _) });

        Ok(())
    }

    /// Get peer by MAC address
    pub fn get_peer(&mut self, peer_address: &[u8; 6]) -> Result<PeerInfo, EspNowError> {
        let mut raw_peer = esp_now_peer_info_t {
            peer_addr: [0u8; 6],
            lmk: [0u8; 16],
            channel: 0,
            ifidx: 0,
            encrypt: false,
            priv_: core::ptr::null_mut(),
        };
        check_error!({ esp_now_get_peer(peer_address.as_ptr(), &mut raw_peer as *mut _) });

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

    /// Fetch a peer from peer list
    ///
    /// Only returns peers which address is unicast, for multicast/broadcast addresses, the function will
    /// skip the entry and find the next in the peer list.
    pub fn fetch_peer(&mut self, from_head: bool) -> Result<PeerInfo, EspNowError> {
        let mut raw_peer = esp_now_peer_info_t {
            peer_addr: [0u8; 6],
            lmk: [0u8; 16],
            channel: 0,
            ifidx: 0,
            encrypt: false,
            priv_: core::ptr::null_mut(),
        };
        check_error!({ esp_now_fetch_peer(from_head, &mut raw_peer as *mut _) });

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

    /// Check is peer is known
    pub fn peer_exists(&mut self, peer_address: &[u8; 6]) -> bool {
        unsafe { esp_now_is_peer_exist(peer_address.as_ptr()) }
    }

    /// Get the number of peers
    pub fn peer_count(&mut self) -> Result<PeerCount, EspNowError> {
        let mut peer_num = esp_now_peer_num_t {
            total_num: 0,
            encrypt_num: 0,
        };
        check_error!({ esp_now_get_peer_num(&mut peer_num as *mut _) });

        Ok(PeerCount {
            total_count: peer_num.total_num,
            encrypted_count: peer_num.encrypt_num,
        })
    }

    /// Set the primary master key
    pub fn set_pmk(&mut self, pmk: &[u8; 16]) -> Result<(), EspNowError> {
        check_error!({ esp_now_set_pmk(pmk.as_ptr()) });

        Ok(())
    }

    /// Set wake window for esp_now to wake up in interval unit
    ///
    /// Window is milliseconds the chip keep waked each interval, from 0 to 65535.
    pub fn set_wake_window(&mut self, wake_window: u16) -> Result<(), EspNowError> {
        check_error!({ esp_now_set_wake_window(wake_window) });

        Ok(())
    }

    /// Config ESPNOW rate
    pub fn set_rate(&mut self, rate: WifiPhyRate) -> Result<(), EspNowError> {
        check_error!({ esp_wifi_config_espnow_rate(wifi_interface_t_WIFI_IF_STA, rate as u32,) });

        Ok(())
    }

    /// Send data to peer
    ///
    /// The peer needs to be added to the peer list first
    pub fn send(&mut self, dst_addr: &[u8; 6], data: &[u8]) -> Result<(), EspNowError> {
        let mut addr = [0u8; 6];
        addr.copy_from_slice(dst_addr);
        check_error!({ esp_now_send(addr.as_ptr(), data.as_ptr(), data.len() as u32) });

        Ok(())
    }

    /// Receive data
    pub fn receive(&self) -> Option<ReceivedData> {
        critical_section::with(|cs| {
            let mut queue = RECEIVE_QUEUE.borrow_ref_mut(cs);
            queue.dequeue()
        })
    }
}

impl Drop for EspNow<'_> {
    fn drop(&mut self) {
        unsafe {
            esp_now_unregister_recv_cb();
            esp_now_deinit();
        }
    }
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
    #[cfg(not(any(esp32c6)))]
    let rx_control = RxControlInfo {
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

    #[cfg(any(esp32c6))]
    let rx_control = RxControlInfo {
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
        data_rssi: (*rx_cntl).data_rssi(),
        noise_floor: (*rx_cntl).noise_floor(),
        is_group: (*rx_cntl).is_group(),
        rxend_state: (*rx_cntl).rxend_state(),
        rxmatch3: (*rx_cntl).rxmatch3(),
        rxmatch2: (*rx_cntl).rxmatch2(),
        rxmatch1: (*rx_cntl).rxmatch1(),
        rxmatch0: (*rx_cntl).rxmatch0(),
    };

    let info = ReceiveInfo {
        src_address: src,
        dst_address: dst,
        rx_control,
    };
    let slice = core::slice::from_raw_parts(data, data_len as usize);
    critical_section::with(|cs| {
        let mut queue = RECEIVE_QUEUE.borrow_ref_mut(cs);
        let mut data = [0u8; 256];
        data[..slice.len()].copy_from_slice(slice);

        if queue.is_full() {
            queue.dequeue();
        }

        queue
            .enqueue(ReceivedData {
                len: slice.len() as u8,
                data: data,
                info,
            })
            .unwrap();

        #[cfg(feature = "async")]
        asynch::ESP_NOW_WAKER.wake();
    });
}

#[cfg(feature = "async")]
mod asynch {
    use super::*;
    use core::task::{Context, Poll};
    use embassy_sync::waitqueue::AtomicWaker;

    pub(super) static ESP_NOW_WAKER: AtomicWaker = AtomicWaker::new();

    impl<'d> EspNow<'d> {
        pub async fn receive_async(&mut self) -> ReceivedData {
            ReceiveFuture.await
        }
    }

    struct ReceiveFuture;

    impl core::future::Future for ReceiveFuture {
        type Output = ReceivedData;

        fn poll(self: core::pin::Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
            ESP_NOW_WAKER.register(cx.waker());

            if let Some(data) = critical_section::with(|cs| {
                let mut queue = RECEIVE_QUEUE.borrow_ref_mut(cs);
                queue.dequeue()
            }) {
                Poll::Ready(data)
            } else {
                Poll::Pending
            }
        }
    }
}
