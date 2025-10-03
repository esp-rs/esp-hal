//! Bluetooth Low Energy HCI interface
//!
//! The usage of BLE is currently incompatible with the usage of IEEE 802.15.4.

#[cfg(bt_controller = "btdm")]
pub(crate) mod btdm;

#[cfg(bt_controller = "npl")]
pub(crate) mod npl;

use alloc::{boxed::Box, collections::vec_deque::VecDeque, vec::Vec};
use core::mem::MaybeUninit;

pub use ble::ble_os_adapter_chip_specific::Config;
pub(crate) use ble::{ble_deinit, ble_init, send_hci};
use esp_sync::NonReentrantMutex;

#[cfg(bt_controller = "btdm")]
use self::btdm as ble;
#[cfg(bt_controller = "npl")]
use self::npl as ble;

unstable_module! {
    pub mod controller;
}

pub(crate) unsafe extern "C" fn malloc(size: u32) -> *mut crate::binary::c_types::c_void {
    unsafe { crate::compat::malloc::malloc(size as usize).cast() }
}

#[cfg(any(esp32, esp32c3, esp32s3))]
pub(crate) unsafe extern "C" fn malloc_internal(size: u32) -> *mut crate::binary::c_types::c_void {
    unsafe { crate::compat::malloc::malloc_internal(size as usize).cast() }
}

pub(crate) unsafe extern "C" fn free(ptr: *mut crate::binary::c_types::c_void) {
    unsafe { crate::compat::malloc::free(ptr.cast()) }
}

struct BleState {
    pub rx_queue: VecDeque<ReceivedPacket>,
    pub hci_read_data: Vec<u8>,
}

static BT_STATE: NonReentrantMutex<BleState> = NonReentrantMutex::new(BleState {
    rx_queue: VecDeque::new(),
    hci_read_data: Vec::new(),
});

static mut HCI_OUT_COLLECTOR: MaybeUninit<HciOutCollector> = MaybeUninit::uninit();

#[derive(PartialEq, Debug)]
enum HciOutType {
    Unknown,
    Acl,
    Command,
}

struct HciOutCollector {
    data: [u8; 256],
    index: usize,
    ready: bool,
    kind: HciOutType,
}

impl HciOutCollector {
    fn new() -> HciOutCollector {
        HciOutCollector {
            data: [0u8; 256],
            index: 0,
            ready: false,
            kind: HciOutType::Unknown,
        }
    }

    fn is_ready(&self) -> bool {
        self.ready
    }

    fn push(&mut self, data: &[u8]) {
        self.data[self.index..(self.index + data.len())].copy_from_slice(data);
        self.index += data.len();

        if self.kind == HciOutType::Unknown {
            self.kind = match self.data[0] {
                1 => HciOutType::Command,
                2 => HciOutType::Acl,
                _ => HciOutType::Unknown,
            };
        }

        if !self.ready {
            if self.kind == HciOutType::Command && self.index >= 4 {
                if self.index == self.data[3] as usize + 4 {
                    self.ready = true;
                }
            } else if self.kind == HciOutType::Acl
                && self.index >= 5
                && self.index == (self.data[3] as usize) + ((self.data[4] as usize) << 8) + 5
            {
                self.ready = true;
            }
        }
    }

    fn reset(&mut self) {
        self.index = 0;
        self.ready = false;
        self.kind = HciOutType::Unknown;
    }

    fn packet(&self) -> &[u8] {
        &self.data[0..self.index]
    }
}

#[derive(Debug, Clone)]
/// Represents a received BLE packet.
#[instability::unstable]
pub struct ReceivedPacket {
    /// The data of the received packet.
    pub data: Box<[u8]>,
}

#[cfg(feature = "defmt")]
impl defmt::Format for ReceivedPacket {
    fn format(&self, fmt: defmt::Formatter<'_>) {
        defmt::write!(fmt, "ReceivedPacket {}", &self.data[..])
    }
}

/// Checks if there is any HCI data available to read.
#[instability::unstable]
pub fn have_hci_read_data() -> bool {
    BT_STATE.with(|state| !state.rx_queue.is_empty() || !state.hci_read_data.is_empty())
}

pub(crate) fn read_next(data: &mut [u8]) -> usize {
    if let Some(packet) = BT_STATE.with(|state| state.rx_queue.pop_front()) {
        data[..packet.data.len()].copy_from_slice(&packet.data[..packet.data.len()]);
        packet.data.len()
    } else {
        0
    }
}

/// Reads the next HCI packet from the BLE controller.
#[instability::unstable]
pub fn read_hci(data: &mut [u8]) -> usize {
    BT_STATE.with(|state| {
        if state.hci_read_data.is_empty()
            && let Some(packet) = state.rx_queue.pop_front()
        {
            state.hci_read_data.extend_from_slice(&packet.data);
        }

        let l = usize::min(state.hci_read_data.len(), data.len());
        data[..l].copy_from_slice(&state.hci_read_data[..l]);
        state.hci_read_data.drain(..l);
        l
    })
}

fn dump_packet_info(_buffer: &[u8]) {
    #[cfg(dump_packets)]
    info!("@HCIFRAME {:?}", _buffer);
}
