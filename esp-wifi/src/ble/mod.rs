//! Bluetooth Low Energy HCI interface

#[cfg(any(esp32, esp32c3, esp32s3))]
pub(crate) mod btdm;

#[cfg(any(esp32c2, esp32c6, esp32h2))]
pub(crate) mod npl;

use alloc::vec::Vec;
use core::{cell::RefCell, mem::MaybeUninit};

pub(crate) use ble::{ble_init, send_hci};
use critical_section::Mutex;

#[cfg(any(esp32, esp32c3, esp32s3))]
use self::btdm as ble;
#[cfg(any(esp32c2, esp32c6, esp32h2))]
use self::npl as ble;

pub mod controller;

pub(crate) unsafe extern "C" fn malloc(size: u32) -> *mut crate::binary::c_types::c_void {
    crate::compat::malloc::malloc(size as usize).cast()
}

#[cfg(any(esp32, esp32c3, esp32s3))]
pub(crate) unsafe extern "C" fn malloc_internal(size: u32) -> *mut crate::binary::c_types::c_void {
    crate::compat::malloc::malloc(size as usize).cast()
}

pub(crate) unsafe extern "C" fn free(ptr: *mut crate::binary::c_types::c_void) {
    crate::compat::malloc::free(ptr.cast())
}

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

static BLE_HCI_READ_DATA: Mutex<RefCell<Vec<u8>>> = Mutex::new(RefCell::new(Vec::new()));

#[derive(Debug, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ReceivedPacket {
    pub data: Vec<u8>,
}

#[cfg(feature = "async")]
pub fn have_hci_read_data() -> bool {
    critical_section::with(|cs| {
        let queue = ble::BT_RECEIVE_QUEUE.borrow_ref_mut(cs);
        !queue.is_empty() || BLE_HCI_READ_DATA.borrow_ref(cs).len() > 0
    })
}

pub(crate) fn read_next(data: &mut [u8]) -> usize {
    critical_section::with(|cs| {
        let mut queue = ble::BT_RECEIVE_QUEUE.borrow_ref_mut(cs);

        match queue.pop() {
            Some(packet) => {
                data[..packet.data.len() as usize]
                    .copy_from_slice(&packet.data[..packet.data.len() as usize]);
                packet.data.len() as usize
            }
            None => 0,
        }
    })
}

pub fn read_hci(data: &mut [u8]) -> usize {
    critical_section::with(|cs| {
        let mut hci_read_data = BLE_HCI_READ_DATA.borrow_ref_mut(cs);

        if hci_read_data.len() == 0 {
            let mut queue = ble::BT_RECEIVE_QUEUE.borrow_ref_mut(cs);

            if let Some(packet) = queue.pop() {
                hci_read_data.extend(packet.data);
            }
        }

        let l = usize::min(hci_read_data.len(), data.len());
        if l > 0 {
            data[..l].copy_from_slice(&hci_read_data[..l]);
            hci_read_data.drain(..l);
            l
        } else {
            0
        }
    })
}

#[allow(unreachable_code, unused_variables)]
fn dump_packet_info(buffer: &[u8]) {
    #[cfg(not(feature = "dump-packets"))]
    return;

    critical_section::with(|cs| {
        info!("@HCIFRAME {:?}", buffer);
    });
}
