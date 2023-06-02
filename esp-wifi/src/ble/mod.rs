#[cfg(any(esp32, esp32c3, esp32s3))]
pub(crate) mod btdm;

#[cfg(any(esp32c2))]
pub(crate) mod npl;

use core::mem::MaybeUninit;

#[cfg(any(esp32, esp32c3, esp32s3))]
use self::btdm as ble;

#[cfg(any(esp32c2))]
use self::npl as ble;

pub(crate) use ble::ble_init;
pub(crate) use ble::read_hci;
pub(crate) use ble::read_next;
pub(crate) use ble::send_hci;

pub mod controller;

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
            } else if self.kind == HciOutType::Acl && self.index >= 5 {
                if self.index == (self.data[3] as usize) + ((self.data[4] as usize) << 8) + 5 {
                    self.ready = true;
                }
            }
        }
    }

    fn reset(&mut self) {
        self.index = 0;
        self.ready = false;
        self.kind = HciOutType::Unknown;
    }

    fn packet(&self) -> &[u8] {
        &self.data[0..(self.index as usize)]
    }
}
