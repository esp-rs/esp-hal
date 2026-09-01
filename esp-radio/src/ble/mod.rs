//! Bluetooth Low Energy HCI interface

#[cfg(bt_controller = "btdm")]
pub(crate) mod btdm;

#[cfg(bt_controller = "npl")]
pub(crate) mod npl;

use alloc::{boxed::Box, collections::vec_deque::VecDeque};
use core::mem::MaybeUninit;

pub(crate) use ble::{ble_deinit, ble_init, send_hci};
use docsplay::Display;
use esp_sync::NonReentrantMutex;

/// An error that is returned when the configuration is invalid.
#[derive(Display, Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct InvalidConfigError;

impl core::error::Error for InvalidConfigError {}

// Expose chip-specific configuration types
pub use ble::ble_os_adapter_chip_specific::*;

#[cfg(bt_controller = "btdm")]
use self::btdm as ble;
#[cfg(bt_controller = "npl")]
use self::npl as ble;

unstable_module! {
    pub mod controller;
}

pub(crate) unsafe extern "C" fn malloc(size: u32) -> *mut crate::sys::c_types::c_void {
    unsafe { crate::compat::malloc::malloc(size as usize).cast() }
}

#[cfg(any(esp32, esp32c3, esp32s3))]
pub(crate) unsafe extern "C" fn malloc_internal(size: u32) -> *mut crate::sys::c_types::c_void {
    unsafe { crate::compat::malloc::malloc_internal(size as usize).cast() }
}

pub(crate) unsafe extern "C" fn free(ptr: *mut crate::sys::c_types::c_void) {
    unsafe { crate::compat::malloc::free(ptr.cast()) }
}

struct BleState {
    pub rx_queue: VecDeque<ReceivedPacket>,
    /// The packet that the byte-stream reader is part-way through, and the number of bytes the
    /// host already took from it.
    pub partial_read: Option<(Box<[u8]>, usize)>,
}

static BT_STATE: NonReentrantMutex<BleState> = NonReentrantMutex::new(BleState {
    rx_queue: VecDeque::new(),
    partial_read: None,
});

static mut HCI_OUT_COLLECTOR: MaybeUninit<HciOutCollector> = MaybeUninit::uninit();

#[derive(PartialEq, Debug)]
enum HciOutType {
    Unknown,
    Acl,
    Command,
}

/// The largest HCI packet, including the packet type indicator byte.
const MAX_HCI_PACKET_LEN: usize = 259;

/// Reassembles whole HCI packets out of the byte stream that the host writes.
///
/// The byte-stream write APIs put no constraint on where the caller splits a packet, and one
/// write can hold several packets. The collector takes only the bytes that the packet in progress
/// still needs, so that it never runs past a packet boundary.
struct HciOutCollector {
    data: [u8; MAX_HCI_PACKET_LEN],
    index: usize,
    ready: bool,
    kind: HciOutType,
}

impl HciOutCollector {
    fn new() -> HciOutCollector {
        HciOutCollector {
            data: [0u8; MAX_HCI_PACKET_LEN],
            index: 0,
            ready: false,
            kind: HciOutType::Unknown,
        }
    }

    fn is_ready(&self) -> bool {
        self.ready
    }

    /// The length of the header, including the packet type indicator byte.
    ///
    /// The kind is unknown until the indicator byte arrives, so ask for that byte on its own
    /// first.
    fn header_len(&self) -> usize {
        match self.kind {
            HciOutType::Unknown => 1,
            HciOutType::Command => 4,
            HciOutType::Acl => 5,
        }
    }

    /// The length of the packet in progress, or `None` while its header is incomplete.
    fn packet_len(&self) -> Option<usize> {
        if self.index < self.header_len() {
            return None;
        }

        match self.kind {
            HciOutType::Unknown => None,
            HciOutType::Command => Some(self.data[3] as usize + 4),
            HciOutType::Acl => Some(u16::from_le_bytes([self.data[3], self.data[4]]) as usize + 5),
        }
    }

    /// Copies bytes from `data` until the packet buffer holds `upto` bytes.
    ///
    /// Returns the number of bytes copied.
    fn fill_to(&mut self, data: &[u8], upto: usize) -> usize {
        let take = usize::min(data.len(), upto - self.index);
        self.data[self.index..][..take].copy_from_slice(&data[..take]);
        self.index += take;
        take
    }

    /// Copies as much of `data` as the packet in progress needs, and returns how much it took.
    ///
    /// Bytes that belong to the next packet stay in `data`. The caller must send and reset the
    /// collector once [`Self::is_ready`] holds, before it offers those bytes again.
    fn push(&mut self, data: &[u8]) -> usize {
        if data.is_empty() {
            return 0;
        }

        if self.index == 0 {
            self.kind = match data[0] {
                1 => HciOutType::Command,
                2 => HciOutType::Acl,
                indicator => {
                    warn!(
                        "Dropping HCI byte with unknown packet type indicator {}",
                        indicator
                    );
                    return 1;
                }
            };
        }

        // The packet length lives in the header, so complete the header before asking for the
        // rest of the packet.
        let mut taken = 0;
        if self.packet_len().is_none() {
            taken += self.fill_to(data, self.header_len());
        }

        if let Some(total) = self.packet_len() {
            if total > self.data.len() {
                warn!("Dropping HCI packet of {} bytes, which is too long", total);
                self.reset();
                return taken;
            }

            taken += self.fill_to(&data[taken..], total);
            self.ready = self.index == total;
        }

        taken
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

/// Collects bytes of the host's stream, and passes the packet to `send` once it is complete.
///
/// This behaves like a byte-stream write: it handles at most one packet, and returns the number
/// of bytes it took from `data`. Bytes that belong to the next packet stay in `data`, so the
/// caller offers the rest in a later call. A non-empty `data` always yields a non-zero count.
pub(crate) fn collect_and_send(data: &[u8], send: impl FnOnce(&[u8])) -> usize {
    let hci_out = unsafe { (*core::ptr::addr_of_mut!(HCI_OUT_COLLECTOR)).assume_init_mut() };

    let taken = hci_out.push(data);

    if hci_out.is_ready() {
        send(hci_out.packet());
        hci_out.reset();
    }

    taken
}

#[derive(Debug, Clone, PartialEq, Eq, Hash)]
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

/// Drops packets the host never read, so they don't outlive the controller.
pub(crate) fn clear_bt_state() {
    BT_STATE.with(|state| {
        state.rx_queue.clear();
        state.partial_read = None;
    });
}

/// Checks if there is any HCI data available to read.
#[instability::unstable]
pub fn have_hci_read_data() -> bool {
    BT_STATE.with(|state| !state.rx_queue.is_empty() || state.partial_read.is_some())
}

/// Checks if the receive queue holds a complete packet.
pub(crate) fn have_hci_packet() -> bool {
    BT_STATE.with(|state| !state.rx_queue.is_empty())
}

/// Removes the next packet from the receive queue, without copying it.
pub(crate) fn take_next() -> Option<Box<[u8]>> {
    BT_STATE.with(|state| state.rx_queue.pop_front().map(|packet| packet.data))
}

pub(crate) fn read_next(data: &mut [u8]) -> usize {
    if let Some(packet) = take_next() {
        data[..packet.len()].copy_from_slice(&packet);
        packet.len()
    } else {
        0
    }
}

/// Reads the next HCI packet from the BLE controller.
#[instability::unstable]
pub fn read_hci(data: &mut [u8]) -> usize {
    BT_STATE.with(|state| {
        if state.partial_read.is_none()
            && let Some(packet) = state.rx_queue.pop_front()
        {
            state.partial_read = Some((packet.data, 0));
        }

        let Some((packet, read)) = state.partial_read.as_mut() else {
            return 0;
        };

        let remaining = &packet[*read..];
        let l = usize::min(remaining.len(), data.len());
        data[..l].copy_from_slice(&remaining[..l]);
        *read += l;

        let drained = *read == packet.len();
        if drained {
            state.partial_read = None;
        }

        l
    })
}

fn dump_packet_info(_buffer: &[u8]) {
    #[cfg(dump_packets)]
    info!("@HCIFRAME {:?}", _buffer);
}

macro_rules! validate_range {
    ($this:ident, $field:ident, $min:expr, $max:expr) => {
        if !($min..=$max).contains(&$this.$field) {
            error!(
                "{} must be between {} and {}, current value is {}",
                stringify!($field),
                $min,
                $max,
                $this.$field
            );
            return Err(InvalidConfigError);
        }
    };
}
pub(crate) use validate_range;
