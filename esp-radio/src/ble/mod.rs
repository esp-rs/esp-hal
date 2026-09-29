//! Bluetooth Low Energy HCI interface

mod lp_clk;

#[cfg_attr(bt_controller = "btdm", path = "btdm/mod.rs")]
#[cfg_attr(bt_controller = "npl", path = "npl/mod.rs")]
#[cfg_attr(bt_controller = "btdm2", path = "btdm2/mod.rs")]
pub(crate) mod porting;
use alloc::{boxed::Box, collections::vec_deque::VecDeque};

use docsplay::Display;
use esp_sync::NonReentrantMutex;
pub(crate) use porting::{ble_deinit, ble_init};

/// An error that is returned when the configuration is invalid.
#[derive(Display, Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct InvalidConfigError;

impl core::error::Error for InvalidConfigError {}

// Expose chip-specific configuration types
pub use porting::chip_specific::*;

pub(crate) static ESP_RADIO_LOCK: esp_sync::RawMutex = esp_sync::RawMutex::new();

/// Returns whether the caller runs in an interrupt handler.
///
/// The controller calls some of the OS glue from its interrupt handlers. Those calls must not
/// block, so they use the try-variant of the operation.
#[cfg(any(bt_controller = "npl", bt_controller = "btdm2"))]
pub(crate) fn in_isr() -> bool {
    !crate::hal::interrupt::RunLevel::current().is_thread()
}

static MODEM_SLEEP: portable_atomic::AtomicBool = portable_atomic::AtomicBool::new(false);
static MODEM_PHY_OFF: portable_atomic::AtomicBool = portable_atomic::AtomicBool::new(false);

pub(crate) fn set_modem_sleep(enabled: bool) {
    MODEM_SLEEP.store(enabled, portable_atomic::Ordering::Relaxed);
}

#[allow(dead_code, reason = "The ESP32 BTDM adapter reads this flag")]
pub(crate) fn modem_sleep_enabled() -> bool {
    MODEM_SLEEP.load(portable_atomic::Ordering::Relaxed)
}

/// Drops the extra PHY reference taken around controller sleep.
///
/// The controller's [`esp_phy::PhyInitGuard`] stays alive. This pairs with
/// [`modem_phy_acquire`].
pub(crate) fn modem_phy_release() {
    if !MODEM_PHY_OFF.swap(true, portable_atomic::Ordering::SeqCst) {
        esp_phy::disable_phy();
        unlock_cpu_frequency();
    }
}

/// Restores the PHY reference if sleep left it off.
///
/// Returns `true` if the reference was restored.
pub(crate) fn modem_phy_acquire() -> bool {
    let restore = MODEM_PHY_OFF.swap(false, portable_atomic::Ordering::SeqCst);
    if restore {
        lock_cpu_frequency();
        core::mem::forget(esp_phy::enable_phy());
    }
    restore
}

/// Keeps the CPU clock at its configured frequency while the controller uses the PHY.
pub(crate) fn lock_cpu_frequency() {
    #[cfg(bt_requires_fast_cpu)]
    esp_hal::if_unstable_hal! {
        esp_hal::clock::CpuFrequencyLock::acquire();
    }
}

/// Releases the lock taken by [`lock_cpu_frequency`].
pub(crate) fn unlock_cpu_frequency() {
    #[cfg(bt_requires_fast_cpu)]
    esp_hal::if_unstable_hal! {
        esp_hal::clock::CpuFrequencyLock::release();
    }
}

unstable_module! {
    pub mod controller;
}

// btdm2 registers its own `wr_btdm_osal_malloc` / `wr_btdm_osal_free` wrappers.
#[cfg(not(bt_controller = "btdm2"))]
pub(crate) unsafe extern "C" fn malloc(size: u32) -> *mut crate::sys::c_types::c_void {
    unsafe { crate::compat::malloc::malloc(size as usize).cast() }
}

#[cfg(not(bt_controller = "btdm2"))]
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
pub(crate) struct HciOutCollector {
    data: [u8; MAX_HCI_PACKET_LEN],
    index: usize,
    ready: bool,
    kind: HciOutType,
}

impl HciOutCollector {
    pub(crate) const fn new() -> HciOutCollector {
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

    pub(crate) fn write(&mut self, buf: &[u8]) -> usize {
        let taken = self.push(buf);

        if self.is_ready() {
            porting::send(self.packet());
            self.reset();
        }

        taken
    }

    pub(crate) async fn write_async(&mut self, buf: &[u8]) -> usize {
        let taken = self.push(buf);

        if self.is_ready() {
            porting::send_async(self.packet()).await;
            self.reset();
        }

        taken
    }
}

impl embedded_io_07::ErrorType for HciOutCollector {
    type Error = controller::BleConnectorError;
}

impl embedded_io_async_07::Write for HciOutCollector {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        Ok(self.write_async(buf).await)
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        // nothing to do
        Ok(())
    }
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
