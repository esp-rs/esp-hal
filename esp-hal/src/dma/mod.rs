//! # Direct Memory Access
//!
//! ## Overview
//!
//! The `DMA` driver provides an interface to efficiently transfer data between
//! different memory regions and peripherals within the ESP microcontroller
//! without involving the CPU. The `Direct Memory Access` (DMA) controller is a
//! hardware block responsible for managing these data transfers.
//!
//! Notice, that this module is a common version of the DMA driver, `ESP32` and
//! `ESP32-S2` are using older `PDMA` controller, whenever other chips are using
//! newer `GDMA` controller.
//!
//! ## Example
//!
//! ### Initialize and utilize DMA controller in `SPI`
//!
//! ```no_run
//! let dma = Dma::new(peripherals.DMA);
//! let dma_channel = dma.channel0;
//!
//! let mut descriptors = [DmaDescriptor::EMPTY; 8];
//! let mut rx_descriptors = [DmaDescriptor::EMPTY; 8];
//!
//! let mut spi = Spi::new(
//!     peripherals.SPI2,
//!     sclk,
//!     mosi,
//!     miso,
//!     cs,
//!     100.kHz(),
//!     SpiMode::Mode0,
//!     &clocks,
//! )
//! .with_dma(dma_channel.configure(
//!     false,
//!     &mut descriptors,
//!     &mut rx_descriptors,
//!     DmaPriority::Priority0,
//! ));
//! ```
//!
//! ⚠️ Note: Descriptors should be sized as `(CHUNK_SIZE + 4091) / 4092`.
//! I.e., to transfer buffers of size `1..=4092`, you need 1 descriptor.
//!
//! For convenience you can use the [crate::dma_buffers] macro.
#![warn(missing_docs)]

use core::{marker::PhantomData, ptr::addr_of_mut, sync::atomic::compiler_fence};

bitfield::bitfield! {
    #[doc(hidden)]
    #[derive(Clone, Copy)]
    pub struct DmaDescriptorFlags(u32);

    u16;
    size, set_size: 11, 0;
    length, set_length: 23, 12;
    suc_eof, set_suc_eof: 30;
    owner, set_owner: 31;
}

/// A DMA transfer descriptor.
#[derive(Clone, Copy)]
pub struct DmaDescriptor {
    pub(crate) flags: DmaDescriptorFlags,
    pub(crate) buffer: *mut u8,
    pub(crate) next: *mut DmaDescriptor,
}

impl DmaDescriptor {
    /// An empty DMA descriptor used to initialize the descriptor list.
    pub const EMPTY: Self = Self {
        flags: DmaDescriptorFlags(0),
        buffer: core::ptr::null_mut(),
        next: core::ptr::null_mut(),
    };

    fn set_size(&mut self, len: usize) {
        self.flags.set_size(len as u16)
    }

    fn set_length(&mut self, len: usize) {
        self.flags.set_length(len as u16)
    }

    fn len(&self) -> usize {
        self.flags.length() as usize
    }

    fn is_empty(&self) -> bool {
        self.len() == 0
    }

    fn set_suc_eof(&mut self, suc_eof: bool) {
        self.flags.set_suc_eof(suc_eof)
    }

    fn set_owner(&mut self, owner: Owner) {
        let owner = match owner {
            Owner::Cpu => false,
            Owner::Dma => true,
        };
        self.flags.set_owner(owner)
    }

    fn owner(&self) -> Owner {
        match self.flags.owner() {
            false => Owner::Cpu,
            true => Owner::Dma,
        }
    }
}

use enumset::{EnumSet, EnumSetType};

#[cfg(gdma)]
pub use self::gdma::*;
#[cfg(pdma)]
pub use self::pdma::*;
use crate::{interrupt::InterruptHandler, Mode};

#[cfg(gdma)]
mod gdma;
#[cfg(pdma)]
mod pdma;

/// Kinds of interrupt to listen to
#[derive(EnumSetType)]
pub enum DmaInterrupt {
    /// TX is done
    TxDone,
    /// RX is done
    RxDone,
}

const CHUNK_SIZE: usize = 4092;

/// Convenience macro to create DMA buffers and descriptors
///
/// ## Usage
/// ```rust,no_run
/// // TX and RX buffers are 32000 bytes - passing only one parameter makes TX and RX the same size
/// let (tx_buffer, mut tx_descriptors, rx_buffer, mut rx_descriptors) = dma_buffers!(32000, 32000);
/// ```
#[macro_export]
macro_rules! dma_buffers {
    ($tx_size:expr, $rx_size:expr) => {{
        static mut TX_BUFFER: [u8; $tx_size] = [0u8; $tx_size];
        static mut RX_BUFFER: [u8; $rx_size] = [0u8; $rx_size];
        let tx_descriptors = [$crate::dma::DmaDescriptor::EMPTY; ($tx_size + 4091) / 4092];
        let rx_descriptors = [$crate::dma::DmaDescriptor::EMPTY; ($rx_size + 4091) / 4092];
        unsafe {
            (
                &mut TX_BUFFER,
                tx_descriptors,
                &mut RX_BUFFER,
                rx_descriptors,
            )
        }
    }};

    ($size:expr) => {{
        static mut TX_BUFFER: [u8; $size] = [0u8; $size];
        static mut RX_BUFFER: [u8; $size] = [0u8; $size];
        let tx_descriptors = [$crate::dma::DmaDescriptor::EMPTY; ($size + 4091) / 4092];
        let rx_descriptors = [$crate::dma::DmaDescriptor::EMPTY; ($size + 4091) / 4092];
        unsafe {
            (
                &mut TX_BUFFER,
                tx_descriptors,
                &mut RX_BUFFER,
                rx_descriptors,
            )
        }
    }};
}

/// Convenience macro to create circular DMA buffers and descriptors
///
/// ## Usage
/// ```rust,no_run
/// // TX and RX buffers are 32000 bytes - passing only one parameter makes TX and RX the same size
/// let (tx_buffer, mut tx_descriptors, rx_buffer, mut rx_descriptors) =
///     dma_circular_buffers!(32000, 32000);
/// ```
#[macro_export]
macro_rules! dma_circular_buffers {
    ($tx_size:expr, $rx_size:expr) => {{
        static mut TX_BUFFER: [u8; $tx_size] = [0u8; $tx_size];
        static mut RX_BUFFER: [u8; $rx_size] = [0u8; $rx_size];

        const tx_descriptor_len: usize = if $tx_size > 4092 * 2 {
            ($tx_size + 4091) / 4092
        } else {
            3
        };

        const rx_descriptor_len: usize = if $rx_size > 4092 * 2 {
            ($rx_size + 4091) / 4092
        } else {
            3
        };

        let tx_descriptors = [$crate::dma::DmaDescriptor::EMPTY; tx_descriptor_len];
        let rx_descriptors = [$crate::dma::DmaDescriptor::EMPTY; rx_descriptor_len];
        unsafe {
            (
                &mut TX_BUFFER,
                tx_descriptors,
                &mut RX_BUFFER,
                rx_descriptors,
            )
        }
    }};

    ($size:expr) => {{
        static mut TX_BUFFER: [u8; $size] = [0u8; $size];
        static mut RX_BUFFER: [u8; $size] = [0u8; $size];

        const descriptor_len: usize = if $size > 4092 * 2 {
            ($size + 4091) / 4092
        } else {
            3
        };

        let tx_descriptors = [$crate::dma::DmaDescriptor::EMPTY; descriptor_len];
        let rx_descriptors = [$crate::dma::DmaDescriptor::EMPTY; descriptor_len];
        unsafe {
            (
                &mut TX_BUFFER,
                tx_descriptors,
                &mut RX_BUFFER,
                rx_descriptors,
            )
        }
    }};
}

/// Convenience macro to create DMA descriptors
///
/// ## Usage
/// ```rust,no_run
/// // Create TX and RX descriptors for transactions up to 32000 bytes - passing only one parameter assumes TX and RX are the same size
/// let (mut tx_descriptors, mut rx_descriptors) = dma_descriptors!(32000, 32000);
/// ```
#[macro_export]
macro_rules! dma_descriptors {
    ($tx_size:expr, $rx_size:expr) => {{
        let tx_descriptors = [$crate::dma::DmaDescriptor::EMPTY; ($tx_size + 4091) / 4092];
        let rx_descriptors = [$crate::dma::DmaDescriptor::EMPTY; ($rx_size + 4091) / 4092];
        (tx_descriptors, rx_descriptors)
    }};

    ($size:expr) => {{
        let tx_descriptors = [$crate::dma::DmaDescriptor::EMPTY; ($size + 4091) / 4092];
        let rx_descriptors = [$crate::dma::DmaDescriptor::EMPTY; ($size + 4091) / 4092];
        (tx_descriptors, rx_descriptors)
    }};
}

/// Convenience macro to create circular DMA descriptors
///
/// ## Usage
/// ```rust,no_run
/// // Create TX and RX descriptors for transactions up to 32000 bytes - passing only one parameter assumes TX and RX are the same size
/// let (mut tx_descriptors, mut rx_descriptors) = dma_circular_descriptors!(32000, 32000);
/// ```
#[macro_export]
macro_rules! dma_circular_descriptors {
    ($tx_size:expr, $rx_size:expr) => {{
        const tx_descriptor_len: usize = if $tx_size > 4092 * 2 {
            ($tx_size + 4091) / 4092
        } else {
            3
        };

        const rx_descriptor_len: usize = if $rx_size > 4092 * 2 {
            ($rx_size + 4091) / 4092
        } else {
            3
        };

        let tx_descriptors = [$crate::dma::DmaDescriptor::EMPTY; tx_descriptor_len];
        let rx_descriptors = [$crate::dma::DmaDescriptor::EMPTY; rx_descriptor_len];
        (tx_descriptors, rx_descriptors)
    }};

    ($size:expr) => {{
        const descriptor_len: usize = if $size > 4092 * 2 {
            ($size + 4091) / 4092
        } else {
            3
        };

        let tx_descriptors = [$crate::dma::DmaDescriptor::EMPTY; descriptor_len];
        let rx_descriptors = [$crate::dma::DmaDescriptor::EMPTY; descriptor_len];
        (tx_descriptors, rx_descriptors)
    }};
}

/// DMA Errors
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DmaError {
    /// The alignment of data is invalid
    InvalidAlignment,
    /// More descriptors are needed for the buffer size
    OutOfDescriptors,
    /// DescriptorError the DMA rejected the descriptor configuration. This
    /// could be because the source address of the data is not in RAM. Ensure
    /// your source data is in a valid address space, or try using
    /// [`crate::FlashSafeDma`] wrapper.
    DescriptorError,
    /// The available free buffer is less than the amount of data to push
    Overflow,
    /// The available amount of data is less than requested
    Exhausted,
    /// The given buffer is too small
    BufferTooSmall,
}

/// DMA Priorities
#[cfg(gdma)]
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(missing_docs)]
pub enum DmaPriority {
    Priority0 = 0,
    Priority1 = 1,
    Priority2 = 2,
    Priority3 = 3,
    Priority4 = 4,
    Priority5 = 5,
    Priority6 = 6,
    Priority7 = 7,
    Priority8 = 8,
    Priority9 = 9,
}

/// DMA Priorities
/// The values need to match the TRM
#[cfg(pdma)]
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(missing_docs)]
pub enum DmaPriority {
    Priority0 = 0,
}

/// DMA capable peripherals
/// The values need to match the TRM
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[doc(hidden)]
pub enum DmaPeripheral {
    Spi2   = 0,
    #[cfg(any(pdma, esp32s3))]
    Spi3   = 1,
    #[cfg(any(esp32c3, esp32c6, esp32h2, esp32s3))]
    Uhci0  = 2,
    #[cfg(any(esp32, esp32s2, esp32c3, esp32c6, esp32h2, esp32s3))]
    I2s0   = 3,
    #[cfg(any(esp32, esp32s3))]
    I2s1   = 4,
    #[cfg(esp32s3)]
    LcdCam = 5,
    #[cfg(not(esp32c2))]
    Aes    = 6,
    #[cfg(gdma)]
    Sha    = 7,
    #[cfg(any(esp32c3, esp32c6, esp32h2, esp32s3))]
    Adc    = 8,
    #[cfg(esp32s3)]
    Rmt    = 9,
    #[cfg(parl_io)]
    ParlIo = 9,
}

#[derive(PartialEq, PartialOrd)]
enum Owner {
    Cpu = 0,
    Dma = 1,
}

impl From<u32> for Owner {
    fn from(value: u32) -> Self {
        match value {
            0 => Owner::Cpu,
            _ => Owner::Dma,
        }
    }
}

/// Marks channels as useable for SPI
#[doc(hidden)]
pub trait SpiPeripheral: PeripheralMarker {}

/// Marks channels as useable for SPI2
#[doc(hidden)]
pub trait Spi2Peripheral: SpiPeripheral + PeripheralMarker {}

/// Marks channels as useable for SPI3
#[cfg(any(esp32, esp32s2, esp32s3))]
#[doc(hidden)]
pub trait Spi3Peripheral: SpiPeripheral + PeripheralMarker {}

/// Marks channels as useable for I2S
#[doc(hidden)]
pub trait I2sPeripheral: PeripheralMarker {}

/// Marks channels as useable for I2S0
#[doc(hidden)]
pub trait I2s0Peripheral: I2sPeripheral + PeripheralMarker {}

/// Marks channels as useable for I2S1
#[doc(hidden)]
pub trait I2s1Peripheral: I2sPeripheral + PeripheralMarker {}

/// Marks channels as useable for PARL_IO
#[doc(hidden)]
pub trait ParlIoPeripheral: PeripheralMarker {}

/// Marks channels as useable for AES
#[doc(hidden)]
pub trait AesPeripheral: PeripheralMarker {}

/// Marks channels as usable for LCD_CAM
#[doc(hidden)]
pub trait LcdCamPeripheral: PeripheralMarker {}

/// DMA Rx
#[doc(hidden)]
pub trait Rx: RxPrivate {}

/// DMA Tx
#[doc(hidden)]
pub trait Tx: TxPrivate {}

/// Marker trait
#[doc(hidden)]
pub trait PeripheralMarker {}

/// The functions here are not meant to be used outside the HAL
#[doc(hidden)]
pub trait RxPrivate: crate::private::Sealed {
    fn init(&mut self, burst_mode: bool, priority: DmaPriority);

    fn init_channel(&mut self);

    unsafe fn prepare_transfer_without_start(
        &mut self,
        circular: bool,
        peri: DmaPeripheral,
        data: *mut u8,
        len: usize,
    ) -> Result<(), DmaError>;

    fn start_transfer(&mut self) -> Result<(), DmaError>;

    fn listen_ch_in_done(&self);

    fn clear_ch_in_done(&self);

    fn is_ch_in_done_set(&self) -> bool;

    fn unlisten_ch_in_done(&self);

    fn is_listening_ch_in_done(&self) -> bool;

    fn is_done(&self) -> bool;

    fn is_listening_eof(&self) -> bool;

    fn listen_eof(&self);

    fn unlisten_eof(&self);

    fn available(&mut self) -> usize;

    fn pop(&mut self, data: &mut [u8]) -> Result<usize, DmaError>;

    fn drain_buffer(&mut self, dst: &mut [u8]) -> Result<usize, DmaError>;

    /// Descriptor error detected
    fn has_error(&self) -> bool;

    /// ERR_DSCR_EMPTY error detected
    fn has_dscr_empty_error(&self) -> bool;

    /// ERR_EOF error detected
    fn has_eof_error(&self) -> bool;

    fn is_listening_in_descriptor_error(&self) -> bool;

    fn listen_in_descriptor_error(&self);

    fn unlisten_in_descriptor_error(&self);

    fn is_listening_in_descriptor_error_dscr_empty(&self) -> bool;

    fn listen_in_descriptor_error_dscr_empty(&self);

    fn unlisten_in_descriptor_error_dscr_empty(&self);

    fn is_listening_in_descriptor_error_err_eof(&self) -> bool;

    fn listen_in_descriptor_error_err_eof(&self);

    fn unlisten_in_descriptor_error_err_eof(&self);

    fn clear_interrupts(&self);

    #[cfg(feature = "async")]
    fn waker() -> &'static embassy_sync::waitqueue::AtomicWaker;
}

#[doc(hidden)]
pub trait RxChannel<R>: crate::private::Sealed
where
    R: RegisterAccess,
{
    fn init(&mut self, burst_mode: bool, priority: DmaPriority) {
        R::set_in_burstmode(burst_mode);
        R::set_in_priority(priority);
    }

    unsafe fn prepare_transfer_without_start(
        &mut self,
        descriptors: &mut [DmaDescriptor],
        circular: bool,
        peri: DmaPeripheral,
        data: *mut u8,
        len: usize,
    ) -> Result<(), DmaError> {
        descriptors.fill(DmaDescriptor::EMPTY);

        compiler_fence(core::sync::atomic::Ordering::SeqCst);

        let max_chunk_size = if !circular || len > CHUNK_SIZE * 2 {
            CHUNK_SIZE
        } else {
            len / 3 + len % 3
        };

        let mut processed = 0;
        let mut descr = 0;
        loop {
            let chunk_size = usize::min(max_chunk_size, len - processed);
            let last = processed + chunk_size >= len;

            let next = if last {
                if circular {
                    addr_of_mut!(descriptors[0])
                } else {
                    core::ptr::null_mut()
                }
            } else {
                addr_of_mut!(descriptors[descr + 1])
            };

            // buffer flags
            let dw0 = &mut descriptors[descr];

            dw0.set_suc_eof(false);
            dw0.set_owner(Owner::Dma);
            dw0.set_size(chunk_size); // align to 32 bits?
            dw0.set_length(0); // hardware will fill in the received number of bytes

            // pointer to current data
            dw0.buffer = unsafe { data.add(processed) };

            // pointer to next descriptor
            dw0.next = next;

            if last {
                break;
            }

            processed += chunk_size;
            descr += 1;
        }

        R::clear_in_interrupts();
        R::reset_in();
        R::set_in_descriptors(descriptors.as_ptr() as u32);
        R::set_in_peripheral(peri as u8);

        Ok(())
    }

    fn start_transfer(&mut self) -> Result<(), DmaError> {
        R::start_in();

        if R::has_in_descriptor_error() {
            Err(DmaError::DescriptorError)
        } else {
            Ok(())
        }
    }

    fn is_done(&self) -> bool {
        R::is_in_done()
    }

    fn last_in_dscr_address(&self) -> usize {
        R::last_in_dscr_address()
    }

    #[cfg(feature = "async")]
    fn waker() -> &'static embassy_sync::waitqueue::AtomicWaker;
}

// DMA receive channel
#[non_exhaustive]
#[doc(hidden)]
pub struct ChannelRx<'a, T, R>
where
    T: RxChannel<R>,
    R: RegisterAccess,
{
    pub(crate) descriptors: &'a mut [DmaDescriptor],
    pub(crate) burst_mode: bool,
    pub(crate) rx_impl: T,
    pub(crate) read_descr_ptr: *mut DmaDescriptor,
    pub(crate) available: usize,
    pub(crate) last_seen_handled_descriptor_ptr: *mut DmaDescriptor,
    pub(crate) read_buffer_start: *mut u8,
    pub(crate) _phantom: PhantomData<R>,
}

impl<'a, T, R> ChannelRx<'a, T, R>
where
    T: RxChannel<R>,
    R: RegisterAccess,
{
    fn new(descriptors: &'a mut [DmaDescriptor], rx_impl: T, burst_mode: bool) -> Self {
        Self {
            descriptors,
            burst_mode,
            rx_impl,
            read_descr_ptr: core::ptr::null_mut(),
            available: 0,
            last_seen_handled_descriptor_ptr: core::ptr::null_mut(),
            read_buffer_start: core::ptr::null_mut(),
            _phantom: PhantomData,
        }
    }
}

impl<'a, T, R> Rx for ChannelRx<'a, T, R>
where
    T: RxChannel<R>,
    R: RegisterAccess,
{
}

impl<'a, T, R> crate::private::Sealed for ChannelRx<'a, T, R>
where
    T: RxChannel<R>,
    R: RegisterAccess,
{
}

impl<'a, T, R> RxPrivate for ChannelRx<'a, T, R>
where
    T: RxChannel<R>,
    R: RegisterAccess,
{
    fn init(&mut self, burst_mode: bool, priority: DmaPriority) {
        self.rx_impl.init(burst_mode, priority);
    }

    unsafe fn prepare_transfer_without_start(
        &mut self,
        circular: bool,
        peri: DmaPeripheral,
        data: *mut u8,
        len: usize,
    ) -> Result<(), DmaError> {
        if self.descriptors.len() < (len + CHUNK_SIZE - 1) / CHUNK_SIZE {
            return Err(DmaError::OutOfDescriptors);
        }

        if self.burst_mode && (len % 4 != 0 || data as u32 % 4 != 0) {
            return Err(DmaError::InvalidAlignment);
        }

        if circular && len <= 3 {
            return Err(DmaError::BufferTooSmall);
        }

        self.available = 0;
        self.read_descr_ptr = self.descriptors.as_mut_ptr();
        self.last_seen_handled_descriptor_ptr = core::ptr::null_mut();
        self.read_buffer_start = data;

        self.rx_impl
            .prepare_transfer_without_start(self.descriptors, circular, peri, data, len)
    }

    fn start_transfer(&mut self) -> Result<(), DmaError> {
        self.rx_impl.start_transfer()
    }

    fn listen_ch_in_done(&self) {
        R::listen_ch_in_done();
    }

    fn clear_ch_in_done(&self) {
        R::clear_ch_in_done();
    }

    fn is_ch_in_done_set(&self) -> bool {
        R::is_ch_in_done_set()
    }

    fn unlisten_ch_in_done(&self) {
        R::unlisten_ch_in_done();
    }

    fn is_listening_ch_in_done(&self) -> bool {
        R::is_listening_ch_in_done()
    }

    fn is_done(&self) -> bool {
        self.rx_impl.is_done()
    }

    fn init_channel(&mut self) {
        R::init_channel();
    }

    fn available(&mut self) -> usize {
        if self.last_seen_handled_descriptor_ptr.is_null() {
            self.last_seen_handled_descriptor_ptr = self.descriptors.as_mut_ptr();
            return 0;
        }

        if self.available != 0 {
            return self.available;
        }

        let descr_address = self.last_seen_handled_descriptor_ptr;
        let mut dw0 = unsafe { descr_address.read_volatile() };

        if dw0.owner() == Owner::Cpu && !dw0.is_empty() {
            let descriptor_buffer = dw0.buffer;
            let next_descriptor = dw0.next;

            self.read_buffer_start = descriptor_buffer;
            self.available = dw0.len();

            dw0.set_owner(Owner::Dma);
            dw0.set_length(0);
            dw0.set_suc_eof(false);

            unsafe {
                descr_address.write_volatile(dw0);
            }

            self.last_seen_handled_descriptor_ptr = if next_descriptor.is_null() {
                self.descriptors.as_mut_ptr()
            } else {
                next_descriptor
            };
        }

        self.available
    }

    fn pop(&mut self, data: &mut [u8]) -> Result<usize, DmaError> {
        let avail = self.available;

        if avail < data.len() {
            return Err(DmaError::Exhausted);
        }

        unsafe {
            let dst = data.as_mut_ptr();
            let src = self.read_buffer_start;
            let count = self.available;
            core::ptr::copy_nonoverlapping(src, dst, count);
        }

        self.available = 0;
        Ok(data.len())
    }

    fn drain_buffer(&mut self, mut dst: &mut [u8]) -> Result<usize, DmaError> {
        let mut len = 0;
        let mut idx = 0;
        loop {
            let chunk_len = dst.len().min(self.descriptors[idx].len());
            if chunk_len == 0 {
                break;
            }

            let buffer_ptr = self.descriptors[idx].buffer;
            let next_dscr = self.descriptors[idx].next;

            // Copy data to destination
            let (dst_chunk, dst_remaining) = dst.split_at_mut(chunk_len);
            dst = dst_remaining;

            dst_chunk
                .copy_from_slice(unsafe { core::slice::from_raw_parts(buffer_ptr, chunk_len) });

            len += chunk_len;

            if next_dscr.is_null() {
                break;
            }

            idx += 3;
        }

        Ok(len)
    }

    fn is_listening_eof(&self) -> bool {
        R::is_listening_in_eof()
    }

    fn listen_eof(&self) {
        R::listen_in_eof()
    }

    fn unlisten_eof(&self) {
        R::unlisten_in_eof()
    }

    fn has_error(&self) -> bool {
        R::has_in_descriptor_error()
    }

    fn has_dscr_empty_error(&self) -> bool {
        R::has_in_descriptor_error_dscr_empty()
    }

    fn has_eof_error(&self) -> bool {
        R::has_in_descriptor_error_err_eof()
    }

    fn is_listening_in_descriptor_error(&self) -> bool {
        R::is_listening_in_descriptor_error()
    }

    fn listen_in_descriptor_error(&self) {
        R::listen_in_descriptor_error();
    }

    fn unlisten_in_descriptor_error(&self) {
        R::unlisten_in_descriptor_error();
    }

    fn is_listening_in_descriptor_error_dscr_empty(&self) -> bool {
        R::is_listening_in_descriptor_error_dscr_empty()
    }

    fn listen_in_descriptor_error_dscr_empty(&self) {
        R::listen_in_descriptor_error_dscr_empty();
    }

    fn unlisten_in_descriptor_error_dscr_empty(&self) {
        R::unlisten_in_descriptor_error_dscr_empty();
    }

    fn is_listening_in_descriptor_error_err_eof(&self) -> bool {
        R::is_listening_in_descriptor_error_err_eof()
    }

    fn listen_in_descriptor_error_err_eof(&self) {
        R::listen_in_descriptor_error_err_eof();
    }

    fn unlisten_in_descriptor_error_err_eof(&self) {
        R::unlisten_in_descriptor_error_err_eof();
    }

    fn clear_interrupts(&self) {
        R::clear_in_interrupts();
    }

    #[cfg(feature = "async")]
    fn waker() -> &'static embassy_sync::waitqueue::AtomicWaker {
        T::waker()
    }
}

/// The functions here are not meant to be used outside the HAL
#[doc(hidden)]
pub trait TxPrivate: crate::private::Sealed {
    fn init(&mut self, burst_mode: bool, priority: DmaPriority);

    fn init_channel(&mut self);

    fn prepare_transfer_without_start(
        &mut self,
        peri: DmaPeripheral,
        circular: bool,
        data: *const u8,
        len: usize,
    ) -> Result<(), DmaError>;

    fn start_transfer(&mut self) -> Result<(), DmaError>;

    fn clear_ch_out_done(&self);

    fn is_ch_out_done_set(&self) -> bool;

    fn listen_ch_out_done(&self);

    fn unlisten_ch_out_done(&self);

    fn is_listening_ch_out_done(&self) -> bool;

    fn is_done(&self) -> bool;

    fn is_listening_eof(&self) -> bool;

    fn listen_eof(&self);

    fn unlisten_eof(&self);

    fn is_listening_out_descriptor_error(&self) -> bool;

    fn listen_out_descriptor_error(&self);

    fn unlisten_out_descriptor_error(&self);

    fn available(&mut self) -> usize;

    fn has_error(&self) -> bool;

    fn push(&mut self, data: &[u8]) -> Result<usize, DmaError>;

    fn push_with(&mut self, f: impl FnOnce(&mut [u8]) -> usize) -> Result<usize, DmaError>;

    fn clear_interrupts(&self);

    #[cfg(feature = "async")]
    fn waker() -> &'static embassy_sync::waitqueue::AtomicWaker;
}

#[doc(hidden)]
pub trait TxChannel<R>: crate::private::Sealed
where
    R: RegisterAccess,
{
    fn init(&mut self, burst_mode: bool, priority: DmaPriority) {
        R::set_out_burstmode(burst_mode);
        R::set_out_priority(priority);
    }

    fn prepare_transfer_without_start(
        &mut self,
        descriptors: &mut [DmaDescriptor],
        circular: bool,
        peri: DmaPeripheral,
        data: *const u8,
        len: usize,
    ) -> Result<(), DmaError> {
        descriptors.fill(DmaDescriptor::EMPTY);

        compiler_fence(core::sync::atomic::Ordering::SeqCst);

        let max_chunk_size = if !circular || len > CHUNK_SIZE * 2 {
            CHUNK_SIZE
        } else {
            len / 3 + len % 3
        };

        let mut processed = 0;
        let mut descr = 0;
        loop {
            let chunk_size = usize::min(max_chunk_size, len - processed);
            let last = processed + chunk_size >= len;

            let next = if last {
                if circular {
                    addr_of_mut!(descriptors[0])
                } else {
                    core::ptr::null_mut()
                }
            } else {
                addr_of_mut!(descriptors[descr + 1])
            };

            // buffer flags
            let dw0 = &mut descriptors[descr];

            // The `suc_eof` bit doesn't affect the transfer itself, but signals when the
            // hardware should trigger an interrupt request. In circular mode,
            // we set the `suc_eof` bit for every buffer we send. We use this for
            // I2S to track progress of a transfer by checking OUTLINK_DSCR_ADDR.
            dw0.set_suc_eof(circular || last);
            dw0.set_owner(Owner::Dma);
            dw0.set_size(chunk_size); // align to 32 bits?
            dw0.set_length(chunk_size); // the hardware will transmit this many bytes

            // pointer to current data
            dw0.buffer = unsafe { data.cast_mut().add(processed) };

            // pointer to next descriptor
            dw0.next = next;

            if last {
                break;
            }

            processed += chunk_size;
            descr += 1;
        }

        R::clear_out_interrupts();
        R::reset_out();
        R::set_out_descriptors(addr_of_mut!(descriptors[0]) as u32);
        R::set_out_peripheral(peri as u8);

        Ok(())
    }

    fn start_transfer(&mut self) -> Result<(), DmaError> {
        R::start_out();

        if R::has_out_descriptor_error() {
            Err(DmaError::DescriptorError)
        } else {
            Ok(())
        }
    }

    fn clear_ch_out_done(&self) {
        R::clear_ch_out_done();
    }

    fn is_ch_out_done_set(&self) -> bool {
        R::is_ch_out_done_set()
    }

    fn listen_ch_out_done(&self) {
        R::listen_ch_out_done();
    }

    fn unlisten_ch_out_done(&self) {
        R::unlisten_ch_out_done();
    }

    fn is_listening_ch_out_done(&self) -> bool {
        R::is_listening_ch_out_done()
    }

    fn is_done(&self) -> bool {
        R::is_out_done()
    }

    fn descriptors_handled(&self) -> bool {
        R::is_out_eof_interrupt_set()
    }

    fn reset_descriptors_handled(&self) {
        R::reset_out_eof_interrupt();
    }

    fn last_out_dscr_address(&self) -> usize {
        R::last_out_dscr_address()
    }

    #[cfg(feature = "async")]
    fn waker() -> &'static embassy_sync::waitqueue::AtomicWaker;
}

/// DMA transmit channel
#[doc(hidden)]
pub struct ChannelTx<'a, T, R>
where
    T: TxChannel<R>,
    R: RegisterAccess,
{
    pub(crate) descriptors: &'a mut [DmaDescriptor],
    #[allow(unused)]
    pub(crate) burst_mode: bool,
    pub(crate) tx_impl: T,
    pub(crate) write_offset: usize,
    pub(crate) write_descr_ptr: *mut DmaDescriptor,
    pub(crate) available: usize,
    pub(crate) last_seen_handled_descriptor_ptr: *mut DmaDescriptor,
    pub(crate) buffer_start: *const u8,
    pub(crate) buffer_len: usize,
    pub(crate) _phantom: PhantomData<R>,
}

impl<'a, T, R> ChannelTx<'a, T, R>
where
    T: TxChannel<R>,
    R: RegisterAccess,
{
    fn new(descriptors: &'a mut [DmaDescriptor], tx_impl: T, burst_mode: bool) -> Self {
        Self {
            descriptors,
            burst_mode,
            tx_impl,
            write_offset: 0,
            write_descr_ptr: core::ptr::null_mut(),
            available: 0,
            last_seen_handled_descriptor_ptr: core::ptr::null_mut(),
            buffer_start: core::ptr::null_mut(),
            buffer_len: 0,
            _phantom: PhantomData,
        }
    }
}

impl<'a, T, R> Tx for ChannelTx<'a, T, R>
where
    T: TxChannel<R>,
    R: RegisterAccess,
{
}

impl<'a, T, R> crate::private::Sealed for ChannelTx<'a, T, R>
where
    T: TxChannel<R>,
    R: RegisterAccess,
{
}

impl<'a, T, R> TxPrivate for ChannelTx<'a, T, R>
where
    T: TxChannel<R>,
    R: RegisterAccess,
{
    fn init(&mut self, burst_mode: bool, priority: DmaPriority) {
        self.tx_impl.init(burst_mode, priority);
    }

    fn init_channel(&mut self) {
        R::init_channel();
    }

    fn prepare_transfer_without_start(
        &mut self,
        peri: DmaPeripheral,
        circular: bool,
        data: *const u8,
        len: usize,
    ) -> Result<(), DmaError> {
        if self.descriptors.len() < (len + CHUNK_SIZE - 1) / CHUNK_SIZE {
            return Err(DmaError::OutOfDescriptors);
        }

        if circular && len <= 3 {
            return Err(DmaError::BufferTooSmall);
        }

        self.write_offset = 0;
        self.available = 0;
        self.write_descr_ptr = self.descriptors.as_mut_ptr();
        self.last_seen_handled_descriptor_ptr = self.descriptors.as_mut_ptr();
        self.buffer_start = data;
        self.buffer_len = len;

        self.tx_impl
            .prepare_transfer_without_start(self.descriptors, circular, peri, data, len)
    }

    fn start_transfer(&mut self) -> Result<(), DmaError> {
        self.tx_impl.start_transfer()
    }

    fn clear_ch_out_done(&self) {
        self.tx_impl.clear_ch_out_done();
    }

    fn is_ch_out_done_set(&self) -> bool {
        self.tx_impl.is_ch_out_done_set()
    }

    fn listen_ch_out_done(&self) {
        self.tx_impl.listen_ch_out_done();
    }

    fn unlisten_ch_out_done(&self) {
        self.tx_impl.unlisten_ch_out_done();
    }

    fn is_listening_ch_out_done(&self) -> bool {
        self.tx_impl.is_listening_ch_out_done()
    }

    fn is_done(&self) -> bool {
        self.tx_impl.is_done()
    }

    fn available(&mut self) -> usize {
        if self.tx_impl.descriptors_handled() {
            self.tx_impl.reset_descriptors_handled();
            let descr_address = self.tx_impl.last_out_dscr_address() as *mut DmaDescriptor;

            let mut ptr = self.last_seen_handled_descriptor_ptr;
            if descr_address >= self.last_seen_handled_descriptor_ptr {
                unsafe {
                    while ptr < descr_address {
                        let dw0 = ptr.read_volatile();
                        self.available += dw0.len();
                        ptr = ptr.offset(1);
                    }
                }
            } else {
                unsafe {
                    while !((*ptr).next.is_null()
                        || (*ptr).next == addr_of_mut!(self.descriptors[0]))
                    {
                        let dw0 = ptr.read_volatile();
                        self.available += dw0.len();
                        ptr = ptr.offset(1);
                    }

                    // add bytes pointed to by the last descriptor
                    let dw0 = ptr.read_volatile();
                    self.available += dw0.len();

                    // in circular mode we need to honor the now available bytes at start
                    if (*ptr).next == addr_of_mut!(self.descriptors[0]) {
                        ptr = addr_of_mut!(self.descriptors[0]);
                        while ptr < descr_address {
                            let dw0 = ptr.read_volatile();
                            self.available += dw0.len();
                            ptr = ptr.offset(1);
                        }
                    }
                }
            }

            if self.available >= self.buffer_len {
                unsafe {
                    let dw0 = self.write_descr_ptr.read_volatile();
                    let segment_len = dw0.len();
                    let next_descriptor = dw0.next;
                    self.available -= segment_len;
                    self.write_offset = (self.write_offset + segment_len) % self.buffer_len;

                    self.write_descr_ptr = if next_descriptor.is_null() {
                        self.descriptors.as_mut_ptr()
                    } else {
                        next_descriptor
                    }
                }
            }

            self.last_seen_handled_descriptor_ptr = descr_address;
        }

        self.available
    }

    fn push(&mut self, data: &[u8]) -> Result<usize, DmaError> {
        let avail = self.available();

        if avail < data.len() {
            return Err(DmaError::Overflow);
        }

        let mut remaining = data.len();
        let mut offset = 0;
        while self.available() >= remaining && remaining > 0 {
            let written = self.push_with(|buffer| {
                let len = usize::min(buffer.len(), data.len() - offset);
                buffer[..len].copy_from_slice(&data[offset..][..len]);
                len
            })?;
            offset += written;
            remaining -= written;
        }

        Ok(data.len())
    }

    fn push_with(&mut self, f: impl FnOnce(&mut [u8]) -> usize) -> Result<usize, DmaError> {
        let written = unsafe {
            let dst = self.buffer_start.add(self.write_offset).cast_mut();
            let block_size = usize::min(self.available(), self.buffer_len - self.write_offset);
            let buffer = core::slice::from_raw_parts_mut(dst, block_size);
            f(buffer)
        };

        let mut forward = written;
        loop {
            unsafe {
                let dw0 = self.write_descr_ptr.read_volatile();
                let segment_len = dw0.len();
                self.write_descr_ptr = if dw0.next.is_null() {
                    self.descriptors.as_mut_ptr()
                } else {
                    dw0.next
                };

                if forward <= segment_len {
                    break;
                }

                forward -= segment_len;
            }
        }

        self.write_offset = (self.write_offset + written) % self.buffer_len;
        self.available -= written;

        Ok(written)
    }

    fn is_listening_eof(&self) -> bool {
        R::is_listening_out_eof()
    }

    fn listen_eof(&self) {
        R::listen_out_eof()
    }

    fn unlisten_eof(&self) {
        R::unlisten_out_eof()
    }

    fn has_error(&self) -> bool {
        R::has_out_descriptor_error()
    }

    #[cfg(feature = "async")]
    fn waker() -> &'static embassy_sync::waitqueue::AtomicWaker {
        T::waker()
    }

    fn is_listening_out_descriptor_error(&self) -> bool {
        R::is_listening_out_descriptor_error()
    }

    fn listen_out_descriptor_error(&self) {
        R::listen_out_descriptor_error();
    }

    fn unlisten_out_descriptor_error(&self) {
        R::unlisten_out_descriptor_error();
    }

    fn clear_interrupts(&self) {
        R::clear_out_interrupts();
    }
}

#[doc(hidden)]
pub trait RegisterAccess: crate::private::Sealed {
    fn init_channel();
    fn set_out_burstmode(burst_mode: bool);
    fn set_out_priority(priority: DmaPriority);
    fn clear_out_interrupts();
    fn reset_out();
    fn set_out_descriptors(address: u32);
    fn has_out_descriptor_error() -> bool;
    fn set_out_peripheral(peripheral: u8);
    fn start_out();
    fn clear_ch_out_done();
    fn is_ch_out_done_set() -> bool;
    fn listen_ch_out_done();
    fn unlisten_ch_out_done();
    fn is_listening_ch_out_done() -> bool;
    fn is_out_done() -> bool;
    fn is_out_eof_interrupt_set() -> bool;
    fn reset_out_eof_interrupt();
    fn last_out_dscr_address() -> usize;

    fn set_in_burstmode(burst_mode: bool);
    fn set_in_priority(priority: DmaPriority);
    fn clear_in_interrupts();
    fn reset_in();
    fn set_in_descriptors(address: u32);
    fn has_in_descriptor_error() -> bool;
    fn has_in_descriptor_error_dscr_empty() -> bool;
    fn has_in_descriptor_error_err_eof() -> bool;
    fn set_in_peripheral(peripheral: u8);
    fn start_in();
    fn is_in_done() -> bool;
    fn last_in_dscr_address() -> usize;

    fn is_listening_in_eof() -> bool;
    fn is_listening_out_eof() -> bool;

    fn listen_in_eof();
    fn listen_out_eof();
    fn unlisten_in_eof();
    fn unlisten_out_eof();

    fn listen_in_descriptor_error();
    fn unlisten_in_descriptor_error();
    fn is_listening_in_descriptor_error() -> bool;

    fn listen_in_descriptor_error_dscr_empty();
    fn unlisten_in_descriptor_error_dscr_empty();
    fn is_listening_in_descriptor_error_dscr_empty() -> bool;

    fn listen_in_descriptor_error_err_eof();
    fn unlisten_in_descriptor_error_err_eof();
    fn is_listening_in_descriptor_error_err_eof() -> bool;

    fn listen_out_descriptor_error();
    fn unlisten_out_descriptor_error();
    fn is_listening_out_descriptor_error() -> bool;

    fn listen_ch_in_done();
    fn clear_ch_in_done();
    fn is_ch_in_done_set() -> bool;
    fn unlisten_ch_in_done();
    fn is_listening_ch_in_done() -> bool;
}

#[doc(hidden)]
pub trait ChannelTypes: crate::private::Sealed {
    type P: PeripheralMarker;
    type Tx<'a>: Tx;
    type Rx<'a>: Rx;
    type Binder: InterruptBinder;
}

#[doc(hidden)]
pub trait InterruptBinder: crate::private::Sealed {
    fn set_isr(handler: InterruptHandler);
}

/// DMA Channel
pub struct Channel<'d, C, MODE>
where
    C: ChannelTypes,
    MODE: Mode,
{
    /// TX half of the channel
    pub tx: C::Tx<'d>,
    /// RX half of the channel
    pub rx: C::Rx<'d>,
    phantom: PhantomData<MODE>,
}

impl<'d, C> Channel<'d, C, crate::Blocking>
where
    C: ChannelTypes,
{
    /// Sets the interrupt handler for TX and RX interrupts, enables them
    /// with [crate::interrupt::Priority::max()]
    ///
    /// Interrupts are not enabled at the peripheral level here.
    pub fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        C::Binder::set_isr(handler);
    }

    /// Listen for the given interrupts
    pub fn listen(&mut self, interrupts: EnumSet<DmaInterrupt>) {
        for interrupt in interrupts {
            match interrupt {
                DmaInterrupt::TxDone => self.tx.listen_ch_out_done(),
                DmaInterrupt::RxDone => self.rx.listen_ch_in_done(),
            }
        }
    }

    /// Unlisten the given interrupts
    pub fn unlisten(&mut self, interrupts: EnumSet<DmaInterrupt>) {
        for interrupt in interrupts {
            match interrupt {
                DmaInterrupt::TxDone => self.tx.unlisten_ch_out_done(),
                DmaInterrupt::RxDone => self.rx.unlisten_ch_in_done(),
            }
        }
    }

    /// Gets asserted interrupts
    pub fn interrupts(&mut self) -> EnumSet<DmaInterrupt> {
        let mut res = EnumSet::new();
        if self.tx.is_done() {
            res.insert(DmaInterrupt::TxDone);
        }
        if self.rx.is_done() {
            res.insert(DmaInterrupt::RxDone);
        }
        res
    }

    /// Resets asserted interrupts
    pub fn clear_interrupts(&mut self, interrupts: EnumSet<DmaInterrupt>) {
        for interrupt in interrupts {
            match interrupt {
                DmaInterrupt::TxDone => self.tx.clear_ch_out_done(),
                DmaInterrupt::RxDone => self.rx.clear_ch_in_done(),
            }
        }
    }
}

pub(crate) mod dma_private {
    use super::*;

    pub trait DmaSupport {
        /// Wait until the transfer is done.
        ///
        /// Depending on the peripheral this might include checking the DMA
        /// channel and/or the peripheral.
        ///
        /// After this all data should be processed by the peripheral - i.e. the
        /// peripheral should have processed it's FIFO(s)
        fn peripheral_wait_dma(&mut self, is_tx: bool, is_rx: bool);

        /// Only used by circular DMA transfers
        fn peripheral_dma_stop(&mut self);
    }

    pub trait DmaSupportTx: DmaSupport {
        type TX: Tx;

        fn tx(&mut self) -> &mut Self::TX;
    }

    pub trait DmaSupportRx: DmaSupport {
        type RX: Rx;

        fn rx(&mut self) -> &mut Self::RX;
    }
}

/// DMA transaction for TX only transfers
#[non_exhaustive]
#[must_use]
pub struct DmaTransferTx<'a, I>
where
    I: dma_private::DmaSupportTx,
{
    instance: &'a mut I,
}

impl<'a, I> DmaTransferTx<'a, I>
where
    I: dma_private::DmaSupportTx,
{
    pub(crate) fn new(instance: &'a mut I) -> Self {
        Self { instance }
    }

    /// Wait for the transfer to finish.
    pub fn wait(self) -> Result<(), DmaError> {
        self.instance.peripheral_wait_dma(true, false);

        if self.instance.tx().has_error() {
            Err(DmaError::DescriptorError)
        } else {
            Ok(())
        }
    }

    /// Check if the transfer is finished.
    pub fn is_done(&mut self) -> bool {
        self.instance.tx().is_done()
    }
}

impl<'a, I> Drop for DmaTransferTx<'a, I>
where
    I: dma_private::DmaSupportTx,
{
    fn drop(&mut self) {
        self.instance.peripheral_wait_dma(true, false);
    }
}

/// DMA transaction for RX only transfers
#[non_exhaustive]
#[must_use]
pub struct DmaTransferRx<'a, I>
where
    I: dma_private::DmaSupportRx,
{
    instance: &'a mut I,
}

impl<'a, I> DmaTransferRx<'a, I>
where
    I: dma_private::DmaSupportRx,
{
    pub(crate) fn new(instance: &'a mut I) -> Self {
        Self { instance }
    }

    /// Wait for the transfer to finish.
    pub fn wait(self) -> Result<(), DmaError> {
        self.instance.peripheral_wait_dma(false, true);

        if self.instance.rx().has_error() {
            Err(DmaError::DescriptorError)
        } else {
            Ok(())
        }
    }

    /// Check if the transfer is finished.
    pub fn is_done(&mut self) -> bool {
        self.instance.rx().is_done()
    }
}

impl<'a, I> Drop for DmaTransferRx<'a, I>
where
    I: dma_private::DmaSupportRx,
{
    fn drop(&mut self) {
        self.instance.peripheral_wait_dma(false, true);
    }
}

/// DMA transaction for TX+RX transfers
#[non_exhaustive]
#[must_use]
pub struct DmaTransferTxRx<'a, I>
where
    I: dma_private::DmaSupportTx + dma_private::DmaSupportRx,
{
    instance: &'a mut I,
}

impl<'a, I> DmaTransferTxRx<'a, I>
where
    I: dma_private::DmaSupportTx + dma_private::DmaSupportRx,
{
    pub(crate) fn new(instance: &'a mut I) -> Self {
        Self { instance }
    }

    /// Wait for the transfer to finish.
    pub fn wait(self) -> Result<(), DmaError> {
        self.instance.peripheral_wait_dma(true, true);

        if self.instance.tx().has_error() || self.instance.rx().has_error() {
            Err(DmaError::DescriptorError)
        } else {
            Ok(())
        }
    }

    /// Check if the transfer is finished.
    pub fn is_done(&mut self) -> bool {
        self.instance.tx().is_done() && self.instance.rx().is_done()
    }
}

impl<'a, I> Drop for DmaTransferTxRx<'a, I>
where
    I: dma_private::DmaSupportTx + dma_private::DmaSupportRx,
{
    fn drop(&mut self) {
        self.instance.peripheral_wait_dma(true, true);
    }
}

/// DMA transaction for TX only circular transfers
#[non_exhaustive]
#[must_use]
pub struct DmaTransferTxCircular<'a, I>
where
    I: dma_private::DmaSupportTx,
{
    instance: &'a mut I,
}

impl<'a, I> DmaTransferTxCircular<'a, I>
where
    I: dma_private::DmaSupportTx,
{
    #[allow(unused)] // currently used by peripherals not available on all chips
    pub(crate) fn new(instance: &'a mut I) -> Self {
        Self { instance }
    }

    /// Amount of bytes which can be pushed.
    pub fn available(&mut self) -> usize {
        self.instance.tx().available()
    }

    /// Push bytes into the DMA buffer.
    pub fn push(&mut self, data: &[u8]) -> Result<usize, DmaError> {
        self.instance.tx().push(data)
    }

    /// Push bytes into the DMA buffer via the given closure.
    /// The closure *must* return the actual number of bytes written.
    /// The closure *might* get called with a slice which is smaller than the
    /// total available buffer.
    pub fn push_with(&mut self, f: impl FnOnce(&mut [u8]) -> usize) -> Result<usize, DmaError> {
        self.instance.tx().push_with(f)
    }

    /// Stop the DMA transfer
    #[allow(clippy::type_complexity)]
    pub fn stop(self) -> Result<(), DmaError> {
        self.instance.peripheral_dma_stop();

        if self.instance.tx().has_error() {
            Err(DmaError::DescriptorError)
        } else {
            Ok(())
        }
    }
}

impl<'a, I> Drop for DmaTransferTxCircular<'a, I>
where
    I: dma_private::DmaSupportTx,
{
    fn drop(&mut self) {
        self.instance.peripheral_dma_stop();
    }
}

/// DMA transaction for RX only circular transfers
#[non_exhaustive]
#[must_use]
pub struct DmaTransferRxCircular<'a, I>
where
    I: dma_private::DmaSupportRx,
{
    instance: &'a mut I,
}

impl<'a, I> DmaTransferRxCircular<'a, I>
where
    I: dma_private::DmaSupportRx,
{
    #[allow(unused)] // currently used by peripherals not available on all chips
    pub(crate) fn new(instance: &'a mut I) -> Self {
        Self { instance }
    }

    /// Amount of bytes which can be popped
    pub fn available(&mut self) -> usize {
        self.instance.rx().available()
    }

    /// Get available data
    pub fn pop(&mut self, data: &mut [u8]) -> Result<usize, DmaError> {
        self.instance.rx().pop(data)
    }
}

impl<'a, I> Drop for DmaTransferRxCircular<'a, I>
where
    I: dma_private::DmaSupportRx,
{
    fn drop(&mut self) {
        self.instance.peripheral_dma_stop();
    }
}

#[cfg(feature = "async")]
pub(crate) mod asynch {
    use core::task::Poll;

    use super::*;

    pub struct DmaTxFuture<'a, TX> {
        pub(crate) tx: &'a mut TX,
        _a: (),
    }

    impl<'a, TX> DmaTxFuture<'a, TX>
    where
        TX: Tx,
    {
        pub fn new(tx: &'a mut TX) -> Self {
            tx.listen_eof();
            tx.listen_out_descriptor_error();
            Self { tx, _a: () }
        }

        pub fn tx(&mut self) -> &mut TX {
            self.tx
        }
    }

    impl<'a, TX> core::future::Future for DmaTxFuture<'a, TX>
    where
        TX: Tx,
    {
        type Output = Result<(), DmaError>;

        fn poll(
            self: core::pin::Pin<&mut Self>,
            cx: &mut core::task::Context<'_>,
        ) -> Poll<Self::Output> {
            TX::waker().register(cx.waker());
            if self.tx.is_listening_eof() {
                Poll::Pending
            } else {
                if self.tx.has_error() {
                    self.tx.clear_interrupts();
                    Poll::Ready(Err(DmaError::DescriptorError))
                } else {
                    Poll::Ready(Ok(()))
                }
            }
        }
    }

    pub struct DmaRxFuture<'a, RX> {
        pub(crate) rx: &'a mut RX,
        _a: (),
    }

    impl<'a, RX> DmaRxFuture<'a, RX>
    where
        RX: Rx,
    {
        pub fn new(rx: &'a mut RX) -> Self {
            rx.listen_eof();
            rx.listen_in_descriptor_error();
            rx.listen_in_descriptor_error_dscr_empty();
            rx.listen_in_descriptor_error_err_eof();
            Self { rx, _a: () }
        }

        pub fn rx(&mut self) -> &mut RX {
            self.rx
        }
    }

    impl<'a, RX> core::future::Future for DmaRxFuture<'a, RX>
    where
        RX: Rx,
    {
        type Output = Result<(), DmaError>;

        fn poll(
            self: core::pin::Pin<&mut Self>,
            cx: &mut core::task::Context<'_>,
        ) -> Poll<Self::Output> {
            RX::waker().register(cx.waker());
            if self.rx.is_listening_eof() {
                Poll::Pending
            } else {
                if self.rx.has_error() || self.rx.has_dscr_empty_error() || self.rx.has_eof_error()
                {
                    self.rx.clear_interrupts();
                    Poll::Ready(Err(DmaError::DescriptorError))
                } else {
                    Poll::Ready(Ok(()))
                }
            }
        }
    }

    #[cfg(any(i2s0, i2s1))]
    pub struct DmaTxDoneChFuture<'a, TX> {
        pub(crate) tx: &'a mut TX,
        _a: (),
    }

    #[cfg(any(i2s0, i2s1))]
    impl<'a, TX> DmaTxDoneChFuture<'a, TX>
    where
        TX: Tx,
    {
        pub fn new(tx: &'a mut TX) -> Self {
            tx.listen_ch_out_done();
            tx.listen_out_descriptor_error();
            Self { tx, _a: () }
        }
    }

    #[cfg(any(i2s0, i2s1))]
    impl<'a, TX> core::future::Future for DmaTxDoneChFuture<'a, TX>
    where
        TX: Tx,
    {
        type Output = Result<(), DmaError>;

        fn poll(
            self: core::pin::Pin<&mut Self>,
            cx: &mut core::task::Context<'_>,
        ) -> Poll<Self::Output> {
            TX::waker().register(cx.waker());
            if self.tx.is_listening_ch_out_done() {
                Poll::Pending
            } else {
                if self.tx.has_error() {
                    self.tx.clear_interrupts();
                    Poll::Ready(Err(DmaError::DescriptorError))
                } else {
                    Poll::Ready(Ok(()))
                }
            }
        }
    }

    #[cfg(any(i2s0, i2s1))]
    pub struct DmaRxDoneChFuture<'a, RX> {
        pub(crate) rx: &'a mut RX,
        _a: (),
    }

    #[cfg(any(i2s0, i2s1))]
    impl<'a, RX> DmaRxDoneChFuture<'a, RX>
    where
        RX: Rx,
    {
        pub fn new(rx: &'a mut RX) -> Self {
            rx.listen_ch_in_done();
            rx.listen_in_descriptor_error();
            rx.listen_in_descriptor_error_dscr_empty();
            rx.listen_in_descriptor_error_err_eof();
            Self { rx, _a: () }
        }
    }

    #[cfg(any(i2s0, i2s1))]
    impl<'a, RX> core::future::Future for DmaRxDoneChFuture<'a, RX>
    where
        RX: Rx,
    {
        type Output = Result<(), DmaError>;

        fn poll(
            self: core::pin::Pin<&mut Self>,
            cx: &mut core::task::Context<'_>,
        ) -> Poll<Self::Output> {
            RX::waker().register(cx.waker());
            if self.rx.is_listening_ch_in_done() {
                Poll::Pending
            } else {
                if self.rx.has_error() || self.rx.has_dscr_empty_error() || self.rx.has_eof_error()
                {
                    self.rx.clear_interrupts();
                    Poll::Ready(Err(DmaError::DescriptorError))
                } else {
                    Poll::Ready(Ok(()))
                }
            }
        }
    }

    fn handle_interrupt<Channel: RegisterAccess, Rx: RxChannel<Channel>, Tx: TxChannel<Channel>>() {
        if Channel::has_in_descriptor_error()
            || Channel::has_in_descriptor_error_dscr_empty()
            || Channel::has_in_descriptor_error_err_eof()
        {
            Channel::unlisten_in_descriptor_error();
            Channel::unlisten_in_descriptor_error_dscr_empty();
            Channel::unlisten_in_descriptor_error_err_eof();
            Channel::unlisten_in_eof();
            Channel::unlisten_ch_in_done();
            Rx::waker().wake()
        }

        if Channel::has_out_descriptor_error() {
            Channel::unlisten_out_descriptor_error();
            Channel::unlisten_out_eof();
            Channel::unlisten_ch_out_done();
            Tx::waker().wake()
        }

        if Channel::is_in_done() && Channel::is_listening_in_eof() {
            Channel::clear_in_interrupts();
            Channel::unlisten_in_eof();
            Rx::waker().wake()
        }

        if Channel::is_ch_in_done_set() {
            Channel::clear_ch_in_done();
            Channel::unlisten_ch_in_done();
            Rx::waker().wake()
        }

        if Channel::is_out_done() && Channel::is_listening_out_eof() {
            Channel::clear_out_interrupts();
            Channel::unlisten_out_eof();
            Tx::waker().wake()
        }

        if Channel::is_ch_out_done_set() {
            Channel::clear_ch_out_done();
            Channel::unlisten_ch_out_done();
            Tx::waker().wake()
        }
    }

    #[cfg(not(any(esp32, esp32s2)))]
    pub(crate) mod interrupt {
        use procmacros::handler;

        use super::*;

        #[handler(priority = crate::interrupt::Priority::max())]
        pub(crate) fn interrupt_handler_ch0() {
            use crate::dma::gdma::{
                Channel0 as Channel,
                Channel0RxImpl as ChannelRxImpl,
                Channel0TxImpl as ChannelTxImpl,
            };

            handle_interrupt::<Channel, ChannelRxImpl, ChannelTxImpl>();
        }

        #[cfg(not(esp32c2))]
        #[handler(priority = crate::interrupt::Priority::max())]
        pub(crate) fn interrupt_handler_ch1() {
            use crate::dma::gdma::{
                Channel1 as Channel,
                Channel1RxImpl as ChannelRxImpl,
                Channel1TxImpl as ChannelTxImpl,
            };

            handle_interrupt::<Channel, ChannelRxImpl, ChannelTxImpl>();
        }

        #[cfg(not(esp32c2))]
        #[handler(priority = crate::interrupt::Priority::max())]
        pub(crate) fn interrupt_handler_ch2() {
            use crate::dma::gdma::{
                Channel2 as Channel,
                Channel2RxImpl as ChannelRxImpl,
                Channel2TxImpl as ChannelTxImpl,
            };

            handle_interrupt::<Channel, ChannelRxImpl, ChannelTxImpl>();
        }

        #[cfg(esp32s3)]
        #[handler(priority = crate::interrupt::Priority::max())]
        pub(crate) fn interrupt_handler_ch3() {
            use crate::dma::gdma::{
                Channel3 as Channel,
                Channel3RxImpl as ChannelRxImpl,
                Channel3TxImpl as ChannelTxImpl,
            };

            handle_interrupt::<Channel, ChannelRxImpl, ChannelTxImpl>();
        }

        #[cfg(esp32s3)]
        #[handler(priority = crate::interrupt::Priority::max())]
        pub(crate) fn interrupt_handler_ch4() {
            use crate::dma::gdma::{
                Channel4 as Channel,
                Channel4RxImpl as ChannelRxImpl,
                Channel4TxImpl as ChannelTxImpl,
            };

            handle_interrupt::<Channel, ChannelRxImpl, ChannelTxImpl>();
        }
    }

    #[cfg(any(esp32, esp32s2))]
    pub(crate) mod interrupt {
        use procmacros::handler;

        use super::*;

        #[handler(priority = crate::interrupt::Priority::max())]
        pub(crate) fn interrupt_handler_spi2_dma() {
            use crate::dma::pdma::{
                Spi2DmaChannel as Channel,
                Spi2DmaChannelRxImpl as ChannelRxImpl,
                Spi2DmaChannelTxImpl as ChannelTxImpl,
            };

            handle_interrupt::<Channel, ChannelRxImpl, ChannelTxImpl>();
        }

        #[handler(priority = crate::interrupt::Priority::max())]
        pub(crate) fn interrupt_handler_spi3_dma() {
            use crate::dma::pdma::{
                Spi3DmaChannel as Channel,
                Spi3DmaChannelRxImpl as ChannelRxImpl,
                Spi3DmaChannelTxImpl as ChannelTxImpl,
            };

            handle_interrupt::<Channel, ChannelRxImpl, ChannelTxImpl>();
        }

        #[handler(priority = crate::interrupt::Priority::max())]
        pub(crate) fn interrupt_handler_i2s0() {
            use crate::dma::pdma::{
                I2s0DmaChannel as Channel,
                I2s0DmaChannelRxImpl as ChannelRxImpl,
                I2s0DmaChannelTxImpl as ChannelTxImpl,
            };

            handle_interrupt::<Channel, ChannelRxImpl, ChannelTxImpl>();
        }

        #[cfg(i2s1)]
        #[handler(priority = crate::interrupt::Priority::max())]
        pub(crate) fn interrupt_handler_i2s1() {
            use crate::dma::pdma::{
                I2s1DmaChannel as Channel,
                I2s1DmaChannelRxImpl as ChannelRxImpl,
                I2s1DmaChannelTxImpl as ChannelTxImpl,
            };

            handle_interrupt::<Channel, ChannelRxImpl, ChannelTxImpl>();
        }
    }
}
