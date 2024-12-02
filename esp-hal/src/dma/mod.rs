//! # Direct Memory Access (DMA)
//!
//! ## Overview
//!
//! The DMA driver provides an interface to efficiently transfer data between
//! different memory regions and peripherals within the ESP microcontroller
//! without involving the CPU. The DMA controller is responsible for managing
//! these data transfers.
//!
//! Notice, that this module is a common version of the DMA driver, `ESP32` and
//! `ESP32-S2` are using older `PDMA` controller, whenever other chips are using
//! newer `GDMA` controller.
//!
//! ## Examples
//!
//! ### Initialize and utilize DMA controller in `SPI`
//!
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::dma_buffers;
//! # use esp_hal::spi::{master::{Config, Spi}, SpiMode};
#![cfg_attr(pdma, doc = "let dma_channel = peripherals.DMA_SPI2;")]
#![cfg_attr(gdma, doc = "let dma_channel = peripherals.DMA_CH0;")]
//! let sclk = peripherals.GPIO0;
//! let miso = peripherals.GPIO2;
//! let mosi = peripherals.GPIO4;
//! let cs = peripherals.GPIO5;
//!
//! let mut spi = Spi::new(
//!     peripherals.SPI2,
//!     Config::default().with_frequency(100.kHz()).with_mode(SpiMode::Mode0)
//! )
//! .unwrap()
//! .with_sck(sclk)
//! .with_mosi(mosi)
//! .with_miso(miso)
//! .with_cs(cs)
//! .with_dma(dma_channel);
//! # }
//! ```
//! 
//! ⚠️ Note: Descriptors should be sized as `(max_transfer_size + CHUNK_SIZE - 1) / CHUNK_SIZE`.
//! I.e., to transfer buffers of size `1..=CHUNK_SIZE`, you need 1 descriptor.
//!
//! ⚠️ Note: For chips that support DMA to/from PSRAM (ESP32-S3) DMA transfers to/from PSRAM
//! have extra alignment requirements. The address and size of the buffer pointed to by
//! each descriptor must be a multiple of the cache line (block) size. This is 32 bytes
//! on ESP32-S3.
//!
//! For convenience you can use the [crate::dma_buffers] macro.

use core::{cmp::min, fmt::Debug, marker::PhantomData, sync::atomic::compiler_fence};

use enumset::{EnumSet, EnumSetType};

pub use self::buffers::*;
#[cfg(gdma)]
pub use self::gdma::*;
#[cfg(gdma)]
pub use self::m2m::*;
#[cfg(pdma)]
pub use self::pdma::*;
use crate::{
    interrupt::InterruptHandler,
    peripheral::{Peripheral, PeripheralRef},
    peripherals::Interrupt,
    soc::{is_slice_in_dram, is_valid_memory_address, is_valid_ram_address},
    system,
    Async,
    Blocking,
    Cpu,
    Mode,
};

trait Word: crate::private::Sealed {}

macro_rules! impl_word {
    ($w:ty) => {
        impl $crate::private::Sealed for $w {}
        impl Word for $w {}
    };
}

impl_word!(u8);
impl_word!(u16);
impl_word!(u32);
impl_word!(i8);
impl_word!(i16);
impl_word!(i32);

impl<W, const S: usize> crate::private::Sealed for [W; S] where W: Word {}

impl<W, const S: usize> crate::private::Sealed for &[W; S] where W: Word {}

impl<W> crate::private::Sealed for &[W] where W: Word {}

impl<W> crate::private::Sealed for &mut [W] where W: Word {}

/// Trait for buffers that can be given to DMA for reading.
///
/// # Safety
///
/// Once the `read_buffer` method has been called, it is unsafe to call any
/// `&mut self` methods on this object as long as the returned value is in use
/// (by DMA).
pub unsafe trait ReadBuffer {
    /// Provide a buffer usable for DMA reads.
    ///
    /// The return value is:
    ///
    /// - pointer to the start of the buffer
    /// - buffer size in bytes
    ///
    /// # Safety
    ///
    /// Once this method has been called, it is unsafe to call any `&mut self`
    /// methods on this object as long as the returned value is in use (by DMA).
    unsafe fn read_buffer(&self) -> (*const u8, usize);
}

unsafe impl<W, const S: usize> ReadBuffer for [W; S]
where
    W: Word,
{
    unsafe fn read_buffer(&self) -> (*const u8, usize) {
        (self.as_ptr() as *const u8, core::mem::size_of_val(self))
    }
}

unsafe impl<W, const S: usize> ReadBuffer for &[W; S]
where
    W: Word,
{
    unsafe fn read_buffer(&self) -> (*const u8, usize) {
        (self.as_ptr() as *const u8, core::mem::size_of_val(*self))
    }
}

unsafe impl<W, const S: usize> ReadBuffer for &mut [W; S]
where
    W: Word,
{
    unsafe fn read_buffer(&self) -> (*const u8, usize) {
        (self.as_ptr() as *const u8, core::mem::size_of_val(*self))
    }
}

unsafe impl<W> ReadBuffer for &[W]
where
    W: Word,
{
    unsafe fn read_buffer(&self) -> (*const u8, usize) {
        (self.as_ptr() as *const u8, core::mem::size_of_val(*self))
    }
}

unsafe impl<W> ReadBuffer for &mut [W]
where
    W: Word,
{
    unsafe fn read_buffer(&self) -> (*const u8, usize) {
        (self.as_ptr() as *const u8, core::mem::size_of_val(*self))
    }
}

/// Trait for buffers that can be given to DMA for writing.
///
/// # Safety
///
/// Once the `write_buffer` method has been called, it is unsafe to call any
/// `&mut self` methods, except for `write_buffer`, on this object as long as
/// the returned value is in use (by DMA).
pub unsafe trait WriteBuffer {
    /// Provide a buffer usable for DMA writes.
    ///
    /// The return value is:
    ///
    /// - pointer to the start of the buffer
    /// - buffer size in bytes
    ///
    /// # Safety
    ///
    /// Once this method has been called, it is unsafe to call any `&mut self`
    /// methods, except for `write_buffer`, on this object as long as the
    /// returned value is in use (by DMA).
    unsafe fn write_buffer(&mut self) -> (*mut u8, usize);
}

unsafe impl<W, const S: usize> WriteBuffer for [W; S]
where
    W: Word,
{
    unsafe fn write_buffer(&mut self) -> (*mut u8, usize) {
        (self.as_mut_ptr() as *mut u8, core::mem::size_of_val(self))
    }
}

unsafe impl<W, const S: usize> WriteBuffer for &mut [W; S]
where
    W: Word,
{
    unsafe fn write_buffer(&mut self) -> (*mut u8, usize) {
        (self.as_mut_ptr() as *mut u8, core::mem::size_of_val(*self))
    }
}

unsafe impl<W> WriteBuffer for &mut [W]
where
    W: Word,
{
    unsafe fn write_buffer(&mut self) -> (*mut u8, usize) {
        (self.as_mut_ptr() as *mut u8, core::mem::size_of_val(*self))
    }
}

bitfield::bitfield! {
    /// DMA descriptor flags.
    #[derive(Clone, Copy)]
    pub struct DmaDescriptorFlags(u32);

    u16;

    /// Specifies the size of the buffer that this descriptor points to.
    pub size, set_size: 11, 0;

    /// Specifies the number of valid bytes in the buffer that this descriptor points to.
    ///
    /// This field in a transmit descriptor is written by software and indicates how many bytes can
    /// be read from the buffer.
    ///
    /// This field in a receive descriptor is written by hardware automatically and indicates how
    /// many valid bytes have been stored into the buffer.
    pub length, set_length: 23, 12;

    /// For receive descriptors, software needs to clear this bit to 0, and hardware will set it to 1 after receiving
    /// data containing the EOF flag.
    /// For transmit descriptors, software needs to set this bit to 1 as needed.
    /// If software configures this bit to 1 in a descriptor, the DMA will include the EOF flag in the data sent to
    /// the corresponding peripheral, indicating to the peripheral that this data segment marks the end of one
    /// transfer phase.
    pub suc_eof, set_suc_eof: 30;

    /// Specifies who is allowed to access the buffer that this descriptor points to.
    /// - 0: CPU can access the buffer;
    /// - 1: The GDMA controller can access the buffer.
    pub owner, set_owner: 31;
}

impl Debug for DmaDescriptorFlags {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("DmaDescriptorFlags")
            .field("size", &self.size())
            .field("length", &self.length())
            .field("suc_eof", &self.suc_eof())
            .field("owner", &(if self.owner() { "DMA" } else { "CPU" }))
            .finish()
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for DmaDescriptorFlags {
    fn format(&self, fmt: defmt::Formatter<'_>) {
        defmt::write!(
            fmt,
            "DmaDescriptorFlags {{ size: {}, length: {}, suc_eof: {}, owner: {} }}",
            self.size(),
            self.length(),
            self.suc_eof(),
            if self.owner() { "DMA" } else { "CPU" }
        );
    }
}

/// A DMA transfer descriptor.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DmaDescriptor {
    /// Descriptor flags.
    pub flags: DmaDescriptorFlags,

    /// Address of the buffer.
    pub buffer: *mut u8,

    /// Address of the next descriptor.
    /// If the current descriptor is the last one, this value is 0.
    /// This field can only point to internal RAM.
    pub next: *mut DmaDescriptor,
}

impl DmaDescriptor {
    /// An empty DMA descriptor used to initialize the descriptor list.
    pub const EMPTY: Self = Self {
        flags: DmaDescriptorFlags(0),
        buffer: core::ptr::null_mut(),
        next: core::ptr::null_mut(),
    };

    /// Resets the descriptor for a new receive transfer.
    pub fn reset_for_rx(&mut self) {
        // Give ownership to the DMA
        self.set_owner(Owner::Dma);

        // Clear this to allow hardware to set it when the peripheral returns an EOF
        // bit.
        self.set_suc_eof(false);

        // Clear this to allow hardware to set it when it's
        // done receiving data for this descriptor.
        self.set_length(0);
    }

    /// Resets the descriptor for a new transmit transfer. See
    /// [DmaDescriptorFlags::suc_eof] for more details on the `set_eof`
    /// parameter.
    pub fn reset_for_tx(&mut self, set_eof: bool) {
        // Give ownership to the DMA
        self.set_owner(Owner::Dma);

        // The `suc_eof` bit doesn't affect the transfer itself, but signals when the
        // hardware should trigger an interrupt request.
        self.set_suc_eof(set_eof);
    }

    /// Set the size of the buffer. See [DmaDescriptorFlags::size].
    pub fn set_size(&mut self, len: usize) {
        self.flags.set_size(len as u16)
    }

    /// Set the length of the descriptor. See [DmaDescriptorFlags::length].
    pub fn set_length(&mut self, len: usize) {
        self.flags.set_length(len as u16)
    }

    /// Returns the size of the buffer. See [DmaDescriptorFlags::size].
    pub fn size(&self) -> usize {
        self.flags.size() as usize
    }

    /// Returns the length of the descriptor. See [DmaDescriptorFlags::length].
    #[allow(clippy::len_without_is_empty)]
    pub fn len(&self) -> usize {
        self.flags.length() as usize
    }

    /// Set the suc_eof bit. See [DmaDescriptorFlags::suc_eof].
    pub fn set_suc_eof(&mut self, suc_eof: bool) {
        self.flags.set_suc_eof(suc_eof)
    }

    /// Set the owner. See [DmaDescriptorFlags::owner].
    pub fn set_owner(&mut self, owner: Owner) {
        let owner = match owner {
            Owner::Cpu => false,
            Owner::Dma => true,
        };
        self.flags.set_owner(owner)
    }

    /// Returns the owner. See [DmaDescriptorFlags::owner].
    pub fn owner(&self) -> Owner {
        match self.flags.owner() {
            false => Owner::Cpu,
            true => Owner::Dma,
        }
    }
}

// The pointers in the descriptor can be Sent.
// Marking this Send also allows DmaBuffer implementations to automatically be
// Send (where the compiler sees fit).
unsafe impl Send for DmaDescriptor {}

mod buffers;
#[cfg(gdma)]
mod gdma;
#[cfg(gdma)]
mod m2m;
#[cfg(pdma)]
mod pdma;

/// Kinds of interrupt to listen to.
#[derive(Debug, EnumSetType)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DmaInterrupt {
    /// RX is done
    RxDone,
    /// TX is done
    TxDone,
}

/// Types of interrupts emitted by the TX channel.
#[derive(Debug, EnumSetType)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DmaTxInterrupt {
    /// Triggered when all data corresponding to a linked list (including
    /// multiple descriptors) have been sent via transmit channel.
    TotalEof,

    /// Triggered when an error is detected in a transmit descriptor on transmit
    /// channel.
    DescriptorError,

    /// Triggered when EOF in a transmit descriptor is true and data
    /// corresponding to this descriptor have been sent via transmit
    /// channel.
    Eof,

    /// Triggered when all data corresponding to a transmit descriptor have been
    /// sent via transmit channel.
    Done,
}

/// Types of interrupts emitted by the RX channel.
#[derive(Debug, EnumSetType)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DmaRxInterrupt {
    /// Triggered when the size of the buffer pointed by receive descriptors
    /// is smaller than the length of data to be received via receive channel.
    DescriptorEmpty,

    /// Triggered when an error is detected in a receive descriptor on receive
    /// channel.
    DescriptorError,

    /// Triggered when an error is detected in the data segment corresponding to
    /// a descriptor received via receive channel n.
    /// This interrupt is used only for UHCI0 peripheral (UART0 or UART1).
    ErrorEof,

    /// Triggered when the suc_eof bit in a receive descriptor is 1 and the data
    /// corresponding to this receive descriptor has been received via receive
    /// channel.
    SuccessfulEof,

    /// Triggered when all data corresponding to a receive descriptor have been
    /// received via receive channel.
    Done,
}

/// The default chunk size used for DMA transfers.
pub const CHUNK_SIZE: usize = 4092;

/// Convenience macro to create DMA buffers and descriptors.
///
/// ## Usage
/// ```rust,no_run
#[doc = crate::before_snippet!()]
/// use esp_hal::dma_buffers;
///
/// // RX and TX buffers are 32000 bytes - passing only one parameter makes RX
/// // and TX the same size.
/// let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) =
///     dma_buffers!(32000, 32000);
/// # }
/// ```
#[macro_export]
macro_rules! dma_buffers {
    ($rx_size:expr, $tx_size:expr) => {
        $crate::dma_buffers_chunk_size!($rx_size, $tx_size, $crate::dma::CHUNK_SIZE)
    };
    ($size:expr) => {
        $crate::dma_buffers_chunk_size!($size, $crate::dma::CHUNK_SIZE)
    };
}

/// Convenience macro to create circular DMA buffers and descriptors.
///
/// ## Usage
/// ```rust,no_run
#[doc = crate::before_snippet!()]
/// use esp_hal::dma_circular_buffers;
///
/// // RX and TX buffers are 32000 bytes - passing only one parameter makes RX
/// // and TX the same size.
/// let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) =
///     dma_circular_buffers!(32000, 32000);
/// # }
/// ```
#[macro_export]
macro_rules! dma_circular_buffers {
    ($rx_size:expr, $tx_size:expr) => {
        $crate::dma_circular_buffers_chunk_size!($rx_size, $tx_size, $crate::dma::CHUNK_SIZE)
    };

    ($size:expr) => {
        $crate::dma_circular_buffers_chunk_size!($size, $size, $crate::dma::CHUNK_SIZE)
    };
}

/// Convenience macro to create DMA descriptors.
///
/// ## Usage
/// ```rust,no_run
#[doc = crate::before_snippet!()]
/// use esp_hal::dma_descriptors;
///
/// // Create RX and TX descriptors for transactions up to 32000 bytes - passing
/// // only one parameter assumes RX and TX are the same size.
/// let (rx_descriptors, tx_descriptors) = dma_descriptors!(32000, 32000);
/// # }
/// ```
#[macro_export]
macro_rules! dma_descriptors {
    ($rx_size:expr, $tx_size:expr) => {
        $crate::dma_descriptors_chunk_size!($rx_size, $tx_size, $crate::dma::CHUNK_SIZE)
    };

    ($size:expr) => {
        $crate::dma_descriptors_chunk_size!($size, $size, $crate::dma::CHUNK_SIZE)
    };
}

/// Convenience macro to create circular DMA descriptors.
///
/// ## Usage
/// ```rust,no_run
#[doc = crate::before_snippet!()]
/// use esp_hal::dma_circular_descriptors;
///
/// // Create RX and TX descriptors for transactions up to 32000
/// // bytes - passing only one parameter assumes RX and TX are the same size.
/// let (rx_descriptors, tx_descriptors) =
///     dma_circular_descriptors!(32000, 32000);
/// # }
/// ```
#[macro_export]
macro_rules! dma_circular_descriptors {
    ($rx_size:expr, $tx_size:expr) => {
        $crate::dma_circular_descriptors_chunk_size!($rx_size, $tx_size, $crate::dma::CHUNK_SIZE)
    };

    ($size:expr) => {
        $crate::dma_circular_descriptors_chunk_size!($size, $size, $crate::dma::CHUNK_SIZE)
    };
}

/// Declares a DMA buffer with a specific size, aligned to 4 bytes
#[doc(hidden)]
#[macro_export]
macro_rules! declare_aligned_dma_buffer {
    ($name:ident, $size:expr) => {
        // ESP32 requires word alignment for DMA buffers.
        // ESP32-S2 technically supports byte-aligned DMA buffers, but the
        // transfer ends up writing out of bounds.
        // if the buffer's length is 2 or 3 (mod 4).
        static mut $name: [u32; ($size + 3) / 4] = [0; ($size + 3) / 4];
    };
}

/// Turns the potentially oversized static `u32`` array reference into a
/// correctly sized `u8` one
#[doc(hidden)]
#[macro_export]
macro_rules! as_mut_byte_array {
    ($name:expr, $size:expr) => {
        unsafe { &mut *($name.as_mut_ptr() as *mut [u8; $size]) }
    };
}

/// Convenience macro to create DMA buffers and descriptors with specific chunk
/// size.
///
/// ## Usage
/// ```rust,no_run
#[doc = crate::before_snippet!()]
/// use esp_hal::dma_buffers_chunk_size;
///
/// // TX and RX buffers are 32000 bytes - passing only one parameter makes TX
/// // and RX the same size.
/// let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) =
///     dma_buffers_chunk_size!(32000, 32000, 4032);
/// # }
/// ```
#[macro_export]
macro_rules! dma_buffers_chunk_size {
    ($rx_size:expr, $tx_size:expr, $chunk_size:expr) => {{
        $crate::dma_buffers_impl!($rx_size, $tx_size, $chunk_size, is_circular = false)
    }};

    ($size:expr, $chunk_size:expr) => {
        $crate::dma_buffers_chunk_size!($size, $size, $chunk_size)
    };
}

/// Convenience macro to create circular DMA buffers and descriptors with
/// specific chunk size.
///
/// ## Usage
/// ```rust,no_run
#[doc = crate::before_snippet!()]
/// use esp_hal::dma_circular_buffers_chunk_size;
///
/// // RX and TX buffers are 32000 bytes - passing only one parameter makes RX
/// // and TX the same size.
/// let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) =
///     dma_circular_buffers_chunk_size!(32000, 32000, 4032);
/// # }
/// ```
#[macro_export]
macro_rules! dma_circular_buffers_chunk_size {
    ($rx_size:expr, $tx_size:expr, $chunk_size:expr) => {{
        $crate::dma_buffers_impl!($rx_size, $tx_size, $chunk_size, is_circular = true)
    }};

    ($size:expr, $chunk_size:expr) => {{
        $crate::dma_circular_buffers_chunk_size!($size, $size, $chunk_size)
    }};
}

/// Convenience macro to create DMA descriptors with specific chunk size
///
/// ## Usage
/// ```rust,no_run
#[doc = crate::before_snippet!()]
/// use esp_hal::dma_descriptors_chunk_size;
///
/// // Create RX and TX descriptors for transactions up to 32000 bytes - passing
/// // only one parameter assumes RX and TX are the same size.
/// let (rx_descriptors, tx_descriptors) =
///     dma_descriptors_chunk_size!(32000, 32000, 4032);
/// # }
/// ```
#[macro_export]
macro_rules! dma_descriptors_chunk_size {
    ($rx_size:expr, $tx_size:expr, $chunk_size:expr) => {{
        $crate::dma_descriptors_impl!($rx_size, $tx_size, $chunk_size, is_circular = false)
    }};

    ($size:expr, $chunk_size:expr) => {
        $crate::dma_descriptors_chunk_size!($size, $size, $chunk_size)
    };
}

/// Convenience macro to create circular DMA descriptors with specific chunk
/// size
///
/// ## Usage
/// ```rust,no_run
#[doc = crate::before_snippet!()]
/// use esp_hal::dma_circular_descriptors_chunk_size;
///
/// // Create RX and TX descriptors for transactions up to 32000 bytes - passing
/// // only one parameter assumes RX and TX are the same size.
/// let (rx_descriptors, tx_descriptors) =
///     dma_circular_descriptors_chunk_size!(32000, 32000, 4032);
/// # }
/// ```
#[macro_export]
macro_rules! dma_circular_descriptors_chunk_size {
    ($rx_size:expr, $tx_size:expr, $chunk_size:expr) => {{
        $crate::dma_descriptors_impl!($rx_size, $tx_size, $chunk_size, is_circular = true)
    }};

    ($size:expr, $chunk_size:expr) => {
        $crate::dma_circular_descriptors_chunk_size!($size, $size, $chunk_size)
    };
}

#[doc(hidden)]
#[macro_export]
macro_rules! dma_buffers_impl {
    ($rx_size:expr, $tx_size:expr, $chunk_size:expr, is_circular = $circular:tt) => {{
        let rx = $crate::dma_buffers_impl!($rx_size, $chunk_size, is_circular = $circular);
        let tx = $crate::dma_buffers_impl!($tx_size, $chunk_size, is_circular = $circular);
        (rx.0, rx.1, tx.0, tx.1)
    }};

    ($size:expr, $chunk_size:expr, is_circular = $circular:tt) => {{
        $crate::declare_aligned_dma_buffer!(BUFFER, $size);

        unsafe {
            (
                $crate::as_mut_byte_array!(BUFFER, $size),
                $crate::dma_descriptors_impl!($size, $chunk_size, is_circular = $circular),
            )
        }
    }};

    ($size:expr, is_circular = $circular:tt) => {
        $crate::dma_buffers_impl!(
            $size,
            $crate::dma::BurstConfig::DEFAULT.max_compatible_chunk_size(),
            is_circular = $circular
        );
    };
}

#[doc(hidden)]
#[macro_export]
macro_rules! dma_descriptors_impl {
    ($rx_size:expr, $tx_size:expr, $chunk_size:expr, is_circular = $circular:tt) => {{
        let rx = $crate::dma_descriptors_impl!($rx_size, $chunk_size, is_circular = $circular);
        let tx = $crate::dma_descriptors_impl!($tx_size, $chunk_size, is_circular = $circular);
        (rx, tx)
    }};

    ($size:expr, $chunk_size:expr, is_circular = $circular:tt) => {{
        const COUNT: usize =
            $crate::dma_descriptor_count!($size, $chunk_size, is_circular = $circular);

        static mut DESCRIPTORS: [$crate::dma::DmaDescriptor; COUNT] =
            [$crate::dma::DmaDescriptor::EMPTY; COUNT];

        unsafe { &mut DESCRIPTORS }
    }};
}

#[doc(hidden)]
#[macro_export]
macro_rules! dma_descriptor_count {
    ($size:expr, $chunk_size:expr, is_circular = $is_circular:tt) => {{
        const {
            ::core::assert!($chunk_size <= 4095, "chunk size must be <= 4095");
            ::core::assert!($chunk_size > 0, "chunk size must be > 0");
        }

        // We allow 0 in the macros as a "not needed" case.
        if $size == 0 {
            0
        } else {
            $crate::dma::descriptor_count($size, $chunk_size, $is_circular)
        }
    }};
}

/// Convenience macro to create a DmaTxBuf from buffer size. The buffer and
/// descriptors are statically allocated and used to create the `DmaTxBuf`.
///
/// ## Usage
/// ```rust,no_run
#[doc = crate::before_snippet!()]
/// use esp_hal::dma_tx_buffer;
///
/// let tx_buf = dma_tx_buffer!(32000);
/// # }
/// ```
#[macro_export]
macro_rules! dma_tx_buffer {
    ($tx_size:expr) => {{
        let (tx_buffer, tx_descriptors) = $crate::dma_buffers_impl!($tx_size, is_circular = false);

        $crate::dma::DmaTxBuf::new(tx_descriptors, tx_buffer)
    }};
}

/// Convenience macro to create a [DmaRxStreamBuf] from buffer size and
/// optional chunk size (uses max if unspecified).
/// The buffer and descriptors are statically allocated and
/// used to create the [DmaRxStreamBuf].
///
/// Smaller chunk sizes are recommended for lower latency.
///
/// ## Usage
/// ```rust,no_run
#[doc = crate::before_snippet!()]
/// use esp_hal::dma_rx_stream_buffer;
///
/// let buf = dma_rx_stream_buffer!(32000);
/// let buf = dma_rx_stream_buffer!(32000, 1000);
/// # }
/// ```
#[macro_export]
macro_rules! dma_rx_stream_buffer {
    ($rx_size:expr) => {
        $crate::dma_rx_stream_buffer!($rx_size, 4095)
    };
    ($rx_size:expr, $chunk_size:expr) => {{
        let (buffer, descriptors) =
            $crate::dma_buffers_impl!($rx_size, $chunk_size, is_circular = false);

        $crate::dma::DmaRxStreamBuf::new(descriptors, buffer).unwrap()
    }};
}

/// Convenience macro to create a [DmaLoopBuf] from a buffer size.
///
/// ## Usage
/// ```rust,no_run
#[doc = crate::before_snippet!()]
/// use esp_hal::dma_loop_buffer;
///
/// let buf = dma_loop_buffer!(2000);
/// # }
/// ```
#[macro_export]
macro_rules! dma_loop_buffer {
    ($size:expr) => {{
        const {
            ::core::assert!($size <= 4095, "size must be <= 4095");
            ::core::assert!($size > 0, "size must be > 0");
        }

        let (buffer, descriptors) = $crate::dma_buffers_impl!($size, $size, is_circular = false);

        $crate::dma::DmaLoopBuf::new(&mut descriptors[0], buffer).unwrap()
    }};
}

/// DMA Errors
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DmaError {
    /// The alignment of data is invalid
    InvalidAlignment(DmaAlignmentError),
    /// More descriptors are needed for the buffer size
    OutOfDescriptors,
    /// DescriptorError the DMA rejected the descriptor configuration. This
    /// could be because the source address of the data is not in RAM. Ensure
    /// your source data is in a valid address space.
    DescriptorError,
    /// The available free buffer is less than the amount of data to push
    Overflow,
    /// The given buffer is too small
    BufferTooSmall,
    /// Descriptors or buffers are not located in a supported memory region
    UnsupportedMemoryRegion,
    /// Invalid DMA chunk size
    InvalidChunkSize,
    /// Indicates writing to or reading from a circular DMA transaction is done
    /// too late and the DMA buffers already overrun / underrun.
    Late,
}

impl From<DmaBufError> for DmaError {
    fn from(error: DmaBufError) -> Self {
        // FIXME: use nested errors
        match error {
            DmaBufError::InsufficientDescriptors => DmaError::OutOfDescriptors,
            DmaBufError::UnsupportedMemoryRegion => DmaError::UnsupportedMemoryRegion,
            DmaBufError::InvalidAlignment(err) => DmaError::InvalidAlignment(err),
            DmaBufError::InvalidChunkSize => DmaError::InvalidChunkSize,
        }
    }
}

/// DMA Priorities
#[cfg(gdma)]
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DmaPriority {
    /// The lowest priority level (Priority 0).
    Priority0 = 0,
    /// Priority level 1.
    Priority1 = 1,
    /// Priority level 2.
    Priority2 = 2,
    /// Priority level 3.
    Priority3 = 3,
    /// Priority level 4.
    Priority4 = 4,
    /// Priority level 5.
    Priority5 = 5,
    /// Priority level 6.
    Priority6 = 6,
    /// Priority level 7.
    Priority7 = 7,
    /// Priority level 8.
    Priority8 = 8,
    /// The highest priority level (Priority 9).
    Priority9 = 9,
}

/// DMA Priorities
/// The values need to match the TRM
#[cfg(pdma)]
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DmaPriority {
    /// The lowest priority level (Priority 0).
    Priority0 = 0,
}

/// DMA capable peripherals
/// The values need to match the TRM
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[doc(hidden)]
pub enum DmaPeripheral {
    Spi2      = 0,
    #[cfg(any(pdma, esp32s3))]
    Spi3      = 1,
    #[cfg(any(esp32c2, esp32c6, esp32h2))]
    Mem2Mem1  = 1,
    #[cfg(any(esp32c3, esp32c6, esp32h2, esp32s3))]
    Uhci0     = 2,
    #[cfg(any(esp32, esp32s2, esp32c3, esp32c6, esp32h2, esp32s3))]
    I2s0      = 3,
    #[cfg(any(esp32, esp32s3))]
    I2s1      = 4,
    #[cfg(any(esp32c6, esp32h2))]
    Mem2Mem4  = 4,
    #[cfg(esp32s3)]
    LcdCam    = 5,
    #[cfg(any(esp32c6, esp32h2))]
    Mem2Mem5  = 5,
    #[cfg(not(esp32c2))]
    Aes       = 6,
    #[cfg(gdma)]
    Sha       = 7,
    #[cfg(any(esp32c3, esp32c6, esp32h2, esp32s3))]
    Adc       = 8,
    #[cfg(esp32s3)]
    Rmt       = 9,
    #[cfg(parl_io)]
    ParlIo    = 9,
    #[cfg(any(esp32c6, esp32h2))]
    Mem2Mem10 = 10,
    #[cfg(any(esp32c6, esp32h2))]
    Mem2Mem11 = 11,
    #[cfg(any(esp32c6, esp32h2))]
    Mem2Mem12 = 12,
    #[cfg(any(esp32c6, esp32h2))]
    Mem2Mem13 = 13,
    #[cfg(any(esp32c6, esp32h2))]
    Mem2Mem14 = 14,
    #[cfg(any(esp32c6, esp32h2))]
    Mem2Mem15 = 15,
}

/// The owner bit of a DMA descriptor.
#[derive(PartialEq, PartialOrd)]
pub enum Owner {
    /// Owned by CPU
    Cpu = 0,
    /// Owned by DMA
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

#[doc(hidden)]
pub trait DmaEligible {
    /// The most specific DMA channel type usable by this peripheral.
    type Dma: DmaChannel;

    fn dma_peripheral(&self) -> DmaPeripheral;
}

#[doc(hidden)]
#[derive(Debug)]
pub struct DescriptorChain {
    pub(crate) descriptors: &'static mut [DmaDescriptor],
    chunk_size: usize,
}

impl DescriptorChain {
    pub fn new(descriptors: &'static mut [DmaDescriptor]) -> Self {
        Self::new_with_chunk_size(descriptors, CHUNK_SIZE)
    }

    pub fn new_with_chunk_size(
        descriptors: &'static mut [DmaDescriptor],
        chunk_size: usize,
    ) -> Self {
        Self {
            descriptors,
            chunk_size,
        }
    }

    pub fn first_mut(&mut self) -> *mut DmaDescriptor {
        self.descriptors.as_mut_ptr()
    }

    pub fn first(&self) -> *const DmaDescriptor {
        self.descriptors.as_ptr()
    }

    pub fn last_mut(&mut self) -> *mut DmaDescriptor {
        self.descriptors.last_mut().unwrap()
    }

    pub fn last(&self) -> *const DmaDescriptor {
        self.descriptors.last().unwrap()
    }

    #[allow(clippy::not_unsafe_ptr_arg_deref)]
    pub fn fill_for_rx(
        &mut self,
        circular: bool,
        data: *mut u8,
        len: usize,
    ) -> Result<(), DmaError> {
        self.fill(circular, data, len, |desc, _| {
            desc.reset_for_rx();
            // Descriptor::size has been set up by `fill`
        })
    }

    #[allow(clippy::not_unsafe_ptr_arg_deref)]
    pub fn fill_for_tx(
        &mut self,
        is_circular: bool,
        data: *const u8,
        len: usize,
    ) -> Result<(), DmaError> {
        self.fill(is_circular, data.cast_mut(), len, |desc, chunk_size| {
            // In circular mode, we set the `suc_eof` bit for every buffer we send. We use
            // this for I2S to track progress of a transfer by checking OUTLINK_DSCR_ADDR.
            // In non-circular mode, we only set `suc_eof` for the last descriptor to signal
            // the end of the transfer.
            desc.reset_for_tx(desc.next.is_null() || is_circular);
            desc.set_length(chunk_size); // align to 32 bits?
        })
    }

    #[allow(clippy::not_unsafe_ptr_arg_deref)]
    pub fn fill(
        &mut self,
        circular: bool,
        data: *mut u8,
        len: usize,
        prepare_descriptor: impl Fn(&mut DmaDescriptor, usize),
    ) -> Result<(), DmaError> {
        if !is_valid_ram_address(self.first() as usize)
            || !is_valid_ram_address(self.last() as usize)
            || !is_valid_memory_address(data as usize)
            || !is_valid_memory_address(unsafe { data.add(len) } as usize)
        {
            return Err(DmaError::UnsupportedMemoryRegion);
        }

        let max_chunk_size = if circular && len <= self.chunk_size * 2 {
            if len <= 3 {
                return Err(DmaError::BufferTooSmall);
            }
            len / 3 + len % 3
        } else {
            self.chunk_size
        };

        DescriptorSet::set_up_buffer_ptrs(
            unsafe { core::slice::from_raw_parts_mut(data, len) },
            self.descriptors,
            max_chunk_size,
            circular,
        )?;
        DescriptorSet::set_up_descriptors(
            self.descriptors,
            len,
            max_chunk_size,
            circular,
            prepare_descriptor,
        )?;

        Ok(())
    }
}

/// Computes the number of descriptors required for a given buffer size with
/// a given chunk size.
pub const fn descriptor_count(buffer_size: usize, chunk_size: usize, is_circular: bool) -> usize {
    if is_circular && buffer_size <= chunk_size * 2 {
        return 3;
    }

    if buffer_size < chunk_size {
        // At least one descriptor is always required.
        return 1;
    }

    buffer_size.div_ceil(chunk_size)
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct DescriptorSet<'a> {
    descriptors: &'a mut [DmaDescriptor],
}

impl<'a> DescriptorSet<'a> {
    /// Creates a new `DescriptorSet` from a slice of descriptors and associates
    /// them with the given buffer.
    fn new(descriptors: &'a mut [DmaDescriptor]) -> Result<Self, DmaBufError> {
        if !is_slice_in_dram(descriptors) {
            return Err(DmaBufError::UnsupportedMemoryRegion);
        }

        descriptors.fill(DmaDescriptor::EMPTY);

        Ok(unsafe { Self::new_unchecked(descriptors) })
    }

    /// Creates a new `DescriptorSet` from a slice of descriptors and associates
    /// them with the given buffer.
    ///
    /// # Safety
    ///
    /// The caller must ensure that the descriptors are located in a supported
    /// memory region.
    unsafe fn new_unchecked(descriptors: &'a mut [DmaDescriptor]) -> Self {
        Self { descriptors }
    }

    /// Consumes the `DescriptorSet` and returns the inner slice of descriptors.
    fn into_inner(self) -> &'a mut [DmaDescriptor] {
        self.descriptors
    }

    /// Returns a pointer to the first descriptor in the chain.
    fn head(&mut self) -> *mut DmaDescriptor {
        self.descriptors.as_mut_ptr()
    }

    /// Returns an iterator over the linked descriptors.
    fn linked_iter(&self) -> impl Iterator<Item = &DmaDescriptor> {
        let mut was_last = false;
        self.descriptors.iter().take_while(move |d| {
            if was_last {
                false
            } else {
                was_last = d.next.is_null();
                true
            }
        })
    }

    /// Returns an iterator over the linked descriptors.
    fn linked_iter_mut(&mut self) -> impl Iterator<Item = &mut DmaDescriptor> {
        let mut was_last = false;
        self.descriptors.iter_mut().take_while(move |d| {
            if was_last {
                false
            } else {
                was_last = d.next.is_null();
                true
            }
        })
    }

    /// Associate each descriptor with a chunk of the buffer.
    ///
    /// This function checks the alignment and location of the buffer.
    ///
    /// See [`Self::set_up_buffer_ptrs`] for more details.
    fn link_with_buffer(
        &mut self,
        buffer: &mut [u8],
        chunk_size: usize,
    ) -> Result<(), DmaBufError> {
        Self::set_up_buffer_ptrs(buffer, self.descriptors, chunk_size, false)
    }

    /// Prepares descriptors for transferring `len` bytes of data.
    ///
    /// See [`Self::set_up_descriptors`] for more details.
    fn set_length(
        &mut self,
        len: usize,
        chunk_size: usize,
        prepare: fn(&mut DmaDescriptor, usize),
    ) -> Result<(), DmaBufError> {
        Self::set_up_descriptors(self.descriptors, len, chunk_size, false, prepare)
    }

    /// Prepares descriptors for reading `len` bytes of data.
    ///
    /// See [`Self::set_up_descriptors`] for more details.
    fn set_rx_length(&mut self, len: usize, chunk_size: usize) -> Result<(), DmaBufError> {
        self.set_length(len, chunk_size, |desc, chunk_size| {
            desc.set_size(chunk_size);
        })
    }

    /// Prepares descriptors for writing `len` bytes of data.
    ///
    /// See [`Self::set_up_descriptors`] for more details.
    fn set_tx_length(&mut self, len: usize, chunk_size: usize) -> Result<(), DmaBufError> {
        self.set_length(len, chunk_size, |desc, chunk_size| {
            desc.set_length(chunk_size);
        })
    }

    /// Returns a slice of descriptors that can cover a buffer of length `len`.
    fn descriptors_for_buffer_len(
        descriptors: &mut [DmaDescriptor],
        len: usize,
        chunk_size: usize,
        is_circular: bool,
    ) -> Result<&mut [DmaDescriptor], DmaBufError> {
        // First, pick enough descriptors to cover the buffer.
        let required_descriptors = descriptor_count(len, chunk_size, is_circular);
        if descriptors.len() < required_descriptors {
            return Err(DmaBufError::InsufficientDescriptors);
        }
        Ok(&mut descriptors[..required_descriptors])
    }

    /// Prepares descriptors for transferring `len` bytes of data.
    ///
    /// `Prepare` means setting up the descriptor lengths and flags, as well as
    /// linking the descriptors into a linked list.
    ///
    /// The actual descriptor setup is done in a callback, because different
    /// transfer directions require different descriptor setup.
    fn set_up_descriptors(
        descriptors: &mut [DmaDescriptor],
        len: usize,
        chunk_size: usize,
        is_circular: bool,
        prepare: impl Fn(&mut DmaDescriptor, usize),
    ) -> Result<(), DmaBufError> {
        let descriptors =
            Self::descriptors_for_buffer_len(descriptors, len, chunk_size, is_circular)?;

        // Link up the descriptors.
        let mut next = if is_circular {
            descriptors.as_mut_ptr()
        } else {
            core::ptr::null_mut()
        };
        for desc in descriptors.iter_mut().rev() {
            desc.next = next;
            next = desc;
        }

        // Prepare each descriptor.
        let mut remaining_length = len;
        for desc in descriptors.iter_mut() {
            let chunk_size = min(chunk_size, remaining_length);
            prepare(desc, chunk_size);
            remaining_length -= chunk_size;
        }
        debug_assert_eq!(remaining_length, 0);

        Ok(())
    }

    /// Associate each descriptor with a chunk of the buffer.
    ///
    /// This function does not check the alignment and location of the buffer,
    /// because some callers may not have enough information currently.
    ///
    /// This function does not set up descriptor lengths or states.
    ///
    /// This function also does not link descriptors into a linked list. This is
    /// intentional, because it is done in `set_up_descriptors` to support
    /// changing length without requiring buffer pointers to be set
    /// repeatedly.
    fn set_up_buffer_ptrs(
        buffer: &mut [u8],
        descriptors: &mut [DmaDescriptor],
        chunk_size: usize,
        is_circular: bool,
    ) -> Result<(), DmaBufError> {
        let descriptors =
            Self::descriptors_for_buffer_len(descriptors, buffer.len(), chunk_size, is_circular)?;

        let chunks = buffer.chunks_mut(chunk_size);
        for (desc, chunk) in descriptors.iter_mut().zip(chunks) {
            desc.set_size(chunk.len());
            desc.buffer = chunk.as_mut_ptr();
        }

        Ok(())
    }
}

/// Block size for transfers to/from PSRAM
#[cfg(psram_dma)]
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum DmaExtMemBKSize {
    /// External memory block size of 16 bytes.
    Size16 = 0,
    /// External memory block size of 32 bytes.
    Size32 = 1,
    /// External memory block size of 64 bytes.
    Size64 = 2,
}

#[cfg(psram_dma)]
impl From<ExternalBurstConfig> for DmaExtMemBKSize {
    fn from(size: ExternalBurstConfig) -> Self {
        match size {
            ExternalBurstConfig::Size16 => DmaExtMemBKSize::Size16,
            ExternalBurstConfig::Size32 => DmaExtMemBKSize::Size32,
            ExternalBurstConfig::Size64 => DmaExtMemBKSize::Size64,
        }
    }
}

pub(crate) struct TxCircularState {
    write_offset: usize,
    write_descr_ptr: *mut DmaDescriptor,
    pub(crate) available: usize,
    last_seen_handled_descriptor_ptr: *mut DmaDescriptor,
    buffer_start: *const u8,
    buffer_len: usize,

    first_desc_ptr: *mut DmaDescriptor,
}

impl TxCircularState {
    pub(crate) fn new(chain: &mut DescriptorChain) -> Self {
        Self {
            write_offset: 0,
            write_descr_ptr: chain.first_mut(),
            available: 0,
            last_seen_handled_descriptor_ptr: chain.first_mut(),
            buffer_start: chain.descriptors[0].buffer as _,
            buffer_len: chain.descriptors.iter().map(|d| d.len()).sum(),

            first_desc_ptr: chain.first_mut(),
        }
    }

    pub(crate) fn update<T>(&mut self, channel: &T) -> Result<(), DmaError>
    where
        T: Tx,
    {
        if channel
            .pending_out_interrupts()
            .contains(DmaTxInterrupt::Eof)
        {
            channel.clear_out(DmaTxInterrupt::Eof);

            // check if all descriptors are owned by CPU - this indicates we failed to push
            // data fast enough in future we can enable `check_owner` and check
            // the interrupt instead
            let mut current = self.last_seen_handled_descriptor_ptr;
            loop {
                let descr = unsafe { current.read_volatile() };
                if descr.owner() == Owner::Cpu {
                    current = descr.next;
                } else {
                    break;
                }

                if current == self.last_seen_handled_descriptor_ptr {
                    return Err(DmaError::Late);
                }
            }

            let descr_address = channel.last_out_dscr_address() as *mut DmaDescriptor;

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
                    while !((*ptr).next.is_null() || (*ptr).next == self.first_desc_ptr) {
                        let dw0 = ptr.read_volatile();
                        self.available += dw0.len();
                        ptr = ptr.offset(1);
                    }

                    // add bytes pointed to by the last descriptor
                    let dw0 = ptr.read_volatile();
                    self.available += dw0.len();

                    // in circular mode we need to honor the now available bytes at start
                    if (*ptr).next == self.first_desc_ptr {
                        ptr = self.first_desc_ptr;
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
                        self.first_desc_ptr
                    } else {
                        next_descriptor
                    }
                }
            }

            self.last_seen_handled_descriptor_ptr = descr_address;
        }

        Ok(())
    }

    pub(crate) fn push(&mut self, data: &[u8]) -> Result<usize, DmaError> {
        let avail = self.available;

        if avail < data.len() {
            return Err(DmaError::Overflow);
        }

        let mut remaining = data.len();
        let mut offset = 0;
        while self.available >= remaining && remaining > 0 {
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

    pub(crate) fn push_with(
        &mut self,
        f: impl FnOnce(&mut [u8]) -> usize,
    ) -> Result<usize, DmaError> {
        // this might write less than available in case of a wrap around
        // caller needs to check and write the remaining part
        let written = unsafe {
            let dst = self.buffer_start.add(self.write_offset).cast_mut();
            let block_size = usize::min(self.available, self.buffer_len - self.write_offset);
            let buffer = core::slice::from_raw_parts_mut(dst, block_size);
            f(buffer)
        };

        let mut forward = written;
        loop {
            unsafe {
                let mut descr = self.write_descr_ptr.read_volatile();
                descr.set_owner(Owner::Dma);
                self.write_descr_ptr.write_volatile(descr);

                let segment_len = descr.len();
                self.write_descr_ptr = if descr.next.is_null() {
                    self.first_desc_ptr
                } else {
                    descr.next
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
}

pub(crate) struct RxCircularState {
    read_descr_ptr: *mut DmaDescriptor,
    pub(crate) available: usize,
    last_seen_handled_descriptor_ptr: *mut DmaDescriptor,
    last_descr_ptr: *mut DmaDescriptor,
}

impl RxCircularState {
    pub(crate) fn new(chain: &mut DescriptorChain) -> Self {
        Self {
            read_descr_ptr: chain.first_mut(),
            available: 0,
            last_seen_handled_descriptor_ptr: core::ptr::null_mut(),
            last_descr_ptr: chain.last_mut(),
        }
    }

    pub(crate) fn update(&mut self) -> Result<(), DmaError> {
        if self.last_seen_handled_descriptor_ptr.is_null() {
            // initially start at last descriptor (so that next will be the first
            // descriptor)
            self.last_seen_handled_descriptor_ptr = self.last_descr_ptr;
        }

        let mut current_in_descr_ptr =
            unsafe { self.last_seen_handled_descriptor_ptr.read_volatile() }.next;
        let mut current_in_descr = unsafe { current_in_descr_ptr.read_volatile() };

        let last_seen_ptr = self.last_seen_handled_descriptor_ptr;
        while current_in_descr.owner() == Owner::Cpu {
            self.available += current_in_descr.len();
            self.last_seen_handled_descriptor_ptr = current_in_descr_ptr;

            current_in_descr_ptr =
                unsafe { self.last_seen_handled_descriptor_ptr.read_volatile() }.next;
            current_in_descr = unsafe { current_in_descr_ptr.read_volatile() };

            if current_in_descr_ptr == last_seen_ptr {
                return Err(DmaError::Late);
            }
        }

        Ok(())
    }

    pub(crate) fn pop(&mut self, data: &mut [u8]) -> Result<usize, DmaError> {
        let len = data.len();
        let mut avail = self.available;

        if avail > len {
            return Err(DmaError::BufferTooSmall);
        }

        let mut remaining_buffer = data;
        let mut descr_ptr = self.read_descr_ptr;

        if descr_ptr.is_null() {
            return Ok(0);
        }

        let mut descr = unsafe { descr_ptr.read_volatile() };

        while avail > 0 && !remaining_buffer.is_empty() && remaining_buffer.len() >= descr.len() {
            unsafe {
                let dst = remaining_buffer.as_mut_ptr();
                let src = descr.buffer;
                let count = descr.len();
                core::ptr::copy_nonoverlapping(src, dst, count);

                descr.set_owner(Owner::Dma);
                descr.set_suc_eof(false);
                descr.set_length(0);
                descr_ptr.write_volatile(descr);

                remaining_buffer = &mut remaining_buffer[count..];
                avail -= count;
                descr_ptr = descr.next;
            }

            if descr_ptr.is_null() {
                break;
            }

            descr = unsafe { descr_ptr.read_volatile() };
        }

        self.read_descr_ptr = descr_ptr;
        self.available = avail;
        Ok(len - remaining_buffer.len())
    }
}

#[doc(hidden)]
#[macro_export]
macro_rules! impl_dma_eligible {
    ([$dma_ch:ident] $name:ident => $dma:ident) => {
        impl $crate::dma::DmaEligible for $crate::peripherals::$name {
            type Dma = $dma_ch;

            fn dma_peripheral(&self) -> $crate::dma::DmaPeripheral {
                $crate::dma::DmaPeripheral::$dma
            }
        }
    };

    (
        $dma_ch:ident {
            $($(#[$cfg:meta])? $name:ident => $dma:ident,)*
        }
    ) => {
        $(
            $(#[$cfg])?
            $crate::impl_dma_eligible!([$dma_ch] $name => $dma);
        )*
    };
}

/// Helper type to get the DMA (Rx and Tx) channel for a peripheral.
pub type PeripheralDmaChannel<T> = <T as DmaEligible>::Dma;
/// Helper type to get the DMA Rx channel for a peripheral.
pub type PeripheralRxChannel<T> = <PeripheralDmaChannel<T> as DmaChannel>::Rx;
/// Helper type to get the DMA Tx channel for a peripheral.
pub type PeripheralTxChannel<T> = <PeripheralDmaChannel<T> as DmaChannel>::Tx;

#[doc(hidden)]
pub trait DmaRxChannel:
    RxRegisterAccess + InterruptAccess<DmaRxInterrupt> + Peripheral<P = Self>
{
}

#[doc(hidden)]
pub trait DmaTxChannel:
    TxRegisterAccess + InterruptAccess<DmaTxInterrupt> + Peripheral<P = Self>
{
}

/// A description of a DMA Channel.
pub trait DmaChannel: Peripheral<P = Self> {
    /// A description of the RX half of a DMA Channel.
    type Rx: DmaRxChannel;

    /// A description of the TX half of a DMA Channel.
    type Tx: DmaTxChannel;

    /// Sets the priority of the DMA channel.
    #[cfg(gdma)]
    fn set_priority(&self, priority: DmaPriority);

    /// Splits the DMA channel into its RX and TX halves.
    #[cfg(any(esp32c6, esp32h2, esp32s3))] // TODO relax this to allow splitting on all chips
    fn split(self) -> (Self::Rx, Self::Tx) {
        // This function is exposed safely on chips that have separate IN and OUT
        // interrupt handlers.
        // TODO: this includes the P4 as well.
        unsafe { self.split_internal(crate::private::Internal) }
    }

    /// Splits the DMA channel into its RX and TX halves.
    ///
    /// # Safety
    ///
    /// This function must only be used if the separate halves are used by the
    /// same peripheral.
    unsafe fn split_internal(self, _: crate::private::Internal) -> (Self::Rx, Self::Tx);
}

#[doc(hidden)]
pub trait DmaChannelExt: DmaChannel {
    fn rx_interrupts() -> impl InterruptAccess<DmaRxInterrupt>;
    fn tx_interrupts() -> impl InterruptAccess<DmaTxInterrupt>;
}

#[diagnostic::on_unimplemented(
    message = "The DMA channel isn't suitable for this peripheral",
    label = "This DMA channel",
    note = "Not all channels are useable with all peripherals"
)]
#[doc(hidden)]
pub trait DmaChannelConvert<DEG> {
    fn degrade(self) -> DEG;
}

impl<DEG: DmaChannel> DmaChannelConvert<DEG> for DEG {
    fn degrade(self) -> DEG {
        self
    }
}

/// Trait implemented for DMA channels that are compatible with a particular
/// peripheral.
///
/// You can use this in places where a peripheral driver would expect a
/// `DmaChannel` implementation.
#[cfg_attr(pdma, doc = "")]
#[cfg_attr(
    pdma,
    doc = "Note that using mismatching channels (e.g. trying to use `DMA_SPI2` with SPI3) may compile, but will panic in runtime."
)]
#[cfg_attr(pdma, doc = "")]
/// ## Example
///
/// The following example demonstrates how this trait can be used to only accept
/// types compatible with a specific peripheral.
///
/// ```rust,no_run
#[doc = crate::before_snippet!()]
/// use esp_hal::spi::master::{Spi, SpiDma, Config, Instance as SpiInstance};
/// use esp_hal::dma::DmaChannelFor;
/// use esp_hal::peripheral::Peripheral;
/// use esp_hal::Blocking;
///
/// fn configures_spi_dma<'d, S, CH>(
///     spi: Spi<'d, Blocking, S>,
///     channel: impl Peripheral<P = CH> + 'd,
/// ) -> SpiDma<'d, Blocking, S>
/// where
///     S: SpiInstance,
///     CH: DmaChannelFor<S> + 'd,
///  {
///     spi.with_dma(channel)
/// }
#[cfg_attr(pdma, doc = "let dma_channel = peripherals.DMA_SPI2;")]
#[cfg_attr(gdma, doc = "let dma_channel = peripherals.DMA_CH0;")]
#[doc = ""]
/// let spi = Spi::new(
///     peripherals.SPI2,
///     Config::default(),
/// )
/// .unwrap();
///
/// let spi_dma = configures_spi_dma(spi, dma_channel);
/// # }
/// ```
pub trait DmaChannelFor<P: DmaEligible>:
    DmaChannel + DmaChannelConvert<PeripheralDmaChannel<P>>
{
}
impl<P, CH> DmaChannelFor<P> for CH
where
    P: DmaEligible,
    CH: DmaChannel + DmaChannelConvert<PeripheralDmaChannel<P>>,
{
}

/// Trait implemented for the RX half of split DMA channels that are compatible
/// with a particular peripheral. Accepts complete DMA channels or split halves.
///
/// This trait is similar in use to [`DmaChannelFor`].
///
/// You can use this in places where a peripheral driver would expect a
/// `DmaRxChannel` implementation.
pub trait RxChannelFor<P: DmaEligible>: DmaChannelConvert<PeripheralRxChannel<P>> {}
impl<P, RX> RxChannelFor<P> for RX
where
    P: DmaEligible,
    RX: DmaChannelConvert<PeripheralRxChannel<P>>,
{
}

/// Trait implemented for the TX half of split DMA channels that are compatible
/// with a particular peripheral. Accepts complete DMA channels or split halves.
///
/// This trait is similar in use to [`DmaChannelFor`].
///
/// You can use this in places where a peripheral driver would expect a
/// `DmaTxChannel` implementation.
pub trait TxChannelFor<PER: DmaEligible>: DmaChannelConvert<PeripheralTxChannel<PER>> {}
impl<P, TX> TxChannelFor<P> for TX
where
    P: DmaEligible,
    TX: DmaChannelConvert<PeripheralTxChannel<P>>,
{
}

/// The functions here are not meant to be used outside the HAL
#[doc(hidden)]
pub trait Rx: crate::private::Sealed {
    unsafe fn prepare_transfer_without_start(
        &mut self,
        peri: DmaPeripheral,
        chain: &DescriptorChain,
    ) -> Result<(), DmaError>;

    unsafe fn prepare_transfer<BUF: DmaRxBuffer>(
        &mut self,
        peri: DmaPeripheral,
        buffer: &mut BUF,
    ) -> Result<(), DmaError>;

    fn start_transfer(&mut self) -> Result<(), DmaError>;

    fn stop_transfer(&mut self);

    #[cfg(gdma)]
    fn set_mem2mem_mode(&mut self, value: bool);

    fn listen_in(&self, interrupts: impl Into<EnumSet<DmaRxInterrupt>>);

    fn unlisten_in(&self, interrupts: impl Into<EnumSet<DmaRxInterrupt>>);

    fn is_listening_in(&self) -> EnumSet<DmaRxInterrupt>;

    fn clear_in(&self, interrupts: impl Into<EnumSet<DmaRxInterrupt>>);

    fn pending_in_interrupts(&self) -> EnumSet<DmaRxInterrupt>;

    fn is_done(&self) -> bool;

    fn has_error(&self) -> bool {
        self.pending_in_interrupts()
            .contains(DmaRxInterrupt::DescriptorError)
    }

    fn has_dscr_empty_error(&self) -> bool {
        self.pending_in_interrupts()
            .contains(DmaRxInterrupt::DescriptorEmpty)
    }

    fn has_eof_error(&self) -> bool {
        self.pending_in_interrupts()
            .contains(DmaRxInterrupt::ErrorEof)
    }

    fn clear_interrupts(&self);

    fn waker(&self) -> &'static crate::asynch::AtomicWaker;
}

// NOTE(p4): because the P4 has two different GDMAs, we won't be able to use
// `GenericPeripheralGuard`.
cfg_if::cfg_if! {
    if #[cfg(pdma)] {
        type PeripheralGuard = system::GenericPeripheralGuard<{ system::Peripheral::Dma as u8}>;
    } else {
        type PeripheralGuard = system::GenericPeripheralGuard<{ system::Peripheral::Gdma as u8}>;
    }
}

fn create_guard(_ch: &impl RegisterAccess) -> PeripheralGuard {
    // NOTE(p4): this function will read the channel's DMA peripheral from `_ch`
    system::GenericPeripheralGuard::new_with(init_dma)
}

// DMA receive channel
#[non_exhaustive]
#[doc(hidden)]
pub struct ChannelRx<'a, M, CH>
where
    M: Mode,
    CH: DmaRxChannel,
{
    pub(crate) rx_impl: PeripheralRef<'a, CH>,
    pub(crate) _phantom: PhantomData<M>,
    pub(crate) _guard: PeripheralGuard,
}

impl<'a, CH> ChannelRx<'a, Blocking, CH>
where
    CH: DmaRxChannel,
{
    /// Creates a new RX channel half.
    pub fn new(rx_impl: impl Peripheral<P = CH> + 'a) -> Self {
        crate::into_ref!(rx_impl);

        let _guard = create_guard(&*rx_impl);

        #[cfg(gdma)]
        // clear the mem2mem mode to avoid failed DMA if this
        // channel was previously used for a mem2mem transfer.
        rx_impl.set_mem2mem_mode(false);

        if let Some(interrupt) = rx_impl.peripheral_interrupt() {
            for cpu in Cpu::all() {
                crate::interrupt::disable(cpu, interrupt);
            }
        }
        rx_impl.set_async(false);

        Self {
            rx_impl,
            _phantom: PhantomData,
            _guard,
        }
    }

    /// Converts a blocking channel to an async channel.
    pub(crate) fn into_async(mut self) -> ChannelRx<'a, Async, CH> {
        if let Some(handler) = self.rx_impl.async_handler() {
            self.set_interrupt_handler(handler);
        }
        self.rx_impl.set_async(true);
        ChannelRx {
            rx_impl: self.rx_impl,
            _phantom: PhantomData,
            _guard: self._guard,
        }
    }

    fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        self.unlisten_in(EnumSet::all());
        self.clear_in(EnumSet::all());

        if let Some(interrupt) = self.rx_impl.peripheral_interrupt() {
            for core in crate::Cpu::other() {
                crate::interrupt::disable(core, interrupt);
            }
            unsafe { crate::interrupt::bind_interrupt(interrupt, handler.handler()) };
            unwrap!(crate::interrupt::enable(interrupt, handler.priority()));
        }
    }
}

impl<'a, CH> ChannelRx<'a, Async, CH>
where
    CH: DmaRxChannel,
{
    /// Converts an async channel into a blocking channel.
    pub(crate) fn into_blocking(self) -> ChannelRx<'a, Blocking, CH> {
        if let Some(interrupt) = self.rx_impl.peripheral_interrupt() {
            crate::interrupt::disable(Cpu::current(), interrupt);
        }
        self.rx_impl.set_async(false);
        ChannelRx {
            rx_impl: self.rx_impl,
            _phantom: PhantomData,
            _guard: self._guard,
        }
    }
}

impl<'a, M, CH> ChannelRx<'a, M, CH>
where
    M: Mode,
    CH: DmaRxChannel,
{
    /// Configure the channel.
    #[cfg(gdma)]
    pub fn set_priority(&mut self, priority: DmaPriority) {
        self.rx_impl.set_priority(priority);
    }

    fn do_prepare(
        &mut self,
        preparation: Preparation,
        peri: DmaPeripheral,
    ) -> Result<(), DmaError> {
        debug_assert_eq!(preparation.direction, TransferDirection::In);

        #[cfg(psram_dma)]
        if preparation.accesses_psram && !self.rx_impl.can_access_psram() {
            return Err(DmaError::UnsupportedMemoryRegion);
        }

        #[cfg(psram_dma)]
        self.rx_impl
            .set_ext_mem_block_size(preparation.burst_transfer.external_memory.into());
        self.rx_impl.set_burst_mode(preparation.burst_transfer);
        self.rx_impl.set_descr_burst_mode(true);
        self.rx_impl.set_check_owner(preparation.check_owner);

        compiler_fence(core::sync::atomic::Ordering::SeqCst);

        self.rx_impl.clear_all();
        self.rx_impl.reset();
        self.rx_impl.set_link_addr(preparation.start as u32);
        self.rx_impl.set_peripheral(peri as u8);

        Ok(())
    }
}

impl<M, CH> crate::private::Sealed for ChannelRx<'_, M, CH>
where
    M: Mode,
    CH: DmaRxChannel,
{
}

impl<M, CH> Rx for ChannelRx<'_, M, CH>
where
    M: Mode,
    CH: DmaRxChannel,
{
    // TODO: used by I2S, which should be rewritten to use the Preparation-based
    // API.
    unsafe fn prepare_transfer_without_start(
        &mut self,
        peri: DmaPeripheral,
        chain: &DescriptorChain,
    ) -> Result<(), DmaError> {
        // We check each descriptor buffer that points to PSRAM for
        // alignment and invalidate the cache for that buffer.
        // NOTE: for RX the `buffer` and `size` need to be aligned but the `len` does
        // not. TRM section 3.4.9
        // Note that DmaBuffer implementations are required to do this for us.
        cfg_if::cfg_if! {
            if #[cfg(psram_dma)] {
                let mut uses_psram = false;
                let psram_range = crate::soc::psram_range();
                for des in chain.descriptors.iter() {
                    // we are forcing the DMA alignment to the cache line size
                    // required when we are using dcache
                    let alignment = crate::soc::cache_get_dcache_line_size() as usize;
                    if crate::soc::addr_in_range(des.buffer as usize, psram_range.clone()) {
                        uses_psram = true;
                        // both the size and address of the buffer must be aligned
                        if des.buffer as usize % alignment != 0 && des.size() % alignment != 0 {
                            return Err(DmaError::InvalidAlignment);
                        }
                        crate::soc::cache_invalidate_addr(des.buffer as u32, des.size() as u32);
                    }
                }
            }
        }

        let preparation = Preparation {
            start: chain.first().cast_mut(),
            direction: TransferDirection::In,
            #[cfg(psram_dma)]
            accesses_psram: uses_psram,
            burst_transfer: BurstConfig::default(),
            check_owner: Some(false),
        };
        self.do_prepare(preparation, peri)
    }

    unsafe fn prepare_transfer<BUF: DmaRxBuffer>(
        &mut self,
        peri: DmaPeripheral,
        buffer: &mut BUF,
    ) -> Result<(), DmaError> {
        let preparation = buffer.prepare();

        self.do_prepare(preparation, peri)
    }

    fn start_transfer(&mut self) -> Result<(), DmaError> {
        self.rx_impl.start();

        if self
            .pending_in_interrupts()
            .contains(DmaRxInterrupt::DescriptorError)
        {
            Err(DmaError::DescriptorError)
        } else {
            Ok(())
        }
    }

    fn stop_transfer(&mut self) {
        self.rx_impl.stop()
    }

    #[cfg(gdma)]
    fn set_mem2mem_mode(&mut self, value: bool) {
        self.rx_impl.set_mem2mem_mode(value);
    }

    fn listen_in(&self, interrupts: impl Into<EnumSet<DmaRxInterrupt>>) {
        self.rx_impl.listen(interrupts);
    }

    fn unlisten_in(&self, interrupts: impl Into<EnumSet<DmaRxInterrupt>>) {
        self.rx_impl.unlisten(interrupts);
    }

    fn is_listening_in(&self) -> EnumSet<DmaRxInterrupt> {
        self.rx_impl.is_listening()
    }

    fn clear_in(&self, interrupts: impl Into<EnumSet<DmaRxInterrupt>>) {
        self.rx_impl.clear(interrupts);
    }

    fn pending_in_interrupts(&self) -> EnumSet<DmaRxInterrupt> {
        self.rx_impl.pending_interrupts()
    }

    fn is_done(&self) -> bool {
        self.pending_in_interrupts()
            .contains(DmaRxInterrupt::SuccessfulEof)
    }

    fn clear_interrupts(&self) {
        self.rx_impl.clear_all();
    }

    fn waker(&self) -> &'static crate::asynch::AtomicWaker {
        self.rx_impl.waker()
    }
}

/// The functions here are not meant to be used outside the HAL
#[doc(hidden)]
pub trait Tx: crate::private::Sealed {
    unsafe fn prepare_transfer_without_start(
        &mut self,
        peri: DmaPeripheral,
        chain: &DescriptorChain,
    ) -> Result<(), DmaError>;

    unsafe fn prepare_transfer<BUF: DmaTxBuffer>(
        &mut self,
        peri: DmaPeripheral,
        buffer: &mut BUF,
    ) -> Result<(), DmaError>;

    fn listen_out(&self, interrupts: impl Into<EnumSet<DmaTxInterrupt>>);

    fn unlisten_out(&self, interrupts: impl Into<EnumSet<DmaTxInterrupt>>);

    fn is_listening_out(&self) -> EnumSet<DmaTxInterrupt>;

    fn clear_out(&self, interrupts: impl Into<EnumSet<DmaTxInterrupt>>);

    fn pending_out_interrupts(&self) -> EnumSet<DmaTxInterrupt>;

    fn start_transfer(&mut self) -> Result<(), DmaError>;

    fn stop_transfer(&mut self);

    fn is_done(&self) -> bool {
        self.pending_out_interrupts()
            .contains(DmaTxInterrupt::TotalEof)
    }

    fn has_error(&self) -> bool {
        self.pending_out_interrupts()
            .contains(DmaTxInterrupt::DescriptorError)
    }

    fn clear_interrupts(&self);

    fn waker(&self) -> &'static crate::asynch::AtomicWaker;

    fn last_out_dscr_address(&self) -> usize;
}

/// DMA transmit channel
#[doc(hidden)]
pub struct ChannelTx<'a, M, CH>
where
    M: Mode,
    CH: DmaTxChannel,
{
    pub(crate) tx_impl: PeripheralRef<'a, CH>,
    pub(crate) _phantom: PhantomData<M>,
    pub(crate) _guard: PeripheralGuard,
}

impl<'a, CH> ChannelTx<'a, Blocking, CH>
where
    CH: DmaTxChannel,
{
    /// Creates a new TX channel half.
    pub fn new(tx_impl: impl Peripheral<P = CH> + 'a) -> Self {
        crate::into_ref!(tx_impl);

        let _guard = create_guard(&*tx_impl);

        if let Some(interrupt) = tx_impl.peripheral_interrupt() {
            for cpu in Cpu::all() {
                crate::interrupt::disable(cpu, interrupt);
            }
        }
        tx_impl.set_async(false);
        Self {
            tx_impl,
            _phantom: PhantomData,
            _guard,
        }
    }

    /// Converts a blocking channel to an async channel.
    pub(crate) fn into_async(mut self) -> ChannelTx<'a, Async, CH> {
        if let Some(handler) = self.tx_impl.async_handler() {
            self.set_interrupt_handler(handler);
        }
        self.tx_impl.set_async(true);
        ChannelTx {
            tx_impl: self.tx_impl,
            _phantom: PhantomData,
            _guard: self._guard,
        }
    }

    fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        self.unlisten_out(EnumSet::all());
        self.clear_out(EnumSet::all());

        if let Some(interrupt) = self.tx_impl.peripheral_interrupt() {
            for core in crate::Cpu::other() {
                crate::interrupt::disable(core, interrupt);
            }
            unsafe { crate::interrupt::bind_interrupt(interrupt, handler.handler()) };
            unwrap!(crate::interrupt::enable(interrupt, handler.priority()));
        }
    }
}

impl<'a, CH> ChannelTx<'a, Async, CH>
where
    CH: DmaTxChannel,
{
    /// Converts an async channel into a blocking channel.
    pub(crate) fn into_blocking(self) -> ChannelTx<'a, Blocking, CH> {
        if let Some(interrupt) = self.tx_impl.peripheral_interrupt() {
            crate::interrupt::disable(Cpu::current(), interrupt);
        }
        self.tx_impl.set_async(false);
        ChannelTx {
            tx_impl: self.tx_impl,
            _phantom: PhantomData,
            _guard: self._guard,
        }
    }
}

impl<'a, M, CH> ChannelTx<'a, M, CH>
where
    M: Mode,
    CH: DmaTxChannel,
{
    /// Configure the channel priority.
    #[cfg(gdma)]
    pub fn set_priority(&mut self, priority: DmaPriority) {
        self.tx_impl.set_priority(priority);
    }

    fn do_prepare(
        &mut self,
        preparation: Preparation,
        peri: DmaPeripheral,
    ) -> Result<(), DmaError> {
        debug_assert_eq!(preparation.direction, TransferDirection::Out);

        #[cfg(psram_dma)]
        if preparation.accesses_psram && !self.tx_impl.can_access_psram() {
            return Err(DmaError::UnsupportedMemoryRegion);
        }

        #[cfg(psram_dma)]
        self.tx_impl
            .set_ext_mem_block_size(preparation.burst_transfer.external_memory.into());
        self.tx_impl.set_burst_mode(preparation.burst_transfer);
        self.tx_impl.set_descr_burst_mode(true);
        self.tx_impl.set_check_owner(preparation.check_owner);

        compiler_fence(core::sync::atomic::Ordering::SeqCst);

        self.tx_impl.clear_all();
        self.tx_impl.reset();
        self.tx_impl.set_link_addr(preparation.start as u32);
        self.tx_impl.set_peripheral(peri as u8);

        Ok(())
    }
}

impl<M, CH> crate::private::Sealed for ChannelTx<'_, M, CH>
where
    M: Mode,
    CH: DmaTxChannel,
{
}

impl<M, CH> Tx for ChannelTx<'_, M, CH>
where
    M: Mode,
    CH: DmaTxChannel,
{
    // TODO: used by I2S, which should be rewritten to use the Preparation-based
    // API.
    unsafe fn prepare_transfer_without_start(
        &mut self,
        peri: DmaPeripheral,
        chain: &DescriptorChain,
    ) -> Result<(), DmaError> {
        // Based on the ESP32-S3 TRM the alignment check is not needed for TX

        // We check each descriptor buffer that points to PSRAM for
        // alignment and writeback the cache for that buffer.
        // Note that DmaBuffer implementations are required to do this for us.
        #[cfg(psram_dma)]
        cfg_if::cfg_if! {
            if #[cfg(psram_dma)] {
                let mut uses_psram = false;
                let psram_range = crate::soc::psram_range();
                for des in chain.descriptors.iter() {
                    // we are forcing the DMA alignment to the cache line size
                    // required when we are using dcache
                    let alignment = crate::soc::cache_get_dcache_line_size() as usize;
                    if crate::soc::addr_in_range(des.buffer as usize, psram_range.clone()) {
                        uses_psram = true;
                        // both the size and address of the buffer must be aligned
                        if des.buffer as usize % alignment != 0 && des.size() % alignment != 0 {
                            return Err(DmaError::InvalidAlignment);
                        }
                        crate::soc::cache_writeback_addr(des.buffer as u32, des.size() as u32);
                    }
                }
            }
        }

        let preparation = Preparation {
            start: chain.first().cast_mut(),
            direction: TransferDirection::Out,
            #[cfg(psram_dma)]
            accesses_psram: uses_psram,
            burst_transfer: BurstConfig::default(),
            check_owner: Some(false),
        };
        self.do_prepare(preparation, peri)?;

        // enable descriptor write back in circular mode
        self.tx_impl
            .set_auto_write_back(!(*chain.last()).next.is_null());

        Ok(())
    }

    unsafe fn prepare_transfer<BUF: DmaTxBuffer>(
        &mut self,
        peri: DmaPeripheral,
        buffer: &mut BUF,
    ) -> Result<(), DmaError> {
        let preparation = buffer.prepare();

        self.do_prepare(preparation, peri)
    }

    fn start_transfer(&mut self) -> Result<(), DmaError> {
        self.tx_impl.start();

        if self
            .pending_out_interrupts()
            .contains(DmaTxInterrupt::DescriptorError)
        {
            Err(DmaError::DescriptorError)
        } else {
            Ok(())
        }
    }

    fn stop_transfer(&mut self) {
        self.tx_impl.stop()
    }

    fn listen_out(&self, interrupts: impl Into<EnumSet<DmaTxInterrupt>>) {
        self.tx_impl.listen(interrupts);
    }

    fn unlisten_out(&self, interrupts: impl Into<EnumSet<DmaTxInterrupt>>) {
        self.tx_impl.unlisten(interrupts);
    }

    fn is_listening_out(&self) -> EnumSet<DmaTxInterrupt> {
        self.tx_impl.is_listening()
    }

    fn clear_out(&self, interrupts: impl Into<EnumSet<DmaTxInterrupt>>) {
        self.tx_impl.clear(interrupts);
    }

    fn pending_out_interrupts(&self) -> EnumSet<DmaTxInterrupt> {
        self.tx_impl.pending_interrupts()
    }

    fn waker(&self) -> &'static crate::asynch::AtomicWaker {
        self.tx_impl.waker()
    }

    fn clear_interrupts(&self) {
        self.tx_impl.clear_all();
    }

    fn last_out_dscr_address(&self) -> usize {
        self.tx_impl.last_dscr_address()
    }
}

#[doc(hidden)]
pub trait RegisterAccess: crate::private::Sealed {
    /// Reset the state machine of the channel and FIFO pointer.
    fn reset(&self);

    /// Enable/Disable INCR burst transfer for channel reading
    /// accessing data in internal RAM.
    fn set_burst_mode(&self, burst_mode: BurstConfig);

    /// Enable/Disable burst transfer for channel reading
    /// descriptors in internal RAM.
    fn set_descr_burst_mode(&self, burst_mode: bool);

    /// The priority of the channel. The larger the value, the higher the
    /// priority.
    #[cfg(gdma)]
    fn set_priority(&self, priority: DmaPriority);

    /// Select a peripheral for the channel.
    fn set_peripheral(&self, peripheral: u8);

    /// Set the address of the first descriptor.
    fn set_link_addr(&self, address: u32);

    /// Enable the channel for data transfer.
    fn start(&self);

    /// Stop the channel from transferring data.
    fn stop(&self);

    /// Mount a new descriptor.
    fn restart(&self);

    /// Configure the bit to enable checking the owner attribute of the
    /// descriptor.
    fn set_check_owner(&self, check_owner: Option<bool>);

    #[cfg(psram_dma)]
    fn set_ext_mem_block_size(&self, size: DmaExtMemBKSize);

    #[cfg(pdma)]
    fn is_compatible_with(&self, peripheral: DmaPeripheral) -> bool;

    #[cfg(psram_dma)]
    fn can_access_psram(&self) -> bool;
}

#[doc(hidden)]
pub trait RxRegisterAccess: RegisterAccess {
    #[cfg(gdma)]
    fn set_mem2mem_mode(&self, value: bool);

    fn peripheral_interrupt(&self) -> Option<Interrupt>;
    fn async_handler(&self) -> Option<InterruptHandler>;
}

#[doc(hidden)]
pub trait TxRegisterAccess: RegisterAccess {
    /// Enable/disable outlink-writeback
    fn set_auto_write_back(&self, enable: bool);

    /// Outlink descriptor address when EOF occurs of Tx channel.
    fn last_dscr_address(&self) -> usize;

    fn peripheral_interrupt(&self) -> Option<Interrupt>;
    fn async_handler(&self) -> Option<InterruptHandler>;
}

#[doc(hidden)]
pub trait InterruptAccess<T: EnumSetType>: crate::private::Sealed {
    fn listen(&self, interrupts: impl Into<EnumSet<T>>) {
        self.enable_listen(interrupts.into(), true)
    }
    fn unlisten(&self, interrupts: impl Into<EnumSet<T>>) {
        self.enable_listen(interrupts.into(), false)
    }

    fn clear_all(&self) {
        self.clear(EnumSet::all());
    }

    fn enable_listen(&self, interrupts: EnumSet<T>, enable: bool);
    fn is_listening(&self) -> EnumSet<T>;
    fn clear(&self, interrupts: impl Into<EnumSet<T>>);
    fn pending_interrupts(&self) -> EnumSet<T>;
    fn waker(&self) -> &'static crate::asynch::AtomicWaker;

    fn is_async(&self) -> bool;
    fn set_async(&self, is_async: bool);
}

/// DMA Channel
#[non_exhaustive]
pub struct Channel<'d, M, CH>
where
    M: Mode,
    CH: DmaChannel,
{
    /// RX half of the channel
    pub rx: ChannelRx<'d, M, CH::Rx>,
    /// TX half of the channel
    pub tx: ChannelTx<'d, M, CH::Tx>,
}

impl<'d, CH> Channel<'d, Blocking, CH>
where
    CH: DmaChannel,
{
    pub(crate) fn new(channel: impl Peripheral<P = CH>) -> Self {
        let (rx, tx) = unsafe {
            channel
                .clone_unchecked()
                .split_internal(crate::private::Internal)
        };
        Self {
            rx: ChannelRx::new(rx),
            tx: ChannelTx::new(tx),
        }
    }

    /// Sets the interrupt handler for RX and TX interrupts.
    ///
    /// Interrupts are not enabled at the peripheral level here.
    pub fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        self.rx.set_interrupt_handler(handler);
        self.tx.set_interrupt_handler(handler);
    }

    /// Listen for the given interrupts
    pub fn listen(&mut self, interrupts: impl Into<EnumSet<DmaInterrupt>>) {
        for interrupt in interrupts.into() {
            match interrupt {
                DmaInterrupt::RxDone => self.rx.listen_in(DmaRxInterrupt::Done),
                DmaInterrupt::TxDone => self.tx.listen_out(DmaTxInterrupt::Done),
            }
        }
    }

    /// Unlisten the given interrupts
    pub fn unlisten(&mut self, interrupts: impl Into<EnumSet<DmaInterrupt>>) {
        for interrupt in interrupts.into() {
            match interrupt {
                DmaInterrupt::RxDone => self.rx.unlisten_in(DmaRxInterrupt::Done),
                DmaInterrupt::TxDone => self.tx.unlisten_out(DmaTxInterrupt::Done),
            }
        }
    }

    /// Gets asserted interrupts
    pub fn interrupts(&mut self) -> EnumSet<DmaInterrupt> {
        let mut res = EnumSet::new();
        if self.rx.is_done() {
            res.insert(DmaInterrupt::RxDone);
        }
        if self.tx.is_done() {
            res.insert(DmaInterrupt::TxDone);
        }
        res
    }

    /// Resets asserted interrupts
    pub fn clear_interrupts(&mut self, interrupts: impl Into<EnumSet<DmaInterrupt>>) {
        for interrupt in interrupts.into() {
            match interrupt {
                DmaInterrupt::RxDone => self.rx.clear_in(DmaRxInterrupt::Done),
                DmaInterrupt::TxDone => self.tx.clear_out(DmaTxInterrupt::Done),
            }
        }
    }

    /// Configure the channel priorities.
    #[cfg(gdma)]
    pub fn set_priority(&mut self, priority: DmaPriority) {
        self.tx.set_priority(priority);
        self.rx.set_priority(priority);
    }

    /// Converts a blocking channel to an async channel.
    pub fn into_async(self) -> Channel<'d, Async, CH> {
        Channel {
            rx: self.rx.into_async(),
            tx: self.tx.into_async(),
        }
    }
}

impl<'d, CH> Channel<'d, Async, CH>
where
    CH: DmaChannel,
{
    /// Converts an async channel to a blocking channel.
    pub fn into_blocking(self) -> Channel<'d, Blocking, CH> {
        Channel {
            rx: self.rx.into_blocking(),
            tx: self.tx.into_blocking(),
        }
    }
}

impl<'d, CH: DmaChannel> From<Channel<'d, Blocking, CH>> for Channel<'d, Async, CH> {
    fn from(channel: Channel<'d, Blocking, CH>) -> Self {
        channel.into_async()
    }
}

impl<'d, CH: DmaChannel> From<Channel<'d, Async, CH>> for Channel<'d, Blocking, CH> {
    fn from(channel: Channel<'d, Async, CH>) -> Self {
        channel.into_blocking()
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
        ///
        /// Please note: This is called in the transfer's `wait` function _and_
        /// by it's [Drop] implementation.
        fn peripheral_wait_dma(&mut self, is_rx: bool, is_tx: bool);

        /// Only used by circular DMA transfers in both, the `stop` function
        /// _and_ it's [Drop] implementation
        fn peripheral_dma_stop(&mut self);
    }

    pub trait DmaSupportTx: DmaSupport {
        type TX: Tx;

        fn tx(&mut self) -> &mut Self::TX;

        fn chain(&mut self) -> &mut DescriptorChain;
    }

    pub trait DmaSupportRx: DmaSupport {
        type RX: Rx;

        fn rx(&mut self) -> &mut Self::RX;

        fn chain(&mut self) -> &mut DescriptorChain;
    }
}

/// DMA transaction for TX only transfers
///
/// # Safety
///
/// Never use [core::mem::forget] on an in-progress transfer
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
        self.instance.peripheral_wait_dma(false, true);

        if self
            .instance
            .tx()
            .pending_out_interrupts()
            .contains(DmaTxInterrupt::DescriptorError)
        {
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

impl<I> Drop for DmaTransferTx<'_, I>
where
    I: dma_private::DmaSupportTx,
{
    fn drop(&mut self) {
        self.instance.peripheral_wait_dma(true, false);
    }
}

/// DMA transaction for RX only transfers
///
/// # Safety
///
/// Never use [core::mem::forget] on an in-progress transfer
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
        self.instance.peripheral_wait_dma(true, false);

        if self
            .instance
            .rx()
            .pending_in_interrupts()
            .contains(DmaRxInterrupt::DescriptorError)
        {
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

impl<I> Drop for DmaTransferRx<'_, I>
where
    I: dma_private::DmaSupportRx,
{
    fn drop(&mut self) {
        self.instance.peripheral_wait_dma(true, false);
    }
}

/// DMA transaction for TX+RX transfers
///
/// # Safety
///
/// Never use [core::mem::forget] on an in-progress transfer
#[non_exhaustive]
#[must_use]
pub struct DmaTransferRxTx<'a, I>
where
    I: dma_private::DmaSupportTx + dma_private::DmaSupportRx,
{
    instance: &'a mut I,
}

impl<'a, I> DmaTransferRxTx<'a, I>
where
    I: dma_private::DmaSupportTx + dma_private::DmaSupportRx,
{
    #[allow(dead_code)]
    pub(crate) fn new(instance: &'a mut I) -> Self {
        Self { instance }
    }

    /// Wait for the transfer to finish.
    pub fn wait(self) -> Result<(), DmaError> {
        self.instance.peripheral_wait_dma(true, true);

        if self
            .instance
            .tx()
            .pending_out_interrupts()
            .contains(DmaTxInterrupt::DescriptorError)
            || self
                .instance
                .rx()
                .pending_in_interrupts()
                .contains(DmaRxInterrupt::DescriptorError)
        {
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

impl<I> Drop for DmaTransferRxTx<'_, I>
where
    I: dma_private::DmaSupportTx + dma_private::DmaSupportRx,
{
    fn drop(&mut self) {
        self.instance.peripheral_wait_dma(true, true);
    }
}

/// DMA transaction for TX only circular transfers
///
/// # Safety
///
/// Never use [core::mem::forget] on an in-progress transfer
#[non_exhaustive]
#[must_use]
pub struct DmaTransferTxCircular<'a, I>
where
    I: dma_private::DmaSupportTx,
{
    instance: &'a mut I,
    state: TxCircularState,
}

impl<'a, I> DmaTransferTxCircular<'a, I>
where
    I: dma_private::DmaSupportTx,
{
    #[allow(unused)] // currently used by peripherals not available on all chips
    pub(crate) fn new(instance: &'a mut I) -> Self {
        let state = TxCircularState::new(instance.chain());
        Self { instance, state }
    }

    /// Amount of bytes which can be pushed.
    pub fn available(&mut self) -> Result<usize, DmaError> {
        self.state.update(self.instance.tx())?;
        Ok(self.state.available)
    }

    /// Push bytes into the DMA buffer.
    pub fn push(&mut self, data: &[u8]) -> Result<usize, DmaError> {
        self.state.update(self.instance.tx())?;
        self.state.push(data)
    }

    /// Push bytes into the DMA buffer via the given closure.
    /// The closure *must* return the actual number of bytes written.
    /// The closure *might* get called with a slice which is smaller than the
    /// total available buffer.
    pub fn push_with(&mut self, f: impl FnOnce(&mut [u8]) -> usize) -> Result<usize, DmaError> {
        self.state.update(self.instance.tx())?;
        self.state.push_with(f)
    }

    /// Stop the DMA transfer
    #[allow(clippy::type_complexity)]
    pub fn stop(self) -> Result<(), DmaError> {
        self.instance.peripheral_dma_stop();

        if self
            .instance
            .tx()
            .pending_out_interrupts()
            .contains(DmaTxInterrupt::DescriptorError)
        {
            Err(DmaError::DescriptorError)
        } else {
            Ok(())
        }
    }
}

impl<I> Drop for DmaTransferTxCircular<'_, I>
where
    I: dma_private::DmaSupportTx,
{
    fn drop(&mut self) {
        self.instance.peripheral_dma_stop();
    }
}

/// DMA transaction for RX only circular transfers
///
/// # Safety
///
/// Never use [core::mem::forget] on an in-progress transfer
#[non_exhaustive]
#[must_use]
pub struct DmaTransferRxCircular<'a, I>
where
    I: dma_private::DmaSupportRx,
{
    instance: &'a mut I,
    state: RxCircularState,
}

impl<'a, I> DmaTransferRxCircular<'a, I>
where
    I: dma_private::DmaSupportRx,
{
    #[allow(unused)] // currently used by peripherals not available on all chips
    pub(crate) fn new(instance: &'a mut I) -> Self {
        let state = RxCircularState::new(instance.chain());
        Self { instance, state }
    }

    /// Amount of bytes which can be popped.
    ///
    /// It's expected to call this before trying to [DmaTransferRxCircular::pop]
    /// data.
    pub fn available(&mut self) -> Result<usize, DmaError> {
        self.state.update()?;
        Ok(self.state.available)
    }

    /// Get available data.
    ///
    /// It's expected that the amount of available data is checked before by
    /// calling [DmaTransferRxCircular::available] and that the buffer can hold
    /// all available data.
    ///
    /// Fails with [DmaError::BufferTooSmall] if the given buffer is too small
    /// to hold all available data
    pub fn pop(&mut self, data: &mut [u8]) -> Result<usize, DmaError> {
        self.state.update()?;
        self.state.pop(data)
    }
}

impl<I> Drop for DmaTransferRxCircular<'_, I>
where
    I: dma_private::DmaSupportRx,
{
    fn drop(&mut self) {
        self.instance.peripheral_dma_stop();
    }
}

pub(crate) mod asynch {
    use core::task::Poll;

    use super::*;

    #[must_use = "futures do nothing unless you `.await` or poll them"]
    pub struct DmaTxFuture<'a, TX>
    where
        TX: Tx,
    {
        pub(crate) tx: &'a mut TX,
    }

    impl<'a, TX> DmaTxFuture<'a, TX>
    where
        TX: Tx,
    {
        pub fn new(tx: &'a mut TX) -> Self {
            Self { tx }
        }
    }

    impl<TX> core::future::Future for DmaTxFuture<'_, TX>
    where
        TX: Tx,
    {
        type Output = Result<(), DmaError>;

        fn poll(
            self: core::pin::Pin<&mut Self>,
            cx: &mut core::task::Context<'_>,
        ) -> Poll<Self::Output> {
            self.tx.waker().register(cx.waker());
            if self.tx.is_done() {
                self.tx.clear_interrupts();
                Poll::Ready(Ok(()))
            } else if self
                .tx
                .pending_out_interrupts()
                .contains(DmaTxInterrupt::DescriptorError)
            {
                self.tx.clear_interrupts();
                Poll::Ready(Err(DmaError::DescriptorError))
            } else {
                self.tx
                    .listen_out(DmaTxInterrupt::TotalEof | DmaTxInterrupt::DescriptorError);
                Poll::Pending
            }
        }
    }

    impl<TX> Drop for DmaTxFuture<'_, TX>
    where
        TX: Tx,
    {
        fn drop(&mut self) {
            self.tx
                .unlisten_out(DmaTxInterrupt::TotalEof | DmaTxInterrupt::DescriptorError);
        }
    }

    #[must_use = "futures do nothing unless you `.await` or poll them"]
    pub struct DmaRxFuture<'a, RX>
    where
        RX: Rx,
    {
        pub(crate) rx: &'a mut RX,
    }

    impl<'a, RX> DmaRxFuture<'a, RX>
    where
        RX: Rx,
    {
        pub fn new(rx: &'a mut RX) -> Self {
            Self { rx }
        }
    }

    impl<RX> core::future::Future for DmaRxFuture<'_, RX>
    where
        RX: Rx,
    {
        type Output = Result<(), DmaError>;

        fn poll(
            self: core::pin::Pin<&mut Self>,
            cx: &mut core::task::Context<'_>,
        ) -> Poll<Self::Output> {
            self.rx.waker().register(cx.waker());
            if self.rx.is_done() {
                self.rx.clear_interrupts();
                Poll::Ready(Ok(()))
            } else if !self.rx.pending_in_interrupts().is_disjoint(
                DmaRxInterrupt::DescriptorError
                    | DmaRxInterrupt::DescriptorEmpty
                    | DmaRxInterrupt::ErrorEof,
            ) {
                self.rx.clear_interrupts();
                Poll::Ready(Err(DmaError::DescriptorError))
            } else {
                self.rx.listen_in(
                    DmaRxInterrupt::SuccessfulEof
                        | DmaRxInterrupt::DescriptorError
                        | DmaRxInterrupt::DescriptorEmpty
                        | DmaRxInterrupt::ErrorEof,
                );
                Poll::Pending
            }
        }
    }

    impl<RX> Drop for DmaRxFuture<'_, RX>
    where
        RX: Rx,
    {
        fn drop(&mut self) {
            self.rx.unlisten_in(
                DmaRxInterrupt::DescriptorError
                    | DmaRxInterrupt::DescriptorEmpty
                    | DmaRxInterrupt::ErrorEof,
            );
        }
    }

    #[cfg(any(i2s0, i2s1))]
    pub struct DmaTxDoneChFuture<'a, TX>
    where
        TX: Tx,
    {
        pub(crate) tx: &'a mut TX,
        _a: (),
    }

    #[cfg(any(i2s0, i2s1))]
    impl<'a, TX> DmaTxDoneChFuture<'a, TX>
    where
        TX: Tx,
    {
        pub fn new(tx: &'a mut TX) -> Self {
            Self { tx, _a: () }
        }
    }

    #[cfg(any(i2s0, i2s1))]
    impl<TX> core::future::Future for DmaTxDoneChFuture<'_, TX>
    where
        TX: Tx,
    {
        type Output = Result<(), DmaError>;

        fn poll(
            self: core::pin::Pin<&mut Self>,
            cx: &mut core::task::Context<'_>,
        ) -> Poll<Self::Output> {
            self.tx.waker().register(cx.waker());
            if self
                .tx
                .pending_out_interrupts()
                .contains(DmaTxInterrupt::Done)
            {
                self.tx.clear_out(DmaTxInterrupt::Done);
                Poll::Ready(Ok(()))
            } else if self
                .tx
                .pending_out_interrupts()
                .contains(DmaTxInterrupt::DescriptorError)
            {
                self.tx.clear_interrupts();
                Poll::Ready(Err(DmaError::DescriptorError))
            } else {
                self.tx
                    .listen_out(DmaTxInterrupt::Done | DmaTxInterrupt::DescriptorError);
                Poll::Pending
            }
        }
    }

    #[cfg(any(i2s0, i2s1))]
    impl<TX> Drop for DmaTxDoneChFuture<'_, TX>
    where
        TX: Tx,
    {
        fn drop(&mut self) {
            self.tx
                .unlisten_out(DmaTxInterrupt::Done | DmaTxInterrupt::DescriptorError);
        }
    }

    #[cfg(any(i2s0, i2s1))]
    pub struct DmaRxDoneChFuture<'a, RX>
    where
        RX: Rx,
    {
        pub(crate) rx: &'a mut RX,
        _a: (),
    }

    #[cfg(any(i2s0, i2s1))]
    impl<'a, RX> DmaRxDoneChFuture<'a, RX>
    where
        RX: Rx,
    {
        pub fn new(rx: &'a mut RX) -> Self {
            Self { rx, _a: () }
        }
    }

    #[cfg(any(i2s0, i2s1))]
    impl<RX> core::future::Future for DmaRxDoneChFuture<'_, RX>
    where
        RX: Rx,
    {
        type Output = Result<(), DmaError>;

        fn poll(
            self: core::pin::Pin<&mut Self>,
            cx: &mut core::task::Context<'_>,
        ) -> Poll<Self::Output> {
            self.rx.waker().register(cx.waker());
            if self
                .rx
                .pending_in_interrupts()
                .contains(DmaRxInterrupt::Done)
            {
                self.rx.clear_in(DmaRxInterrupt::Done);
                Poll::Ready(Ok(()))
            } else if !self.rx.pending_in_interrupts().is_disjoint(
                DmaRxInterrupt::DescriptorError
                    | DmaRxInterrupt::DescriptorEmpty
                    | DmaRxInterrupt::ErrorEof,
            ) {
                self.rx.clear_interrupts();
                Poll::Ready(Err(DmaError::DescriptorError))
            } else {
                self.rx.listen_in(
                    DmaRxInterrupt::Done
                        | DmaRxInterrupt::DescriptorError
                        | DmaRxInterrupt::DescriptorEmpty
                        | DmaRxInterrupt::ErrorEof,
                );
                Poll::Pending
            }
        }
    }

    #[cfg(any(i2s0, i2s1))]
    impl<RX> Drop for DmaRxDoneChFuture<'_, RX>
    where
        RX: Rx,
    {
        fn drop(&mut self) {
            self.rx.unlisten_in(
                DmaRxInterrupt::Done
                    | DmaRxInterrupt::DescriptorError
                    | DmaRxInterrupt::DescriptorEmpty
                    | DmaRxInterrupt::ErrorEof,
            );
        }
    }

    pub(super) fn handle_in_interrupt<CH: DmaChannelExt>() {
        let rx = CH::rx_interrupts();

        if !rx.is_async() {
            return;
        }

        if rx.pending_interrupts().is_disjoint(
            DmaRxInterrupt::DescriptorError
                | DmaRxInterrupt::DescriptorEmpty
                | DmaRxInterrupt::ErrorEof,
        ) {
            rx.unlisten(
                DmaRxInterrupt::DescriptorError
                    | DmaRxInterrupt::DescriptorEmpty
                    | DmaRxInterrupt::ErrorEof
                    | DmaRxInterrupt::SuccessfulEof
                    | DmaRxInterrupt::Done,
            );
            rx.waker().wake()
        }

        if rx
            .pending_interrupts()
            .contains(DmaRxInterrupt::SuccessfulEof)
        {
            rx.unlisten(DmaRxInterrupt::SuccessfulEof);
            rx.waker().wake()
        }

        if rx.pending_interrupts().contains(DmaRxInterrupt::Done) {
            rx.unlisten(DmaRxInterrupt::Done);
            rx.waker().wake()
        }
    }

    pub(super) fn handle_out_interrupt<CH: DmaChannelExt>() {
        let tx = CH::tx_interrupts();

        if !tx.is_async() {
            return;
        }

        if tx
            .pending_interrupts()
            .contains(DmaTxInterrupt::DescriptorError)
        {
            tx.unlisten(
                DmaTxInterrupt::DescriptorError | DmaTxInterrupt::TotalEof | DmaTxInterrupt::Done,
            );
            tx.waker().wake()
        }

        if tx.pending_interrupts().contains(DmaTxInterrupt::TotalEof)
            && tx.is_listening().contains(DmaTxInterrupt::TotalEof)
        {
            tx.unlisten(DmaTxInterrupt::TotalEof);
            tx.waker().wake()
        }

        if tx.pending_interrupts().contains(DmaTxInterrupt::Done) {
            tx.unlisten(DmaTxInterrupt::Done);
            tx.waker().wake()
        }
    }
}
