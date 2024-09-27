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
//! ## Example
//!
//! ### Initialize and utilize DMA controller in `SPI`
//!
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::dma_buffers;
//! # use esp_hal::gpio::Io;
//! # use esp_hal::spi::{master::Spi, SpiMode};
//! # use esp_hal::dma::{Dma, DmaPriority};
//! let dma = Dma::new(peripherals.DMA);
#![cfg_attr(any(esp32, esp32s2), doc = "let dma_channel = dma.spi2channel;")]
#![cfg_attr(not(any(esp32, esp32s2)), doc = "let dma_channel = dma.channel0;")]
//! let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
//! let sclk = io.pins.gpio0;
//! let miso = io.pins.gpio2;
//! let mosi = io.pins.gpio4;
//! let cs = io.pins.gpio5;
//!
//! let mut spi = Spi::new(
//!     peripherals.SPI2,
//!     100.kHz(),
//!     SpiMode::Mode0,
//! )
//! .with_pins(sclk, mosi, miso, cs)
//! .with_dma(dma_channel.configure(
//!     false,
//!     DmaPriority::Priority0,
//! ));
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

use core::{
    cmp::min,
    fmt::Debug,
    marker::PhantomData,
    ptr::null_mut,
    sync::atomic::compiler_fence,
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

use enumset::{EnumSet, EnumSetType};

#[cfg(gdma)]
pub use self::gdma::*;
#[cfg(pdma)]
pub use self::pdma::*;
#[cfg(esp32s3)]
use crate::soc::is_slice_in_psram;
use crate::{interrupt::InterruptHandler, soc::is_slice_in_dram, Mode};

#[cfg(gdma)]
mod gdma;
#[cfg(pdma)]
mod pdma;

/// Kinds of interrupt to listen to.
#[derive(EnumSetType)]
pub enum DmaInterrupt {
    /// RX is done
    RxDone,
    /// TX is done
    TxDone,
}

/// Types of interrupts emitted by the TX channel.
#[derive(EnumSetType)]
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
#[derive(EnumSetType)]
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
/// use esp_hal::dma::DmaBufBlkSize;
///
/// let tx_buf = dma_tx_buffer!(32000);
/// # }
/// ```
#[macro_export]
macro_rules! dma_tx_buffer {
    ($tx_size:expr) => {{
        let (tx_buffer, tx_descriptors) = $crate::dma_buffers_impl!(
            $tx_size,
            $crate::dma::DmaTxBuf::compute_chunk_size(None),
            is_circular = false
        );

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
}

impl From<DmaBufError> for DmaError {
    fn from(error: DmaBufError) -> Self {
        // FIXME: use nested errors
        match error {
            DmaBufError::InsufficientDescriptors => DmaError::OutOfDescriptors,
            DmaBufError::UnsupportedMemoryRegion => DmaError::UnsupportedMemoryRegion,
            DmaBufError::InvalidAlignment => DmaError::InvalidAlignment,
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

/// Marks channels as useable for SPI
#[doc(hidden)]
pub trait DmaEligible {
    /// The DMA peripheral
    const DMA_PERIPHERAL: DmaPeripheral;
    fn dma_peripheral(&self) -> DmaPeripheral {
        Self::DMA_PERIPHERAL
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

/// Marker trait
#[doc(hidden)]
pub trait PeripheralMarker {}

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
        if !crate::soc::is_valid_ram_address(self.first() as usize)
            || !crate::soc::is_valid_ram_address(self.last() as usize)
            || !crate::soc::is_valid_memory_address(data as usize)
            || !crate::soc::is_valid_memory_address(unsafe { data.add(len) } as usize)
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

/// Compute max chunk size based on block size.
const fn max_chunk_size(block_size: Option<DmaBufBlkSize>) -> usize {
    match block_size {
        Some(size) => 4096 - size as usize,
        #[cfg(esp32)]
        None => 4092, // esp32 requires 4 byte alignment
        #[cfg(not(esp32))]
        None => 4095,
    }
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
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum DmaExtMemBKSize {
    /// External memory block size of 16 bytes.
    Size16 = 0,
    /// External memory block size of 32 bytes.
    Size32 = 1,
    /// External memory block size of 64 bytes.
    Size64 = 2,
}

impl From<DmaBufBlkSize> for DmaExtMemBKSize {
    fn from(size: DmaBufBlkSize) -> Self {
        match size {
            DmaBufBlkSize::Size16 => DmaExtMemBKSize::Size16,
            DmaBufBlkSize::Size32 => DmaExtMemBKSize::Size32,
            DmaBufBlkSize::Size64 => DmaExtMemBKSize::Size64,
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

    pub(crate) fn update<T>(&mut self, channel: &T)
    where
        T: Tx,
    {
        if channel
            .pending_out_interrupts()
            .contains(DmaTxInterrupt::Eof)
        {
            channel.clear_out(DmaTxInterrupt::Eof);

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
        let written = unsafe {
            let dst = self.buffer_start.add(self.write_offset).cast_mut();
            let block_size = usize::min(self.available, self.buffer_len - self.write_offset);
            let buffer = core::slice::from_raw_parts_mut(dst, block_size);
            f(buffer)
        };

        let mut forward = written;
        loop {
            unsafe {
                let dw0 = self.write_descr_ptr.read_volatile();
                let segment_len = dw0.len();
                self.write_descr_ptr = if dw0.next.is_null() {
                    self.first_desc_ptr
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

    pub(crate) fn update(&mut self) {
        if self.last_seen_handled_descriptor_ptr.is_null() {
            // initially start at last descriptor (so that next will be the first
            // descriptor)
            self.last_seen_handled_descriptor_ptr = self.last_descr_ptr;
        }

        let mut current_in_descr_ptr =
            unsafe { self.last_seen_handled_descriptor_ptr.read_volatile() }.next;
        let mut current_in_descr = unsafe { current_in_descr_ptr.read_volatile() };

        while current_in_descr.owner() == Owner::Cpu {
            self.available += current_in_descr.len();
            self.last_seen_handled_descriptor_ptr = current_in_descr_ptr;

            current_in_descr_ptr =
                unsafe { self.last_seen_handled_descriptor_ptr.read_volatile() }.next;
            current_in_descr = unsafe { current_in_descr_ptr.read_volatile() };
        }
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

/// A description of a DMA Channel.
pub trait DmaChannel: crate::private::Sealed {
    #[doc(hidden)]
    type Channel: RegisterAccess;

    /// A description of the RX half of a DMA Channel.
    type Rx: RxChannel<Self::Channel>;

    /// A description of the TX half of a DMA Channel.
    type Tx: TxChannel<Self::Channel>;

    /// A suitable peripheral for this DMA channel.
    type P: PeripheralMarker;

    #[doc(hidden)]
    fn set_isr(handler: InterruptHandler);
}

/// The functions here are not meant to be used outside the HAL
#[doc(hidden)]
pub trait Rx: crate::private::Sealed {
    fn init(&mut self, burst_mode: bool, priority: DmaPriority);

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

    #[cfg(esp32s3)]
    fn set_ext_mem_block_size(&self, size: DmaExtMemBKSize);

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
        // clear the mem2mem mode to avoid failed DMA if this
        // channel was previously used for a mem2mem transfer.
        #[cfg(gdma)]
        R::set_mem2mem_mode(false);
    }

    unsafe fn prepare_transfer_without_start(
        &mut self,
        first_desc: *mut DmaDescriptor,
        peri: DmaPeripheral,
    ) -> Result<(), DmaError> {
        compiler_fence(core::sync::atomic::Ordering::SeqCst);

        R::clear_in_interrupts();
        R::reset_in();
        R::set_in_descriptors(first_desc as u32);
        R::set_in_peripheral(peri as u8);

        Ok(())
    }

    fn start_transfer(&mut self) -> Result<(), DmaError> {
        R::start_in();

        if R::pending_in_interrupts().contains(DmaRxInterrupt::DescriptorError) {
            Err(DmaError::DescriptorError)
        } else {
            Ok(())
        }
    }

    fn stop_transfer(&mut self) {
        R::stop_in();
    }

    fn waker() -> &'static embassy_sync::waitqueue::AtomicWaker;
}

// DMA receive channel
#[non_exhaustive]
#[doc(hidden)]
pub struct ChannelRx<'a, CH>
where
    CH: DmaChannel,
{
    pub(crate) burst_mode: bool,
    pub(crate) rx_impl: CH::Rx,
    pub(crate) _phantom: PhantomData<(&'a (), CH)>,
}

impl<'a, CH> ChannelRx<'a, CH>
where
    CH: DmaChannel,
{
    fn new(rx_impl: CH::Rx, burst_mode: bool) -> Self {
        Self {
            burst_mode,
            rx_impl,
            _phantom: PhantomData,
        }
    }
}

impl<'a, CH> crate::private::Sealed for ChannelRx<'a, CH> where CH: DmaChannel {}

impl<'a, CH> Rx for ChannelRx<'a, CH>
where
    CH: DmaChannel,
{
    fn init(&mut self, burst_mode: bool, priority: DmaPriority) {
        self.rx_impl.init(burst_mode, priority);
    }

    unsafe fn prepare_transfer_without_start(
        &mut self,
        peri: DmaPeripheral,
        chain: &DescriptorChain,
    ) -> Result<(), DmaError> {
        if self.burst_mode
            && chain
                .descriptors
                .iter()
                .any(|d| d.len() % 4 != 0 || d.buffer as u32 % 4 != 0)
        {
            return Err(DmaError::InvalidAlignment);
        }

        // for esp32s3 we check each descriptor buffer that points to psram for
        // alignment and invalidate the cache for that buffer
        // NOTE: for RX the `buffer` and `size` need to be aligned but the `len` does
        // not. TRM section 3.4.9
        #[cfg(esp32s3)]
        for des in chain.descriptors.iter() {
            // we are forcing the DMA alignment to the cache line size
            // required when we are using dcache
            let alignment = crate::soc::cache_get_dcache_line_size() as usize;
            if crate::soc::is_valid_psram_address(des.buffer as usize) {
                // both the size and address of the buffer must be aligned
                if des.buffer as usize % alignment != 0 && des.size() % alignment != 0 {
                    return Err(DmaError::InvalidAlignment);
                }
                crate::soc::cache_invalidate_addr(des.buffer as u32, des.size() as u32);
            }
        }

        self.rx_impl
            .prepare_transfer_without_start(chain.first() as _, peri)
    }

    unsafe fn prepare_transfer<BUF: DmaRxBuffer>(
        &mut self,
        peri: DmaPeripheral,
        buffer: &mut BUF,
    ) -> Result<(), DmaError> {
        let preparation = buffer.prepare();

        // TODO: Get burst mode from DmaBuf.
        if self.burst_mode {
            return Err(DmaError::InvalidAlignment);
        }

        self.rx_impl
            .prepare_transfer_without_start(preparation.start, peri)
    }

    fn start_transfer(&mut self) -> Result<(), DmaError> {
        self.rx_impl.start_transfer()
    }

    fn stop_transfer(&mut self) {
        self.rx_impl.stop_transfer()
    }

    #[cfg(esp32s3)]
    fn set_ext_mem_block_size(&self, size: DmaExtMemBKSize) {
        CH::Channel::set_in_ext_mem_block_size(size);
    }

    #[cfg(gdma)]
    fn set_mem2mem_mode(&mut self, value: bool) {
        CH::Channel::set_mem2mem_mode(value);
    }

    fn listen_in(&self, interrupts: impl Into<EnumSet<DmaRxInterrupt>>) {
        CH::Channel::listen_in(interrupts);
    }

    fn unlisten_in(&self, interrupts: impl Into<EnumSet<DmaRxInterrupt>>) {
        CH::Channel::unlisten_in(interrupts);
    }

    fn is_listening_in(&self) -> EnumSet<DmaRxInterrupt> {
        CH::Channel::is_listening_in()
    }

    fn clear_in(&self, interrupts: impl Into<EnumSet<DmaRxInterrupt>>) {
        CH::Channel::clear_in(interrupts);
    }

    fn pending_in_interrupts(&self) -> EnumSet<DmaRxInterrupt> {
        CH::Channel::pending_in_interrupts()
    }

    fn is_done(&self) -> bool {
        self.pending_in_interrupts()
            .contains(DmaRxInterrupt::SuccessfulEof)
    }

    fn clear_interrupts(&self) {
        CH::Channel::clear_in_interrupts();
    }

    fn waker() -> &'static embassy_sync::waitqueue::AtomicWaker {
        CH::Rx::waker()
    }
}

/// The functions here are not meant to be used outside the HAL
#[doc(hidden)]
pub trait Tx: crate::private::Sealed {
    fn init(&mut self, burst_mode: bool, priority: DmaPriority);

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

    #[cfg(esp32s3)]
    fn set_ext_mem_block_size(&self, size: DmaExtMemBKSize);

    fn is_done(&self) -> bool {
        self.pending_out_interrupts()
            .contains(DmaTxInterrupt::TotalEof)
    }

    fn has_error(&self) -> bool {
        self.pending_out_interrupts()
            .contains(DmaTxInterrupt::DescriptorError)
    }

    fn clear_interrupts(&self);

    fn waker() -> &'static embassy_sync::waitqueue::AtomicWaker;

    fn last_out_dscr_address(&self) -> usize;
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

    unsafe fn prepare_transfer_without_start(
        &mut self,
        first_desc: *mut DmaDescriptor,
        peri: DmaPeripheral,
    ) -> Result<(), DmaError> {
        compiler_fence(core::sync::atomic::Ordering::SeqCst);

        R::clear_out_interrupts();
        R::reset_out();
        R::set_out_descriptors(first_desc as u32);
        R::set_out_peripheral(peri as u8);

        Ok(())
    }

    fn start_transfer(&mut self) -> Result<(), DmaError> {
        R::start_out();

        if R::pending_out_interrupts().contains(DmaTxInterrupt::DescriptorError) {
            Err(DmaError::DescriptorError)
        } else {
            Ok(())
        }
    }

    fn stop_transfer(&mut self) {
        R::stop_out();
    }

    fn listen_out(&self, interrupts: impl Into<EnumSet<DmaTxInterrupt>>) {
        R::listen_out(interrupts)
    }
    fn unlisten_out(&self, interrupts: impl Into<EnumSet<DmaTxInterrupt>>) {
        R::unlisten_out(interrupts)
    }
    fn is_listening_out(&self) -> EnumSet<DmaTxInterrupt> {
        R::is_listening_out()
    }
    fn clear_out(&self, interrupts: impl Into<EnumSet<DmaTxInterrupt>>) {
        R::clear_out(interrupts)
    }
    fn pending_out_interrupts(&self) -> EnumSet<DmaTxInterrupt> {
        R::pending_out_interrupts()
    }

    fn waker() -> &'static embassy_sync::waitqueue::AtomicWaker;
}

/// DMA transmit channel
#[doc(hidden)]
pub struct ChannelTx<'a, CH>
where
    CH: DmaChannel,
{
    #[allow(unused)]
    pub(crate) burst_mode: bool,
    pub(crate) tx_impl: CH::Tx,
    pub(crate) _phantom: PhantomData<(&'a (), CH)>,
}

impl<'a, CH> ChannelTx<'a, CH>
where
    CH: DmaChannel,
{
    fn new(tx_impl: CH::Tx, burst_mode: bool) -> Self {
        Self {
            burst_mode,
            tx_impl,
            _phantom: PhantomData,
        }
    }
}

impl<'a, CH> crate::private::Sealed for ChannelTx<'a, CH> where CH: DmaChannel {}

impl<'a, CH> Tx for ChannelTx<'a, CH>
where
    CH: DmaChannel,
{
    fn init(&mut self, burst_mode: bool, priority: DmaPriority) {
        self.tx_impl.init(burst_mode, priority);
    }

    unsafe fn prepare_transfer_without_start(
        &mut self,
        peri: DmaPeripheral,
        chain: &DescriptorChain,
    ) -> Result<(), DmaError> {
        // TODO: based on the ESP32-S3 TRM the alignment check is not needed for TX!
        // for esp32s3 we check each descriptor buffer that points to psram for
        // alignment and writeback the cache for that buffer
        #[cfg(esp32s3)]
        for des in chain.descriptors.iter() {
            // we are forcing the DMA alignment to the cache line size
            // required when we are using dcache
            let alignment = crate::soc::cache_get_dcache_line_size() as usize;
            if crate::soc::is_valid_psram_address(des.buffer as usize) {
                // both the size and address of the buffer must be aligned
                if des.buffer as usize % alignment != 0 && des.size() % alignment != 0 {
                    return Err(DmaError::InvalidAlignment);
                }
                crate::soc::cache_writeback_addr(des.buffer as u32, des.size() as u32);
            }
        }
        self.tx_impl
            .prepare_transfer_without_start(chain.first() as _, peri)
    }

    unsafe fn prepare_transfer<BUF: DmaTxBuffer>(
        &mut self,
        peri: DmaPeripheral,
        buffer: &mut BUF,
    ) -> Result<(), DmaError> {
        let preparation = buffer.prepare();
        cfg_if::cfg_if!(
            if #[cfg(esp32s3)] {
                if let Some(block_size) = preparation.block_size {
                    self.set_ext_mem_block_size(block_size.into());
                }
            } else {
                // we insure that block_size is some only for PSRAM addresses
                if preparation.block_size.is_some() {
                    return Err(DmaError::UnsupportedMemoryRegion);
                }
            }
        );
        // TODO: Get burst mode from DmaBuf.
        self.tx_impl
            .prepare_transfer_without_start(preparation.start, peri)
    }

    fn start_transfer(&mut self) -> Result<(), DmaError> {
        self.tx_impl.start_transfer()
    }

    fn stop_transfer(&mut self) {
        self.tx_impl.stop_transfer()
    }

    #[cfg(esp32s3)]
    fn set_ext_mem_block_size(&self, size: DmaExtMemBKSize) {
        CH::Channel::set_out_ext_mem_block_size(size);
    }

    fn listen_out(&self, interrupts: impl Into<EnumSet<DmaTxInterrupt>>) {
        CH::Channel::listen_out(interrupts);
    }

    fn unlisten_out(&self, interrupts: impl Into<EnumSet<DmaTxInterrupt>>) {
        CH::Channel::unlisten_out(interrupts);
    }

    fn is_listening_out(&self) -> EnumSet<DmaTxInterrupt> {
        CH::Channel::is_listening_out()
    }

    fn clear_out(&self, interrupts: impl Into<EnumSet<DmaTxInterrupt>>) {
        CH::Channel::clear_out(interrupts);
    }

    fn pending_out_interrupts(&self) -> EnumSet<DmaTxInterrupt> {
        CH::Channel::pending_out_interrupts()
    }

    fn waker() -> &'static embassy_sync::waitqueue::AtomicWaker {
        CH::Tx::waker()
    }

    fn clear_interrupts(&self) {
        CH::Channel::clear_out_interrupts();
    }

    fn last_out_dscr_address(&self) -> usize {
        CH::Channel::last_out_dscr_address()
    }
}

#[doc(hidden)]
pub trait RegisterAccess: crate::private::Sealed {
    #[cfg(gdma)]
    fn set_mem2mem_mode(value: bool);
    #[cfg(esp32s3)]
    fn set_out_ext_mem_block_size(size: DmaExtMemBKSize);
    fn set_out_burstmode(burst_mode: bool);
    fn set_out_priority(priority: DmaPriority);
    fn clear_out_interrupts();
    fn reset_out();
    fn set_out_descriptors(address: u32);
    fn set_out_peripheral(peripheral: u8);
    fn start_out();
    fn stop_out();

    fn listen_out(interrupts: impl Into<EnumSet<DmaTxInterrupt>>);
    fn unlisten_out(interrupts: impl Into<EnumSet<DmaTxInterrupt>>);
    fn is_listening_out() -> EnumSet<DmaTxInterrupt>;
    fn clear_out(interrupts: impl Into<EnumSet<DmaTxInterrupt>>);
    fn pending_out_interrupts() -> EnumSet<DmaTxInterrupt>;

    fn listen_in(interrupts: impl Into<EnumSet<DmaRxInterrupt>>);
    fn unlisten_in(interrupts: impl Into<EnumSet<DmaRxInterrupt>>);
    fn is_listening_in() -> EnumSet<DmaRxInterrupt>;
    fn clear_in(interrupts: impl Into<EnumSet<DmaRxInterrupt>>);
    fn pending_in_interrupts() -> EnumSet<DmaRxInterrupt>;

    fn last_out_dscr_address() -> usize;

    #[cfg(esp32s3)]
    fn set_in_ext_mem_block_size(size: DmaExtMemBKSize);
    fn set_in_burstmode(burst_mode: bool);
    fn set_in_priority(priority: DmaPriority);
    fn clear_in_interrupts();
    fn reset_in();
    fn set_in_descriptors(address: u32);
    fn set_in_peripheral(peripheral: u8);
    fn start_in();
    fn stop_in();
}

/// DMA Channel
pub struct Channel<'d, CH, MODE>
where
    CH: DmaChannel,
    MODE: Mode,
{
    /// RX half of the channel
    pub rx: ChannelRx<'d, CH>,
    /// TX half of the channel
    pub tx: ChannelTx<'d, CH>,
    phantom: PhantomData<MODE>,
}

impl<'d, C> Channel<'d, C, crate::Blocking>
where
    C: DmaChannel,
{
    /// Sets the interrupt handler for RX and TX interrupts, enables them
    /// with [crate::interrupt::Priority::max()]
    ///
    /// Interrupts are not enabled at the peripheral level here.
    pub fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        C::set_isr(handler);
    }

    /// Listen for the given interrupts
    pub fn listen(&mut self, interrupts: EnumSet<DmaInterrupt>) {
        for interrupt in interrupts {
            match interrupt {
                DmaInterrupt::RxDone => self.rx.listen_in(DmaRxInterrupt::Done),
                DmaInterrupt::TxDone => self.tx.listen_out(DmaTxInterrupt::Done),
            }
        }
    }

    /// Unlisten the given interrupts
    pub fn unlisten(&mut self, interrupts: EnumSet<DmaInterrupt>) {
        for interrupt in interrupts {
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
    pub fn clear_interrupts(&mut self, interrupts: EnumSet<DmaInterrupt>) {
        for interrupt in interrupts {
            match interrupt {
                DmaInterrupt::RxDone => self.rx.clear_in(DmaRxInterrupt::Done),
                DmaInterrupt::TxDone => self.tx.clear_out(DmaTxInterrupt::Done),
            }
        }
    }
}

/// Holds all the information needed to configure a DMA channel for a transfer.
pub struct Preparation {
    start: *mut DmaDescriptor,
    /// block size for PSRAM transfers (TODO: enable burst mode for non external
    /// memory?)
    #[cfg_attr(not(esp32s3), allow(dead_code))]
    block_size: Option<DmaBufBlkSize>,
    // burst_mode, alignment, check_owner, etc.
}

/// [DmaTxBuffer] is a DMA descriptor + memory combo that can be used for
/// transmitting data from a DMA channel to a peripheral's FIFO.
///
/// # Safety
///
/// The implementing type must keep all its descriptors and the buffers they
/// point to valid while the buffer is being transferred.
pub unsafe trait DmaTxBuffer {
    /// A type providing operations that are safe to perform on the buffer
    /// whilst the DMA is actively using it.
    type View;

    /// Prepares the buffer for an imminent transfer and returns
    /// information required to use this buffer.
    ///
    /// Note: This operation is idempotent.
    fn prepare(&mut self) -> Preparation;

    /// This is called before the DMA starts using the buffer.
    fn into_view(self) -> Self::View;

    /// This is called after the DMA is done using the buffer.
    fn from_view(view: Self::View) -> Self;

    /// Returns the maximum number of bytes that would be transmitted by this
    /// buffer.
    ///
    /// This is a convenience hint for SPI. Most peripherals don't care how long
    /// the transfer is.
    fn length(&self) -> usize;
}

/// [DmaRxBuffer] is a DMA descriptor + memory combo that can be used for
/// receiving data from a peripheral's FIFO to a DMA channel.
///
/// Note: Implementations of this trait may only support having a single EOF bit
/// which resides in the last descriptor. There will be a separate trait in
/// future to support multiple EOFs.
///
/// # Safety
///
/// The implementing type must keep all its descriptors and the buffers they
/// point to valid while the buffer is being transferred.
pub unsafe trait DmaRxBuffer {
    /// A type providing operations that are safe to perform on the buffer
    /// whilst the DMA is actively using it.
    type View;

    /// Prepares the buffer for an imminent transfer and returns
    /// information required to use this buffer.
    ///
    /// Note: This operation is idempotent.
    fn prepare(&mut self) -> Preparation;

    /// This is called before the DMA starts using the buffer.
    fn into_view(self) -> Self::View;

    /// This is called after the DMA is done using the buffer.
    fn from_view(view: Self::View) -> Self;

    /// Returns the maximum number of bytes that can be received by this buffer.
    ///
    /// This is a convenience hint for SPI. Most peripherals don't care how long
    /// the transfer is.
    fn length(&self) -> usize;
}

/// An in-progress view into [DmaRxBuf]/[DmaTxBuf].
///
/// In the future, this could support peeking into state of the
/// descriptors/buffers.
pub struct BufView<T>(T);

/// Error returned from Dma[Rx|Tx|RxTx]Buf operations.
#[derive(Debug, PartialEq, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DmaBufError {
    /// More descriptors are needed for the buffer size
    InsufficientDescriptors,
    /// Descriptors or buffers are not located in a supported memory region
    UnsupportedMemoryRegion,
    /// Buffer is not aligned to the required size
    InvalidAlignment,
    /// Invalid chunk size: must be > 0 and <= 4095
    InvalidChunkSize,
}

/// DMA buffer alignments
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DmaBufBlkSize {
    /// 16 bytes
    Size16 = 16,
    /// 32 bytes
    Size32 = 32,
    /// 64 bytes
    Size64 = 64,
}

/// DMA transmit buffer
///
/// This is a contiguous buffer linked together by DMA descriptors of length
/// 4095 at most. It can only be used for transmitting data to a peripheral's
/// FIFO. See [DmaRxBuf] for receiving data.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DmaTxBuf {
    descriptors: DescriptorSet<'static>,
    buffer: &'static mut [u8],
    block_size: Option<DmaBufBlkSize>,
}

impl DmaTxBuf {
    /// Creates a new [DmaTxBuf] from some descriptors and a buffer.
    ///
    /// There must be enough descriptors for the provided buffer.
    /// Each descriptor can handle 4092 bytes worth of buffer.
    ///
    /// Both the descriptors and buffer must be in DMA-capable memory.
    /// Only DRAM is supported for descriptors.
    pub fn new(
        descriptors: &'static mut [DmaDescriptor],
        buffer: &'static mut [u8],
    ) -> Result<Self, DmaBufError> {
        Self::new_with_block_size(descriptors, buffer, None)
    }

    /// Compute max chunk size based on block size
    pub const fn compute_chunk_size(block_size: Option<DmaBufBlkSize>) -> usize {
        max_chunk_size(block_size)
    }

    /// Compute the number of descriptors required for a given block size and
    /// buffer size
    pub const fn compute_descriptor_count(
        buffer_size: usize,
        block_size: Option<DmaBufBlkSize>,
    ) -> usize {
        descriptor_count(buffer_size, Self::compute_chunk_size(block_size), false)
    }

    /// Creates a new [DmaTxBuf] from some descriptors and a buffer.
    ///
    /// There must be enough descriptors for the provided buffer.
    /// Each descriptor can handle at most 4095 bytes worth of buffer.
    /// Optionally, a block size can be provided for PSRAM & Burst transfers.
    ///
    /// Both the descriptors and buffer must be in DMA-capable memory.
    /// Only DRAM is supported for descriptors.
    pub fn new_with_block_size(
        descriptors: &'static mut [DmaDescriptor],
        buffer: &'static mut [u8],
        block_size: Option<DmaBufBlkSize>,
    ) -> Result<Self, DmaBufError> {
        cfg_if::cfg_if! {
            if #[cfg(esp32s3)] {
                // buffer can be either DRAM or PSRAM (if supported)
                if !is_slice_in_dram(buffer) && !is_slice_in_psram(buffer) {
                    return Err(DmaBufError::UnsupportedMemoryRegion);
                }
                // if its PSRAM, the block_size/alignment must be specified
                if is_slice_in_psram(buffer) && block_size.is_none() {
                    return Err(DmaBufError::InvalidAlignment);
                }
            } else {
                #[cfg(any(esp32,esp32s2))]
                if buffer.len() % 4 != 0 && buffer.as_ptr() as usize % 4 != 0 {
                    // ESP32 requires word alignment for DMA buffers.
                    // ESP32-S2 technically supports byte-aligned DMA buffers, but the
                    // transfer ends up writing out of bounds if the buffer's length
                    // is 2 or 3 (mod 4).
                    return Err(DmaBufError::InvalidAlignment);
                }
                // buffer can only be DRAM
                if !is_slice_in_dram(buffer) {
                    return Err(DmaBufError::UnsupportedMemoryRegion);
                }
            }
        }

        let block_size = if is_slice_in_dram(buffer) {
            // no need for block size if the buffer is in DRAM
            None
        } else {
            block_size
        };
        let mut buf = Self {
            descriptors: DescriptorSet::new(descriptors)?,
            buffer,
            block_size,
        };

        buf.descriptors
            .link_with_buffer(buf.buffer, max_chunk_size(block_size))?;
        buf.set_length(buf.capacity());

        Ok(buf)
    }

    /// Consume the buf, returning the descriptors and buffer.
    pub fn split(self) -> (&'static mut [DmaDescriptor], &'static mut [u8]) {
        (self.descriptors.into_inner(), self.buffer)
    }

    /// Returns the size of the underlying buffer
    pub fn capacity(&self) -> usize {
        self.buffer.len()
    }

    /// Return the number of bytes that would be transmitted by this buf.
    #[allow(clippy::len_without_is_empty)]
    pub fn len(&self) -> usize {
        self.descriptors
            .linked_iter()
            .map(|d| d.len())
            .sum::<usize>()
    }

    /// Reset the descriptors to only transmit `len` amount of bytes from this
    /// buf.
    ///
    /// The number of bytes in data must be less than or equal to the buffer
    /// size.
    pub fn set_length(&mut self, len: usize) {
        assert!(len <= self.buffer.len());

        unwrap!(self
            .descriptors
            .set_tx_length(len, max_chunk_size(self.block_size)));
    }

    /// Fills the TX buffer with the bytes provided in `data` and reset the
    /// descriptors to only cover the filled section.
    ///
    /// The number of bytes in data must be less than or equal to the buffer
    /// size.
    pub fn fill(&mut self, data: &[u8]) {
        self.set_length(data.len());
        self.as_mut_slice()[..data.len()].copy_from_slice(data);
    }

    /// Returns the buf as a mutable slice than can be written.
    pub fn as_mut_slice(&mut self) -> &mut [u8] {
        self.buffer
    }

    /// Returns the buf as a slice than can be read.
    pub fn as_slice(&self) -> &[u8] {
        self.buffer
    }
}

unsafe impl DmaTxBuffer for DmaTxBuf {
    type View = BufView<DmaTxBuf>;

    fn prepare(&mut self) -> Preparation {
        for desc in self.descriptors.linked_iter_mut() {
            // In non-circular mode, we only set `suc_eof` for the last descriptor to signal
            // the end of the transfer.
            desc.reset_for_tx(desc.next.is_null());
        }

        #[cfg(esp32s3)]
        if crate::soc::is_valid_psram_address(self.buffer.as_ptr() as usize) {
            unsafe {
                crate::soc::cache_writeback_addr(
                    self.buffer.as_ptr() as u32,
                    self.buffer.len() as u32,
                )
            };
        }

        Preparation {
            start: self.descriptors.head(),
            block_size: self.block_size,
        }
    }

    fn into_view(self) -> BufView<DmaTxBuf> {
        BufView(self)
    }

    fn from_view(view: Self::View) -> Self {
        view.0
    }

    fn length(&self) -> usize {
        self.len()
    }
}

/// DMA receive buffer
///
/// This is a contiguous buffer linked together by DMA descriptors of length
/// 4092. It can only be used for receiving data from a peripheral's FIFO.
/// See [DmaTxBuf] for transmitting data.
pub struct DmaRxBuf {
    descriptors: DescriptorSet<'static>,
    buffer: &'static mut [u8],
}

impl DmaRxBuf {
    /// Creates a new [DmaRxBuf] from some descriptors and a buffer.
    ///
    /// There must be enough descriptors for the provided buffer.
    /// Each descriptor can handle 4092 bytes worth of buffer.
    ///
    /// Both the descriptors and buffer must be in DMA-capable memory.
    /// Only DRAM is supported.
    pub fn new(
        descriptors: &'static mut [DmaDescriptor],
        buffer: &'static mut [u8],
    ) -> Result<Self, DmaBufError> {
        if !is_slice_in_dram(buffer) {
            return Err(DmaBufError::UnsupportedMemoryRegion);
        }

        let mut buf = Self {
            descriptors: DescriptorSet::new(descriptors)?,
            buffer,
        };

        buf.descriptors
            .link_with_buffer(buf.buffer, max_chunk_size(None))?;
        buf.set_length(buf.capacity());

        Ok(buf)
    }

    /// Consume the buf, returning the descriptors and buffer.
    pub fn split(self) -> (&'static mut [DmaDescriptor], &'static mut [u8]) {
        (self.descriptors.into_inner(), self.buffer)
    }

    /// Returns the size of the underlying buffer
    pub fn capacity(&self) -> usize {
        self.buffer.len()
    }

    /// Returns the maximum number of bytes that this buf has been configured to
    /// receive.
    #[allow(clippy::len_without_is_empty)]
    pub fn len(&self) -> usize {
        self.descriptors
            .linked_iter()
            .map(|d| d.size())
            .sum::<usize>()
    }

    /// Reset the descriptors to only receive `len` amount of bytes into this
    /// buf.
    ///
    /// The number of bytes in data must be less than or equal to the buffer
    /// size.
    pub fn set_length(&mut self, len: usize) {
        assert!(len <= self.buffer.len());

        unwrap!(self.descriptors.set_rx_length(len, max_chunk_size(None)));
    }

    /// Returns the entire underlying buffer as a slice than can be read.
    pub fn as_slice(&self) -> &[u8] {
        self.buffer
    }

    /// Returns the entire underlying buffer as a slice than can be written.
    pub fn as_mut_slice(&mut self) -> &mut [u8] {
        self.buffer
    }

    /// Return the number of bytes that was received by this buf.
    pub fn number_of_received_bytes(&self) -> usize {
        self.descriptors
            .linked_iter()
            .map(|d| d.len())
            .sum::<usize>()
    }

    /// Reads the received data into the provided `buf`.
    ///
    /// If `buf.len()` is less than the amount of received data then only the
    /// first `buf.len()` bytes of received data is written into `buf`.
    ///
    /// Returns the number of bytes in written to `buf`.
    pub fn read_received_data(&self, mut buf: &mut [u8]) -> usize {
        let capacity = buf.len();
        for chunk in self.received_data() {
            if buf.is_empty() {
                break;
            }
            let to_fill;
            (to_fill, buf) = buf.split_at_mut(chunk.len());
            to_fill.copy_from_slice(chunk);
        }

        capacity - buf.len()
    }

    /// Returns the received data as an iterator of slices.
    pub fn received_data(&self) -> impl Iterator<Item = &[u8]> {
        self.descriptors.linked_iter().map(|desc| {
            // SAFETY: We set up the descriptor to point to a subslice of the buffer, and
            // here we are only recreating that slice with a perhaps shorter length.
            // We are also not accessing `self.buffer` while this slice is alive, so we
            // are not violating any aliasing rules.
            unsafe { core::slice::from_raw_parts(desc.buffer.cast_const(), desc.len()) }
        })
    }
}

unsafe impl DmaRxBuffer for DmaRxBuf {
    type View = BufView<DmaRxBuf>;

    fn prepare(&mut self) -> Preparation {
        for desc in self.descriptors.linked_iter_mut() {
            desc.reset_for_rx();
        }

        Preparation {
            start: self.descriptors.head(),
            block_size: None,
        }
    }

    fn into_view(self) -> BufView<DmaRxBuf> {
        BufView(self)
    }

    fn from_view(view: Self::View) -> Self {
        view.0
    }

    fn length(&self) -> usize {
        self.len()
    }
}

/// DMA transmit and receive buffer.
///
/// This is a (single) contiguous buffer linked together by two sets of DMA
/// descriptors of length 4092 each.
/// It can be used for simultaneously transmitting to and receiving from a
/// peripheral's FIFO. These are typically full-duplex transfers.
pub struct DmaRxTxBuf {
    rx_descriptors: DescriptorSet<'static>,
    tx_descriptors: DescriptorSet<'static>,
    buffer: &'static mut [u8],
}

impl DmaRxTxBuf {
    /// Creates a new [DmaRxTxBuf] from some descriptors and a buffer.
    ///
    /// There must be enough descriptors for the provided buffer.
    /// Each descriptor can handle 4092 bytes worth of buffer.
    ///
    /// Both the descriptors and buffer must be in DMA-capable memory.
    /// Only DRAM is supported.
    pub fn new(
        rx_descriptors: &'static mut [DmaDescriptor],
        tx_descriptors: &'static mut [DmaDescriptor],
        buffer: &'static mut [u8],
    ) -> Result<Self, DmaBufError> {
        if !is_slice_in_dram(buffer) {
            return Err(DmaBufError::UnsupportedMemoryRegion);
        }

        let mut buf = Self {
            rx_descriptors: DescriptorSet::new(rx_descriptors)?,
            tx_descriptors: DescriptorSet::new(tx_descriptors)?,
            buffer,
        };
        buf.rx_descriptors
            .link_with_buffer(buf.buffer, max_chunk_size(None))?;
        buf.tx_descriptors
            .link_with_buffer(buf.buffer, max_chunk_size(None))?;
        buf.set_length(buf.capacity());

        Ok(buf)
    }

    /// Consume the buf, returning the rx descriptors, tx descriptors and
    /// buffer.
    pub fn split(
        self,
    ) -> (
        &'static mut [DmaDescriptor],
        &'static mut [DmaDescriptor],
        &'static mut [u8],
    ) {
        (
            self.rx_descriptors.into_inner(),
            self.tx_descriptors.into_inner(),
            self.buffer,
        )
    }

    /// Return the size of the underlying buffer.
    pub fn capacity(&self) -> usize {
        self.buffer.len()
    }

    /// Return the number of bytes that would be transmitted by this buf.
    #[allow(clippy::len_without_is_empty)]
    pub fn len(&self) -> usize {
        self.tx_descriptors
            .linked_iter()
            .map(|d| d.len())
            .sum::<usize>()
    }

    /// Returns the entire buf as a slice than can be read.
    pub fn as_slice(&self) -> &[u8] {
        self.buffer
    }

    /// Returns the entire buf as a slice than can be written.
    pub fn as_mut_slice(&mut self) -> &mut [u8] {
        self.buffer
    }

    /// Reset the descriptors to only transmit/receive `len` amount of bytes
    /// with this buf.
    ///
    /// `len` must be less than or equal to the buffer size.
    pub fn set_length(&mut self, len: usize) {
        assert!(len <= self.buffer.len());

        unwrap!(self.rx_descriptors.set_rx_length(len, max_chunk_size(None)));
        unwrap!(self.tx_descriptors.set_tx_length(len, max_chunk_size(None)));
    }
}

unsafe impl DmaTxBuffer for DmaRxTxBuf {
    type View = BufView<DmaRxTxBuf>;

    fn prepare(&mut self) -> Preparation {
        for desc in self.tx_descriptors.linked_iter_mut() {
            // In non-circular mode, we only set `suc_eof` for the last descriptor to signal
            // the end of the transfer.
            desc.reset_for_tx(desc.next.is_null());
        }

        Preparation {
            start: self.tx_descriptors.head(),
            block_size: None, // TODO: support block size!
        }
    }

    fn into_view(self) -> BufView<DmaRxTxBuf> {
        BufView(self)
    }

    fn from_view(view: Self::View) -> Self {
        view.0
    }

    fn length(&self) -> usize {
        self.len()
    }
}

unsafe impl DmaRxBuffer for DmaRxTxBuf {
    type View = BufView<DmaRxTxBuf>;

    fn prepare(&mut self) -> Preparation {
        for desc in self.rx_descriptors.linked_iter_mut() {
            desc.reset_for_rx();
        }

        Preparation {
            start: self.rx_descriptors.head(),
            block_size: None, // TODO: support block size!
        }
    }

    fn into_view(self) -> BufView<DmaRxTxBuf> {
        BufView(self)
    }

    fn from_view(view: Self::View) -> Self {
        view.0
    }

    fn length(&self) -> usize {
        self.len()
    }
}

/// DMA Streaming Receive Buffer.
///
/// This is a contiguous buffer linked together by DMA descriptors, and the
/// buffer is evenly distributed between each descriptor provided.
///
/// It is used for continuously streaming data from a peripheral's FIFO.
///
/// It does so by maintaining sliding window of descriptors that progresses when
/// you call [DmaRxStreamBufView::consume].
///
/// The list starts out like so `A (empty) -> B (empty) -> C (empty) -> D
/// (empty) -> NULL`.
///
/// As the DMA writes to the buffers the list progresses like so:
/// - `A (empty) -> B (empty) -> C (empty) -> D (empty) -> NULL`
/// - `A (full)  -> B (empty) -> C (empty) -> D (empty) -> NULL`
/// - `A (full)  -> B (full)  -> C (empty) -> D (empty) -> NULL`
/// - `A (full)  -> B (full)  -> C (full)  -> D (empty) -> NULL`
///
/// As you call [DmaRxStreamBufView::consume] the list (approximately)
/// progresses like so:
/// - `A (full)  -> B (full)  -> C (full)  -> D (empty) -> NULL`
/// - `B (full)  -> C (full)  -> D (empty) -> A (empty) -> NULL`
/// - `C (full)  -> D (empty) -> A (empty) -> B (empty) -> NULL`
/// - `D (empty) -> A (empty) -> B (empty) -> C (empty) -> NULL`
///
/// If all the descriptors fill up, the [DmaRxInterrupt::DescriptorEmpty]
/// interrupt will fire and the DMA will stop writing, at which point it is up
/// to you to resume/restart the transfer.
///
/// Note: This buffer will not tell you when this condition occurs, you should
/// check with the driver to see if the DMA has stopped.
///
/// When constructing this buffer, it is important to tune the ratio between the
/// chunk size and buffer size appropriately. Smaller chunk sizes means you
/// receive data more frequently but this means the DMA interrupts
/// ([DmaRxInterrupt::Done]) also fire more frequently (if you use them).
///
/// See [DmaRxStreamBufView] for APIs available whilst a transfer is in
/// progress.
pub struct DmaRxStreamBuf {
    descriptors: &'static mut [DmaDescriptor],
    buffer: &'static mut [u8],
}

impl DmaRxStreamBuf {
    /// Creates a new [DmaRxStreamBuf] evenly distributing the buffer between
    /// the provided descriptors.
    pub fn new(
        descriptors: &'static mut [DmaDescriptor],
        buffer: &'static mut [u8],
    ) -> Result<Self, DmaBufError> {
        if !is_slice_in_dram(descriptors) {
            return Err(DmaBufError::UnsupportedMemoryRegion);
        }
        if !is_slice_in_dram(buffer) {
            return Err(DmaBufError::UnsupportedMemoryRegion);
        }

        if descriptors.is_empty() {
            return Err(DmaBufError::InsufficientDescriptors);
        }

        // Evenly distribute the buffer between the descriptors.
        let chunk_size = buffer.len() / descriptors.len();

        if chunk_size > 4095 {
            return Err(DmaBufError::InsufficientDescriptors);
        }

        // Check that the last descriptor can hold the excess
        let excess = buffer.len() % descriptors.len();
        if chunk_size + excess > 4095 {
            return Err(DmaBufError::InsufficientDescriptors);
        }

        // Link up all the descriptors (but not in a circle).
        let mut next = null_mut();
        for desc in descriptors.iter_mut().rev() {
            desc.next = next;
            next = desc;
        }

        let mut chunks = buffer.chunks_exact_mut(chunk_size);
        for (desc, chunk) in descriptors.iter_mut().zip(chunks.by_ref()) {
            desc.buffer = chunk.as_mut_ptr();
            desc.set_size(chunk.len());
        }

        let remainder = chunks.into_remainder();
        debug_assert_eq!(remainder.len(), excess);

        if !remainder.is_empty() {
            // Append any excess to the last descriptor.
            let last_descriptor = descriptors.last_mut().unwrap();
            last_descriptor.set_size(last_descriptor.size() + remainder.len());
        }

        Ok(Self {
            descriptors,
            buffer,
        })
    }

    /// Consume the buf, returning the descriptors and buffer.
    pub fn split(self) -> (&'static mut [DmaDescriptor], &'static mut [u8]) {
        (self.descriptors, self.buffer)
    }
}

unsafe impl DmaRxBuffer for DmaRxStreamBuf {
    type View = DmaRxStreamBufView;

    fn prepare(&mut self) -> Preparation {
        for desc in self.descriptors.iter_mut() {
            desc.reset_for_rx();
        }
        Preparation {
            start: self.descriptors.as_mut_ptr(),
            block_size: None,
        }
    }

    fn into_view(self) -> DmaRxStreamBufView {
        DmaRxStreamBufView {
            buf: self,
            descriptor_idx: 0,
            descriptor_offset: 0,
        }
    }

    fn from_view(view: Self::View) -> Self {
        view.buf
    }

    fn length(&self) -> usize {
        panic!("DmaCircularBuf doesn't have a length")
    }
}

/// A view into a [DmaRxStreamBuf]
pub struct DmaRxStreamBufView {
    buf: DmaRxStreamBuf,
    descriptor_idx: usize,
    descriptor_offset: usize,
}

impl DmaRxStreamBufView {
    /// Returns the number of bytes that are available to read from the buf.
    pub fn available_bytes(&self) -> usize {
        let (tail, head) = self.buf.descriptors.split_at(self.descriptor_idx);
        let mut result = 0;
        for desc in head.iter().chain(tail) {
            if desc.owner() == Owner::Dma {
                break;
            }
            result += desc.len();
        }
        result - self.descriptor_offset
    }

    /// Reads as much as possible into the buf from the available data.
    pub fn pop(&mut self, buf: &mut [u8]) -> usize {
        if buf.is_empty() {
            return 0;
        }
        let total_bytes = buf.len();

        let mut remaining = buf;
        loop {
            let available = self.peek();
            if available.len() >= remaining.len() {
                remaining.copy_from_slice(&available[0..remaining.len()]);
                self.consume(remaining.len());
                let consumed = remaining.len();
                remaining = &mut remaining[consumed..];
                break;
            } else {
                let to_consume = available.len();
                remaining[0..to_consume].copy_from_slice(available);
                self.consume(to_consume);
                remaining = &mut remaining[to_consume..];
            }
        }

        total_bytes - remaining.len()
    }

    /// Returns a slice into the buffer containing available data.
    /// This will be the longest possible contiguous slice into the buffer that
    /// contains data that is available to read.
    ///
    /// Note: This function ignores EOFs, see [Self::peek_until_eof] if you need
    /// EOF support.
    pub fn peek(&self) -> &[u8] {
        let (slice, _) = self.peek_internal(false);
        slice
    }

    /// Same as [Self::peek] but will not skip over any EOFs.
    ///
    /// It also returns a boolean indicating whether this slice ends with an EOF
    /// or not.
    pub fn peek_until_eof(&self) -> (&[u8], bool) {
        self.peek_internal(true)
    }

    /// Consumes the first `n` bytes from the available data, returning any
    /// fully consumed descriptors back to the DMA.
    /// This is typically called after [Self::peek]/[Self::peek_until_eof].
    ///
    /// Returns the number of bytes that were actually consumed.
    pub fn consume(&mut self, n: usize) -> usize {
        let mut remaining_bytes_to_consume = n;

        loop {
            let desc = &mut self.buf.descriptors[self.descriptor_idx];

            if desc.owner() == Owner::Dma {
                // Descriptor is still owned by DMA so it can't be read yet.
                // This should only happen when there is no more data available to read.
                break;
            }

            let remaining_bytes_in_descriptor = desc.len() - self.descriptor_offset;
            if remaining_bytes_to_consume < remaining_bytes_in_descriptor {
                self.descriptor_offset += remaining_bytes_to_consume;
                remaining_bytes_to_consume = 0;
                break;
            }

            // Reset the descriptor for reuse.
            desc.set_owner(Owner::Dma);
            desc.set_suc_eof(false);
            desc.set_length(0);

            // Before connecting this descriptor to the end of the list, the next descriptor
            // must be disconnected from this one to prevent the DMA from
            // overtaking.
            desc.next = null_mut();

            let desc_ptr: *mut _ = desc;

            let prev_descriptor_index = self
                .descriptor_idx
                .checked_sub(1)
                .unwrap_or(self.buf.descriptors.len() - 1);

            // Connect this consumed descriptor to the end of the chain.
            self.buf.descriptors[prev_descriptor_index].next = desc_ptr;

            self.descriptor_idx += 1;
            if self.descriptor_idx >= self.buf.descriptors.len() {
                self.descriptor_idx = 0;
            }
            self.descriptor_offset = 0;

            remaining_bytes_to_consume -= remaining_bytes_in_descriptor;
        }

        n - remaining_bytes_to_consume
    }

    fn peek_internal(&self, stop_at_eof: bool) -> (&[u8], bool) {
        let descriptors = &self.buf.descriptors[self.descriptor_idx..];

        // There must be at least one descriptor.
        debug_assert!(!descriptors.is_empty());

        if descriptors.len() == 1 {
            let last_descriptor = &descriptors[0];
            if last_descriptor.owner() == Owner::Dma {
                // No data available.
                (&[], false)
            } else {
                let length = last_descriptor.len() - self.descriptor_offset;
                (
                    &self.buf.buffer[self.buf.buffer.len() - length..],
                    last_descriptor.flags.suc_eof(),
                )
            }
        } else {
            let chunk_size = descriptors[0].size();
            let mut found_eof = false;

            let mut number_of_contiguous_bytes = 0;
            for desc in descriptors {
                if desc.owner() == Owner::Dma {
                    break;
                }
                number_of_contiguous_bytes += desc.len();

                if stop_at_eof && desc.flags.suc_eof() {
                    found_eof = true;
                    break;
                }
                // If the length is smaller than the size, the contiguous-ness ends here.
                if desc.len() < desc.size() {
                    break;
                }
            }

            (
                &self.buf.buffer[chunk_size * self.descriptor_idx..][..number_of_contiguous_bytes]
                    [self.descriptor_offset..],
                found_eof,
            )
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

impl<'a, I> Drop for DmaTransferTx<'a, I>
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

impl<'a, I> Drop for DmaTransferRx<'a, I>
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

impl<'a, I> Drop for DmaTransferRxTx<'a, I>
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
    pub fn available(&mut self) -> usize {
        self.state.update(self.instance.tx());
        self.state.available
    }

    /// Push bytes into the DMA buffer.
    pub fn push(&mut self, data: &[u8]) -> Result<usize, DmaError> {
        self.state.update(self.instance.tx());
        self.state.push(data)
    }

    /// Push bytes into the DMA buffer via the given closure.
    /// The closure *must* return the actual number of bytes written.
    /// The closure *might* get called with a slice which is smaller than the
    /// total available buffer.
    pub fn push_with(&mut self, f: impl FnOnce(&mut [u8]) -> usize) -> Result<usize, DmaError> {
        self.state.update(self.instance.tx());
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

impl<'a, I> Drop for DmaTransferTxCircular<'a, I>
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
    pub fn available(&mut self) -> usize {
        self.state.update();
        self.state.available
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
        self.state.update();
        self.state.pop(data)
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

    impl<'a, TX> Drop for DmaTxFuture<'a, TX>
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

    impl<'a, RX> Drop for DmaRxFuture<'a, RX>
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
    impl<'a, TX> Drop for DmaTxDoneChFuture<'a, TX>
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
    impl<'a, RX> Drop for DmaRxDoneChFuture<'a, RX>
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

    fn handle_interrupt<CH: DmaChannel>() {
        if CH::Channel::pending_in_interrupts().is_disjoint(
            DmaRxInterrupt::DescriptorError
                | DmaRxInterrupt::DescriptorEmpty
                | DmaRxInterrupt::ErrorEof,
        ) {
            CH::Channel::unlisten_in(
                DmaRxInterrupt::DescriptorError
                    | DmaRxInterrupt::DescriptorEmpty
                    | DmaRxInterrupt::ErrorEof
                    | DmaRxInterrupt::SuccessfulEof
                    | DmaRxInterrupt::Done,
            );
            CH::Rx::waker().wake()
        }

        if CH::Channel::pending_out_interrupts().contains(DmaTxInterrupt::DescriptorError) {
            CH::Channel::unlisten_out(
                DmaTxInterrupt::DescriptorError | DmaTxInterrupt::TotalEof | DmaTxInterrupt::Done,
            );
            CH::Tx::waker().wake()
        }

        if CH::Channel::pending_in_interrupts().contains(DmaRxInterrupt::SuccessfulEof) {
            CH::Channel::unlisten_in(DmaRxInterrupt::SuccessfulEof);
            CH::Rx::waker().wake()
        }

        if CH::Channel::pending_in_interrupts().contains(DmaRxInterrupt::Done) {
            CH::Channel::unlisten_in(DmaRxInterrupt::Done);
            CH::Rx::waker().wake()
        }

        if CH::Channel::pending_out_interrupts().contains(DmaTxInterrupt::TotalEof)
            && CH::Channel::is_listening_out().contains(DmaTxInterrupt::TotalEof)
        {
            CH::Channel::unlisten_out(DmaTxInterrupt::TotalEof);
            CH::Tx::waker().wake()
        }

        if CH::Channel::pending_out_interrupts().contains(DmaTxInterrupt::Done) {
            CH::Channel::unlisten_out(DmaTxInterrupt::Done);
            CH::Tx::waker().wake()
        }
    }

    #[cfg(not(any(esp32, esp32s2)))]
    pub(crate) mod interrupt {
        use procmacros::handler;

        use super::*;

        #[handler(priority = crate::interrupt::Priority::max())]
        pub(crate) fn interrupt_handler_ch0() {
            handle_interrupt::<DmaChannel0>();
        }

        #[cfg(not(esp32c2))]
        #[handler(priority = crate::interrupt::Priority::max())]
        pub(crate) fn interrupt_handler_ch1() {
            handle_interrupt::<DmaChannel1>();
        }

        #[cfg(not(esp32c2))]
        #[handler(priority = crate::interrupt::Priority::max())]
        pub(crate) fn interrupt_handler_ch2() {
            handle_interrupt::<DmaChannel2>();
        }

        #[cfg(esp32s3)]
        #[handler(priority = crate::interrupt::Priority::max())]
        pub(crate) fn interrupt_handler_ch3() {
            handle_interrupt::<DmaChannel3>();
        }

        #[cfg(esp32s3)]
        #[handler(priority = crate::interrupt::Priority::max())]
        pub(crate) fn interrupt_handler_ch4() {
            handle_interrupt::<DmaChannel4>();
        }
    }

    #[cfg(any(esp32, esp32s2))]
    pub(crate) mod interrupt {
        use procmacros::handler;

        use super::*;

        #[handler(priority = crate::interrupt::Priority::max())]
        pub(crate) fn interrupt_handler_spi2_dma() {
            handle_interrupt::<Spi2DmaChannel>();
        }

        #[cfg(spi3)]
        #[handler(priority = crate::interrupt::Priority::max())]
        pub(crate) fn interrupt_handler_spi3_dma() {
            handle_interrupt::<Spi3DmaChannel>();
        }

        #[cfg(i2s0)]
        #[handler(priority = crate::interrupt::Priority::max())]
        pub(crate) fn interrupt_handler_i2s0() {
            handle_interrupt::<I2s0DmaChannel>();
        }

        #[cfg(i2s1)]
        #[handler(priority = crate::interrupt::Priority::max())]
        pub(crate) fn interrupt_handler_i2s1() {
            handle_interrupt::<I2s1DmaChannel>();
        }
    }
}
