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
//!     &clocks
//! )
//! .with_pins(Some(sclk), Some(mosi), Some(miso), Some(cs))
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
    ptr::addr_of_mut,
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
    #[doc(hidden)]
    #[derive(Clone, Copy)]
    pub struct DmaDescriptorFlags(u32);

    u16;
    size, set_size: 11, 0;
    length, set_length: 23, 12;
    suc_eof, set_suc_eof: 30;
    owner, set_owner: 31;
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

/// A DMA transfer descriptor.
#[derive(Clone, Copy, Debug)]
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

    #[allow(unused)]
    fn size(&self) -> usize {
        self.flags.size() as usize
    }

    fn len(&self) -> usize {
        self.flags.length() as usize
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
use crate::{interrupt::InterruptHandler, soc::is_slice_in_dram, Mode};

#[cfg(gdma)]
mod gdma;
#[cfg(pdma)]
mod pdma;

/// Kinds of interrupt to listen to.
#[derive(EnumSetType)]
pub enum DmaInterrupt {
    /// TX is done
    TxDone,
    /// RX is done
    RxDone,
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
/// // TX and RX buffers are 32000 bytes - passing only one parameter makes TX
/// // and RX the same size.
/// let (tx_buffer, tx_descriptors, rx_buffer, rx_descriptors) =
///     dma_buffers!(32000, 32000);
/// # }
/// ```
#[macro_export]
macro_rules! dma_buffers {
    ($tx_size:expr, $rx_size:expr) => {
        $crate::dma_buffers_chunk_size!($tx_size, $rx_size, $crate::dma::CHUNK_SIZE)
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
/// // TX and RX buffers are 32000 bytes - passing only one parameter makes TX
/// // and RX the same size.
/// let (tx_buffer, tx_descriptors, rx_buffer, rx_descriptors) =
///     dma_circular_buffers!(32000, 32000);
/// # }
/// ```
#[macro_export]
macro_rules! dma_circular_buffers {
    ($tx_size:expr, $rx_size:expr) => {
        $crate::dma_circular_buffers_chunk_size!($tx_size, $rx_size, $crate::dma::CHUNK_SIZE)
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
/// // Create TX and RX descriptors for transactions up to 32000 bytes - passing
/// // only one parameter assumes TX and RX are the same size.
/// let (tx_descriptors, rx_descriptors) = dma_descriptors!(32000, 32000);
/// # }
/// ```
#[macro_export]
macro_rules! dma_descriptors {
    ($tx_size:expr, $rx_size:expr) => {
        $crate::dma_descriptors_chunk_size!($tx_size, $rx_size, $crate::dma::CHUNK_SIZE)
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
/// // Create TX and RX descriptors for transactions up to 32000
/// // bytes - passing only one parameter assumes TX and RX are the same size.
/// let (tx_descriptors, rx_descriptors) =
///     dma_circular_descriptors!(32000, 32000);
/// # }
/// ```
#[macro_export]
macro_rules! dma_circular_descriptors {
    ($tx_size:expr, $rx_size:expr) => {
        $crate::dma_circular_descriptors_chunk_size!($tx_size, $rx_size, $crate::dma::CHUNK_SIZE)
    };

    ($size:expr) => {
        $crate::dma_circular_descriptors_chunk_size!($size, $size, $crate::dma::CHUNK_SIZE)
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
/// let (tx_buffer, tx_descriptors, rx_buffer, rx_descriptors) =
///     dma_buffers_chunk_size!(32000, 32000, 4032);
/// # }
/// ```
#[macro_export]
macro_rules! dma_buffers_chunk_size {
    ($tx_size:expr, $rx_size:expr, $chunk_size:expr) => {{
        static mut TX_BUFFER: [u8; $tx_size] = [0u8; $tx_size];
        static mut RX_BUFFER: [u8; $rx_size] = [0u8; $rx_size];
        let (mut tx_descriptors, mut rx_descriptors) =
            $crate::dma_descriptors_chunk_size!($tx_size, $rx_size, $chunk_size);
        unsafe {
            (
                &mut TX_BUFFER,
                tx_descriptors,
                &mut RX_BUFFER,
                rx_descriptors,
            )
        }
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
/// // TX and RX buffers are 32000 bytes - passing only one parameter makes TX
/// // and RX the same size.
/// let (tx_buffer, tx_descriptors, rx_buffer, rx_descriptors) =
///     dma_circular_buffers_chunk_size!(32000, 32000, 4032);
/// # }
/// ```
#[macro_export]
macro_rules! dma_circular_buffers_chunk_size {
    ($tx_size:expr, $rx_size:expr, $chunk_size:expr) => {{
        static mut TX_BUFFER: [u8; $tx_size] = [0u8; $tx_size];
        static mut RX_BUFFER: [u8; $rx_size] = [0u8; $rx_size];
        let (mut tx_descriptors, mut rx_descriptors) =
            $crate::dma_circular_descriptors_chunk_size!($tx_size, $rx_size, $chunk_size);
        unsafe {
            (
                &mut TX_BUFFER,
                tx_descriptors,
                &mut RX_BUFFER,
                rx_descriptors,
            )
        }
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
/// // Create TX and RX descriptors for transactions up to 32000 bytes - passing
/// // only one parameter assumes TX and RX are the same size.
/// let (tx_descriptors, rx_descriptors) =
///     dma_descriptors_chunk_size!(32000, 32000, 4032);
/// # }
/// ```
#[macro_export]
macro_rules! dma_descriptors_chunk_size {
    ($tx_size:expr, $rx_size:expr, $chunk_size:expr) => {{
        // these will check for size at compile time
        const _: () = ::core::assert!($chunk_size <= 4092, "chunk size must be <= 4092");
        const _: () = ::core::assert!($chunk_size > 0, "chunk size must be > 0");

        static mut TX_DESCRIPTORS: [$crate::dma::DmaDescriptor;
            ($tx_size + $chunk_size - 1) / $chunk_size] =
            [$crate::dma::DmaDescriptor::EMPTY; ($tx_size + $chunk_size - 1) / $chunk_size];
        static mut RX_DESCRIPTORS: [$crate::dma::DmaDescriptor;
            ($rx_size + $chunk_size - 1) / $chunk_size] =
            [$crate::dma::DmaDescriptor::EMPTY; ($rx_size + $chunk_size - 1) / $chunk_size];
        unsafe { (&mut TX_DESCRIPTORS, &mut RX_DESCRIPTORS) }
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
/// // Create TX and RX descriptors for transactions up to 32000 bytes - passing
/// // only one parameter assumes TX and RX are the same size.
/// let (tx_descriptors, rx_descriptors) =
///     dma_circular_descriptors_chunk_size!(32000, 32000, 4032);
/// # }
/// ```
#[macro_export]
macro_rules! dma_circular_descriptors_chunk_size {
    ($tx_size:expr, $rx_size:expr, $chunk_size:expr) => {{
        // these will check for size at compile time
        const _: () = ::core::assert!($chunk_size <= 4092, "chunk size must be <= 4092");
        const _: () = ::core::assert!($chunk_size > 0, "chunk size must be > 0");

        const tx_descriptor_len: usize = if $tx_size > $chunk_size * 2 {
            ($tx_size + $chunk_size - 1) / $chunk_size
        } else {
            3
        };

        const rx_descriptor_len: usize = if $rx_size > $chunk_size * 2 {
            ($rx_size + $chunk_size - 1) / $chunk_size
        } else {
            3
        };

        static mut TX_DESCRIPTORS: [$crate::dma::DmaDescriptor; tx_descriptor_len] =
            [$crate::dma::DmaDescriptor::EMPTY; tx_descriptor_len];
        static mut RX_DESCRIPTORS: [$crate::dma::DmaDescriptor; rx_descriptor_len] =
            [$crate::dma::DmaDescriptor::EMPTY; rx_descriptor_len];
        unsafe { (&mut TX_DESCRIPTORS, &mut RX_DESCRIPTORS) }
    }};

    ($size:expr, $chunk_size:expr) => {
        $crate::dma_circular_descriptors_chunk_size!($size, $size, $chunk_size)
    };
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
    #[cfg(any(esp32c6, esp32h2))]
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

/// DMA Rx
#[doc(hidden)]
pub trait Rx: RxPrivate {}

/// DMA Tx
#[doc(hidden)]
pub trait Tx: TxPrivate {}

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
        Self {
            descriptors,
            chunk_size: CHUNK_SIZE,
        }
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
        if !crate::soc::is_valid_ram_address(self.first() as u32)
            || !crate::soc::is_valid_ram_address(self.last() as u32)
            || !crate::soc::is_valid_memory_address(data as u32)
            || !crate::soc::is_valid_memory_address(unsafe { data.add(len) } as u32)
        {
            return Err(DmaError::UnsupportedMemoryRegion);
        }

        if self.descriptors.len() < len.div_ceil(self.chunk_size) {
            return Err(DmaError::OutOfDescriptors);
        }

        if circular && len <= 3 {
            return Err(DmaError::BufferTooSmall);
        }

        self.descriptors.fill(DmaDescriptor::EMPTY);

        let max_chunk_size = if !circular || len > self.chunk_size * 2 {
            self.chunk_size
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
                    addr_of_mut!(self.descriptors[0])
                } else {
                    core::ptr::null_mut()
                }
            } else {
                addr_of_mut!(self.descriptors[descr + 1])
            };

            // buffer flags
            let dw0 = &mut self.descriptors[descr];

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

        Ok(())
    }

    #[allow(clippy::not_unsafe_ptr_arg_deref)]
    pub fn fill_for_tx(
        &mut self,
        circular: bool,
        data: *const u8,
        len: usize,
    ) -> Result<(), DmaError> {
        if !crate::soc::is_valid_ram_address(self.first() as u32)
            || !crate::soc::is_valid_ram_address(self.last() as u32)
            || !crate::soc::is_valid_memory_address(data as u32)
            || !crate::soc::is_valid_memory_address(unsafe { data.add(len) } as u32)
        {
            return Err(DmaError::UnsupportedMemoryRegion);
        }

        if circular && len <= 3 {
            return Err(DmaError::BufferTooSmall);
        }

        if self.descriptors.len() < len.div_ceil(self.chunk_size) {
            return Err(DmaError::OutOfDescriptors);
        }

        self.descriptors.fill(DmaDescriptor::EMPTY);

        let max_chunk_size = if !circular || len > self.chunk_size * 2 {
            self.chunk_size
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
                    addr_of_mut!(self.descriptors[0])
                } else {
                    core::ptr::null_mut()
                }
            } else {
                addr_of_mut!(self.descriptors[descr + 1])
            };

            // buffer flags
            let dw0 = &mut self.descriptors[descr];

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

        Ok(())
    }
}

/// Block size for transfers to/from psram
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum DmaExtMemBKSize {
    /// External memory block size of 16 bytes.
    Size16 = 0,
    /// External memory block size of 32 bytes.
    Size32 = 1,
    /// External memory block size of 64 bytes.
    Size64 = 2,
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
        T: TxPrivate,
    {
        if channel.descriptors_handled() {
            channel.reset_descriptors_handled();
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
    type Channel: ChannelTypes + RegisterAccess;

    /// A description of the RX half of a DMA Channel.
    type Rx: RxChannel<Self::Channel>;

    /// A description of the TX half of a DMA Channel.
    type Tx: TxChannel<Self::Channel>;

    /// A suitable peripheral for this DMA channel.
    type P: PeripheralMarker;
}

/// The functions here are not meant to be used outside the HAL
#[doc(hidden)]
pub trait RxPrivate: crate::private::Sealed {
    fn init(&mut self, burst_mode: bool, priority: DmaPriority);

    fn init_channel(&mut self);

    unsafe fn prepare_transfer_without_start(
        &mut self,
        peri: DmaPeripheral,
        chain: &DescriptorChain,
    ) -> Result<(), DmaError>;

    unsafe fn prepare_transfer(
        &mut self,
        peri: DmaPeripheral,
        first_desc: *mut DmaDescriptor,
    ) -> Result<(), DmaError>;

    fn start_transfer(&mut self) -> Result<(), DmaError>;

    #[cfg(esp32s3)]
    fn set_ext_mem_block_size(&self, size: DmaExtMemBKSize);

    #[cfg(gdma)]
    fn set_mem2mem_mode(&mut self, value: bool);

    fn listen_ch_in_done(&self);

    fn clear_ch_in_done(&self);

    fn is_ch_in_done_set(&self) -> bool;

    fn unlisten_ch_in_done(&self);

    fn is_listening_ch_in_done(&self) -> bool;

    fn is_done(&self) -> bool;

    fn is_listening_eof(&self) -> bool;

    fn listen_eof(&self);

    fn unlisten_eof(&self);

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

        if R::has_in_descriptor_error() {
            Err(DmaError::DescriptorError)
        } else {
            Ok(())
        }
    }

    fn is_done(&self) -> bool {
        R::is_in_done()
    }

    #[cfg(feature = "async")]
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

impl<'a, CH> Rx for ChannelRx<'a, CH> where CH: DmaChannel {}

impl<'a, CH> crate::private::Sealed for ChannelRx<'a, CH> where CH: DmaChannel {}

impl<'a, CH> RxPrivate for ChannelRx<'a, CH>
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
            if crate::soc::is_valid_psram_address(des.buffer as u32) {
                // both the size and address of the buffer must be aligned
                if des.buffer as usize % alignment != 0 && des.size() % alignment != 0 {
                    return Err(DmaError::InvalidAlignment);
                }
                // TODO: make this optional?
                crate::soc::cache_invalidate_addr(des.buffer as u32, des.size() as u32);
            }
        }

        self.rx_impl
            .prepare_transfer_without_start(chain.first() as _, peri)
    }

    unsafe fn prepare_transfer(
        &mut self,
        peri: DmaPeripheral,
        first_desc: *mut DmaDescriptor,
    ) -> Result<(), DmaError> {
        // TODO: Figure out burst mode for DmaBuf.
        if self.burst_mode {
            return Err(DmaError::InvalidAlignment);
        }

        self.rx_impl
            .prepare_transfer_without_start(first_desc, peri)
    }

    fn start_transfer(&mut self) -> Result<(), DmaError> {
        self.rx_impl.start_transfer()
    }

    #[cfg(esp32s3)]
    fn set_ext_mem_block_size(&self, size: DmaExtMemBKSize) {
        CH::Channel::set_in_ext_mem_block_size(size);
    }

    #[cfg(gdma)]
    fn set_mem2mem_mode(&mut self, value: bool) {
        CH::Channel::set_mem2mem_mode(value);
    }

    fn listen_ch_in_done(&self) {
        CH::Channel::listen_ch_in_done();
    }

    fn clear_ch_in_done(&self) {
        CH::Channel::clear_ch_in_done();
    }

    fn is_ch_in_done_set(&self) -> bool {
        CH::Channel::is_ch_in_done_set()
    }

    fn unlisten_ch_in_done(&self) {
        CH::Channel::unlisten_ch_in_done();
    }

    fn is_listening_ch_in_done(&self) -> bool {
        CH::Channel::is_listening_ch_in_done()
    }

    fn is_done(&self) -> bool {
        self.rx_impl.is_done()
    }

    fn init_channel(&mut self) {
        CH::Channel::init_channel();
    }

    fn is_listening_eof(&self) -> bool {
        CH::Channel::is_listening_in_eof()
    }

    fn listen_eof(&self) {
        CH::Channel::listen_in_eof()
    }

    fn unlisten_eof(&self) {
        CH::Channel::unlisten_in_eof()
    }

    fn has_error(&self) -> bool {
        CH::Channel::has_in_descriptor_error()
    }

    fn has_dscr_empty_error(&self) -> bool {
        CH::Channel::has_in_descriptor_error_dscr_empty()
    }

    fn has_eof_error(&self) -> bool {
        CH::Channel::has_in_descriptor_error_err_eof()
    }

    fn is_listening_in_descriptor_error(&self) -> bool {
        CH::Channel::is_listening_in_descriptor_error()
    }

    fn listen_in_descriptor_error(&self) {
        CH::Channel::listen_in_descriptor_error();
    }

    fn unlisten_in_descriptor_error(&self) {
        CH::Channel::unlisten_in_descriptor_error();
    }

    fn is_listening_in_descriptor_error_dscr_empty(&self) -> bool {
        CH::Channel::is_listening_in_descriptor_error_dscr_empty()
    }

    fn listen_in_descriptor_error_dscr_empty(&self) {
        CH::Channel::listen_in_descriptor_error_dscr_empty();
    }

    fn unlisten_in_descriptor_error_dscr_empty(&self) {
        CH::Channel::unlisten_in_descriptor_error_dscr_empty();
    }

    fn is_listening_in_descriptor_error_err_eof(&self) -> bool {
        CH::Channel::is_listening_in_descriptor_error_err_eof()
    }

    fn listen_in_descriptor_error_err_eof(&self) {
        CH::Channel::listen_in_descriptor_error_err_eof();
    }

    fn unlisten_in_descriptor_error_err_eof(&self) {
        CH::Channel::unlisten_in_descriptor_error_err_eof();
    }

    fn clear_interrupts(&self) {
        CH::Channel::clear_in_interrupts();
    }

    #[cfg(feature = "async")]
    fn waker() -> &'static embassy_sync::waitqueue::AtomicWaker {
        CH::Rx::waker()
    }
}

/// The functions here are not meant to be used outside the HAL
#[doc(hidden)]
pub trait TxPrivate: crate::private::Sealed {
    fn init(&mut self, burst_mode: bool, priority: DmaPriority);

    fn init_channel(&mut self);

    unsafe fn prepare_transfer_without_start(
        &mut self,
        peri: DmaPeripheral,
        chain: &DescriptorChain,
    ) -> Result<(), DmaError>;

    unsafe fn prepare_transfer(
        &mut self,
        peri: DmaPeripheral,
        desc: *mut DmaDescriptor,
    ) -> Result<(), DmaError>;

    fn start_transfer(&mut self) -> Result<(), DmaError>;

    #[cfg(esp32s3)]
    fn set_ext_mem_block_size(&self, size: DmaExtMemBKSize);

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

    fn has_error(&self) -> bool;

    fn clear_interrupts(&self);

    #[cfg(feature = "async")]
    fn waker() -> &'static embassy_sync::waitqueue::AtomicWaker;

    fn descriptors_handled(&self) -> bool;

    fn reset_descriptors_handled(&self);

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

impl<'a, CH> Tx for ChannelTx<'a, CH> where CH: DmaChannel {}

impl<'a, CH> crate::private::Sealed for ChannelTx<'a, CH> where CH: DmaChannel {}

impl<'a, CH> TxPrivate for ChannelTx<'a, CH>
where
    CH: DmaChannel,
{
    fn init(&mut self, burst_mode: bool, priority: DmaPriority) {
        self.tx_impl.init(burst_mode, priority);
    }

    fn init_channel(&mut self) {
        CH::Channel::init_channel();
    }

    unsafe fn prepare_transfer_without_start(
        &mut self,
        peri: DmaPeripheral,
        chain: &DescriptorChain,
    ) -> Result<(), DmaError> {
        // for esp32s3 we check each descriptor buffer that points to psram for
        // alignment and writeback the cache for that buffer
        #[cfg(esp32s3)]
        for des in chain.descriptors.iter() {
            // we are forcing the DMA alignment to the cache line size
            // required when we are using dcache
            let alignment = crate::soc::cache_get_dcache_line_size() as usize;
            if crate::soc::is_valid_psram_address(des.buffer as u32) {
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

    unsafe fn prepare_transfer(
        &mut self,
        peri: DmaPeripheral,
        desc: *mut DmaDescriptor,
    ) -> Result<(), DmaError> {
        self.tx_impl.prepare_transfer_without_start(desc, peri)
    }

    fn start_transfer(&mut self) -> Result<(), DmaError> {
        self.tx_impl.start_transfer()
    }

    #[cfg(esp32s3)]
    fn set_ext_mem_block_size(&self, size: DmaExtMemBKSize) {
        CH::Channel::set_out_ext_mem_block_size(size);
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

    fn is_listening_eof(&self) -> bool {
        CH::Channel::is_listening_out_eof()
    }

    fn listen_eof(&self) {
        CH::Channel::listen_out_eof()
    }

    fn unlisten_eof(&self) {
        CH::Channel::unlisten_out_eof()
    }

    fn has_error(&self) -> bool {
        CH::Channel::has_out_descriptor_error()
    }

    #[cfg(feature = "async")]
    fn waker() -> &'static embassy_sync::waitqueue::AtomicWaker {
        CH::Tx::waker()
    }

    fn is_listening_out_descriptor_error(&self) -> bool {
        CH::Channel::is_listening_out_descriptor_error()
    }

    fn listen_out_descriptor_error(&self) {
        CH::Channel::listen_out_descriptor_error();
    }

    fn unlisten_out_descriptor_error(&self) {
        CH::Channel::unlisten_out_descriptor_error();
    }

    fn clear_interrupts(&self) {
        CH::Channel::clear_out_interrupts();
    }

    fn descriptors_handled(&self) -> bool {
        self.tx_impl.descriptors_handled()
    }

    fn reset_descriptors_handled(&self) {
        self.tx_impl.reset_descriptors_handled()
    }

    fn last_out_dscr_address(&self) -> usize {
        self.tx_impl.last_out_dscr_address()
    }
}

#[doc(hidden)]
pub trait RegisterAccess: crate::private::Sealed {
    fn init_channel();
    #[cfg(gdma)]
    fn set_mem2mem_mode(value: bool);
    #[cfg(esp32s3)]
    fn set_out_ext_mem_block_size(size: DmaExtMemBKSize);
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

    #[cfg(esp32s3)]
    fn set_in_ext_mem_block_size(size: DmaExtMemBKSize);
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
    type Binder: InterruptBinder;
}

#[doc(hidden)]
pub trait InterruptBinder: crate::private::Sealed {
    fn set_isr(handler: InterruptHandler);
}

/// DMA Channel
pub struct Channel<'d, CH, MODE>
where
    CH: DmaChannel,
    MODE: Mode,
{
    /// TX half of the channel
    pub tx: ChannelTx<'d, CH>,
    /// RX half of the channel
    pub rx: ChannelRx<'d, CH>,
    phantom: PhantomData<MODE>,
}

impl<'d, C> Channel<'d, C, crate::Blocking>
where
    C: DmaChannel,
{
    /// Sets the interrupt handler for TX and RX interrupts, enables them
    /// with [crate::interrupt::Priority::max()]
    ///
    /// Interrupts are not enabled at the peripheral level here.
    pub fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        <C::Channel as ChannelTypes>::Binder::set_isr(handler);
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

/// Error returned from Dma[Tx|Rx|TxRx]Buf operations.
#[derive(Debug)]
pub enum DmaBufError {
    /// More descriptors are needed for the buffer size
    InsufficientDescriptors,
    /// Descriptors or buffers are not located in a supported memory region
    UnsupportedMemoryRegion,
}

/// DMA transmit buffer
///
/// This is a contiguous buffer linked together by DMA descriptors of length
/// 4092. It can only be used for transmitting data to a peripheral's FIFO.
/// See [DmaRxBuf] for receiving data.
#[derive(Debug)]
pub struct DmaTxBuf {
    descriptors: &'static mut [DmaDescriptor],
    buffer: &'static mut [u8],
}

impl DmaTxBuf {
    /// Creates a new [DmaTxBuf] from some descriptors and a buffer.
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
        let min_descriptors = buffer.len().div_ceil(CHUNK_SIZE);
        if descriptors.len() < min_descriptors {
            return Err(DmaBufError::InsufficientDescriptors);
        }

        if !is_slice_in_dram(descriptors) || !is_slice_in_dram(buffer) {
            return Err(DmaBufError::UnsupportedMemoryRegion);
        }

        // Setup size and buffer pointer as these will not change for the remainder of
        // this object's lifetime
        let chunk_iter = descriptors.iter_mut().zip(buffer.chunks_mut(CHUNK_SIZE));
        for (desc, chunk) in chunk_iter {
            desc.set_size(chunk.len());
            desc.buffer = chunk.as_mut_ptr();
        }

        let mut buf = Self {
            descriptors,
            buffer,
        };
        buf.set_length(buf.capacity());

        Ok(buf)
    }

    /// Consume the buf, returning the descriptors and buffer.
    pub fn split(self) -> (&'static mut [DmaDescriptor], &'static mut [u8]) {
        (self.descriptors, self.buffer)
    }

    /// Returns the size of the underlying buffer
    pub fn capacity(&self) -> usize {
        self.buffer.len()
    }

    /// Return the number of bytes that would be transmitted by this buf.
    #[allow(clippy::len_without_is_empty)]
    pub fn len(&self) -> usize {
        let mut result = 0;
        for desc in self.descriptors.iter() {
            result += desc.len();
            if desc.next.is_null() {
                break;
            }
        }
        result
    }

    /// Reset the descriptors to only transmit `len` amount of bytes from this
    /// buf.
    ///
    /// The number of bytes in data must be less than or equal to the buffer
    /// size.
    pub fn set_length(&mut self, len: usize) {
        assert!(len <= self.buffer.len());

        // Get the minimum number of descriptors needed for this length of data.
        let descriptor_count = len.div_ceil(CHUNK_SIZE).max(1);
        let required_descriptors = &mut self.descriptors[0..descriptor_count];

        // Link up the relevant descriptors.
        let mut next = core::ptr::null_mut();
        for desc in required_descriptors.iter_mut().rev() {
            desc.next = next;
            next = desc;
        }

        let mut remaining_length = len;
        for desc in required_descriptors.iter_mut() {
            // As this is a simple dma buffer implementation we won't
            // be making use of this feature.
            desc.set_suc_eof(false);

            // This isn't strictly needed for this simple implementation,
            // but it is useful for debugging.
            desc.set_owner(Owner::Dma);

            let chunk_size = min(remaining_length, desc.flags.size() as usize);
            desc.set_length(chunk_size);
            remaining_length -= chunk_size;
        }
        debug_assert_eq!(remaining_length, 0);

        required_descriptors.last_mut().unwrap().set_suc_eof(true);
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
        &mut self.buffer[..]
    }

    /// Returns the buf as a slice than can be read.
    pub fn as_slice(&self) -> &[u8] {
        self.buffer
    }

    pub(crate) fn first(&self) -> *mut DmaDescriptor {
        self.descriptors.as_ptr() as _
    }
}

/// DMA receive buffer
///
/// This is a contiguous buffer linked together by DMA descriptors of length
/// 4092. It can only be used for receiving data from a peripheral's FIFO.
/// See [DmaTxBuf] for transmitting data.
pub struct DmaRxBuf {
    descriptors: &'static mut [DmaDescriptor],
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
        let min_descriptors = buffer.len().div_ceil(CHUNK_SIZE);
        if descriptors.len() < min_descriptors {
            return Err(DmaBufError::InsufficientDescriptors);
        }

        if !is_slice_in_dram(descriptors) || !is_slice_in_dram(buffer) {
            return Err(DmaBufError::UnsupportedMemoryRegion);
        }

        // Setup size and buffer pointer as these will not change for the remainder of
        // this object's lifetime
        let chunk_iter = descriptors.iter_mut().zip(buffer.chunks_mut(CHUNK_SIZE));
        for (desc, chunk) in chunk_iter {
            desc.set_size(chunk.len());
            desc.buffer = chunk.as_mut_ptr();
        }

        let mut buf = Self {
            descriptors,
            buffer,
        };

        buf.set_length(buf.capacity());

        Ok(buf)
    }

    /// Consume the buf, returning the descriptors and buffer.
    pub fn split(self) -> (&'static mut [DmaDescriptor], &'static mut [u8]) {
        (self.descriptors, self.buffer)
    }

    /// Returns the size of the underlying buffer
    pub fn capacity(&self) -> usize {
        self.buffer.len()
    }

    /// Returns the maximum number of bytes that this buf has been configured to
    /// receive.
    #[allow(clippy::len_without_is_empty)]
    pub fn len(&self) -> usize {
        let mut result = 0;
        for desc in self.descriptors.iter() {
            result += desc.flags.size() as usize;
            if desc.next.is_null() {
                break;
            }
        }
        result
    }

    /// Reset the descriptors to only receive `len` amount of bytes into this
    /// buf.
    ///
    /// The number of bytes in data must be less than or equal to the buffer
    /// size.
    pub fn set_length(&mut self, len: usize) {
        assert!(len <= self.buffer.len());

        // Get the minimum number of descriptors needed for this length of data.
        let descriptor_count = len.div_ceil(CHUNK_SIZE).max(1);
        let required_descriptors = &mut self.descriptors[..descriptor_count];

        // Link up the relevant descriptors.
        let mut next = core::ptr::null_mut();
        for desc in required_descriptors.iter_mut().rev() {
            desc.next = next;
            next = desc;
        }

        // Get required part of the buffer.
        let mut remaining_length = len;
        for desc in required_descriptors.iter_mut() {
            // Clear this to allow hardware to set it when the peripheral returns an EOF
            // bit.
            desc.set_suc_eof(false);

            // This isn't strictly needed for this simple implementation,
            // but it is useful for debugging.
            desc.set_owner(Owner::Dma);

            // Clear this to allow hardware to set it when it's
            // done receiving data for this descriptor.
            desc.set_length(0);

            let chunk_size = min(CHUNK_SIZE, remaining_length);
            desc.set_size(chunk_size);
            remaining_length -= chunk_size;
        }
        debug_assert_eq!(remaining_length, 0);
    }

    /// Returns the entire underlying buffer as a slice than can be read.
    pub fn as_slice(&self) -> &[u8] {
        self.buffer
    }

    /// Returns the entire underlying buffer as a slice than can be written.
    pub fn as_mut_slice(&mut self) -> &mut [u8] {
        &mut self.buffer[..]
    }

    /// Return the number of bytes that was received by this buf.
    pub fn number_of_received_bytes(&self) -> usize {
        let mut result = 0;
        for desc in self.descriptors.iter() {
            result += desc.len();
            if desc.next.is_null() {
                break;
            }
        }
        result
    }

    /// Reads the received data into the provided `buf`.
    ///
    /// If `buf.len()` is less than the amount of received data then only the
    /// first `buf.len()` bytes of received data is written into `buf`.
    ///
    /// Returns the number of bytes in written to `buf`.
    pub fn read_received_data(&self, buf: &mut [u8]) -> usize {
        let mut remaining = &mut buf[..];

        let mut buffer_offset = 0;
        for desc in self.descriptors.iter() {
            if remaining.is_empty() {
                break;
            }

            let amount_to_copy = min(desc.len(), remaining.len());

            let (to_fill, to_remain) = remaining.split_at_mut(amount_to_copy);
            to_fill.copy_from_slice(&self.buffer[buffer_offset..][..amount_to_copy]);
            remaining = to_remain;

            if desc.next.is_null() {
                break;
            }
            buffer_offset += desc.flags.size() as usize;
        }

        let remaining_bytes = remaining.len();
        buf.len() - remaining_bytes
    }

    /// Returns the received data as an iterator of slices.
    pub fn received_data(&self) -> impl Iterator<Item = &[u8]> {
        let mut descriptors = self.descriptors.iter();
        #[allow(clippy::redundant_slicing)] // Clippy can't see why this is needed.
        let mut buf = &self.buffer[..];

        core::iter::from_fn(move || {
            let mut chunk_size = 0;
            let mut skip_size = 0;
            while let Some(desc) = descriptors.next() {
                chunk_size += desc.len();
                skip_size += desc.flags.size() as usize;

                // If this is the end of the linked list, we can skip the remaining descriptors.
                if desc.next.is_null() {
                    while descriptors.next().is_some() {
                        // Drain the iterator so the next call to from_fn return
                        // None.
                    }
                    break;
                }

                // This typically happens when the DMA gets an EOF bit from the peripheral.
                // It can also happen if the DMA is restarted.
                if desc.len() < desc.flags.size() as usize {
                    break;
                }
            }

            if chunk_size == 0 {
                return None;
            }

            let chunk = &buf[..chunk_size];
            buf = &buf[skip_size..];
            Some(chunk)
        })
    }

    pub(crate) fn first(&self) -> *mut DmaDescriptor {
        self.descriptors.as_ptr() as _
    }
}

/// DMA transmit and receive buffer.
///
/// This is a (single) contiguous buffer linked together by two sets of DMA
/// descriptors of length 4092 each.
/// It can be used for simultaneously transmitting to and receiving from a
/// peripheral's FIFO. These are typically full-duplex transfers.
pub struct DmaTxRxBuf {
    tx_descriptors: &'static mut [DmaDescriptor],
    rx_descriptors: &'static mut [DmaDescriptor],
    buffer: &'static mut [u8],
}

impl DmaTxRxBuf {
    /// Creates a new [DmaTxRxBuf] from some descriptors and a buffer.
    ///
    /// There must be enough descriptors for the provided buffer.
    /// Each descriptor can handle 4092 bytes worth of buffer.
    ///
    /// Both the descriptors and buffer must be in DMA-capable memory.
    /// Only DRAM is supported.
    pub fn new(
        tx_descriptors: &'static mut [DmaDescriptor],
        rx_descriptors: &'static mut [DmaDescriptor],
        buffer: &'static mut [u8],
    ) -> Result<Self, DmaBufError> {
        let min_descriptors = buffer.len().div_ceil(CHUNK_SIZE);
        if tx_descriptors.len() < min_descriptors {
            return Err(DmaBufError::InsufficientDescriptors);
        }
        if rx_descriptors.len() < min_descriptors {
            return Err(DmaBufError::InsufficientDescriptors);
        }

        if !is_slice_in_dram(tx_descriptors)
            || !is_slice_in_dram(rx_descriptors)
            || !is_slice_in_dram(buffer)
        {
            return Err(DmaBufError::UnsupportedMemoryRegion);
        }

        // Reset the provided descriptors
        tx_descriptors.fill(DmaDescriptor::EMPTY);
        rx_descriptors.fill(DmaDescriptor::EMPTY);

        let descriptors = tx_descriptors.iter_mut().zip(rx_descriptors.iter_mut());
        let chunks = buffer.chunks_mut(CHUNK_SIZE);

        for ((tx_desc, rx_desc), chunk) in descriptors.zip(chunks) {
            tx_desc.set_size(chunk.len());
            tx_desc.buffer = chunk.as_mut_ptr();
            rx_desc.set_size(chunk.len());
            rx_desc.buffer = chunk.as_mut_ptr();
        }

        let mut buf = Self {
            tx_descriptors,
            rx_descriptors,
            buffer,
        };
        buf.set_length(buf.capacity());

        Ok(buf)
    }

    /// Consume the buf, returning the tx descriptors, rx descriptors and
    /// buffer.
    pub fn split(
        self,
    ) -> (
        &'static mut [DmaDescriptor],
        &'static mut [DmaDescriptor],
        &'static mut [u8],
    ) {
        (self.tx_descriptors, self.rx_descriptors, self.buffer)
    }

    /// Return the size of the underlying buffer.
    pub fn capacity(&self) -> usize {
        self.buffer.len()
    }

    /// Returns the entire buf as a slice than can be read.
    pub fn as_slice(&self) -> &[u8] {
        self.buffer
    }

    /// Returns the entire buf as a slice than can be written.
    pub fn as_slice_mut(&mut self) -> &mut [u8] {
        &mut self.buffer[..]
    }

    /// Reset the descriptors to only transmit/receive `len` amount of bytes
    /// with this buf.
    ///
    /// `len` must be less than or equal to the buffer size.
    pub fn set_length(&mut self, len: usize) {
        assert!(len <= self.buffer.len());

        // Get the minimum number of descriptors needed for this length of data.
        let descriptor_count = len.div_ceil(CHUNK_SIZE).max(1);

        let relevant_tx_descriptors = &mut self.tx_descriptors[..descriptor_count];
        let relevant_rx_descriptors = &mut self.rx_descriptors[..descriptor_count];

        // Link up the relevant descriptors.
        for descriptors in [
            &mut relevant_tx_descriptors[..],
            &mut relevant_rx_descriptors[..],
        ] {
            let mut next = core::ptr::null_mut();
            for desc in descriptors.iter_mut().rev() {
                desc.next = next;
                next = desc;
            }
        }

        let mut remaining_length = len;
        for desc in relevant_tx_descriptors.iter_mut() {
            // As this is a simple dma buffer implementation we won't
            // be making use of this feature.
            desc.set_suc_eof(false);

            // This isn't strictly needed for this simple implementation,
            // but it is useful for debugging.
            desc.set_owner(Owner::Dma);

            let chunk_size = min(desc.size(), remaining_length);
            desc.set_length(chunk_size);
            remaining_length -= chunk_size;
        }
        debug_assert_eq!(remaining_length, 0);
        relevant_tx_descriptors
            .last_mut()
            .unwrap()
            .set_suc_eof(true);

        let mut remaining_length = len;
        for desc in relevant_rx_descriptors.iter_mut() {
            // Clear this to allow hardware to set it when the peripheral returns an EOF
            // bit.
            desc.set_suc_eof(false);

            // This isn't strictly needed for this simple implementation,
            // but it is useful for debugging.
            desc.set_owner(Owner::Dma);

            // Clear this to allow hardware to set it when it is
            // done receiving data for this descriptor.
            desc.set_length(0);

            let chunk_size = min(CHUNK_SIZE, remaining_length);
            desc.set_size(chunk_size);
            remaining_length -= chunk_size;
        }
        debug_assert_eq!(remaining_length, 0);
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
        fn peripheral_wait_dma(&mut self, is_tx: bool, is_rx: bool);

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
///
/// # Safety
///
/// Never use [core::mem::forget] on an in-progress transfer
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
    #[allow(dead_code)]
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

#[cfg(feature = "async")]
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
            } else if self.tx.has_error() {
                self.tx.clear_interrupts();
                Poll::Ready(Err(DmaError::DescriptorError))
            } else {
                self.tx.listen_eof();
                self.tx.listen_out_descriptor_error();
                Poll::Pending
            }
        }
    }

    impl<'a, TX> Drop for DmaTxFuture<'a, TX>
    where
        TX: Tx,
    {
        fn drop(&mut self) {
            self.tx.unlisten_eof();
            self.tx.unlisten_out_descriptor_error();
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

        #[allow(dead_code)] // Dead on the C2
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
            if self.rx.is_done() {
                self.rx.clear_interrupts();
                Poll::Ready(Ok(()))
            } else if self.rx.has_error()
                || self.rx.has_dscr_empty_error()
                || self.rx.has_eof_error()
            {
                self.rx.clear_interrupts();
                Poll::Ready(Err(DmaError::DescriptorError))
            } else {
                self.rx.listen_eof();
                self.rx.listen_in_descriptor_error();
                self.rx.listen_in_descriptor_error_dscr_empty();
                self.rx.listen_in_descriptor_error_err_eof();
                Poll::Pending
            }
        }
    }

    impl<'a, RX> Drop for DmaRxFuture<'a, RX>
    where
        RX: Rx,
    {
        fn drop(&mut self) {
            self.rx.unlisten_eof();
            self.rx.unlisten_in_descriptor_error();
            self.rx.unlisten_in_descriptor_error_dscr_empty();
            self.rx.unlisten_in_descriptor_error_err_eof();
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
            if self.tx.is_ch_out_done_set() {
                self.tx.clear_ch_out_done();
                Poll::Ready(Ok(()))
            } else if self.tx.has_error() {
                self.tx.clear_interrupts();
                Poll::Ready(Err(DmaError::DescriptorError))
            } else {
                self.tx.listen_ch_out_done();
                self.tx.listen_out_descriptor_error();
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
            self.tx.unlisten_ch_out_done();
            self.tx.unlisten_out_descriptor_error();
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
            if self.rx.is_ch_in_done_set() {
                self.rx.clear_ch_in_done();
                Poll::Ready(Ok(()))
            } else if self.rx.has_error()
                || self.rx.has_dscr_empty_error()
                || self.rx.has_eof_error()
            {
                self.rx.clear_interrupts();
                Poll::Ready(Err(DmaError::DescriptorError))
            } else {
                self.rx.listen_ch_in_done();
                self.rx.listen_in_descriptor_error();
                self.rx.listen_in_descriptor_error_dscr_empty();
                self.rx.listen_in_descriptor_error_err_eof();
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
            self.rx.unlisten_ch_in_done();
            self.rx.unlisten_in_descriptor_error();
            self.rx.unlisten_in_descriptor_error_dscr_empty();
            self.rx.unlisten_in_descriptor_error_err_eof();
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
            Channel::unlisten_in_eof();
            Rx::waker().wake()
        }

        if Channel::is_ch_in_done_set() {
            Channel::unlisten_ch_in_done();
            Rx::waker().wake()
        }

        if Channel::is_out_done() && Channel::is_listening_out_eof() {
            Channel::unlisten_out_eof();
            Tx::waker().wake()
        }

        if Channel::is_ch_out_done_set() {
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
