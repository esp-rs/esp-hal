//! # Direct Memory Access Commons
//!
//! ## Overview
//!
//! The `DMA` driver provides an interface to efficiently transfer data between
//! different memory regions within the ESP microcontroller without involving
//! the CPU. The `Direct Memory Access` (DMA) controller is a hardware
//! block responsible for managing these data transfers.
//!
//! The driver is organized into several components and traits, each responsible
//! for handling specific functionalities of the `DMA` controller. Below is an
//! overview of the main components and their functionalities:
//!   * `Tx` and `Rx` traits:
//!      - These traits define the behaviors and functionalities required for
//!        DMA transmit and receive operations.<br> The `Tx` trait includes
//!        functions to start, stop, and check the completion status of an
//!        outbound DMA transfer.<br> On the other hand, the Rx trait provides
//!        similar functionalities for inbound DMA transfers.
//!   * `DmaTransfer` and `DmaTransferRxTx` traits:
//!      - The `DmaTransfer` trait and `DmaTransferRxTx` trait are used for
//!        in-progress DMA transfers.<br> They allow waiting for the transfer to
//!        complete and checking its status. Additionally, the `DmaTransferRxTx`
//!        trait extends the functionalities to support both receive and
//!        transmit operations in a single trait.
//!   * `RegisterAccess` trait:
//!      - This trait defines a set of methods that allow low-level access to
//!        the DMA controller's registers.<br> It provides functions to
//!        initialize DMA channels, configure burst mode, priority, and
//!        peripheral for both input and output data transfers.<br>Additionally,
//!        it supports clearing interrupts, resetting channels, setting
//!        descriptor addresses, and checking for descriptor errors.
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
//! let dma = Gdma::new(peripherals.DMA);
//! let dma_channel = dma.channel0;
//!
//! // For `ESP32` and `ESP32-S2` chips use `pdma::Dma` instead:
//! // let dma = Dma::new(system.dma);
//! // let dma_channel = dma.spi2channel;
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
//!     100u32.kHz(),
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

use core::{marker::PhantomData, ptr::addr_of_mut, sync::atomic::compiler_fence};

bitfield::bitfield! {
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
    flags: DmaDescriptorFlags,
    buffer: *mut u8,
    next: *mut DmaDescriptor,
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

#[cfg(gdma)]
pub mod gdma;
#[cfg(pdma)]
pub mod pdma;

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

/// DMA Errors
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DmaError {
    InvalidAlignment,
    OutOfDescriptors,
    InvalidDescriptorSize,
    /// DescriptorError the DMA rejected the descriptor configuration. This
    /// could be because the source address of the data is not in RAM. Ensure
    /// your source data is in a valid address space, or try using
    /// [`crate::FlashSafeDma`] wrapper.
    DescriptorError,
    Overflow,
    Exhausted,
    BufferTooSmall,
}

/// DMA Priorities
#[cfg(gdma)]
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
pub enum DmaPriority {
    Priority0 = 0,
}

/// DMA capable peripherals
/// The values need to match the TRM
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
pub trait SpiPeripheral: PeripheralMarker {}

/// Marks channels as useable for SPI2
pub trait Spi2Peripheral: SpiPeripheral + PeripheralMarker {}

/// Marks channels as useable for SPI3
#[cfg(any(esp32, esp32s2, esp32s3))]
pub trait Spi3Peripheral: SpiPeripheral + PeripheralMarker {}

/// Marks channels as useable for I2S
pub trait I2sPeripheral: PeripheralMarker {}

/// Marks channels as useable for I2S0
pub trait I2s0Peripheral: I2sPeripheral + PeripheralMarker {}

/// Marks channels as useable for I2S1
pub trait I2s1Peripheral: I2sPeripheral + PeripheralMarker {}

/// Marks channels as useable for PARL_IO
pub trait ParlIoPeripheral: PeripheralMarker {}

/// Marks channels as useable for AES
pub trait AesPeripheral: PeripheralMarker {}

/// DMA Rx
pub trait Rx: RxPrivate {}

/// DMA Tx
pub trait Tx: TxPrivate {}

/// Marker trait
pub trait PeripheralMarker {}

/// The functions here are not meant to be used outside the HAL
pub trait RxPrivate {
    fn init(&mut self, burst_mode: bool, priority: DmaPriority);

    fn init_channel(&mut self);

    fn prepare_transfer_without_start(
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

    #[cfg(feature = "async")]
    fn waker() -> &'static embassy_sync::waitqueue::AtomicWaker;
}

pub trait RxChannel<R>
where
    R: RegisterAccess,
{
    fn init(&mut self, burst_mode: bool, priority: DmaPriority) {
        R::set_in_burstmode(burst_mode);
        R::set_in_priority(priority);
    }

    fn prepare_transfer_without_start(
        &mut self,
        descriptors: &mut [DmaDescriptor],
        circular: bool,
        peri: DmaPeripheral,
        data: *mut u8,
        len: usize,
    ) -> Result<(), DmaError> {
        descriptors.fill(DmaDescriptor::EMPTY);

        compiler_fence(core::sync::atomic::Ordering::SeqCst);

        let mut processed = 0;
        let mut descr = 0;
        loop {
            let chunk_size = usize::min(CHUNK_SIZE, len - processed);
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
pub struct ChannelRx<'a, T, R>
where
    T: RxChannel<R>,
    R: RegisterAccess,
{
    pub descriptors: &'a mut [DmaDescriptor],
    pub burst_mode: bool,
    pub rx_impl: T,
    pub read_descr_ptr: *mut DmaDescriptor,
    pub available: usize,
    pub last_seen_handled_descriptor_ptr: *mut DmaDescriptor,
    pub read_buffer_start: *mut u8,
    pub _phantom: PhantomData<R>,
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
            _phantom: PhantomData::default(),
        }
    }
}

impl<'a, T, R> Rx for ChannelRx<'a, T, R>
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

    fn prepare_transfer_without_start(
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

        if circular && len < CHUNK_SIZE * 2 {
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

            // Copy data to desination
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

    #[cfg(feature = "async")]
    fn waker() -> &'static embassy_sync::waitqueue::AtomicWaker {
        T::waker()
    }
}

/// The functions here are not meant to be used outside the HAL
pub trait TxPrivate {
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

    fn available(&mut self) -> usize;

    fn has_error(&self) -> bool;

    fn push(&mut self, data: &[u8]) -> Result<usize, DmaError>;

    #[cfg(feature = "async")]
    fn waker() -> &'static embassy_sync::waitqueue::AtomicWaker;
}

pub trait TxChannel<R>
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

        let mut processed = 0;
        let mut descr = 0;
        loop {
            let chunk_size = usize::min(CHUNK_SIZE, len - processed);
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
pub struct ChannelTx<'a, T, R>
where
    T: TxChannel<R>,
    R: RegisterAccess,
{
    pub descriptors: &'a mut [DmaDescriptor],
    #[allow(unused)]
    pub burst_mode: bool,
    pub tx_impl: T,
    pub write_offset: usize,
    pub write_descr_ptr: *mut DmaDescriptor,
    pub available: usize,
    pub last_seen_handled_descriptor_ptr: *mut DmaDescriptor,
    pub buffer_start: *const u8,
    pub buffer_len: usize,
    pub _phantom: PhantomData<R>,
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
            _phantom: PhantomData::default(),
        }
    }
}

impl<'a, T, R> Tx for ChannelTx<'a, T, R>
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

        if circular && len < CHUNK_SIZE * 2 {
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
                    while !(*ptr).next.is_null() {
                        let dw0 = ptr.read_volatile();
                        self.available += dw0.len();
                        ptr = ptr.offset(1);
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

        unsafe {
            let src = data.as_ptr();
            let dst = self.buffer_start.add(self.write_offset).cast_mut();
            let count = usize::min(data.len(), self.buffer_len - self.write_offset);
            core::ptr::copy_nonoverlapping(src, dst, count);
        }

        if self.write_offset + data.len() >= self.buffer_len {
            let remainder = (self.write_offset + data.len()) % self.buffer_len;
            let dst = self.buffer_start.cast_mut();
            unsafe {
                let src = data.as_ptr().add(data.len() - remainder);
                core::ptr::copy_nonoverlapping(src, dst, remainder);
            }
        }

        let mut forward = data.len();
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

        self.write_offset = (self.write_offset + data.len()) % self.buffer_len;
        self.available -= data.len();

        Ok(data.len())
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
}

pub trait RegisterAccess {
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

    fn listen_ch_in_done();
    fn clear_ch_in_done();
    fn is_ch_in_done_set() -> bool;
    fn unlisten_ch_in_done();
    fn is_listening_ch_in_done() -> bool;
}

pub trait ChannelTypes {
    type P: PeripheralMarker;
    type Tx<'a>: Tx;
    type Rx<'a>: Rx;
}

/// DMA Channel
pub struct Channel<'d, C>
where
    C: ChannelTypes,
{
    pub(crate) tx: C::Tx<'d>,
    pub(crate) rx: C::Rx<'d>,
}

/// Trait to be implemented for an in progress dma transfer.
#[allow(drop_bounds)]
pub trait DmaTransfer<B, T>: Drop {
    /// Wait for the transfer to finish.
    fn wait(self) -> Result<(B, T), (DmaError, B, T)>;
    /// Check if the transfer is finished.
    fn is_done(&self) -> bool;
}

/// Trait to be implemented for an in progress dma transfer.
#[allow(drop_bounds)]
pub trait DmaTransferRxTx<BR, BT, T>: Drop {
    /// Wait for the transfer to finish.
    fn wait(self) -> Result<(BR, BT, T), (DmaError, BR, BT, T)>;
    /// Check if the transfer is finished.
    fn is_done(&self) -> bool;
}

#[cfg(feature = "async")]
pub(crate) mod asynch {
    use core::task::Poll;

    use super::*;
    use crate::macros::interrupt;

    pub struct DmaTxFuture<'a, TX> {
        pub(crate) tx: &'a mut TX,
        _a: (),
    }

    impl<'a, TX> DmaTxFuture<'a, TX>
    where
        TX: Tx,
    {
        pub fn new(tx: &'a mut TX) -> Self {
            Self { tx, _a: () }
        }
    }

    impl<'a, TX> core::future::Future for DmaTxFuture<'a, TX>
    where
        TX: Tx,
    {
        type Output = (); // TODO handle DMA errors

        fn poll(
            self: core::pin::Pin<&mut Self>,
            cx: &mut core::task::Context<'_>,
        ) -> Poll<Self::Output> {
            TX::waker().register(cx.waker());
            if self.tx.is_listening_eof() {
                Poll::Pending
            } else {
                Poll::Ready(())
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
            Self { rx, _a: () }
        }
    }

    impl<'a, RX> core::future::Future for DmaRxFuture<'a, RX>
    where
        RX: Rx,
    {
        type Output = (); // TODO handle DMA errors

        fn poll(
            self: core::pin::Pin<&mut Self>,
            cx: &mut core::task::Context<'_>,
        ) -> Poll<Self::Output> {
            RX::waker().register(cx.waker());
            if self.rx.is_listening_eof() {
                Poll::Pending
            } else {
                Poll::Ready(())
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
            Self { tx, _a: () }
        }
    }

    #[cfg(any(i2s0, i2s1))]
    impl<'a, TX> core::future::Future for DmaTxDoneChFuture<'a, TX>
    where
        TX: Tx,
    {
        type Output = (); // TODO handle DMA errors

        fn poll(
            self: core::pin::Pin<&mut Self>,
            cx: &mut core::task::Context<'_>,
        ) -> Poll<Self::Output> {
            TX::waker().register(cx.waker());
            if self.tx.is_listening_ch_out_done() {
                Poll::Pending
            } else {
                Poll::Ready(())
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
            Self { rx, _a: () }
        }
    }

    #[cfg(any(i2s0, i2s1))]
    impl<'a, RX> core::future::Future for DmaRxDoneChFuture<'a, RX>
    where
        RX: Rx,
    {
        type Output = (); // TODO handle DMA errors

        fn poll(
            self: core::pin::Pin<&mut Self>,
            cx: &mut core::task::Context<'_>,
        ) -> Poll<Self::Output> {
            RX::waker().register(cx.waker());
            if self.rx.is_listening_ch_in_done() {
                Poll::Pending
            } else {
                Poll::Ready(())
            }
        }
    }

    #[cfg(esp32c2)]
    mod interrupt {
        use super::*;

        #[interrupt]
        fn DMA_CH0() {
            use crate::dma::gdma::{
                Channel0 as Channel,
                Channel0RxImpl as ChannelRxImpl,
                Channel0TxImpl as ChannelTxImpl,
            };

            if Channel::is_in_done() && Channel::is_listening_in_eof() {
                Channel::clear_in_interrupts();
                Channel::unlisten_in_eof();
                ChannelRxImpl::waker().wake()
            }

            if Channel::is_out_done() {
                Channel::clear_out_interrupts();
                Channel::unlisten_out_eof();
                ChannelTxImpl::waker().wake()
            }

            if Channel::is_ch_out_done_set() {
                Channel::clear_ch_out_done();
                Channel::unlisten_ch_out_done();
                ChannelTxImpl::waker().wake()
            }
        }
    }

    #[cfg(esp32c3)]
    mod interrupt {
        use super::*;

        #[interrupt]
        fn DMA_CH0() {
            use crate::dma::gdma::{
                Channel0 as Channel,
                Channel0RxImpl as ChannelRxImpl,
                Channel0TxImpl as ChannelTxImpl,
            };

            if Channel::is_in_done() && Channel::is_listening_in_eof() {
                Channel::clear_in_interrupts();
                Channel::unlisten_in_eof();
                ChannelRxImpl::waker().wake()
            }

            if Channel::is_out_done() {
                Channel::clear_out_interrupts();
                Channel::unlisten_out_eof();
                ChannelTxImpl::waker().wake()
            }

            if Channel::is_ch_out_done_set() {
                Channel::clear_ch_out_done();
                Channel::unlisten_ch_out_done();
                ChannelTxImpl::waker().wake()
            }

            if Channel::is_ch_in_done_set() {
                Channel::clear_ch_in_done();
                Channel::unlisten_ch_in_done();
                ChannelRxImpl::waker().wake()
            }
        }

        #[interrupt]
        fn DMA_CH1() {
            use crate::dma::gdma::{
                Channel1 as Channel,
                Channel1RxImpl as ChannelRxImpl,
                Channel1TxImpl as ChannelTxImpl,
            };

            if Channel::is_in_done() && Channel::is_listening_in_eof() {
                Channel::clear_in_interrupts();
                Channel::unlisten_in_eof();
                ChannelRxImpl::waker().wake()
            }

            if Channel::is_out_done() {
                Channel::clear_out_interrupts();
                Channel::unlisten_out_eof();
                ChannelTxImpl::waker().wake()
            }

            if Channel::is_ch_out_done_set() {
                Channel::clear_ch_out_done();
                Channel::unlisten_ch_out_done();
                ChannelTxImpl::waker().wake()
            }

            if Channel::is_ch_in_done_set() {
                Channel::clear_ch_in_done();
                Channel::unlisten_ch_in_done();
                ChannelRxImpl::waker().wake()
            }
        }

        #[interrupt]
        fn DMA_CH2() {
            use crate::dma::gdma::{
                Channel2 as Channel,
                Channel2RxImpl as ChannelRxImpl,
                Channel2TxImpl as ChannelTxImpl,
            };

            if Channel::is_in_done() && Channel::is_listening_in_eof() {
                Channel::clear_in_interrupts();
                Channel::unlisten_in_eof();
                ChannelRxImpl::waker().wake()
            }

            if Channel::is_out_done() {
                Channel::clear_out_interrupts();
                Channel::unlisten_out_eof();
                ChannelTxImpl::waker().wake()
            }

            if Channel::is_ch_out_done_set() {
                Channel::clear_ch_out_done();
                Channel::unlisten_ch_out_done();
                ChannelTxImpl::waker().wake()
            }

            if Channel::is_ch_in_done_set() {
                Channel::clear_ch_in_done();
                Channel::unlisten_ch_in_done();
                ChannelRxImpl::waker().wake()
            }
        }
    }

    #[cfg(any(esp32c6, esp32h2))]
    mod interrupt {
        use super::*;

        #[interrupt]
        fn DMA_IN_CH0() {
            use crate::dma::gdma::{Channel0 as Channel, Channel0RxImpl as ChannelRxImpl};

            if Channel::is_in_done() && Channel::is_listening_in_eof() {
                Channel::clear_in_interrupts();
                Channel::unlisten_in_eof();
                ChannelRxImpl::waker().wake()
            }

            if Channel::is_ch_in_done_set() {
                Channel::clear_ch_in_done();
                Channel::unlisten_ch_in_done();
                ChannelRxImpl::waker().wake()
            }
        }

        #[interrupt]
        fn DMA_OUT_CH0() {
            use crate::dma::gdma::{Channel0 as Channel, Channel0TxImpl as ChannelTxImpl};

            if Channel::is_out_done() {
                Channel::clear_out_interrupts();
                Channel::unlisten_out_eof();
                ChannelTxImpl::waker().wake()
            }

            if Channel::is_ch_out_done_set() {
                Channel::clear_ch_out_done();
                Channel::unlisten_ch_out_done();
                ChannelTxImpl::waker().wake()
            }
        }

        #[interrupt]
        fn DMA_IN_CH1() {
            use crate::dma::gdma::{Channel1 as Channel, Channel1RxImpl as ChannelRxImpl};

            if Channel::is_in_done() && Channel::is_listening_in_eof() {
                Channel::clear_in_interrupts();
                Channel::unlisten_in_eof();
                ChannelRxImpl::waker().wake()
            }

            if Channel::is_ch_in_done_set() {
                Channel::clear_ch_in_done();
                Channel::unlisten_ch_in_done();
                ChannelRxImpl::waker().wake()
            }
        }

        #[interrupt]
        fn DMA_OUT_CH1() {
            use crate::dma::gdma::{Channel1 as Channel, Channel1TxImpl as ChannelTxImpl};

            if Channel::is_out_done() {
                Channel::clear_out_interrupts();
                Channel::unlisten_out_eof();
                ChannelTxImpl::waker().wake()
            }

            if Channel::is_ch_out_done_set() {
                Channel::clear_ch_out_done();
                Channel::unlisten_ch_out_done();
                ChannelTxImpl::waker().wake()
            }
        }

        #[interrupt]
        fn DMA_IN_CH2() {
            use crate::dma::gdma::{Channel2 as Channel, Channel2RxImpl as ChannelRxImpl};

            if Channel::is_in_done() && Channel::is_listening_in_eof() {
                Channel::clear_in_interrupts();
                Channel::unlisten_in_eof();
                ChannelRxImpl::waker().wake()
            }

            if Channel::is_ch_in_done_set() {
                Channel::clear_ch_in_done();
                Channel::unlisten_ch_in_done();
                ChannelRxImpl::waker().wake()
            }
        }

        #[interrupt]
        fn DMA_OUT_CH2() {
            use crate::dma::gdma::{Channel2 as Channel, Channel2TxImpl as ChannelTxImpl};

            if Channel::is_out_done() {
                Channel::clear_out_interrupts();
                Channel::unlisten_out_eof();
                ChannelTxImpl::waker().wake()
            }

            if Channel::is_ch_out_done_set() {
                Channel::clear_ch_out_done();
                Channel::unlisten_ch_out_done();
                ChannelTxImpl::waker().wake()
            }
        }
    }

    #[cfg(esp32s3)]
    mod interrupt {
        use super::*;

        #[interrupt]
        fn DMA_IN_CH0() {
            use crate::dma::gdma::{Channel0 as Channel, Channel0RxImpl as ChannelRxImpl};

            if Channel::is_in_done() && Channel::is_listening_in_eof() {
                Channel::clear_in_interrupts();
                Channel::unlisten_in_eof();
                ChannelRxImpl::waker().wake()
            }

            if Channel::is_ch_in_done_set() {
                Channel::clear_ch_in_done();
                Channel::unlisten_ch_in_done();
                ChannelRxImpl::waker().wake()
            }
        }

        #[interrupt]
        fn DMA_OUT_CH0() {
            use crate::dma::gdma::{Channel0 as Channel, Channel0TxImpl as ChannelTxImpl};

            if Channel::is_out_done() {
                Channel::clear_out_interrupts();
                Channel::unlisten_out_eof();
                ChannelTxImpl::waker().wake()
            }

            if Channel::is_ch_out_done_set() {
                Channel::clear_ch_out_done();
                Channel::unlisten_ch_out_done();
                ChannelTxImpl::waker().wake()
            }
        }

        #[interrupt]
        fn DMA_IN_CH1() {
            use crate::dma::gdma::{Channel1 as Channel, Channel1RxImpl as ChannelRxImpl};

            if Channel::is_in_done() && Channel::is_listening_in_eof() {
                Channel::clear_in_interrupts();
                Channel::unlisten_in_eof();
                ChannelRxImpl::waker().wake()
            }

            if Channel::is_ch_in_done_set() {
                Channel::clear_ch_in_done();
                Channel::unlisten_ch_in_done();
                ChannelRxImpl::waker().wake()
            }
        }

        #[interrupt]
        fn DMA_OUT_CH1() {
            use crate::dma::gdma::{Channel1 as Channel, Channel1TxImpl as ChannelTxImpl};

            if Channel::is_out_done() {
                Channel::clear_out_interrupts();
                Channel::unlisten_out_eof();
                ChannelTxImpl::waker().wake()
            }

            if Channel::is_ch_out_done_set() {
                Channel::clear_ch_out_done();
                Channel::unlisten_ch_out_done();
                ChannelTxImpl::waker().wake()
            }
        }

        #[interrupt]
        fn DMA_IN_CH3() {
            use crate::dma::gdma::{Channel3 as Channel, Channel3RxImpl as ChannelRxImpl};

            if Channel::is_in_done() && Channel::is_listening_in_eof() {
                Channel::clear_in_interrupts();
                Channel::unlisten_in_eof();
                ChannelRxImpl::waker().wake()
            }

            if Channel::is_ch_in_done_set() {
                Channel::clear_ch_in_done();
                Channel::unlisten_ch_in_done();
                ChannelRxImpl::waker().wake()
            }
        }

        #[interrupt]
        fn DMA_OUT_CH3() {
            use crate::dma::gdma::{Channel3 as Channel, Channel3TxImpl as ChannelTxImpl};

            if Channel::is_out_done() {
                Channel::clear_out_interrupts();
                Channel::unlisten_out_eof();
                ChannelTxImpl::waker().wake()
            }

            if Channel::is_ch_out_done_set() {
                Channel::clear_ch_out_done();
                Channel::unlisten_ch_out_done();
                ChannelTxImpl::waker().wake()
            }
        }

        #[interrupt]
        fn DMA_IN_CH4() {
            use crate::dma::gdma::{Channel4 as Channel, Channel4RxImpl as ChannelRxImpl};

            if Channel::is_in_done() && Channel::is_listening_in_eof() {
                Channel::clear_in_interrupts();
                Channel::unlisten_in_eof();
                ChannelRxImpl::waker().wake()
            }

            if Channel::is_ch_in_done_set() {
                Channel::clear_ch_in_done();
                Channel::unlisten_ch_in_done();
                ChannelRxImpl::waker().wake()
            }
        }

        #[interrupt]
        fn DMA_OUT_CH4() {
            use crate::dma::gdma::{Channel4 as Channel, Channel4TxImpl as ChannelTxImpl};

            if Channel::is_out_done() {
                Channel::clear_out_interrupts();
                Channel::unlisten_out_eof();
                ChannelTxImpl::waker().wake()
            }

            if Channel::is_ch_out_done_set() {
                Channel::clear_ch_out_done();
                Channel::unlisten_ch_out_done();
                ChannelTxImpl::waker().wake()
            }
        }
    }

    #[cfg(any(esp32, esp32s2))]
    mod interrupt {
        use super::*;

        #[interrupt]
        fn SPI2_DMA() {
            use crate::dma::pdma::{
                Spi2DmaChannel as Channel,
                Spi2DmaChannelRxImpl as ChannelRxImpl,
                Spi2DmaChannelTxImpl as ChannelTxImpl,
            };

            if Channel::is_in_done() && Channel::is_listening_in_eof() {
                Channel::clear_in_interrupts();
                Channel::unlisten_in_eof();
                ChannelRxImpl::waker().wake()
            }

            if Channel::is_out_done() {
                Channel::clear_out_interrupts();
                Channel::unlisten_out_eof();
                ChannelTxImpl::waker().wake()
            }

            if Channel::is_ch_out_done_set() {
                Channel::clear_ch_out_done();
                Channel::unlisten_ch_out_done();
                ChannelTxImpl::waker().wake()
            }

            if Channel::is_ch_in_done_set() {
                Channel::clear_ch_in_done();
                Channel::unlisten_ch_in_done();
                ChannelRxImpl::waker().wake()
            }
        }

        #[interrupt]
        fn SPI3_DMA() {
            use crate::dma::pdma::{
                Spi3DmaChannel as Channel,
                Spi3DmaChannelRxImpl as ChannelRxImpl,
                Spi3DmaChannelTxImpl as ChannelTxImpl,
            };

            if Channel::is_in_done() && Channel::is_listening_in_eof() {
                Channel::clear_in_interrupts();
                Channel::unlisten_in_eof();
                ChannelRxImpl::waker().wake()
            }

            if Channel::is_out_done() {
                Channel::clear_out_interrupts();
                Channel::unlisten_out_eof();
                ChannelTxImpl::waker().wake()
            }

            if Channel::is_ch_out_done_set() {
                Channel::clear_ch_out_done();
                Channel::unlisten_ch_out_done();
                ChannelTxImpl::waker().wake()
            }

            if Channel::is_ch_in_done_set() {
                Channel::clear_ch_in_done();
                Channel::unlisten_ch_in_done();
                ChannelRxImpl::waker().wake()
            }
        }

        #[interrupt]
        fn I2S0() {
            use crate::dma::pdma::{
                I2s0DmaChannel as Channel,
                I2s0DmaChannelRxImpl as ChannelRxImpl,
                I2s0DmaChannelTxImpl as ChannelTxImpl,
            };

            if Channel::is_in_done() && Channel::is_listening_in_eof() {
                Channel::clear_in_interrupts();
                Channel::unlisten_in_eof();
                ChannelRxImpl::waker().wake()
            }

            if Channel::is_out_done() && Channel::is_listening_out_eof() {
                Channel::clear_out_interrupts();
                Channel::unlisten_out_eof();
                ChannelTxImpl::waker().wake()
            }

            if Channel::is_ch_out_done_set() {
                Channel::clear_ch_out_done();
                Channel::unlisten_ch_out_done();
                ChannelTxImpl::waker().wake()
            }

            if Channel::is_ch_in_done_set() {
                Channel::clear_ch_in_done();
                Channel::unlisten_ch_in_done();
                ChannelRxImpl::waker().wake()
            }
        }

        #[cfg(esp32)]
        #[interrupt]
        fn I2S1() {
            use crate::dma::pdma::{
                I2s1DmaChannel as Channel,
                I2s1DmaChannelRxImpl as ChannelRxImpl,
                I2s1DmaChannelTxImpl as ChannelTxImpl,
            };

            if Channel::is_in_done() && Channel::is_listening_in_eof() {
                Channel::clear_in_interrupts();
                Channel::unlisten_in_eof();
                ChannelRxImpl::waker().wake()
            }

            if Channel::is_out_done() && Channel::is_listening_out_eof() {
                Channel::clear_out_interrupts();
                Channel::unlisten_out_eof();
                ChannelTxImpl::waker().wake()
            }

            if Channel::is_ch_out_done_set() {
                Channel::clear_ch_out_done();
                Channel::unlisten_ch_out_done();
                ChannelTxImpl::waker().wake()
            }

            if Channel::is_ch_in_done_set() {
                Channel::clear_ch_in_done();
                Channel::unlisten_ch_in_done();
                ChannelRxImpl::waker().wake()
            }
        }
    }
}
