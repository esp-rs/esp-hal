//! Direct Memory Access Commons

use core::{marker::PhantomData, sync::atomic::compiler_fence};

use private::*;

#[cfg(gdma)]
pub mod gdma;
#[cfg(pdma)]
pub mod pdma;

const CHUNK_SIZE: usize = 4092;

/// DMA Errors
#[derive(Debug, Clone, Copy)]
pub enum DmaError {
    InvalidAlignment,
    OutOfDescriptors,
    InvalidDescriptorSize,
    DescriptorError,
    Overflow,
    Exhausted,
    BufferTooSmall,
}

/// DMA Priorities
#[cfg(gdma)]
#[derive(Clone, Copy)]
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
#[derive(Clone, Copy)]
pub enum DmaPriority {
    Priority0 = 0,
}

/// DMA capable peripherals
/// The values need to match the TRM
#[derive(Clone, Copy)]
pub enum DmaPeripheral {
    Spi2   = 0,
    #[cfg(any(pdma, esp32s3))]
    Spi3   = 1,
    #[cfg(any(esp32c3, esp32s3))]
    Uhci0  = 2,
    #[cfg(any(esp32, esp32s2, esp32c3, esp32s3))]
    I2s0   = 3,
    #[cfg(any(esp32, esp32s3))]
    I2s1   = 4,
    #[cfg(esp32s3)]
    LcdCam = 5,
    #[cfg(any(esp32c3, esp32s3))]
    Aes    = 6,
    #[cfg(gdma)]
    Sha    = 7,
    #[cfg(any(esp32c3, esp32s3))]
    Adc    = 8,
    #[cfg(esp32s3)]
    Rmt    = 9,
}

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

trait DmaLinkedListDw0 {
    fn set_size(&mut self, len: u16);
    fn get_size(&mut self) -> u16;
    fn set_length(&mut self, len: u16);
    fn get_length(&mut self) -> u16;
    fn set_err_eof(&mut self, err_eof: bool);
    #[cfg(not(esp32))]
    fn get_err_eof(&mut self) -> bool;
    fn set_suc_eof(&mut self, suc_eof: bool);
    fn get_suc_eof(&mut self) -> bool;
    fn set_owner(&mut self, owner: Owner);
    fn get_owner(&mut self) -> Owner;
}

impl DmaLinkedListDw0 for &mut u32 {
    fn set_size(&mut self, len: u16) {
        let mask = 0b111111111111;
        let bit_s = 0;
        **self = (**self & !(mask << bit_s)) | (len as u32) << bit_s;
    }

    fn get_size(&mut self) -> u16 {
        let mask = 0b111111111111;
        let bit_s = 0;
        ((**self & (mask << bit_s)) >> bit_s) as u16
    }

    fn set_length(&mut self, len: u16) {
        let mask = 0b111111111111;
        let bit_s = 12;
        **self = (**self & !(mask << bit_s)) | (len as u32) << bit_s;
    }

    fn get_length(&mut self) -> u16 {
        let mask = 0b111111111111;
        let bit_s = 12;
        ((**self & (mask << bit_s)) >> bit_s) as u16
    }

    fn set_err_eof(&mut self, err_eof: bool) {
        let mask = 0b1;
        let bit_s = 28;
        **self = (**self & !(mask << bit_s)) | (err_eof as u32) << bit_s;
    }

    #[cfg(not(esp32))]
    fn get_err_eof(&mut self) -> bool {
        let mask = 0b1;
        let bit_s = 28;
        ((**self & (mask << bit_s)) >> bit_s) != 0
    }

    fn set_suc_eof(&mut self, suc_eof: bool) {
        let mask = 0b1;
        let bit_s = 30;
        **self = (**self & !(mask << bit_s)) | (suc_eof as u32) << bit_s;
    }

    fn get_suc_eof(&mut self) -> bool {
        let mask = 0b1;
        let bit_s = 30;
        ((**self & (mask << bit_s)) >> bit_s) != 0
    }

    fn set_owner(&mut self, owner: Owner) {
        let mask = 0b1;
        let bit_s = 31;
        **self = (**self & !(mask << bit_s)) | (owner as u32) << bit_s;
    }

    fn get_owner(&mut self) -> Owner {
        let mask = 0b1;
        let bit_s = 31;
        ((**self & (mask << bit_s)) >> bit_s).into()
    }
}

/// Crate private implementatin details
pub(crate) mod private {
    use super::*;

    pub trait PeripheralMarker {}

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

    /// DMA Rx
    ///
    /// The functions here are not meant to be used outside the HAL and will be
    /// hidden/inaccessible in future.
    pub trait Rx {
        fn init(&mut self, burst_mode: bool, priority: DmaPriority);

        fn init_channel(&mut self);

        fn prepare_transfer(
            &mut self,
            circular: bool,
            peri: DmaPeripheral,
            data: *mut u8,
            len: usize,
        ) -> Result<(), DmaError>;

        fn is_done(&mut self) -> bool;

        fn available(&mut self) -> usize;

        fn pop(&mut self, data: &mut [u8]) -> Result<usize, DmaError>;
    }

    pub trait RxChannel<R>
    where
        R: RegisterAccess,
    {
        fn init(&mut self, burst_mode: bool, priority: DmaPriority) {
            R::set_in_burstmode(burst_mode);
            R::set_in_priority(priority);
        }

        fn prepare_transfer(
            &mut self,
            descriptors: &mut [u32],
            circular: bool,
            peri: DmaPeripheral,
            data: *mut u8,
            len: usize,
        ) -> Result<(), DmaError> {
            for descr in descriptors.iter_mut() {
                *descr = 0;
            }

            compiler_fence(core::sync::atomic::Ordering::SeqCst);

            let mut processed = 0;
            let mut descr = 0;
            loop {
                let chunk_size = usize::min(CHUNK_SIZE, len - processed);
                let last = processed + chunk_size >= len;

                descriptors[descr + 1] = data as u32 + processed as u32;

                let mut dw0 = &mut descriptors[descr];

                dw0.set_suc_eof(true);
                dw0.set_owner(Owner::Dma);
                dw0.set_size(chunk_size as u16); // align to 32 bits?
                dw0.set_length(0); // actual size of the data!?

                if !last {
                    descriptors[descr + 2] =
                        (&descriptors[descr + 3]) as *const _ as *const () as u32;
                } else {
                    descriptors[descr + 2] = if circular {
                        descriptors.as_ptr() as *const () as u32
                    } else {
                        0
                    };
                }

                processed += chunk_size;
                descr += 3;

                if processed >= len {
                    break;
                }
            }

            R::clear_in_interrupts();
            R::reset_in();
            R::set_in_descriptors(descriptors.as_ptr() as u32);
            R::set_in_peripheral(peri as u8);
            R::start_in();

            if R::has_in_descriptor_error() {
                return Err(DmaError::DescriptorError);
            }

            Ok(())
        }

        fn is_done(&mut self) -> bool {
            R::is_in_done()
        }

        fn last_in_dscr_address(&self) -> usize {
            R::last_in_dscr_address()
        }
    }

    pub struct ChannelRx<'a, T, R>
    where
        T: RxChannel<R>,
        R: RegisterAccess,
    {
        pub descriptors: &'a mut [u32],
        pub burst_mode: bool,
        pub rx_impl: T,
        pub read_offset: usize,
        pub read_descr_ptr: *const u32,
        pub available: usize,
        pub last_seen_handled_descriptor_ptr: *const u32,
        pub buffer_start: *const u8,
        pub buffer_len: usize,
        pub _phantom: PhantomData<R>,
    }

    impl<'a, T, R> Rx for ChannelRx<'a, T, R>
    where
        T: RxChannel<R>,
        R: RegisterAccess,
    {
        fn init(&mut self, burst_mode: bool, priority: DmaPriority) {
            self.rx_impl.init(burst_mode, priority);
        }

        fn prepare_transfer(
            &mut self,
            circular: bool,
            peri: DmaPeripheral,
            data: *mut u8,
            len: usize,
        ) -> Result<(), DmaError> {
            if self.descriptors.len() % 3 != 0 {
                return Err(DmaError::InvalidDescriptorSize);
            }

            if self.descriptors.len() / 3 < len / CHUNK_SIZE {
                return Err(DmaError::OutOfDescriptors);
            }

            if self.burst_mode && (len % 4 != 0 || data as u32 % 4 != 0) {
                return Err(DmaError::InvalidAlignment);
            }

            if circular && len < CHUNK_SIZE * 2 {
                return Err(DmaError::BufferTooSmall);
            }

            self.read_offset = 0;
            self.available = 0;
            self.read_descr_ptr = self.descriptors.as_ptr() as *const u32;
            self.last_seen_handled_descriptor_ptr = core::ptr::null();
            self.buffer_start = data;
            self.buffer_len = len;

            self.rx_impl
                .prepare_transfer(self.descriptors, circular, peri, data, len)?;
            Ok(())
        }

        fn is_done(&mut self) -> bool {
            self.rx_impl.is_done()
        }

        fn init_channel(&mut self) {
            R::init_channel();
        }

        fn available(&mut self) -> usize {
            let last_dscr = self.rx_impl.last_in_dscr_address() as *const u32;
            if !last_dscr.is_null()
                && !self.last_seen_handled_descriptor_ptr.is_null()
                && self.last_seen_handled_descriptor_ptr != last_dscr
            {
                let descr_address = last_dscr;

                if descr_address >= self.last_seen_handled_descriptor_ptr {
                    let mut ptr = self.last_seen_handled_descriptor_ptr as *const u32;

                    unsafe {
                        while ptr < descr_address as *const u32 {
                            let mut dw0 = &mut ptr.read_volatile();
                            self.available += dw0.get_length() as usize;
                            ptr = ptr.offset(3);
                        }
                    }
                } else {
                    let mut ptr = self.last_seen_handled_descriptor_ptr as *const u32;

                    unsafe {
                        loop {
                            if ptr.offset(2).read_volatile() == 0 {
                                break;
                            }

                            let mut dw0 = &mut ptr.read_volatile();
                            self.available += dw0.get_length() as usize;
                            ptr = ptr.offset(3);
                        }
                    }
                }

                if self.available >= self.buffer_len {
                    unsafe {
                        let segment_len =
                            (&mut self.read_descr_ptr.read_volatile()).get_length() as usize;
                        self.available -= segment_len;
                        self.read_offset = (self.read_offset + segment_len) % self.buffer_len;
                        let next_descriptor =
                            self.read_descr_ptr.offset(2).read_volatile() as *const u32;
                        self.read_descr_ptr = if next_descriptor.is_null() {
                            self.descriptors.as_ptr() as *const u32
                        } else {
                            next_descriptor
                        }
                    }
                }

                self.last_seen_handled_descriptor_ptr = descr_address;
            } else {
                self.last_seen_handled_descriptor_ptr = last_dscr;
            }

            self.available
        }

        fn pop(&mut self, data: &mut [u8]) -> Result<usize, super::DmaError> {
            let avail = self.available();

            if avail < data.len() {
                return Err(super::DmaError::Exhausted);
            }

            unsafe {
                let dst = data.as_mut_ptr();
                let src = self.buffer_start.offset(self.read_offset as isize) as *const u8;
                let count = usize::min(data.len(), self.buffer_len - self.read_offset);
                core::ptr::copy_nonoverlapping(src, dst, count);
            }

            if self.read_offset + data.len() >= self.buffer_len {
                let remainder = (self.read_offset + data.len()) % self.buffer_len;
                let src = self.buffer_start as *const u8;
                unsafe {
                    let dst = data.as_mut_ptr().offset((data.len() - remainder) as isize);
                    core::ptr::copy_nonoverlapping(src, dst, remainder);
                }
            }

            let mut forward = data.len();
            loop {
                unsafe {
                    let next_descriptor =
                        self.read_descr_ptr.offset(2).read_volatile() as *const u32;
                    let segment_len =
                        (&mut self.read_descr_ptr.read_volatile()).get_length() as usize;
                    self.read_descr_ptr = if next_descriptor.is_null() {
                        self.descriptors.as_ptr() as *const u32
                    } else {
                        next_descriptor
                    };

                    if forward <= segment_len {
                        break;
                    }

                    forward -= segment_len;

                    if forward == 0 {
                        break;
                    }
                }
            }

            self.read_offset = (self.read_offset + data.len()) % self.buffer_len;
            self.available -= data.len();

            Ok(data.len())
        }
    }

    /// DMA Tx
    ///
    /// The functions here are not meant to be used outside the HAL and will be
    /// hidden/inaccessible in future.
    pub trait Tx {
        fn init(&mut self, burst_mode: bool, priority: DmaPriority);

        fn init_channel(&mut self);

        fn prepare_transfer(
            &mut self,
            peri: DmaPeripheral,
            circular: bool,
            data: *const u8,
            len: usize,
        ) -> Result<(), DmaError>;

        fn is_done(&mut self) -> bool;

        fn available(&mut self) -> usize;

        fn push(&mut self, data: &[u8]) -> Result<usize, super::DmaError>;
    }

    pub trait TxChannel<R>
    where
        R: RegisterAccess,
    {
        fn init(&mut self, burst_mode: bool, priority: DmaPriority) {
            R::set_out_burstmode(burst_mode);
            R::set_out_priority(priority);
        }

        fn prepare_transfer(
            &mut self,
            descriptors: &mut [u32],
            circular: bool,
            peri: DmaPeripheral,
            data: *const u8,
            len: usize,
        ) -> Result<(), DmaError> {
            for descr in descriptors.iter_mut() {
                *descr = 0;
            }

            compiler_fence(core::sync::atomic::Ordering::SeqCst);

            let mut processed = 0;
            let mut descr = 0;
            loop {
                let chunk_size = usize::min(CHUNK_SIZE, len - processed);
                let last = processed + chunk_size >= len;

                descriptors[descr + 1] = data as u32 + processed as u32;

                let mut dw0 = &mut descriptors[descr];

                dw0.set_suc_eof(true);
                dw0.set_owner(Owner::Dma);
                dw0.set_size(chunk_size as u16); // align to 32 bits?
                dw0.set_length(chunk_size as u16); // actual size of the data!?

                if !last {
                    descriptors[descr + 2] =
                        (&descriptors[descr + 3]) as *const _ as *const () as u32;
                } else {
                    if !circular {
                        descriptors[descr + 2] = 0;
                    } else {
                        descriptors[descr + 2] = descriptors.as_ptr() as u32;
                    }
                }

                processed += chunk_size;
                descr += 3;

                if processed >= len {
                    break;
                }
            }

            R::clear_out_interrupts();
            R::reset_out();
            R::set_out_descriptors(descriptors.as_ptr() as u32);
            R::set_out_peripheral(peri as u8);
            R::start_out();

            if R::has_out_descriptor_error() {
                return Err(DmaError::DescriptorError);
            }

            Ok(())
        }

        fn is_done(&mut self) -> bool {
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
    }

    pub struct ChannelTx<'a, T, R>
    where
        T: TxChannel<R>,
        R: RegisterAccess,
    {
        pub descriptors: &'a mut [u32],
        #[allow(unused)]
        pub burst_mode: bool,
        pub tx_impl: T,
        pub write_offset: usize,
        pub write_descr_ptr: *const u32,
        pub available: usize,
        pub last_seen_handled_descriptor_ptr: *const u32,
        pub buffer_start: *const u8,
        pub buffer_len: usize,
        pub _phantom: PhantomData<R>,
    }

    impl<'a, T, R> Tx for ChannelTx<'a, T, R>
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

        fn prepare_transfer(
            &mut self,
            peri: DmaPeripheral,
            circular: bool,
            data: *const u8,
            len: usize,
        ) -> Result<(), DmaError> {
            if self.descriptors.len() % 3 != 0 {
                return Err(DmaError::InvalidDescriptorSize);
            }

            if self.descriptors.len() / 3 < len / CHUNK_SIZE {
                return Err(DmaError::OutOfDescriptors);
            }

            if circular && len < CHUNK_SIZE * 2 {
                return Err(DmaError::BufferTooSmall);
            }

            self.write_offset = 0;
            self.available = 0;
            self.write_descr_ptr = self.descriptors.as_ptr() as *const u32;
            self.last_seen_handled_descriptor_ptr = self.descriptors.as_ptr() as *const u32;
            self.buffer_start = data;
            self.buffer_len = len;

            self.tx_impl
                .prepare_transfer(self.descriptors, circular, peri, data, len)?;

            Ok(())
        }

        fn is_done(&mut self) -> bool {
            self.tx_impl.is_done()
        }

        fn available(&mut self) -> usize {
            if self.tx_impl.descriptors_handled() {
                self.tx_impl.reset_descriptors_handled();
                let descr_address = self.tx_impl.last_out_dscr_address() as *const u32;

                if descr_address >= self.last_seen_handled_descriptor_ptr {
                    let mut ptr = self.last_seen_handled_descriptor_ptr as *const u32;

                    unsafe {
                        while ptr < descr_address as *const u32 {
                            let mut dw0 = &mut ptr.read_volatile();
                            self.available += dw0.get_length() as usize;
                            ptr = ptr.offset(3);
                        }
                    }
                } else {
                    let mut ptr = self.last_seen_handled_descriptor_ptr as *const u32;

                    unsafe {
                        loop {
                            if ptr.offset(2).read_volatile() == 0 {
                                break;
                            }

                            let mut dw0 = &mut ptr.read_volatile();
                            self.available += dw0.get_length() as usize;
                            ptr = ptr.offset(3);
                        }
                    }
                }

                if self.available >= self.buffer_len {
                    unsafe {
                        let segment_len =
                            (&mut self.write_descr_ptr.read_volatile()).get_length() as usize;
                        self.available -= segment_len;
                        self.write_offset = (self.write_offset + segment_len) % self.buffer_len;
                        let next_descriptor =
                            self.write_descr_ptr.offset(2).read_volatile() as *const u32;
                        self.write_descr_ptr = if next_descriptor.is_null() {
                            self.descriptors.as_ptr() as *const u32
                        } else {
                            next_descriptor
                        }
                    }
                }

                self.last_seen_handled_descriptor_ptr = descr_address;
            }

            self.available
        }

        fn push(&mut self, data: &[u8]) -> Result<usize, super::DmaError> {
            let avail = self.available();

            if avail < data.len() {
                return Err(super::DmaError::Overflow);
            }

            unsafe {
                let src = data.as_ptr();
                let dst = self.buffer_start.offset(self.write_offset as isize) as *mut u8;
                let count = usize::min(data.len(), self.buffer_len - self.write_offset);
                core::ptr::copy_nonoverlapping(src, dst, count);
            }

            if self.write_offset + data.len() >= self.buffer_len {
                let remainder = (self.write_offset + data.len()) % self.buffer_len;
                let dst = self.buffer_start as *mut u8;
                unsafe {
                    let src = data.as_ptr().offset((data.len() - remainder) as isize);
                    core::ptr::copy_nonoverlapping(src, dst, remainder);
                }
            }

            let mut forward = data.len();
            loop {
                unsafe {
                    let next_descriptor =
                        self.write_descr_ptr.offset(2).read_volatile() as *const u32;
                    let segment_len =
                        (&mut self.write_descr_ptr.read_volatile()).get_length() as usize;
                    self.write_descr_ptr = if next_descriptor.is_null() {
                        self.descriptors.as_ptr() as *const u32
                    } else {
                        next_descriptor
                    };

                    if forward <= segment_len {
                        break;
                    }

                    forward -= segment_len;

                    if forward == 0 {
                        break;
                    }
                }
            }

            self.write_offset = (self.write_offset + data.len()) % self.buffer_len;
            self.available -= data.len();

            Ok(data.len())
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
        fn set_in_peripheral(peripheral: u8);
        fn start_in();
        fn is_in_done() -> bool;
        fn last_in_dscr_address() -> usize;
    }
}

/// DMA Channel
pub struct Channel<TX, RX, P>
where
    TX: Tx,
    RX: Rx,
    P: PeripheralMarker,
{
    pub(crate) tx: TX,
    pub(crate) rx: RX,
    _phantom: PhantomData<P>,
}

/// Trait to be implemented for an in progress dma transfer.
#[allow(drop_bounds)]
pub trait DmaTransfer<B, T>: Drop {
    /// Wait for the transfer to finish.
    fn wait(self) -> (B, T);
}

/// Trait to be implemented for an in progress dma transfer.
#[allow(drop_bounds)]
pub trait DmaTransferRxTx<BR, BT, T>: Drop {
    /// Wait for the transfer to finish.
    fn wait(self) -> (BR, BT, T);
}
