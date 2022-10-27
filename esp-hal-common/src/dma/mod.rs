//! Direct Memory Access Commons

use core::{marker::PhantomData, sync::atomic::compiler_fence};

use private::*;

#[cfg(any(esp32c2, esp32c3))]
pub mod gdma;

#[cfg(any(esp32, esp32s2))]
pub mod pdma;

/// DMA Errors
#[derive(Debug, Clone, Copy)]
pub enum DmaError {
    InvalidAlignment,
    OutOfDescriptors,
    InvalidDescriptorSize,
    DescriptorError,
}

/// DMA Priorities
#[cfg(any(esp32c2, esp32c3, esp32s3))]
#[derive(Clone, Copy)]
pub enum DmaPriority {
    Priority0  = 0,
    Priority1  = 1,
    Priority2  = 2,
    Priority3  = 3,
    Priority4  = 4,
    Priority5  = 5,
    Priority6  = 6,
    Priority7  = 7,
    Priority8  = 8,
    Priority9  = 9,
    Priority10 = 10,
    Priority11 = 11,
    Priority12 = 12,
    Priority13 = 13,
    Priority14 = 14,
    Priority15 = 15,
}

/// DMA Priorities
/// The values need to match the TRM
#[cfg(any(esp32, esp32s2))]
#[derive(Clone, Copy)]
pub enum DmaPriority {
    Priority0 = 0,
}

/// DMA capable peripherals
/// The values need to match the TRM
#[cfg(esp32c2)]
#[derive(Clone, Copy)]
pub enum DmaPeripheral {
    Spi2 = 0,
    Sha  = 7,
}

/// DMA capable peripherals
/// The values need to match the TRM
#[cfg(esp32c3)]
#[derive(Clone, Copy)]
pub enum DmaPeripheral {
    Spi2  = 0,
    Uhci0 = 2,
    I2s   = 3,
    Aes   = 6,
    Sha   = 7,
    Adc   = 8,
}

/// DMA capable peripherals
/// The values need to match the TRM
#[cfg(any(esp32, esp32s2))]
#[derive(Clone, Copy)]
pub enum DmaPeripheral {
    Spi2 = 0,
    Spi3 = 1,
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
    #[cfg(not(esp32))]
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

    #[cfg(not(esp32))]
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
    #[cfg(any(esp32, esp32s2))]
    pub trait Spi3Peripheral: SpiPeripheral + PeripheralMarker {}

    /// DMA Rx
    ///
    /// The functions here are not meant to be used outside the HAL and will be
    /// hidden/inaccessible in future.
    pub trait Rx {
        fn init(&mut self, burst_mode: bool, priority: DmaPriority);

        fn init_channel(&mut self);

        fn prepare_transfer(
            &mut self,
            peri: DmaPeripheral,
            data: *mut u8,
            len: usize,
        ) -> Result<(), DmaError>;

        fn is_done(&mut self) -> bool;
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
                let chunk_size = usize::min(4092, len - processed);
                let last = processed + chunk_size >= len;

                descriptors[descr + 1] = data as u32 + processed as u32;

                let mut dw0 = &mut descriptors[descr];

                #[cfg(not(esp32))]
                dw0.set_suc_eof(last);

                dw0.set_owner(Owner::Dma);
                dw0.set_size(chunk_size as u16); // align to 32 bits?
                dw0.set_length(0); // actual size of the data!?

                if !last {
                    descriptors[descr + 2] =
                        (&descriptors[descr + 3]) as *const _ as *const () as u32;
                } else {
                    descriptors[descr + 2] = 0;
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
    }

    pub struct ChannelRx<'a, T, R>
    where
        T: RxChannel<R>,
        R: RegisterAccess,
    {
        pub descriptors: &'a mut [u32],
        pub burst_mode: bool,
        pub rx_impl: T,
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
            peri: DmaPeripheral,
            data: *mut u8,
            len: usize,
        ) -> Result<(), DmaError> {
            if self.descriptors.len() % 3 != 0 {
                return Err(DmaError::InvalidDescriptorSize);
            }

            if self.descriptors.len() / 3 < len / 4092 {
                return Err(DmaError::OutOfDescriptors);
            }

            if self.burst_mode && (len % 4 != 0 || data as u32 % 4 != 0) {
                return Err(DmaError::InvalidAlignment);
            }

            self.rx_impl
                .prepare_transfer(self.descriptors, peri, data, len)?;
            Ok(())
        }

        fn is_done(&mut self) -> bool {
            self.rx_impl.is_done()
        }

        fn init_channel(&mut self) {
            R::init_channel();
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
            data: *const u8,
            len: usize,
        ) -> Result<(), DmaError>;

        fn is_done(&mut self) -> bool;
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
                let chunk_size = usize::min(4092, len - processed);
                let last = processed + chunk_size >= len;

                descriptors[descr + 1] = data as u32 + processed as u32;

                let mut dw0 = &mut descriptors[descr];

                #[cfg(not(esp32))]
                dw0.set_suc_eof(last);

                dw0.set_owner(Owner::Dma);
                dw0.set_size(chunk_size as u16); // align to 32 bits?
                dw0.set_length(chunk_size as u16); // actual size of the data!?

                if !last {
                    descriptors[descr + 2] =
                        (&descriptors[descr + 3]) as *const _ as *const () as u32;
                } else {
                    descriptors[descr + 2] = 0;
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
            data: *const u8,
            len: usize,
        ) -> Result<(), DmaError> {
            if self.descriptors.len() % 3 != 0 {
                return Err(DmaError::InvalidDescriptorSize);
            }

            if self.descriptors.len() / 3 < len / 4092 {
                return Err(DmaError::OutOfDescriptors);
            }

            self.tx_impl
                .prepare_transfer(self.descriptors, peri, data, len)?;

            Ok(())
        }

        fn is_done(&mut self) -> bool {
            self.tx_impl.is_done()
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
        fn set_in_burstmode(burst_mode: bool);
        fn set_in_priority(priority: DmaPriority);
        fn clear_in_interrupts();
        fn reset_in();
        fn set_in_descriptors(address: u32);
        fn has_in_descriptor_error() -> bool;
        fn set_in_peripheral(peripheral: u8);
        fn start_in();
        fn is_in_done() -> bool;
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
