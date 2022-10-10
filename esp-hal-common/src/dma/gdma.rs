//! Direct Memory Access

use core::{marker::PhantomData, sync::atomic::compiler_fence};

use crate::system::{Peripheral, PeripheralClockControl};

/// DMA Errors
#[derive(Debug, Clone, Copy)]
pub enum DmaError {
    InvalidAlignment,
    OutOfDescriptors,
    InvalidDescriptorSize,
    DescriptorError,
}

/// DMA Priorities
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

/// DMA capable peripherals
pub enum DmaPeripheral {
    Spi2  = 0,
    Uhci0 = 2,
    I2s   = 3,
    Aes   = 6,
    Sha   = 7,
    Adc   = 8,
}

/// DMA Rx
///
/// The functions here are not meant to be used outside the HAL and will be
/// hidden/inaccessible in future.
pub trait Rx {
    fn init(&mut self, burst_mode: bool, priority: DmaPriority);
    fn prepare_transfer(
        &mut self,
        peri: DmaPeripheral,
        data: *mut u8,
        len: usize,
    ) -> Result<(), DmaError>;
    fn is_done(&mut self) -> bool;
}

pub(crate) trait RxChannel<R>
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

            dw0.set_suc_eof(last);
            dw0.set_owner(Owner::Dma);
            dw0.set_size(chunk_size as u16); // align to 32 bits?
            dw0.set_length(0); // actual size of the data!?

            if !last {
                descriptors[descr + 2] = (&descriptors[descr + 3]) as *const _ as *const () as u32;
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

pub(crate) struct ChannelRx<'a, T, R>
where
    T: RxChannel<R>,
    R: RegisterAccess,
{
    descriptors: &'a mut [u32],
    burst_mode: bool,
    rx_impl: T,
    _phantom: PhantomData<R>,
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
}

/// DMA Tx
///
/// The functions here are not meant to be used outside the HAL and will be
/// hidden/inaccessible in future.
pub trait Tx {
    fn init(&mut self, burst_mode: bool, priority: DmaPriority);
    fn prepare_transfer(
        &mut self,
        peri: DmaPeripheral,
        data: *const u8,
        len: usize,
    ) -> Result<(), DmaError>;
    fn is_done(&mut self) -> bool;
}

pub(crate) trait TxChannel<R>
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

            dw0.set_suc_eof(last);
            dw0.set_owner(Owner::Dma);
            dw0.set_size(chunk_size as u16); // align to 32 bits?
            dw0.set_length(chunk_size as u16); // actual size of the data!?

            if !last {
                descriptors[descr + 2] = (&descriptors[descr + 3]) as *const _ as *const () as u32;
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

pub(crate) struct ChannelTx<'a, T, R>
where
    T: TxChannel<R>,
    R: RegisterAccess,
{
    descriptors: &'a mut [u32],
    #[allow(unused)]
    burst_mode: bool,
    tx_impl: T,
    _phantom: PhantomData<R>,
}

impl<'a, T, R> Tx for ChannelTx<'a, T, R>
where
    T: TxChannel<R>,
    R: RegisterAccess,
{
    fn init(&mut self, burst_mode: bool, priority: DmaPriority) {
        self.tx_impl.init(burst_mode, priority);
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

pub(crate) trait RegisterAccess {
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

macro_rules! ImplChannel {
    ($num: literal) => {
        paste::paste! {
            pub(crate) struct [<Channel $num>] {}

            impl RegisterAccess for [<Channel $num>] {
                fn set_out_burstmode(burst_mode: bool) {
                    let dma = unsafe { &*crate::pac::DMA::PTR };

                    dma.[<out_conf0_ch $num>].modify(|_,w| {
                        w.[<out_data_burst_en_ch $num>]().bit(burst_mode)
                            .[<outdscr_burst_en_ch $num>]().bit(burst_mode)
                    });
                }

                fn set_out_priority(priority: DmaPriority) {
                    let dma = unsafe { &*crate::pac::DMA::PTR };

                    dma.[<out_pri_ch $num>].write(|w| {
                        w.[<tx_pri_ch $num>]().variant(priority as u8)
                    });
                }

                fn clear_out_interrupts() {
                    let dma = unsafe { &*crate::pac::DMA::PTR };

                    dma.[<int_clr_ch $num>].write(|w| {
                        w.[<out_eof_ch $num _int_clr>]()
                            .set_bit()
                            .[<out_dscr_err_ch $num _int_clr>]()
                            .set_bit()
                            .[<out_done_ch $num _int_clr>]()
                            .set_bit()
                            .[<out_total_eof_ch $num _int_clr>]()
                            .set_bit()
                            .[<outfifo_ovf_ch $num _int_clr>]()
                            .set_bit()
                            .[<outfifo_udf_ch $num _int_clr>]()
                            .set_bit()
                    });
                }

                fn reset_out() {
                    let dma = unsafe { &*crate::pac::DMA::PTR };
                    dma.[<out_conf0_ch $num>].modify(|_, w| w.[<out_rst_ch $num>]().set_bit());
                    dma.[<out_conf0_ch $num>].modify(|_, w| w.[<out_rst_ch $num>]().clear_bit());
                }

                fn set_out_descriptors(address: u32) {
                    let dma = unsafe { &*crate::pac::DMA::PTR };
                    dma.[<out_link_ch $num>]
                        .modify(|_, w| unsafe { w.[<outlink_addr_ch $num>]().bits(address) });
                }

                fn has_out_descriptor_error() -> bool {
                    let dma = unsafe { &*crate::pac::DMA::PTR };
                    dma.[<int_raw_ch $num>].read().[<out_dscr_err_ch $num _int_raw>]().bit()
                }

                fn set_out_peripheral(peripheral: u8) {
                    let dma = unsafe { &*crate::pac::DMA::PTR };
                    dma.[<out_peri_sel_ch $num>]
                        .modify(|_, w| w.[<peri_out_sel_ch $num>]().variant(peripheral));
                }

                fn start_out() {
                    let dma = unsafe { &*crate::pac::DMA::PTR };
                    dma.[<out_link_ch $num>]
                        .modify(|_, w| w.[<outlink_start_ch $num>]().set_bit());
                }

                fn is_out_done() -> bool {
                    let dma = unsafe { &*crate::pac::DMA::PTR };
                    dma.[<int_raw_ch $num>].read().[<out_total_eof_ch $num _int_raw>]().bit()
                }

                fn set_in_burstmode(burst_mode: bool) {
                    let dma = unsafe { &*crate::pac::DMA::PTR };

                    dma.[<in_conf0_ch $num>].modify(|_,w| {
                        w.[<in_data_burst_en_ch $num>]().bit(burst_mode)
                            .[<indscr_burst_en_ch $num>]().bit(burst_mode)
                    });
                }

                fn set_in_priority(priority: DmaPriority) {
                    let dma = unsafe { &*crate::pac::DMA::PTR };

                    dma.[<in_pri_ch $num>].write(|w| {
                        w.[<rx_pri_ch $num>]().variant(priority as u8)
                    });
                }

                fn clear_in_interrupts() {
                    let dma = unsafe { &*crate::pac::DMA::PTR };

                    dma.[<int_clr_ch $num>].write(|w| {
                        w.[<in_suc_eof_ch $num _int_clr>]()
                            .set_bit()
                            .[<in_err_eof_ch $num _int_clr>]()
                            .set_bit()
                            .[<in_dscr_err_ch $num _int_clr>]()
                            .set_bit()
                            .[<in_dscr_empty_ch $num _int_clr>]()
                            .set_bit()
                            .[<in_done_ch $num _int_clr>]()
                            .set_bit()
                            .[<infifo_ovf_ch $num _int_clr>]()
                            .set_bit()
                            .[<infifo_udf_ch $num _int_clr>]()
                            .set_bit()
                    });
                }

                fn reset_in() {
                    let dma = unsafe { &*crate::pac::DMA::PTR };
                    dma.[<in_conf0_ch $num>].modify(|_, w| w.[<in_rst_ch $num>]().set_bit());
                    dma.[<in_conf0_ch $num>].modify(|_, w| w.[<in_rst_ch $num>]().clear_bit());
                }

                fn set_in_descriptors(address: u32) {
                    let dma = unsafe { &*crate::pac::DMA::PTR };
                    dma.[<in_link_ch $num>]
                        .modify(|_, w| unsafe { w.[<inlink_addr_ch $num>]().bits(address) });
                }

                fn has_in_descriptor_error() -> bool {
                    let dma = unsafe { &*crate::pac::DMA::PTR };
                    dma.[<int_raw_ch $num>].read().[<in_dscr_err_ch $num _int_raw>]().bit()
                }

                fn set_in_peripheral(peripheral: u8) {
                    let dma = unsafe { &*crate::pac::DMA::PTR };
                    dma.[<in_peri_sel_ch $num>]
                        .modify(|_, w| w.[<peri_in_sel_ch $num>]().variant(peripheral));
                }

                fn start_in() {
                    let dma = unsafe { &*crate::pac::DMA::PTR };
                    dma.[<in_link_ch $num>]
                        .modify(|_, w| w.[<inlink_start_ch $num>]().set_bit());
                }

                fn is_in_done() -> bool {
                    let dma = unsafe { &*crate::pac::DMA::PTR };
                    dma.[<int_raw_ch $num>].read().[<in_suc_eof_ch $num _int_raw>]().bit()
                }
            }

            pub(crate) struct [<Channel $num TxImpl>] {}

            impl<'a> TxChannel<[<Channel $num>]> for [<Channel $num TxImpl>] {}

            pub(crate) struct [<Channel $num RxImpl>] {}

            impl<'a> RxChannel<[<Channel $num>]> for [<Channel $num RxImpl>] {}

            /// Create the Tx half of the DMA channel
            pub struct [<TxCreator $num>] {
            }

            impl [<TxCreator $num>] {
                pub fn get<'a>(
                    self,
                    descriptors: &'a mut [u32],
                    burst_mode: bool,
                    priority: DmaPriority,
                ) -> impl Tx + 'a {
                    let mut tx_impl = [<Channel $num TxImpl>] {};
                    tx_impl.init(burst_mode, priority);

                    ChannelTx {
                        descriptors,
                        burst_mode,
                        tx_impl: tx_impl,
                        _phantom: PhantomData::default(),
                    }
                }
            }

            /// Create the Rx half of the DMA channel
            pub struct [<RxCreator $num>] {
            }

            impl [<RxCreator $num>] {
                pub fn get<'a>(
                    self,
                    descriptors: &'a mut [u32],
                    burst_mode: bool,
                    priority: DmaPriority,
                ) -> impl Rx + 'a {
                    let mut rx_impl = [<Channel $num RxImpl>] {};
                    rx_impl.init(burst_mode, priority);

                    ChannelRx {
                        descriptors,
                        burst_mode,
                        rx_impl: rx_impl,
                        _phantom: PhantomData::default(),
                    }
                }
            }
        }
    };
}

ImplChannel!(0);
ImplChannel!(1);
ImplChannel!(2);

/// DMA Channel
pub struct Channel<TX, RX> {
    pub tx: TX,
    pub rx: RX,
}

/// GDMA Peripheral
/// This offers the available DMA channels.
pub struct Gdma {
    _inner: crate::pac::DMA,
    pub channel0: Channel<TxCreator0, RxCreator0>,
    pub channel1: Channel<TxCreator1, RxCreator1>,
    pub channel2: Channel<TxCreator2, RxCreator2>,
}

impl Gdma {
    /// Create a DMA instance.
    pub fn new(
        dma: crate::pac::DMA,
        peripheral_clock_control: &mut PeripheralClockControl,
    ) -> Gdma {
        peripheral_clock_control.enable(Peripheral::Gdma);
        dma.misc_conf.modify(|_, w| w.ahbm_rst_inter().set_bit());
        dma.misc_conf.modify(|_, w| w.ahbm_rst_inter().clear_bit());
        dma.misc_conf.modify(|_, w| w.clk_en().set_bit());

        Gdma {
            _inner: dma,
            channel0: Channel {
                rx: RxCreator0 {},
                tx: TxCreator0 {},
            },
            channel1: Channel {
                rx: RxCreator1 {},
                tx: TxCreator1 {},
            },
            channel2: Channel {
                rx: RxCreator2 {},
                tx: TxCreator2 {},
            },
        }
    }
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
