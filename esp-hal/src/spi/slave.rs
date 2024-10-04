//! # Serial Peripheral Interface - Slave Mode
//!
//! ## Overview
//!
//! In this mode, the SPI acts as slave and transfers data with its master when
//! its CS is asserted.
//!
//! ## Configuration
//!
//! The SPI slave driver allows using full-duplex and can only be used with DMA.
//!
//! ## Example
//!
//! ### SPI Slave with DMA
//!
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::dma::DmaPriority;
//! # use esp_hal::dma_buffers;
//! # use esp_hal::spi::SpiMode;
//! # use esp_hal::spi::slave::Spi;
//! # use esp_hal::dma::Dma;
//! # use esp_hal::gpio::Io;
//! let dma = Dma::new(peripherals.DMA);
#![cfg_attr(esp32s2, doc = "let dma_channel = dma.spi2channel;")]
#![cfg_attr(not(esp32s2), doc = "let dma_channel = dma.channel0;")]
//! let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
//! let sclk = io.pins.gpio0;
//! let miso = io.pins.gpio1;
//! let mosi = io.pins.gpio2;
//! let cs = io.pins.gpio3;
//!
//! let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) =
//! dma_buffers!(32000); let mut spi = Spi::new(
//!     peripherals.SPI2,
//!     sclk,
//!     mosi,
//!     miso,
//!     cs,
//!     SpiMode::Mode0,
//! )
//! .with_dma(dma_channel.configure(
//!     false,
//!     DmaPriority::Priority0,
//! ), rx_descriptors, tx_descriptors);
//!
//! let mut receive = rx_buffer;
//! let mut send = tx_buffer;
//!
//! let transfer = spi
//!     .dma_transfer(&mut receive, &mut send)
//!     .unwrap();
//!
//! transfer.wait().unwrap();
//! # }
//! ```
//! 
//! ## Implementation State
//!
//! There are several options for working with the SPI peripheral in slave mode,
//! but the code currently only supports single transfers (not segmented
//! transfers), full duplex, single bit (not dual or quad SPI), and DMA mode
//! (not CPU mode). It also does not support blocking operations, as the actual
//! transfer is controlled by the SPI master; if these are necessary,
//! then the DmaTransfer trait instance can be wait()ed on or polled for
//! is_done().
//! - ESP32 does not support SPI Slave. See [tracking issue].
//!
//! [tracking issue]: https://github.com/esp-rs/esp-hal/issues/469

use core::marker::PhantomData;

use super::{Error, FullDuplexMode, SpiMode};
use crate::{
    dma::{DescriptorChain, DmaPeripheral, Rx, Tx},
    gpio::{InputSignal, OutputSignal, PeripheralInput, PeripheralOutput},
    peripheral::{Peripheral, PeripheralRef},
    peripherals::spi2::RegisterBlock,
    private,
    system::PeripheralClockControl,
};

const MAX_DMA_SIZE: usize = 32768 - 32;

/// SPI peripheral driver
pub struct Spi<'d, T, M> {
    spi: PeripheralRef<'d, T>,
    #[allow(dead_code)]
    data_mode: SpiMode,
    _mode: PhantomData<M>,
}

impl<'d, T> Spi<'d, T, FullDuplexMode>
where
    T: Instance,
{
    /// Constructs an SPI instance in 8bit dataframe mode.
    pub fn new<
        SCK: PeripheralInput,
        MOSI: PeripheralInput,
        MISO: PeripheralOutput,
        CS: PeripheralInput,
    >(
        spi: impl Peripheral<P = T> + 'd,
        sclk: impl Peripheral<P = SCK> + 'd,
        mosi: impl Peripheral<P = MOSI> + 'd,
        miso: impl Peripheral<P = MISO> + 'd,
        cs: impl Peripheral<P = CS> + 'd,
        mode: SpiMode,
    ) -> Spi<'d, T, FullDuplexMode> {
        crate::into_ref!(spi, sclk, mosi, miso, cs);

        sclk.enable_input(true, private::Internal);
        sclk.connect_input_to_peripheral(spi.sclk_signal(), private::Internal);

        mosi.enable_input(true, private::Internal);
        mosi.connect_input_to_peripheral(spi.mosi_signal(), private::Internal);

        miso.set_to_push_pull_output(private::Internal);
        miso.connect_peripheral_to_output(spi.miso_signal(), private::Internal);

        cs.enable_input(true, private::Internal);
        cs.connect_input_to_peripheral(spi.cs_signal(), private::Internal);

        Self::new_internal(spi, mode)
    }

    pub(crate) fn new_internal(
        spi: PeripheralRef<'d, T>,
        mode: SpiMode,
    ) -> Spi<'d, T, FullDuplexMode> {
        spi.reset_peripheral();
        spi.enable_peripheral();

        let mut spi = Spi {
            spi,
            data_mode: mode,
            _mode: PhantomData,
        };
        spi.spi.init();
        spi.spi.set_data_mode(mode, false);

        spi
    }
}

/// DMA (Direct Memory Access) functionality (Slave).
pub mod dma {
    use super::*;
    #[cfg(spi3)]
    use crate::dma::Spi3Peripheral;
    use crate::{
        dma::{
            dma_private::{DmaSupport, DmaSupportRx, DmaSupportTx},
            Channel,
            ChannelRx,
            ChannelTx,
            DescriptorChain,
            DmaChannel,
            DmaDescriptor,
            DmaTransferRx,
            DmaTransferRxTx,
            DmaTransferTx,
            ReadBuffer,
            Rx,
            Spi2Peripheral,
            SpiPeripheral,
            Tx,
            WriteBuffer,
        },
        Mode,
    };

    impl<'d> Spi<'d, crate::peripherals::SPI2, FullDuplexMode> {
        /// Configures the SPI3 peripheral with the provided DMA channel and
        /// descriptors.
        #[cfg_attr(esp32, doc = "\nNote: ESP32 only supports Mode 1 and 3")]
        pub fn with_dma<C, DmaMode>(
            mut self,
            channel: Channel<'d, C, DmaMode>,
            rx_descriptors: &'static mut [DmaDescriptor],
            tx_descriptors: &'static mut [DmaDescriptor],
        ) -> SpiDma<'d, crate::peripherals::SPI2, C, DmaMode>
        where
            C: DmaChannel,
            C::P: SpiPeripheral + Spi2Peripheral,
            DmaMode: Mode,
        {
            self.spi.set_data_mode(self.data_mode, true);
            SpiDma {
                spi: self.spi,
                channel,
                rx_chain: DescriptorChain::new(rx_descriptors),
                tx_chain: DescriptorChain::new(tx_descriptors),
            }
        }
    }

    #[cfg(spi3)]
    impl<'d> Spi<'d, crate::peripherals::SPI3, FullDuplexMode> {
        /// Configures the SPI3 peripheral with the provided DMA channel and
        /// descriptors.
        #[cfg_attr(esp32, doc = "\nNote: ESP32 only supports Mode 1 and 3")]
        pub fn with_dma<C, DmaMode>(
            mut self,
            channel: Channel<'d, C, DmaMode>,
            rx_descriptors: &'static mut [DmaDescriptor],
            tx_descriptors: &'static mut [DmaDescriptor],
        ) -> SpiDma<'d, crate::peripherals::SPI3, C, DmaMode>
        where
            C: DmaChannel,
            C::P: SpiPeripheral + Spi3Peripheral,
            DmaMode: Mode,
        {
            self.spi.set_data_mode(self.data_mode, true);
            SpiDma {
                spi: self.spi,
                channel,
                rx_chain: DescriptorChain::new(rx_descriptors),
                tx_chain: DescriptorChain::new(tx_descriptors),
            }
        }
    }

    /// A DMA capable SPI instance.
    pub struct SpiDma<'d, T, C, DmaMode>
    where
        C: DmaChannel,
        C::P: SpiPeripheral,
        DmaMode: Mode,
    {
        pub(crate) spi: PeripheralRef<'d, T>,
        pub(crate) channel: Channel<'d, C, DmaMode>,
        rx_chain: DescriptorChain,
        tx_chain: DescriptorChain,
    }

    impl<'d, T, C, DmaMode> core::fmt::Debug for SpiDma<'d, T, C, DmaMode>
    where
        C: DmaChannel,
        C::P: SpiPeripheral,
        DmaMode: Mode,
    {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
            f.debug_struct("SpiDma").finish()
        }
    }

    impl<'d, T, C, DmaMode> DmaSupport for SpiDma<'d, T, C, DmaMode>
    where
        T: InstanceDma,
        C: DmaChannel,
        C::P: SpiPeripheral,
        DmaMode: Mode,
    {
        fn peripheral_wait_dma(&mut self, is_rx: bool, is_tx: bool) {
            while (is_tx && !self.channel.tx.is_done()) || (is_rx && !self.channel.rx.is_done()) {}

            self.spi.flush().ok();
        }

        fn peripheral_dma_stop(&mut self) {
            unreachable!("unsupported")
        }
    }

    impl<'d, T, C, DmaMode> DmaSupportTx for SpiDma<'d, T, C, DmaMode>
    where
        T: InstanceDma,
        C: DmaChannel,
        C::P: SpiPeripheral,
        DmaMode: Mode,
    {
        type TX = ChannelTx<'d, C>;

        fn tx(&mut self) -> &mut Self::TX {
            &mut self.channel.tx
        }

        fn chain(&mut self) -> &mut DescriptorChain {
            &mut self.tx_chain
        }
    }

    impl<'d, T, C, DmaMode> DmaSupportRx for SpiDma<'d, T, C, DmaMode>
    where
        T: InstanceDma,
        C: DmaChannel,
        C::P: SpiPeripheral,
        DmaMode: Mode,
    {
        type RX = ChannelRx<'d, C>;

        fn rx(&mut self) -> &mut Self::RX {
            &mut self.channel.rx
        }

        fn chain(&mut self) -> &mut DescriptorChain {
            &mut self.rx_chain
        }
    }

    impl<'d, T, C, DmaMode> SpiDma<'d, T, C, DmaMode>
    where
        T: InstanceDma,
        C: DmaChannel,
        C::P: SpiPeripheral,
        DmaMode: Mode,
    {
        /// Register a buffer for a DMA write.
        ///
        /// This will return a [DmaTransferTx]. The maximum amount of data to be
        /// sent is 32736 bytes.
        ///
        /// The write is driven by the SPI master's sclk signal and cs line.
        pub fn dma_write<'t, TXBUF>(
            &'t mut self,
            words: &'t TXBUF,
        ) -> Result<DmaTransferTx<'t, Self>, Error>
        where
            TXBUF: ReadBuffer,
        {
            let (ptr, len) = unsafe { words.read_buffer() };

            if len > MAX_DMA_SIZE {
                return Err(Error::MaxDmaTransferSizeExceeded);
            }

            unsafe {
                self.spi
                    .start_transfer_dma(
                        &mut self.rx_chain,
                        &mut self.tx_chain,
                        core::ptr::null_mut(),
                        0,
                        ptr,
                        len,
                        &mut self.channel.rx,
                        &mut self.channel.tx,
                    )
                    .map(move |_| DmaTransferTx::new(self))
            }
        }

        /// Register a buffer for a DMA read.
        ///
        /// This will return a [DmaTransferRx]. The maximum amount of data to be
        /// received is 32736 bytes.
        ///
        /// The read is driven by the SPI master's sclk signal and cs line.
        pub fn dma_read<'t, RXBUF>(
            &'t mut self,
            words: &'t mut RXBUF,
        ) -> Result<DmaTransferRx<'t, Self>, Error>
        where
            RXBUF: WriteBuffer,
        {
            let (ptr, len) = unsafe { words.write_buffer() };

            if len > MAX_DMA_SIZE {
                return Err(Error::MaxDmaTransferSizeExceeded);
            }

            unsafe {
                self.spi
                    .start_transfer_dma(
                        &mut self.rx_chain,
                        &mut self.tx_chain,
                        ptr,
                        len,
                        core::ptr::null(),
                        0,
                        &mut self.channel.rx,
                        &mut self.channel.tx,
                    )
                    .map(move |_| DmaTransferRx::new(self))
            }
        }

        /// Register buffers for a DMA transfer.
        ///
        /// This will return a [DmaTransferRxTx]. The maximum amount of data to
        /// be sent/received is 32736 bytes.
        ///
        /// The data transfer is driven by the SPI master's sclk signal and cs
        /// line.
        pub fn dma_transfer<'t, RXBUF, TXBUF>(
            &'t mut self,
            read_buffer: &'t mut RXBUF,
            words: &'t TXBUF,
        ) -> Result<DmaTransferRxTx<'t, Self>, Error>
        where
            RXBUF: WriteBuffer,
            TXBUF: ReadBuffer,
        {
            let (write_ptr, write_len) = unsafe { words.read_buffer() };
            let (read_ptr, read_len) = unsafe { read_buffer.write_buffer() };

            if write_len > MAX_DMA_SIZE || read_len > MAX_DMA_SIZE {
                return Err(Error::MaxDmaTransferSizeExceeded);
            }

            unsafe {
                self.spi
                    .start_transfer_dma(
                        &mut self.rx_chain,
                        &mut self.tx_chain,
                        read_ptr,
                        read_len,
                        write_ptr,
                        write_len,
                        &mut self.channel.rx,
                        &mut self.channel.tx,
                    )
                    .map(move |_| DmaTransferRxTx::new(self))
            }
        }
    }
}

#[doc(hidden)]
pub trait InstanceDma: Instance {
    fn dma_peripheral(&self) -> DmaPeripheral;

    #[allow(clippy::too_many_arguments)]
    unsafe fn start_transfer_dma<RX, TX>(
        &mut self,
        rx_chain: &mut DescriptorChain,
        tx_chain: &mut DescriptorChain,
        read_buffer_ptr: *mut u8,
        read_buffer_len: usize,
        write_buffer_ptr: *const u8,
        write_buffer_len: usize,
        rx: &mut RX,
        tx: &mut TX,
    ) -> Result<(), Error>
    where
        RX: Rx,
        TX: Tx,
    {
        let reg_block = self.register_block();

        self.enable_dma();

        reset_spi(reg_block);

        if read_buffer_len > 0 {
            rx_chain.fill_for_rx(false, read_buffer_ptr, read_buffer_len)?;
            rx.prepare_transfer_without_start(self.dma_peripheral(), rx_chain)?;
        }

        if write_buffer_len > 0 {
            tx_chain.fill_for_tx(false, write_buffer_ptr, write_buffer_len)?;
            tx.prepare_transfer_without_start(self.dma_peripheral(), tx_chain)?;
        }

        #[cfg(esp32)]
        self.prepare_length_and_lines(read_buffer_len, write_buffer_len);

        reset_dma_before_usr_cmd(reg_block);

        #[cfg(not(esp32))]
        reg_block
            .dma_conf()
            .modify(|_, w| w.dma_slv_seg_trans_en().clear_bit());

        self.clear_dma_interrupts();
        self.setup_for_flush();
        reg_block.cmd().modify(|_, w| w.usr().set_bit());

        if read_buffer_len > 0 {
            rx.start_transfer()?;
        }

        if write_buffer_len > 0 {
            tx.start_transfer()?;
        }

        Ok(())
    }

    fn enable_dma(&self) {
        let reg_block = self.register_block();
        #[cfg(gdma)]
        {
            reg_block.dma_conf().modify(|_, w| {
                w.dma_tx_ena().set_bit();
                w.dma_rx_ena().set_bit();
                w.rx_eof_en().clear_bit()
            });
        }

        #[cfg(pdma)]
        {
            fn set_rst_bit(reg_block: &RegisterBlock, bit: bool) {
                reg_block.dma_conf().modify(|_, w| {
                    w.in_rst().bit(bit);
                    w.out_rst().bit(bit);
                    w.ahbm_fifo_rst().bit(bit);
                    w.ahbm_rst().bit(bit)
                });

                #[cfg(esp32s2)]
                reg_block
                    .dma_conf()
                    .modify(|_, w| w.dma_infifo_full_clr().bit(bit));
            }
            set_rst_bit(reg_block, true);
            set_rst_bit(reg_block, false);
        }
    }

    fn clear_dma_interrupts(&self) {
        let reg_block = self.register_block();

        #[cfg(gdma)]
        reg_block.dma_int_clr().write(|w| {
            w.dma_infifo_full_err().clear_bit_by_one();
            w.dma_outfifo_empty_err().clear_bit_by_one();
            w.trans_done().clear_bit_by_one();
            w.mst_rx_afifo_wfull_err().clear_bit_by_one();
            w.mst_tx_afifo_rempty_err().clear_bit_by_one()
        });

        #[cfg(pdma)]
        reg_block.dma_int_clr().write(|w| {
            w.inlink_dscr_empty().clear_bit_by_one();
            w.outlink_dscr_error().clear_bit_by_one();
            w.inlink_dscr_error().clear_bit_by_one();
            w.in_done().clear_bit_by_one();
            w.in_err_eof().clear_bit_by_one();
            w.in_suc_eof().clear_bit_by_one();
            w.out_done().clear_bit_by_one();
            w.out_eof().clear_bit_by_one();
            w.out_total_eof().clear_bit_by_one()
        });
    }
}

fn reset_spi(reg_block: &RegisterBlock) {
    #[cfg(esp32)]
    {
        reg_block.slave().modify(|_, w| w.sync_reset().set_bit());
        reg_block.slave().modify(|_, w| w.sync_reset().clear_bit());
    }

    #[cfg(not(esp32))]
    {
        reg_block.slave().modify(|_, w| w.soft_reset().set_bit());
        reg_block.slave().modify(|_, w| w.soft_reset().clear_bit());
    }
}

fn reset_dma_before_usr_cmd(reg_block: &RegisterBlock) {
    #[cfg(gdma)]
    reg_block.dma_conf().modify(|_, w| {
        w.rx_afifo_rst().set_bit();
        w.buf_afifo_rst().set_bit();
        w.dma_afifo_rst().set_bit()
    });

    #[cfg(pdma)]
    let _ = reg_block;
}

impl InstanceDma for crate::peripherals::SPI2 {
    fn dma_peripheral(&self) -> DmaPeripheral {
        DmaPeripheral::Spi2
    }
}
#[cfg(spi3)]
impl InstanceDma for crate::peripherals::SPI3 {
    fn dma_peripheral(&self) -> DmaPeripheral {
        DmaPeripheral::Spi3
    }
}

#[doc(hidden)]
pub trait Instance: private::Sealed {
    fn register_block(&self) -> &RegisterBlock;

    fn peripheral(&self) -> crate::system::Peripheral;

    fn sclk_signal(&self) -> InputSignal;

    fn mosi_signal(&self) -> InputSignal;

    fn miso_signal(&self) -> OutputSignal;

    fn cs_signal(&self) -> InputSignal;

    #[inline(always)]
    fn reset_peripheral(&self) {
        PeripheralClockControl::reset(self.peripheral());
    }

    #[inline(always)]
    fn enable_peripheral(&self) {
        PeripheralClockControl::enable(self.peripheral());
    }

    fn spi_num(&self) -> u8;

    #[cfg(esp32)]
    fn prepare_length_and_lines(&self, rx_len: usize, tx_len: usize) {
        let reg_block = self.register_block();

        reg_block
            .slv_rdbuf_dlen()
            .write(|w| unsafe { w.bits((rx_len as u32 * 8).saturating_sub(1)) });
        reg_block
            .slv_wrbuf_dlen()
            .write(|w| unsafe { w.bits((tx_len as u32 * 8).saturating_sub(1)) });

        // SPI Slave mode on ESP32 requires MOSI/MISO enable
        reg_block.user().modify(|_, w| {
            w.usr_mosi().bit(rx_len > 0);
            w.usr_miso().bit(tx_len > 0)
        });
    }

    /// Initialize for full-duplex 1 bit mode
    fn init(&mut self) {
        let reg_block = self.register_block();

        reg_block.clock().write(|w| unsafe { w.bits(0) });
        reg_block.user().write(|w| unsafe { w.bits(0) });
        reg_block.ctrl().write(|w| unsafe { w.bits(0) });

        reg_block.slave().write(|w| {
            #[cfg(esp32)]
            w.slv_wr_rd_buf_en().set_bit();

            w.mode().set_bit()
        });
        reset_spi(reg_block);

        reg_block.user().modify(|_, w| {
            w.doutdin().set_bit();
            w.sio().clear_bit()
        });

        #[cfg(not(esp32))]
        reg_block.misc().write(|w| unsafe { w.bits(0) });
    }

    fn set_data_mode(&mut self, data_mode: SpiMode, dma: bool) -> &mut Self {
        let reg_block = self.register_block();
        #[cfg(esp32)]
        {
            reg_block.pin().modify(|_, w| {
                w.ck_idle_edge()
                    .bit(matches!(data_mode, SpiMode::Mode0 | SpiMode::Mode1))
            });
            reg_block.user().modify(|_, w| {
                w.ck_i_edge()
                    .bit(matches!(data_mode, SpiMode::Mode1 | SpiMode::Mode2))
            });
            reg_block.ctrl2().modify(|_, w| unsafe {
                match data_mode {
                    SpiMode::Mode0 => {
                        w.miso_delay_mode().bits(0);
                        w.miso_delay_num().bits(0);
                        w.mosi_delay_mode().bits(2);
                        w.mosi_delay_num().bits(2)
                    }
                    SpiMode::Mode1 => {
                        w.miso_delay_mode().bits(2);
                        w.miso_delay_num().bits(0);
                        w.mosi_delay_mode().bits(0);
                        w.mosi_delay_num().bits(0)
                    }
                    SpiMode::Mode2 => {
                        w.miso_delay_mode().bits(0);
                        w.miso_delay_num().bits(0);
                        w.mosi_delay_mode().bits(1);
                        w.mosi_delay_num().bits(2)
                    }
                    SpiMode::Mode3 => {
                        w.miso_delay_mode().bits(1);
                        w.miso_delay_num().bits(0);
                        w.mosi_delay_mode().bits(0);
                        w.mosi_delay_num().bits(0)
                    }
                }
            });

            if dma {
                assert!(
                    matches!(data_mode, SpiMode::Mode1 | SpiMode::Mode3),
                    "Mode {:?} is not supported with DMA",
                    data_mode
                );
            }
        }

        #[cfg(not(esp32))]
        {
            _ = dma;
            cfg_if::cfg_if! {
                if #[cfg(esp32s2)] {
                    let ctrl1_reg = reg_block.ctrl1();
                } else {
                    let ctrl1_reg = reg_block.slave();
                }
            }
            reg_block.user().modify(|_, w| {
                w.tsck_i_edge()
                    .bit(matches!(data_mode, SpiMode::Mode1 | SpiMode::Mode2));
                w.rsck_i_edge()
                    .bit(matches!(data_mode, SpiMode::Mode1 | SpiMode::Mode2))
            });
            ctrl1_reg.modify(|_, w| {
                w.clk_mode_13()
                    .bit(matches!(data_mode, SpiMode::Mode1 | SpiMode::Mode3))
            });
        }

        self
    }

    fn is_bus_busy(&self) -> bool {
        let reg_block = self.register_block();

        #[cfg(pdma)]
        {
            reg_block.slave().read().trans_done().bit_is_clear()
        }
        #[cfg(gdma)]
        {
            reg_block.dma_int_raw().read().trans_done().bit_is_clear()
        }
    }

    // Check if the bus is busy and if it is wait for it to be idle
    fn flush(&mut self) -> Result<(), Error> {
        while self.is_bus_busy() {
            // Wait for bus to be clear
        }
        Ok(())
    }

    // Clear the transaction-done interrupt flag so flush() can work properly. Not
    // used in DMA mode.
    fn setup_for_flush(&self) {
        #[cfg(pdma)]
        self.register_block()
            .slave()
            .modify(|_, w| w.trans_done().clear_bit());
        #[cfg(gdma)]
        self.register_block()
            .dma_int_clr()
            .write(|w| w.trans_done().clear_bit_by_one());
    }
}

#[cfg(spi2)]
impl Instance for crate::peripherals::SPI2 {
    #[inline(always)]
    fn register_block(&self) -> &RegisterBlock {
        self
    }

    #[inline(always)]
    fn spi_num(&self) -> u8 {
        2
    }

    #[inline(always)]
    fn peripheral(&self) -> crate::system::Peripheral {
        crate::system::Peripheral::Spi2
    }

    #[inline(always)]
    fn sclk_signal(&self) -> InputSignal {
        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
                InputSignal::HSPICLK
            } else {
                InputSignal::FSPICLK
            }
        }
    }

    #[inline(always)]
    fn mosi_signal(&self) -> InputSignal {
        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
                InputSignal::HSPID
            } else {
                InputSignal::FSPID
            }
        }
    }

    #[inline(always)]
    fn miso_signal(&self) -> OutputSignal {
        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
                OutputSignal::HSPIQ
            } else {
                OutputSignal::FSPIQ
            }
        }
    }

    #[inline(always)]
    fn cs_signal(&self) -> InputSignal {
        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
                InputSignal::HSPICS0
            } else {
                InputSignal::FSPICS0
            }
        }
    }
}

#[cfg(spi3)]
impl Instance for crate::peripherals::SPI3 {
    #[inline(always)]
    fn register_block(&self) -> &RegisterBlock {
        self
    }

    #[inline(always)]
    fn spi_num(&self) -> u8 {
        3
    }

    #[inline(always)]
    fn peripheral(&self) -> crate::system::Peripheral {
        crate::system::Peripheral::Spi3
    }

    #[inline(always)]
    fn sclk_signal(&self) -> InputSignal {
        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
                InputSignal::VSPICLK
            } else {
                InputSignal::SPI3_CLK
            }
        }
    }

    #[inline(always)]
    fn mosi_signal(&self) -> InputSignal {
        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
                InputSignal::VSPID
            } else {
                InputSignal::SPI3_D
            }
        }
    }

    #[inline(always)]
    fn miso_signal(&self) -> OutputSignal {
        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
                OutputSignal::VSPIQ
            } else {
                OutputSignal::SPI3_Q
            }
        }
    }

    #[inline(always)]
    fn cs_signal(&self) -> InputSignal {
        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
                InputSignal::VSPICS0
            } else {
                InputSignal::SPI3_CS0
            }
        }
    }
}
