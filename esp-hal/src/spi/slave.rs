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
//! ## Examples
//!
//! ### SPI Slave with DMA
//!
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::dma_buffers;
//! # use esp_hal::spi::Mode;
//! # use esp_hal::spi::slave::Spi;
#![cfg_attr(pdma, doc = "let dma_channel = peripherals.DMA_SPI2;")]
#![cfg_attr(gdma, doc = "let dma_channel = peripherals.DMA_CH0;")]
//! let sclk = peripherals.GPIO0;
//! let miso = peripherals.GPIO1;
//! let mosi = peripherals.GPIO2;
//! let cs = peripherals.GPIO3;
//!
//! let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) =
//! dma_buffers!(32000);
//! let mut spi = Spi::new(
//!     peripherals.SPI2,
//!     Mode::Mode0,
//! )
//! .with_sck(sclk)
//! .with_mosi(mosi)
//! .with_miso(miso)
//! .with_cs(cs)
//! .with_dma(
//!     dma_channel,
//!     rx_descriptors,
//!     tx_descriptors,
//! );
//!
//! let mut receive = rx_buffer;
//! let mut send = tx_buffer;
//!
//! let transfer = spi
//!     .transfer(&mut receive, &mut send)
//!     .unwrap();
//!
//! transfer.wait().unwrap();
//! # }
//! ```
//! 
//! ## Implementation State
//!
//! There are several options for working with the SPI peripheral in slave mode,
//! but the code currently only supports:
//! - Single transfers (not segmented transfers)
//! - Full duplex, single bit (not dual or quad SPI)
//! - DMA mode (not CPU mode).
#![cfg_attr(esp32, doc = "- ESP32 only supports SPI mode 1 and 3.\n\n")]
//! It also does not support blocking operations, as the actual
//! transfer is controlled by the SPI master; if these are necessary,
//! then the `DmaTransfer` object can be `wait()`ed on or polled for
//! `is_done()`.
//!
//! See [tracking issue](https://github.com/esp-rs/esp-hal/issues/469) for more information.

use core::marker::PhantomData;

use super::{Error, Mode};
use crate::{
    dma::DmaEligible,
    gpio::{
        interconnect::{PeripheralInput, PeripheralOutput},
        InputSignal,
        NoPin,
        OutputSignal,
    },
    peripheral::{Peripheral, PeripheralRef},
    peripherals::spi2::RegisterBlock,
    private,
    spi::AnySpi,
    system::PeripheralGuard,
    Blocking,
};

const MAX_DMA_SIZE: usize = 32768 - 32;

/// SPI peripheral driver.
///
/// See the [module-level documentation][self] for more details.
pub struct Spi<'d, Dm, T = AnySpi> {
    spi: PeripheralRef<'d, T>,
    #[allow(dead_code)]
    data_mode: Mode,
    _mode: PhantomData<Dm>,
    _guard: PeripheralGuard,
}

impl<'d> Spi<'d, Blocking> {
    /// Constructs an SPI instance in 8bit dataframe mode.
    pub fn new(spi: impl Peripheral<P = impl Instance> + 'd, mode: Mode) -> Spi<'d, Blocking> {
        Self::new_typed(spi.map_into(), mode)
    }
}

impl<'d, Dm, T> Spi<'d, Dm, T>
where
    T: Instance,
{
    /// Constructs an SPI instance in 8bit dataframe mode.
    pub fn new_typed(spi: impl Peripheral<P = T> + 'd, mode: Mode) -> Spi<'d, Dm, T> {
        crate::into_ref!(spi);

        let guard = PeripheralGuard::new(spi.info().peripheral);

        let this = Spi {
            spi,
            data_mode: mode,
            _mode: PhantomData,
            _guard: guard,
        };

        this.spi.info().init();
        this.spi.info().set_data_mode(mode, false);

        this.with_mosi(NoPin)
            .with_miso(NoPin)
            .with_sck(NoPin)
            .with_cs(NoPin)
    }

    /// Assign the SCK (Serial Clock) pin for the SPI instance.
    pub fn with_sck(self, sclk: impl Peripheral<P = impl PeripheralInput> + 'd) -> Self {
        crate::into_mapped_ref!(sclk);
        sclk.enable_input(true, private::Internal);
        self.spi.info().sclk.connect_to(sclk);
        self
    }

    /// Assign the MOSI (Master Out Slave In) pin for the SPI instance.
    pub fn with_mosi(self, mosi: impl Peripheral<P = impl PeripheralInput> + 'd) -> Self {
        crate::into_mapped_ref!(mosi);
        mosi.enable_input(true, private::Internal);
        self.spi.info().mosi.connect_to(mosi);
        self
    }

    /// Assign the MISO (Master In Slave Out) pin for the SPI instance.
    pub fn with_miso(self, miso: impl Peripheral<P = impl PeripheralOutput> + 'd) -> Self {
        crate::into_mapped_ref!(miso);
        miso.set_to_push_pull_output(private::Internal);
        self.spi.info().miso.connect_to(miso);
        self
    }

    /// Assign the CS (Chip Select) pin for the SPI instance.
    pub fn with_cs(self, cs: impl Peripheral<P = impl PeripheralInput> + 'd) -> Self {
        crate::into_mapped_ref!(cs);
        cs.enable_input(true, private::Internal);
        self.spi.info().cs.connect_to(cs);
        self
    }
}

/// DMA (Direct Memory Access) functionality (Slave).
pub mod dma {
    use super::*;
    use crate::{
        dma::{
            dma_private::{DmaSupport, DmaSupportRx, DmaSupportTx},
            Channel,
            ChannelRx,
            ChannelTx,
            DescriptorChain,
            DmaChannelFor,
            DmaDescriptor,
            DmaTransferRx,
            DmaTransferRxTx,
            DmaTransferTx,
            PeripheralDmaChannel,
            PeripheralRxChannel,
            PeripheralTxChannel,
            ReadBuffer,
            Rx,
            Tx,
            WriteBuffer,
        },
        DriverMode,
    };

    impl<'d, T> Spi<'d, Blocking, T>
    where
        T: InstanceDma,
    {
        /// Configures the SPI peripheral with the provided DMA channel and
        /// descriptors.
        #[cfg_attr(esp32, doc = "\n\n**Note**: ESP32 only supports Mode 1 and 3.")]
        pub fn with_dma<CH>(
            self,
            channel: impl Peripheral<P = CH> + 'd,
            rx_descriptors: &'static mut [DmaDescriptor],
            tx_descriptors: &'static mut [DmaDescriptor],
        ) -> SpiDma<'d, Blocking, T>
        where
            CH: DmaChannelFor<T>,
        {
            self.spi.info().set_data_mode(self.data_mode, true);
            SpiDma::new(
                self.spi,
                channel.map(|ch| ch.degrade()).into_ref(),
                rx_descriptors,
                tx_descriptors,
            )
        }
    }

    /// A DMA capable SPI instance.
    pub struct SpiDma<'d, Dm, T = AnySpi>
    where
        T: InstanceDma,
        Dm: DriverMode,
    {
        pub(crate) spi: PeripheralRef<'d, T>,
        pub(crate) channel: Channel<'d, Dm, PeripheralDmaChannel<T>>,
        rx_chain: DescriptorChain,
        tx_chain: DescriptorChain,
        _guard: PeripheralGuard,
    }

    impl<Dm, T> core::fmt::Debug for SpiDma<'_, Dm, T>
    where
        T: InstanceDma,
        Dm: DriverMode,
    {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
            f.debug_struct("SpiDma").finish()
        }
    }

    impl<Dm, T> DmaSupport for SpiDma<'_, Dm, T>
    where
        T: InstanceDma,
        Dm: DriverMode,
    {
        fn peripheral_wait_dma(&mut self, is_rx: bool, is_tx: bool) {
            while !((!is_tx || self.channel.tx.is_done())
                && (!is_rx || self.channel.rx.is_done())
                && !self.spi.info().is_bus_busy())
            {}

            self.spi.info().flush().ok();
        }

        fn peripheral_dma_stop(&mut self) {
            unreachable!("unsupported")
        }
    }

    impl<'d, Dm, T> DmaSupportTx for SpiDma<'d, Dm, T>
    where
        T: InstanceDma,
        Dm: DriverMode,
    {
        type TX = ChannelTx<'d, Dm, PeripheralTxChannel<T>>;

        fn tx(&mut self) -> &mut Self::TX {
            &mut self.channel.tx
        }

        fn chain(&mut self) -> &mut DescriptorChain {
            &mut self.tx_chain
        }
    }

    impl<'d, Dm, T> DmaSupportRx for SpiDma<'d, Dm, T>
    where
        T: InstanceDma,
        Dm: DriverMode,
    {
        type RX = ChannelRx<'d, Dm, PeripheralRxChannel<T>>;

        fn rx(&mut self) -> &mut Self::RX {
            &mut self.channel.rx
        }

        fn chain(&mut self) -> &mut DescriptorChain {
            &mut self.rx_chain
        }
    }

    impl<'d, T> SpiDma<'d, Blocking, T>
    where
        T: InstanceDma,
    {
        fn new(
            spi: PeripheralRef<'d, T>,
            channel: PeripheralRef<'d, PeripheralDmaChannel<T>>,
            rx_descriptors: &'static mut [DmaDescriptor],
            tx_descriptors: &'static mut [DmaDescriptor],
        ) -> Self {
            let channel = Channel::new(channel);
            channel.runtime_ensure_compatible(&spi);
            let guard = PeripheralGuard::new(spi.info().peripheral);

            Self {
                spi,
                channel,
                rx_chain: DescriptorChain::new(rx_descriptors),
                tx_chain: DescriptorChain::new(tx_descriptors),
                _guard: guard,
            }
        }
    }

    impl<Dm, T> SpiDma<'_, Dm, T>
    where
        Dm: DriverMode,
        T: InstanceDma,
    {
        fn driver(&self) -> DmaDriver {
            DmaDriver {
                info: self.spi.info(),
                dma_peripheral: self.spi.dma_peripheral(),
            }
        }

        /// Register a buffer for a DMA write.
        ///
        /// This will return a [DmaTransferTx]. The maximum amount of data to be
        /// sent is 32736 bytes.
        ///
        /// The write is driven by the SPI master's sclk signal and cs line.
        pub fn write<'t, TXBUF>(
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
                self.driver()
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
        pub fn read<'t, RXBUF>(
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
                self.driver()
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
        pub fn transfer<'t, RXBUF, TXBUF>(
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
                self.driver()
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

    struct DmaDriver {
        info: &'static Info,
        dma_peripheral: crate::dma::DmaPeripheral,
    }

    impl DmaDriver {
        #[allow(clippy::too_many_arguments)]
        unsafe fn start_transfer_dma<RX, TX>(
            &self,
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
            self.enable_dma();

            self.info.reset_spi();

            if read_buffer_len > 0 {
                rx_chain.fill_for_rx(false, read_buffer_ptr, read_buffer_len)?;
                rx.prepare_transfer_without_start(self.dma_peripheral, rx_chain)?;
            }

            if write_buffer_len > 0 {
                tx_chain.fill_for_tx(false, write_buffer_ptr, write_buffer_len)?;
                tx.prepare_transfer_without_start(self.dma_peripheral, tx_chain)?;
            }

            #[cfg(esp32)]
            self.info
                .prepare_length_and_lines(read_buffer_len, write_buffer_len);

            self.reset_dma_before_usr_cmd();

            let reg_block = self.info.register_block();
            #[cfg(not(esp32))]
            reg_block
                .dma_conf()
                .modify(|_, w| w.dma_slv_seg_trans_en().clear_bit());

            self.clear_dma_interrupts();
            self.info.setup_for_flush();
            reg_block.cmd().modify(|_, w| w.usr().set_bit());

            if read_buffer_len > 0 {
                rx.start_transfer()?;
            }

            if write_buffer_len > 0 {
                tx.start_transfer()?;
            }

            Ok(())
        }

        fn reset_dma_before_usr_cmd(&self) {
            let reg_block = self.info.register_block();
            #[cfg(gdma)]
            reg_block.dma_conf().modify(|_, w| {
                w.rx_afifo_rst().set_bit();
                w.buf_afifo_rst().set_bit();
                w.dma_afifo_rst().set_bit()
            });

            #[cfg(pdma)]
            let _ = reg_block;
        }

        fn enable_dma(&self) {
            let reg_block = self.info.register_block();
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
            let reg_block = self.info.register_block();

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
}

/// SPI peripheral instance.
pub trait Instance: Peripheral<P = Self> + Into<AnySpi> + 'static {
    /// Returns the peripheral data describing this SPI instance.
    fn info(&self) -> &'static Info;
}

/// A marker for DMA-capable SPI peripheral instances.
pub trait InstanceDma: Instance + DmaEligible {}

impl InstanceDma for crate::peripherals::SPI2 {}
#[cfg(spi3)]
impl InstanceDma for crate::peripherals::SPI3 {}

/// Peripheral data describing a particular SPI instance.
#[non_exhaustive]
pub struct Info {
    /// Pointer to the register block for this SPI instance.
    ///
    /// Use [Self::register_block] to access the register block.
    pub register_block: *const RegisterBlock,

    /// System peripheral marker.
    pub peripheral: crate::system::Peripheral,

    /// SCLK signal.
    pub sclk: InputSignal,

    /// MOSI signal.
    pub mosi: InputSignal,

    /// MISO signal.
    pub miso: OutputSignal,

    /// Chip select signal.
    pub cs: InputSignal,
}

impl Info {
    /// Returns the register block for this SPI instance.
    pub fn register_block(&self) -> &RegisterBlock {
        unsafe { &*self.register_block }
    }

    fn reset_spi(&self) {
        let reg_block = self.register_block();

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
    fn init(&self) {
        let reg_block = self.register_block();

        reg_block.clock().write(|w| unsafe { w.bits(0) });
        reg_block.user().write(|w| unsafe { w.bits(0) });
        reg_block.ctrl().write(|w| unsafe { w.bits(0) });

        reg_block.slave().write(|w| {
            #[cfg(esp32)]
            w.slv_wr_rd_buf_en().set_bit();

            w.mode().set_bit()
        });
        self.reset_spi();

        reg_block.user().modify(|_, w| {
            w.doutdin().set_bit();
            w.sio().clear_bit()
        });

        #[cfg(not(esp32))]
        reg_block.misc().write(|w| unsafe { w.bits(0) });
    }

    fn set_data_mode(&self, data_mode: Mode, dma: bool) {
        let reg_block = self.register_block();
        #[cfg(esp32)]
        {
            reg_block.pin().modify(|_, w| {
                w.ck_idle_edge()
                    .bit(matches!(data_mode, Mode::Mode0 | Mode::Mode1))
            });
            reg_block.user().modify(|_, w| {
                w.ck_i_edge()
                    .bit(matches!(data_mode, Mode::Mode1 | Mode::Mode2))
            });
            reg_block.ctrl2().modify(|_, w| unsafe {
                match data_mode {
                    Mode::Mode0 => {
                        w.miso_delay_mode().bits(0);
                        w.miso_delay_num().bits(0);
                        w.mosi_delay_mode().bits(2);
                        w.mosi_delay_num().bits(2)
                    }
                    Mode::Mode1 => {
                        w.miso_delay_mode().bits(2);
                        w.miso_delay_num().bits(0);
                        w.mosi_delay_mode().bits(0);
                        w.mosi_delay_num().bits(0)
                    }
                    Mode::Mode2 => {
                        w.miso_delay_mode().bits(0);
                        w.miso_delay_num().bits(0);
                        w.mosi_delay_mode().bits(1);
                        w.mosi_delay_num().bits(2)
                    }
                    Mode::Mode3 => {
                        w.miso_delay_mode().bits(1);
                        w.miso_delay_num().bits(0);
                        w.mosi_delay_mode().bits(0);
                        w.mosi_delay_num().bits(0)
                    }
                }
            });

            if dma {
                assert!(
                    matches!(data_mode, Mode::Mode1 | Mode::Mode3),
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
                    .bit(matches!(data_mode, Mode::Mode1 | Mode::Mode2));
                w.rsck_i_edge()
                    .bit(matches!(data_mode, Mode::Mode1 | Mode::Mode2))
            });
            ctrl1_reg.modify(|_, w| {
                w.clk_mode_13()
                    .bit(matches!(data_mode, Mode::Mode1 | Mode::Mode3))
            });
        }
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
    fn flush(&self) -> Result<(), Error> {
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

impl PartialEq for Info {
    fn eq(&self, other: &Self) -> bool {
        self.register_block == other.register_block
    }
}

unsafe impl Sync for Info {}

macro_rules! spi_instance {
    ($num:literal, $sclk:ident, $mosi:ident, $miso:ident, $cs:ident) => {
        paste::paste! {
            impl Instance for crate::peripherals::[<SPI $num>] {
                #[inline(always)]
                fn info(&self) -> &'static Info {
                    static INFO: Info = Info {
                        register_block: crate::peripherals::[<SPI $num>]::PTR,
                        peripheral: crate::system::Peripheral::[<Spi $num>],
                        sclk: InputSignal::$sclk,
                        mosi: InputSignal::$mosi,
                        miso: OutputSignal::$miso,
                        cs: InputSignal::$cs,
                    };

                    &INFO
                }
            }
        }
    };
}

cfg_if::cfg_if! {
    if #[cfg(esp32)] {
        #[cfg(spi2)]
        spi_instance!(2, HSPICLK, HSPID, HSPIQ, HSPICS0);
        #[cfg(spi3)]
        spi_instance!(3, VSPICLK, VSPID, VSPIQ, VSPICS0);
    } else {
        #[cfg(spi2)]
        spi_instance!(2, FSPICLK, FSPID, FSPIQ, FSPICS0);
        #[cfg(spi3)]
        spi_instance!(3, SPI3_CLK, SPI3_D, SPI3_Q, SPI3_CS0);
    }
}

impl Instance for super::AnySpi {
    delegate::delegate! {
        to match &self.0 {
            super::AnySpiInner::Spi2(spi) => spi,
            #[cfg(spi3)]
            super::AnySpiInner::Spi3(spi) => spi,
        } {
            fn info(&self) -> &'static Info;
        }
    }
}

impl InstanceDma for super::AnySpi {}
