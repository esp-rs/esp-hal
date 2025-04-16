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
//! # use esp_hal::dma::{DmaRxBuf, DmaTxBuf};
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
//! let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
//! let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();
//! let mut spi = Spi::new(
//!     peripherals.SPI2,
//!     Mode::_0,
//! )
//! .with_sck(sclk)
//! .with_mosi(mosi)
//! .with_miso(miso)
//! .with_cs(cs)
//! .with_dma(dma_channel);
//!
//! let transfer = spi
//!     .transfer(50, dma_rx_buf, 50, dma_tx_buf)?;
//!
//! transfer.wait();
//! # Ok(())
//! # }
//! ```
//! 
//! ## Implementation State
//!
//! This driver is currently **unstable**.
//!
//! There are several options for working with the SPI peripheral in slave mode,
//! but the code currently only supports:
//! - Single transfers (not segmented transfers)
//! - Full duplex, single bit (not dual or quad SPI)
//! - DMA mode (not CPU mode).
#![cfg_attr(esp32, doc = "- ESP32 only supports SPI mode 1 and 3.\n\n")]
//! It also does not support blocking operations, as the actual
//! transfer is controlled by the SPI master; if these are necessary,
//! then the `SpiDmaTransfer` object can be `wait()`ed on or polled for
//! `is_done()`.
//!
//! See [tracking issue](https://github.com/esp-rs/esp-hal/issues/469) for more information.

use core::marker::PhantomData;

use super::{Error, Mode};
use crate::{
    Blocking,
    DriverMode,
    dma::DmaEligible,
    gpio::{
        InputSignal,
        NoPin,
        OutputSignal,
        interconnect::{PeripheralInput, PeripheralOutput},
    },
    pac::spi2::RegisterBlock,
    spi::AnySpi,
    system::PeripheralGuard,
};

const MAX_DMA_SIZE: usize = 32768 - 32;

/// SPI peripheral driver.
///
/// See the [module-level documentation][self] for more details.
#[instability::unstable]
pub struct Spi<'d, Dm: DriverMode> {
    spi: AnySpi<'d>,
    #[allow(dead_code)]
    data_mode: Mode,
    _mode: PhantomData<Dm>,
    _guard: PeripheralGuard,
}
impl<'d> Spi<'d, Blocking> {
    /// Constructs an SPI instance in 8bit dataframe mode.
    #[instability::unstable]
    pub fn new(spi: impl Instance + 'd, mode: Mode) -> Spi<'d, Blocking> {
        let guard = PeripheralGuard::new(spi.info().peripheral);

        let this = Spi {
            spi: spi.degrade(),
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
    #[instability::unstable]
    pub fn with_sck(self, sclk: impl PeripheralInput<'d>) -> Self {
        let sclk = sclk.into();
        sclk.enable_input(true);
        self.spi.info().sclk.connect_to(&sclk);
        self
    }

    /// Assign the MOSI (Master Out Slave In) pin for the SPI instance.
    #[instability::unstable]
    pub fn with_mosi(self, mosi: impl PeripheralInput<'d>) -> Self {
        let mosi = mosi.into();
        mosi.enable_input(true);
        self.spi.info().mosi.connect_to(&mosi);
        self
    }

    /// Assign the MISO (Master In Slave Out) pin for the SPI instance.
    #[instability::unstable]
    pub fn with_miso(self, miso: impl PeripheralOutput<'d>) -> Self {
        let miso = miso.into();
        miso.set_to_push_pull_output();
        self.spi.info().miso.connect_to(&miso);
        self
    }

    /// Assign the CS (Chip Select) pin for the SPI instance.
    #[instability::unstable]
    pub fn with_cs(self, cs: impl PeripheralInput<'d>) -> Self {
        let cs = cs.into();
        cs.enable_input(true);
        self.spi.info().cs.connect_to(&cs);
        self
    }
}

/// DMA (Direct Memory Access) functionality (Slave).
#[instability::unstable]
pub mod dma {
    use core::mem::ManuallyDrop;

    use enumset::enum_set;

    use super::*;
    use crate::{
        DriverMode,
        dma::{
            Channel,
            DmaChannelFor,
            DmaRxBuffer,
            DmaRxInterrupt,
            DmaTxBuffer,
            EmptyBuf,
            PeripheralDmaChannel,
        },
    };

    impl<'d> Spi<'d, Blocking> {
        /// Configures the SPI peripheral with the provided DMA channel and
        /// descriptors.
        #[cfg_attr(esp32, doc = "\n\n**Note**: ESP32 only supports Mode 1 and 3.")]
        #[instability::unstable]
        pub fn with_dma(self, channel: impl DmaChannelFor<AnySpi<'d>>) -> SpiDma<'d, Blocking> {
            self.spi.info().set_data_mode(self.data_mode, true);
            SpiDma::new(self.spi, channel.degrade())
        }
    }

    /// A structure representing a DMA transfer for SPI.
    #[instability::unstable]
    pub struct SpiDma<'d, Dm>
    where
        Dm: DriverMode,
    {
        pub(crate) spi: AnySpi<'d>,
        pub(crate) channel: Channel<Dm, PeripheralDmaChannel<AnySpi<'d>>>,
        _guard: PeripheralGuard,
    }

    impl<Dm> core::fmt::Debug for SpiDma<'_, Dm>
    where
        Dm: DriverMode,
    {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
            f.debug_struct("SpiDma").finish()
        }
    }

    impl<'d> SpiDma<'d, Blocking> {
        fn new(spi: AnySpi<'d>, channel: PeripheralDmaChannel<AnySpi<'d>>) -> Self {
            let channel = Channel::new(channel);
            channel.runtime_ensure_compatible(&spi);
            let guard = PeripheralGuard::new(spi.info().peripheral);

            Self {
                spi,
                channel,
                _guard: guard,
            }
        }
    }

    impl<'d, Dm> SpiDma<'d, Dm>
    where
        Dm: DriverMode,
    {
        fn driver(&self) -> DmaDriver {
            DmaDriver {
                info: self.spi.info(),
                dma_peripheral: self.spi.dma_peripheral(),
            }
        }

        /// Register a buffer for a DMA write.
        ///
        /// This will return a [SpiDmaTransfer]. The maximum amount of data to
        /// be sent is 32736 bytes.
        ///
        /// The write is driven by the SPI master's sclk signal and cs line.
        #[instability::unstable]
        pub fn write<TX>(
            mut self,
            bytes_to_write: usize,
            mut buffer: TX,
        ) -> Result<SpiDmaTransfer<'d, Dm, TX>, (Error, Self, TX)>
        where
            TX: DmaTxBuffer,
        {
            if bytes_to_write > MAX_DMA_SIZE {
                return Err((Error::MaxDmaTransferSizeExceeded, self, buffer));
            }

            let result = unsafe {
                self.driver().start_transfer_dma(
                    0,
                    bytes_to_write,
                    &mut EmptyBuf,
                    &mut buffer,
                    &mut self.channel,
                )
            };
            if let Err(err) = result {
                return Err((err, self, buffer));
            }

            Ok(SpiDmaTransfer::new(self, buffer, false, true))
        }

        /// Register a buffer for a DMA read.
        ///
        /// This will return a [SpiDmaTransfer]. The maximum amount of data to
        /// be received is 32736 bytes.
        ///
        /// The read is driven by the SPI master's sclk signal and cs line.
        #[instability::unstable]
        pub fn read<RX>(
            mut self,
            bytes_to_read: usize,
            mut buffer: RX,
        ) -> Result<SpiDmaTransfer<'d, Dm, RX>, (Error, Self, RX)>
        where
            RX: DmaRxBuffer,
        {
            if bytes_to_read > MAX_DMA_SIZE {
                return Err((Error::MaxDmaTransferSizeExceeded, self, buffer));
            }

            let result = unsafe {
                self.driver().start_transfer_dma(
                    bytes_to_read,
                    0,
                    &mut buffer,
                    &mut EmptyBuf,
                    &mut self.channel,
                )
            };
            if let Err(err) = result {
                return Err((err, self, buffer));
            }

            Ok(SpiDmaTransfer::new(self, buffer, true, false))
        }

        /// Register buffers for a DMA transfer.
        ///
        /// This will return a [SpiDmaTransfer]. The maximum amount of data to
        /// be sent/received is 32736 bytes.
        ///
        /// The data transfer is driven by the SPI master's sclk signal and cs
        /// line.
        #[instability::unstable]
        #[allow(clippy::type_complexity)]
        pub fn transfer<RX, TX>(
            mut self,
            bytes_to_read: usize,
            mut rx_buffer: RX,
            bytes_to_write: usize,
            mut tx_buffer: TX,
        ) -> Result<SpiDmaTransfer<'d, Dm, (RX, TX)>, (Error, Self, RX, TX)>
        where
            RX: DmaRxBuffer,
            TX: DmaTxBuffer,
        {
            if bytes_to_read > MAX_DMA_SIZE || bytes_to_write > MAX_DMA_SIZE {
                return Err((
                    Error::MaxDmaTransferSizeExceeded,
                    self,
                    rx_buffer,
                    tx_buffer,
                ));
            }

            let result = unsafe {
                self.driver().start_transfer_dma(
                    bytes_to_read,
                    bytes_to_write,
                    &mut rx_buffer,
                    &mut tx_buffer,
                    &mut self.channel,
                )
            };
            if let Err(err) = result {
                return Err((err, self, rx_buffer, tx_buffer));
            }

            Ok(SpiDmaTransfer::new(
                self,
                (rx_buffer, tx_buffer),
                true,
                true,
            ))
        }
    }

    /// A structure representing a DMA transfer for SPI.
    ///
    /// This structure holds references to the SPI instance, DMA buffers, and
    /// transfer status.
    #[instability::unstable]
    pub struct SpiDmaTransfer<'d, Dm, Buf>
    where
        Dm: DriverMode,
    {
        spi_dma: ManuallyDrop<SpiDma<'d, Dm>>,
        dma_buf: ManuallyDrop<Buf>,
        has_rx: bool,
        has_tx: bool,
    }

    impl<'d, Dm, Buf> SpiDmaTransfer<'d, Dm, Buf>
    where
        Dm: DriverMode,
    {
        fn new(spi_dma: SpiDma<'d, Dm>, dma_buf: Buf, has_rx: bool, has_tx: bool) -> Self {
            Self {
                spi_dma: ManuallyDrop::new(spi_dma),
                dma_buf: ManuallyDrop::new(dma_buf),
                has_rx,
                has_tx,
            }
        }

        /// Checks if the transfer is complete.
        ///
        /// This method returns `true` if both RX and TX operations are done,
        /// and the SPI instance is no longer busy.
        #[instability::unstable]
        pub fn is_done(&self) -> bool {
            if self.has_rx {
                let done_int =
                    enum_set!(DmaRxInterrupt::SuccessfulEof | DmaRxInterrupt::DescriptorEmpty);
                if self
                    .spi_dma
                    .channel
                    .rx
                    .pending_in_interrupts()
                    .is_disjoint(done_int)
                {
                    return false;
                }
            }
            !self.spi_dma.spi.info().is_bus_busy()
        }

        /// Waits for the DMA transfer to complete.
        ///
        /// This method blocks until the transfer is finished and returns the
        /// `SpiDma` instance and the associated buffer.
        #[instability::unstable]
        pub fn wait(mut self) -> (SpiDma<'d, Dm>, Buf) {
            while !self.is_done() {
                // Wait for the SPI to become idle
            }

            if self.has_tx {
                // In case DMA TX buffer is bigger than what the SPI consumes, stop the DMA.
                if !self.spi_dma.channel.tx.is_done() {
                    self.spi_dma.channel.tx.stop_transfer();
                }
            }

            let retval = unsafe {
                (
                    ManuallyDrop::take(&mut self.spi_dma),
                    ManuallyDrop::take(&mut self.dma_buf),
                )
            };
            core::mem::forget(self);
            retval
        }
    }

    impl<Dm, Buf> Drop for SpiDmaTransfer<'_, Dm, Buf>
    where
        Dm: DriverMode,
    {
        fn drop(&mut self) {
            while !self.is_done() {
                // Wait for the SPI to become idle
            }
            unsafe {
                ManuallyDrop::drop(&mut self.spi_dma);
                ManuallyDrop::drop(&mut self.dma_buf);
            }
        }
    }

    struct DmaDriver {
        info: &'static Info,
        dma_peripheral: crate::dma::DmaPeripheral,
    }

    impl DmaDriver {
        fn regs(&self) -> &RegisterBlock {
            self.info.regs()
        }

        #[allow(clippy::too_many_arguments)]
        unsafe fn start_transfer_dma<Dm: DriverMode>(
            &self,
            read_buffer_len: usize,
            write_buffer_len: usize,
            rx_buffer: &mut impl DmaRxBuffer,
            tx_buffer: &mut impl DmaTxBuffer,
            channel: &mut Channel<Dm, PeripheralDmaChannel<AnySpi<'_>>>,
        ) -> Result<(), Error> {
            unsafe {
                self.enable_dma();

                self.info.reset_spi();

                if read_buffer_len > 0 {
                    channel
                        .rx
                        .prepare_transfer(self.dma_peripheral, rx_buffer)?;
                }

                if write_buffer_len > 0 {
                    channel
                        .tx
                        .prepare_transfer(self.dma_peripheral, tx_buffer)?;
                }

                #[cfg(esp32)]
                self.info
                    .prepare_length_and_lines(read_buffer_len, write_buffer_len);

                self.reset_dma_before_usr_cmd();

                #[cfg(not(esp32))]
                self.regs()
                    .dma_conf()
                    .modify(|_, w| w.dma_slv_seg_trans_en().clear_bit());

                self.clear_dma_interrupts();
                self.info.setup_for_flush();
                self.regs().cmd().modify(|_, w| w.usr().set_bit());

                if read_buffer_len > 0 {
                    channel.rx.start_transfer()?;
                }

                if write_buffer_len > 0 {
                    channel.tx.start_transfer()?;
                }

                Ok(())
            }
        }

        fn reset_dma_before_usr_cmd(&self) {
            #[cfg(gdma)]
            self.regs().dma_conf().modify(|_, w| {
                w.rx_afifo_rst().set_bit();
                w.buf_afifo_rst().set_bit();
                w.dma_afifo_rst().set_bit()
            });
        }

        fn enable_dma(&self) {
            #[cfg(gdma)]
            self.regs().dma_conf().modify(|_, w| {
                w.dma_tx_ena().set_bit();
                w.dma_rx_ena().set_bit();
                w.rx_eof_en().clear_bit()
            });

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
                set_rst_bit(self.regs(), true);
                set_rst_bit(self.regs(), false);
            }
        }

        fn clear_dma_interrupts(&self) {
            #[cfg(gdma)]
            self.regs().dma_int_clr().write(|w| {
                w.dma_infifo_full_err().clear_bit_by_one();
                w.dma_outfifo_empty_err().clear_bit_by_one();
                w.trans_done().clear_bit_by_one();
                w.mst_rx_afifo_wfull_err().clear_bit_by_one();
                w.mst_tx_afifo_rempty_err().clear_bit_by_one()
            });

            #[cfg(pdma)]
            self.regs().dma_int_clr().write(|w| {
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

/// A peripheral singleton compatible with the SPI slave driver.
pub trait Instance: crate::private::Sealed + super::IntoAnySpi {
    /// Returns the peripheral data describing this SPI instance.
    #[doc(hidden)]
    fn info(&self) -> &'static Info;
}

/// A marker for DMA-capable SPI peripheral instances.
#[doc(hidden)]
#[allow(private_bounds)]
pub trait InstanceDma: Instance + DmaEligible {}

impl InstanceDma for crate::peripherals::SPI2<'_> {}
#[cfg(spi3)]
impl InstanceDma for crate::peripherals::SPI3<'_> {}

/// Peripheral data describing a particular SPI instance.
#[non_exhaustive]
#[doc(hidden)]
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
    #[instability::unstable]
    pub fn regs(&self) -> &RegisterBlock {
        unsafe { &*self.register_block }
    }

    fn reset_spi(&self) {
        #[cfg(esp32)]
        {
            self.regs().slave().modify(|_, w| w.sync_reset().set_bit());
            self.regs()
                .slave()
                .modify(|_, w| w.sync_reset().clear_bit());
        }

        #[cfg(not(esp32))]
        {
            self.regs().slave().modify(|_, w| w.soft_reset().set_bit());
            self.regs()
                .slave()
                .modify(|_, w| w.soft_reset().clear_bit());
        }
    }

    #[cfg(esp32)]
    fn prepare_length_and_lines(&self, rx_len: usize, tx_len: usize) {
        self.regs()
            .slv_rdbuf_dlen()
            .write(|w| unsafe { w.bits((rx_len as u32 * 8).saturating_sub(1)) });
        self.regs()
            .slv_wrbuf_dlen()
            .write(|w| unsafe { w.bits((tx_len as u32 * 8).saturating_sub(1)) });

        // SPI Slave mode on ESP32 requires MOSI/MISO enable
        self.regs().user().modify(|_, w| {
            w.usr_mosi().bit(rx_len > 0);
            w.usr_miso().bit(tx_len > 0)
        });
    }

    /// Initialize for full-duplex 1 bit mode
    fn init(&self) {
        self.regs().clock().write(|w| unsafe { w.bits(0) });
        self.regs().user().write(|w| unsafe { w.bits(0) });
        self.regs().ctrl().write(|w| unsafe { w.bits(0) });

        self.regs().slave().write(|w| {
            #[cfg(esp32)]
            w.slv_wr_rd_buf_en().set_bit();

            w.mode().set_bit()
        });
        self.reset_spi();

        self.regs().user().modify(|_, w| {
            w.doutdin().set_bit();
            w.sio().clear_bit()
        });

        #[cfg(not(esp32))]
        self.regs().misc().write(|w| unsafe { w.bits(0) });
    }

    fn set_data_mode(&self, data_mode: Mode, dma: bool) {
        #[cfg(esp32)]
        {
            self.regs().pin().modify(|_, w| {
                w.ck_idle_edge()
                    .bit(matches!(data_mode, Mode::_0 | Mode::_1))
            });
            self.regs()
                .user()
                .modify(|_, w| w.ck_i_edge().bit(matches!(data_mode, Mode::_1 | Mode::_2)));
            self.regs().ctrl2().modify(|_, w| unsafe {
                match data_mode {
                    Mode::_0 => {
                        w.miso_delay_mode().bits(0);
                        w.miso_delay_num().bits(0);
                        w.mosi_delay_mode().bits(2);
                        w.mosi_delay_num().bits(2)
                    }
                    Mode::_1 => {
                        w.miso_delay_mode().bits(2);
                        w.miso_delay_num().bits(0);
                        w.mosi_delay_mode().bits(0);
                        w.mosi_delay_num().bits(0)
                    }
                    Mode::_2 => {
                        w.miso_delay_mode().bits(0);
                        w.miso_delay_num().bits(0);
                        w.mosi_delay_mode().bits(1);
                        w.mosi_delay_num().bits(2)
                    }
                    Mode::_3 => {
                        w.miso_delay_mode().bits(1);
                        w.miso_delay_num().bits(0);
                        w.mosi_delay_mode().bits(0);
                        w.mosi_delay_num().bits(0)
                    }
                }
            });

            if dma {
                assert!(
                    matches!(data_mode, Mode::_1 | Mode::_3),
                    "Mode {:?} is not supported with DMA",
                    data_mode
                );
            }
        }

        #[cfg(not(esp32))]
        {
            _ = dma;
            self.regs().user().modify(|_, w| {
                w.tsck_i_edge()
                    .bit(matches!(data_mode, Mode::_1 | Mode::_2));
                w.rsck_i_edge()
                    .bit(matches!(data_mode, Mode::_1 | Mode::_2))
            });
            cfg_if::cfg_if! {
                if #[cfg(esp32s2)] {
                    let ctrl1_reg = self.regs().ctrl1();
                } else {
                    let ctrl1_reg = self.regs().slave();
                }
            }
            ctrl1_reg.modify(|_, w| {
                w.clk_mode_13()
                    .bit(matches!(data_mode, Mode::_1 | Mode::_3))
            });
        }
    }

    fn is_bus_busy(&self) -> bool {
        #[cfg(pdma)]
        {
            self.regs().slave().read().trans_done().bit_is_clear()
        }
        #[cfg(gdma)]
        {
            self.regs().dma_int_raw().read().trans_done().bit_is_clear()
        }
    }

    // Clear the transaction-done interrupt flag so flush() can work properly. Not
    // used in DMA mode.
    fn setup_for_flush(&self) {
        #[cfg(pdma)]
        self.regs()
            .slave()
            .modify(|_, w| w.trans_done().clear_bit());
        #[cfg(gdma)]
        self.regs()
            .dma_int_clr()
            .write(|w| w.trans_done().clear_bit_by_one());
    }
}

impl PartialEq for Info {
    fn eq(&self, other: &Self) -> bool {
        core::ptr::eq(self.register_block, other.register_block)
    }
}

unsafe impl Sync for Info {}

macro_rules! spi_instance {
    ($num:literal, $sclk:ident, $mosi:ident, $miso:ident, $cs:ident) => {
        paste::paste! {
            impl Instance for crate::peripherals::[<SPI $num>]<'_> {
                #[inline(always)]
                fn info(&self) -> &'static Info {
                    static INFO: Info = Info {
                        register_block: crate::peripherals::[<SPI $num>]::regs(),
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

impl Instance for super::AnySpi<'_> {
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

impl InstanceDma for super::AnySpi<'_> {}
