//! # Serial Peripheral Interface - Slave Mode
//!
//! ## Overview
//!
//! There are multiple ways to use SPI, depending on your needs. Regardless of
//! which way you choose, you must first create an SPI instance with
//! [`Spi::new`].
//!
//! ## Example
//!
//! ```rust
//! let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
//! let sclk = io.pins.gpio12;
//! let miso = io.pins.gpio11;
//! let mosi = io.pins.gpio13;
//! let cs = io.pins.gpio10;
//!
//! let mut spi = hal::spi::slave::Spi::new(peripherals.SPI2, sclk, mosi, miso, cs, SpiMode::Mode0);
//! ```
//!
//! There are several options for working with the SPI peripheral in slave mode,
//! but the code currently only supports single transfers (not segmented
//! transfers), full duplex, single bit (not dual or quad SPI), and DMA mode
//! (not CPU mode). It also does not support blocking operations, as the actual
//! transfer is controlled by the SPI master; if these are necessary,
//! then the DmaTransfer trait instance can be wait()ed on or polled for
//! is_done().
//!
//! ```rust
//! let dma = Gdma::new(peripherals.DMA);
//! const N: usize = (buffer_size + 4091) / 4092;
//! let mut tx_descriptors = [0u32; N * 3];
//! let mut rx_descriptors = [0u32; N * 3];
//! let mut spi = spi.with_dma(dma.channel0.configure(
//!     /* circular = */ false,
//!     tx_descriptors,
//!     rx_descriptors,
//!     DmaPriority::Priority0,
//! ));
//! // This is not legal rust, but any method of getting a &mut 'static is good.
//! let tx_buf = &'static [0u8; N * 4092];
//! let rx_buf = &mut 'static [0u8; N * 4092];
//! let transfer = spi.dma_transfer(tx_buf, rx_buf).unwrap();
//! // Do other operations, checking transfer.is_done()
//! // When the master sends enough clock pulses, is_done() will be true.
//! (tx_buf, rx_buf, spi) = transfer.wait();
//! ```

use core::marker::PhantomData;

use super::{Error, FullDuplexMode, SpiMode};
use crate::{
    dma::{DmaPeripheral, Rx, Tx},
    gpio::{InputPin, InputSignal, OutputPin, OutputSignal},
    peripheral::{Peripheral, PeripheralRef},
    peripherals::spi2::RegisterBlock,
    private,
    system::PeripheralClockControl,
};

/// Prelude for the SPI (Slave) driver
pub mod prelude {
    #[cfg(spi3)]
    pub use super::dma::WithDmaSpi3 as _esp_hal_spi_slave_dma_WithDmaSpi3;
    pub use super::{
        dma::WithDmaSpi2 as _esp_hal_spi_slave_dma_WithDmaSpi2,
        Instance as _esp_hal_spi_slave_Instance,
        InstanceDma as _esp_hal_spi_slave_InstanceDma,
    };
}

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
    pub fn new<SCK: InputPin, MOSI: InputPin, MISO: OutputPin, CS: InputPin>(
        spi: impl Peripheral<P = T> + 'd,
        sck: impl Peripheral<P = SCK> + 'd,
        mosi: impl Peripheral<P = MOSI> + 'd,
        miso: impl Peripheral<P = MISO> + 'd,
        cs: impl Peripheral<P = CS> + 'd,
        mode: SpiMode,
    ) -> Spi<'d, T, FullDuplexMode> {
        crate::into_ref!(spi, sck, mosi, miso, cs);
        sck.set_to_input(private::Internal);
        sck.connect_input_to_peripheral(spi.sclk_signal(), private::Internal);

        mosi.set_to_input(private::Internal);
        mosi.connect_input_to_peripheral(spi.mosi_signal(), private::Internal);

        miso.set_to_push_pull_output(private::Internal);
        miso.connect_peripheral_to_output(spi.miso_signal(), private::Internal);

        cs.set_to_input(private::Internal);
        cs.connect_input_to_peripheral(spi.cs_signal(), private::Internal);

        Self::new_internal(spi, mode)
    }

    pub(crate) fn new_internal(
        spi: PeripheralRef<'d, T>,
        mode: SpiMode,
    ) -> Spi<'d, T, FullDuplexMode> {
        spi.enable_peripheral();

        let mut spi = Spi {
            spi,
            data_mode: mode,
            _mode: PhantomData,
        };
        spi.spi.init();
        spi.spi.set_data_mode(mode);

        spi
    }
}

pub mod dma {
    use embedded_dma::{ReadBuffer, WriteBuffer};

    use super::*;
    #[cfg(spi3)]
    use crate::dma::Spi3Peripheral;
    use crate::{
        dma::{
            dma_private::{DmaSupport, DmaSupportRx, DmaSupportTx},
            Channel,
            ChannelTypes,
            DmaTransferRx,
            DmaTransferTx,
            DmaTransferTxRx,
            RxPrivate,
            Spi2Peripheral,
            SpiPeripheral,
            TxPrivate,
        },
        Mode,
    };

    pub trait WithDmaSpi2<'d, C, DmaMode>
    where
        C: ChannelTypes,
        C::P: SpiPeripheral,
        DmaMode: Mode,
    {
        fn with_dma(
            self,
            channel: Channel<'d, C, DmaMode>,
        ) -> SpiDma<'d, crate::peripherals::SPI2, C, DmaMode>;
    }

    #[cfg(spi3)]
    pub trait WithDmaSpi3<'d, C, DmaMode>
    where
        C: ChannelTypes,
        C::P: SpiPeripheral,
        DmaMode: Mode,
    {
        fn with_dma(
            self,
            channel: Channel<'d, C, DmaMode>,
        ) -> SpiDma<'d, crate::peripherals::SPI3, C, DmaMode>;
    }

    impl<'d, C, DmaMode> WithDmaSpi2<'d, C, DmaMode>
        for Spi<'d, crate::peripherals::SPI2, FullDuplexMode>
    where
        C: ChannelTypes,
        C::P: SpiPeripheral + Spi2Peripheral,
        DmaMode: Mode,
    {
        fn with_dma(
            self,
            mut channel: Channel<'d, C, DmaMode>,
        ) -> SpiDma<'d, crate::peripherals::SPI2, C, DmaMode> {
            channel.tx.init_channel(); // no need to call this for both, TX and RX

            #[cfg(esp32)]
            match self.data_mode {
                SpiMode::Mode0 | SpiMode::Mode2 => {
                    self.spi.invert_i_edge();
                }
                _ => {}
            }

            SpiDma {
                spi: self.spi,
                channel,
            }
        }
    }

    #[cfg(spi3)]
    impl<'d, C, DmaMode> WithDmaSpi3<'d, C, DmaMode>
        for Spi<'d, crate::peripherals::SPI3, FullDuplexMode>
    where
        C: ChannelTypes,
        C::P: SpiPeripheral + Spi3Peripheral,
        DmaMode: Mode,
    {
        fn with_dma(
            self,
            mut channel: Channel<'d, C, DmaMode>,
        ) -> SpiDma<'d, crate::peripherals::SPI3, C, DmaMode> {
            channel.tx.init_channel(); // no need to call this for both, TX and RX

            #[cfg(esp32)]
            match self.data_mode {
                SpiMode::Mode0 | SpiMode::Mode2 => {
                    self.spi.invert_i_edge();
                }
                _ => {}
            }

            SpiDma {
                spi: self.spi,
                channel,
            }
        }
    }

    /// A DMA capable SPI instance.
    pub struct SpiDma<'d, T, C, DmaMode>
    where
        C: ChannelTypes,
        C::P: SpiPeripheral,
        DmaMode: Mode,
    {
        pub(crate) spi: PeripheralRef<'d, T>,
        pub(crate) channel: Channel<'d, C, DmaMode>,
    }

    impl<'d, T, C, DmaMode> core::fmt::Debug for SpiDma<'d, T, C, DmaMode>
    where
        C: ChannelTypes,
        C::P: SpiPeripheral,
        DmaMode: Mode,
    {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
            f.debug_struct("SpiDma").finish()
        }
    }

    impl<'d, T, C, DmaMode> DmaSupport for SpiDma<'d, T, C, DmaMode>
    where
        T: InstanceDma<C::Tx<'d>, C::Rx<'d>>,
        C: ChannelTypes,
        C::P: SpiPeripheral,
        DmaMode: Mode,
    {
        fn peripheral_wait_dma(&mut self, is_tx: bool, is_rx: bool) {
            while !((!is_tx || self.channel.tx.is_done())
                && (!is_rx || self.channel.rx.is_done())
                && !self.spi.is_bus_busy())
            {}

            self.spi.flush().ok();
        }

        fn peripheral_dma_stop(&mut self) {
            unreachable!("unsupported")
        }
    }

    impl<'d, T, C, DmaMode> DmaSupportTx for SpiDma<'d, T, C, DmaMode>
    where
        T: InstanceDma<C::Tx<'d>, C::Rx<'d>>,
        C: ChannelTypes,
        C::P: SpiPeripheral,
        DmaMode: Mode,
    {
        type TX = C::Tx<'d>;

        fn tx(&mut self) -> &mut Self::TX {
            &mut self.channel.tx
        }
    }

    impl<'d, T, C, DmaMode> DmaSupportRx for SpiDma<'d, T, C, DmaMode>
    where
        T: InstanceDma<C::Tx<'d>, C::Rx<'d>>,
        C: ChannelTypes,
        C::P: SpiPeripheral,
        DmaMode: Mode,
    {
        type RX = C::Rx<'d>;

        fn rx(&mut self) -> &mut Self::RX {
            &mut self.channel.rx
        }
    }

    impl<'d, T, C, DmaMode> SpiDma<'d, T, C, DmaMode>
    where
        T: InstanceDma<C::Tx<'d>, C::Rx<'d>>,
        C: ChannelTypes,
        C::P: SpiPeripheral,
        DmaMode: Mode,
    {
        /// Register a buffer for a DMA write.
        ///
        /// This will return a [SpiDmaTransferTx] owning the buffer(s) and the
        /// SPI instance. The maximum amount of data to be sent is 32736
        /// bytes.
        ///
        /// The write is driven by the SPI master's sclk signal and cs line.
        pub fn dma_write<'t, TXBUF>(
            &'t mut self,
            words: &'t TXBUF,
        ) -> Result<DmaTransferTx<Self>, Error>
        where
            TXBUF: ReadBuffer<Word = u8>,
        {
            let (ptr, len) = unsafe { words.read_buffer() };

            if len > MAX_DMA_SIZE {
                return Err(Error::MaxDmaTransferSizeExceeded);
            }

            self.spi
                .start_write_bytes_dma(ptr, len, &mut self.channel.tx)
                .map(move |_| DmaTransferTx::new(self))
        }

        /// Register a buffer for a DMA read.
        ///
        /// This will return a [SpiDmaTransferRx] owning the buffer(s) and the
        /// SPI instance. The maximum amount of data to be received is
        /// 32736 bytes.
        ///
        /// The read is driven by the SPI master's sclk signal and cs line.
        pub fn dma_read<'t, RXBUF>(
            &'t mut self,
            words: &'t mut RXBUF,
        ) -> Result<DmaTransferRx<Self>, Error>
        where
            RXBUF: WriteBuffer<Word = u8>,
        {
            let (ptr, len) = unsafe { words.write_buffer() };

            if len > MAX_DMA_SIZE {
                return Err(Error::MaxDmaTransferSizeExceeded);
            }

            unsafe {
                self.spi
                    .start_read_bytes_dma(ptr, len, &mut self.channel.rx)
                    .map(move |_| DmaTransferRx::new(self))
            }
        }

        /// Register buffers for a DMA transfer.
        ///
        /// This will return a [SpiDmaTransferRxTx] owning the buffer(s) and the
        /// SPI instance. The maximum amount of data to be sent/received
        /// is 32736 bytes.
        ///
        /// The data transfer is driven by the SPI master's sclk signal and cs
        /// line.
        pub fn dma_transfer<'t, TXBUF, RXBUF>(
            &'t mut self,
            words: &'t TXBUF,
            read_buffer: &'t mut RXBUF,
        ) -> Result<DmaTransferTxRx<Self>, Error>
        where
            TXBUF: ReadBuffer<Word = u8>,
            RXBUF: WriteBuffer<Word = u8>,
        {
            let (write_ptr, write_len) = unsafe { words.read_buffer() };
            let (read_ptr, read_len) = unsafe { read_buffer.write_buffer() };

            if write_len > MAX_DMA_SIZE || read_len > MAX_DMA_SIZE {
                return Err(Error::MaxDmaTransferSizeExceeded);
            }

            unsafe {
                self.spi
                    .start_transfer_dma(
                        write_ptr,
                        write_len,
                        read_ptr,
                        read_len,
                        &mut self.channel.tx,
                        &mut self.channel.rx,
                    )
                    .map(move |_| DmaTransferTxRx::new(self))
            }
        }
    }
}

#[doc(hidden)]
pub trait InstanceDma<TX, RX>: Instance
where
    TX: Tx,
    RX: Rx,
{
    unsafe fn start_transfer_dma(
        &mut self,
        write_buffer_ptr: *const u8,
        write_buffer_len: usize,
        read_buffer_ptr: *mut u8,
        read_buffer_len: usize,
        tx: &mut TX,
        rx: &mut RX,
    ) -> Result<(), Error> {
        let reg_block = self.register_block();

        tx.is_done();
        rx.is_done();

        self.enable_dma();

        reset_dma_before_load_dma_dscr(reg_block);
        tx.prepare_transfer_without_start(
            self.dma_peripheral(),
            false,
            write_buffer_ptr,
            write_buffer_len,
        )?;
        rx.prepare_transfer_without_start(
            false,
            self.dma_peripheral(),
            read_buffer_ptr,
            read_buffer_len,
        )?;

        self.clear_dma_interrupts();
        reset_dma_before_usr_cmd(reg_block);

        // On the esp32, all full-duplex transfers are single, and all half-duplex
        // transfers use the cmd/addr/dummy/data sequence (but are still
        // single).
        #[cfg(not(esp32))]
        reg_block
            .dma_conf()
            .modify(|_, w| w.dma_slv_seg_trans_en().clear_bit());

        tx.start_transfer()?;
        Ok(rx.start_transfer()?)
    }

    fn start_write_bytes_dma(
        &mut self,
        ptr: *const u8,
        len: usize,
        tx: &mut TX,
    ) -> Result<(), Error> {
        let reg_block = self.register_block();

        tx.is_done();

        self.enable_dma();

        reset_dma_before_load_dma_dscr(reg_block);
        tx.prepare_transfer_without_start(self.dma_peripheral(), false, ptr, len)?;

        self.clear_dma_interrupts();
        reset_dma_before_usr_cmd(reg_block);

        // On the esp32, all full-duplex transfers are single, and all half-duplex
        // transfers use the cmd/addr/dummy/data sequence (but are still
        // single).
        #[cfg(not(esp32))]
        reg_block
            .dma_conf()
            .modify(|_, w| w.dma_slv_seg_trans_en().clear_bit());

        Ok(tx.start_transfer()?)
    }

    unsafe fn start_read_bytes_dma(
        &mut self,
        ptr: *mut u8,
        len: usize,
        rx: &mut RX,
    ) -> Result<(), Error> {
        let reg_block = self.register_block();

        rx.is_done();

        self.enable_dma();

        reset_dma_before_load_dma_dscr(reg_block);
        unsafe {
            rx.prepare_transfer_without_start(false, self.dma_peripheral(), ptr, len)?;
        }

        self.clear_dma_interrupts();
        reset_dma_before_usr_cmd(reg_block);

        // On the esp32, all full-duplex transfers are single, and all half-duplex
        // transfers use the cmd/addr/dummy/data sequence (but are still
        // single).
        #[cfg(not(esp32))]
        reg_block
            .dma_conf()
            .modify(|_, w| w.dma_slv_seg_trans_en().clear_bit());

        Ok(rx.start_transfer()?)
    }

    fn dma_peripheral(&self) -> DmaPeripheral {
        match self.spi_num() {
            2 => DmaPeripheral::Spi2,
            #[cfg(spi3)]
            3 => DmaPeripheral::Spi3,
            _ => panic!("Illegal SPI instance"),
        }
    }

    #[cfg(any(esp32c2, esp32c3, esp32c6, esp32h2, esp32s3))]
    fn enable_dma(&self) {
        let reg_block = self.register_block();
        reg_block.dma_conf().modify(|_, w| {
            w.dma_tx_ena()
                .set_bit()
                .dma_rx_ena()
                .set_bit()
                .rx_eof_en()
                .clear_bit()
        });
    }

    #[cfg(any(esp32, esp32s2))]
    fn enable_dma(&self) {
        // for non GDMA this is done in `assign_tx_device` / `assign_rx_device`
    }

    #[cfg(any(esp32c2, esp32c3, esp32c6, esp32h2, esp32s3))]
    fn clear_dma_interrupts(&self) {
        let reg_block = self.register_block();
        reg_block.dma_int_clr().write(|w| {
            w.dma_infifo_full_err()
                .clear_bit_by_one()
                .dma_outfifo_empty_err()
                .clear_bit_by_one()
                .trans_done()
                .clear_bit_by_one()
                .mst_rx_afifo_wfull_err()
                .clear_bit_by_one()
                .mst_tx_afifo_rempty_err()
                .clear_bit_by_one()
        });
    }

    #[cfg(any(esp32, esp32s2))]
    fn clear_dma_interrupts(&self) {
        let reg_block = self.register_block();
        reg_block.dma_int_clr.write(|w| {
            w.inlink_dscr_empty()
                .clear_bit_by_one()
                .outlink_dscr_error()
                .clear_bit_by_one()
                .inlink_dscr_error()
                .clear_bit_by_one()
                .in_done()
                .clear_bit_by_one()
                .in_err_eof()
                .clear_bit_by_one()
                .in_suc_eof()
                .clear_bit_by_one()
                .out_done()
                .clear_bit_by_one()
                .out_eof()
                .clear_bit_by_one()
                .out_total_eof()
                .clear_bit_by_one()
        });
    }
}

#[cfg(not(any(esp32, esp32s2)))]
fn reset_dma_before_usr_cmd(reg_block: &RegisterBlock) {
    reg_block.dma_conf().modify(|_, w| {
        w.rx_afifo_rst()
            .set_bit()
            .buf_afifo_rst()
            .set_bit()
            .dma_afifo_rst()
            .set_bit()
    });
}

#[cfg(any(esp32, esp32s2))]
fn reset_dma_before_usr_cmd(_reg_block: &RegisterBlock) {}

#[cfg(not(any(esp32, esp32s2)))]
fn reset_dma_before_load_dma_dscr(_reg_block: &RegisterBlock) {}

#[cfg(any(esp32, esp32s2))]
fn reset_dma_before_load_dma_dscr(reg_block: &RegisterBlock) {
    reg_block.dma_conf.modify(|_, w| {
        w.out_rst()
            .set_bit()
            .in_rst()
            .set_bit()
            .ahbm_fifo_rst()
            .set_bit()
            .ahbm_rst()
            .set_bit()
    });
}

impl<TX, RX> InstanceDma<TX, RX> for crate::peripherals::SPI2
where
    TX: Tx,
    RX: Rx,
{
}

#[cfg(spi3)]
impl<TX, RX> InstanceDma<TX, RX> for crate::peripherals::SPI3
where
    TX: Tx,
    RX: Rx,
{
}

#[doc(hidden)]
pub trait Instance: private::Sealed {
    fn register_block(&self) -> &RegisterBlock;

    fn sclk_signal(&self) -> InputSignal;

    fn mosi_signal(&self) -> InputSignal;

    fn miso_signal(&self) -> OutputSignal;

    fn cs_signal(&self) -> InputSignal;

    fn enable_peripheral(&self);

    fn spi_num(&self) -> u8;

    /// Initialize for full-duplex 1 bit mode
    fn init(&mut self) {
        let reg_block = self.register_block();
        reg_block.slave().write(|w| w.mode().set_bit());

        reg_block.user().modify(|_, w| {
            w.usr_miso_highpart()
                .clear_bit()
                .doutdin()
                .set_bit()
                .usr_miso()
                .set_bit()
                .usr_mosi()
                .set_bit()
                .usr_dummy_idle()
                .clear_bit()
                .usr_addr()
                .clear_bit()
                .usr_command()
                .clear_bit()
        });

        #[cfg(not(any(esp32, esp32s2)))]
        reg_block.clk_gate().modify(|_, w| {
            w.clk_en()
                .clear_bit()
                .mst_clk_active()
                .clear_bit()
                .mst_clk_sel()
                .clear_bit()
        });

        #[cfg(not(any(esp32, esp32s2)))]
        reg_block.ctrl().modify(|_, w| {
            w.q_pol()
                .clear_bit()
                .d_pol()
                .clear_bit()
                .hold_pol()
                .clear_bit()
        });

        #[cfg(esp32s2)]
        reg_block
            .ctrl()
            .modify(|_, w| w.q_pol().clear_bit().d_pol().clear_bit().wp().clear_bit());

        #[cfg(esp32)]
        reg_block.ctrl().modify(|_, w| w.wp().clear_bit());

        #[cfg(not(esp32))]
        reg_block.misc().write(|w| unsafe { w.bits(0) });
    }

    #[cfg(not(esp32))]
    fn set_data_mode(&mut self, data_mode: SpiMode) -> &mut Self {
        let reg_block = self.register_block();

        match data_mode {
            SpiMode::Mode0 => {
                reg_block
                    .user()
                    .modify(|_, w| w.tsck_i_edge().clear_bit().rsck_i_edge().clear_bit());
                #[cfg(esp32s2)]
                reg_block.ctrl1.modify(|_, w| w.clk_mode_13().clear_bit());
                #[cfg(not(esp32s2))]
                reg_block.slave().modify(|_, w| w.clk_mode_13().clear_bit());
            }
            SpiMode::Mode1 => {
                reg_block
                    .user()
                    .modify(|_, w| w.tsck_i_edge().set_bit().rsck_i_edge().set_bit());
                #[cfg(esp32s2)]
                reg_block.ctrl1().modify(|_, w| w.clk_mode_13().set_bit());
                #[cfg(not(esp32s2))]
                reg_block.slave().modify(|_, w| w.clk_mode_13().set_bit());
            }
            SpiMode::Mode2 => {
                reg_block
                    .user()
                    .modify(|_, w| w.tsck_i_edge().set_bit().rsck_i_edge().set_bit());
                #[cfg(esp32s2)]
                reg_block.ctrl1().modify(|_, w| w.clk_mode_13().clear_bit());
                #[cfg(not(esp32s2))]
                reg_block.slave().modify(|_, w| w.clk_mode_13().clear_bit());
            }
            SpiMode::Mode3 => {
                reg_block
                    .user()
                    .modify(|_, w| w.tsck_i_edge().clear_bit().rsck_i_edge().clear_bit());
                #[cfg(esp32s2)]
                reg_block.ctrl1().modify(|_, w| w.clk_mode_13().set_bit());
                #[cfg(not(esp32s2))]
                reg_block.slave().modify(|_, w| w.clk_mode_13().set_bit());
            }
        }
        self
    }

    #[cfg(esp32)]
    fn set_data_mode(&mut self, data_mode: SpiMode) -> &mut Self {
        let reg_block = self.register_block();

        match data_mode {
            SpiMode::Mode0 => {
                reg_block.pin.modify(|_, w| w.ck_idle_edge().set_bit());
                reg_block.user.modify(|_, w| w.ck_i_edge().clear_bit());
            }
            SpiMode::Mode1 => {
                reg_block.pin.modify(|_, w| w.ck_idle_edge().set_bit());
                reg_block.user.modify(|_, w| w.ck_i_edge().set_bit());
            }
            SpiMode::Mode2 => {
                reg_block.pin.modify(|_, w| w.ck_idle_edge().clear_bit());
                reg_block.user.modify(|_, w| w.ck_i_edge().set_bit());
            }
            SpiMode::Mode3 => {
                reg_block.pin.modify(|_, w| w.ck_idle_edge().clear_bit());
                reg_block.user.modify(|_, w| w.ck_i_edge().clear_bit());
            }
        }
        self
    }

    // The ESP32 needs its _edge bits inverted in DMA slave mode, when in mode 0 or
    // 2. set_data_mode above sets the registers up for non-DMA mode.
    #[cfg(esp32)]
    fn invert_i_edge(&self) {
        let reg_block = self.register_block();

        reg_block
            .pin
            .modify(|r, w| w.ck_idle_edge().variant(r.ck_idle_edge().bit_is_clear()));
        reg_block
            .user
            .modify(|r, w| w.ck_i_edge().variant(r.ck_i_edge().bit_is_clear()));
    }

    fn is_bus_busy(&self) -> bool {
        let reg_block = self.register_block();

        #[cfg(any(esp32, esp32s2))]
        {
            reg_block.slave.read().trans_done().bit_is_clear()
        }
        #[cfg(not(any(esp32, esp32s2)))]
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
        #[cfg(any(esp32, esp32s2))]
        self.register_block()
            .slave()
            .modify(|_, w| w.trans_done().clear_bit());
        #[cfg(not(any(esp32, esp32s2)))]
        self.register_block()
            .dma_int_clr()
            .write(|w| w.trans_done().clear_bit_by_one());
    }
}

#[cfg(any(esp32c2, esp32c3, esp32c6, esp32h2))]
impl Instance for crate::peripherals::SPI2 {
    #[inline(always)]
    fn register_block(&self) -> &RegisterBlock {
        self
    }

    #[inline(always)]
    fn sclk_signal(&self) -> InputSignal {
        InputSignal::FSPICLK
    }

    #[inline(always)]
    fn mosi_signal(&self) -> InputSignal {
        InputSignal::FSPID
    }

    #[inline(always)]
    fn miso_signal(&self) -> OutputSignal {
        OutputSignal::FSPIQ
    }

    #[inline(always)]
    fn cs_signal(&self) -> InputSignal {
        InputSignal::FSPICS0
    }

    #[inline(always)]
    fn enable_peripheral(&self) {
        PeripheralClockControl::enable(crate::system::Peripheral::Spi2);
    }

    #[inline(always)]
    fn spi_num(&self) -> u8 {
        2
    }
}

#[cfg(esp32)]
impl Instance for crate::peripherals::SPI2 {
    #[inline(always)]
    fn register_block(&self) -> &RegisterBlock {
        self
    }

    #[inline(always)]
    fn sclk_signal(&self) -> InputSignal {
        InputSignal::HSPICLK
    }

    #[inline(always)]
    fn mosi_signal(&self) -> InputSignal {
        InputSignal::HSPID
    }

    #[inline(always)]
    fn miso_signal(&self) -> OutputSignal {
        OutputSignal::HSPIQ
    }

    #[inline(always)]
    fn cs_signal(&self) -> InputSignal {
        InputSignal::HSPICS0
    }

    #[inline(always)]
    fn enable_peripheral(&self) {
        PeripheralClockControl::enable(crate::system::Peripheral::Spi2);
    }

    #[inline(always)]
    fn spi_num(&self) -> u8 {
        2
    }
}

#[cfg(esp32)]
impl Instance for crate::peripherals::SPI3 {
    #[inline(always)]
    fn register_block(&self) -> &RegisterBlock {
        self
    }

    #[inline(always)]
    fn sclk_signal(&self) -> InputSignal {
        InputSignal::VSPICLK
    }

    #[inline(always)]
    fn mosi_signal(&self) -> InputSignal {
        InputSignal::VSPID
    }

    #[inline(always)]
    fn miso_signal(&self) -> OutputSignal {
        OutputSignal::VSPIQ
    }

    #[inline(always)]
    fn cs_signal(&self) -> InputSignal {
        InputSignal::VSPICS0
    }

    #[inline(always)]
    fn enable_peripheral(&self) {
        PeripheralClockControl::enable(crate::system::Peripheral::Spi3)
    }

    #[inline(always)]
    fn spi_num(&self) -> u8 {
        3
    }
}

#[cfg(any(esp32s2, esp32s3))]
impl Instance for crate::peripherals::SPI2 {
    #[inline(always)]
    fn register_block(&self) -> &RegisterBlock {
        self
    }

    #[inline(always)]
    fn sclk_signal(&self) -> InputSignal {
        InputSignal::FSPICLK
    }

    #[inline(always)]
    fn mosi_signal(&self) -> InputSignal {
        InputSignal::FSPID
    }

    #[inline(always)]
    fn miso_signal(&self) -> OutputSignal {
        OutputSignal::FSPIQ
    }

    #[inline(always)]
    fn cs_signal(&self) -> InputSignal {
        InputSignal::FSPICS0
    }

    #[inline(always)]
    fn enable_peripheral(&self) {
        PeripheralClockControl::enable(crate::system::Peripheral::Spi2)
    }

    #[inline(always)]
    fn spi_num(&self) -> u8 {
        2
    }
}

#[cfg(any(esp32s2, esp32s3))]
impl Instance for crate::peripherals::SPI3 {
    #[inline(always)]
    fn register_block(&self) -> &RegisterBlock {
        self
    }

    #[inline(always)]
    fn sclk_signal(&self) -> InputSignal {
        InputSignal::SPI3_CLK
    }

    #[inline(always)]
    fn mosi_signal(&self) -> InputSignal {
        InputSignal::SPI3_D
    }

    #[inline(always)]
    fn miso_signal(&self) -> OutputSignal {
        OutputSignal::SPI3_Q
    }

    #[inline(always)]
    fn cs_signal(&self) -> InputSignal {
        InputSignal::SPI3_CS0
    }

    #[inline(always)]
    fn enable_peripheral(&self) {
        PeripheralClockControl::enable(crate::system::Peripheral::Spi3)
    }

    #[inline(always)]
    fn spi_num(&self) -> u8 {
        3
    }
}
