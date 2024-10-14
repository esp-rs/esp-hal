//! # Parallel Interface
//!
//! ## Overview
//!
//! ## Notes
//!
//! - In 16 bit mode the buffer needs to be interleaved and is expected to have
//!   a length thats a multiple of 32 bits
//!
//! ## Configuration
//!
//! The driver uses DMA (Direct Memory Access) for efficient data transfer and
//! supports various configurations, such as different data formats, standards
//! (e.g., Philips) and pin configurations. It relies on other peripheral
//! modules, such as
//!   - `GPIO`
//!   - `DMA`
//!   - `system` (to configure and enable the I2S peripheral)
use core::{
    marker::PhantomData,
    mem::ManuallyDrop,
    ops::{Deref, DerefMut},
};

use fugit::HertzU32;
use private::TxPins;

use crate::{
    dma::{
        asynch::DmaTxFuture,
        Channel,
        ChannelTx,
        DmaChannelConvert,
        DmaEligible,
        DmaError,
        DmaTxBuffer,
        PeripheralMarker,
        Tx,
    },
    gpio::{OutputSignal, PeripheralOutput},
    peripheral::{Peripheral, PeripheralRef},
    peripherals::{i2s0::RegisterBlock, I2S0, I2S1},
    system::PeripheralClockControl,
    Mode,
};

/// Represents a group of 16 output pins configured for 16-bit parallel data
/// transmission.
pub struct TxSixteenBits<'d, P0, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11, P12, P13, P14, P15> {
    pin_0: PeripheralRef<'d, P0>,
    pin_1: PeripheralRef<'d, P1>,
    pin_2: PeripheralRef<'d, P2>,
    pin_3: PeripheralRef<'d, P3>,
    pin_4: PeripheralRef<'d, P4>,
    pin_5: PeripheralRef<'d, P5>,
    pin_6: PeripheralRef<'d, P6>,
    pin_7: PeripheralRef<'d, P7>,
    pin_8: PeripheralRef<'d, P8>,
    pin_9: PeripheralRef<'d, P9>,
    pin_10: PeripheralRef<'d, P10>,
    pin_11: PeripheralRef<'d, P11>,
    pin_12: PeripheralRef<'d, P12>,
    pin_13: PeripheralRef<'d, P13>,
    pin_14: PeripheralRef<'d, P14>,
    pin_15: PeripheralRef<'d, P15>,
}

impl<'d, P0, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11, P12, P13, P14, P15>
    TxSixteenBits<'d, P0, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11, P12, P13, P14, P15>
where
    P0: PeripheralOutput,
    P1: PeripheralOutput,
    P2: PeripheralOutput,
    P3: PeripheralOutput,
    P4: PeripheralOutput,
    P5: PeripheralOutput,
    P6: PeripheralOutput,
    P7: PeripheralOutput,
    P8: PeripheralOutput,
    P9: PeripheralOutput,
    P10: PeripheralOutput,
    P11: PeripheralOutput,
    P12: PeripheralOutput,
    P13: PeripheralOutput,
    P14: PeripheralOutput,
    P15: PeripheralOutput,
{
    #[allow(clippy::too_many_arguments)]
    /// Creates a new `TxSixteenBits` instance with the provided output pins.
    pub fn new(
        pin_0: impl Peripheral<P = P0> + 'd,
        pin_1: impl Peripheral<P = P1> + 'd,
        pin_2: impl Peripheral<P = P2> + 'd,
        pin_3: impl Peripheral<P = P3> + 'd,
        pin_4: impl Peripheral<P = P4> + 'd,
        pin_5: impl Peripheral<P = P5> + 'd,
        pin_6: impl Peripheral<P = P6> + 'd,
        pin_7: impl Peripheral<P = P7> + 'd,
        pin_8: impl Peripheral<P = P8> + 'd,
        pin_9: impl Peripheral<P = P9> + 'd,
        pin_10: impl Peripheral<P = P10> + 'd,
        pin_11: impl Peripheral<P = P11> + 'd,
        pin_12: impl Peripheral<P = P12> + 'd,
        pin_13: impl Peripheral<P = P13> + 'd,
        pin_14: impl Peripheral<P = P14> + 'd,
        pin_15: impl Peripheral<P = P15> + 'd,
    ) -> Self {
        crate::into_ref!(pin_0);
        crate::into_ref!(pin_1);
        crate::into_ref!(pin_2);
        crate::into_ref!(pin_3);
        crate::into_ref!(pin_4);
        crate::into_ref!(pin_5);
        crate::into_ref!(pin_6);
        crate::into_ref!(pin_7);
        crate::into_ref!(pin_8);
        crate::into_ref!(pin_9);
        crate::into_ref!(pin_10);
        crate::into_ref!(pin_11);
        crate::into_ref!(pin_12);
        crate::into_ref!(pin_13);
        crate::into_ref!(pin_14);
        crate::into_ref!(pin_15);

        Self {
            pin_0,
            pin_1,
            pin_2,
            pin_3,
            pin_4,
            pin_5,
            pin_6,
            pin_7,
            pin_8,
            pin_9,
            pin_10,
            pin_11,
            pin_12,
            pin_13,
            pin_14,
            pin_15,
        }
    }
}

impl<'d, P0, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11, P12, P13, P14, P15> TxPins
    for TxSixteenBits<'d, P0, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11, P12, P13, P14, P15>
where
    P0: PeripheralOutput,
    P1: PeripheralOutput,
    P2: PeripheralOutput,
    P3: PeripheralOutput,
    P4: PeripheralOutput,
    P5: PeripheralOutput,
    P6: PeripheralOutput,
    P7: PeripheralOutput,
    P8: PeripheralOutput,
    P9: PeripheralOutput,
    P10: PeripheralOutput,
    P11: PeripheralOutput,
    P12: PeripheralOutput,
    P13: PeripheralOutput,
    P14: PeripheralOutput,
    P15: PeripheralOutput,
{
    fn bits(&self) -> u8 {
        16
    }

    fn configure<I: Instance>(&mut self, _instance: &PeripheralRef<'_, I>) {
        let bits: u8 = self.bits();
        self.pin_0.set_to_push_pull_output(crate::private::Internal);
        self.pin_0
            .connect_peripheral_to_output(I::data_out_signal(0, bits), crate::private::Internal);
        self.pin_1.set_to_push_pull_output(crate::private::Internal);
        self.pin_1
            .connect_peripheral_to_output(I::data_out_signal(1, bits), crate::private::Internal);
        self.pin_2.set_to_push_pull_output(crate::private::Internal);
        self.pin_2
            .connect_peripheral_to_output(I::data_out_signal(2, bits), crate::private::Internal);
        self.pin_3.set_to_push_pull_output(crate::private::Internal);
        self.pin_3
            .connect_peripheral_to_output(I::data_out_signal(3, bits), crate::private::Internal);
        self.pin_4.set_to_push_pull_output(crate::private::Internal);
        self.pin_4
            .connect_peripheral_to_output(I::data_out_signal(4, bits), crate::private::Internal);
        self.pin_5.set_to_push_pull_output(crate::private::Internal);
        self.pin_5
            .connect_peripheral_to_output(I::data_out_signal(5, bits), crate::private::Internal);
        self.pin_6.set_to_push_pull_output(crate::private::Internal);
        self.pin_6
            .connect_peripheral_to_output(I::data_out_signal(6, bits), crate::private::Internal);
        self.pin_7.set_to_push_pull_output(crate::private::Internal);
        self.pin_7
            .connect_peripheral_to_output(I::data_out_signal(7, bits), crate::private::Internal);
        self.pin_8.set_to_push_pull_output(crate::private::Internal);
        self.pin_8
            .connect_peripheral_to_output(I::data_out_signal(8, bits), crate::private::Internal);
        self.pin_9.set_to_push_pull_output(crate::private::Internal);
        self.pin_9
            .connect_peripheral_to_output(I::data_out_signal(9, bits), crate::private::Internal);
        self.pin_10
            .set_to_push_pull_output(crate::private::Internal);
        self.pin_10
            .connect_peripheral_to_output(I::data_out_signal(10, bits), crate::private::Internal);
        self.pin_11
            .set_to_push_pull_output(crate::private::Internal);
        self.pin_11
            .connect_peripheral_to_output(I::data_out_signal(11, bits), crate::private::Internal);
        self.pin_12
            .set_to_push_pull_output(crate::private::Internal);
        self.pin_12
            .connect_peripheral_to_output(I::data_out_signal(12, bits), crate::private::Internal);
        self.pin_13
            .set_to_push_pull_output(crate::private::Internal);
        self.pin_13
            .connect_peripheral_to_output(I::data_out_signal(13, bits), crate::private::Internal);
        self.pin_14
            .set_to_push_pull_output(crate::private::Internal);
        self.pin_14
            .connect_peripheral_to_output(I::data_out_signal(14, bits), crate::private::Internal);
        self.pin_15
            .set_to_push_pull_output(crate::private::Internal);
        self.pin_15
            .connect_peripheral_to_output(I::data_out_signal(15, bits), crate::private::Internal);
    }
}

/// Represents a group of 8 output pins configured for 8-bit parallel data
/// transmission.
pub struct TxEightBits<'d, P0, P1, P2, P3, P4, P5, P6, P7> {
    pin_0: PeripheralRef<'d, P0>,
    pin_1: PeripheralRef<'d, P1>,
    pin_2: PeripheralRef<'d, P2>,
    pin_3: PeripheralRef<'d, P3>,
    pin_4: PeripheralRef<'d, P4>,
    pin_5: PeripheralRef<'d, P5>,
    pin_6: PeripheralRef<'d, P6>,
    pin_7: PeripheralRef<'d, P7>,
}

impl<'d, P0, P1, P2, P3, P4, P5, P6, P7> TxEightBits<'d, P0, P1, P2, P3, P4, P5, P6, P7>
where
    P0: PeripheralOutput,
    P1: PeripheralOutput,
    P2: PeripheralOutput,
    P3: PeripheralOutput,
    P4: PeripheralOutput,
    P5: PeripheralOutput,
    P6: PeripheralOutput,
    P7: PeripheralOutput,
{
    #[allow(clippy::too_many_arguments)]
    /// Creates a new `TxSixteenBits` instance with the provided output pins.
    pub fn new(
        pin_0: impl Peripheral<P = P0> + 'd,
        pin_1: impl Peripheral<P = P1> + 'd,
        pin_2: impl Peripheral<P = P2> + 'd,
        pin_3: impl Peripheral<P = P3> + 'd,
        pin_4: impl Peripheral<P = P4> + 'd,
        pin_5: impl Peripheral<P = P5> + 'd,
        pin_6: impl Peripheral<P = P6> + 'd,
        pin_7: impl Peripheral<P = P7> + 'd,
    ) -> Self {
        crate::into_ref!(pin_0);
        crate::into_ref!(pin_1);
        crate::into_ref!(pin_2);
        crate::into_ref!(pin_3);
        crate::into_ref!(pin_4);
        crate::into_ref!(pin_5);
        crate::into_ref!(pin_6);
        crate::into_ref!(pin_7);

        Self {
            pin_0,
            pin_1,
            pin_2,
            pin_3,
            pin_4,
            pin_5,
            pin_6,
            pin_7,
        }
    }
}

impl<'d, P0, P1, P2, P3, P4, P5, P6, P7> TxPins for TxEightBits<'d, P0, P1, P2, P3, P4, P5, P6, P7>
where
    P0: PeripheralOutput,
    P1: PeripheralOutput,
    P2: PeripheralOutput,
    P3: PeripheralOutput,
    P4: PeripheralOutput,
    P5: PeripheralOutput,
    P6: PeripheralOutput,
    P7: PeripheralOutput,
{
    fn bits(&self) -> u8 {
        8
    }

    fn configure<I: Instance>(&mut self, _instance: &PeripheralRef<'_, I>) {
        let bits: u8 = self.bits();
        self.pin_0.set_to_push_pull_output(crate::private::Internal);
        self.pin_0
            .connect_peripheral_to_output(I::data_out_signal(0, bits), crate::private::Internal);
        self.pin_1.set_to_push_pull_output(crate::private::Internal);
        self.pin_1
            .connect_peripheral_to_output(I::data_out_signal(1, bits), crate::private::Internal);
        self.pin_2.set_to_push_pull_output(crate::private::Internal);
        self.pin_2
            .connect_peripheral_to_output(I::data_out_signal(2, bits), crate::private::Internal);
        self.pin_3.set_to_push_pull_output(crate::private::Internal);
        self.pin_3
            .connect_peripheral_to_output(I::data_out_signal(3, bits), crate::private::Internal);
        self.pin_4.set_to_push_pull_output(crate::private::Internal);
        self.pin_4
            .connect_peripheral_to_output(I::data_out_signal(4, bits), crate::private::Internal);
        self.pin_5.set_to_push_pull_output(crate::private::Internal);
        self.pin_5
            .connect_peripheral_to_output(I::data_out_signal(5, bits), crate::private::Internal);
        self.pin_6.set_to_push_pull_output(crate::private::Internal);
        self.pin_6
            .connect_peripheral_to_output(I::data_out_signal(6, bits), crate::private::Internal);
        self.pin_7.set_to_push_pull_output(crate::private::Internal);
        self.pin_7
            .connect_peripheral_to_output(I::data_out_signal(7, bits), crate::private::Internal);
    }
}

/// I2S Parallel Interface
pub struct I2sParallel<'d, I: Instance, DM: Mode> {
    tx_channel: ChannelTx<'d, <I as DmaEligible>::Dma>,
    instance: PhantomData<I>,
    mode: PhantomData<DM>,
}

impl<'d, I: Instance, DM: Mode> I2sParallel<'d, I, DM> {
    /// Create a new I2S Parallel Interface
    pub fn new<P, CH, CLK>(
        i2s: impl Peripheral<P = I>,
        channel: Channel<'d, CH, DM>,
        frequency: impl Into<fugit::HertzU32>,
        mut pins: P,
        mut clock_pin: CLK,
    ) -> Self
    where
        CH: DmaChannelConvert<I::Dma>,
        P: TxPins,
        CLK: PeripheralOutput,
    {
        crate::into_ref!(i2s);
        channel.runtime_ensure_compatible(&i2s);
        let channel = channel.degrade();

        PeripheralClockControl::reset(I::get_peripheral());
        PeripheralClockControl::enable(I::get_peripheral());
        // configure the I2S peripheral for parallel mode
        I::setup(frequency, pins.bits());
        // setup the clock pin
        clock_pin.set_to_push_pull_output(crate::private::Internal);
        clock_pin.connect_peripheral_to_output(I::ws_signal(), crate::private::Internal);

        pins.configure(&i2s);
        Self {
            tx_channel: channel.tx,
            instance: PhantomData,
            mode: PhantomData,
        }
    }

    /// Write data to the I2S peripheral
    pub fn send<BUF: DmaTxBuffer>(
        mut self,
        mut data: BUF,
    ) -> Result<I2sParallelTransfer<'d, I, BUF, DM>, (DmaError, Self, BUF)> {
        I::tx_reset();
        I::tx_fifo_reset();
        I::tx_dma_reset();
        let result = unsafe {
            self.tx_channel
                .prepare_transfer(I::get_dma_peripheral(), &mut data)
        }
        .and_then(|_| self.tx_channel.start_transfer());
        if let Err(err) = result {
            return Err((err, self, data));
        }
        I::tx_clear_interrupts();
        I::tx_start();
        Ok(I2sParallelTransfer {
            i2s: ManuallyDrop::new(self),
            buf_view: ManuallyDrop::new(data.into_view()),
        })
    }

    #[cfg(all(feature = "debug", feature = "log"))]
    /// Dump the I2S peripheral configuration
    pub fn dump(&self) {
        I::dump();
    }
}

/// Represents an ongoing (or potentially finished) transfer using the i2s
/// parallel interface
pub struct I2sParallelTransfer<'d, I, BUF, DM>
where
    I: Instance,
    BUF: DmaTxBuffer,
    DM: Mode,
{
    i2s: ManuallyDrop<I2sParallel<'d, I, DM>>,
    buf_view: ManuallyDrop<BUF::View>,
}

impl<'d, I, BUF, DM> I2sParallelTransfer<'d, I, BUF, DM>
where
    I: Instance,
    BUF: DmaTxBuffer,
    DM: Mode,
{
    /// Returns true when [Self::wait] will not block.
    pub fn is_done(&self) -> bool {
        I::is_tx_done()
    }

    /// Wait for the transfer to finish
    pub fn wait(mut self) -> (I2sParallel<'d, I, DM>, BUF) {
        I::tx_wait_done();
        let i2s = unsafe { ManuallyDrop::take(&mut self.i2s) };
        let view = unsafe { ManuallyDrop::take(&mut self.buf_view) };
        (i2s, BUF::from_view(view))
    }

    #[cfg(all(feature = "debug", feature = "log"))]
    /// Dump the I2S peripheral configuration
    pub fn dump(&self) {
        I::dump();
    }
}

impl<'d, I, BUF> I2sParallelTransfer<'d, I, BUF, crate::Async>
where
    I: Instance,
    BUF: DmaTxBuffer,
{
    /// Wait for the transfer to finish
    pub async fn wait_for_done(&mut self) -> Result<(), DmaError> {
        let future = DmaTxFuture::new(&mut self.i2s.tx_channel);
        future.await
    }
}

impl<'d, I, BUF, DM> Deref for I2sParallelTransfer<'d, I, BUF, DM>
where
    I: Instance,
    BUF: DmaTxBuffer,
    DM: Mode,
{
    type Target = BUF::View;

    fn deref(&self) -> &Self::Target {
        &self.buf_view
    }
}

impl<'d, I, BUF, DM> DerefMut for I2sParallelTransfer<'d, I, BUF, DM>
where
    I: Instance,
    BUF: DmaTxBuffer,
    DM: Mode,
{
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.buf_view
    }
}

mod private {
    use crate::peripheral::PeripheralRef;

    pub trait TxPins {
        fn bits(&self) -> u8;
        fn configure<I: super::Instance>(&mut self, instance: &PeripheralRef<'_, I>);
    }
}

#[allow(missing_docs)]
pub trait RegBlock: PeripheralMarker + DmaEligible {
    fn register_block() -> &'static RegisterBlock;
}

#[allow(missing_docs)]
pub trait Signals {
    fn get_peripheral() -> crate::system::Peripheral;
    fn get_dma_peripheral() -> crate::dma::DmaPeripheral;
    fn ws_signal() -> OutputSignal;
    fn data_out_signal(i: usize, bits: u8) -> OutputSignal;
}

#[allow(missing_docs)]
pub trait Instance: Signals + RegBlock {
    fn rx_reset() {
        let r = Self::register_block();
        r.conf().modify(|_, w| w.rx_reset().set_bit());
        r.conf().modify(|_, w| w.rx_reset().clear_bit());
    }

    fn rx_dma_reset() {
        let r = Self::register_block();
        r.lc_conf().modify(|_, w| w.in_rst().set_bit());
        r.lc_conf().modify(|_, w| w.in_rst().clear_bit());
    }

    fn rx_fifo_reset() {
        let r = Self::register_block();
        r.conf().modify(|_, w| w.rx_fifo_reset().set_bit());
        r.conf().modify(|_, w| w.rx_fifo_reset().clear_bit());
    }

    fn tx_reset() {
        let r = Self::register_block();
        r.conf().modify(|_, w| w.tx_reset().set_bit());
        r.conf().modify(|_, w| w.tx_reset().clear_bit());
    }

    fn tx_dma_reset() {
        let r = Self::register_block();
        r.lc_conf().modify(|_, w| w.out_rst().set_bit());
        r.lc_conf().modify(|_, w| w.out_rst().clear_bit());
    }

    fn tx_fifo_reset() {
        let r = Self::register_block();
        r.conf().modify(|_, w| w.tx_fifo_reset().set_bit());
        r.conf().modify(|_, w| w.tx_fifo_reset().clear_bit());
    }

    fn tx_clear_interrupts() {
        let r = Self::register_block();
        r.int_clr().write(|w| {
            w.out_done().clear_bit_by_one();
            w.out_total_eof().clear_bit_by_one()
        });
    }

    fn tx_start() {
        let r = Self::register_block();
        r.conf().modify(|_, w| w.tx_start().set_bit());

        while r.state().read().tx_idle().bit_is_set() {
            // wait
        }
    }

    fn tx_stop() {
        let r = Self::register_block();
        r.conf().modify(|_, w| w.tx_start().clear_bit());
    }

    fn is_tx_done() -> bool {
        Self::register_block().state().read().tx_idle().bit_is_set()
    }

    fn tx_wait_done() {
        let r = Self::register_block();
        while r.state().read().tx_idle().bit_is_clear() {
            // wait
        }

        r.conf().modify(|_, w| w.tx_start().clear_bit());
    }

    fn setup(frequency: impl Into<fugit::HertzU32>, bits: u8) {
        let frequency: HertzU32 = frequency.into();

        // Initialize I2S dev
        Self::rx_reset();
        Self::tx_reset();
        Self::rx_dma_reset();
        Self::tx_dma_reset();
        Self::rx_fifo_reset();
        Self::tx_fifo_reset();

        let r = Self::register_block();

        // clear all bits and enable lcd mode
        r.conf2().write(|w| {
            // 8 bit mode needs this or it updates on half clocks!
            w.lcd_tx_wrx2_en().bit(bits == 8);
            w.lcd_en().set_bit()
        });

        r.sample_rate_conf().write(|w| unsafe {
            w.rx_bits_mod().bits(bits);
            w.tx_bits_mod().bits(bits);
            w.rx_bck_div_num().bits(2); // I think this is the number of "bclocks" per "sample"
            w.tx_bck_div_num().bits(2) // ??
        });

        r.clkm_conf().write(|w| unsafe {
            w.clka_ena().clear_bit();
            w.clkm_div_a().bits(63);
            w.clkm_div_b().bits(63);
            // We ignore the possibility for fractional division here, clkspeed_hz must
            // round up for a fractional clock speed, must result in >= 2
            // TODO: proper clock configuration!!!!
            w.clkm_div_num()
                .bits((80000000u32 / (frequency.to_Hz() + 1)) as u8)
        });

        r.fifo_conf().write(|w| unsafe {
            w.rx_fifo_mod_force_en().set_bit();
            w.tx_fifo_mod_force_en().set_bit();
            w.rx_fifo_mod().bits(1);
            w.tx_fifo_mod().bits(1);
            w.rx_data_num().bits(32);
            w.tx_data_num().bits(32);
            w.dscr_en().set_bit()
        });

        r.conf1().write(|w| {
            w.tx_stop_en().set_bit();
            w.rx_pcm_bypass().set_bit();
            w.tx_pcm_bypass().set_bit()
        });

        r.conf_chan().write(|w| unsafe {
            w.rx_chan_mod().bits(1);
            w.tx_chan_mod().bits(1)
        });

        r.conf().modify(|_, w| {
            w.rx_mono().set_bit();
            w.tx_mono().set_bit();
            w.rx_right_first().set_bit();
            w.tx_right_first().set_bit()
        });
        r.timing().reset();

        r.pd_conf().modify(|_, w| {
            w.fifo_force_pu().set_bit();
            w.fifo_force_pd().clear_bit()
        });
    }

    #[cfg(all(feature = "debug", feature = "log"))]
    /// Dump the I2S peripheral configuration
    fn dump() {
        let r = Self::register_block();
        info!("conf: {:#?}", r.conf().read());
        info!("conf1: {:#?}", r.conf1().read());
        info!("conf2: {:#?}", r.conf2().read());
        info!("conf_chan: {:#?}", r.conf_chan().read());
        info!("timing: {:#?}", r.timing().read());
        info!("sample_rate_conf: {:#?}", r.sample_rate_conf().read());
        info!("fifo_conf: {:#?}", r.fifo_conf().read());
        info!("lc_conf: {:#?}", r.lc_conf().read());
        info!("int_raw: {:#?}", r.int_raw().read());
        info!("int_st: {:#?}", r.int_st().read());
        info!("int_ena: {:#?}", r.int_ena().read());
        info!("state: {:#?}", r.state().read());
    }
}

impl RegBlock for I2S0 {
    fn register_block() -> &'static RegisterBlock {
        unsafe { &*I2S0::PTR.cast::<RegisterBlock>() }
    }
}

impl Signals for I2S0 {
    fn get_peripheral() -> crate::system::Peripheral {
        crate::system::Peripheral::I2s0
    }

    fn get_dma_peripheral() -> crate::dma::DmaPeripheral {
        crate::dma::DmaPeripheral::I2s0
    }

    fn ws_signal() -> OutputSignal {
        OutputSignal::I2S0O_WS
    }
    fn data_out_signal(i: usize, bits: u8) -> OutputSignal {
        // Because of... reasons... the 16-bit values for i2s1 appear on d8...d23
        let offset = match bits {
            8 => 8,
            16 => 8,
            _ => panic!("Invalid number of bits"),
        };
        match i + offset {
            0 => OutputSignal::I2S0O_DATA_0,
            1 => OutputSignal::I2S0O_DATA_1,
            2 => OutputSignal::I2S0O_DATA_2,
            3 => OutputSignal::I2S0O_DATA_3,
            4 => OutputSignal::I2S0O_DATA_4,
            5 => OutputSignal::I2S0O_DATA_5,
            6 => OutputSignal::I2S0O_DATA_6,
            7 => OutputSignal::I2S0O_DATA_7,
            8 => OutputSignal::I2S0O_DATA_8,
            9 => OutputSignal::I2S0O_DATA_9,
            10 => OutputSignal::I2S0O_DATA_10,
            11 => OutputSignal::I2S0O_DATA_11,
            12 => OutputSignal::I2S0O_DATA_12,
            13 => OutputSignal::I2S0O_DATA_13,
            14 => OutputSignal::I2S0O_DATA_14,
            15 => OutputSignal::I2S0O_DATA_15,
            16 => OutputSignal::I2S0O_DATA_16,
            17 => OutputSignal::I2S0O_DATA_17,
            18 => OutputSignal::I2S0O_DATA_18,
            19 => OutputSignal::I2S0O_DATA_19,
            20 => OutputSignal::I2S0O_DATA_20,
            21 => OutputSignal::I2S0O_DATA_21,
            22 => OutputSignal::I2S0O_DATA_22,
            23 => OutputSignal::I2S0O_DATA_23,
            _ => panic!("Invalid I2S0 Dout pin"),
        }
    }
}
impl Instance for I2S0 {}

impl RegBlock for I2S1 {
    fn register_block() -> &'static RegisterBlock {
        unsafe { &*I2S1::PTR.cast::<RegisterBlock>() }
    }
}
impl Signals for I2S1 {
    fn get_peripheral() -> crate::system::Peripheral {
        crate::system::Peripheral::I2s1
    }
    fn get_dma_peripheral() -> crate::dma::DmaPeripheral {
        crate::dma::DmaPeripheral::I2s1
    }

    fn ws_signal() -> OutputSignal {
        OutputSignal::I2S1O_WS
    }
    fn data_out_signal(i: usize, bits: u8) -> OutputSignal {
        // Because of... reasons... the 16-bit values for i2s1 appear on d8...d23
        let offset = if bits == 16 { 8 } else { 0 };
        match i + offset {
            0 => OutputSignal::I2S1O_DATA_0,
            1 => OutputSignal::I2S1O_DATA_1,
            2 => OutputSignal::I2S1O_DATA_2,
            3 => OutputSignal::I2S1O_DATA_3,
            4 => OutputSignal::I2S1O_DATA_4,
            5 => OutputSignal::I2S1O_DATA_5,
            6 => OutputSignal::I2S1O_DATA_6,
            7 => OutputSignal::I2S1O_DATA_7,
            8 => OutputSignal::I2S1O_DATA_8,
            9 => OutputSignal::I2S1O_DATA_9,
            10 => OutputSignal::I2S1O_DATA_10,
            11 => OutputSignal::I2S1O_DATA_11,
            12 => OutputSignal::I2S1O_DATA_12,
            13 => OutputSignal::I2S1O_DATA_13,
            14 => OutputSignal::I2S1O_DATA_14,
            15 => OutputSignal::I2S1O_DATA_15,
            16 => OutputSignal::I2S1O_DATA_16,
            17 => OutputSignal::I2S1O_DATA_17,
            18 => OutputSignal::I2S1O_DATA_18,
            19 => OutputSignal::I2S1O_DATA_19,
            20 => OutputSignal::I2S1O_DATA_20,
            21 => OutputSignal::I2S1O_DATA_21,
            22 => OutputSignal::I2S1O_DATA_22,
            23 => OutputSignal::I2S1O_DATA_23,
            _ => panic!("Invalid I2S1 Dout pin"),
        }
    }
}
impl Instance for I2S1 {}
