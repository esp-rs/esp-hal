//! # Parallel Interface (via I2S)
//!
//! ## Overview
//! The I2S parallel interface allows for high-speed data transfer between the
//! ESP32 and external devices. It is commonly used to external devices such as
//! LED matrix, LCD display, and Printer. Only TX is implemented. Each
//! unit can have up to 8 or 16 data signals (depending on your target hardware)
//! plus 1 clock signal.
//!
//! ## Notes
//!
//! Data output is interleaved:
//! - 8bit: [A, B, C, D] is output as [C, D, A, B]  (i.e., swapped as 16bit
//!   words)
//! - 16bit: [A, B, C, D] is output as [B, A, D, C] (i.e., 16bit words are
//!   swapped)
#![cfg_attr(esp32, doc = "")]
#![cfg_attr(
    esp32,
    doc = "I2S0 does not support true 8bit parallel output, so if you want to do 8bit"
)]
#![cfg_attr(
    esp32,
    doc = "you should use I2S1.  If you have to use I2S0, it will only output the even"
)]
#![cfg_attr(esp32, doc = "bytes! so [A, B, C, D] will be output as [A, C]!!!!")]
#![cfg_attr(esp32, doc = "")]
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
use private::{calculate_clock, I2sClockDividers, TxPins};

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
    gpio::{
        interconnect::{OutputConnection, PeripheralOutput},
        OutputSignal,
    },
    peripheral::{Peripheral, PeripheralRef},
    peripherals::{i2s0::RegisterBlock, I2S0, I2S1},
    system::PeripheralClockControl,
    Mode,
};

/// Represents a group of 16 output pins configured for 16-bit parallel data
/// transmission.
pub struct TxSixteenBits<'d> {
    pins: [PeripheralRef<'d, OutputConnection>; 16],
}

impl<'d> TxSixteenBits<'d> {
    #[allow(clippy::too_many_arguments)]
    /// Creates a new `TxSixteenBits` instance with the provided output pins.
    pub fn new(
        pin_0: impl Peripheral<P = impl PeripheralOutput> + 'd,
        pin_1: impl Peripheral<P = impl PeripheralOutput> + 'd,
        pin_2: impl Peripheral<P = impl PeripheralOutput> + 'd,
        pin_3: impl Peripheral<P = impl PeripheralOutput> + 'd,
        pin_4: impl Peripheral<P = impl PeripheralOutput> + 'd,
        pin_5: impl Peripheral<P = impl PeripheralOutput> + 'd,
        pin_6: impl Peripheral<P = impl PeripheralOutput> + 'd,
        pin_7: impl Peripheral<P = impl PeripheralOutput> + 'd,
        pin_8: impl Peripheral<P = impl PeripheralOutput> + 'd,
        pin_9: impl Peripheral<P = impl PeripheralOutput> + 'd,
        pin_10: impl Peripheral<P = impl PeripheralOutput> + 'd,
        pin_11: impl Peripheral<P = impl PeripheralOutput> + 'd,
        pin_12: impl Peripheral<P = impl PeripheralOutput> + 'd,
        pin_13: impl Peripheral<P = impl PeripheralOutput> + 'd,
        pin_14: impl Peripheral<P = impl PeripheralOutput> + 'd,
        pin_15: impl Peripheral<P = impl PeripheralOutput> + 'd,
    ) -> Self {
        crate::into_mapped_ref!(
            pin_0, pin_1, pin_2, pin_3, pin_4, pin_5, pin_6, pin_7, pin_8, pin_9, pin_10, pin_11,
            pin_12, pin_13, pin_14, pin_15
        );

        Self {
            pins: [
                pin_0, pin_1, pin_2, pin_3, pin_4, pin_5, pin_6, pin_7, pin_8, pin_9, pin_10,
                pin_11, pin_12, pin_13, pin_14, pin_15,
            ],
        }
    }
}

impl<'d> TxPins for TxSixteenBits<'d> {
    fn bits(&self) -> u8 {
        16
    }

    fn configure<I: Instance>(&mut self, instance: &PeripheralRef<'_, I>) {
        use crate::private::Internal;
        let bits: u8 = self.bits();
        for (i, pin) in self.pins.iter_mut().enumerate() {
            pin.set_to_push_pull_output(Internal);
            pin.connect_peripheral_to_output(instance.data_out_signal(i, bits), Internal);
        }
    }
}

/// Represents a group of 8 output pins configured for 8-bit parallel data
/// transmission.
pub struct TxEightBits<'d> {
    pins: [PeripheralRef<'d, OutputConnection>; 8],
}

impl<'d> TxEightBits<'d> {
    #[allow(clippy::too_many_arguments)]
    /// Creates a new `TxSEightBits` instance with the provided output pins.
    pub fn new(
        pin_0: impl Peripheral<P = impl PeripheralOutput> + 'd,
        pin_1: impl Peripheral<P = impl PeripheralOutput> + 'd,
        pin_2: impl Peripheral<P = impl PeripheralOutput> + 'd,
        pin_3: impl Peripheral<P = impl PeripheralOutput> + 'd,
        pin_4: impl Peripheral<P = impl PeripheralOutput> + 'd,
        pin_5: impl Peripheral<P = impl PeripheralOutput> + 'd,
        pin_6: impl Peripheral<P = impl PeripheralOutput> + 'd,
        pin_7: impl Peripheral<P = impl PeripheralOutput> + 'd,
    ) -> Self {
        crate::into_mapped_ref!(pin_0);
        crate::into_mapped_ref!(pin_1);
        crate::into_mapped_ref!(pin_2);
        crate::into_mapped_ref!(pin_3);
        crate::into_mapped_ref!(pin_4);
        crate::into_mapped_ref!(pin_5);
        crate::into_mapped_ref!(pin_6);
        crate::into_mapped_ref!(pin_7);

        Self {
            pins: [pin_0, pin_1, pin_2, pin_3, pin_4, pin_5, pin_6, pin_7],
        }
    }
}

impl<'d> TxPins for TxEightBits<'d> {
    fn bits(&self) -> u8 {
        8
    }

    fn configure<I: Instance>(&mut self, instance: &PeripheralRef<'_, I>) {
        use crate::private::Internal;
        let bits: u8 = self.bits();
        for (i, pin) in self.pins.iter_mut().enumerate() {
            pin.set_to_push_pull_output(Internal);
            pin.connect_peripheral_to_output(instance.data_out_signal(i, bits), Internal);
        }
    }
}

/// I2S Parallel Interface
pub struct I2sParallel<'d, I: Instance, DM: Mode> {
    instance: PeripheralRef<'d, I>,
    tx_channel: ChannelTx<'d, <I as DmaEligible>::Dma>,
    mode: PhantomData<DM>,
}

impl<'d, I: Instance, DM: Mode> I2sParallel<'d, I, DM> {
    /// Create a new I2S Parallel Interface
    pub fn new<P, CH, CLK: PeripheralOutput>(
        i2s: impl Peripheral<P = I> + 'd,
        channel: Channel<'d, CH, DM>,
        frequency: impl Into<fugit::HertzU32>,
        mut pins: P,
        clock_pin: impl Peripheral<P = CLK> + 'd,
    ) -> Self
    where
        CH: DmaChannelConvert<I::Dma>,
        P: TxPins,
    {
        crate::into_ref!(i2s);
        crate::into_mapped_ref!(clock_pin);

        channel.runtime_ensure_compatible(&i2s);
        let channel = channel.degrade();

        PeripheralClockControl::reset(i2s.get_peripheral());
        PeripheralClockControl::enable(i2s.get_peripheral());
        // configure the I2S peripheral for parallel mode
        i2s.setup(frequency, pins.bits());
        // setup the clock pin
        clock_pin.set_to_push_pull_output(crate::private::Internal);
        clock_pin.connect_peripheral_to_output(i2s.ws_signal(), crate::private::Internal);

        pins.configure(&i2s);
        Self {
            instance: i2s,
            tx_channel: channel.tx,
            mode: PhantomData,
        }
    }

    /// Write data to the I2S peripheral
    pub fn send<BUF: DmaTxBuffer>(
        mut self,
        mut data: BUF,
    ) -> Result<I2sParallelTransfer<'d, I, BUF, DM>, (DmaError, Self, BUF)> {
        self.instance.tx_reset();
        self.instance.tx_fifo_reset();
        let result = unsafe {
            self.tx_channel
                .prepare_transfer(self.instance.get_dma_peripheral(), &mut data)
        }
        .and_then(|_| self.tx_channel.start_transfer());
        if let Err(err) = result {
            return Err((err, self, data));
        }
        crate::rom::ets_delay_us(1);
        self.instance.tx_start();
        Ok(I2sParallelTransfer {
            i2s: ManuallyDrop::new(self),
            buf_view: ManuallyDrop::new(data.into_view()),
        })
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
        self.i2s.instance.is_tx_done()
    }

    /// Wait for the transfer to finish
    pub fn wait(mut self) -> (I2sParallel<'d, I, DM>, BUF) {
        self.i2s.instance.tx_wait_done();
        let i2s = unsafe { ManuallyDrop::take(&mut self.i2s) };
        let view = unsafe { ManuallyDrop::take(&mut self.buf_view) };
        (i2s, BUF::from_view(view))
    }

    fn stop_peripherals(&mut self) {
        self.i2s.instance.tx_stop();
        self.i2s.tx_channel.stop_transfer();
    }
}

impl<'d, I, BUF> I2sParallelTransfer<'d, I, BUF, crate::Async>
where
    I: Instance,
    BUF: DmaTxBuffer,
{
    /// Wait for the transfer to finish
    pub async fn wait_for_done(&mut self) -> Result<(), DmaError> {
        DmaTxFuture::new(&mut self.i2s.tx_channel).await
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

impl<'d, I, BUF, DM> Drop for I2sParallelTransfer<'d, I, BUF, DM>
where
    I: Instance,
    BUF: DmaTxBuffer,
    DM: Mode,
{
    fn drop(&mut self) {
        self.stop_peripherals();

        // SAFETY: This is Drop, we know that self.i2s and self.buf_view
        // won't be touched again.
        let view = unsafe {
            ManuallyDrop::drop(&mut self.i2s);
            ManuallyDrop::take(&mut self.buf_view)
        };
        let _ = BUF::from_view(view);
    }
}

mod private {
    use fugit::HertzU32;

    use crate::peripheral::PeripheralRef;

    pub trait TxPins {
        fn bits(&self) -> u8;
        fn configure<I: super::Instance>(&mut self, instance: &PeripheralRef<'_, I>);
    }

    #[derive(Debug)]
    pub struct I2sClockDividers {
        pub mclk_divider: u32,
        pub bclk_divider: u32,
        pub denominator: u32,
        pub numerator: u32,
    }

    #[cfg(any(esp32, esp32s2))]
    const I2S_LL_MCLK_DIVIDER_BIT_WIDTH: usize = 6;

    const I2S_LL_MCLK_DIVIDER_MAX: usize = (1 << I2S_LL_MCLK_DIVIDER_BIT_WIDTH) - 1;

    pub fn calculate_clock(
        sample_rate: impl Into<fugit::HertzU32>,
        data_bits: u8,
    ) -> I2sClockDividers {
        // this loosely corresponds to `i2s_std_calculate_clock` and
        // `i2s_ll_tx_set_mclk` in esp-idf
        //
        // main difference is we are using fixed-point arithmetic here
        // plus adjusted for parallel interface clocking

        let sclk = crate::soc::constants::I2S_SCLK; // for now it's fixed 160MHz and 96MHz (just H2)

        let rate_hz: HertzU32 = sample_rate.into();
        let rate = rate_hz.raw();

        let mclk = rate * 2;
        let bclk_divider: u32 = if data_bits == 8 { 2 } else { 1 };
        let mut mclk_divider = sclk / mclk;

        let mut ma: u32;
        let mut mb: u32;
        let mut denominator: u32 = 0;
        let mut numerator: u32 = 0;

        let freq_diff = sclk.abs_diff(mclk * mclk_divider);

        if freq_diff != 0 {
            let decimal = freq_diff as u64 * 10000 / mclk as u64;
            // Carry bit if the decimal is greater than 1.0 - 1.0 / (63.0 * 2) = 125.0 /
            // 126.0
            if decimal > 1250000 / 126 {
                mclk_divider += 1;
            } else {
                let mut min: u32 = !0;

                for a in 2..=I2S_LL_MCLK_DIVIDER_MAX {
                    let b = (a as u64) * (freq_diff as u64 * 10000u64 / mclk as u64) + 5000;
                    ma = ((freq_diff as u64 * 10000u64 * a as u64) / 10000) as u32;
                    mb = (mclk as u64 * (b / 10000)) as u32;

                    if ma == mb {
                        denominator = a as u32;
                        numerator = (b / 10000) as u32;
                        break;
                    }

                    if mb.abs_diff(ma) < min {
                        denominator = a as u32;
                        numerator = b as u32;
                        min = mb.abs_diff(ma);
                    }
                }
            }
        }

        I2sClockDividers {
            mclk_divider,
            bclk_divider,
            denominator,
            numerator,
        }
    }
}

#[allow(missing_docs)]
pub trait RegBlock: PeripheralMarker + DmaEligible {
    fn register_block(&self) -> &'static RegisterBlock;
}

#[allow(missing_docs)]
pub trait Signals {
    fn get_peripheral(&self) -> crate::system::Peripheral;
    fn get_dma_peripheral(&self) -> crate::dma::DmaPeripheral;
    fn ws_signal(&self) -> OutputSignal;
    fn data_out_signal(&self, i: usize, bits: u8) -> OutputSignal;
}

#[allow(missing_docs)]
pub trait Instance: Signals + RegBlock {
    fn rx_reset(&self) {
        let r = self.register_block();
        r.conf().modify(|_, w| w.rx_reset().set_bit());
        r.conf().modify(|_, w| w.rx_reset().clear_bit());
    }

    fn rx_dma_reset(&self) {
        let r = self.register_block();
        r.lc_conf().modify(|_, w| w.in_rst().set_bit());
        r.lc_conf().modify(|_, w| w.in_rst().clear_bit());
    }

    fn rx_fifo_reset(&self) {
        let r = self.register_block();
        r.conf().modify(|_, w| w.rx_fifo_reset().set_bit());
        r.conf().modify(|_, w| w.rx_fifo_reset().clear_bit());
    }

    fn tx_reset(&self) {
        let r = self.register_block();
        r.conf().modify(|_, w| w.tx_reset().set_bit());
        // without this delay starting a subsequent transfer will hang waiting
        // for tx_idle to clear (the transfer does not start).
        // While 20 clocks works for 80MHz cpu but 100 is needed for 240MHz!
        xtensa_lx::timer::delay(100);
        r.conf().modify(|_, w| w.tx_reset().clear_bit());
    }

    fn tx_dma_reset(&self) {
        let r = self.register_block();
        r.lc_conf().modify(|_, w| w.out_rst().set_bit());
        r.lc_conf().modify(|_, w| w.out_rst().clear_bit());
    }

    fn tx_fifo_reset(&self) {
        let r = self.register_block();
        r.conf().modify(|_, w| w.tx_fifo_reset().set_bit());
        r.conf().modify(|_, w| w.tx_fifo_reset().clear_bit());
    }

    fn tx_clear_interrupts(&self) {
        let r = self.register_block();
        r.int_clr().write(|w| {
            w.out_done().clear_bit_by_one();
            w.out_total_eof().clear_bit_by_one()
        });
    }

    fn tx_start(&self) {
        let r = self.register_block();

        // wait for data to show up in the fifo
        while r.int_raw().read().tx_rempty().bit_is_clear() {
            // wait
        }

        // without this transfers are not reliable!
        xtensa_lx::timer::delay(1);

        r.conf().modify(|_, w| w.tx_start().set_bit());

        while r.state().read().tx_idle().bit_is_set() {
            // wait
        }
    }

    fn tx_stop(&self) {
        let r = self.register_block();
        r.conf().modify(|_, w| w.tx_start().clear_bit());
    }

    fn is_tx_done(&self) -> bool {
        self.register_block().state().read().tx_idle().bit_is_set()
    }

    fn tx_wait_done(&self) {
        let r = self.register_block();
        while r.state().read().tx_idle().bit_is_clear() {
            // wait
        }

        r.conf().modify(|_, w| w.tx_start().clear_bit());
        r.int_clr().write(|w| {
            w.out_done().clear_bit_by_one();
            w.out_total_eof().clear_bit_by_one()
        });
    }

    fn set_clock(&self, clock_settings: I2sClockDividers) {
        let r = self.register_block();

        r.clkm_conf().modify(|r, w| unsafe {
            w.bits(r.bits() | (crate::soc::constants::I2S_DEFAULT_CLK_SRC << 21))
            // select PLL_160M
        });

        #[cfg(esp32)]
        r.clkm_conf().modify(|_, w| w.clka_ena().clear_bit());

        r.clkm_conf().modify(|_, w| unsafe {
            w.clk_en().set_bit();
            w.clkm_div_num().bits(clock_settings.mclk_divider as u8)
        });

        r.clkm_conf().modify(|_, w| unsafe {
            w.clkm_div_a().bits(clock_settings.denominator as u8);
            w.clkm_div_b().bits(clock_settings.numerator as u8)
        });

        r.sample_rate_conf().modify(|_, w| unsafe {
            w.tx_bck_div_num().bits(clock_settings.bclk_divider as u8);
            w.rx_bck_div_num().bits(clock_settings.bclk_divider as u8)
        });
    }

    fn setup(&self, frequency: impl Into<fugit::HertzU32>, bits: u8) {
        let frequency: HertzU32 = frequency.into();

        self.set_clock(calculate_clock(frequency, bits));

        // Initialize I2S dev
        self.rx_reset();
        self.tx_reset();
        self.rx_fifo_reset();
        self.tx_fifo_reset();
        self.rx_dma_reset();
        self.tx_dma_reset();

        let r = self.register_block();

        // clear all bits and enable lcd mode
        r.conf2().write(|w| {
            // 8 bit mode needs this or it updates on half clocks!
            w.lcd_tx_wrx2_en().bit(bits == 8);
            w.lcd_en().set_bit()
        });

        r.sample_rate_conf().modify(|_, w| unsafe {
            w.rx_bits_mod().bits(bits);
            w.tx_bits_mod().bits(bits)
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
}

impl RegBlock for I2S0 {
    fn register_block(&self) -> &'static RegisterBlock {
        unsafe { &*I2S0::PTR.cast::<RegisterBlock>() }
    }
}

impl Signals for I2S0 {
    fn get_peripheral(&self) -> crate::system::Peripheral {
        crate::system::Peripheral::I2s0
    }

    fn get_dma_peripheral(&self) -> crate::dma::DmaPeripheral {
        crate::dma::DmaPeripheral::I2s0
    }

    fn ws_signal(&self) -> OutputSignal {
        OutputSignal::I2S0O_WS
    }
    fn data_out_signal(&self, i: usize, bits: u8) -> OutputSignal {
        // signals for 8bit and 16bit both start at an offset of 8 for I2S0
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
    fn register_block(&self) -> &'static RegisterBlock {
        unsafe { &*I2S1::PTR.cast::<RegisterBlock>() }
    }
}
impl Signals for I2S1 {
    fn get_peripheral(&self) -> crate::system::Peripheral {
        crate::system::Peripheral::I2s1
    }
    fn get_dma_peripheral(&self) -> crate::dma::DmaPeripheral {
        crate::dma::DmaPeripheral::I2s1
    }

    fn ws_signal(&self) -> OutputSignal {
        OutputSignal::I2S1O_WS
    }
    fn data_out_signal(&self, i: usize, bits: u8) -> OutputSignal {
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
