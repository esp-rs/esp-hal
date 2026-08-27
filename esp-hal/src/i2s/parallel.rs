#![cfg_attr(docsrs, procmacros::doc_replace)]
//! # Parallel Interface (via I2S)
//!
//! ## Overview
//! The I2S parallel interface provides high-speed data transfer between the
//! ESP32 and external devices. It is commonly used with devices such as LED
//! matrices, LCD displays, and printers. Only TX is implemented. Each
//! unit can have up to 8 or 16 data signals (depending on the target hardware)
//! plus one clock signal.
//!
//! ## Notes
//!
//! Data output is interleaved:
//! - 8-bit: [A, B, C, D] is output as [C, D, A, B]  (i.e., swapped as 16-bit words)
//! - 16-bit: [A, B, C, D] is output as [B, A, D, C] (i.e., 16-bit words are swapped)
#![cfg_attr(esp32, doc = "")]
#![cfg_attr(
    esp32,
    doc = "I2S0 does not support true 8-bit parallel output, so if you want to do 8-bit"
)]
#![cfg_attr(
    esp32,
    doc = "you should use I2S1.  If you have to use I2S0, it only outputs the even"
)]
#![cfg_attr(esp32, doc = "bytes! so [A, B, C, D] is output as [A, C]!")]
#![cfg_attr(esp32, doc = "")]
//! ## Configuration
//!
//! The driver uses DMA for efficient data transfer and
//! supports various configurations, such as different data formats, standards
//! (e.g., Philips) and pin configurations. It relies on other peripheral
//! modules, such as
//!   - `GPIO`
//!   - `DMA`
//!   - `system` (to configure and enable the I2S peripheral)
//!
//! ## Examples
//!
//! ```rust, no_run
//! # {before_snippet}
//! # use esp_hal::dma::DmaTxBuf;
//! # use esp_hal::dma_tx_buffer;
//! # use esp_hal::delay::Delay;
//! # use esp_hal::i2s::parallel::{I2sParallel, TxEightBits};
//!
//! const BUFFER_SIZE: usize = 256;
//!
//! let delay = Delay::new();
//! let dma_channel = peripherals.DMA_I2S1;
//! let i2s = peripherals.I2S1;
//! let clock = peripherals.GPIO25;
//!
//! let pins = TxEightBits::new(
//!     peripherals.GPIO16,
//!     peripherals.GPIO4,
//!     peripherals.GPIO17,
//!     peripherals.GPIO18,
//!     peripherals.GPIO5,
//!     peripherals.GPIO19,
//!     peripherals.GPIO12,
//!     peripherals.GPIO14,
//! );
//!
//! let mut parallel =
//!     I2sParallel::new(i2s, dma_channel, Rate::from_mhz(1), pins, clock).into_async();
//!
//! let mut tx_buf = dma_tx_buffer!(BUFFER_SIZE).unwrap();
//! for (i, data) in tx_buf.as_mut_slice().chunks_mut(4).enumerate() {
//!     let offset = i * 4;
//!     // I2S parallel driver expects the buffer to be interleaved
//!     data[0] = (offset + 2) as u8;
//!     data[1] = (offset + 3) as u8;
//!     data[2] = offset as u8;
//!     data[3] = (offset + 1) as u8;
//! }
//!
//! // Sending 256 bytes.
//! loop {
//!     let xfer = match parallel.send(tx_buf) {
//!         Ok(xfer) => xfer,
//!         Err(_) => {
//!             panic!("Failed to send buffer");
//!         }
//!     };
//!     (parallel, tx_buf) = xfer.wait();
//!     delay.delay_millis(10);
//! }
//! # }
//! ```
use core::{
    mem::ManuallyDrop,
    ops::{Deref, DerefMut},
};

use enumset::{EnumSet, EnumSetType};

use crate::{
    Async,
    Blocking,
    DriverMode,
    RegisterToggle,
    clock::dividers::FractionalDivider,
    dma::{ChannelTx, DmaEligiblePeripheral, DmaError, DmaTxBuffer, asynch::DmaTxFuture},
    gpio::{
        OutputConfig,
        OutputSignal,
        interconnect::{self, PeripheralOutput},
    },
    i2s::AnyI2s,
    interrupt::InterruptHandler,
    pac::i2s0::RegisterBlock,
    peripherals::{I2S0, I2S1},
    system::PeripheralGuard,
    time::Rate,
};

#[doc(hidden)]
pub trait TxPins<'d> {
    fn bus_width(&self) -> u8;
    fn configure(&mut self, instance: &(impl Instance + 'd));
}

/// Represents a group of 16 output pins configured for 16-bit parallel data
/// transmission.
#[instability::unstable]
pub struct TxSixteenBits<'d> {
    pins: [interconnect::OutputSignal<'d>; 16],
}

impl<'d> TxSixteenBits<'d> {
    #[expect(clippy::too_many_arguments)]
    /// Creates a new `TxSixteenBits` instance with the provided output pins.
    pub fn new(
        pin_0: impl PeripheralOutput<'d>,
        pin_1: impl PeripheralOutput<'d>,
        pin_2: impl PeripheralOutput<'d>,
        pin_3: impl PeripheralOutput<'d>,
        pin_4: impl PeripheralOutput<'d>,
        pin_5: impl PeripheralOutput<'d>,
        pin_6: impl PeripheralOutput<'d>,
        pin_7: impl PeripheralOutput<'d>,
        pin_8: impl PeripheralOutput<'d>,
        pin_9: impl PeripheralOutput<'d>,
        pin_10: impl PeripheralOutput<'d>,
        pin_11: impl PeripheralOutput<'d>,
        pin_12: impl PeripheralOutput<'d>,
        pin_13: impl PeripheralOutput<'d>,
        pin_14: impl PeripheralOutput<'d>,
        pin_15: impl PeripheralOutput<'d>,
    ) -> Self {
        Self {
            pins: [
                pin_0.into(),
                pin_1.into(),
                pin_2.into(),
                pin_3.into(),
                pin_4.into(),
                pin_5.into(),
                pin_6.into(),
                pin_7.into(),
                pin_8.into(),
                pin_9.into(),
                pin_10.into(),
                pin_11.into(),
                pin_12.into(),
                pin_13.into(),
                pin_14.into(),
                pin_15.into(),
            ],
        }
    }
}

impl<'d> TxPins<'d> for TxSixteenBits<'d> {
    fn bus_width(&self) -> u8 {
        self.pins.len() as u8
    }

    fn configure(&mut self, instance: &(impl Instance + 'd)) {
        let bits = self.bus_width();
        for (i, pin) in self.pins.iter_mut().enumerate() {
            pin.apply_output_config(&OutputConfig::default());
            pin.set_output_enable(true);
            instance.data_out_signal(i, bits).connect_to(pin);
        }
    }
}

/// Represents a group of 8 output pins configured for 8-bit parallel data
/// transmission.
#[instability::unstable]
pub struct TxEightBits<'d> {
    pins: [interconnect::OutputSignal<'d>; 8],
}

impl<'d> TxEightBits<'d> {
    #[expect(clippy::too_many_arguments)]
    /// Creates a new [`TxEightBits`] instance with the provided output pins.
    pub fn new(
        pin_0: impl PeripheralOutput<'d>,
        pin_1: impl PeripheralOutput<'d>,
        pin_2: impl PeripheralOutput<'d>,
        pin_3: impl PeripheralOutput<'d>,
        pin_4: impl PeripheralOutput<'d>,
        pin_5: impl PeripheralOutput<'d>,
        pin_6: impl PeripheralOutput<'d>,
        pin_7: impl PeripheralOutput<'d>,
    ) -> Self {
        Self {
            pins: [
                pin_0.into(),
                pin_1.into(),
                pin_2.into(),
                pin_3.into(),
                pin_4.into(),
                pin_5.into(),
                pin_6.into(),
                pin_7.into(),
            ],
        }
    }
}

impl<'d> TxPins<'d> for TxEightBits<'d> {
    fn bus_width(&self) -> u8 {
        self.pins.len() as u8
    }

    fn configure(&mut self, instance: &(impl Instance + 'd)) {
        let bits = self.bus_width();
        for (i, pin) in self.pins.iter_mut().enumerate() {
            pin.apply_output_config(&OutputConfig::default());
            pin.set_output_enable(true);
            instance.data_out_signal(i, bits).connect_to(pin);
        }
    }
}

/// Interrupts from the I2S parallel peripheral.
#[derive(Debug, EnumSetType)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
#[instability::unstable]
pub enum I2sParallelInterrupt {
    /// The DMA finishes the out descriptor chain (OUT_DONE).
    Done,

    /// The DMA uses a descriptor with the suc_eof flag set (OUT_EOF).
    ///
    /// The peripheral sets the OUT_EOF flag for every descriptor with suc_eof
    /// set. This interrupt also works on a circular DMA chain. On a circular
    /// chain, the transfer does not end. The peripheral does not set
    /// OUT_TOTAL_EOF on a circular chain.
    Eof,

    /// The DMA gets a descriptor with an error (OUT_DSCR_ERR).
    DescriptorError,

    /// The DMA sends all the data of the transfer (OUT_TOTAL_EOF).
    ///
    /// On a circular DMA chain, the transfer does not end. The peripheral does
    /// not set OUT_TOTAL_EOF on a circular chain. Use [`Self::Eof`] with a
    /// circular chain.
    TotalEof,
}

fn internal_listen(regs: &RegisterBlock, interrupts: EnumSet<I2sParallelInterrupt>, enable: bool) {
    regs.int_ena().modify(|_, w| {
        for interrupt in interrupts {
            match interrupt {
                I2sParallelInterrupt::Done => w.out_done().bit(enable),
                I2sParallelInterrupt::Eof => w.out_eof().bit(enable),
                I2sParallelInterrupt::DescriptorError => w.out_dscr_err().bit(enable),
                I2sParallelInterrupt::TotalEof => w.out_total_eof().bit(enable),
            };
        }
        w
    });
}

fn internal_interrupts(regs: &RegisterBlock) -> EnumSet<I2sParallelInterrupt> {
    let mut result = EnumSet::new();
    let ints = regs.int_st().read();

    if ints.out_done().bit() {
        result.insert(I2sParallelInterrupt::Done);
    }
    if ints.out_eof().bit() {
        result.insert(I2sParallelInterrupt::Eof);
    }
    if ints.out_dscr_err().bit() {
        result.insert(I2sParallelInterrupt::DescriptorError);
    }
    if ints.out_total_eof().bit() {
        result.insert(I2sParallelInterrupt::TotalEof);
    }

    result
}

fn internal_clear_interrupts(regs: &RegisterBlock, interrupts: EnumSet<I2sParallelInterrupt>) {
    regs.int_clr().write(|w| {
        for interrupt in interrupts {
            match interrupt {
                I2sParallelInterrupt::Done => w.out_done().clear_bit_by_one(),
                I2sParallelInterrupt::Eof => w.out_eof().clear_bit_by_one(),
                I2sParallelInterrupt::DescriptorError => w.out_dscr_err().clear_bit_by_one(),
                I2sParallelInterrupt::TotalEof => w.out_total_eof().clear_bit_by_one(),
            };
        }
        w
    });
}

/// I2S parallel interface
#[instability::unstable]
pub struct I2sParallel<'d, Dm>
where
    Dm: DriverMode,
{
    instance: AnyI2s<'d>,
    tx_channel: ChannelTx<Dm, I2sParallelTxErased<'d>>,
    _guard: PeripheralGuard,
    _clk_guard: crate::i2s::master::I2sDirClkGuard,
}

impl<'d> I2sParallel<'d, Blocking> {
    fn new_internal(
        i2s: impl Instance + 'd,
        channel: I2sParallelTxErased<'d>,
        frequency: Rate,
        mut pins: impl TxPins<'d>,
        clock_pin: impl PeripheralOutput<'d>,
    ) -> Self {
        let channel = ChannelTx::new(channel);
        let i2s = i2s.degrade();
        channel.runtime_ensure_compatible(i2s.dma_peripheral());

        let guard = PeripheralGuard::new(i2s.peripheral());

        // configure the I2S peripheral for parallel mode
        i2s.setup(frequency, pins.bus_width());
        // setup the clock pin
        let clock_pin = clock_pin.into();

        clock_pin.apply_output_config(&OutputConfig::default());
        clock_pin.set_output_enable(true);

        i2s.ws_signal().connect_to(&clock_pin);

        pins.configure(&i2s);
        let clk_guard = crate::i2s::master::I2sDirClkGuard::request_tx(i2s.clock_instance());
        Self {
            instance: i2s,
            tx_channel: channel,
            _guard: guard,
            _clk_guard: clk_guard,
        }
    }

    /// Converts the I2S parallel instance into [`Async`] mode.
    pub fn into_async(self) -> I2sParallel<'d, Async> {
        I2sParallel {
            instance: self.instance,
            tx_channel: self.tx_channel.into_async(),
            _guard: self._guard,
            _clk_guard: self._clk_guard,
        }
    }

    /// Sets the interrupt handler for the I2S parallel peripheral.
    ///
    /// The new handler removes the old handler. This method does not turn on
    /// the interrupt sources. Use [`Self::listen`] to turn on interrupt
    /// sources.
    #[instability::unstable]
    pub fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        self.instance.set_interrupt_handler(handler);
    }

    /// Turns on the given interrupt sources.
    #[instability::unstable]
    pub fn listen(&mut self, interrupts: impl Into<EnumSet<I2sParallelInterrupt>>) {
        internal_listen(self.instance.regs(), interrupts.into(), true);
    }

    /// Turns off the given interrupt sources.
    #[instability::unstable]
    pub fn unlisten(&mut self, interrupts: impl Into<EnumSet<I2sParallelInterrupt>>) {
        internal_listen(self.instance.regs(), interrupts.into(), false);
    }

    /// Returns the interrupt sources that are set.
    #[instability::unstable]
    pub fn interrupts(&mut self) -> EnumSet<I2sParallelInterrupt> {
        internal_interrupts(self.instance.regs())
    }

    /// Clears the interrupt flags of the given interrupt sources.
    #[instability::unstable]
    pub fn clear_interrupts(&mut self, interrupts: impl Into<EnumSet<I2sParallelInterrupt>>) {
        internal_clear_interrupts(self.instance.regs(), interrupts.into());
    }
}

impl crate::private::Sealed for I2sParallel<'_, Blocking> {}

#[instability::unstable]
impl crate::interrupt::InterruptConfigurable for I2sParallel<'_, Blocking> {
    fn set_interrupt_handler(&mut self, handler: crate::interrupt::InterruptHandler) {
        I2sParallel::set_interrupt_handler(self, handler);
    }
}

impl<'d> I2sParallel<'d, Async> {
    /// Converts the I2S parallel instance into [`Blocking`] mode.
    pub fn into_blocking(self) -> I2sParallel<'d, Blocking> {
        I2sParallel {
            instance: self.instance,
            tx_channel: self.tx_channel.into_blocking(),
            _guard: self._guard,
            _clk_guard: self._clk_guard,
        }
    }
}

impl<'d, Dm> I2sParallel<'d, Dm>
where
    Dm: DriverMode,
{
    /// Starts a DMA transfer that writes the buffer to the I2S parallel
    /// interface and returns an [`I2sParallelTransfer`] that can be used to
    /// wait for the transfer to complete.
    ///
    /// # Errors
    ///
    /// Returns a [`DmaError`] when the buffer cannot be prepared for the
    /// transfer or the transfer cannot be started. On error, the driver and
    /// the buffer are returned together with the error.
    pub fn send<BUF: DmaTxBuffer>(
        mut self,
        mut data: BUF,
    ) -> Result<I2sParallelTransfer<'d, BUF, Dm>, (DmaError, Self, BUF)> {
        self.instance.tx_reset();
        self.instance.tx_fifo_reset();
        let result = unsafe {
            self.tx_channel
                .prepare_transfer(self.instance.dma_peripheral(), &mut data)
        }
        .and_then(|_| self.tx_channel.start_transfer());
        if let Err(err) = result {
            return Err((err, self, data));
        }
        self.instance.tx_start();
        Ok(I2sParallelTransfer {
            i2s: ManuallyDrop::new(self),
            buf_view: ManuallyDrop::new(data.into_view()),
        })
    }
}

/// Represents an ongoing (or potentially finished) transfer using the I2S
/// parallel interface.
#[instability::unstable]
pub struct I2sParallelTransfer<'d, BUF, Dm>
where
    BUF: DmaTxBuffer,
    Dm: DriverMode,
{
    i2s: ManuallyDrop<I2sParallel<'d, Dm>>,
    buf_view: ManuallyDrop<BUF::View>,
}

impl<'d, BUF, Dm> I2sParallelTransfer<'d, BUF, Dm>
where
    BUF: DmaTxBuffer,
    Dm: DriverMode,
{
    /// Returns whether the transfer is complete.
    pub fn is_done(&self) -> bool {
        self.i2s.instance.is_tx_done()
    }

    /// Waits for the transfer to finish and returns the driver and the
    /// buffer.
    pub fn wait(mut self) -> (I2sParallel<'d, Dm>, BUF::Final) {
        self.i2s.instance.tx_wait_done();
        let i2s = unsafe { ManuallyDrop::take(&mut self.i2s) };
        let view = unsafe { ManuallyDrop::take(&mut self.buf_view) };
        core::mem::forget(self);
        (i2s, BUF::from_view(view))
    }

    fn stop_peripherals(&mut self) {
        self.i2s.instance.tx_stop();
        self.i2s.tx_channel.stop_transfer();
    }
}

/// Interrupt management for an ongoing transfer.
///
/// The driver itself has been consumed by [`I2sParallel::send`], so these
/// methods take `&self` and allow an interrupt handler to manage the
/// interrupt sources through the transfer object.
#[instability::unstable]
impl<'d, BUF> I2sParallelTransfer<'d, BUF, Blocking>
where
    BUF: DmaTxBuffer,
{
    /// Sets the interrupt handler for the I2S parallel peripheral.
    ///
    /// The new handler removes the old handler. This method does not turn on
    /// the interrupt sources. Use [`Self::listen`] to turn on interrupt
    /// sources.
    pub fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        self.i2s.instance.set_interrupt_handler(handler);
    }

    /// Turns on the given interrupt sources.
    pub fn listen(&self, interrupts: impl Into<EnumSet<I2sParallelInterrupt>>) {
        internal_listen(self.i2s.instance.regs(), interrupts.into(), true);
    }

    /// Turns off the given interrupt sources.
    pub fn unlisten(&self, interrupts: impl Into<EnumSet<I2sParallelInterrupt>>) {
        internal_listen(self.i2s.instance.regs(), interrupts.into(), false);
    }

    /// Clears the interrupt flags of the given interrupt sources.
    pub fn clear_interrupts(&self, interrupts: impl Into<EnumSet<I2sParallelInterrupt>>) {
        internal_clear_interrupts(self.i2s.instance.regs(), interrupts.into());
    }

    /// Returns the interrupt sources that are set.
    pub fn interrupts(&self) -> EnumSet<I2sParallelInterrupt> {
        internal_interrupts(self.i2s.instance.regs())
    }
}

impl<BUF> I2sParallelTransfer<'_, BUF, Async>
where
    BUF: DmaTxBuffer,
{
    /// Waits for [`Self::is_done`] to return true.
    ///
    /// # Errors
    ///
    /// Returns a [`DmaError`] when the transfer ends with a descriptor error.
    ///
    /// # Cancellation Safety
    ///
    /// This method is cancellation safe. Dropping the future does not stop
    /// the transfer, and the method can be called again to wait for the
    /// transfer to complete.
    pub async fn wait_for_done(&mut self) -> Result<(), DmaError> {
        DmaTxFuture::new(&mut self.i2s.tx_channel).await
    }
}

impl<BUF, Dm> Deref for I2sParallelTransfer<'_, BUF, Dm>
where
    BUF: DmaTxBuffer,
    Dm: DriverMode,
{
    type Target = BUF::View;

    fn deref(&self) -> &Self::Target {
        &self.buf_view
    }
}

impl<BUF, Dm> DerefMut for I2sParallelTransfer<'_, BUF, Dm>
where
    BUF: DmaTxBuffer,
    Dm: DriverMode,
{
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.buf_view
    }
}

impl<BUF, Dm> Drop for I2sParallelTransfer<'_, BUF, Dm>
where
    BUF: DmaTxBuffer,
    Dm: DriverMode,
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

#[doc(hidden)]
#[derive(Debug)]
pub struct I2sClockDividers {
    pub mclk_divider: u32,
    pub bclk_divider: u32,
    pub denominator: u32,
    pub numerator: u32,
}

fn calculate_clock(
    sample_rate: Rate,
    data_bits: u8,
    clock_source: crate::i2s::master::I2sClockSource,
) -> I2sClockDividers {
    // this loosely corresponds to `i2s_std_calculate_clock` and
    // `i2s_ll_tx_set_mclk` in esp-idf, adjusted for parallel interface clocking

    let sclk = crate::i2s::master::source_frequency(clock_source);
    let mclk = sample_rate.as_hz() * 2;
    let bclk_divider: u32 = if data_bits == 8 { 2 } else { 1 };

    let divider = FractionalDivider::new(
        sclk,
        mclk,
        crate::i2s::master::I2S_LL_MCLK_DIVIDER_MAX as u32,
    );

    I2sClockDividers {
        mclk_divider: divider.integer,
        bclk_divider,
        // An integer divider is described as `0 / 1`, not as `0 / 0`.
        denominator: divider.denominator.max(1),
        numerator: divider.numerator,
    }
}

#[doc(hidden)]
pub trait PrivateInstance: crate::private::Sealed {
    fn regs(&self) -> &RegisterBlock;
    fn peripheral(&self) -> crate::system::Peripheral;
    fn ws_signal(&self) -> OutputSignal;
    fn data_out_signal(&self, i: usize, bits: u8) -> OutputSignal;
    fn clock_instance(&self) -> crate::clock::ll::I2sInstance;

    fn set_interrupt_handler(&self, handler: crate::interrupt::InterruptHandler);

    fn rx_reset(&self) {
        self.regs().conf().toggle(|w, bit| w.rx_reset().bit(bit));
    }

    fn rx_dma_reset(&self) {
        self.regs().lc_conf().toggle(|w, bit| w.in_rst().bit(bit));
    }

    fn rx_fifo_reset(&self) {
        self.regs()
            .conf()
            .toggle(|w, bit| w.rx_fifo_reset().bit(bit));
    }

    fn tx_reset(&self) {
        self.regs().conf().modify(|_, w| w.tx_reset().set_bit());
        // without this delay starting a subsequent transfer will hang waiting
        // for tx_idle to clear (the transfer does not start).
        // While 20 clocks works for 80MHz cpu but 100 is needed for 240MHz!
        xtensa_lx::timer::delay(100);
        self.regs().conf().modify(|_, w| w.tx_reset().clear_bit());
    }

    fn tx_dma_reset(&self) {
        self.regs().lc_conf().toggle(|w, bit| w.out_rst().bit(bit));
    }

    fn tx_fifo_reset(&self) {
        self.regs()
            .conf()
            .toggle(|w, bit| w.tx_fifo_reset().bit(bit));
    }

    fn tx_clear_interrupts(&self) {
        self.regs().int_clr().write(|w| {
            w.out_done().clear_bit_by_one();
            w.out_total_eof().clear_bit_by_one()
        });
    }

    fn tx_start(&self) {
        // wait for data to show up in the fifo
        while self.regs().int_raw().read().tx_rempty().bit_is_clear() {
            // wait
        }

        // without this transfers are not reliable!
        xtensa_lx::timer::delay(1);

        self.regs().conf().modify(|_, w| w.tx_start().set_bit());

        while self.regs().state().read().tx_idle().bit_is_set() {
            // wait
        }
    }

    fn tx_stop(&self) {
        self.regs().conf().modify(|_, w| w.tx_start().clear_bit());
    }

    fn is_tx_done(&self) -> bool {
        self.regs().state().read().tx_idle().bit_is_set()
    }

    fn tx_wait_done(&self) {
        while self.regs().state().read().tx_idle().bit_is_clear() {
            // wait
        }

        self.regs().conf().modify(|_, w| w.tx_start().clear_bit());
        self.regs().int_clr().write(|w| {
            w.out_done().clear_bit_by_one();
            w.out_total_eof().clear_bit_by_one()
        });
    }

    fn set_clock(
        &self,
        clock_settings: I2sClockDividers,
        clock_source: crate::i2s::master::I2sClockSource,
    ) {
        crate::clock::ll::ClockTree::with(|clocks| {
            let config = crate::clock::ll::I2sMclkConfig::new(
                clock_source,
                clock_settings.mclk_divider,
                clock_settings.denominator,
                clock_settings.numerator,
            );
            self.clock_instance().configure_mclk(clocks, config);
        });

        self.regs().sample_rate_conf().modify(|_, w| unsafe {
            w.tx_bck_div_num().bits(clock_settings.bclk_divider as u8);
            w.rx_bck_div_num().bits(clock_settings.bclk_divider as u8)
        });
    }

    fn setup(&self, frequency: Rate, bits: u8) {
        let clock_source = crate::i2s::master::I2sClockSource::default();
        self.set_clock(calculate_clock(frequency, bits, clock_source), clock_source);

        // Initialize I2S dev
        self.rx_reset();
        self.tx_reset();
        self.rx_fifo_reset();
        self.tx_fifo_reset();
        self.rx_dma_reset();
        self.tx_dma_reset();

        // clear all bits and enable lcd mode
        self.regs().conf2().write(|w| {
            // 8 bit mode needs this or it updates on half clocks!
            w.lcd_tx_wrx2_en().bit(bits == 8);
            w.lcd_en().set_bit()
        });

        self.regs().sample_rate_conf().modify(|_, w| unsafe {
            w.rx_bits_mod().bits(bits);
            w.tx_bits_mod().bits(bits)
        });

        self.regs().fifo_conf().write(|w| unsafe {
            w.rx_fifo_mod_force_en().set_bit();
            w.tx_fifo_mod_force_en().set_bit();
            w.rx_fifo_mod().bits(1);
            w.tx_fifo_mod().bits(1);
            w.rx_data_num().bits(32);
            w.tx_data_num().bits(32);
            w.dscr_en().set_bit()
        });

        self.regs().conf1().write(|w| {
            w.tx_stop_en().set_bit();
            w.rx_pcm_bypass().set_bit();
            w.tx_pcm_bypass().set_bit()
        });

        self.regs().conf_chan().write(|w| unsafe {
            w.rx_chan_mod().bits(1);
            w.tx_chan_mod().bits(1)
        });

        self.regs().conf().modify(|_, w| {
            w.rx_mono().set_bit();
            w.tx_mono().set_bit();
            w.rx_right_first().set_bit();
            w.tx_right_first().set_bit()
        });
        self.regs().timing().reset();

        self.regs().pd_conf().modify(|_, w| {
            w.fifo_force_pu().set_bit();
            w.fifo_force_pd().clear_bit()
        });
    }
}

impl PrivateInstance for I2S0<'_> {
    fn regs(&self) -> &RegisterBlock {
        unsafe { &*I2S0::PTR.cast::<RegisterBlock>() }
    }

    fn peripheral(&self) -> crate::system::Peripheral {
        crate::system::Peripheral::I2s0
    }

    fn ws_signal(&self) -> OutputSignal {
        OutputSignal::I2S0O_WS
    }
    fn data_out_signal(&self, i: usize, bits: u8) -> OutputSignal {
        assert!(
            bits == 8 || bits == 16,
            "Number of bits must be 8 or 16, got {}",
            bits
        );

        // signals for 8bit and 16bit both start at an offset of 8 for I2S0
        // https://github.com/espressif/esp-idf/blob/9106c43accd9f5e75379f62f12597677213f5023/components/esp_lcd/i80/esp_lcd_panel_io_i2s.c#L701
        match i + 8 {
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
            other => panic!("Invalid I2S0 Dout pin {}", other),
        }
    }

    fn set_interrupt_handler(&self, handler: crate::interrupt::InterruptHandler) {
        self.disable_peri_interrupt_on_all_cores();
        self.bind_peri_interrupt(handler);
    }

    fn clock_instance(&self) -> crate::clock::ll::I2sInstance {
        crate::clock::ll::I2sInstance::I2s0
    }
}

impl PrivateInstance for I2S1<'_> {
    fn regs(&self) -> &RegisterBlock {
        unsafe { &*I2S1::PTR.cast::<RegisterBlock>() }
    }

    fn peripheral(&self) -> crate::system::Peripheral {
        crate::system::Peripheral::I2s1
    }

    fn ws_signal(&self) -> OutputSignal {
        OutputSignal::I2S1O_WS
    }
    fn data_out_signal(&self, i: usize, bits: u8) -> OutputSignal {
        assert!(
            bits == 8 || bits == 16,
            "Number of bits must be 8 or 16, got {}",
            bits
        );

        // signals for 8bit  start at an offset of  8 for 16bit on I2S1
        let pin_offset = if bits == 16 { 8 } else { 0 };

        match i + pin_offset {
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
            other => panic!("Invalid I2S1 Dout pin {}", other),
        }
    }

    fn set_interrupt_handler(&self, handler: crate::interrupt::InterruptHandler) {
        self.disable_peri_interrupt_on_all_cores();
        self.bind_peri_interrupt(handler);
    }

    fn clock_instance(&self) -> crate::clock::ll::I2sInstance {
        crate::clock::ll::I2sInstance::I2s1
    }
}

impl PrivateInstance for AnyI2s<'_> {
    delegate::delegate! {
        to match &self.0 {
            super::any::Inner::I2s0(i2s) => i2s,
            super::any::Inner::I2s1(i2s) => i2s,
        } {
            fn regs(&self) -> &RegisterBlock;
            fn peripheral(&self) -> crate::system::Peripheral;
            fn ws_signal(&self) -> OutputSignal;
            fn data_out_signal(&self, i: usize, bits: u8) -> OutputSignal ;
            fn set_interrupt_handler(&self, handler: crate::interrupt::InterruptHandler);
            fn clock_instance(&self) -> crate::clock::ll::I2sInstance;
        }
    }
}

/// A peripheral singleton compatible with the I2S parallel driver.
pub trait Instance: PrivateInstance + super::any::Degrade {}

impl Instance for I2S0<'_> {}
#[cfg(soc_has_i2s1)]
impl Instance for I2S1<'_> {}
impl Instance for AnyI2s<'_> {}

/// DMA channel trait for I2S peripherals.
#[diagnostic::on_unimplemented(
    message = "The DMA channel cannot be used with this I2S peripheral",
    label = "This DMA channel",
    note = "Use a channel that matches the I2S instance."
)]
pub trait I2sParallelDmaChannel<'d, S>:
    Into<I2sParallelTxErased<'d>> + crate::private::Sealed
{
}

type I2sParallelTxErased<'d> = crate::dma::I2sDmaTxChannel<'d>;

with_i2s_dma_engine! {
    ($engine:tt, $any_channel:ident) => {
        crate::macros::impl_dma_channel_trait! {
            $engine,
            any_peri = AnyI2s<'d>,
            peris = for_each_i2s,
            ($peri:path, $ch:path) => {
                impl<'d> I2sParallelDmaChannel<'d, $peri> for $ch {}
            }
        }
    };
}

// `impl_dma_channel_trait!` only covers full channels; the TX half must be
// listed explicitly because the trait erases to `I2sDmaTxChannel`.
impl<'d> I2sParallelDmaChannel<'d, AnyI2s<'d>> for crate::dma::I2sDmaTxChannel<'d> {}

for_each_i2s! {
    ($i2s:ident) => {
        impl<'d> I2sParallelDmaChannel<'d, crate::peripherals::$i2s<'d>> for crate::dma::I2sDmaTxChannel<'d> {}
    };
}

impl<'d> I2sParallel<'d, crate::Blocking> {
    /// Creates a new [`I2sParallel`] instance.
    ///
    /// # Panics
    ///
    /// Panics if the DMA channel is not compatible with the I2S instance.
    pub fn new<I: Instance + 'd>(
        i2s: I,
        channel: impl I2sParallelDmaChannel<'d, I>,
        frequency: Rate,
        pins: impl TxPins<'d>,
        clock_pin: impl PeripheralOutput<'d>,
    ) -> Self {
        Self::new_internal(i2s, channel.into(), frequency, pins, clock_pin)
    }
}
