use portable_atomic::AtomicBool;

use super::*;
use crate::asynch::AtomicWaker;

#[cfg_attr(uart_version = "1", path = "v1.rs")]
#[cfg_attr(uart_version = "2", path = "v2.rs")]
mod version;

#[inline(always)]
pub(super) fn sync_regs(register_block: &RegisterBlock) {
    version::sync_regs(register_block);
}

#[derive(Debug, EnumSetType)]
pub(super) enum TxEvent {
    Done,
    FiFoEmpty,
}

#[derive(Debug, EnumSetType)]
pub(super) enum RxEvent {
    FifoFull,
    CmdCharDetected,
    FifoOvf,
    FifoTout,
    GlitchDetected,
    FrameError,
    ParityError,
    BreakDetected,
}

pub(super) fn rx_event_check_for_error(events: EnumSet<RxEvent>) -> Result<(), RxError> {
    for event in events {
        match event {
            RxEvent::FifoOvf => return Err(RxError::FifoOverflowed),
            RxEvent::GlitchDetected => return Err(RxError::GlitchOccurred),
            RxEvent::FrameError => return Err(RxError::FrameFormatViolated),
            RxEvent::ParityError => return Err(RxError::ParityMismatch),
            RxEvent::FifoFull
            | RxEvent::CmdCharDetected
            | RxEvent::FifoTout
            | RxEvent::BreakDetected => continue,
        }
    }

    Ok(())
}

/// A future that resolves when the passed interrupt is triggered,
/// or has been triggered in the meantime (flag set in INT_RAW).
/// Upon construction the future enables the passed interrupt and when it
/// is dropped it disables the interrupt again. The future returns the event
/// that was initially passed, when it resolves.
#[must_use = "futures do nothing unless you `.await` or poll them"]
pub(super) struct UartRxFuture {
    events: EnumSet<RxEvent>,
    uart: &'static Info,
    state: &'static State,
    registered: bool,
}

impl UartRxFuture {
    pub(super) fn new(uart: impl Instance, events: impl Into<EnumSet<RxEvent>>) -> Self {
        Self {
            events: events.into(),
            uart: uart.info(),
            state: uart.state(),
            registered: false,
        }
    }
}

impl core::future::Future for UartRxFuture {
    type Output = EnumSet<RxEvent>;

    fn poll(
        mut self: core::pin::Pin<&mut Self>,
        cx: &mut core::task::Context<'_>,
    ) -> core::task::Poll<Self::Output> {
        let events = self.uart.rx_events().intersection(self.events);
        if !events.is_empty() {
            self.uart.clear_rx_events(events);
            Poll::Ready(events)
        } else {
            self.state.rx_waker.register(cx.waker());
            if !self.registered {
                self.uart.enable_listen_rx(self.events, true);
                self.registered = true;
            }
            Poll::Pending
        }
    }
}

impl Drop for UartRxFuture {
    fn drop(&mut self) {
        // Although the isr disables the interrupt that occurred directly, we need to
        // disable the other interrupts (= the ones that did not occur), as
        // soon as this future goes out of scope.
        self.uart.enable_listen_rx(self.events, false);
    }
}

#[must_use = "futures do nothing unless you `.await` or poll them"]
pub(super) struct UartTxFuture {
    events: EnumSet<TxEvent>,
    uart: &'static Info,
    state: &'static State,
    registered: bool,
}

impl UartTxFuture {
    pub(super) fn new(uart: impl Instance, events: impl Into<EnumSet<TxEvent>>) -> Self {
        Self {
            events: events.into(),
            uart: uart.info(),
            state: uart.state(),
            registered: false,
        }
    }
}

impl core::future::Future for UartTxFuture {
    type Output = ();

    fn poll(
        mut self: core::pin::Pin<&mut Self>,
        cx: &mut core::task::Context<'_>,
    ) -> core::task::Poll<Self::Output> {
        let events = self.uart.tx_events().intersection(self.events);
        if !events.is_empty() {
            self.uart.clear_tx_events(events);
            Poll::Ready(())
        } else {
            self.state.tx_waker.register(cx.waker());
            if !self.registered {
                self.uart.enable_listen_tx(self.events, true);
                self.registered = true;
            }
            Poll::Pending
        }
    }
}

impl Drop for UartTxFuture {
    fn drop(&mut self) {
        // Although the isr disables the interrupt that occurred directly, we need to
        // disable the other interrupts (= the ones that did not occur), as
        // soon as this future goes out of scope.
        self.uart.enable_listen_tx(self.events, false);
    }
}

/// Interrupt handler for all UART instances
/// Clears and disables interrupts that have occurred and have their enable
/// bit set. The fact that an interrupt has been disabled is used by the
/// futures to detect that they should indeed resolve after being woken up
#[ram]
pub(super) fn intr_handler(uart: &Info, state: &State) {
    let interrupts = uart.regs().int_st().read();
    let interrupt_bits = interrupts.bits(); // = int_raw & int_ena
    let rx_wake = interrupts.rxfifo_full().bit_is_set()
        | interrupts.rxfifo_ovf().bit_is_set()
        | interrupts.rxfifo_tout().bit_is_set()
        | interrupts.at_cmd_char_det().bit_is_set()
        | interrupts.glitch_det().bit_is_set()
        | interrupts.frm_err().bit_is_set()
        | interrupts.parity_err().bit_is_set()
        | interrupts.brk_det().bit_is_set();
    let tx_wake = interrupts.tx_done().bit_is_set() | interrupts.txfifo_empty().bit_is_set();

    uart.regs()
        .int_ena()
        .modify(|r, w| unsafe { w.bits(r.bits() & !interrupt_bits) });

    if tx_wake {
        state.tx_waker.wake();
    }
    if rx_wake {
        state.rx_waker.wake();
    }
}

/// A peripheral singleton compatible with the UART driver.
pub trait Instance: crate::private::Sealed + any::Degrade {
    #[doc(hidden)]
    /// Returns the peripheral data and state describing this UART instance.
    fn parts(&self) -> (&'static Info, &'static State);

    /// Returns the peripheral data describing this UART instance.
    #[inline(always)]
    #[doc(hidden)]
    fn info(&self) -> &'static Info {
        self.parts().0
    }

    /// Returns the peripheral state for this UART instance.
    #[inline(always)]
    #[doc(hidden)]
    fn state(&self) -> &'static State {
        self.parts().1
    }
}

/// Peripheral data describing a particular UART instance.
#[doc(hidden)]
#[non_exhaustive]
#[allow(private_interfaces, reason = "Unstable details")]
pub struct Info {
    /// Pointer to the register block for this UART instance.
    ///
    /// Use [Self::register_block] to access the register block.
    pub register_block: *const RegisterBlock,

    /// The system peripheral marker.
    pub peripheral: crate::system::Peripheral,

    /// UART clock group instance.
    pub clock_instance: clocks::UartInstance,

    /// Interrupt handler for the asynchronous operations of this UART instance.
    pub async_handler: InterruptHandler,

    /// TX pin
    pub tx_signal: OutputSignal,

    /// RX pin
    pub rx_signal: InputSignal,

    /// CTS (Clear to Send) pin
    pub cts_signal: InputSignal,

    /// RTS (Request to Send) pin
    pub rts_signal: OutputSignal,
}

/// Peripheral state for a UART instance.
#[doc(hidden)]
#[non_exhaustive]
pub struct State {
    /// Waker for the asynchronous RX operations.
    pub rx_waker: AtomicWaker,

    /// Waker for the asynchronous TX operations.
    pub tx_waker: AtomicWaker,

    /// Stores whether the RX half is configured for async operation.
    pub is_rx_async: AtomicBool,

    /// Stores whether the TX half is configured for async operation.
    pub is_tx_async: AtomicBool,
}

impl Info {
    // Currently we don't support merging adjacent FIFO memory, so the max size is
    // 128 bytes, the max threshold is 127 bytes.
    pub(super) const UART_FIFO_SIZE: u16 = property!("uart.ram_size");
    pub(super) const RX_FIFO_MAX_THRHD: u16 = Self::UART_FIFO_SIZE - 1;
    pub(super) const TX_FIFO_MAX_THRHD: u16 = Self::RX_FIFO_MAX_THRHD;

    /// Returns the register block for this UART instance.
    pub fn regs(&self) -> &RegisterBlock {
        unsafe { &*self.register_block }
    }

    /// Listen for the given interrupts
    pub(super) fn enable_listen(&self, interrupts: EnumSet<UartInterrupt>, enable: bool) {
        let reg_block = self.regs();

        reg_block.int_ena().modify(|_, w| {
            for interrupt in interrupts {
                match interrupt {
                    UartInterrupt::AtCmd => w.at_cmd_char_det().bit(enable),
                    UartInterrupt::TxDone => w.tx_done().bit(enable),
                    UartInterrupt::RxBreakDetected => w.brk_det().bit(enable),
                    UartInterrupt::RxFifoFull => w.rxfifo_full().bit(enable),
                    UartInterrupt::RxTimeout => w.rxfifo_tout().bit(enable),
                };
            }
            w
        });
    }

    pub(super) fn interrupts(&self) -> EnumSet<UartInterrupt> {
        let mut res = EnumSet::new();
        let reg_block = self.regs();

        let ints = reg_block.int_raw().read();

        if ints.at_cmd_char_det().bit_is_set() {
            res.insert(UartInterrupt::AtCmd);
        }
        if ints.tx_done().bit_is_set() {
            res.insert(UartInterrupt::TxDone);
        }
        if ints.brk_det().bit_is_set() {
            res.insert(UartInterrupt::RxBreakDetected);
        }
        if ints.rxfifo_full().bit_is_set() {
            res.insert(UartInterrupt::RxFifoFull);
        }
        if ints.rxfifo_tout().bit_is_set() {
            res.insert(UartInterrupt::RxTimeout);
        }

        res
    }

    pub(super) fn clear_interrupts(&self, interrupts: EnumSet<UartInterrupt>) {
        let reg_block = self.regs();

        reg_block.int_clr().write(|w| {
            for interrupt in interrupts {
                match interrupt {
                    UartInterrupt::AtCmd => w.at_cmd_char_det().clear_bit_by_one(),
                    UartInterrupt::TxDone => w.tx_done().clear_bit_by_one(),
                    UartInterrupt::RxBreakDetected => w.brk_det().clear_bit_by_one(),
                    UartInterrupt::RxFifoFull => w.rxfifo_full().clear_bit_by_one(),
                    UartInterrupt::RxTimeout => w.rxfifo_tout().clear_bit_by_one(),
                };
            }
            w
        });
    }

    pub(super) fn apply_config(&self, config: &Config) -> Result<(), ConfigError> {
        config.validate()?;
        self.change_baud(config)?;
        self.change_data_bits(config.data_bits);
        self.change_parity(config.parity);
        self.change_stop_bits(config.stop_bits);
        self.change_flow_control(config.sw_flow_ctrl, config.hw_flow_ctrl);

        // Avoid glitch interrupts.
        self.regs().int_clr().write(|w| unsafe { w.bits(u32::MAX) });

        Ok(())
    }

    pub(super) fn enable_listen_tx(&self, events: EnumSet<TxEvent>, enable: bool) {
        self.regs().int_ena().modify(|_, w| {
            for event in events {
                match event {
                    TxEvent::Done => w.tx_done().bit(enable),
                    TxEvent::FiFoEmpty => w.txfifo_empty().bit(enable),
                };
            }
            w
        });
    }

    fn tx_events(&self) -> EnumSet<TxEvent> {
        let pending_interrupts = self.regs().int_raw().read();
        let mut active_events = EnumSet::new();

        if pending_interrupts.tx_done().bit_is_set() {
            active_events |= TxEvent::Done;
        }
        if pending_interrupts.txfifo_empty().bit_is_set() {
            active_events |= TxEvent::FiFoEmpty;
        }

        active_events
    }

    fn clear_tx_events(&self, events: impl Into<EnumSet<TxEvent>>) {
        let events = events.into();
        self.regs().int_clr().write(|w| {
            for event in events {
                match event {
                    TxEvent::FiFoEmpty => w.txfifo_empty().clear_bit_by_one(),
                    TxEvent::Done => w.tx_done().clear_bit_by_one(),
                };
            }
            w
        });
    }

    pub(super) fn enable_listen_rx(&self, events: EnumSet<RxEvent>, enable: bool) {
        self.regs().int_ena().modify(|_, w| {
            for event in events {
                match event {
                    RxEvent::FifoFull => w.rxfifo_full().bit(enable),
                    RxEvent::BreakDetected => w.brk_det().bit(enable),
                    RxEvent::CmdCharDetected => w.at_cmd_char_det().bit(enable),

                    RxEvent::FifoOvf => w.rxfifo_ovf().bit(enable),
                    RxEvent::FifoTout => w.rxfifo_tout().bit(enable),
                    RxEvent::GlitchDetected => w.glitch_det().bit(enable),
                    RxEvent::FrameError => w.frm_err().bit(enable),
                    RxEvent::ParityError => w.parity_err().bit(enable),
                };
            }
            w
        });
    }

    fn rx_events(&self) -> EnumSet<RxEvent> {
        let pending_interrupts = self.regs().int_raw().read();
        let mut active_events = EnumSet::new();

        if pending_interrupts.rxfifo_full().bit_is_set() {
            active_events |= RxEvent::FifoFull;
        }
        if pending_interrupts.brk_det().bit_is_set() {
            active_events |= RxEvent::BreakDetected;
        }
        if pending_interrupts.at_cmd_char_det().bit_is_set() {
            active_events |= RxEvent::CmdCharDetected;
        }
        if pending_interrupts.rxfifo_ovf().bit_is_set() {
            active_events |= RxEvent::FifoOvf;
        }
        if pending_interrupts.rxfifo_tout().bit_is_set() {
            active_events |= RxEvent::FifoTout;
        }
        if pending_interrupts.glitch_det().bit_is_set() {
            active_events |= RxEvent::GlitchDetected;
        }
        if pending_interrupts.frm_err().bit_is_set() {
            active_events |= RxEvent::FrameError;
        }
        if pending_interrupts.parity_err().bit_is_set() {
            active_events |= RxEvent::ParityError;
        }

        active_events
    }

    fn clear_rx_events(&self, events: impl Into<EnumSet<RxEvent>>) {
        let events = events.into();
        self.regs().int_clr().write(|w| {
            for event in events {
                match event {
                    RxEvent::FifoFull => w.rxfifo_full().clear_bit_by_one(),
                    RxEvent::BreakDetected => w.brk_det().clear_bit_by_one(),
                    RxEvent::CmdCharDetected => w.at_cmd_char_det().clear_bit_by_one(),

                    RxEvent::FifoOvf => w.rxfifo_ovf().clear_bit_by_one(),
                    RxEvent::FifoTout => w.rxfifo_tout().clear_bit_by_one(),
                    RxEvent::GlitchDetected => w.glitch_det().clear_bit_by_one(),
                    RxEvent::FrameError => w.frm_err().clear_bit_by_one(),
                    RxEvent::ParityError => w.parity_err().clear_bit_by_one(),
                };
            }
            w
        });
    }

    /// Configures the RX-FIFO threshold
    ///
    /// ## Errors
    ///
    /// [`ConfigError::RxFifoThresholdNotSupported`] if the provided value is zero
    /// or exceeds [`Info::RX_FIFO_MAX_THRHD`].
    pub(super) fn set_rx_fifo_full_threshold(&self, threshold: u16) -> Result<(), ConfigError> {
        if threshold == 0 || threshold > Self::RX_FIFO_MAX_THRHD {
            return Err(ConfigError::RxFifoThresholdNotSupported);
        }

        self.regs()
            .conf1()
            .modify(|_, w| unsafe { w.rxfifo_full_thrhd().bits(threshold as _) });

        Ok(())
    }

    /// Reads the RX-FIFO threshold
    #[allow(clippy::useless_conversion)]
    pub(super) fn rx_fifo_full_threshold(&self) -> u16 {
        self.regs().conf1().read().rxfifo_full_thrhd().bits().into()
    }

    /// Configures the TX-FIFO threshold
    ///
    /// ## Errors
    ///
    /// [`ConfigError::TxFifoThresholdNotSupported`] if the provided value exceeds
    /// [`Info::TX_FIFO_MAX_THRHD`].
    pub(super) fn set_tx_fifo_empty_threshold(&self, threshold: u16) -> Result<(), ConfigError> {
        if threshold > Self::TX_FIFO_MAX_THRHD {
            return Err(ConfigError::TxFifoThresholdNotSupported);
        }

        self.regs()
            .conf1()
            .modify(|_, w| unsafe { w.txfifo_empty_thrhd().bits(threshold as _) });

        Ok(())
    }

    #[cfg(uart_has_sclk_enable)]
    pub(super) fn set_at_cmd_clock_enabled(&self, enabled: bool) {
        self.regs()
            .clk_conf()
            .modify(|_, w| w.sclk_en().bit(enabled));
    }

    #[procmacros::doc_replace(
        "rx_timeout_limit" => {
            cfg(esp32) => "- Symbol size is fixed to 8, do not pass a value > **0x7F**.",
            _ => "- The value you pass times the symbol size must be <= **0x3FF**.",
        }
    )]
    /// Configures the Receive Timeout detection setting
    ///
    /// ## Arguments
    ///
    /// `timeout` - the number of symbols ("bytes") to wait for before
    /// triggering a timeout. Pass None to disable the timeout.
    ///
    /// ## Errors
    ///
    /// [`ConfigError::TimeoutTooLong`] if the provided value exceeds
    /// the maximum value for SOC:
    /// {rx_timeout_limit}
    pub(super) fn set_rx_timeout(
        &self,
        timeout: Option<u8>,
        symbol_len: u8,
    ) -> Result<(), ConfigError> {
        version::set_rx_timeout(self, timeout, symbol_len)
    }

    pub(super) fn rx_timeout_enabled(&self) -> bool {
        version::rx_timeout_enabled(self)
    }

    pub(super) fn is_tx_idle(&self) -> bool {
        version::is_tx_idle(self)
    }

    fn sync_regs(&self) {
        sync_regs(self.regs());
    }

    fn change_baud(&self, config: &Config) -> Result<(), ConfigError> {
        ClockTree::with(|clocks| {
            let clock = self.clock_instance;

            let source_config = ClockConfig::new(
                config.clock_source,
                #[cfg(any(uart_has_sclk_divider, soc_has_pcr, esp32p4))]
                0,
            );
            let clk = clock.function_clock_config_frequency(clocks, source_config);

            // The UART baud rate clock divider is, depending on the device, either a
            // 20.4 bit, or a 12.4 bit divider.
            const FRAC_BITS: u32 = const {
                let largest_divider: u32 =
                    property!("clock_tree.uart.baud_rate_generator.fractional").1;
                ::core::assert!((largest_divider + 1).is_power_of_two());
                largest_divider.count_ones()
            };
            const FRAC_MASK: u32 = (1 << FRAC_BITS) - 1;

            // TODO: this block should only prepare the new clock config, and it should
            // be applied only after validating the resulting baud rate.
            cfg_if::cfg_if! {
                if #[cfg(any(uart_has_sclk_divider, soc_has_pcr, esp32p4))] {
                    const MAX_DIV: u32 = property!("clock_tree.uart.baud_rate_generator.integral").1;
                    let clk_div = clk.div_ceil(MAX_DIV).div_ceil(config.baudrate);
                    debug!("SCLK: {} divider: {}", clk, clk_div);

                    let conf = ClockConfig::new(config.clock_source, clk_div - 1);
                    let divider = (clk << FRAC_BITS) / (config.baudrate * clk_div);
                } else {
                    debug!("SCLK: {}", clk);
                    let conf = ClockConfig::new(config.clock_source);
                    let divider = (clk << FRAC_BITS) / config.baudrate;
                }
            }

            let divider_integer = divider >> FRAC_BITS;
            let divider_frag = divider & FRAC_MASK;
            debug!(
                "UART CLK divider: {} + {}/16",
                divider_integer, divider_frag
            );

            clock.configure_function_clock(clocks, conf);
            clock.configure_baud_rate_generator(
                clocks,
                BaudRateConfig::new(divider_frag, divider_integer),
            );

            self.sync_regs();

            #[cfg(feature = "unstable")]
            {
                let deviation_limit = match config.baudrate_tolerance {
                    BaudrateTolerance::Exact => 1, // Still allow a tiny deviation
                    BaudrateTolerance::ErrorPercent(percent) => percent as u32,
                    _ => return Ok(()),
                };

                let actual_baud = clock.baud_rate_generator_frequency();
                if actual_baud == 0 {
                    return Err(ConfigError::BaudrateNotAchievable);
                }

                let deviation = (config.baudrate.abs_diff(actual_baud) * 100) / actual_baud;
                debug!(
                    "Nominal baud: {}, actual: {}, deviation: {}%",
                    config.baudrate, actual_baud, deviation
                );

                if deviation > deviation_limit {
                    return Err(ConfigError::BaudrateNotAchievable);
                }
            }

            Ok(())
        })
    }

    fn change_data_bits(&self, data_bits: DataBits) {
        self.regs()
            .conf0()
            .modify(|_, w| unsafe { w.bit_num().bits(data_bits as u8) });
    }

    fn change_parity(&self, parity: Parity) {
        self.regs().conf0().modify(|_, w| match parity {
            Parity::None => w.parity_en().clear_bit(),
            Parity::Even => w.parity_en().set_bit().parity().clear_bit(),
            Parity::Odd => w.parity_en().set_bit().parity().set_bit(),
        });
    }

    fn change_stop_bits(&self, stop_bits: StopBits) {
        version::change_stop_bits(self, stop_bits);
    }

    fn change_flow_control(&self, sw_flow_ctrl: SwFlowControl, hw_flow_ctrl: HwFlowControl) {
        version::change_flow_control(self, sw_flow_ctrl, hw_flow_ctrl);
    }

    pub(super) fn rxfifo_reset(&self) {
        fn rxfifo_rst(reg_block: &RegisterBlock, enable: bool) {
            reg_block.conf0().modify(|_, w| w.rxfifo_rst().bit(enable));
            sync_regs(reg_block);
        }

        rxfifo_rst(self.regs(), true);
        rxfifo_rst(self.regs(), false);
    }

    pub(super) fn txfifo_reset(&self) {
        fn txfifo_rst(reg_block: &RegisterBlock, enable: bool) {
            reg_block.conf0().modify(|_, w| w.txfifo_rst().bit(enable));
            sync_regs(reg_block);
        }

        txfifo_rst(self.regs(), true);
        txfifo_rst(self.regs(), false);
    }

    pub(super) fn current_symbol_length(&self) -> u8 {
        version::current_symbol_length(self)
    }

    /// Reads one byte from the RX FIFO.
    ///
    /// If the FIFO is empty, the value of the returned byte is not specified.
    pub(super) fn read_next_from_fifo(&self) -> u8 {
        version::read_next_from_fifo(self)
    }

    #[allow(clippy::useless_conversion)]
    pub(super) fn tx_fifo_count(&self) -> u16 {
        u16::from(self.regs().status().read().txfifo_cnt().bits())
    }

    pub(super) fn write_byte(&self, byte: u8) {
        self.regs()
            .fifo()
            .write(|w| unsafe { w.rxfifo_rd_byte().bits(byte) });
    }

    pub(super) fn check_for_errors(&self) -> Result<(), RxError> {
        let errors = RxEvent::FifoOvf
            | RxEvent::FifoTout
            | RxEvent::GlitchDetected
            | RxEvent::FrameError
            | RxEvent::ParityError;
        let events = self.rx_events().intersection(errors);
        let result = rx_event_check_for_error(events);
        if result.is_err() {
            self.clear_rx_events(errors);
            if events.contains(RxEvent::FifoOvf) {
                self.rxfifo_reset();
            }
        }
        result
    }

    pub(super) fn rx_fifo_count(&self) -> u16 {
        version::rx_fifo_count(self)
    }

    pub(super) fn write(&self, data: &[u8]) -> Result<usize, TxError> {
        if data.is_empty() {
            return Ok(0);
        }

        while self.tx_fifo_count() >= Info::UART_FIFO_SIZE {}

        let space = (Info::UART_FIFO_SIZE - self.tx_fifo_count()) as usize;
        let to_write = space.min(data.len());
        for &byte in &data[..to_write] {
            self.write_byte(byte);
        }

        Ok(to_write)
    }

    pub(super) fn read(&self, buf: &mut [u8]) -> Result<usize, RxError> {
        if buf.is_empty() {
            return Ok(0);
        }

        while self.rx_fifo_count() == 0 {
            // Block until we received at least one byte
            self.check_for_errors()?;
        }

        self.read_buffered(buf)
    }

    pub(super) fn read_buffered(&self, buf: &mut [u8]) -> Result<usize, RxError> {
        // Get the count first, to avoid accidentally reading a corrupted byte received
        // after the error check.
        let to_read = (self.rx_fifo_count() as usize).min(buf.len());
        self.check_for_errors()?;

        for byte_into in buf[..to_read].iter_mut() {
            *byte_into = self.read_next_from_fifo();
        }

        // This bit is not cleared until the FIFO actually drops below the threshold.
        self.clear_rx_events(RxEvent::FifoFull);

        Ok(to_read)
    }
}

impl PartialEq for Info {
    fn eq(&self, other: &Self) -> bool {
        core::ptr::eq(self.register_block, other.register_block)
    }
}

unsafe impl Sync for Info {}

for_each_uart! {
    ($id:literal, $inst:ident, $peri:ident, $rxd:ident, $txd:ident, $cts:ident, $rts:ident) => {
        impl Instance for crate::peripherals::$inst<'_> {
            fn parts(&self) -> (&'static Info, &'static State) {
                #[handler]
                #[ram]
                pub(super) fn irq_handler() {
                    intr_handler(&PERIPHERAL, &STATE);
                }

                static STATE: State = State {
                    tx_waker: AtomicWaker::new(),
                    rx_waker: AtomicWaker::new(),
                    is_rx_async: AtomicBool::new(false),
                    is_tx_async: AtomicBool::new(false),
                };

                static PERIPHERAL: Info = Info {
                    register_block: crate::peripherals::$inst::ptr(),
                    peripheral: crate::system::Peripheral::$peri,
                    clock_instance: clocks::UartInstance::$peri,
                    async_handler: irq_handler,
                    tx_signal: OutputSignal::$txd,
                    rx_signal: InputSignal::$rxd,
                    cts_signal: InputSignal::$cts,
                    rts_signal: OutputSignal::$rts,
                };
                (&PERIPHERAL, &STATE)
            }
        }
    };
}

pub(super) struct UartClockGuard<'t> {
    uart: AnyUart<'t>,
}

impl<'t> UartClockGuard<'t> {
    pub(super) fn new(uart: AnyUart<'t>) -> Self {
        ClockTree::with(|clocks| {
            let clock = uart.info().clock_instance;

            // Apply default SCLK configuration
            let sclk_config = ClockConfig::new(
                Default::default(),
                #[cfg(any(uart_has_sclk_divider, soc_has_pcr, esp32p4))]
                0,
            );
            clock.configure_function_clock(clocks, sclk_config);
            clock.request_function_clock(clocks);
            clock.request_baud_rate_generator(clocks);
            #[cfg(soc_has_clock_node_uart_mem_clock)]
            clock.request_mem_clock(clocks);
        });

        Self { uart }
    }
}

impl Clone for UartClockGuard<'_> {
    fn clone(&self) -> Self {
        Self::new(unsafe { self.uart.clone_unchecked() })
    }
}

impl Drop for UartClockGuard<'_> {
    fn drop(&mut self) {
        ClockTree::with(|clocks| {
            let clock = self.uart.info().clock_instance;

            #[cfg(soc_has_clock_node_uart_mem_clock)]
            clock.release_mem_clock(clocks);
            clock.release_baud_rate_generator(clocks);
            clock.release_function_clock(clocks);
        });
    }
}
