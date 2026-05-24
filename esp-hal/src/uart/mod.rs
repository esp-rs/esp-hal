//! # Universal Asynchronous Receiver/Transmitter (UART)
//!
//! ## Overview
//!
//! The UART is a hardware peripheral which handles communication using serial
//! communication interfaces, such as RS232 and RS485. This peripheral provides!
//! a cheap and ubiquitous method for full- and half-duplex communication
//! between devices.
//!
//! Depending on your device, two or more UART controllers are available for
//! use, all of which can be configured and used in the same way. All UART
//! controllers are compatible with UART-enabled devices from various
//! manufacturers, and can also support Infrared Data Association (IrDA)
//! protocols.
//!
//! ## Configuration
//!
//! Each UART controller is individually configurable, and the usual setting
//! such as baud rate, data bits, parity, and stop bits can easily be
//! configured. Additionally, the receive (RX) and transmit (TX) pins need to
//! be specified.
//!
//! The UART controller can be configured to invert the polarity of the pins.
//! This is achieved by inverting the desired pins, and then constructing the
//! UART instance using the inverted pins.
//!
//! ## Usage
//!
//! The UART driver implements a number of third-party traits, with the
//! intention of making the HAL inter-compatible with various device drivers
//! from the community. This includes, but is not limited to, the [embedded-hal]
//! and [embedded-io] blocking traits, and the [embedded-hal-async] and
//! [embedded-io-async] asynchronous traits.
//!
//! In addition to the interfaces provided by these traits, native APIs are also
//! available. See the examples below for more information on how to interact
//! with this driver.
//!
//! [embedded-hal]: embedded_hal
//! [embedded-io]: embedded_io_07
//! [embedded-hal-async]: embedded_hal_async
//! [embedded-io-async]: embedded_io_async_07

crate::unstable_driver! {
    /// UHCI wrapper around UART
    #[cfg(uhci_driver_supported)]
    pub mod uhci;
}

mod compat;
mod low_level;

use core::{marker::PhantomData, sync::atomic::Ordering, task::Poll};

use embedded_hal_async::delay::DelayNs;
use enumset::{EnumSet, EnumSetType};
pub use low_level::Instance;
use low_level::{
    Info,
    RxEvent,
    State,
    TxEvent,
    UartClockGuard,
    UartRxFuture,
    UartTxFuture,
    rx_event_check_for_error,
    sync_regs,
};

use crate::{
    Async,
    Blocking,
    DriverMode,
    gpio::{
        InputConfig,
        InputSignal,
        OutputConfig,
        OutputSignal,
        PinGuard,
        Pull,
        interconnect::{PeripheralInput, PeripheralOutput},
    },
    handler,
    interrupt::InterruptHandler,
    pac::uart0::RegisterBlock,
    private::DropGuard,
    ram,
    soc::clocks::{self, ClockTree},
    system::PeripheralGuard,
};

crate::any_peripheral! {
    /// Any UART peripheral.
    pub peripheral AnyUart<'d> {
        #[cfg(soc_has_uart0)]
        Uart0(crate::peripherals::UART0<'d>),
        #[cfg(soc_has_uart1)]
        Uart1(crate::peripherals::UART1<'d>),
        #[cfg(soc_has_uart2)]
        Uart2(crate::peripherals::UART2<'d>),
        #[cfg(soc_has_uart3)]
        Uart3(crate::peripherals::UART3<'d>),
        #[cfg(soc_has_uart4)]
        Uart4(crate::peripherals::UART4<'d>),
    }
}

impl Instance for AnyUart<'_> {
    #[inline]
    fn parts(&self) -> (&'static Info, &'static State) {
        any::delegate!(self, uart => { uart.parts() })
    }
}

impl AnyUart<'_> {
    pub(super) fn bind_peri_interrupt(&self, handler: InterruptHandler) {
        any::delegate!(self, uart => { uart.bind_peri_interrupt(handler) })
    }

    pub(super) fn disable_peri_interrupt_on_all_cores(&self) {
        any::delegate!(self, uart => { uart.disable_peri_interrupt_on_all_cores() })
    }

    pub(super) fn set_interrupt_handler(&self, handler: InterruptHandler) {
        self.disable_peri_interrupt_on_all_cores();

        self.info().enable_listen(EnumSet::all(), false);
        self.info().clear_interrupts(EnumSet::all());

        self.bind_peri_interrupt(handler);
    }
}

/// UART RX Error
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum RxError {
    /// An RX FIFO overflow happened.
    ///
    /// This error occurs when RX FIFO is full and a new byte is received. The
    /// RX FIFO is then automatically reset by the driver.
    FifoOverflowed,

    /// A glitch was detected on the RX line.
    ///
    /// This error occurs when an unexpected or erroneous signal (glitch) is
    /// detected on the UART RX line, which could lead to incorrect data
    /// reception.
    GlitchOccurred,

    /// A framing error was detected on the RX line.
    ///
    /// This error occurs when the received data does not conform to the
    /// expected UART frame format.
    FrameFormatViolated,

    /// A parity error was detected on the RX line.
    ///
    /// This error occurs when the parity bit in the received data does not
    /// match the expected parity configuration.
    ParityMismatch,
}

impl core::error::Error for RxError {}

impl core::fmt::Display for RxError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            RxError::FifoOverflowed => write!(f, "The RX FIFO overflowed"),
            RxError::GlitchOccurred => write!(f, "A glitch was detected on the RX line"),
            RxError::FrameFormatViolated => {
                write!(f, "A framing error was detected on the RX line")
            }
            RxError::ParityMismatch => write!(f, "A parity error was detected on the RX line"),
        }
    }
}

/// UART TX Error
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum TxError {}

impl core::fmt::Display for TxError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "Tx error")
    }
}

impl core::error::Error for TxError {}

#[instability::unstable]
pub use crate::soc::clocks::UartFunctionClockSclk as ClockSource;
use crate::soc::clocks::{
    UartBaudRateGeneratorConfig as BaudRateConfig,
    UartFunctionClockConfig as ClockConfig,
};

/// Number of data bits
///
/// This enum represents the various configurations for the number of data
/// bits used in UART communication. The number of data bits defines the
/// length of each transmitted or received data frame.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DataBits {
    /// 5 data bits per frame.
    _5,
    /// 6 data bits per frame.
    _6,
    /// 7 data bits per frame.
    _7,
    /// 8 data bits per frame.
    #[default]
    _8,
}

/// Parity check
///
/// Parity is a form of error detection in UART communication, used to
/// ensure that the data has not been corrupted during transmission. The
/// parity bit is added to the data bits to make the number of 1-bits
/// either even or odd.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Parity {
    /// No parity bit is used.
    #[default]
    None,
    /// Even parity: the parity bit is set to make the total number of
    /// 1-bits even.
    Even,
    /// Odd parity: the parity bit is set to make the total number of 1-bits
    /// odd.
    Odd,
}

/// Number of stop bits
///
/// The stop bit(s) signal the end of a data packet in UART communication.
/// This enum defines the possible configurations for the number of stop
/// bits.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum StopBits {
    /// 1 stop bit.
    #[default]
    _1,
    /// 1.5 stop bits.
    _1p5,
    /// 2 stop bits.
    _2,
}

/// Software flow control settings.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub enum SwFlowControl {
    #[default]
    /// Disables software flow control.
    Disabled,
    /// Enables software flow control with configured parameters
    Enabled {
        /// Xon flow control byte.
        xon_char: u8,
        /// Xoff flow control byte.
        xoff_char: u8,
        /// If the software flow control is enabled and the data amount in
        /// rxfifo is less than xon_thrd, an xon_char will be sent.
        xon_threshold: u8,
        /// If the software flow control is enabled and the data amount in
        /// rxfifo is more than xoff_thrd, an xoff_char will be sent
        xoff_threshold: u8,
    },
}

/// Configuration for CTS (Clear To Send) flow control.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub enum CtsConfig {
    /// Enable CTS flow control (TX).
    Enabled,
    #[default]
    /// Disable CTS flow control (TX).
    Disabled,
}

/// Configuration for RTS (Request To Send) flow control.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub enum RtsConfig {
    /// Enable RTS flow control with a FIFO threshold (RX).
    Enabled(u8),
    #[default]
    /// Disable RTS flow control.
    Disabled,
}

/// Hardware flow control configuration.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub struct HwFlowControl {
    /// CTS configuration.
    pub cts: CtsConfig,
    /// RTS configuration.
    pub rts: RtsConfig,
}

/// Defines how strictly the requested baud rate must be met.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub enum BaudrateTolerance {
    /// Accept the closest achievable baud rate without restriction.
    #[default]
    Closest,
    /// In this setting, the deviation of only 1% from the desired baud value is
    /// tolerated.
    Exact,
    /// Allow a certain percentage of deviation.
    ErrorPercent(u8),
}

/// UART Configuration
#[derive(Debug, Clone, Copy, procmacros::BuilderLite)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Config {
    /// The baud rate (speed) of the UART communication in bits per second
    /// (bps).
    baudrate: u32,
    /// Determines how close to the desired baud rate value the driver should
    /// set the baud rate.
    #[builder_lite(unstable)]
    baudrate_tolerance: BaudrateTolerance,
    /// Number of data bits in each frame (5, 6, 7, or 8 bits).
    data_bits: DataBits,
    /// Parity setting (None, Even, or Odd).
    parity: Parity,
    /// Number of stop bits in each frame (1, 1.5, or 2 bits).
    stop_bits: StopBits,
    /// Software flow control.
    #[builder_lite(unstable)]
    sw_flow_ctrl: SwFlowControl,
    /// Hardware flow control.
    #[builder_lite(unstable)]
    hw_flow_ctrl: HwFlowControl,
    /// Clock source used by the UART peripheral.
    #[builder_lite(unstable)]
    clock_source: ClockSource,
    /// UART Receive part configuration.
    rx: RxConfig,
    /// UART Transmit part configuration.
    tx: TxConfig,
}

impl Default for Config {
    fn default() -> Config {
        Config {
            rx: RxConfig::default(),
            tx: TxConfig::default(),
            baudrate: 115_200,
            baudrate_tolerance: BaudrateTolerance::default(),
            data_bits: Default::default(),
            parity: Default::default(),
            stop_bits: Default::default(),
            sw_flow_ctrl: Default::default(),
            hw_flow_ctrl: Default::default(),
            clock_source: Default::default(),
        }
    }
}

impl Config {
    fn validate(&self) -> Result<(), ConfigError> {
        if let BaudrateTolerance::ErrorPercent(percentage) = self.baudrate_tolerance {
            assert!(percentage > 0 && percentage <= 100);
        }

        // Max supported baud rate is 5Mbaud
        if self.baudrate == 0 || self.baudrate > 5_000_000 {
            return Err(ConfigError::BaudrateNotSupported);
        }
        Ok(())
    }
}

/// UART Receive part configuration.
#[derive(Debug, Clone, Copy, procmacros::BuilderLite)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct RxConfig {
    /// Threshold level at which the RX FIFO is considered full.
    fifo_full_threshold: u16,
    /// Optional timeout value for RX operations.
    timeout: Option<u8>,
}

impl Default for RxConfig {
    fn default() -> RxConfig {
        RxConfig {
            // see <https://github.com/espressif/esp-idf/blob/8760e6d2a/components/esp_driver_uart/src/uart.c#L61>
            fifo_full_threshold: 120,
            // see <https://github.com/espressif/esp-idf/blob/8760e6d2a/components/esp_driver_uart/src/uart.c#L63>
            timeout: Some(10),
        }
    }
}

/// UART Transmit part configuration.
#[derive(Debug, Clone, Copy, procmacros::BuilderLite)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct TxConfig {
    /// Threshold level at which the TX FIFO is considered empty.
    fifo_empty_threshold: u16,
}

impl Default for TxConfig {
    fn default() -> TxConfig {
        TxConfig {
            // see <https://github.com/espressif/esp-idf/blob/8760e6d2a/components/esp_driver_uart/src/uart.c#L59>
            fifo_empty_threshold: 10,
        }
    }
}

/// Configuration for the AT-CMD detection functionality
#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug, procmacros::BuilderLite)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
#[non_exhaustive]
pub struct AtCmdConfig {
    /// Optional idle time before the AT command detection begins, in clock
    /// cycles.
    pre_idle_count: Option<u16>,
    /// Optional idle time after the AT command detection ends, in clock
    /// cycles.
    post_idle_count: Option<u16>,
    /// Optional timeout between bytes in the AT command, in clock
    /// cycles.
    gap_timeout: Option<u16>,
    /// The byte (character) that triggers the AT command detection.
    cmd_char: u8,
    /// Optional number of bytes to detect as part of the AT command.
    char_num: u8,
}

impl Default for AtCmdConfig {
    fn default() -> Self {
        Self {
            pre_idle_count: None,
            post_idle_count: None,
            gap_timeout: None,
            cmd_char: b'+',
            char_num: 1,
        }
    }
}

struct UartBuilder<'d, Dm: DriverMode> {
    uart: AnyUart<'d>,
    phantom: PhantomData<Dm>,
}

impl<'d, Dm> UartBuilder<'d, Dm>
where
    Dm: DriverMode,
{
    fn new(uart: impl Instance + 'd) -> Self {
        let uart = uart.degrade();

        // Make sure inputs are well-defined.
        // Connect RX to an idle high level.
        uart.info().rx_signal.connect_to(&crate::gpio::Level::High);
        uart.info().cts_signal.connect_to(&crate::gpio::Level::Low);

        Self {
            uart,
            phantom: PhantomData,
        }
    }

    fn init(self, config: Config) -> Result<Uart<'d, Dm>, ConfigError> {
        let rx_guard = PeripheralGuard::new(self.uart.info().peripheral);
        let tx_guard = PeripheralGuard::new(self.uart.info().peripheral);

        let peri_clock_guard = UartClockGuard::new(unsafe { self.uart.clone_unchecked() });

        let rts_pin = PinGuard::new_unconnected();
        let tx_pin = PinGuard::new_unconnected();

        let mut serial = Uart {
            rx: UartRx {
                uart: unsafe { self.uart.clone_unchecked() },
                phantom: PhantomData,
                guard: rx_guard,
                peri_clock_guard: peri_clock_guard.clone(),
            },
            tx: UartTx {
                uart: self.uart,
                phantom: PhantomData,
                guard: tx_guard,
                peri_clock_guard,
                rts_pin,
                tx_pin,
                baudrate: config.baudrate,
            },
        };
        serial.init(config)?;

        Ok(serial)
    }
}

#[procmacros::doc_replace]
/// UART (Full-duplex)
///
/// ## Example
///
/// ```rust, no_run
/// # {before_snippet}
/// use esp_hal::uart::{Config, Uart};
/// let mut uart = Uart::new(peripherals.UART0, Config::default())?
///     .with_rx(peripherals.GPIO1)
///     .with_tx(peripherals.GPIO2);
///
/// uart.write(b"Hello world!")?;
/// # {after_snippet}
/// ```
pub struct Uart<'d, Dm: DriverMode> {
    rx: UartRx<'d, Dm>,
    tx: UartTx<'d, Dm>,
}

/// UART (Transmit)
#[instability::unstable]
pub struct UartTx<'d, Dm: DriverMode> {
    uart: AnyUart<'d>,
    phantom: PhantomData<Dm>,
    guard: PeripheralGuard,
    peri_clock_guard: UartClockGuard<'d>,
    rts_pin: PinGuard,
    tx_pin: PinGuard,
    baudrate: u32,
}

/// UART (Receive)
#[instability::unstable]
pub struct UartRx<'d, Dm: DriverMode> {
    uart: AnyUart<'d>,
    phantom: PhantomData<Dm>,
    guard: PeripheralGuard,
    peri_clock_guard: UartClockGuard<'d>,
}

/// A configuration error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum ConfigError {
    /// The requested baud rate is not achievable.
    #[cfg(feature = "unstable")]
    #[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
    BaudrateNotAchievable,

    /// The requested baud rate is not supported.
    ///
    /// This error is returned if:
    ///  * the baud rate exceeds 5MBaud or is equal to zero.
    ///  * the user has specified an exact baud rate or with some percentage of deviation to the
    ///    desired value, and the driver cannot reach this speed.
    BaudrateNotSupported,

    /// The requested timeout exceeds the maximum value (
    #[cfg_attr(esp32, doc = "127")]
    #[cfg_attr(not(esp32), doc = "1023")]
    /// ).
    TimeoutTooLong,

    /// The requested RX FIFO threshold exceeds the maximum value (127 bytes).
    RxFifoThresholdNotSupported,

    /// The requested TX FIFO threshold exceeds the maximum value (127 bytes).
    TxFifoThresholdNotSupported,
}

impl core::error::Error for ConfigError {}

impl core::fmt::Display for ConfigError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            #[cfg(feature = "unstable")]
            ConfigError::BaudrateNotAchievable => {
                write!(f, "The requested baud rate is not achievable")
            }
            ConfigError::BaudrateNotSupported => {
                write!(f, "The requested baud rate is not supported")
            }
            ConfigError::TimeoutTooLong => write!(f, "The requested timeout is not supported"),
            ConfigError::RxFifoThresholdNotSupported => {
                write!(f, "The requested RX FIFO threshold is not supported")
            }
            ConfigError::TxFifoThresholdNotSupported => {
                write!(f, "The requested TX FIFO threshold is not supported")
            }
        }
    }
}

impl<'d> UartTx<'d, Blocking> {
    #[procmacros::doc_replace]
    /// Create a new UART TX instance in [`Blocking`] mode.
    ///
    /// ## Errors
    ///
    /// This function returns a [`ConfigError`] if the configuration is not
    /// supported by the hardware.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::uart::{Config, UartTx};
    /// let tx = UartTx::new(peripherals.UART0, Config::default())?.with_tx(peripherals.GPIO1);
    /// # {after_snippet}
    /// ```
    #[instability::unstable]
    pub fn new(uart: impl Instance + 'd, config: Config) -> Result<Self, ConfigError> {
        let (_, uart_tx) = UartBuilder::new(uart).init(config)?.split();

        Ok(uart_tx)
    }

    /// Reconfigures the driver to operate in [`Async`] mode.
    #[instability::unstable]
    pub fn into_async(self) -> UartTx<'d, Async> {
        if !self.uart.state().is_rx_async.load(Ordering::Acquire) {
            self.uart
                .set_interrupt_handler(self.uart.info().async_handler);
        }
        self.uart.state().is_tx_async.store(true, Ordering::Release);

        UartTx {
            uart: self.uart,
            phantom: PhantomData,
            guard: self.guard,
            peri_clock_guard: self.peri_clock_guard,
            rts_pin: self.rts_pin,
            tx_pin: self.tx_pin,
            baudrate: self.baudrate,
        }
    }
}

impl<'d> UartTx<'d, Async> {
    /// Reconfigures the driver to operate in [`Blocking`] mode.
    #[instability::unstable]
    pub fn into_blocking(self) -> UartTx<'d, Blocking> {
        self.uart
            .state()
            .is_tx_async
            .store(false, Ordering::Release);
        if !self.uart.state().is_rx_async.load(Ordering::Acquire) {
            self.uart.disable_peri_interrupt_on_all_cores();
        }

        UartTx {
            uart: self.uart,
            phantom: PhantomData,
            guard: self.guard,
            peri_clock_guard: self.peri_clock_guard,
            rts_pin: self.rts_pin,
            tx_pin: self.tx_pin,
            baudrate: self.baudrate,
        }
    }

    /// Write data into the TX buffer.
    ///
    /// This function writes the provided buffer `bytes` into the UART transmit
    /// buffer. If the buffer is full, the function waits asynchronously for
    /// space in the buffer to become available.
    ///
    /// The function returns the number of bytes written into the buffer. This
    /// may be less than the length of the buffer.
    ///
    /// Upon an error, the function returns immediately and the contents of the
    /// internal FIFO are not modified.
    ///
    /// ## Cancellation
    ///
    /// This function is cancellation safe.
    pub async fn write_async(&mut self, bytes: &[u8]) -> Result<usize, TxError> {
        // We need to loop in case the TX empty interrupt was fired but not cleared
        // before, but the FIFO itself was filled up by a previous write.
        let space = loop {
            let tx_fifo_count = self.uart.info().tx_fifo_count();
            let space = Info::UART_FIFO_SIZE - tx_fifo_count;
            if space != 0 {
                break space;
            }
            UartTxFuture::new(self.uart.reborrow(), TxEvent::FiFoEmpty).await;
        };

        let free = (space as usize).min(bytes.len());

        for &byte in &bytes[..free] {
            self.uart
                .info()
                .regs()
                .fifo()
                .write(|w| unsafe { w.rxfifo_rd_byte().bits(byte) });
        }

        Ok(free)
    }

    /// Asynchronously flushes the UART transmit buffer.
    ///
    /// This function ensures that all pending data in the transmit FIFO has
    /// been sent over the UART. If the FIFO contains data, it waits for the
    /// transmission to complete before returning.
    ///
    /// ## Cancellation
    ///
    /// This function is cancellation safe.
    pub async fn flush_async(&mut self) -> Result<(), TxError> {
        // Nothing is guaranteed to clear the Done status, so let's loop here in case Tx
        // was Done before the last write operation that pushed data into the
        // FIFO.
        while self.uart.info().tx_fifo_count() > 0 {
            UartTxFuture::new(self.uart.reborrow(), TxEvent::Done).await;
        }

        self.flush_last_byte();

        Ok(())
    }

    /// Sends a break signal for a specified duration in bit time.
    ///
    /// Duration is in bits, the time it takes to transfer one bit at the
    /// current baud rate.
    ///
    /// This function restores the original TX line state after the break signal is sent, even if
    /// the future is cancelled.
    #[instability::unstable]
    pub async fn send_break_async<D: DelayNs>(&mut self, delay: &mut D, bits: u32) {
        // Calculate total delay in microseconds
        let total_delay_us = (bits as u64 * 1_000_000) / self.baudrate as u64;
        let delay_us = (total_delay_us as u32).max(1);

        let break_guard = self.start_break();

        delay.delay_us(delay_us).await;

        core::mem::drop(break_guard);
    }
}

impl<'d, Dm> UartTx<'d, Dm>
where
    Dm: DriverMode,
{
    /// Configure RTS pin
    #[instability::unstable]
    pub fn with_rts(mut self, rts: impl PeripheralOutput<'d>) -> Self {
        let rts = rts.into();

        rts.apply_output_config(&OutputConfig::default());
        rts.set_output_enable(true);

        self.rts_pin = rts.connect_with_guard(self.uart.info().rts_signal);

        self
    }

    /// Assign the TX pin for UART instance.
    ///
    /// Sets the specified pin to push-pull output and connects it to the UART
    /// TX signal.
    ///
    /// Disconnects the previous pin that was assigned with `with_tx`.
    #[instability::unstable]
    pub fn with_tx(mut self, tx: impl PeripheralOutput<'d>) -> Self {
        let tx = tx.into();

        // Make sure we don't cause an unexpected low pulse on the pin.
        tx.set_output_high(true);
        tx.apply_output_config(&OutputConfig::default());
        tx.set_output_enable(true);

        self.tx_pin = tx.connect_with_guard(self.uart.info().tx_signal);

        self
    }

    /// Change the configuration.
    ///
    /// ## Errors
    ///
    /// This function returns a [`ConfigError`] if the configuration is not
    /// supported by the hardware.
    #[instability::unstable]
    pub fn apply_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        self.uart
            .info()
            .set_tx_fifo_empty_threshold(config.tx.fifo_empty_threshold)?;
        self.uart.info().txfifo_reset();
        Ok(())
    }

    /// Returns whether the UART buffer is ready to accept more data.
    ///
    /// If this function returns `true`, [`Self::write`] will not block.
    #[instability::unstable]
    pub fn write_ready(&self) -> bool {
        self.uart.info().tx_fifo_count() < Info::UART_FIFO_SIZE
    }

    /// Write bytes.
    ///
    /// This function writes data to the internal TX FIFO of the UART
    /// peripheral. The data is then transmitted over the UART TX line.
    ///
    /// The function returns the number of bytes written to the FIFO. This may
    /// be less than the length of the provided data. The function may only
    /// return 0 if the provided data is empty.
    ///
    /// ## Errors
    ///
    /// This function returns a [`TxError`] if an error occurred during the
    /// write operation.
    #[instability::unstable]
    pub fn write(&mut self, data: &[u8]) -> Result<usize, TxError> {
        self.uart.info().write(data)
    }

    fn write_all(&mut self, mut data: &[u8]) -> Result<(), TxError> {
        while !data.is_empty() {
            let bytes_written = self.write(data)?;
            data = &data[bytes_written..];
        }
        Ok(())
    }

    /// Flush the transmit buffer.
    ///
    /// This function blocks until all data in the TX FIFO has been
    /// transmitted.
    #[instability::unstable]
    pub fn flush(&mut self) -> Result<(), TxError> {
        while self.uart.info().tx_fifo_count() > 0 {}
        self.flush_last_byte();
        Ok(())
    }

    fn flush_last_byte(&mut self) {
        // This function handles an edge case that happens when the TX FIFO count
        // changes to 0. The FSM is in the Idle state for a short while after
        // the last byte is moved out of the FIFO. It is unclear how long this
        // takes, but 10us seems to be a good enough duration to wait, for both
        // fast and slow baud rates.
        crate::rom::ets_delay_us(10);
        while !self.is_tx_idle() {}
    }

    /// Sends a break signal for a specified duration in bit time.
    ///
    /// Duration is in bits, the time it takes to transfer one bit at the
    /// current baud rate. The delay during the break is just busy-waiting.
    #[instability::unstable]
    pub fn send_break(&mut self, bits: u32) {
        // Calculate total delay in microseconds
        let total_delay_us = (bits as u64 * 1_000_000) / self.baudrate as u64;
        let delay_us = (total_delay_us as u32).max(1);

        let break_guard = self.start_break();

        crate::rom::ets_delay_us(delay_us);

        core::mem::drop(break_guard);
    }

    fn start_break(&mut self) -> impl Drop + '_ {
        // Read the current TX inversion state
        let original_conf0 = self.uart.info().regs().conf0().read();
        let original_txd_inv = original_conf0.txd_inv().bit();

        // Invert the TX line (toggle the current state)
        self.uart
            .info()
            .regs()
            .conf0()
            .modify(|_, w| w.txd_inv().bit(!original_txd_inv));

        sync_regs(self.uart.info().regs());

        // Restore the original register state when dropped.
        DropGuard::new(self, move |this| {
            this.uart
                .info()
                .regs()
                .conf0()
                .write(|w| unsafe { w.bits(original_conf0.bits()) });
            sync_regs(this.uart.info().regs());
        })
    }

    /// Checks if the TX line is idle for this UART instance.
    ///
    /// Returns `true` if the transmit line is idle, meaning no data is
    /// currently being transmitted.
    fn is_tx_idle(&self) -> bool {
        self.uart.info().is_tx_idle()
    }

    /// Disables all TX-related interrupts for this UART instance.
    ///
    /// This function clears and disables the `transmit FIFO empty` interrupt,
    /// `transmit break done`, `transmit break idle done`, and `transmit done`
    /// interrupts.
    fn disable_tx_interrupts(&self) {
        self.regs().int_clr().write(|w| {
            w.txfifo_empty().clear_bit_by_one();
            w.tx_brk_done().clear_bit_by_one();
            w.tx_brk_idle_done().clear_bit_by_one();
            w.tx_done().clear_bit_by_one()
        });

        self.regs().int_ena().write(|w| {
            w.txfifo_empty().clear_bit();
            w.tx_brk_done().clear_bit();
            w.tx_brk_idle_done().clear_bit();
            w.tx_done().clear_bit()
        });
    }

    fn regs(&self) -> &RegisterBlock {
        self.uart.info().regs()
    }
}

impl<'d> UartRx<'d, Blocking> {
    #[procmacros::doc_replace]
    /// Create a new UART RX instance in [`Blocking`] mode.
    ///
    /// ## Errors
    ///
    /// This function returns a [`ConfigError`] if the configuration is not
    /// supported by the hardware.
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::uart::{Config, UartRx};
    /// let rx = UartRx::new(peripherals.UART0, Config::default())?.with_rx(peripherals.GPIO2);
    /// # {after_snippet}
    /// ```
    #[instability::unstable]
    pub fn new(uart: impl Instance + 'd, config: Config) -> Result<Self, ConfigError> {
        let (uart_rx, _) = UartBuilder::new(uart).init(config)?.split();

        Ok(uart_rx)
    }

    /// Waits for a break condition to be detected.
    ///
    /// This is a blocking function that will continuously check for a break condition.
    /// After detection, the break interrupt flag is automatically cleared.
    #[instability::unstable]
    pub fn wait_for_break(&mut self) {
        self.enable_break_detection();

        while !self.regs().int_raw().read().brk_det().bit_is_set() {
            // wait
        }

        self.regs()
            .int_clr()
            .write(|w| w.brk_det().clear_bit_by_one());
    }

    /// Waits for a break condition to be detected with a timeout.
    ///
    /// This is a blocking function that will check for a break condition up to
    /// the specified timeout. Returns `true` if a break was detected, `false` if
    /// the timeout elapsed. After successful detection, the break interrupt flag
    /// is automatically cleared.
    ///
    /// ## Arguments
    /// * `timeout` - Maximum time to wait for a break condition
    #[instability::unstable]
    pub fn wait_for_break_with_timeout(&mut self, timeout: crate::time::Duration) -> bool {
        self.enable_break_detection();

        let start = crate::time::Instant::now();

        while !self.regs().int_raw().read().brk_det().bit_is_set() {
            if crate::time::Instant::now() - start >= timeout {
                return false;
            }
        }

        self.regs()
            .int_clr()
            .write(|w| w.brk_det().clear_bit_by_one());
        true
    }

    /// Reconfigures the driver to operate in [`Async`] mode.
    #[instability::unstable]
    pub fn into_async(self) -> UartRx<'d, Async> {
        if !self.uart.state().is_tx_async.load(Ordering::Acquire) {
            self.uart
                .set_interrupt_handler(self.uart.info().async_handler);
        }
        self.uart.state().is_rx_async.store(true, Ordering::Release);

        UartRx {
            uart: self.uart,
            phantom: PhantomData,
            guard: self.guard,
            peri_clock_guard: self.peri_clock_guard,
        }
    }
}

impl<'d> UartRx<'d, Async> {
    /// Reconfigures the driver to operate in [`Blocking`] mode.
    #[instability::unstable]
    pub fn into_blocking(self) -> UartRx<'d, Blocking> {
        self.uart
            .state()
            .is_rx_async
            .store(false, Ordering::Release);
        if !self.uart.state().is_tx_async.load(Ordering::Acquire) {
            self.uart.disable_peri_interrupt_on_all_cores();
        }

        UartRx {
            uart: self.uart,
            phantom: PhantomData,
            guard: self.guard,
            peri_clock_guard: self.peri_clock_guard,
        }
    }

    async fn wait_for_buffered_data(
        &mut self,
        minimum: usize,
        max_threshold: usize,
        listen_for_timeout: bool,
    ) -> Result<(), RxError> {
        let current_threshold = self.uart.info().rx_fifo_full_threshold();

        // User preference takes priority.
        let max_threshold = max_threshold.min(current_threshold as usize) as u16;
        let minimum = minimum.min(Info::RX_FIFO_MAX_THRHD as usize) as u16;

        // The effective threshold must be >= minimum. We ensure this by lowering the minimum number
        // of returnable bytes.
        let minimum = minimum.min(max_threshold);

        // loop to prevent returning 0 bytes
        while self.uart.info().rx_fifo_count() < minimum {
            // We're ignoring the user configuration here to ensure that this is not waiting
            // for more data than the buffer. We'll restore the original value after the
            // future resolved.
            let info = self.uart.info();
            unwrap!(info.set_rx_fifo_full_threshold(max_threshold));
            let _guard = DropGuard::new((), |_| {
                unwrap!(info.set_rx_fifo_full_threshold(current_threshold));
            });

            // Wait for space or event
            let mut events = RxEvent::FifoFull
                | RxEvent::FifoOvf
                | RxEvent::FrameError
                | RxEvent::GlitchDetected
                | RxEvent::ParityError;

            if self.regs().at_cmd_char().read().char_num().bits() > 0 {
                events |= RxEvent::CmdCharDetected;
            }

            if listen_for_timeout && self.uart.info().rx_timeout_enabled() {
                events |= RxEvent::FifoTout;
            }

            let events = UartRxFuture::new(self.uart.reborrow(), events).await;

            let result = rx_event_check_for_error(events);
            if let Err(error) = result {
                if error == RxError::FifoOverflowed {
                    self.uart.info().rxfifo_reset();
                }
                return Err(error);
            }
        }

        Ok(())
    }

    /// Read data asynchronously.
    ///
    /// This function reads data from the UART receive buffer into the
    /// provided buffer. If the buffer is empty, the function waits
    /// asynchronously for data to become available, or for an error to occur.
    ///
    /// The function returns the number of bytes read into the buffer. This may
    /// be less than the length of the buffer.
    ///
    /// Note that this function may ignore the `rx_fifo_full_threshold` setting
    /// to ensure that it does not wait for more data than the buffer can hold.
    ///
    /// Upon an error, the function returns immediately and the contents of the
    /// internal FIFO are not modified.
    ///
    /// ## Cancellation
    ///
    /// This function is cancellation safe.
    pub async fn read_async(&mut self, buf: &mut [u8]) -> Result<usize, RxError> {
        if buf.is_empty() {
            return Ok(0);
        }

        self.wait_for_buffered_data(1, buf.len(), true).await?;

        self.read_buffered(buf)
    }

    /// Fill buffer asynchronously.
    ///
    /// This function reads data into the provided buffer. If the internal FIFO
    /// does not contain enough data, the function waits asynchronously for data
    /// to become available, or for an error to occur.
    ///
    /// Note that this function may ignore the `rx_fifo_full_threshold` setting
    /// to ensure that it does not wait for more data than the buffer can hold.
    ///
    /// ## Cancellation
    ///
    /// This function is **not** cancellation safe. If the future is dropped
    /// before it resolves, or if an error occurs during the read operation,
    /// previously read data may be lost.
    pub async fn read_exact_async(&mut self, mut buf: &mut [u8]) -> Result<(), RxError> {
        if buf.is_empty() {
            return Ok(());
        }

        // Drain the buffer first, there's no point in waiting for data we've already received.
        let read = self.uart.info().read_buffered(buf)?;
        buf = &mut buf[read..];

        while !buf.is_empty() {
            // No point in listening for timeouts, as we're waiting for an exact amount of
            // data. On ESP32 and S2, the timeout interrupt can't be cleared unless the FIFO
            // is empty, so listening could cause an infinite loop here.
            self.wait_for_buffered_data(buf.len(), buf.len(), false)
                .await?;

            let read = self.uart.info().read_buffered(buf)?;
            buf = &mut buf[read..];
        }

        Ok(())
    }

    /// Waits for a break condition to be detected asynchronously.
    ///
    /// This is an async function that will await until a break condition is
    /// detected on the RX line. After detection, the break interrupt flag is
    /// automatically cleared.
    #[instability::unstable]
    pub async fn wait_for_break_async(&mut self) {
        UartRxFuture::new(self.uart.reborrow(), RxEvent::BreakDetected).await;
        self.regs()
            .int_clr()
            .write(|w| w.brk_det().clear_bit_by_one());
    }
}

impl<'d, Dm> UartRx<'d, Dm>
where
    Dm: DriverMode,
{
    fn regs(&self) -> &RegisterBlock {
        self.uart.info().regs()
    }

    /// Assign the CTS pin for UART instance.
    ///
    /// Sets the specified pin to input and connects it to the UART CTS signal.
    #[instability::unstable]
    pub fn with_cts(self, cts: impl PeripheralInput<'d>) -> Self {
        let cts = cts.into();

        cts.apply_input_config(&InputConfig::default());
        cts.set_input_enable(true);

        self.uart.info().cts_signal.connect_to(&cts);

        self
    }

    /// Assign the RX pin for UART instance.
    ///
    /// Sets the specified pin to input and connects it to the UART RX signal.
    ///
    /// Note: when you listen for the output of the UART peripheral, you should
    /// configure the driver side (i.e. the TX pin), or ensure that the line is
    /// initially high, to avoid receiving a non-data byte caused by an
    /// initial low signal level.
    #[instability::unstable]
    pub fn with_rx(self, rx: impl PeripheralInput<'d>) -> Self {
        let rx = rx.into();

        rx.apply_input_config(&InputConfig::default().with_pull(Pull::Up));
        rx.set_input_enable(true);

        self.uart.info().rx_signal.connect_to(&rx);

        self
    }

    /// Enable break detection.
    ///
    /// This must be called before any breaks are expected to be received.
    /// Break detection is enabled automatically by [`Self::wait_for_break`]
    /// and [`Self::wait_for_break_with_timeout`], but calling this method
    /// explicitly ensures that breaks occurring before the first wait call
    /// will be reliably detected.
    #[instability::unstable]
    pub fn enable_break_detection(&mut self) {
        self.uart
            .info()
            .enable_listen_rx(RxEvent::BreakDetected.into(), true);

        sync_regs(self.regs());
    }

    /// Change the configuration.
    ///
    /// ## Errors
    ///
    /// This function returns a [`ConfigError`] if the configuration is not
    /// supported by the hardware.
    #[instability::unstable]
    pub fn apply_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        self.uart
            .info()
            .set_rx_fifo_full_threshold(config.rx.fifo_full_threshold)?;
        self.uart
            .info()
            .set_rx_timeout(config.rx.timeout, self.uart.info().current_symbol_length())?;

        self.uart.info().rxfifo_reset();
        Ok(())
    }

    /// Reads and clears errors set by received data.
    ///
    /// If a FIFO overflow is detected, the RX FIFO is reset.
    #[instability::unstable]
    pub fn check_for_errors(&mut self) -> Result<(), RxError> {
        self.uart.info().check_for_errors()
    }

    /// Returns whether the UART buffer has data.
    ///
    /// If this function returns `true`, [`Self::read`] will not block.
    #[instability::unstable]
    pub fn read_ready(&self) -> bool {
        self.uart.info().rx_fifo_count() > 0
    }

    /// Read bytes.
    ///
    /// The UART hardware continuously receives bytes and stores them in the RX
    /// FIFO. This function reads the bytes from the RX FIFO and returns
    /// them in the provided buffer. If the hardware buffer is empty, this
    /// function will block until data is available. The [`Self::read_ready`]
    /// function can be used to check if data is available without blocking.
    ///
    /// The function returns the number of bytes read into the buffer. This may
    /// be less than the length of the buffer. This function only returns 0
    /// if the provided buffer is empty.
    ///
    /// ## Errors
    ///
    /// This function returns an [`RxError`] if an error occurred since the last
    /// call to [`Self::check_for_errors`], [`Self::read_buffered`], or this
    /// function.
    ///
    /// If the error occurred before this function was called, the contents of
    /// the FIFO are not modified.
    #[instability::unstable]
    pub fn read(&mut self, buf: &mut [u8]) -> Result<usize, RxError> {
        self.uart.info().read(buf)
    }

    /// Read already received bytes.
    ///
    /// This function reads the already received bytes from the FIFO into the
    /// provided buffer. The function does not wait for the FIFO to actually
    /// contain any bytes.
    ///
    /// The function returns the number of bytes read into the buffer. This may
    /// be less than the length of the buffer, and it may also be 0.
    ///
    /// ## Errors
    ///
    /// This function returns an [`RxError`] if an error occurred since the last
    /// call to [`Self::check_for_errors`], [`Self::read`], or this
    /// function.
    ///
    /// If the error occurred before this function was called, the contents of
    /// the FIFO are not modified.
    #[instability::unstable]
    pub fn read_buffered(&mut self, buf: &mut [u8]) -> Result<usize, RxError> {
        self.uart.info().read_buffered(buf)
    }

    /// Disables all RX-related interrupts for this UART instance.
    ///
    /// This function clears and disables the `receive FIFO full` interrupt,
    /// `receive FIFO overflow`, `receive FIFO timeout`, and `AT command
    /// byte detection` interrupts.
    fn disable_rx_interrupts(&self) {
        self.regs().int_clr().write(|w| {
            w.rxfifo_full().clear_bit_by_one();
            w.rxfifo_ovf().clear_bit_by_one();
            w.rxfifo_tout().clear_bit_by_one();
            w.at_cmd_char_det().clear_bit_by_one()
        });

        self.regs().int_ena().write(|w| {
            w.rxfifo_full().clear_bit();
            w.rxfifo_ovf().clear_bit();
            w.rxfifo_tout().clear_bit();
            w.at_cmd_char_det().clear_bit()
        });
    }
}

impl<'d> Uart<'d, Blocking> {
    #[procmacros::doc_replace]
    /// Create a new UART instance in [`Blocking`] mode.
    ///
    /// ## Errors
    ///
    /// This function returns a [`ConfigError`] if the configuration is not
    /// supported by the hardware.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::uart::{Config, Uart};
    /// let mut uart = Uart::new(peripherals.UART0, Config::default())?
    ///     .with_rx(peripherals.GPIO1)
    ///     .with_tx(peripherals.GPIO2);
    /// # {after_snippet}
    /// ```
    pub fn new(uart: impl Instance + 'd, config: Config) -> Result<Self, ConfigError> {
        UartBuilder::new(uart).init(config)
    }

    /// Reconfigures the driver to operate in [`Async`] mode.
    ///
    /// See the [`Async`] documentation for an example on how to use this
    /// method.
    pub fn into_async(self) -> Uart<'d, Async> {
        Uart {
            rx: self.rx.into_async(),
            tx: self.tx.into_async(),
        }
    }

    #[cfg_attr(
        not(multi_core),
        doc = "Registers an interrupt handler for the peripheral."
    )]
    #[cfg_attr(
        multi_core,
        doc = "Registers an interrupt handler for the peripheral on the current core."
    )]
    #[doc = ""]
    /// Note that this will replace any previously registered interrupt
    /// handlers.
    ///
    /// You can restore the default/unhandled interrupt handler by using
    /// [crate::interrupt::DEFAULT_INTERRUPT_HANDLER]
    #[instability::unstable]
    pub fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        // `self.tx.uart` and `self.rx.uart` are the same
        self.tx.uart.set_interrupt_handler(handler);
    }

    #[procmacros::doc_replace]
    /// Listen for the given interrupts
    ///
    /// ## Example
    ///
    /// **Note**: In practice a proper serial terminal should be used
    /// to connect to the board (espflash won't work)
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::{
    ///     delay::Delay,
    ///     uart::{AtCmdConfig, Config, RxConfig, Uart, UartInterrupt},
    /// };
    /// # let delay = Delay::new();
    /// # let config = Config::default().with_rx(
    /// #    RxConfig::default().with_fifo_full_threshold(30)
    /// # );
    /// # let mut uart = Uart::new(
    /// #    peripherals.UART0,
    /// #    config)?;
    /// uart.set_interrupt_handler(interrupt_handler);
    ///
    /// critical_section::with(|cs| {
    ///     uart.set_at_cmd(AtCmdConfig::default().with_cmd_char(b'#'));
    ///     uart.listen(UartInterrupt::AtCmd | UartInterrupt::RxFifoFull);
    ///
    ///     SERIAL.borrow_ref_mut(cs).replace(uart);
    /// });
    ///
    /// loop {
    ///     println!("Send `#` character or >=30 characters");
    ///     delay.delay(Duration::from_secs(1));
    /// }
    /// # }
    ///
    /// use core::cell::RefCell;
    ///
    /// use critical_section::Mutex;
    /// use esp_hal::uart::Uart;
    /// static SERIAL: Mutex<RefCell<Option<Uart<esp_hal::Blocking>>>> = Mutex::new(RefCell::new(None));
    ///
    /// use core::fmt::Write;
    ///
    /// use esp_hal::uart::UartInterrupt;
    /// #[esp_hal::handler]
    /// fn interrupt_handler() {
    ///     critical_section::with(|cs| {
    ///         let mut serial = SERIAL.borrow_ref_mut(cs);
    ///         if let Some(serial) = serial.as_mut() {
    ///             let mut buf = [0u8; 64];
    ///             if let Ok(cnt) = serial.read_buffered(&mut buf) {
    ///                 println!("Read {} bytes", cnt);
    ///             }
    ///
    ///             let pending_interrupts = serial.interrupts();
    ///             println!(
    ///                 "Interrupt AT-CMD: {} RX-FIFO-FULL: {}",
    ///                 pending_interrupts.contains(UartInterrupt::AtCmd),
    ///                 pending_interrupts.contains(UartInterrupt::RxFifoFull),
    ///             );
    ///
    ///             serial.clear_interrupts(UartInterrupt::AtCmd | UartInterrupt::RxFifoFull);
    ///         }
    ///     });
    /// }
    /// ```
    #[instability::unstable]
    pub fn listen(&mut self, interrupts: impl Into<EnumSet<UartInterrupt>>) {
        self.tx.uart.info().enable_listen(interrupts.into(), true)
    }

    /// Unlisten the given interrupts
    #[instability::unstable]
    pub fn unlisten(&mut self, interrupts: impl Into<EnumSet<UartInterrupt>>) {
        self.tx.uart.info().enable_listen(interrupts.into(), false)
    }

    /// Gets asserted interrupts
    #[instability::unstable]
    pub fn interrupts(&mut self) -> EnumSet<UartInterrupt> {
        self.tx.uart.info().interrupts()
    }

    /// Resets asserted interrupts
    #[instability::unstable]
    pub fn clear_interrupts(&mut self, interrupts: EnumSet<UartInterrupt>) {
        self.tx.uart.info().clear_interrupts(interrupts)
    }

    /// Waits for a break condition to be detected.
    ///
    /// This is a blocking function that will continuously check for a break condition.
    /// After detection, the break interrupt flag is automatically cleared.
    #[instability::unstable]
    pub fn wait_for_break(&mut self) {
        self.rx.wait_for_break()
    }

    /// Waits for a break condition to be detected with a timeout.
    ///
    /// This is a blocking function that will check for a break condition up to
    /// the specified timeout. Returns `true` if a break was detected, `false` if
    /// the timeout elapsed. After successful detection, the break interrupt flag
    /// is automatically cleared.
    ///
    /// ## Arguments
    /// * `timeout` - Maximum time to wait for a break condition
    #[instability::unstable]
    pub fn wait_for_break_with_timeout(&mut self, timeout: crate::time::Duration) -> bool {
        self.rx.wait_for_break_with_timeout(timeout)
    }
}

impl<'d> Uart<'d, Async> {
    /// Reconfigures the driver to operate in [`Blocking`] mode.
    ///
    /// See the [`Blocking`] documentation for an example on how to use this
    /// method.
    pub fn into_blocking(self) -> Uart<'d, Blocking> {
        Uart {
            rx: self.rx.into_blocking(),
            tx: self.tx.into_blocking(),
        }
    }

    #[procmacros::doc_replace]
    /// Write data into the TX buffer.
    ///
    /// This function writes the provided buffer `bytes` into the UART transmit
    /// buffer. If the buffer is full, the function waits asynchronously for
    /// space in the buffer to become available.
    ///
    /// The function returns the number of bytes written into the buffer. This
    /// may be less than the length of the buffer.
    ///
    /// Upon an error, the function returns immediately and the contents of the
    /// internal FIFO are not modified.
    ///
    /// ## Cancellation
    ///
    /// This function is cancellation safe.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::uart::{Config, Uart};
    /// let mut uart = Uart::new(peripherals.UART0, Config::default())?
    ///     .with_rx(peripherals.GPIO1)
    ///     .with_tx(peripherals.GPIO2)
    ///     .into_async();
    ///
    /// const MESSAGE: &[u8] = b"Hello, world!";
    /// uart.write_async(&MESSAGE).await?;
    /// # {after_snippet}
    /// ```
    pub async fn write_async(&mut self, words: &[u8]) -> Result<usize, TxError> {
        self.tx.write_async(words).await
    }

    #[procmacros::doc_replace]
    /// Asynchronously flushes the UART transmit buffer.
    ///
    /// This function ensures that all pending data in the transmit FIFO has
    /// been sent over the UART. If the FIFO contains data, it waits for the
    /// transmission to complete before returning.
    ///
    /// ## Cancellation
    ///
    /// This function is cancellation safe.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::uart::{Config, Uart};
    /// let mut uart = Uart::new(peripherals.UART0, Config::default())?
    ///     .with_rx(peripherals.GPIO1)
    ///     .with_tx(peripherals.GPIO2)
    ///     .into_async();
    ///
    /// const MESSAGE: &[u8] = b"Hello, world!";
    /// uart.write_async(&MESSAGE).await?;
    /// uart.flush_async().await?;
    /// # {after_snippet}
    /// ```
    pub async fn flush_async(&mut self) -> Result<(), TxError> {
        self.tx.flush_async().await
    }

    #[procmacros::doc_replace]
    /// Read data asynchronously.
    ///
    /// This function reads data from the UART receive buffer into the
    /// provided buffer. If the buffer is empty, the function waits
    /// asynchronously for data to become available, or for an error to occur.
    ///
    /// The function returns the number of bytes read into the buffer. This may
    /// be less than the length of the buffer.
    ///
    /// Note that this function may ignore the `rx_fifo_full_threshold` setting
    /// to ensure that it does not wait for more data than the buffer can hold.
    ///
    /// Upon an error, the function returns immediately and the contents of the
    /// internal FIFO are not modified.
    ///
    /// ## Cancellation
    ///
    /// This function is cancellation safe.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::uart::{Config, Uart};
    /// let mut uart = Uart::new(peripherals.UART0, Config::default())?
    ///     .with_rx(peripherals.GPIO1)
    ///     .with_tx(peripherals.GPIO2)
    ///     .into_async();
    ///
    /// const MESSAGE: &[u8] = b"Hello, world!";
    /// uart.write_async(&MESSAGE).await?;
    /// uart.flush_async().await?;
    ///
    /// let mut buf = [0u8; MESSAGE.len()];
    /// uart.read_async(&mut buf[..]).await.unwrap();
    /// # {after_snippet}
    /// ```
    pub async fn read_async(&mut self, buf: &mut [u8]) -> Result<usize, RxError> {
        self.rx.read_async(buf).await
    }

    /// Fill buffer asynchronously.
    ///
    /// This function reads data from the UART receive buffer into the
    /// provided buffer. If the buffer is empty, the function waits
    /// asynchronously for data to become available, or for an error to occur.
    ///
    /// Note that this function may ignore the `rx_fifo_full_threshold` setting
    /// to ensure that it does not wait for more data than the buffer can hold.
    ///
    /// ## Cancellation
    ///
    /// This function is **not** cancellation safe. If the future is dropped
    /// before it resolves, or if an error occurs during the read operation,
    /// previously read data may be lost.
    #[instability::unstable]
    pub async fn read_exact_async(&mut self, buf: &mut [u8]) -> Result<(), RxError> {
        self.rx.read_exact_async(buf).await
    }

    /// Waits for a break condition to be detected asynchronously.
    ///
    /// This is an async function that will await until a break condition is
    /// detected on the RX line. After detection, the break interrupt flag is
    /// automatically cleared.
    #[instability::unstable]
    pub async fn wait_for_break_async(&mut self) {
        self.rx.wait_for_break_async().await
    }

    /// Sends a break signal for a specified duration in bit time.
    ///
    /// Duration is in bits, the time it takes to transfer one bit at the
    /// current baud rate.
    ///
    /// This function restores the original TX line state after the break signal is sent, even if
    /// the future is cancelled.
    #[instability::unstable]
    pub async fn send_break_async<D: DelayNs>(&mut self, delay: &mut D, bits: u32) {
        self.tx.send_break_async(delay, bits).await
    }
}

/// List of exposed UART events.
#[derive(Debug, EnumSetType)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
#[instability::unstable]
pub enum UartInterrupt {
    /// Indicates that the received has detected the configured
    /// [`Uart::set_at_cmd`] byte.
    AtCmd,

    /// The transmitter has finished sending out all data from the FIFO.
    TxDone,

    /// Break condition has been detected.
    /// Triggered when the receiver detects a NULL character (i.e. logic 0 for
    /// one NULL character transmission) after stop bits.
    RxBreakDetected,

    /// The receiver has received more data than what
    /// [`RxConfig::fifo_full_threshold`] specifies.
    RxFifoFull,

    /// The receiver has not received any data for the time
    /// [`RxConfig::with_timeout`] specifies.
    RxTimeout,
}

impl<'d, Dm> Uart<'d, Dm>
where
    Dm: DriverMode,
{
    #[procmacros::doc_replace]
    /// Assign the RX pin for UART instance.
    ///
    /// Sets the specified pin to input and connects it to the UART RX signal.
    ///
    /// Note: when you listen for the output of the UART peripheral, you should
    /// configure the driver side (i.e. the TX pin), or ensure that the line is
    /// initially high, to avoid receiving a non-data byte caused by an
    /// initial low signal level.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::uart::{Config, Uart};
    /// let uart = Uart::new(peripherals.UART0, Config::default())?.with_rx(peripherals.GPIO1);
    ///
    /// # {after_snippet}
    /// ```
    pub fn with_rx(mut self, rx: impl PeripheralInput<'d>) -> Self {
        self.rx = self.rx.with_rx(rx);
        self
    }

    #[procmacros::doc_replace]
    /// Assign the TX pin for UART instance.
    ///
    /// Sets the specified pin to push-pull output and connects it to the UART
    /// TX signal.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::uart::{Config, Uart};
    /// let uart = Uart::new(peripherals.UART0, Config::default())?.with_tx(peripherals.GPIO2);
    ///
    /// # {after_snippet}
    /// ```
    pub fn with_tx(mut self, tx: impl PeripheralOutput<'d>) -> Self {
        self.tx = self.tx.with_tx(tx);
        self
    }

    #[procmacros::doc_replace]
    /// Configure CTS pin
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::uart::{Config, Uart};
    /// let uart = Uart::new(peripherals.UART0, Config::default())?
    ///     .with_rx(peripherals.GPIO1)
    ///     .with_cts(peripherals.GPIO3);
    ///
    /// # {after_snippet}
    /// ```
    pub fn with_cts(mut self, cts: impl PeripheralInput<'d>) -> Self {
        self.rx = self.rx.with_cts(cts);
        self
    }

    #[procmacros::doc_replace]
    /// Configure RTS pin
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::uart::{Config, Uart};
    /// let uart = Uart::new(peripherals.UART0, Config::default())?
    ///     .with_tx(peripherals.GPIO2)
    ///     .with_rts(peripherals.GPIO3);
    ///
    /// # {after_snippet}
    /// ```
    pub fn with_rts(mut self, rts: impl PeripheralOutput<'d>) -> Self {
        self.tx = self.tx.with_rts(rts);
        self
    }

    fn regs(&self) -> &RegisterBlock {
        // `self.tx.uart` and `self.rx.uart` are the same
        self.tx.uart.info().regs()
    }

    #[procmacros::doc_replace]
    /// Returns whether the UART TX buffer is ready to accept more data.
    ///
    /// If this function returns `true`, [`Self::write`] and [`Self::write_async`]
    /// will not block. Otherwise, the functions will not return until the buffer is
    /// ready.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::uart::{Config, Uart};
    /// let mut uart = Uart::new(peripherals.UART0, Config::default())?;
    ///
    /// if uart.write_ready() {
    ///     // Because write_ready has returned true, the following call will immediately
    ///     // copy some bytes into the FIFO and return a non-zero value.
    ///     let written = uart.write(b"Hello")?;
    ///     // ... handle written bytes
    /// } else {
    ///     // Calling write would have blocked, but here we can do something useful
    ///     // instead of waiting for the buffer to become ready.
    /// }
    /// # {after_snippet}
    /// ```
    pub fn write_ready(&self) -> bool {
        self.tx.write_ready()
    }

    #[procmacros::doc_replace]
    /// Writes bytes.
    ///
    /// This function writes data to the internal TX FIFO of the UART
    /// peripheral. The data is then transmitted over the UART TX line.
    ///
    /// The function returns the number of bytes written to the FIFO. This may
    /// be less than the length of the provided data. The function may only
    /// return 0 if the provided data is empty.
    ///
    /// ## Errors
    ///
    /// This function returns a [`TxError`] if an error occurred during the
    /// write operation.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::uart::{Config, Uart};
    /// let mut uart = Uart::new(peripherals.UART0, Config::default())?;
    ///
    /// const MESSAGE: &[u8] = b"Hello, world!";
    /// uart.write(&MESSAGE)?;
    /// # {after_snippet}
    /// ```
    pub fn write(&mut self, data: &[u8]) -> Result<usize, TxError> {
        self.tx.write(data)
    }

    #[procmacros::doc_replace]
    /// Flush the transmit buffer of the UART
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::uart::{Config, Uart};
    /// let mut uart = Uart::new(peripherals.UART0, Config::default())?;
    ///
    /// const MESSAGE: &[u8] = b"Hello, world!";
    /// uart.write(&MESSAGE)?;
    /// uart.flush()?;
    /// # {after_snippet}
    /// ```
    pub fn flush(&mut self) -> Result<(), TxError> {
        self.tx.flush()
    }

    /// Sends a break signal for a specified duration
    #[instability::unstable]
    pub fn send_break(&mut self, bits: u32) {
        self.tx.send_break(bits)
    }

    #[procmacros::doc_replace]
    /// Returns whether the UART receive buffer has at least one byte of data.
    ///
    /// If this function returns `true`, [`Self::read`] and [`Self::read_async`]
    /// will not block. Otherwise, they will not return until data is available.
    ///
    /// Data that does not get stored due to an error will be lost and does not count
    /// towards the number of bytes in the receive buffer.
    // TODO: once we add support for UART_ERR_WR_MASK it needs to be documented here.
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::uart::{Config, Uart};
    /// let mut uart = Uart::new(peripherals.UART0, Config::default())?;
    ///
    /// while !uart.read_ready() {
    ///     // Do something else while waiting for data to be available.
    /// }
    ///
    /// let mut buf = [0u8; 32];
    /// uart.read(&mut buf[..])?;
    ///
    /// # {after_snippet}
    /// ```
    pub fn read_ready(&self) -> bool {
        self.rx.read_ready()
    }

    #[procmacros::doc_replace]
    /// Read received bytes.
    ///
    /// The UART hardware continuously receives bytes and stores them in the RX
    /// FIFO. This function reads the bytes from the RX FIFO and returns
    /// them in the provided buffer. If the hardware buffer is empty, this
    /// function will block until data is available. The [`Self::read_ready`]
    /// function can be used to check if data is available without blocking.
    ///
    /// The function returns the number of bytes read into the buffer. This may
    /// be less than the length of the buffer. This function only returns 0
    /// if the provided buffer is empty.
    ///
    /// ## Errors
    ///
    /// This function returns an [`RxError`] if an error occurred since the last
    /// check for errors.
    ///
    /// If the error occurred before this function was called, the contents of
    /// the FIFO are not modified.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::uart::{Config, Uart};
    /// let mut uart = Uart::new(peripherals.UART0, Config::default())?;
    ///
    /// const MESSAGE: &[u8] = b"Hello, world!";
    /// uart.write(&MESSAGE)?;
    /// uart.flush()?;
    ///
    /// let mut buf = [0u8; MESSAGE.len()];
    /// uart.read(&mut buf[..])?;
    ///
    /// # {after_snippet}
    /// ```
    pub fn read(&mut self, buf: &mut [u8]) -> Result<usize, RxError> {
        self.rx.read(buf)
    }

    #[procmacros::doc_replace]
    /// Change the configuration.
    ///
    /// ## Errors
    ///
    /// This function returns a [`ConfigError`] if the configuration is not
    /// supported by the hardware.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::uart::{Config, Uart};
    /// let mut uart = Uart::new(peripherals.UART0, Config::default())?;
    ///
    /// uart.apply_config(&Config::default().with_baudrate(19_200))?;
    /// # {after_snippet}
    /// ```
    pub fn apply_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        // Must apply the common settings first, as `rx.apply_config` reads back symbol
        // size.
        self.rx.uart.info().apply_config(config)?;

        self.rx.apply_config(config)?;
        self.tx.apply_config(config)?;
        Ok(())
    }

    #[procmacros::doc_replace]
    /// Split the UART into a transmitter and receiver
    ///
    /// This is particularly useful when having two tasks correlating to
    /// transmitting and receiving.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::uart::{Config, Uart};
    /// let mut uart = Uart::new(peripherals.UART0, Config::default())?
    ///     .with_rx(peripherals.GPIO1)
    ///     .with_tx(peripherals.GPIO2);
    ///
    /// // The UART can be split into separate Transmit and Receive components:
    /// let (mut rx, mut tx) = uart.split();
    ///
    /// // Each component can be used individually to interact with the UART:
    /// tx.write(&[42u8])?;
    /// let mut byte = [0u8; 1];
    /// rx.read(&mut byte);
    /// # {after_snippet}
    /// ```
    #[instability::unstable]
    pub fn split(self) -> (UartRx<'d, Dm>, UartTx<'d, Dm>) {
        (self.rx, self.tx)
    }

    /// Reads and clears errors set by received data.
    #[instability::unstable]
    pub fn check_for_rx_errors(&mut self) -> Result<(), RxError> {
        self.rx.check_for_errors()
    }

    /// Read already received bytes.
    ///
    /// This function reads the already received bytes from the FIFO into the
    /// provided buffer. The function does not wait for the FIFO to actually
    /// contain any bytes.
    ///
    /// The function returns the number of bytes read into the buffer. This may
    /// be less than the length of the buffer, and it may also be 0.
    ///
    /// ## Errors
    ///
    /// This function returns an [`RxError`] if an error occurred since the last
    /// check for errors.
    ///
    /// If the error occurred before this function was called, the contents of
    /// the FIFO are not modified.
    #[instability::unstable]
    pub fn read_buffered(&mut self, buf: &mut [u8]) -> Result<usize, RxError> {
        self.rx.read_buffered(buf)
    }

    /// Configures the AT-CMD detection settings
    #[instability::unstable]
    pub fn set_at_cmd(&mut self, config: AtCmdConfig) {
        #[cfg(uart_has_sclk_enable)]
        self.rx.uart.info().set_at_cmd_clock_enabled(false);

        self.regs().at_cmd_char().write(|w| unsafe {
            w.at_cmd_char().bits(config.cmd_char);
            w.char_num().bits(config.char_num)
        });

        if let Some(pre_idle_count) = config.pre_idle_count {
            self.regs()
                .at_cmd_precnt()
                .write(|w| unsafe { w.pre_idle_num().bits(pre_idle_count as _) });
        }

        if let Some(post_idle_count) = config.post_idle_count {
            self.regs()
                .at_cmd_postcnt()
                .write(|w| unsafe { w.post_idle_num().bits(post_idle_count as _) });
        }

        if let Some(gap_timeout) = config.gap_timeout {
            self.regs()
                .at_cmd_gaptout()
                .write(|w| unsafe { w.rx_gap_tout().bits(gap_timeout as _) });
        }

        #[cfg(uart_has_sclk_enable)]
        self.rx.uart.info().set_at_cmd_clock_enabled(true);

        sync_regs(self.regs());
    }

    #[inline(always)]
    fn init(&mut self, config: Config) -> Result<(), ConfigError> {
        self.rx.disable_rx_interrupts();
        self.tx.disable_tx_interrupts();

        // Reset Tx/Rx FIFOs
        self.rx.uart.info().rxfifo_reset();
        self.rx.uart.info().txfifo_reset();

        self.apply_config(&config)?;

        // Don't wait after transmissions by default,
        // so that bytes written to TX FIFO are always immediately transmitted.
        self.regs()
            .idle_conf()
            .modify(|_, w| unsafe { w.tx_idle_num().bits(0) });

        // Setting err_wr_mask stops uart from storing data when data is wrong according
        // to reference manual
        self.regs().conf0().modify(|_, w| w.err_wr_mask().set_bit());

        crate::rom::ets_delay_us(15);

        // Make sure we are starting in a "clean state" - previous operations might have
        // run into error conditions
        self.regs().int_clr().write(|w| unsafe { w.bits(u32::MAX) });

        Ok(())
    }
}

/// UART Tx or Rx Error
#[instability::unstable]
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum IoError {
    /// UART TX error
    Tx(TxError),
    /// UART RX error
    Rx(RxError),
}

#[instability::unstable]
impl core::error::Error for IoError {}

#[instability::unstable]
impl core::fmt::Display for IoError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            IoError::Tx(e) => e.fmt(f),
            IoError::Rx(e) => e.fmt(f),
        }
    }
}

#[instability::unstable]
impl From<RxError> for IoError {
    fn from(e: RxError) -> Self {
        IoError::Rx(e)
    }
}

#[instability::unstable]
impl From<TxError> for IoError {
    fn from(e: TxError) -> Self {
        IoError::Tx(e)
    }
}

/// Low-power UART
#[cfg(lp_uart_driver_supported)]
#[instability::unstable]
pub mod lp_uart {
    use crate::{
        gpio::lp_io::{LowPowerInput, LowPowerOutput},
        peripherals::{LP_AON, LP_CLKRST, LP_IO, LP_UART, LPWR},
        uart::{DataBits, Parity, StopBits},
    };

    /// LP-UART Configuration
    #[derive(Debug, Clone, Copy, procmacros::BuilderLite)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    #[non_exhaustive]
    pub struct Config {
        /// The baud rate (speed) of the UART communication in bits per second
        /// (bps).
        baudrate: u32,
        /// Number of data bits in each frame (5, 6, 7, or 8 bits).
        data_bits: DataBits,
        /// Parity setting (None, Even, or Odd).
        parity: Parity,
        /// Number of stop bits in each frame (1, 1.5, or 2 bits).
        stop_bits: StopBits,
        /// Clock source used by the UART peripheral.
        #[builder_lite(unstable)]
        clock_source: ClockSource,
    }

    impl Default for Config {
        fn default() -> Config {
            Config {
                baudrate: 115_200,
                data_bits: Default::default(),
                parity: Default::default(),
                stop_bits: Default::default(),
                clock_source: Default::default(),
            }
        }
    }

    /// LP-UART clock source
    #[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Hash)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    #[non_exhaustive]
    #[instability::unstable]
    pub enum ClockSource {
        /// RC_FAST_CLK clock source
        RcFast,

        /// XTAL_D2 clock source
        #[default]
        Xtal,
    }

    /// LP-UART driver
    pub struct LpUart {
        uart: LP_UART<'static>,
    }

    impl LpUart {
        /// Initialize the UART driver using the provided configuration
        // TODO: CTS and RTS pins
        pub fn new(
            uart: LP_UART<'static>,
            config: Config,
            _tx: LowPowerOutput<'_, 5>,
            _rx: LowPowerInput<'_, 4>,
        ) -> Self {
            // FIXME: use GPIO APIs to configure pins
            LP_AON::regs()
                .gpio_mux()
                .modify(|r, w| unsafe { w.sel().bits(r.sel().bits() | (1 << 4) | (1 << 5)) });

            LP_IO::regs()
                .gpio(4)
                .modify(|_, w| unsafe { w.mcu_sel().bits(1) });
            LP_IO::regs()
                .gpio(5)
                .modify(|_, w| unsafe { w.mcu_sel().bits(1) });

            let mut me = Self { uart };
            let uart = me.uart.register_block();

            // Set UART mode - do nothing for LP

            // Disable UART parity
            // 8-bit world
            // 1-bit stop bit
            uart.conf0().modify(|_, w| unsafe {
                w.parity().clear_bit();
                w.parity_en().clear_bit();
                w.bit_num().bits(0x3);
                w.stop_bit_num().bits(0x1)
            });
            // Set tx idle
            uart.idle_conf()
                .modify(|_, w| unsafe { w.tx_idle_num().bits(0) });
            // Disable hw-flow control
            uart.hwfc_conf().modify(|_, w| w.rx_flow_en().clear_bit());

            // Get source clock frequency
            // default == SOC_MOD_CLK_RTC_FAST == 2

            // LPWR.lpperi.lp_uart_clk_sel = 0;
            LPWR::regs()
                .lpperi()
                .modify(|_, w| w.lp_uart_clk_sel().clear_bit());

            // Override protocol parameters from the configuration
            // uart_hal_set_baudrate(&hal, cfg->uart_proto_cfg.baud_rate, sclk_freq);
            me.change_baud_internal(&config);
            // uart_hal_set_parity(&hal, cfg->uart_proto_cfg.parity);
            me.change_parity(config.parity);
            // uart_hal_set_data_bit_num(&hal, cfg->uart_proto_cfg.data_bits);
            me.change_data_bits(config.data_bits);
            // uart_hal_set_stop_bits(&hal, cfg->uart_proto_cfg.stop_bits);
            me.change_stop_bits(config.stop_bits);
            // uart_hal_set_tx_idle_num(&hal, LP_UART_TX_IDLE_NUM_DEFAULT);
            me.change_tx_idle(0); // LP_UART_TX_IDLE_NUM_DEFAULT == 0

            // Reset Tx/Rx FIFOs
            me.rxfifo_reset();
            me.txfifo_reset();

            me
        }

        fn rxfifo_reset(&mut self) {
            self.uart
                .register_block()
                .conf0()
                .modify(|_, w| w.rxfifo_rst().set_bit());
            self.update();

            self.uart
                .register_block()
                .conf0()
                .modify(|_, w| w.rxfifo_rst().clear_bit());
            self.update();
        }

        fn txfifo_reset(&mut self) {
            self.uart
                .register_block()
                .conf0()
                .modify(|_, w| w.txfifo_rst().set_bit());
            self.update();

            self.uart
                .register_block()
                .conf0()
                .modify(|_, w| w.txfifo_rst().clear_bit());
            self.update();
        }

        fn update(&mut self) {
            let register_block = self.uart.register_block();
            register_block
                .reg_update()
                .modify(|_, w| w.reg_update().set_bit());
            while register_block.reg_update().read().reg_update().bit_is_set() {
                // wait
            }
        }

        fn change_baud_internal(&mut self, config: &Config) {
            let clk = match config.clock_source {
                ClockSource::RcFast => crate::soc::clocks::rc_fast_clk_frequency(),
                ClockSource::Xtal => crate::soc::clocks::xtal_d2_clk_frequency(),
            };

            LP_CLKRST::regs().lpperi().modify(|_, w| {
                w.lp_uart_clk_sel().bit(match config.clock_source {
                    ClockSource::RcFast => false,
                    ClockSource::Xtal => true,
                })
            });
            self.uart.register_block().clk_conf().modify(|_, w| {
                w.rx_sclk_en().set_bit();
                w.tx_sclk_en().set_bit()
            });

            let divider = clk / config.baudrate;
            let divider = divider as u16;

            self.uart
                .register_block()
                .clkdiv()
                .write(|w| unsafe { w.clkdiv().bits(divider).frag().bits(0) });

            self.update();
        }

        /// Modify UART baud rate and reset TX/RX fifo.
        pub fn change_baud(&mut self, config: &Config) {
            self.change_baud_internal(config);
            self.txfifo_reset();
            self.rxfifo_reset();
        }

        fn change_parity(&mut self, parity: Parity) -> &mut Self {
            if parity != Parity::None {
                self.uart
                    .register_block()
                    .conf0()
                    .modify(|_, w| w.parity().bit((parity as u8 & 0x1) != 0));
            }

            self.uart
                .register_block()
                .conf0()
                .modify(|_, w| match parity {
                    Parity::None => w.parity_en().clear_bit(),
                    Parity::Even => w.parity_en().set_bit().parity().clear_bit(),
                    Parity::Odd => w.parity_en().set_bit().parity().set_bit(),
                });

            self
        }

        fn change_data_bits(&mut self, data_bits: DataBits) -> &mut Self {
            self.uart
                .register_block()
                .conf0()
                .modify(|_, w| unsafe { w.bit_num().bits(data_bits as u8) });

            self.update();
            self
        }

        fn change_stop_bits(&mut self, stop_bits: StopBits) -> &mut Self {
            self.uart
                .register_block()
                .conf0()
                .modify(|_, w| unsafe { w.stop_bit_num().bits(stop_bits as u8 + 1) });

            self.update();
            self
        }

        fn change_tx_idle(&mut self, idle_num: u16) -> &mut Self {
            self.uart
                .register_block()
                .idle_conf()
                .modify(|_, w| unsafe { w.tx_idle_num().bits(idle_num) });

            self.update();
            self
        }
    }
}
