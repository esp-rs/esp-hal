//! # Universal Asynchronous Receiver/Transmitter (UART)
//!
//! ## Overview
//!
//! The UART is a hardware peripheral which handles communication using serial
//! communication interfaces, such as RS232 and RS485. This peripheral provides
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
//! [embedded-io]: embedded_io
//! [embedded-hal-async]: embedded_hal_async
//! [embedded-io-async]: embedded_io_async

use core::{marker::PhantomData, sync::atomic::Ordering, task::Poll};

#[cfg(feature = "unstable")]
use embedded_io::ReadExactError;
use enumset::{EnumSet, EnumSetType};
use portable_atomic::AtomicBool;

use crate::{
    asynch::AtomicWaker,
    clock::Clocks,
    gpio::{
        interconnect::{OutputConnection, PeripheralInput, PeripheralOutput},
        InputSignal,
        OutputSignal,
        PinGuard,
        Pull,
    },
    interrupt::InterruptHandler,
    pac::uart0::RegisterBlock,
    peripheral::{Peripheral, PeripheralRef},
    peripherals::Interrupt,
    system::{PeripheralClockControl, PeripheralGuard},
    Async,
    Blocking,
    DriverMode,
};

/// UART RX Error
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum RxError {
    /// The RX FIFO overflow happened.
    ///
    /// This error occurs when RX FIFO is full and a new byte is received.
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

#[instability::unstable]
impl embedded_io::Error for RxError {
    fn kind(&self) -> embedded_io::ErrorKind {
        embedded_io::ErrorKind::Other
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

#[instability::unstable]
impl embedded_io::Error for TxError {
    fn kind(&self) -> embedded_io::ErrorKind {
        embedded_io::ErrorKind::Other
    }
}

/// UART clock source
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
#[instability::unstable]
pub enum ClockSource {
    /// APB_CLK clock source
    #[cfg_attr(not(any(esp32c6, esp32h2, lp_uart)), default)]
    Apb,
    /// RC_FAST_CLK clock source (17.5 MHz)
    #[cfg(not(any(esp32, esp32s2)))]
    RcFast,
    /// XTAL_CLK clock source
    #[cfg(not(any(esp32, esp32s2)))]
    #[cfg_attr(any(esp32c6, esp32h2, lp_uart), default)]
    Xtal,
    /// REF_TICK clock source (derived from XTAL or RC_FAST, 1MHz)
    #[cfg(any(esp32, esp32s2))]
    RefTick,
}

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
    baudrate_tolerance: BaudrateTolerance,
    /// Number of data bits in each frame (5, 6, 7, or 8 bits).
    data_bits: DataBits,
    /// Parity setting (None, Even, or Odd).
    parity: Parity,
    /// Number of stop bits in each frame (1, 1.5, or 2 bits).
    stop_bits: StopBits,
    /// Clock source used by the UART peripheral.
    #[cfg_attr(not(feature = "unstable"), builder_lite(skip))]
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
            return Err(ConfigError::UnsupportedBaudrate);
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
    /// Optional timeout between characters in the AT command, in clock
    /// cycles.
    gap_timeout: Option<u16>,
    /// The character that triggers the AT command detection.
    cmd_char: u8,
    /// Optional number of characters to detect as part of the AT command.
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
    uart: PeripheralRef<'d, AnyUart>,
    phantom: PhantomData<Dm>,
}

impl<'d, Dm> UartBuilder<'d, Dm>
where
    Dm: DriverMode,
{
    fn new(uart: impl Peripheral<P = impl Instance> + 'd) -> Self {
        crate::into_mapped_ref!(uart);
        Self {
            uart,
            phantom: PhantomData,
        }
    }

    fn init(self, config: Config) -> Result<Uart<'d, Dm>, ConfigError> {
        let rx_guard = PeripheralGuard::new(self.uart.parts().0.peripheral);
        let tx_guard = PeripheralGuard::new(self.uart.parts().0.peripheral);

        let rts_pin = PinGuard::new_unconnected(self.uart.info().rts_signal);
        let tx_pin = PinGuard::new_unconnected(self.uart.info().tx_signal);

        let mut serial = Uart {
            rx: UartRx {
                uart: unsafe { self.uart.clone_unchecked() },
                phantom: PhantomData,
                guard: rx_guard,
            },
            tx: UartTx {
                uart: self.uart,
                phantom: PhantomData,
                guard: tx_guard,
                rts_pin,
                tx_pin,
            },
        };
        serial.init(config)?;

        Ok(serial)
    }
}

/// UART (Full-duplex)
///
/// ```rust, no_run
#[doc = crate::before_snippet!()]
/// # use esp_hal::uart::{Config, Uart};
/// let mut uart = Uart::new(
///     peripherals.UART0,
///     Config::default())?
/// .with_rx(peripherals.GPIO1)
/// .with_tx(peripherals.GPIO2);
///
/// uart.write(b"Hello world!")?;
/// # Ok(())
/// # }
/// ```
pub struct Uart<'d, Dm: DriverMode> {
    rx: UartRx<'d, Dm>,
    tx: UartTx<'d, Dm>,
}

/// UART (Transmit)
#[instability::unstable]
pub struct UartTx<'d, Dm: DriverMode> {
    uart: PeripheralRef<'d, AnyUart>,
    phantom: PhantomData<Dm>,
    guard: PeripheralGuard,
    rts_pin: PinGuard,
    tx_pin: PinGuard,
}

/// UART (Receive)
#[instability::unstable]
pub struct UartRx<'d, Dm: DriverMode> {
    uart: PeripheralRef<'d, AnyUart>,
    phantom: PhantomData<Dm>,
    guard: PeripheralGuard,
}

/// A configuration error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum ConfigError {
    /// The requested baud rate is not achievable.
    #[cfg(any(doc, feature = "unstable"))]
    #[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
    UnachievableBaudrate,

    /// The requested baud rate is not supported.
    ///
    /// This error is returned if:
    ///  * the baud rate exceeds 5MBaud or is equal to zero.
    ///  * the user has specified an exact baud rate or with some percentage of
    ///    deviation to the desired value, and the driver cannot reach this
    ///    speed.
    UnsupportedBaudrate,

    /// The requested  timeout exceeds the maximum value (
    #[cfg_attr(esp32, doc = "127")]
    #[cfg_attr(not(esp32), doc = "1023")]
    /// ).
    UnsupportedTimeout,

    /// The requested RX FIFO threshold exceeds the maximum value (127 bytes).
    UnsupportedRxFifoThreshold,

    /// The requested TX FIFO threshold exceeds the maximum value (127 bytes).
    UnsupportedTxFifoThreshold,
}

impl core::error::Error for ConfigError {}

impl core::fmt::Display for ConfigError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            #[cfg(feature = "unstable")]
            ConfigError::UnachievableBaudrate => {
                write!(f, "The requested baud rate is not achievable")
            }
            ConfigError::UnsupportedBaudrate => {
                write!(f, "The requested baud rate is not supported")
            }
            ConfigError::UnsupportedTimeout => write!(f, "The requested timeout is not supported"),
            ConfigError::UnsupportedRxFifoThreshold => {
                write!(f, "The requested RX FIFO threshold is not supported")
            }
            ConfigError::UnsupportedTxFifoThreshold => {
                write!(f, "The requested TX FIFO threshold is not supported")
            }
        }
    }
}

#[instability::unstable]
impl<Dm> embassy_embedded_hal::SetConfig for Uart<'_, Dm>
where
    Dm: DriverMode,
{
    type Config = Config;
    type ConfigError = ConfigError;

    fn set_config(&mut self, config: &Self::Config) -> Result<(), Self::ConfigError> {
        self.apply_config(config)
    }
}

#[instability::unstable]
impl<Dm> embassy_embedded_hal::SetConfig for UartRx<'_, Dm>
where
    Dm: DriverMode,
{
    type Config = Config;
    type ConfigError = ConfigError;

    fn set_config(&mut self, config: &Self::Config) -> Result<(), Self::ConfigError> {
        self.apply_config(config)
    }
}

#[instability::unstable]
impl<Dm> embassy_embedded_hal::SetConfig for UartTx<'_, Dm>
where
    Dm: DriverMode,
{
    type Config = Config;
    type ConfigError = ConfigError;

    fn set_config(&mut self, config: &Self::Config) -> Result<(), Self::ConfigError> {
        self.apply_config(config)
    }
}

impl<'d, Dm> UartTx<'d, Dm>
where
    Dm: DriverMode,
{
    /// Configure RTS pin
    #[instability::unstable]
    pub fn with_rts(mut self, rts: impl Peripheral<P = impl PeripheralOutput> + 'd) -> Self {
        crate::into_mapped_ref!(rts);
        rts.set_to_push_pull_output();
        self.rts_pin = OutputConnection::connect_with_guard(rts, self.uart.info().rts_signal);

        self
    }

    /// Assign the TX pin for UART instance.
    ///
    /// Sets the specified pin to push-pull output and connects it to the UART
    /// TX signal.
    ///
    /// Disconnects the previous pin that was assigned with `with_tx`.
    #[instability::unstable]
    pub fn with_tx(mut self, tx: impl Peripheral<P = impl PeripheralOutput> + 'd) -> Self {
        crate::into_mapped_ref!(tx);
        // Make sure we don't cause an unexpected low pulse on the pin.
        tx.set_output_high(true);
        tx.set_to_push_pull_output();
        self.tx_pin = OutputConnection::connect_with_guard(tx, self.uart.info().tx_signal);

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
        if data.is_empty() {
            return Ok(0);
        }

        while self.tx_fifo_count() >= Info::UART_FIFO_SIZE {}

        let space = ((Info::UART_FIFO_SIZE - self.tx_fifo_count()) as usize).min(data.len());
        for &byte in &data[..space] {
            self.write_byte(byte)?;
        }

        Ok(space)
    }

    fn write_byte(&mut self, word: u8) -> Result<(), TxError> {
        self.regs()
            .fifo()
            .write(|w| unsafe { w.rxfifo_rd_byte().bits(word) });

        Ok(())
    }

    #[allow(clippy::useless_conversion)]
    /// Returns the number of bytes currently in the TX FIFO for this UART
    /// instance.
    fn tx_fifo_count(&self) -> u16 {
        self.regs().status().read().txfifo_cnt().bits().into()
    }

    /// Flush the transmit buffer.
    ///
    /// This function blocks until all data in the TX FIFO has been
    /// transmitted.
    #[instability::unstable]
    pub fn flush(&mut self) -> Result<(), TxError> {
        while self.tx_fifo_count() > 0 {}
        // The FSM is in the Idle state for a short while after the last byte is moved
        // out of the FIFO. It is unclear how long this takes, but 10us seems to be a
        // good enough duration to wait, for both fast and slow baud rates.
        crate::rom::ets_delay_us(10);
        while !self.is_tx_idle() {}
        Ok(())
    }

    /// Checks if the TX line is idle for this UART instance.
    ///
    /// Returns `true` if the transmit line is idle, meaning no data is
    /// currently being transmitted.
    fn is_tx_idle(&self) -> bool {
        #[cfg(esp32)]
        let status = self.regs().status();
        #[cfg(not(esp32))]
        let status = self.regs().fsm_status();

        status.read().st_utx_out().bits() == 0x0
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

impl<'d> UartTx<'d, Blocking> {
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
    #[doc = crate::before_snippet!()]
    /// # use esp_hal::uart::{Config, UartTx};
    /// let tx = UartTx::new(
    ///     peripherals.UART0,
    ///     Config::default())?
    /// .with_tx(peripherals.GPIO1);
    /// # Ok(())
    /// # }
    /// ```
    #[instability::unstable]
    pub fn new(
        uart: impl Peripheral<P = impl Instance> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        let (_, uart_tx) = UartBuilder::new(uart).init(config)?.split();

        Ok(uart_tx)
    }

    /// Reconfigures the driver to operate in [`Async`] mode.
    #[instability::unstable]
    pub fn into_async(self) -> UartTx<'d, Async> {
        if !self.uart.state().is_rx_async.load(Ordering::Acquire) {
            self.uart
                .info()
                .set_interrupt_handler(self.uart.info().async_handler);
        }
        self.uart.state().is_tx_async.store(true, Ordering::Release);

        UartTx {
            uart: self.uart,
            phantom: PhantomData,
            guard: self.guard,
            rts_pin: self.rts_pin,
            tx_pin: self.tx_pin,
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
            self.uart.info().disable_interrupts();
        }

        UartTx {
            uart: self.uart,
            phantom: PhantomData,
            guard: self.guard,
            rts_pin: self.rts_pin,
            tx_pin: self.tx_pin,
        }
    }
}

#[inline(always)]
fn sync_regs(_register_block: &RegisterBlock) {
    #[cfg(any(esp32c3, esp32c6, esp32h2, esp32s3))]
    {
        cfg_if::cfg_if! {
            if #[cfg(any(esp32c6, esp32h2))] {
                let update_reg = _register_block.reg_update();
            } else {
                let update_reg = _register_block.id();
            }
        }

        update_reg.modify(|_, w| w.reg_update().set_bit());

        while update_reg.read().reg_update().bit_is_set() {
            // wait
        }
    }
}

impl<'d, Dm> UartRx<'d, Dm>
where
    Dm: DriverMode,
{
    fn regs(&self) -> &RegisterBlock {
        self.uart.info().regs()
    }

    /// Configure CTS pin
    #[instability::unstable]
    pub fn with_cts(self, cts: impl Peripheral<P = impl PeripheralInput> + 'd) -> Self {
        crate::into_mapped_ref!(cts);
        cts.init_input(Pull::None);
        self.uart.info().cts_signal.connect_to(cts);

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
    pub fn with_rx(self, rx: impl Peripheral<P = impl PeripheralInput> + 'd) -> Self {
        crate::into_mapped_ref!(rx);
        rx.init_input(Pull::Up);
        self.uart.info().rx_signal.connect_to(rx);

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
            .set_rx_fifo_full_threshold(config.rx.fifo_full_threshold)?;
        self.uart
            .info()
            .set_rx_timeout(config.rx.timeout, self.uart.info().current_symbol_length())?;

        self.uart.info().rxfifo_reset();
        Ok(())
    }

    /// Reads and clears errors set by received data.
    #[instability::unstable]
    pub fn check_for_errors(&mut self) -> Result<(), RxError> {
        let errors = RxEvent::FifoOvf
            | RxEvent::FifoTout
            | RxEvent::GlitchDetected
            | RxEvent::FrameError
            | RxEvent::ParityError;
        let events = self.uart.info().rx_events().intersection(errors);
        let result = rx_event_check_for_error(events);
        if result.is_err() {
            self.uart.info().clear_rx_events(errors);
        }
        result
    }

    /// Read bytes.
    ///
    /// The UART hardware continuously receives bytes and stores them in the RX
    /// FIFO. This function reads the bytes from the RX FIFO and returns
    /// them in the provided buffer, without blocking.
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
        if buf.is_empty() {
            return Ok(0);
        }

        while self.rx_fifo_count() == 0 {
            // Block until we received at least one byte
            self.check_for_errors()?;
        }

        self.read_buffered(buf)
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
        // Get the count first, to avoid accidentally reading a corrupted byte received
        // after the error check.
        let to_read = (self.rx_fifo_count() as usize).min(buf.len());
        self.check_for_errors()?;

        for byte_into in buf[..to_read].iter_mut() {
            *byte_into = self.uart.info().read_next_from_fifo();
        }

        Ok(to_read)
    }

    /// Busy waits for a break condition to be detected on the RX
    /// line. Condition is met when the receiver detects a NULL character
    /// (i.e. logic 0 for one NULL character transmission) after stop bits.
    ///
    /// Clears the break detection interrupt before returning.
    #[instability::unstable]
    pub fn wait_for_break(&mut self) {
        // Enable the break detection interrupt
        self.regs().int_ena().write(|w| w.brk_det().bit(true));

        while !self.regs().int_raw().read().brk_det().bit() {
            // Just busy waiting
        }

        // Clear the break detection interrupt
        self.regs().int_clr().write(|w| w.brk_det().bit(true));
    }

    #[allow(clippy::useless_conversion)]
    fn rx_fifo_count(&self) -> u16 {
        let fifo_cnt: u16 = self.regs().status().read().rxfifo_cnt().bits().into();

        // Calculate the real count based on the FIFO read and write offset address:
        // https://docs.espressif.com/projects/esp-chip-errata/en/latest/esp32/03-errata-description/esp32/uart-fifo-cnt-indicates-data-length-incorrectly.html
        #[cfg(esp32)]
        {
            let status = self.regs().mem_rx_status().read();
            let rd_addr = status.mem_rx_rd_addr().bits();
            let wr_addr = status.mem_rx_wr_addr().bits();

            if wr_addr > rd_addr {
                wr_addr - rd_addr
            } else if wr_addr < rd_addr {
                (wr_addr + Info::UART_FIFO_SIZE) - rd_addr
            } else if fifo_cnt > 0 {
                Info::UART_FIFO_SIZE
            } else {
                0
            }
        }

        #[cfg(not(esp32))]
        fifo_cnt
    }

    /// Disables all RX-related interrupts for this UART instance.
    ///
    /// This function clears and disables the `receive FIFO full` interrupt,
    /// `receive FIFO overflow`, `receive FIFO timeout`, and `AT command
    /// character detection` interrupts.
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

impl<'d> UartRx<'d, Blocking> {
    /// Create a new UART RX instance in [`Blocking`] mode.
    ///
    /// ## Errors
    ///
    /// This function returns a [`ConfigError`] if the configuration is not
    /// supported by the hardware.
    ///
    /// ```rust, no_run
    #[doc = crate::before_snippet!()]
    /// # use esp_hal::uart::{Config, UartRx};
    /// let rx = UartRx::new(
    ///     peripherals.UART1,
    ///     Config::default())?
    /// .with_rx(peripherals.GPIO2);
    /// # Ok(())
    /// # }
    /// ```
    #[instability::unstable]
    pub fn new(
        uart: impl Peripheral<P = impl Instance> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        let (uart_rx, _) = UartBuilder::new(uart).init(config)?.split();

        Ok(uart_rx)
    }

    /// Reconfigures the driver to operate in [`Async`] mode.
    #[instability::unstable]
    pub fn into_async(self) -> UartRx<'d, Async> {
        if !self.uart.state().is_tx_async.load(Ordering::Acquire) {
            self.uart
                .info()
                .set_interrupt_handler(self.uart.info().async_handler);
        }
        self.uart.state().is_rx_async.store(true, Ordering::Release);

        UartRx {
            uart: self.uart,
            phantom: PhantomData,
            guard: self.guard,
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
            self.uart.info().disable_interrupts();
        }

        UartRx {
            uart: self.uart,
            phantom: PhantomData,
            guard: self.guard,
        }
    }
}

impl<'d> Uart<'d, Blocking> {
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
    #[doc = crate::before_snippet!()]
    /// # use esp_hal::uart::{Config, Uart};
    /// let mut uart1 = Uart::new(
    ///     peripherals.UART1,
    ///     Config::default())?
    /// .with_rx(peripherals.GPIO1)
    /// .with_tx(peripherals.GPIO2);
    /// # Ok(())
    /// # }
    /// ```
    pub fn new(
        uart: impl Peripheral<P = impl Instance> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        UartBuilder::new(uart).init(config)
    }

    /// Reconfigures the driver to operate in [`Async`] mode.
    pub fn into_async(self) -> Uart<'d, Async> {
        Uart {
            rx: self.rx.into_async(),
            tx: self.tx.into_async(),
        }
    }

    /// Assign the RX pin for UART instance.
    ///
    /// Sets the specified pin to input and connects it to the UART RX signal.
    ///
    /// Note: when you listen for the output of the UART peripheral, you should
    /// configure the driver side (i.e. the TX pin), or ensure that the line is
    /// initially high, to avoid receiving a non-data byte caused by an
    /// initial low signal level.
    pub fn with_rx(mut self, rx: impl Peripheral<P = impl PeripheralInput> + 'd) -> Self {
        self.rx = self.rx.with_rx(rx);
        self
    }

    /// Assign the TX pin for UART instance.
    ///
    /// Sets the specified pin to push-pull output and connects it to the UART
    /// TX signal.
    pub fn with_tx(mut self, tx: impl Peripheral<P = impl PeripheralOutput> + 'd) -> Self {
        self.tx = self.tx.with_tx(tx);
        self
    }
}

impl<'d> Uart<'d, Async> {
    /// Reconfigures the driver to operate in [`Blocking`] mode.
    pub fn into_blocking(self) -> Uart<'d, Blocking> {
        Uart {
            rx: self.rx.into_blocking(),
            tx: self.tx.into_blocking(),
        }
    }
}

/// List of exposed UART events.
#[derive(Debug, EnumSetType)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
#[instability::unstable]
pub enum UartInterrupt {
    /// Indicates that the received has detected the configured
    /// [`Uart::set_at_cmd`] character.
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
}

impl<'d, Dm> Uart<'d, Dm>
where
    Dm: DriverMode,
{
    /// Configure CTS pin
    pub fn with_cts(mut self, cts: impl Peripheral<P = impl PeripheralInput> + 'd) -> Self {
        self.rx = self.rx.with_cts(cts);
        self
    }

    /// Configure RTS pin
    pub fn with_rts(mut self, rts: impl Peripheral<P = impl PeripheralOutput> + 'd) -> Self {
        self.tx = self.tx.with_rts(rts);
        self
    }

    fn regs(&self) -> &RegisterBlock {
        // `self.tx.uart` and `self.rx.uart` are the same
        self.tx.uart.info().regs()
    }

    /// Split the UART into a transmitter and receiver
    ///
    /// This is particularly useful when having two tasks correlating to
    /// transmitting and receiving.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    #[doc = crate::before_snippet!()]
    /// # use esp_hal::uart::{Config, Uart};
    /// # let mut uart1 = Uart::new(
    /// #     peripherals.UART1,
    /// #     Config::default())?
    /// # .with_rx(peripherals.GPIO1)
    /// # .with_tx(peripherals.GPIO2);
    /// // The UART can be split into separate Transmit and Receive components:
    /// let (mut rx, mut tx) = uart1.split();
    ///
    /// // Each component can be used individually to interact with the UART:
    /// tx.write(&[42u8])?;
    /// let mut byte = [0u8; 1];
    /// rx.read(&mut byte);
    /// # Ok(())
    /// # }
    /// ```
    #[instability::unstable]
    pub fn split(self) -> (UartRx<'d, Dm>, UartTx<'d, Dm>) {
        (self.rx, self.tx)
    }

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
    #[doc = crate::before_snippet!()]
    /// # use esp_hal::uart::{Config, Uart};
    /// # let mut uart1 = Uart::new(
    /// #     peripherals.UART1,
    /// #     Config::default())?;
    /// // Write bytes out over the UART:
    /// uart1.write(b"Hello, world!")?;
    /// # Ok(())
    /// # }
    /// ```
    pub fn write(&mut self, data: &[u8]) -> Result<usize, TxError> {
        self.tx.write(data)
    }

    /// Reads and clears errors set by received data.
    #[instability::unstable]
    pub fn check_for_rx_errors(&mut self) -> Result<(), RxError> {
        self.rx.check_for_errors()
    }

    /// Read received bytes.
    ///
    /// The UART hardware continuously receives bytes and stores them in the RX
    /// FIFO. This function reads the bytes from the RX FIFO and returns
    /// them in the provided buffer, without blocking.
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
    pub fn read(&mut self, buf: &mut [u8]) -> Result<usize, RxError> {
        self.rx.read(buf)
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
        #[cfg(not(any(esp32, esp32s2)))]
        self.regs()
            .clk_conf()
            .modify(|_, w| w.sclk_en().clear_bit());

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

        #[cfg(not(any(esp32, esp32s2)))]
        self.regs().clk_conf().modify(|_, w| w.sclk_en().set_bit());

        sync_regs(self.regs());
    }

    /// Busy waits for a break condition to be detected on the RX line.
    pub fn wait_for_break(&mut self) {
        self.rx.wait_for_break();
    }

    /// Flush the transmit buffer of the UART
    pub fn flush(&mut self) -> Result<(), TxError> {
        self.tx.flush()
    }

    /// Change the configuration.
    ///
    /// ## Errors
    ///
    /// This function returns a [`ConfigError`] if the configuration is not
    /// supported by the hardware.
    pub fn apply_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        // Must apply the common settings first, as `rx.apply_config` reads back symbol
        // size.
        self.rx.uart.info().apply_config(config)?;
        self.rx.apply_config(config)?;
        self.tx.apply_config(config)?;
        Ok(())
    }

    #[inline(always)]
    fn init(&mut self, config: Config) -> Result<(), ConfigError> {
        cfg_if::cfg_if! {
            if #[cfg(any(esp32, esp32s2))] {
                // Nothing to do
            } else if #[cfg(any(esp32c2, esp32c3, esp32s3))] {
                crate::peripherals::SYSTEM::regs()
                    .perip_clk_en0()
                    .modify(|_, w| w.uart_mem_clk_en().set_bit());
            } else {
                self.regs()
                    .conf0()
                    .modify(|_, w| w.mem_clk_en().set_bit());
            }
        };

        self.uart_peripheral_reset();

        self.rx.disable_rx_interrupts();
        self.tx.disable_tx_interrupts();

        self.apply_config(&config)?;

        // Reset Tx/Rx FIFOs
        self.rx.uart.info().rxfifo_reset();
        self.rx.uart.info().txfifo_reset();

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

    fn is_instance(&self, other: impl Instance) -> bool {
        self.tx.uart.info().is_instance(other)
    }

    #[inline(always)]
    fn uart_peripheral_reset(&self) {
        // don't reset the console UART - this will cause trouble (i.e. the UART will
        // start to transmit garbage)
        //
        // We should only reset the console UART if it was absolutely unused before.
        // Apparently the bootloader (and maybe the ROM code) writing to the UART is
        // already enough to make this a no-go. (i.e. one needs to mute the ROM
        // code via efuse / strapping pin AND use a silent bootloader)
        //
        // TODO: make this configurable
        // see https://github.com/espressif/esp-idf/blob/5f4249357372f209fdd57288265741aaba21a2b1/components/esp_driver_uart/src/uart.c#L179
        if self.is_instance(unsafe { crate::peripherals::UART0::steal() }) {
            return;
        }

        fn rst_core(_reg_block: &RegisterBlock, _enable: bool) {
            #[cfg(not(any(esp32, esp32s2, esp32c6, esp32h2)))]
            _reg_block
                .clk_conf()
                .modify(|_, w| w.rst_core().bit(_enable));
        }

        rst_core(self.regs(), true);
        PeripheralClockControl::reset(self.tx.uart.info().peripheral);
        rst_core(self.regs(), false);
    }
}

impl crate::private::Sealed for Uart<'_, Blocking> {}

#[instability::unstable]
impl crate::interrupt::InterruptConfigurable for Uart<'_, Blocking> {
    fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        // `self.tx.uart` and `self.rx.uart` are the same
        self.tx.uart.info().set_interrupt_handler(handler);
    }
}

impl Uart<'_, Blocking> {
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
        self.tx.uart.info().set_interrupt_handler(handler);
    }

    /// Listen for the given interrupts
    ///
    /// ## Example
    ///
    /// **Note**: In practice a proper serial terminal should be used
    /// to connect to the board (espflash won't work)
    ///
    /// ```rust, no_run
    #[doc = crate::before_snippet!()]
    /// # use esp_hal::delay::Delay;
    /// # use esp_hal::uart::{AtCmdConfig, Config, RxConfig, Uart, UartInterrupt};
    /// # let delay = Delay::new();
    /// # let config = Config::default().with_rx(
    /// #    RxConfig::default().with_fifo_full_threshold(30)
    /// # );
    /// # let mut uart0 = Uart::new(
    /// #    peripherals.UART0,
    /// #    config)?;
    /// uart0.set_interrupt_handler(interrupt_handler);
    ///
    /// critical_section::with(|cs| {
    ///     uart0.set_at_cmd(AtCmdConfig::default().with_cmd_char(b'#'));
    ///     uart0.listen(UartInterrupt::AtCmd | UartInterrupt::RxFifoFull);
    ///
    ///     SERIAL.borrow_ref_mut(cs).replace(uart0);
    /// });
    ///
    /// loop {
    ///     println!("Send `#` character or >=30 characters");
    ///     delay.delay(Duration::from_secs(1));
    /// }
    /// # }
    ///
    /// # use core::cell::RefCell;
    /// # use critical_section::Mutex;
    /// # use esp_hal::uart::Uart;
    /// static SERIAL: Mutex<RefCell<Option<Uart<esp_hal::Blocking>>>> =
    ///     Mutex::new(RefCell::new(None));
    ///
    /// # use esp_hal::uart::UartInterrupt;
    /// # use core::fmt::Write;
    /// #[handler]
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
    ///             serial.clear_interrupts(
    ///                 UartInterrupt::AtCmd | UartInterrupt::RxFifoFull
    ///             );
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
}

#[instability::unstable]
impl<Dm> ufmt_write::uWrite for Uart<'_, Dm>
where
    Dm: DriverMode,
{
    type Error = TxError;

    #[inline]
    fn write_str(&mut self, s: &str) -> Result<(), Self::Error> {
        self.tx.write_str(s)
    }

    #[inline]
    fn write_char(&mut self, ch: char) -> Result<(), Self::Error> {
        self.tx.write_char(ch)
    }
}

#[instability::unstable]
impl<Dm> ufmt_write::uWrite for UartTx<'_, Dm>
where
    Dm: DriverMode,
{
    type Error = TxError;

    #[inline]
    fn write_str(&mut self, s: &str) -> Result<(), Self::Error> {
        self.write(s.as_bytes())?;
        Ok(())
    }
}

impl<Dm> core::fmt::Write for Uart<'_, Dm>
where
    Dm: DriverMode,
{
    #[inline]
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.tx.write_str(s)
    }
}

impl<Dm> core::fmt::Write for UartTx<'_, Dm>
where
    Dm: DriverMode,
{
    #[inline]
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.write(s.as_bytes()).map_err(|_| core::fmt::Error)?;
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
impl embedded_io::Error for IoError {
    fn kind(&self) -> embedded_io::ErrorKind {
        embedded_io::ErrorKind::Other
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

#[instability::unstable]
impl<Dm: DriverMode> embedded_io::ErrorType for Uart<'_, Dm> {
    type Error = IoError;
}

#[instability::unstable]
impl<Dm: DriverMode> embedded_io::ErrorType for UartTx<'_, Dm> {
    type Error = TxError;
}

#[instability::unstable]
impl<Dm: DriverMode> embedded_io::ErrorType for UartRx<'_, Dm> {
    type Error = RxError;
}

#[instability::unstable]
impl<Dm> embedded_io::Read for Uart<'_, Dm>
where
    Dm: DriverMode,
{
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.rx.read(buf).map_err(IoError::Rx)
    }
}

#[instability::unstable]
impl<Dm> embedded_io::Read for UartRx<'_, Dm>
where
    Dm: DriverMode,
{
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.read(buf)
    }
}

#[instability::unstable]
impl<Dm> embedded_io::ReadReady for Uart<'_, Dm>
where
    Dm: DriverMode,
{
    fn read_ready(&mut self) -> Result<bool, Self::Error> {
        self.rx.read_ready().map_err(IoError::Rx)
    }
}

#[instability::unstable]
impl<Dm> embedded_io::ReadReady for UartRx<'_, Dm>
where
    Dm: DriverMode,
{
    fn read_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(self.rx_fifo_count() > 0)
    }
}

#[instability::unstable]
impl<Dm> embedded_io::Write for Uart<'_, Dm>
where
    Dm: DriverMode,
{
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.tx.write(buf).map_err(IoError::Tx)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        self.tx.flush().map_err(IoError::Tx)
    }
}

#[instability::unstable]
impl<Dm> embedded_io::Write for UartTx<'_, Dm>
where
    Dm: DriverMode,
{
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.write(buf)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        self.flush()
    }
}

#[derive(Debug, EnumSetType)]
pub(crate) enum TxEvent {
    Done,
    FiFoEmpty,
}

#[derive(Debug, EnumSetType)]
pub(crate) enum RxEvent {
    FifoFull,
    BreakDetected,
    CmdCharDetected,
    FifoOvf,
    FifoTout,
    GlitchDetected,
    FrameError,
    ParityError,
}

fn rx_event_check_for_error(events: EnumSet<RxEvent>) -> Result<(), RxError> {
    for event in events {
        match event {
            RxEvent::FifoOvf => return Err(RxError::FifoOverflowed),
            RxEvent::GlitchDetected => return Err(RxError::GlitchOccurred),
            RxEvent::FrameError => return Err(RxError::FrameFormatViolated),
            RxEvent::ParityError => return Err(RxError::ParityMismatch),
            RxEvent::FifoFull
            | RxEvent::BreakDetected
            | RxEvent::CmdCharDetected
            | RxEvent::FifoTout => continue,
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
struct UartRxFuture {
    events: EnumSet<RxEvent>,
    uart: &'static Info,
    state: &'static State,
    registered: bool,
}

impl UartRxFuture {
    fn new(uart: impl Peripheral<P = impl Instance>, events: impl Into<EnumSet<RxEvent>>) -> Self {
        crate::into_ref!(uart);
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
struct UartTxFuture {
    events: EnumSet<TxEvent>,
    uart: &'static Info,
    state: &'static State,
    registered: bool,
}

impl UartTxFuture {
    fn new(uart: impl Peripheral<P = impl Instance>, events: impl Into<EnumSet<TxEvent>>) -> Self {
        crate::into_ref!(uart);
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

impl Uart<'_, Async> {
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
        self.rx.read_async(buf).await
    }

    /// Asynchronously waits for a break condition on the RX line.
    /// Condition is met when the receiver detects a NULL character (i.e. logic
    /// 0 for one NULL character transmission) after stop bits.
    pub async fn wait_for_break_async(&mut self) {
        self.rx.wait_for_break_async().await;
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
    pub async fn write_async(&mut self, words: &[u8]) -> Result<usize, TxError> {
        self.tx.write_async(words).await
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
        self.tx.flush_async().await
    }
}

impl UartTx<'_, Async> {
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
            let space = Info::UART_FIFO_SIZE - self.tx_fifo_count();
            if space != 0 {
                break space;
            }
            UartTxFuture::new(self.uart.reborrow(), TxEvent::FiFoEmpty).await;
        };

        let free = (space as usize).min(bytes.len());

        for &byte in &bytes[..free] {
            self.regs()
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
        if self.tx_fifo_count() > 0 {
            UartTxFuture::new(self.uart.reborrow(), TxEvent::Done).await;
        }

        Ok(())
    }
}

impl UartRx<'_, Async> {
    async fn wait_for_buffered_data(
        &mut self,
        minimum: usize,
        preferred: usize,
        listen_for_timeout: bool,
    ) -> Result<(), RxError> {
        while self.rx_fifo_count() < (minimum as u16).min(Info::RX_FIFO_MAX_THRHD) {
            let amount = u16::try_from(preferred)
                .unwrap_or(Info::RX_FIFO_MAX_THRHD)
                .min(Info::RX_FIFO_MAX_THRHD);

            let current = self.uart.info().rx_fifo_full_threshold();
            let _guard = if current > amount {
                // We're ignoring the user configuration here to ensure that this is not waiting
                // for more data than the buffer. We'll restore the original value after the
                // future resolved.
                let info = self.uart.info();
                unwrap!(info.set_rx_fifo_full_threshold(amount));
                Some(OnDrop::new(|| {
                    unwrap!(info.set_rx_fifo_full_threshold(current));
                }))
            } else {
                None
            };

            // Wait for space or event
            let mut events = RxEvent::FifoFull
                | RxEvent::BreakDetected
                | RxEvent::FifoOvf
                | RxEvent::FrameError
                | RxEvent::GlitchDetected
                | RxEvent::ParityError;

            if self.regs().at_cmd_char().read().char_num().bits() > 0 {
                events |= RxEvent::CmdCharDetected;
            }

            cfg_if::cfg_if! {
                if #[cfg(any(esp32c6, esp32h2))] {
                    let reg_en = self.regs().tout_conf();
                } else {
                    let reg_en = self.regs().conf1();
                }
            };
            if listen_for_timeout && reg_en.read().rx_tout_en().bit_is_set() {
                events |= RxEvent::FifoTout;
            }

            let event = UartRxFuture::new(self.uart.reborrow(), events).await;
            rx_event_check_for_error(event)?;
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
        while !buf.is_empty() {
            // No point in listening for timeouts, as we're waiting for an exact amount of
            // data. On ESP32 and S2, the timeout interrupt can't be cleared unless the FIFO
            // is empty, so listening could cause an infinite loop here.
            self.wait_for_buffered_data(buf.len(), buf.len(), false)
                .await?;

            let read = self.read_buffered(buf)?;
            buf = &mut buf[read..];
        }

        Ok(())
    }

    /// Interrupt-driven wait for a break condition on the RX line.
    /// Condition is met when the receiver detects a NULL character (i.e. logic
    /// 0 for one NULL character transmission) after stop bits.
    pub async fn wait_for_break_async(&mut self) {
        UartRxFuture::new(self.uart.reborrow(), RxEvent::BreakDetected).await;
    }
}

#[instability::unstable]
impl embedded_io_async::Read for Uart<'_, Async> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.read_async(buf).await.map_err(IoError::Rx)
    }

    async fn read_exact(&mut self, buf: &mut [u8]) -> Result<(), ReadExactError<Self::Error>> {
        self.read_exact_async(buf)
            .await
            .map_err(|e| ReadExactError::Other(IoError::Rx(e)))
    }
}

#[instability::unstable]
impl embedded_io_async::Read for UartRx<'_, Async> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.read_async(buf).await
    }

    async fn read_exact(&mut self, buf: &mut [u8]) -> Result<(), ReadExactError<Self::Error>> {
        self.read_exact_async(buf)
            .await
            .map_err(ReadExactError::Other)
    }
}

#[instability::unstable]
impl embedded_io_async::Write for Uart<'_, Async> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.write_async(buf).await.map_err(IoError::Tx)
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        self.flush_async().await.map_err(IoError::Tx)
    }
}

#[instability::unstable]
impl embedded_io_async::Write for UartTx<'_, Async> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.write_async(buf).await
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        self.flush_async().await
    }
}

/// Interrupt handler for all UART instances
/// Clears and disables interrupts that have occurred and have their enable
/// bit set. The fact that an interrupt has been disabled is used by the
/// futures to detect that they should indeed resolve after being woken up
pub(super) fn intr_handler(uart: &Info, state: &State) {
    let interrupts = uart.regs().int_st().read();
    let interrupt_bits = interrupts.bits(); // = int_raw & int_ena
    let rx_wake = interrupts.rxfifo_full().bit_is_set()
        | interrupts.rxfifo_ovf().bit_is_set()
        | interrupts.rxfifo_tout().bit_is_set()
        | interrupts.at_cmd_char_det().bit_is_set()
        | interrupts.glitch_det().bit_is_set()
        | interrupts.frm_err().bit_is_set()
        | interrupts.parity_err().bit_is_set();
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

/// Low-power UART
#[cfg(lp_uart)]
#[instability::unstable]
pub mod lp_uart {
    use crate::{
        gpio::lp_io::{LowPowerInput, LowPowerOutput},
        peripherals::{LPWR, LP_AON, LP_IO, LP_UART},
        uart::{Config, DataBits, Parity, StopBits},
    };
    /// LP-UART driver
    ///
    /// The driver uses XTAL as clock source.
    pub struct LpUart {
        uart: LP_UART,
    }

    impl LpUart {
        /// Initialize the UART driver using the provided configuration
        ///
        /// # Panics
        ///
        /// [`Apb`] is a wrong clock source for LP_UART
        ///
        /// [`Apb`]: super::ClockSource::Apb
        // TODO: CTS and RTS pins
        pub fn new(
            uart: LP_UART,
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
            // TODO: Currently it's not possible to use XtalD2Clk
            let clk = 16_000_000_u32;
            let max_div = 0b1111_1111_1111 - 1;
            let clk_div = clk.div_ceil(max_div * config.baudrate);

            self.uart.register_block().clk_conf().modify(|_, w| unsafe {
                w.sclk_div_a().bits(0);
                w.sclk_div_b().bits(0);
                w.sclk_div_num().bits(clk_div as u8 - 1);
                w.sclk_sel().bits(match config.clock_source {
                    super::ClockSource::Xtal => 3,
                    super::ClockSource::RcFast => 2,
                    super::ClockSource::Apb => panic!("Wrong clock source for LP_UART"),
                });
                w.sclk_en().set_bit()
            });

            let clk = clk / clk_div;
            let divider = clk / config.baudrate;
            let divider = divider as u16;

            self.uart
                .register_block()
                .clkdiv()
                .write(|w| unsafe { w.clkdiv().bits(divider).frag().bits(0) });

            self.update();
        }

        /// Modify UART baud rate and reset TX/RX fifo.
        ///
        /// # Panics
        ///
        /// [`Apb`] is a wrong clock source for LP_UART
        ///
        /// [`Apb`]: super::ClockSource::Apb
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

/// UART Peripheral Instance
#[doc(hidden)]
pub trait Instance: Peripheral<P = Self> + Into<AnyUart> + 'static {
    /// Returns the peripheral data and state describing this UART instance.
    fn parts(&self) -> (&'static Info, &'static State);

    /// Returns the peripheral data describing this UART instance.
    #[inline(always)]
    fn info(&self) -> &'static Info {
        self.parts().0
    }

    /// Returns the peripheral state for this UART instance.
    #[inline(always)]
    fn state(&self) -> &'static State {
        self.parts().1
    }
}

/// Peripheral data describing a particular UART instance.
#[doc(hidden)]
#[non_exhaustive]
pub struct Info {
    /// Pointer to the register block for this UART instance.
    ///
    /// Use [Self::register_block] to access the register block.
    pub register_block: *const RegisterBlock,

    /// The system peripheral marker.
    pub peripheral: crate::system::Peripheral,

    /// Interrupt handler for the asynchronous operations of this UART instance.
    pub async_handler: InterruptHandler,

    /// Interrupt for this UART instance.
    pub interrupt: Interrupt,

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

    /// Stores whether the TX half is configured for async operation.
    pub is_rx_async: AtomicBool,

    /// Stores whether the RX half is configured for async operation.
    pub is_tx_async: AtomicBool,
}

impl Info {
    // Currently we don't support merging adjacent FIFO memory, so the max size is
    // 128 bytes, the max threshold is 127 bytes.
    const UART_FIFO_SIZE: u16 = 128;
    const RX_FIFO_MAX_THRHD: u16 = 127;
    const TX_FIFO_MAX_THRHD: u16 = Self::RX_FIFO_MAX_THRHD;

    /// Returns the register block for this UART instance.
    pub fn regs(&self) -> &RegisterBlock {
        unsafe { &*self.register_block }
    }

    /// Listen for the given interrupts
    fn enable_listen(&self, interrupts: EnumSet<UartInterrupt>, enable: bool) {
        let reg_block = self.regs();

        reg_block.int_ena().modify(|_, w| {
            for interrupt in interrupts {
                match interrupt {
                    UartInterrupt::AtCmd => w.at_cmd_char_det().bit(enable),
                    UartInterrupt::TxDone => w.tx_done().bit(enable),
                    UartInterrupt::RxFifoFull => w.rxfifo_full().bit(enable),
                    UartInterrupt::RxBreakDetected => w.brk_det().bit(enable),
                };
            }
            w
        });
    }

    fn interrupts(&self) -> EnumSet<UartInterrupt> {
        let mut res = EnumSet::new();
        let reg_block = self.regs();

        let ints = reg_block.int_raw().read();

        if ints.at_cmd_char_det().bit_is_set() {
            res.insert(UartInterrupt::AtCmd);
        }
        if ints.tx_done().bit_is_set() {
            res.insert(UartInterrupt::TxDone);
        }
        if ints.rxfifo_full().bit_is_set() {
            res.insert(UartInterrupt::RxFifoFull);
        }
        if ints.brk_det().bit_is_set() {
            res.insert(UartInterrupt::RxBreakDetected);
        }

        res
    }

    fn clear_interrupts(&self, interrupts: EnumSet<UartInterrupt>) {
        let reg_block = self.regs();

        reg_block.int_clr().write(|w| {
            for interrupt in interrupts {
                match interrupt {
                    UartInterrupt::AtCmd => w.at_cmd_char_det().clear_bit_by_one(),
                    UartInterrupt::TxDone => w.tx_done().clear_bit_by_one(),
                    UartInterrupt::RxFifoFull => w.rxfifo_full().clear_bit_by_one(),
                    UartInterrupt::RxBreakDetected => w.brk_det().clear_bit_by_one(),
                };
            }
            w
        });
    }

    fn set_interrupt_handler(&self, handler: InterruptHandler) {
        for core in crate::system::Cpu::other() {
            crate::interrupt::disable(core, self.interrupt);
        }
        self.enable_listen(EnumSet::all(), false);
        self.clear_interrupts(EnumSet::all());
        unsafe { crate::interrupt::bind_interrupt(self.interrupt, handler.handler()) };
        unwrap!(crate::interrupt::enable(self.interrupt, handler.priority()));
    }

    fn disable_interrupts(&self) {
        crate::interrupt::disable(crate::system::Cpu::current(), self.interrupt);
    }

    fn apply_config(&self, config: &Config) -> Result<(), ConfigError> {
        config.validate()?;
        self.change_baud(config)?;
        self.change_data_bits(config.data_bits);
        self.change_parity(config.parity);
        self.change_stop_bits(config.stop_bits);

        Ok(())
    }

    fn enable_listen_tx(&self, events: EnumSet<TxEvent>, enable: bool) {
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

    fn enable_listen_rx(&self, events: EnumSet<RxEvent>, enable: bool) {
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
    /// [ConfigError::UnsupportedRxFifoThreshold] if the provided value exceeds
    /// [`Info::RX_FIFO_MAX_THRHD`].
    fn set_rx_fifo_full_threshold(&self, threshold: u16) -> Result<(), ConfigError> {
        if threshold > Self::RX_FIFO_MAX_THRHD {
            return Err(ConfigError::UnsupportedRxFifoThreshold);
        }

        self.regs()
            .conf1()
            .modify(|_, w| unsafe { w.rxfifo_full_thrhd().bits(threshold as _) });

        Ok(())
    }

    /// Reads the RX-FIFO threshold
    #[allow(clippy::useless_conversion)]
    fn rx_fifo_full_threshold(&self) -> u16 {
        self.regs().conf1().read().rxfifo_full_thrhd().bits().into()
    }

    /// Configures the TX-FIFO threshold
    ///
    /// ## Errors
    ///
    /// [ConfigError::UnsupportedTxFifoThreshold] if the provided value exceeds
    /// [`Info::TX_FIFO_MAX_THRHD`].
    fn set_tx_fifo_empty_threshold(&self, threshold: u16) -> Result<(), ConfigError> {
        if threshold > Self::TX_FIFO_MAX_THRHD {
            return Err(ConfigError::UnsupportedTxFifoThreshold);
        }

        self.regs()
            .conf1()
            .modify(|_, w| unsafe { w.txfifo_empty_thrhd().bits(threshold as _) });

        Ok(())
    }

    /// Configures the Receive Timeout detection setting
    ///
    /// ## Arguments
    ///
    /// `timeout` - the number of symbols ("bytes") to wait for before
    /// triggering a timeout. Pass None to disable the timeout.
    ///
    /// ## Errors
    ///
    /// [ConfigError::UnsupportedTimeout] if the provided value exceeds
    /// the maximum value for SOC:
    /// - `esp32`: Symbol size is fixed to 8, do not pass a value > **0x7F**.
    /// - `esp32c2`, `esp32c3`, `esp32c6`, `esp32h2`, esp32s2`, esp32s3`: The
    ///   value you pass times the symbol size must be <= **0x3FF**
    fn set_rx_timeout(&self, timeout: Option<u8>, _symbol_len: u8) -> Result<(), ConfigError> {
        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
                const MAX_THRHD: u8 = 0x7F; // 7 bits
            } else {
                const MAX_THRHD: u16 = 0x3FF; // 10 bits
            }
        }

        let register_block = self.regs();

        if let Some(timeout) = timeout {
            // the esp32 counts directly in number of symbols (symbol len fixed to 8)
            #[cfg(esp32)]
            let timeout_reg = timeout;
            // all other count in bits, so we need to multiply by the symbol len.
            #[cfg(not(esp32))]
            let timeout_reg = timeout as u16 * _symbol_len as u16;

            if timeout_reg > MAX_THRHD {
                return Err(ConfigError::UnsupportedTimeout);
            }

            cfg_if::cfg_if! {
                if #[cfg(esp32)] {
                    let reg_thrhd = register_block.conf1();
                } else if #[cfg(any(esp32c6, esp32h2))] {
                    let reg_thrhd = register_block.tout_conf();
                } else {
                    let reg_thrhd = register_block.mem_conf();
                }
            }
            reg_thrhd.modify(|_, w| unsafe { w.rx_tout_thrhd().bits(timeout_reg) });
        }

        cfg_if::cfg_if! {
            if #[cfg(any(esp32c6, esp32h2))] {
                let reg_en = register_block.tout_conf();
            } else {
                let reg_en = register_block.conf1();
            }
        }
        reg_en.modify(|_, w| w.rx_tout_en().bit(timeout.is_some()));

        self.sync_regs();

        Ok(())
    }

    fn is_instance(&self, other: impl Instance) -> bool {
        self == other.info()
    }

    fn sync_regs(&self) {
        sync_regs(self.regs());
    }

    fn change_baud(&self, config: &Config) -> Result<(), ConfigError> {
        let clocks = Clocks::get();
        let clk = match config.clock_source {
            ClockSource::Apb => clocks.apb_clock.as_hz(),
            #[cfg(not(any(esp32, esp32s2)))]
            ClockSource::Xtal => clocks.xtal_clock.as_hz(),
            #[cfg(not(any(esp32, esp32s2)))]
            ClockSource::RcFast => crate::soc::constants::RC_FAST_CLK.as_hz(),
            #[cfg(any(esp32, esp32s2))]
            ClockSource::RefTick => crate::soc::constants::REF_TICK.as_hz(),
        };

        cfg_if::cfg_if! {
            if #[cfg(any(esp32c2, esp32c3, esp32s3, esp32c6, esp32h2))] {

                const MAX_DIV: u32 = 0b1111_1111_1111 - 1;
                let clk_div = (clk.div_ceil(MAX_DIV)).div_ceil(config.baudrate);

                // define `conf` in scope for modification below
                cfg_if::cfg_if! {
                    if #[cfg(any(esp32c2, esp32c3, esp32s3))] {
                        if matches!(config.clock_source, ClockSource::RcFast) {
                            crate::peripherals::LPWR::regs()
                                .clk_conf()
                                .modify(|_, w| w.dig_clk8m_en().variant(true));
                            // small delay whilst the clock source changes (SOC_DELAY_RC_FAST_DIGI_SWITCH from esp-idf)
                            crate::rom::ets_delay_us(5);
                        }

                        let conf = self.regs().clk_conf();
                    } else {
                        // UART clocks are configured via PCR
                        let pcr = crate::peripherals::PCR::regs();
                        let conf = if self.is_instance(unsafe { crate::peripherals::UART0::steal() }) {
                            pcr.uart(0).clk_conf()
                        } else {
                            pcr.uart(1).clk_conf()
                        };
                    }
                };

                conf.write(|w| unsafe {
                    w.sclk_sel().bits(match config.clock_source {
                        ClockSource::Apb => 1,
                        ClockSource::RcFast => 2,
                        ClockSource::Xtal => 3,
                    });
                    w.sclk_div_a().bits(0);
                    w.sclk_div_b().bits(0);
                    w.sclk_div_num().bits(clk_div as u8 - 1)
                });

                let divider = (clk << 4) / (config.baudrate * clk_div);
            } else {
                self.regs().conf0().modify(|_, w| {
                    w.tick_ref_always_on()
                        .bit(config.clock_source == ClockSource::Apb)
                });

                let divider = (clk << 4) / config.baudrate;
            }
        }

        let divider_integer = divider >> 4;
        let divider_frag = (divider & 0xf) as u8;

        self.regs().clkdiv().write(|w| unsafe {
            w.clkdiv()
                .bits(divider_integer as _)
                .frag()
                .bits(divider_frag)
        });

        self.sync_regs();

        #[cfg(feature = "unstable")]
        self.verify_baudrate(clk, config)?;

        Ok(())
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
        #[cfg(esp32)]
        {
            // workaround for hardware issue, when UART stop bit set as 2-bit mode.
            if stop_bits == StopBits::_2 {
                self.regs()
                    .rs485_conf()
                    .modify(|_, w| w.dl1_en().bit(stop_bits == StopBits::_2));

                self.regs()
                    .conf0()
                    .modify(|_, w| unsafe { w.stop_bit_num().bits(1) });
            }
        }

        #[cfg(not(esp32))]
        self.regs()
            .conf0()
            .modify(|_, w| unsafe { w.stop_bit_num().bits(stop_bits as u8 + 1) });
    }

    fn rxfifo_reset(&self) {
        fn rxfifo_rst(reg_block: &RegisterBlock, enable: bool) {
            reg_block.conf0().modify(|_, w| w.rxfifo_rst().bit(enable));
            sync_regs(reg_block);
        }

        rxfifo_rst(self.regs(), true);
        rxfifo_rst(self.regs(), false);
    }

    fn txfifo_reset(&self) {
        fn txfifo_rst(reg_block: &RegisterBlock, enable: bool) {
            reg_block.conf0().modify(|_, w| w.txfifo_rst().bit(enable));
            sync_regs(reg_block);
        }

        txfifo_rst(self.regs(), true);
        txfifo_rst(self.regs(), false);
    }

    #[cfg(feature = "unstable")]
    fn verify_baudrate(&self, clk: u32, config: &Config) -> Result<(), ConfigError> {
        // taken from https://github.com/espressif/esp-idf/blob/c5865270b50529cd32353f588d8a917d89f3dba4/components/hal/esp32c6/include/hal/uart_ll.h#L433-L444
        // (it's different for different chips)
        let clkdiv_reg = self.regs().clkdiv().read();
        let clkdiv_frag = clkdiv_reg.frag().bits() as u32;
        let clkdiv = clkdiv_reg.clkdiv().bits();

        cfg_if::cfg_if! {
            if #[cfg(any(esp32, esp32s2))] {
                let actual_baud = (clk << 4) / ((clkdiv << 4) | clkdiv_frag);
            } else if #[cfg(any(esp32c2, esp32c3, esp32s3))] {
                let sclk_div_num = self.regs().clk_conf().read().sclk_div_num().bits() as u32;
                let actual_baud = (clk << 4) / ((((clkdiv as u32) << 4) | clkdiv_frag) * (sclk_div_num + 1));
            } else { // esp32c6, esp32h2
                let pcr = crate::peripherals::PCR::regs();
                let conf = if self.is_instance(unsafe { crate::peripherals::UART0::steal() }) {
                    pcr.uart(0).clk_conf()
                } else {
                    pcr.uart(1).clk_conf()
                };
                let sclk_div_num = conf.read().sclk_div_num().bits() as u32;
                let actual_baud = (clk << 4) / ((((clkdiv as u32) << 4) | clkdiv_frag) * (sclk_div_num + 1));
            }
        };

        match config.baudrate_tolerance {
            BaudrateTolerance::Exact => {
                let deviation = ((config.baudrate as i32 - actual_baud as i32).unsigned_abs()
                    * 100)
                    / actual_baud;
                // We tolerate deviation of 1% from the desired baud value, as it never will be
                // exactly the same
                if deviation > 1_u32 {
                    return Err(ConfigError::UnachievableBaudrate);
                }
            }
            BaudrateTolerance::ErrorPercent(percent) => {
                let deviation = ((config.baudrate as i32 - actual_baud as i32).unsigned_abs()
                    * 100)
                    / actual_baud;
                if deviation > percent as u32 {
                    return Err(ConfigError::UnachievableBaudrate);
                }
            }
            _ => {}
        }

        Ok(())
    }

    fn current_symbol_length(&self) -> u8 {
        let conf0 = self.regs().conf0().read();
        let data_bits = conf0.bit_num().bits() + 5; // 5 data bits are encoded as variant 0
        let parity = conf0.parity_en().bit() as u8;
        let mut stop_bits = conf0.stop_bit_num().bits();

        match stop_bits {
            1 => {
                // workaround for hardware issue, when UART stop bit set as 2-bit mode.
                #[cfg(esp32)]
                if self.regs().rs485_conf().read().dl1_en().bit_is_set() {
                    stop_bits = 2;
                }
            }
            // esp-idf also counts 2 bits for settings 1.5 and 2 stop bits
            _ => stop_bits = 2,
        }

        1 + data_bits + parity + stop_bits
    }

    /// Reads one byte from the RX FIFO.
    ///
    /// If the FIFO is empty, the value of the returned byte is not specified.
    fn read_next_from_fifo(&self) -> u8 {
        fn access_fifo_register<R>(f: impl Fn() -> R) -> R {
            // https://docs.espressif.com/projects/esp-chip-errata/en/latest/esp32/03-errata-description/esp32/cpu-subsequent-access-halted-when-get-interrupted.html
            cfg_if::cfg_if! {
                if #[cfg(esp32)] {
                    crate::interrupt::free(f)
                } else {
                    f()
                }
            }
        }

        let fifo_reg = self.regs().fifo();
        cfg_if::cfg_if! {
            if #[cfg(esp32s2)] {
                // On the ESP32-S2 we need to use PeriBus2 to read the FIFO:
                let fifo_reg = unsafe {
                    &*fifo_reg.as_ptr().cast::<u8>().add(0x20C00000).cast::<crate::pac::uart0::FIFO>()
                };
            }
        }

        access_fifo_register(|| fifo_reg.read().rxfifo_rd_byte().bits())
    }
}

impl PartialEq for Info {
    fn eq(&self, other: &Self) -> bool {
        self.register_block == other.register_block
    }
}

unsafe impl Sync for Info {}

macro_rules! impl_instance {
    ($inst:ident, $peri:ident, $txd:ident, $rxd:ident, $cts:ident, $rts:ident) => {
        impl Instance for crate::peripherals::$inst {
            fn parts(&self) -> (&'static Info, &'static State) {
                #[crate::handler]
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
                    async_handler: irq_handler,
                    interrupt: Interrupt::$inst,
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

impl_instance!(UART0, Uart0, U0TXD, U0RXD, U0CTS, U0RTS);
impl_instance!(UART1, Uart1, U1TXD, U1RXD, U1CTS, U1RTS);
#[cfg(uart2)]
impl_instance!(UART2, Uart2, U2TXD, U2RXD, U2CTS, U2RTS);

crate::any_peripheral! {
    /// Any UART peripheral.
    pub peripheral AnyUart {
        #[cfg(uart0)]
        Uart0(crate::peripherals::UART0),
        #[cfg(uart1)]
        Uart1(crate::peripherals::UART1),
        #[cfg(uart2)]
        Uart2(crate::peripherals::UART2),
    }
}

impl Instance for AnyUart {
    #[inline]
    fn parts(&self) -> (&'static Info, &'static State) {
        match &self.0 {
            #[cfg(uart0)]
            AnyUartInner::Uart0(uart) => uart.parts(),
            #[cfg(uart1)]
            AnyUartInner::Uart1(uart) => uart.parts(),
            #[cfg(uart2)]
            AnyUartInner::Uart2(uart) => uart.parts(),
        }
    }
}

struct OnDrop<F: FnOnce()>(Option<F>);
impl<F: FnOnce()> OnDrop<F> {
    fn new(cb: F) -> Self {
        Self(Some(cb))
    }
}

impl<F: FnOnce()> Drop for OnDrop<F> {
    fn drop(&mut self) {
        if let Some(cb) = self.0.take() {
            cb();
        }
    }
}
