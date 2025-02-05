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
//! ## Example
//!
//! ### Handling UART Interrupts
//! Notice, that in practice a proper serial terminal should be used
//! to connect to the board (espmonitor and espflash won't work)
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::delay::Delay;
//! # use esp_hal::uart::{AtCmdConfig, Config, RxConfig, Uart, UartInterrupt};
//! # let delay = Delay::new();
//! # let config = Config::default().with_rx(
//! #    RxConfig::default().with_fifo_full_threshold(30)
//! # );
//! # let mut uart0 = Uart::new(
//! #    peripherals.UART0,
//! #    config)?;
//! uart0.set_interrupt_handler(interrupt_handler);
//!
//! critical_section::with(|cs| {
//!     uart0.set_at_cmd(AtCmdConfig::default().with_cmd_char(b'#'));
//!     uart0.listen(UartInterrupt::AtCmd | UartInterrupt::RxFifoFull);
//!
//!     SERIAL.borrow_ref_mut(cs).replace(uart0);
//! });
//!
//! loop {
//!     println!("Send `#` character or >=30 characters");
//!     delay.delay(Duration::from_secs(1));
//! }
//! # }
//!
//! # use core::cell::RefCell;
//! # use critical_section::Mutex;
//! # use esp_hal::uart::Uart;
//! static SERIAL: Mutex<RefCell<Option<Uart<esp_hal::Blocking>>>> =
//!     Mutex::new(RefCell::new(None));
//!
//! # use esp_hal::uart::UartInterrupt;
//! # use core::fmt::Write;
//! #[handler]
//! fn interrupt_handler() {
//!     critical_section::with(|cs| {
//!         let mut serial = SERIAL.borrow_ref_mut(cs);
//!         if let Some(serial) = serial.as_mut() {
//!             let mut buf = [0u8; 64];
//!             if let Ok(cnt) = serial.read_buffered_bytes(&mut buf) {
//!                 println!("Read {} bytes", cnt);
//!             }
//!
//!             let pending_interrupts = serial.interrupts();
//!             println!(
//!                 "Interrupt AT-CMD: {} RX-FIFO-FULL: {}",
//!                 pending_interrupts.contains(UartInterrupt::AtCmd),
//!                 pending_interrupts.contains(UartInterrupt::RxFifoFull),
//!             );
//!
//!             serial.clear_interrupts(
//!                 UartInterrupt::AtCmd | UartInterrupt::RxFifoFull
//!             );
//!         }
//!     });
//! }
//! ```
//! 
//! [embedded-hal]: embedded_hal
//! [embedded-io]: embedded_io
//! [embedded-hal-async]: embedded_hal_async
//! [embedded-io-async]: embedded_io_async

use core::{marker::PhantomData, sync::atomic::Ordering, task::Poll};

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

const UART_FIFO_SIZE: u16 = 128;
const CMD_CHAR_DEFAULT: u8 = 0x2b;

/// UART Error
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
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

impl core::error::Error for Error {}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Error::FifoOverflowed => write!(f, "The RX FIFO overflowed"),
            Error::GlitchOccurred => write!(f, "A glitch was detected on the RX line"),
            Error::FrameFormatViolated => write!(f, "A framing error was detected on the RX line"),
            Error::ParityMismatch => write!(f, "A parity error was detected on the RX line"),
        }
    }
}

#[instability::unstable]
impl embedded_io::Error for Error {
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

// see <https://github.com/espressif/esp-idf/blob/8760e6d2a/components/esp_driver_uart/src/uart.c#L61>
const UART_FULL_THRESH_DEFAULT: u16 = 120;
// see <https://github.com/espressif/esp-idf/blob/8760e6d2a/components/esp_driver_uart/src/uart.c#L63>
const UART_TOUT_THRESH_DEFAULT: u8 = 10;

/// Number of data bits
///
/// This enum represents the various configurations for the number of data
/// bits used in UART communication. The number of data bits defines the
/// length of each transmitted or received data frame.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DataBits {
    /// 5 data bits per frame.
    _5 = 0,
    /// 6 data bits per frame.
    _6 = 1,
    /// 7 data bits per frame.
    _7 = 2,
    /// 8 data bits per frame.
    #[default]
    _8 = 3,
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
    _1   = 1,
    /// 1.5 stop bits.
    _1p5 = 2,
    /// 2 stop bits.
    _2   = 3,
}

/// UART Configuration
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
    #[cfg_attr(not(feature = "unstable"), builder_lite(skip))]
    clock_source: ClockSource,
    /// UART Receive part configuration.
    rx: RxConfig,
    /// UART Transmit part configuration.
    tx: TxConfig,
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

impl Default for Config {
    fn default() -> Config {
        Config {
            rx: RxConfig::default(),
            tx: TxConfig::default(),
            baudrate: 115_200,
            data_bits: Default::default(),
            parity: Default::default(),
            stop_bits: Default::default(),
            clock_source: Default::default(),
        }
    }
}

/// UART Transmit part configuration.
#[derive(Debug, Clone, Copy, Default, procmacros::BuilderLite)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct TxConfig {}

impl Default for RxConfig {
    fn default() -> RxConfig {
        RxConfig {
            fifo_full_threshold: UART_FULL_THRESH_DEFAULT,
            timeout: Some(UART_TOUT_THRESH_DEFAULT),
        }
    }
}

impl Config {
    /// Calculates the total symbol length in bits based on the configured
    /// data bits, parity, and stop bits.
    fn symbol_length(&self) -> u8 {
        let mut length: u8 = 1; // start bit
        length += match self.data_bits {
            DataBits::_5 => 5,
            DataBits::_6 => 6,
            DataBits::_7 => 7,
            DataBits::_8 => 8,
        };
        length += match self.parity {
            Parity::None => 0,
            _ => 1,
        };
        length += match self.stop_bits {
            StopBits::_1 => 1,
            _ => 2, // esp-idf also counts 2 bits for settings 1.5 and 2 stop bits
        };
        length
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
            cmd_char: CMD_CHAR_DEFAULT,
            char_num: 1,
        }
    }
}

struct UartBuilder<'d, Dm> {
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
pub struct Uart<'d, Dm> {
    rx: UartRx<'d, Dm>,
    tx: UartTx<'d, Dm>,
}

/// UART (Transmit)
pub struct UartTx<'d, Dm> {
    uart: PeripheralRef<'d, AnyUart>,
    phantom: PhantomData<Dm>,
    guard: PeripheralGuard,
    rts_pin: PinGuard,
    tx_pin: PinGuard,
}

/// UART (Receive)
pub struct UartRx<'d, Dm> {
    uart: PeripheralRef<'d, AnyUart>,
    phantom: PhantomData<Dm>,
    guard: PeripheralGuard,
}

/// A configuration error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum ConfigError {
    /// The requested timeout is not supported.
    UnsupportedTimeout,
    /// The requested FIFO threshold is not supported.
    UnsupportedFifoThreshold,
}

impl core::error::Error for ConfigError {}

impl core::fmt::Display for ConfigError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            ConfigError::UnsupportedTimeout => write!(f, "The requested timeout is not supported"),
            ConfigError::UnsupportedFifoThreshold => {
                write!(f, "The requested FIFO threshold is not supported")
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
    /// Note that this also changes the configuration of the RX half.
    // FIXME: when https://github.com/esp-rs/esp-hal/issues/2839 is resolved, add an appropriate `# Error` entry.
    #[instability::unstable]
    pub fn apply_config(&mut self, _config: &Config) -> Result<(), ConfigError> {
        // Nothing to do so far.
        self.uart.info().txfifo_reset();
        Ok(())
    }

    /// Writes bytes
    pub fn write_bytes(&mut self, data: &[u8]) -> Result<usize, Error> {
        let count = data.len();

        for &byte in data {
            self.write_byte(byte);
        }

        Ok(count)
    }

    fn write_byte(&mut self, word: u8) {
        while self.tx_fifo_count() >= UART_FIFO_SIZE {}
        self.regs()
            .fifo()
            .write(|w| unsafe { w.rxfifo_rd_byte().bits(word) });
    }

    #[allow(clippy::useless_conversion)]
    /// Returns the number of bytes currently in the TX FIFO for this UART
    /// instance.
    fn tx_fifo_count(&self) -> u16 {
        self.regs().status().read().txfifo_cnt().bits().into()
    }

    /// Flush the transmit buffer of the UART
    pub fn flush(&mut self) {
        while !self.is_tx_idle() {}
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
    pub fn new(
        uart: impl Peripheral<P = impl Instance> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        let (_, uart_tx) = UartBuilder::new(uart).init(config)?.split();

        Ok(uart_tx)
    }

    /// Reconfigures the driver to operate in [`Async`] mode.
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
    pub fn with_rx(self, rx: impl Peripheral<P = impl PeripheralInput> + 'd) -> Self {
        crate::into_mapped_ref!(rx);
        rx.init_input(Pull::Up);
        self.uart.info().rx_signal.connect_to(rx);

        self
    }

    /// Change the configuration.
    ///
    /// Note that this also changes the configuration of the TX half.
    // FIXME: when https://github.com/esp-rs/esp-hal/issues/2839 is resolved, add an appropriate `# Error` entry.
    #[instability::unstable]
    pub fn apply_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        self.uart
            .info()
            .set_rx_fifo_full_threshold(config.rx.fifo_full_threshold)?;
        self.uart
            .info()
            .set_rx_timeout(config.rx.timeout, config.symbol_length())?;

        self.uart.info().rxfifo_reset();
        Ok(())
    }

    /// Reads and clears errors.
    #[instability::unstable]
    pub fn check_for_errors(&mut self) -> Result<(), Error> {
        let errors = RxEvent::FifoOvf
            | RxEvent::FifoTout
            | RxEvent::GlitchDetected
            | RxEvent::FrameError
            | RxEvent::ParityError;
        let events = self.uart.info().rx_events(errors);
        let result = rx_event_check_for_error(events);
        if result.is_err() {
            self.uart.info().clear_rx_events(errors);
        }
        result
    }

    // Read a byte from the UART
    fn read_byte(&mut self) -> Option<u8> {
        cfg_if::cfg_if! {
            if #[cfg(esp32s2)] {
                // On the ESP32-S2 we need to use PeriBus2 to read the FIFO:
                let fifo = unsafe {
                    &*((self.regs().fifo().as_ptr() as *mut u8).add(0x20C00000)
                        as *mut crate::pac::uart0::FIFO)
                };
            } else {
                let fifo = self.regs().fifo();
            }
        }

        if self.rx_fifo_count() > 0 {
            // https://docs.espressif.com/projects/esp-chip-errata/en/latest/esp32/03-errata-description/esp32/cpu-subsequent-access-halted-when-get-interrupted.html
            cfg_if::cfg_if! {
                if #[cfg(esp32)] {
                    let byte = crate::interrupt::free(|| fifo.read().rxfifo_rd_byte().bits());
                } else {
                    let byte = fifo.read().rxfifo_rd_byte().bits();
                }
            }

            Some(byte)
        } else {
            None
        }
    }

    /// Reads bytes from the UART
    pub fn read_bytes(&mut self, buf: &mut [u8]) -> Result<(), Error> {
        let buffered = self.read_buffered_bytes(buf)?;
        let buf = &mut buf[buffered..];

        for byte in buf.iter_mut() {
            loop {
                if let Some(b) = self.read_byte() {
                    *byte = b;
                    break;
                }
                self.check_for_errors()?;
            }
        }
        Ok(())
    }

    /// Read all available bytes from the RX FIFO into the provided buffer and
    /// returns the number of read bytes without blocking.
    pub fn read_buffered_bytes(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
        let mut count = 0;
        while count < buf.len() {
            if let Some(byte) = self.read_byte() {
                buf[count] = byte;
                count += 1;
            } else {
                break;
            }
        }
        if let Err(err) = self.check_for_errors() {
            // Drain the buffer. We don't know where the error occurred, so returning
            // these bytes would be incorrect. We also don't know if the number of buffered
            // bytes fit into the buffer.
            // TODO: make this behaviour configurable using UART_ERR_WR_MASK. If the user
            // wants to keep the bytes regardless of errors, they should be able to do so.
            while self.read_byte().is_some() {}
            return Err(err);
        }
        Ok(count)
    }

    /// Read bytes from the RX FIFO without checking for errors.
    fn flush_buffer(&mut self, buf: &mut [u8]) -> usize {
        let mut count = 0;
        while count < buf.len() {
            if let Some(byte) = self.read_byte() {
                buf[count] = byte;
                count += 1;
            } else {
                break;
            }
        }
        count
    }

    #[allow(clippy::useless_conversion)]
    fn rx_fifo_count(&self) -> u16 {
        let fifo_cnt: u16 = self.regs().status().read().rxfifo_cnt().bits().into();

        // Calculate the real count based on the FIFO read and write offset address:
        // https://www.espressif.com/sites/default/files/documentation/esp32_errata_en.pdf
        // section 3.17
        #[cfg(esp32)]
        {
            let status = self.regs().mem_rx_status().read();
            let rd_addr = status.mem_rx_rd_addr().bits();
            let wr_addr = status.mem_rx_wr_addr().bits();

            if wr_addr > rd_addr {
                wr_addr - rd_addr
            } else if wr_addr < rd_addr {
                (wr_addr + UART_FIFO_SIZE) - rd_addr
            } else if fifo_cnt > 0 {
                UART_FIFO_SIZE
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
    pub fn new(
        uart: impl Peripheral<P = impl Instance> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        let (uart_rx, _) = UartBuilder::new(uart).init(config)?.split();

        Ok(uart_rx)
    }

    /// Reconfigures the driver to operate in [`Async`] mode.
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
    // FIXME: when https://github.com/esp-rs/esp-hal/issues/2839 is resolved, add an appropriate `# Error` entry.
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
    /// ## Example
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
    /// tx.write_bytes(&[42u8])?;
    /// let mut byte = [0u8; 1];
    /// rx.read_bytes(&mut byte);
    /// # Ok(())
    /// # }
    /// ```
    pub fn split(self) -> (UartRx<'d, Dm>, UartTx<'d, Dm>) {
        (self.rx, self.tx)
    }

    /// Write bytes out over the UART
    /// ```rust, no_run
    #[doc = crate::before_snippet!()]
    /// # use esp_hal::uart::{Config, Uart};
    /// # let mut uart1 = Uart::new(
    /// #     peripherals.UART1,
    /// #     Config::default())?;
    /// // Write bytes out over the UART:
    /// uart1.write_bytes(b"Hello, world!")?;
    /// # Ok(())
    /// # }
    /// ```
    pub fn write_bytes(&mut self, data: &[u8]) -> Result<usize, Error> {
        self.tx.write_bytes(data)
    }

    /// Reads and clears errors set by received data.
    #[instability::unstable]
    pub fn check_for_rx_errors(&mut self) -> Result<(), Error> {
        self.rx.check_for_errors()
    }

    /// Reads bytes from the UART
    pub fn read_bytes(&mut self, buf: &mut [u8]) -> Result<(), Error> {
        self.rx.read_bytes(buf)
    }

    /// Read all available bytes from the RX FIFO into the provided buffer and
    /// returns the number of read bytes without blocking.
    pub fn read_buffered_bytes(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
        self.rx.read_buffered_bytes(buf)
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

    /// Flush the transmit buffer of the UART
    pub fn flush(&mut self) {
        self.tx.flush()
    }

    /// Change the configuration.
    // FIXME: when https://github.com/esp-rs/esp-hal/issues/2839 is resolved, add an appropriate `# Error` entry.
    pub fn apply_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        self.rx.apply_config(config)?;
        self.tx.apply_config(config)?;
        self.rx.uart.info().apply_config(config)?;
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
    type Error = Error;

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
    type Error = Error;

    #[inline]
    fn write_str(&mut self, s: &str) -> Result<(), Self::Error> {
        self.write_bytes(s.as_bytes())?;
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
        self.write_bytes(s.as_bytes())
            .map_err(|_| core::fmt::Error)?;
        Ok(())
    }
}

#[instability::unstable]
impl<Dm> embedded_io::ErrorType for Uart<'_, Dm> {
    type Error = Error;
}

#[instability::unstable]
impl<Dm> embedded_io::ErrorType for UartTx<'_, Dm> {
    type Error = Error;
}

#[instability::unstable]
impl<Dm> embedded_io::ErrorType for UartRx<'_, Dm> {
    type Error = Error;
}

#[instability::unstable]
impl<Dm> embedded_io::Read for Uart<'_, Dm>
where
    Dm: DriverMode,
{
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.rx.read(buf)
    }
}

#[instability::unstable]
impl<Dm> embedded_io::Read for UartRx<'_, Dm>
where
    Dm: DriverMode,
{
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        if buf.is_empty() {
            return Ok(0);
        }

        while self.rx_fifo_count() == 0 {
            // Block until we received at least one byte
        }

        self.read_buffered_bytes(buf)
    }
}

#[instability::unstable]
impl<Dm> embedded_io::ReadReady for Uart<'_, Dm>
where
    Dm: DriverMode,
{
    fn read_ready(&mut self) -> Result<bool, Self::Error> {
        self.rx.read_ready()
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
        self.tx.write(buf)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        embedded_io::Write::flush(&mut self.tx)
    }
}

#[instability::unstable]
impl<Dm> embedded_io::Write for UartTx<'_, Dm>
where
    Dm: DriverMode,
{
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.write_bytes(buf)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        self.flush();
        Ok(())
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
    CmdCharDetected,
    FifoOvf,
    FifoTout,
    GlitchDetected,
    FrameError,
    ParityError,
}

fn rx_event_check_for_error(events: EnumSet<RxEvent>) -> Result<(), Error> {
    for event in events {
        match event {
            RxEvent::FifoOvf => return Err(Error::FifoOverflowed),
            RxEvent::GlitchDetected => return Err(Error::GlitchOccurred),
            RxEvent::FrameError => return Err(Error::FrameFormatViolated),
            RxEvent::ParityError => return Err(Error::ParityMismatch),
            RxEvent::FifoFull | RxEvent::CmdCharDetected | RxEvent::FifoTout => continue,
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
        if !self.registered {
            self.state.rx_waker.register(cx.waker());
            self.uart.enable_listen_rx(self.events, true);
            self.registered = true;
        }
        let events = self.uart.enabled_rx_events(self.events);
        if !events.is_empty() {
            Poll::Ready(events)
        } else {
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

    fn triggered_events(&self) -> bool {
        let interrupts_enabled = self.uart.regs().int_ena().read();
        let mut event_triggered = false;
        for event in self.events {
            event_triggered |= match event {
                TxEvent::Done => interrupts_enabled.tx_done().bit_is_clear(),
                TxEvent::FiFoEmpty => interrupts_enabled.txfifo_empty().bit_is_clear(),
            }
        }
        event_triggered
    }

    fn enable_listen(&self, enable: bool) {
        self.uart.regs().int_ena().modify(|_, w| {
            for event in self.events {
                match event {
                    TxEvent::Done => w.tx_done().bit(enable),
                    TxEvent::FiFoEmpty => w.txfifo_empty().bit(enable),
                };
            }
            w
        });
    }
}

impl core::future::Future for UartTxFuture {
    type Output = ();

    fn poll(
        mut self: core::pin::Pin<&mut Self>,
        cx: &mut core::task::Context<'_>,
    ) -> core::task::Poll<Self::Output> {
        if !self.registered {
            self.state.tx_waker.register(cx.waker());
            self.enable_listen(true);
            self.registered = true;
        }

        if self.triggered_events() {
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    }
}

impl Drop for UartTxFuture {
    fn drop(&mut self) {
        // Although the isr disables the interrupt that occurred directly, we need to
        // disable the other interrupts (= the ones that did not occur), as
        // soon as this future goes out of scope.
        self.enable_listen(false);
    }
}

impl Uart<'_, Async> {
    /// Asynchronously reads data from the UART receive buffer into the
    /// provided buffer.
    pub async fn read_async(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
        self.rx.read_async(buf).await
    }

    /// Asynchronously writes data to the UART transmit buffer.
    pub async fn write_async(&mut self, words: &[u8]) -> Result<usize, Error> {
        self.tx.write_async(words).await
    }

    /// Asynchronously flushes the UART transmit buffer.
    pub async fn flush_async(&mut self) -> Result<(), Error> {
        self.tx.flush_async().await
    }
}

impl UartTx<'_, Async> {
    /// Asynchronously writes data to the UART transmit buffer in chunks.
    ///
    /// This function sends the contents of the provided buffer `words` over
    /// the UART. Data is written in chunks to avoid overflowing the
    /// transmit FIFO, and the function waits asynchronously when
    /// necessary for space in the buffer to become available.
    pub async fn write_async(&mut self, words: &[u8]) -> Result<usize, Error> {
        let mut count = 0;
        let mut offset: usize = 0;
        loop {
            let mut next_offset = offset + (UART_FIFO_SIZE - self.tx_fifo_count()) as usize;
            if next_offset > words.len() {
                next_offset = words.len();
            }

            for byte in &words[offset..next_offset] {
                self.write_byte(*byte);
                count += 1;
            }

            if next_offset >= words.len() {
                break;
            }

            offset = next_offset;
            UartTxFuture::new(self.uart.reborrow(), TxEvent::FiFoEmpty).await;
        }

        Ok(count)
    }

    /// Asynchronously flushes the UART transmit buffer.
    ///
    /// This function ensures that all pending data in the transmit FIFO has
    /// been sent over the UART. If the FIFO contains data, it waits
    /// for the transmission to complete before returning.
    pub async fn flush_async(&mut self) -> Result<(), Error> {
        let count = self.tx_fifo_count();
        if count > 0 {
            UartTxFuture::new(self.uart.reborrow(), TxEvent::Done).await;
        }

        Ok(())
    }
}

impl UartRx<'_, Async> {
    /// Read async to buffer slice `buf`.
    /// Waits until at least one byte is in the Rx FiFo
    /// and one of the following interrupts occurs:
    /// - `RXFIFO_FULL`
    /// - `RXFIFO_OVF`
    /// - `AT_CMD_CHAR_DET` (only if `set_at_cmd` was called)
    /// - `RXFIFO_TOUT` (only if `set_rx_timeout` was called)
    ///
    /// The interrupts in question are enabled during the body of this
    /// function. The method immediately returns when the interrupt
    /// has already occurred before calling this method (e.g. status
    /// bit set, but interrupt not enabled)
    ///
    /// # Params
    /// - `buf` buffer slice to write the bytes into
    ///
    ///
    /// # Ok
    /// When successful, returns the number of bytes written to buf.
    /// If the passed in buffer is of length 0, Ok(0) is returned.
    pub async fn read_async(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
        if buf.is_empty() {
            return Ok(0);
        }

        loop {
            let mut events = RxEvent::FifoFull
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
            if reg_en.read().rx_tout_en().bit_is_set() {
                events |= RxEvent::FifoTout;
            }

            let events_happened = UartRxFuture::new(self.uart.reborrow(), events).await;
            // always drain the fifo, if an error has occurred the data is lost
            let read_bytes = self.flush_buffer(buf);
            // check error events
            rx_event_check_for_error(events_happened)?;
            // Unfortunately, the uart's rx-timeout counter counts up whenever there is
            // data in the fifo, even if the interrupt is disabled and the status bit
            // cleared. Since we do not drain the fifo in the interrupt handler, we need to
            // reset the counter here, after draining the fifo.
            self.regs()
                .int_clr()
                .write(|w| w.rxfifo_tout().clear_bit_by_one());

            if read_bytes > 0 {
                return Ok(read_bytes);
            }
        }
    }
}

#[instability::unstable]
impl embedded_io_async::Read for Uart<'_, Async> {
    /// In contrast to the documentation of embedded_io_async::Read, this
    /// method blocks until an uart interrupt occurs.
    /// See UartRx::read_async for more details.
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.read_async(buf).await
    }
}

#[instability::unstable]
impl embedded_io_async::Read for UartRx<'_, Async> {
    /// In contrast to the documentation of embedded_io_async::Read, this
    /// method blocks until an uart interrupt occurs.
    /// See UartRx::read_async for more details.
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.read_async(buf).await
    }
}

#[instability::unstable]
impl embedded_io_async::Write for Uart<'_, Async> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.write_async(buf).await
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        self.flush_async().await
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
        || interrupts.rxfifo_ovf().bit_is_set()
        || interrupts.rxfifo_tout().bit_is_set()
        || interrupts.at_cmd_char_det().bit_is_set()
        || interrupts.glitch_det().bit_is_set()
        || interrupts.frm_err().bit_is_set()
        || interrupts.parity_err().bit_is_set();
    let tx_wake = interrupts.tx_done().bit_is_set() || interrupts.txfifo_empty().bit_is_set();
    uart.regs()
        .int_clr()
        .write(|w| unsafe { w.bits(interrupt_bits) });
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
                .modify(|_, w| unsafe { w.stop_bit_num().bits(stop_bits as u8) });

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
                };
            }
            w
        });
    }

    fn set_interrupt_handler(&self, handler: InterruptHandler) {
        for core in crate::Cpu::other() {
            crate::interrupt::disable(core, self.interrupt);
        }
        self.enable_listen(EnumSet::all(), false);
        self.clear_interrupts(EnumSet::all());
        unsafe { crate::interrupt::bind_interrupt(self.interrupt, handler.handler()) };
        unwrap!(crate::interrupt::enable(self.interrupt, handler.priority()));
    }

    fn disable_interrupts(&self) {
        crate::interrupt::disable(crate::Cpu::current(), self.interrupt);
    }

    fn apply_config(&self, config: &Config) -> Result<(), ConfigError> {
        self.change_baud(config);
        self.change_data_bits(config.data_bits);
        self.change_parity(config.parity);
        self.change_stop_bits(config.stop_bits);

        // Reset Tx/Rx FIFOs
        self.rxfifo_reset();
        self.txfifo_reset();

        Ok(())
    }

    fn enable_listen_rx(&self, events: EnumSet<RxEvent>, enable: bool) {
        self.regs().int_ena().modify(|_, w| {
            for event in events {
                match event {
                    RxEvent::FifoFull => w.rxfifo_full().bit(enable),
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

    fn enabled_rx_events(&self, events: impl Into<EnumSet<RxEvent>>) -> EnumSet<RxEvent> {
        let events = events.into();
        let interrupts_enabled = self.regs().int_ena().read();
        let mut events_triggered = EnumSet::new();
        for event in events {
            let event_triggered = match event {
                RxEvent::FifoFull => interrupts_enabled.rxfifo_full().bit_is_clear(),
                RxEvent::CmdCharDetected => interrupts_enabled.at_cmd_char_det().bit_is_clear(),

                RxEvent::FifoOvf => interrupts_enabled.rxfifo_ovf().bit_is_clear(),
                RxEvent::FifoTout => interrupts_enabled.rxfifo_tout().bit_is_clear(),
                RxEvent::GlitchDetected => interrupts_enabled.glitch_det().bit_is_clear(),
                RxEvent::FrameError => interrupts_enabled.frm_err().bit_is_clear(),
                RxEvent::ParityError => interrupts_enabled.parity_err().bit_is_clear(),
            };
            if event_triggered {
                events_triggered |= event;
            }
        }
        events_triggered
    }

    fn rx_events(&self, events: impl Into<EnumSet<RxEvent>>) -> EnumSet<RxEvent> {
        let events = events.into();
        let interrupts_enabled = self.regs().int_st().read();
        let mut events_triggered = EnumSet::new();
        for event in events {
            let event_triggered = match event {
                RxEvent::FifoFull => interrupts_enabled.rxfifo_full().bit_is_set(),
                RxEvent::CmdCharDetected => interrupts_enabled.at_cmd_char_det().bit_is_set(),

                RxEvent::FifoOvf => interrupts_enabled.rxfifo_ovf().bit_is_set(),
                RxEvent::FifoTout => interrupts_enabled.rxfifo_tout().bit_is_set(),
                RxEvent::GlitchDetected => interrupts_enabled.glitch_det().bit_is_set(),
                RxEvent::FrameError => interrupts_enabled.frm_err().bit_is_set(),
                RxEvent::ParityError => interrupts_enabled.parity_err().bit_is_set(),
            };
            if event_triggered {
                events_triggered |= event;
            }
        }
        events_triggered
    }

    fn clear_rx_events(&self, events: impl Into<EnumSet<RxEvent>>) {
        let events = events.into();
        self.regs().int_clr().write(|w| {
            for event in events {
                match event {
                    RxEvent::FifoFull => w.rxfifo_full().clear_bit_by_one(),
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
    /// # Errors
    /// [`Err(ConfigError::UnsupportedFifoThreshold)`][ConfigError::UnsupportedFifoThreshold] if provided value exceeds maximum value
    /// for SOC :
    /// - `esp32` **0x7F**
    /// - `esp32c6`, `esp32h2` **0xFF**
    /// - `esp32c3`, `esp32c2`, `esp32s2` **0x1FF**
    /// - `esp32s3` **0x3FF**
    fn set_rx_fifo_full_threshold(&self, threshold: u16) -> Result<(), ConfigError> {
        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
                const MAX_THRHD: u16 = 0x7F;
            } else if #[cfg(any(esp32c6, esp32h2))] {
                const MAX_THRHD: u16 = 0xFF;
            } else if #[cfg(any(esp32c3, esp32c2, esp32s2))] {
                const MAX_THRHD: u16 = 0x1FF;
            } else if #[cfg(esp32s3)] {
                const MAX_THRHD: u16 = 0x3FF;
            }
        }

        if threshold > MAX_THRHD {
            return Err(ConfigError::UnsupportedFifoThreshold);
        }

        self.regs()
            .conf1()
            .modify(|_, w| unsafe { w.rxfifo_full_thrhd().bits(threshold as _) });

        Ok(())
    }

    /// Configures the Receive Timeout detection setting
    ///
    /// # Arguments
    /// `timeout` - the number of symbols ("bytes") to wait for before
    /// triggering a timeout. Pass None to disable the timeout.
    ///
    ///  # Errors
    /// [`Err(ConfigError::UnsupportedTimeout)`][ConfigError::UnsupportedTimeout] if the provided value exceeds
    /// the maximum value for SOC :
    /// - `esp32`: Symbol size is fixed to 8, do not pass a value > **0x7F**.
    /// - `esp32c2`, `esp32c3`, `esp32c6`, `esp32h2`, esp32s2`, esp32s3`: The
    ///   value you pass times the symbol size must be <= **0x3FF**
    // TODO: the above should be a per-chip doc line.
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

    fn change_baud(&self, config: &Config) {
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

                let max_div = 0b1111_1111_1111u64 - 1;
                // this operation may exceed u32::MAX at high bauds, convert to u64
                let clk_div = (clk as u64).div_ceil(max_div * config.baudrate as u64);
                let clk_div = clk_div as u32;

                // define `conf` in scope for modification below
                cfg_if::cfg_if! {
                    if #[cfg(any(esp32c2, esp32c3, esp32s3))] {
                        if matches!(config.clock_source, ClockSource::RcFast) {
                            crate::peripherals::LPWR::regs()
                                .clk_conf()
                                .modify(|_, w| w.dig_clk8m_en().variant(true));
                            // esp_rom_delay_us(SOC_DELAY_RC_FAST_DIGI_SWITCH);
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

        self.regs()
            .clkdiv()
            .write(|w| unsafe { w.clkdiv().bits(divider_integer as _).frag().bits(divider_frag) });

        self.sync_regs();
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
            .modify(|_, w| unsafe { w.stop_bit_num().bits(stop_bits as u8) });
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
