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
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::uart::{Config, Uart};
//!
//! let mut uart1 = Uart::new(
//!     peripherals.UART1,
//!     Config::default(),
//!     peripherals.GPIO1,
//!     peripherals.GPIO2,
//! ).unwrap();
//! # }
//! ```
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
//! ## Examples
//! ### Sending and Receiving Data
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::uart::{Config, Uart};
//! # let mut uart1 = Uart::new(
//! #     peripherals.UART1,
//! #     Config::default(),
//! #     peripherals.GPIO1,
//! #     peripherals.GPIO2,
//! # ).unwrap();
//! // Write bytes out over the UART:
//! uart1.write_bytes(b"Hello, world!").expect("write error!");
//! # }
//! ```
//! 
//! ### Splitting the UART into RX and TX Components
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::uart::{Config, Uart};
//! # let mut uart1 = Uart::new(
//! #     peripherals.UART1,
//! #     Config::default(),
//! #     peripherals.GPIO1,
//! #     peripherals.GPIO2,
//! # ).unwrap();
//! // The UART can be split into separate Transmit and Receive components:
//! let (mut rx, mut tx) = uart1.split();
//!
//! // Each component can be used individually to interact with the UART:
//! tx.write_bytes(&[42u8]).expect("write error!");
//! let byte = rx.read_byte().expect("read error!");
//! # }
//! ```
//! 
//! ### Inverting RX and TX Pins
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::uart::{Config, Uart};
//!
//! let (rx, _) = peripherals.GPIO2.split();
//! let (_, tx) = peripherals.GPIO1.split();
//! let mut uart1 = Uart::new(
//!     peripherals.UART1,
//!     Config::default(),
//!     rx.inverted(),
//!     tx.inverted(),
//! ).unwrap();
//! # }
//! ```
//! 
//! ### Constructing RX and TX Components
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::uart::{Config, UartTx, UartRx};
//!
//! let tx = UartTx::new(
//!     peripherals.UART0,
//!     Config::default(),
//!     peripherals.GPIO1,
//! ).unwrap();
//! let rx = UartRx::new(
//!     peripherals.UART1,
//!     Config::default(),
//!     peripherals.GPIO2,
//! ).unwrap();
//! # }
//! ```
//! 
//! ### Operation with interrupts that by UART/Serial
//! Notice, that in practice a proper serial terminal should be used
//! to connect to the board (espmonitor and espflash won't work)
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::delay::Delay;
//! # use esp_hal::uart::{AtCmdConfig, Config, Uart, UartInterrupt};
//! # use esp_hal::prelude::*;
//! let delay = Delay::new();
//!
//! // Default pins for UART/Serial communication
#![cfg_attr(
    esp32,
    doc = "let (tx_pin, rx_pin) = (peripherals.GPIO1, peripherals.GPIO3);"
)]
#![cfg_attr(
    esp32c2,
    doc = "let (tx_pin, rx_pin) = (peripherals.GPIO20, peripherals.GPIO19);"
)]
#![cfg_attr(
    esp32c3,
    doc = "let (tx_pin, rx_pin) = (peripherals.GPIO21, peripherals.GPIO20);"
)]
#![cfg_attr(
    esp32c6,
    doc = "let (tx_pin, rx_pin) = (peripherals.GPIO16, peripherals.GPIO17);"
)]
#![cfg_attr(
    esp32h2,
    doc = "let (tx_pin, rx_pin) = (peripherals.GPIO24, peripherals.GPIO23);"
)]
#![cfg_attr(
    any(esp32s2, esp32s3),
    doc = "let (tx_pin, rx_pin) = (peripherals.GPIO43, peripherals.GPIO44);"
)]
//! let config = Config::default().rx_fifo_full_threshold(30);
//!
//! let mut uart0 = Uart::new(
//!     peripherals.UART0,
//!     config,
//!     tx_pin,
//!     rx_pin
//! ).unwrap();
//!
//! uart0.set_interrupt_handler(interrupt_handler);
//!
//! critical_section::with(|cs| {
//!     uart0.set_at_cmd(AtCmdConfig::new(None, None, None, b'#', None));
//!     uart0.listen(UartInterrupt::AtCmd | UartInterrupt::RxFifoFull);
//!
//!     SERIAL.borrow_ref_mut(cs).replace(uart0);
//! });
//!
//! loop {
//!     critical_section::with(|cs| {
//!         let mut serial = SERIAL.borrow_ref_mut(cs);
//!         let serial = serial.as_mut().unwrap();
//!         writeln!(serial,
//!             "Hello World! Send a single `#` character or send
//!                 at least 30 characters to trigger interrupts.")
//!         .ok();     
//!     });
//!     delay.delay(1.secs());
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
//!         let serial = serial.as_mut().unwrap();
//!
//!         let mut cnt = 0;
//!         while let nb::Result::Ok(_c) = serial.read_byte() {
//!             cnt += 1;
//!         }
//!         writeln!(serial, "Read {} bytes", cnt).ok();
//!
//!         let pending_interrupts = serial.interrupts();
//!         writeln!(
//!             serial,
//!             "Interrupt AT-CMD: {} RX-FIFO-FULL: {}",
//!             pending_interrupts.contains(UartInterrupt::AtCmd),
//!             pending_interrupts.contains(UartInterrupt::RxFifoFull),
//!         ).ok();
//!
//!         serial.clear_interrupts(
//!             UartInterrupt::AtCmd | UartInterrupt::RxFifoFull
//!         );
//!     });
//! }
//! ```
//! 
//! [embedded-hal]: https://docs.rs/embedded-hal/latest/embedded_hal/
//! [embedded-io]: https://docs.rs/embedded-io/latest/embedded_io/
//! [embedded-hal-async]: https://docs.rs/embedded-hal-async/latest/embedded_hal_async/
//! [embedded-io-async]: https://docs.rs/embedded-io-async/latest/embedded_io_async/

use core::{marker::PhantomData, sync::atomic::Ordering, task::Poll};

#[cfg(any(doc, feature = "unstable"))]
use embassy_embedded_hal::SetConfig;
use enumset::{EnumSet, EnumSetType};
use portable_atomic::AtomicBool;

use crate::{
    asynch::AtomicWaker,
    clock::Clocks,
    gpio::{
        interconnect::{PeripheralInput, PeripheralOutput},
        InputSignal,
        OutputSignal,
        Pull,
    },
    interrupt::{InterruptConfigurable, InterruptHandler},
    peripheral::{Peripheral, PeripheralRef},
    peripherals::{uart0::RegisterBlock, Interrupt},
    private::Internal,
    system::{PeripheralClockControl, PeripheralGuard},
    Async,
    Blocking,
    Mode,
};

const UART_FIFO_SIZE: u16 = 128;

/// UART Error
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// An invalid configuration argument was provided.
    ///
    /// This error occurs when an incorrect or invalid argument is passed during
    /// the configuration of the UART peripheral.
    InvalidArgument,

    /// The RX FIFO overflowed.
    RxFifoOvf,

    /// A glitch was detected on the RX line.
    ///
    /// This error occurs when an unexpected or erroneous signal (glitch) is
    /// detected on the UART RX line, which could lead to incorrect data
    /// reception.
    RxGlitchDetected,

    /// A framing error was detected on the RX line.
    ///
    /// This error occurs when the received data does not conform to the
    /// expected UART frame format.
    RxFrameError,

    /// A parity error was detected on the RX line.
    ///
    /// This error occurs when the parity bit in the received data does not
    /// match the expected parity configuration.
    /// with the `async` feature.
    RxParityError,
}

impl embedded_hal_nb::serial::Error for Error {
    fn kind(&self) -> embedded_hal_nb::serial::ErrorKind {
        embedded_hal_nb::serial::ErrorKind::Other
    }
}

#[cfg(any(doc, feature = "unstable"))]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
impl embedded_io::Error for Error {
    fn kind(&self) -> embedded_io::ErrorKind {
        embedded_io::ErrorKind::Other
    }
}

// (outside of `config` module in order not to "use" it an extra time)
/// UART clock source
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ClockSource {
    /// APB_CLK clock source (default for UART on all the chips except of
    /// esp32c6 and esp32h2)
    #[cfg_attr(not(any(esp32c6, esp32h2, lp_uart)), default)]
    Apb,
    /// RC_FAST_CLK clock source (17.5 MHz)
    #[cfg(not(any(esp32, esp32s2)))]
    RcFast,
    /// XTAL_CLK clock source (default for UART on esp32c6 and esp32h2 and
    /// LP_UART)
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
    DataBits5 = 0,
    /// 6 data bits per frame.
    DataBits6 = 1,
    /// 7 data bits per frame.
    DataBits7 = 2,
    /// 8 data bits per frame (most common).
    #[default]
    DataBits8 = 3,
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
    /// No parity bit is used (most common).
    #[default]
    ParityNone,
    /// Even parity: the parity bit is set to make the total number of
    /// 1-bits even.
    ParityEven,
    /// Odd parity: the parity bit is set to make the total number of 1-bits
    /// odd.
    ParityOdd,
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
    Stop1   = 1,
    /// 1.5 stop bits.
    Stop1P5 = 2,
    /// 2 stop bits.
    Stop2   = 3,
}

/// UART Configuration
#[derive(Debug, Clone, Copy, procmacros::BuilderLite)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Config {
    /// The baud rate (speed) of the UART communication in bits per second
    /// (bps).
    pub baudrate: u32,
    /// Number of data bits in each frame (5, 6, 7, or 8 bits).
    pub data_bits: DataBits,
    /// Parity setting (None, Even, or Odd).
    pub parity: Parity,
    /// Number of stop bits in each frame (1, 1.5, or 2 bits).
    pub stop_bits: StopBits,
    /// Clock source used by the UART peripheral.
    pub clock_source: ClockSource,
    /// Threshold level at which the RX FIFO is considered full.
    pub rx_fifo_full_threshold: u16,
    /// Optional timeout value for RX operations.
    pub rx_timeout: Option<u8>,
    /// Optionally disable forced checks on incoming RX data.
    pub disable_rx_input_checks: bool,
}

impl Config {
    /// Sets the baud rate for the UART configuration.
    pub fn baudrate(mut self, baudrate: u32) -> Self {
        self.baudrate = baudrate;
        self
    }

    /// Configures the UART to use no parity check.
    pub fn parity_none(mut self) -> Self {
        self.parity = Parity::ParityNone;
        self
    }

    /// Configures the UART to use even parity check.
    pub fn parity_even(mut self) -> Self {
        self.parity = Parity::ParityEven;
        self
    }

    /// Configures the UART to use odd parity check.
    pub fn parity_odd(mut self) -> Self {
        self.parity = Parity::ParityOdd;
        self
    }

    /// Sets the number of data bits for the UART configuration.
    pub fn data_bits(mut self, data_bits: DataBits) -> Self {
        self.data_bits = data_bits;
        self
    }

    /// Sets the number of stop bits for the UART configuration.
    pub fn stop_bits(mut self, stop_bits: StopBits) -> Self {
        self.stop_bits = stop_bits;
        self
    }

    /// Sets the clock source for the UART configuration.
    pub fn clock_source(mut self, source: ClockSource) -> Self {
        self.clock_source = source;
        self
    }

    /// Calculates the total symbol length in bits based on the configured
    /// data bits, parity, and stop bits.
    pub fn symbol_length(&self) -> u8 {
        let mut length: u8 = 1; // start bit
        length += match self.data_bits {
            DataBits::DataBits5 => 5,
            DataBits::DataBits6 => 6,
            DataBits::DataBits7 => 7,
            DataBits::DataBits8 => 8,
        };
        length += match self.parity {
            Parity::ParityNone => 0,
            _ => 1,
        };
        length += match self.stop_bits {
            StopBits::Stop1 => 1,
            _ => 2, // esp-idf also counts 2 bits for settings 1.5 and 2 stop bits
        };
        length
    }

    /// Sets the RX FIFO full threshold for the UART configuration.
    pub fn rx_fifo_full_threshold(mut self, threshold: u16) -> Self {
        self.rx_fifo_full_threshold = threshold;
        self
    }

    /// Sets the RX timeout for the UART configuration.
    pub fn rx_timeout(mut self, timeout: Option<u8>) -> Self {
        self.rx_timeout = timeout;
        self
    }

    /// Disables forced checks on incoming RX inputs.
    pub fn disable_rx_input_checks(mut self, disable: bool) -> Self {
        self.disable_rx_input_checks = disable;
        self
    }
}

impl Default for Config {
    fn default() -> Config {
        Config {
            baudrate: 115_200,
            data_bits: Default::default(),
            parity: Default::default(),
            stop_bits: Default::default(),
            clock_source: Default::default(),
            rx_fifo_full_threshold: UART_FULL_THRESH_DEFAULT,
            rx_timeout: Some(UART_TOUT_THRESH_DEFAULT),
            disable_rx_input_checks: false,
        }
    }
}

/// Configuration for the AT-CMD detection functionality
pub struct AtCmdConfig {
    /// Optional idle time before the AT command detection begins, in clock
    /// cycles.
    pub pre_idle_count: Option<u16>,
    /// Optional idle time after the AT command detection ends, in clock
    /// cycles.
    pub post_idle_count: Option<u16>,
    /// Optional timeout between characters in the AT command, in clock
    /// cycles.
    pub gap_timeout: Option<u16>,
    /// The character that triggers the AT command detection.
    pub cmd_char: u8,
    /// Optional number of characters to detect as part of the AT command.
    pub char_num: Option<u8>,
}

impl AtCmdConfig {
    /// Creates a new `AtCmdConfig` with the specified configuration.
    ///
    /// This function sets up the AT command detection parameters, including
    /// pre- and post-idle times, a gap timeout, the triggering command
    /// character, and the number of characters to detect.
    pub fn new(
        pre_idle_count: Option<u16>,
        post_idle_count: Option<u16>,
        gap_timeout: Option<u16>,
        cmd_char: u8,
        char_num: Option<u8>,
    ) -> AtCmdConfig {
        Self {
            pre_idle_count,
            post_idle_count,
            gap_timeout,
            cmd_char,
            char_num,
        }
    }
}

struct UartBuilder<'d, M, T = AnyUart> {
    uart: PeripheralRef<'d, T>,
    phantom: PhantomData<M>,
}

impl<'d, M, T> UartBuilder<'d, M, T>
where
    T: Instance,
    M: Mode,
{
    fn new(uart: impl Peripheral<P = T> + 'd) -> Self {
        crate::into_ref!(uart);
        Self {
            uart,
            phantom: PhantomData,
        }
    }

    fn with_rx(self, rx: impl Peripheral<P = impl PeripheralInput> + 'd) -> Self {
        crate::into_mapped_ref!(rx);
        rx.init_input(Pull::Up, Internal);
        self.uart.info().rx_signal.connect_to(rx);

        self
    }

    fn with_tx(self, tx: impl Peripheral<P = impl PeripheralOutput> + 'd) -> Self {
        crate::into_mapped_ref!(tx);
        // Make sure we don't cause an unexpected low pulse on the pin.
        tx.set_output_high(true, Internal);
        tx.set_to_push_pull_output(Internal);
        self.uart.info().tx_signal.connect_to(tx);

        self
    }

    fn init(self, config: Config) -> Result<Uart<'d, M, T>, ConfigError> {
        let rx_guard = PeripheralGuard::new(self.uart.parts().0.peripheral);
        let tx_guard = PeripheralGuard::new(self.uart.parts().0.peripheral);

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
            },
        };
        serial.init(config)?;

        Ok(serial)
    }
}

/// UART (Full-duplex)
pub struct Uart<'d, M, T = AnyUart> {
    rx: UartRx<'d, M, T>,
    tx: UartTx<'d, M, T>,
}

/// UART (Transmit)
pub struct UartTx<'d, M, T = AnyUart> {
    uart: PeripheralRef<'d, T>,
    phantom: PhantomData<M>,
    guard: PeripheralGuard,
}

/// UART (Receive)
pub struct UartRx<'d, M, T = AnyUart> {
    uart: PeripheralRef<'d, T>,
    phantom: PhantomData<M>,
    guard: PeripheralGuard,
}

/// A configuration error.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ConfigError {
    /// The requested timeout is not supported.
    UnsupportedTimeout,
    /// The requested fifo threshold is not supported.
    UnsupportedFifoThreshold,
}

#[cfg(any(doc, feature = "unstable"))]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
impl<M, T> SetConfig for Uart<'_, M, T>
where
    T: Instance,
    M: Mode,
{
    type Config = Config;
    type ConfigError = ConfigError;

    fn set_config(&mut self, config: &Self::Config) -> Result<(), Self::ConfigError> {
        self.apply_config(config)
    }
}

#[cfg(any(doc, feature = "unstable"))]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
impl<M, T> SetConfig for UartRx<'_, M, T>
where
    T: Instance,
    M: Mode,
{
    type Config = Config;
    type ConfigError = ConfigError;

    fn set_config(&mut self, config: &Self::Config) -> Result<(), Self::ConfigError> {
        self.apply_config(config)
    }
}

#[cfg(any(doc, feature = "unstable"))]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
impl<M, T> SetConfig for UartTx<'_, M, T>
where
    T: Instance,
    M: Mode,
{
    type Config = Config;
    type ConfigError = ConfigError;

    fn set_config(&mut self, config: &Self::Config) -> Result<(), Self::ConfigError> {
        self.apply_config(config)
    }
}

impl<'d, M, T> UartTx<'d, M, T>
where
    T: Instance,
    M: Mode,
{
    /// Configure RTS pin
    pub fn with_rts(self, rts: impl Peripheral<P = impl PeripheralOutput> + 'd) -> Self {
        crate::into_mapped_ref!(rts);
        rts.set_to_push_pull_output(Internal);
        self.uart.info().rts_signal.connect_to(rts);

        self
    }

    /// Change the configuration.
    ///
    /// Note that this also changes the configuration of the RX half.
    pub fn apply_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        self.uart.info().apply_config(config)
    }

    /// Writes bytes
    pub fn write_bytes(&mut self, data: &[u8]) -> Result<usize, Error> {
        let count = data.len();

        data.iter()
            .try_for_each(|c| nb::block!(self.write_byte(*c)))?;

        Ok(count)
    }

    fn write_byte(&mut self, word: u8) -> nb::Result<(), Error> {
        if self.tx_fifo_count() < UART_FIFO_SIZE {
            self.register_block()
                .fifo()
                .write(|w| unsafe { w.rxfifo_rd_byte().bits(word) });

            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    #[allow(clippy::useless_conversion)]
    /// Returns the number of bytes currently in the TX FIFO for this UART
    /// instance.
    fn tx_fifo_count(&self) -> u16 {
        self.register_block()
            .status()
            .read()
            .txfifo_cnt()
            .bits()
            .into()
    }

    /// Flush the transmit buffer of the UART
    pub fn flush_tx(&mut self) -> nb::Result<(), Error> {
        if self.is_tx_idle() {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    /// Checks if the TX line is idle for this UART instance.
    ///
    /// Returns `true` if the transmit line is idle, meaning no data is
    /// currently being transmitted.
    fn is_tx_idle(&self) -> bool {
        #[cfg(esp32)]
        let status = self.register_block().status();
        #[cfg(not(esp32))]
        let status = self.register_block().fsm_status();

        status.read().st_utx_out().bits() == 0x0
    }

    /// Disables all TX-related interrupts for this UART instance.
    ///
    /// This function clears and disables the `transmit FIFO empty` interrupt,
    /// `transmit break done`, `transmit break idle done`, and `transmit done`
    /// interrupts.
    fn disable_tx_interrupts(&self) {
        self.register_block().int_clr().write(|w| {
            w.txfifo_empty().clear_bit_by_one();
            w.tx_brk_done().clear_bit_by_one();
            w.tx_brk_idle_done().clear_bit_by_one();
            w.tx_done().clear_bit_by_one()
        });

        self.register_block().int_ena().write(|w| {
            w.txfifo_empty().clear_bit();
            w.tx_brk_done().clear_bit();
            w.tx_brk_idle_done().clear_bit();
            w.tx_done().clear_bit()
        });
    }

    fn register_block(&self) -> &RegisterBlock {
        self.uart.info().register_block()
    }
}

impl<'d> UartTx<'d, Blocking> {
    /// Create a new UART TX instance in [`Blocking`] mode.
    pub fn new(
        uart: impl Peripheral<P = impl Instance> + 'd,
        config: Config,
        tx: impl Peripheral<P = impl PeripheralOutput> + 'd,
    ) -> Result<Self, ConfigError> {
        Self::new_typed(uart.map_into(), config, tx)
    }
}

impl<'d, T> UartTx<'d, Blocking, T>
where
    T: Instance,
{
    /// Create a new UART TX instance in [`Blocking`] mode.
    pub fn new_typed(
        uart: impl Peripheral<P = T> + 'd,
        config: Config,
        tx: impl Peripheral<P = impl PeripheralOutput> + 'd,
    ) -> Result<Self, ConfigError> {
        let (_, uart_tx) = UartBuilder::<'d, Blocking, T>::new(uart)
            .with_tx(tx)
            .init(config)?
            .split();

        Ok(uart_tx)
    }

    /// Reconfigures the driver to operate in [`Async`] mode.
    pub fn into_async(self) -> UartTx<'d, Async, T> {
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
        }
    }
}

impl<'d, T> UartTx<'d, Async, T>
where
    T: Instance,
{
    /// Reconfigures the driver to operate in [`Blocking`] mode.
    pub fn into_blocking(self) -> UartTx<'d, Blocking, T> {
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

impl<'d, M, T> UartRx<'d, M, T>
where
    T: Instance,
    M: Mode,
{
    /// Configure CTS pin
    pub fn with_cts(self, cts: impl Peripheral<P = impl PeripheralInput> + 'd) -> Self {
        crate::into_mapped_ref!(cts);
        cts.init_input(Pull::None, Internal);
        self.uart.info().cts_signal.connect_to(cts);

        self
    }

    /// Change the configuration.
    ///
    /// Note that this also changes the configuration of the TX half.
    pub fn apply_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        self.uart.info().apply_config(config)
    }

    /// Fill a buffer with received bytes
    pub fn read_bytes(&mut self, buf: &mut [u8]) -> Result<(), Error> {
        cfg_if::cfg_if! {
            if #[cfg(esp32s2)] {
                // On the ESP32-S2 we need to use PeriBus2 to read the FIFO:
                let fifo = unsafe {
                    &*((self.register_block().fifo().as_ptr() as *mut u8).add(0x20C00000)
                        as *mut crate::peripherals::uart0::FIFO)
                };
            } else {
                let fifo = self.register_block().fifo();
            }
        }

        for byte in buf.iter_mut() {
            while self.rx_fifo_count() == 0 {
                // Block until we received at least one byte
            }

            cfg_if::cfg_if! {
                if #[cfg(esp32)] {
                    // https://docs.espressif.com/projects/esp-chip-errata/en/latest/esp32/03-errata-description/esp32/cpu-subsequent-access-halted-when-get-interrupted.html
                    xtensa_lx::interrupt::free(|| {
                        *byte = fifo.read().rxfifo_rd_byte().bits();
                    });
                } else {
                    *byte = fifo.read().rxfifo_rd_byte().bits();
                }
            }
        }

        Ok(())
    }

    /// Read a byte from the UART
    pub fn read_byte(&mut self) -> nb::Result<u8, Error> {
        cfg_if::cfg_if! {
            if #[cfg(esp32s2)] {
                // On the ESP32-S2 we need to use PeriBus2 to read the FIFO:
                let fifo = unsafe {
                    &*((self.register_block().fifo().as_ptr() as *mut u8).add(0x20C00000)
                        as *mut crate::peripherals::uart0::FIFO)
                };
            } else {
                let fifo = self.register_block().fifo();
            }
        }

        if self.rx_fifo_count() > 0 {
            Ok(fifo.read().rxfifo_rd_byte().bits())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    /// Read all available bytes from the RX FIFO into the provided buffer and
    /// returns the number of read bytes. Never blocks
    pub fn drain_fifo(&mut self, buf: &mut [u8]) -> usize {
        let mut count = 0;
        while count < buf.len() {
            if let Ok(byte) = self.read_byte() {
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
        let fifo_cnt: u16 = self
            .register_block()
            .status()
            .read()
            .rxfifo_cnt()
            .bits()
            .into();

        // Calculate the real count based on the FIFO read and write offset address:
        // https://www.espressif.com/sites/default/files/documentation/esp32_errata_en.pdf
        // section 3.17
        #[cfg(esp32)]
        {
            let status = self.register_block().mem_rx_status().read();
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
        self.register_block().int_clr().write(|w| {
            w.rxfifo_full().clear_bit_by_one();
            w.rxfifo_ovf().clear_bit_by_one();
            w.rxfifo_tout().clear_bit_by_one();
            w.at_cmd_char_det().clear_bit_by_one()
        });

        self.register_block().int_ena().write(|w| {
            w.rxfifo_full().clear_bit();
            w.rxfifo_ovf().clear_bit();
            w.rxfifo_tout().clear_bit();
            w.at_cmd_char_det().clear_bit()
        });
    }

    fn register_block(&self) -> &RegisterBlock {
        self.uart.info().register_block()
    }
}

impl<'d> UartRx<'d, Blocking> {
    /// Create a new UART RX instance in [`Blocking`] mode.
    pub fn new(
        uart: impl Peripheral<P = impl Instance> + 'd,
        config: Config,
        rx: impl Peripheral<P = impl PeripheralInput> + 'd,
    ) -> Result<Self, ConfigError> {
        UartRx::new_typed(uart.map_into(), config, rx)
    }
}

impl<'d, T> UartRx<'d, Blocking, T>
where
    T: Instance,
{
    /// Create a new UART RX instance in [`Blocking`] mode.
    pub fn new_typed(
        uart: impl Peripheral<P = T> + 'd,
        config: Config,
        rx: impl Peripheral<P = impl PeripheralInput> + 'd,
    ) -> Result<Self, ConfigError> {
        let (uart_rx, _) = UartBuilder::new(uart).with_rx(rx).init(config)?.split();

        Ok(uart_rx)
    }

    /// Reconfigures the driver to operate in [`Async`] mode.
    pub fn into_async(self) -> UartRx<'d, Async, T> {
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

impl<'d, T> UartRx<'d, Async, T>
where
    T: Instance,
{
    /// Reconfigures the driver to operate in [`Blocking`] mode.
    pub fn into_blocking(self) -> UartRx<'d, Blocking, T> {
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
    pub fn new(
        uart: impl Peripheral<P = impl Instance> + 'd,
        config: Config,
        rx: impl Peripheral<P = impl PeripheralInput> + 'd,
        tx: impl Peripheral<P = impl PeripheralOutput> + 'd,
    ) -> Result<Self, ConfigError> {
        Self::new_typed(uart.map_into(), config, rx, tx)
    }
}

impl<'d, T> Uart<'d, Blocking, T>
where
    T: Instance,
{
    /// Create a new UART instance in [`Blocking`] mode.
    pub fn new_typed(
        uart: impl Peripheral<P = T> + 'd,
        config: Config,
        rx: impl Peripheral<P = impl PeripheralInput> + 'd,
        tx: impl Peripheral<P = impl PeripheralOutput> + 'd,
    ) -> Result<Self, ConfigError> {
        UartBuilder::new(uart).with_tx(tx).with_rx(rx).init(config)
    }

    /// Reconfigures the driver to operate in [`Async`] mode.
    pub fn into_async(self) -> Uart<'d, Async, T> {
        Uart {
            rx: self.rx.into_async(),
            tx: self.tx.into_async(),
        }
    }
}

impl<'d, T> Uart<'d, Async, T>
where
    T: Instance,
{
    /// Reconfigures the driver to operate in [`Blocking`] mode.
    pub fn into_blocking(self) -> Uart<'d, Blocking, T> {
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
pub enum UartInterrupt {
    /// Indicates that the received has detected the configured
    /// [`Uart::set_at_cmd`] character.
    AtCmd,

    /// The transmitter has finished sending out all data from the FIFO.
    TxDone,

    /// The receiver has received more data than what
    /// [`Config::rx_fifo_full_threshold`] specifies.
    RxFifoFull,
}

impl<'d, M, T> Uart<'d, M, T>
where
    T: Instance,
    M: Mode,
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

    fn register_block(&self) -> &RegisterBlock {
        // `self.tx.uart` and `self.rx.uart` are the same
        self.tx.uart.info().register_block()
    }

    /// Split the UART into a transmitter and receiver
    ///
    /// This is particularly useful when having two tasks correlating to
    /// transmitting and receiving.
    pub fn split(self) -> (UartRx<'d, M, T>, UartTx<'d, M, T>) {
        (self.rx, self.tx)
    }

    /// Write bytes out over the UART
    pub fn write_bytes(&mut self, data: &[u8]) -> Result<usize, Error> {
        self.tx.write_bytes(data)
    }

    /// Fill a buffer with received bytes
    pub fn read_bytes(&mut self, buf: &mut [u8]) -> Result<(), Error> {
        self.rx.read_bytes(buf)
    }

    /// Configures the AT-CMD detection settings
    pub fn set_at_cmd(&mut self, config: AtCmdConfig) {
        let register_block = self.register_block();

        #[cfg(not(any(esp32, esp32s2)))]
        register_block
            .clk_conf()
            .modify(|_, w| w.sclk_en().clear_bit());

        register_block.at_cmd_char().write(|w| unsafe {
            w.at_cmd_char().bits(config.cmd_char);
            w.char_num().bits(config.char_num.unwrap_or(1))
        });

        if let Some(pre_idle_count) = config.pre_idle_count {
            register_block
                .at_cmd_precnt()
                .write(|w| unsafe { w.pre_idle_num().bits(pre_idle_count as _) });
        }

        if let Some(post_idle_count) = config.post_idle_count {
            register_block
                .at_cmd_postcnt()
                .write(|w| unsafe { w.post_idle_num().bits(post_idle_count as _) });
        }

        if let Some(gap_timeout) = config.gap_timeout {
            register_block
                .at_cmd_gaptout()
                .write(|w| unsafe { w.rx_gap_tout().bits(gap_timeout as _) });
        }

        #[cfg(not(any(esp32, esp32s2)))]
        register_block
            .clk_conf()
            .modify(|_, w| w.sclk_en().set_bit());

        sync_regs(register_block);
    }

    /// Write a byte out over the UART
    pub fn write_byte(&mut self, word: u8) -> nb::Result<(), Error> {
        self.tx.write_byte(word)
    }

    /// Flush the transmit buffer of the UART
    pub fn flush_tx(&mut self) -> nb::Result<(), Error> {
        self.tx.flush_tx()
    }

    /// Read a byte from the UART
    pub fn read_byte(&mut self) -> nb::Result<u8, Error> {
        self.rx.read_byte()
    }

    /// Change the configuration.
    pub fn apply_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        self.rx.apply_config(config)?;
        Ok(())
    }

    #[inline(always)]
    fn init(&mut self, config: Config) -> Result<(), ConfigError> {
        cfg_if::cfg_if! {
            if #[cfg(any(esp32, esp32s2))] {
                // Nothing to do
            } else if #[cfg(any(esp32c2, esp32c3, esp32s3))] {
                unsafe { crate::peripherals::SYSTEM::steal() }
                    .perip_clk_en0()
                    .modify(|_, w| w.uart_mem_clk_en().set_bit());
            } else {
                self.register_block()
                    .conf0()
                    .modify(|_, w| w.mem_clk_en().set_bit());
            }
        };

        self.uart_peripheral_reset();

        self.rx.disable_rx_interrupts();
        self.tx.disable_tx_interrupts();

        self.rx.uart.info().apply_config(&config)?;

        // Setting err_wr_mask stops uart from storing data when data is wrong according
        // to reference manual
        if !config.disable_rx_input_checks {
            self.register_block()
                .conf0()
                .modify(|_, w| w.err_wr_mask().set_bit());
        }

        crate::rom::ets_delay_us(15);

        // Make sure we are starting in a "clean state" - previous operations might have
        // run into error conditions
        self.register_block()
            .int_clr()
            .write(|w| unsafe { w.bits(u32::MAX) });

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

        rst_core(self.register_block(), true);
        PeripheralClockControl::reset(self.tx.uart.info().peripheral);
        rst_core(self.register_block(), false);
    }
}

impl<T> crate::private::Sealed for Uart<'_, Blocking, T> where T: Instance {}

impl<T> InterruptConfigurable for Uart<'_, Blocking, T>
where
    T: Instance,
{
    fn set_interrupt_handler(&mut self, handler: crate::interrupt::InterruptHandler) {
        // `self.tx.uart` and `self.rx.uart` are the same
        self.tx.uart.info().set_interrupt_handler(handler);
    }
}

impl<T> Uart<'_, Blocking, T>
where
    T: Instance,
{
    /// Listen for the given interrupts
    pub fn listen(&mut self, interrupts: impl Into<EnumSet<UartInterrupt>>) {
        self.tx.uart.info().enable_listen(interrupts.into(), true)
    }

    /// Unlisten the given interrupts
    pub fn unlisten(&mut self, interrupts: impl Into<EnumSet<UartInterrupt>>) {
        self.tx.uart.info().enable_listen(interrupts.into(), false)
    }

    /// Gets asserted interrupts
    pub fn interrupts(&mut self) -> EnumSet<UartInterrupt> {
        self.tx.uart.info().interrupts()
    }

    /// Resets asserted interrupts
    pub fn clear_interrupts(&mut self, interrupts: EnumSet<UartInterrupt>) {
        self.tx.uart.info().clear_interrupts(interrupts)
    }
}

impl<T, M> ufmt_write::uWrite for Uart<'_, M, T>
where
    T: Instance,
    M: Mode,
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

impl<T, M> ufmt_write::uWrite for UartTx<'_, M, T>
where
    T: Instance,
    M: Mode,
{
    type Error = Error;

    #[inline]
    fn write_str(&mut self, s: &str) -> Result<(), Self::Error> {
        self.write_bytes(s.as_bytes())?;
        Ok(())
    }
}

impl<T, M> core::fmt::Write for Uart<'_, M, T>
where
    T: Instance,
    M: Mode,
{
    #[inline]
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.tx.write_str(s)
    }
}

impl<T, M> core::fmt::Write for UartTx<'_, M, T>
where
    T: Instance,
    M: Mode,
{
    #[inline]
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.write_bytes(s.as_bytes())
            .map_err(|_| core::fmt::Error)?;
        Ok(())
    }
}

impl<T, M> embedded_hal_nb::serial::ErrorType for Uart<'_, M, T> {
    type Error = Error;
}

impl<T, M> embedded_hal_nb::serial::ErrorType for UartTx<'_, M, T> {
    type Error = Error;
}

impl<T, M> embedded_hal_nb::serial::ErrorType for UartRx<'_, M, T> {
    type Error = Error;
}

impl<T, M> embedded_hal_nb::serial::Read for Uart<'_, M, T>
where
    T: Instance,
    M: Mode,
{
    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        self.read_byte()
    }
}

impl<T, M> embedded_hal_nb::serial::Read for UartRx<'_, M, T>
where
    T: Instance,
    M: Mode,
{
    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        self.read_byte()
    }
}

impl<T, M> embedded_hal_nb::serial::Write for Uart<'_, M, T>
where
    T: Instance,
    M: Mode,
{
    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        self.write_byte(word)
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        self.flush_tx()
    }
}

impl<T, M> embedded_hal_nb::serial::Write for UartTx<'_, M, T>
where
    T: Instance,
    M: Mode,
{
    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        self.write_byte(word)
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        self.flush_tx()
    }
}

#[cfg(any(doc, feature = "unstable"))]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
impl<T, M> embedded_io::ErrorType for Uart<'_, M, T> {
    type Error = Error;
}

#[cfg(any(doc, feature = "unstable"))]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
impl<T, M> embedded_io::ErrorType for UartTx<'_, M, T> {
    type Error = Error;
}

#[cfg(any(doc, feature = "unstable"))]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
impl<T, M> embedded_io::ErrorType for UartRx<'_, M, T> {
    type Error = Error;
}

#[cfg(any(doc, feature = "unstable"))]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
impl<T, M> embedded_io::Read for Uart<'_, M, T>
where
    T: Instance,
    M: Mode,
{
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.rx.read(buf)
    }
}

#[cfg(any(doc, feature = "unstable"))]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
impl<T, M> embedded_io::Read for UartRx<'_, M, T>
where
    T: Instance,
    M: Mode,
{
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        if buf.is_empty() {
            return Ok(0);
        }

        while self.rx_fifo_count() == 0 {
            // Block until we received at least one byte
        }

        Ok(self.drain_fifo(buf))
    }
}

#[cfg(any(doc, feature = "unstable"))]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
impl<T, M> embedded_io::ReadReady for Uart<'_, M, T>
where
    T: Instance,
    M: Mode,
{
    fn read_ready(&mut self) -> Result<bool, Self::Error> {
        self.rx.read_ready()
    }
}

#[cfg(any(doc, feature = "unstable"))]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
impl<T, M> embedded_io::ReadReady for UartRx<'_, M, T>
where
    T: Instance,
    M: Mode,
{
    fn read_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(self.rx_fifo_count() > 0)
    }
}

#[cfg(any(doc, feature = "unstable"))]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
impl<T, M> embedded_io::Write for Uart<'_, M, T>
where
    T: Instance,
    M: Mode,
{
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.tx.write(buf)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        self.tx.flush()
    }
}

#[cfg(any(doc, feature = "unstable"))]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
impl<T, M> embedded_io::Write for UartTx<'_, M, T>
where
    T: Instance,
    M: Mode,
{
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.write_bytes(buf)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        loop {
            match self.flush_tx() {
                Ok(_) => break,
                Err(nb::Error::WouldBlock) => { /* Wait */ }
                Err(nb::Error::Other(e)) => return Err(e),
            }
        }

        Ok(())
    }
}

#[derive(Debug, EnumSetType)]
pub(crate) enum TxEvent {
    TxDone,
    TxFiFoEmpty,
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

    fn triggered_events(&self) -> EnumSet<RxEvent> {
        let interrupts_enabled = self.uart.register_block().int_ena().read();
        let mut events_triggered = EnumSet::new();
        for event in self.events {
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

    fn enable_listen(&self, enable: bool) {
        self.uart.register_block().int_ena().modify(|_, w| {
            for event in self.events {
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
}

impl core::future::Future for UartRxFuture {
    type Output = EnumSet<RxEvent>;

    fn poll(
        mut self: core::pin::Pin<&mut Self>,
        cx: &mut core::task::Context<'_>,
    ) -> core::task::Poll<Self::Output> {
        if !self.registered {
            self.state.rx_waker.register(cx.waker());
            self.enable_listen(true);
            self.registered = true;
        }
        let events = self.triggered_events();
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
        self.enable_listen(false);
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
        let interrupts_enabled = self.uart.register_block().int_ena().read();
        let mut event_triggered = false;
        for event in self.events {
            event_triggered |= match event {
                TxEvent::TxDone => interrupts_enabled.tx_done().bit_is_clear(),
                TxEvent::TxFiFoEmpty => interrupts_enabled.txfifo_empty().bit_is_clear(),
            }
        }
        event_triggered
    }

    fn enable_listen(&self, enable: bool) {
        self.uart.register_block().int_ena().modify(|_, w| {
            for event in self.events {
                match event {
                    TxEvent::TxDone => w.tx_done().bit(enable),
                    TxEvent::TxFiFoEmpty => w.txfifo_empty().bit(enable),
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

impl<T> Uart<'_, Async, T>
where
    T: Instance,
{
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

impl<T> UartTx<'_, Async, T>
where
    T: Instance,
{
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
                self.write_byte(*byte).unwrap(); // should never fail
                count += 1;
            }

            if next_offset >= words.len() {
                break;
            }

            offset = next_offset;
            UartTxFuture::new(self.uart.reborrow(), TxEvent::TxFiFoEmpty).await;
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
            UartTxFuture::new(self.uart.reborrow(), TxEvent::TxDone).await;
        }

        Ok(())
    }
}

impl<T> UartRx<'_, Async, T>
where
    T: Instance,
{
    /// Read async to buffer slice `buf`.
    /// Waits until at least one byte is in the Rx FiFo
    /// and one of the following interrupts occurs:
    /// - `RXFIFO_FULL`
    /// - `RXFIFO_OVF`
    /// - `AT_CMD_CHAR_DET` (only if `set_at_cmd` was called)
    /// - `RXFIFO_TOUT` (only if `set_rx_timeout was called)
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
    /// This method will never return Ok(0)
    pub async fn read_async(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
        if buf.is_empty() {
            return Err(Error::InvalidArgument);
        }

        loop {
            let mut events = RxEvent::FifoFull
                | RxEvent::FifoOvf
                | RxEvent::FrameError
                | RxEvent::GlitchDetected
                | RxEvent::ParityError;

            let register_block = self.uart.info().register_block();
            if register_block.at_cmd_char().read().char_num().bits() > 0 {
                events |= RxEvent::CmdCharDetected;
            }

            cfg_if::cfg_if! {
                if #[cfg(any(esp32c6, esp32h2))] {
                    let reg_en = register_block.tout_conf();
                } else {
                    let reg_en = register_block.conf1();
                }
            };
            if reg_en.read().rx_tout_en().bit_is_set() {
                events |= RxEvent::FifoTout;
            }

            let events_happened = UartRxFuture::new(self.uart.reborrow(), events).await;
            // always drain the fifo, if an error has occurred the data is lost
            let read_bytes = self.drain_fifo(buf);
            // check error events
            for event_happened in events_happened {
                match event_happened {
                    RxEvent::FifoOvf => return Err(Error::RxFifoOvf),
                    RxEvent::GlitchDetected => return Err(Error::RxGlitchDetected),
                    RxEvent::FrameError => return Err(Error::RxFrameError),
                    RxEvent::ParityError => return Err(Error::RxParityError),
                    RxEvent::FifoFull | RxEvent::CmdCharDetected | RxEvent::FifoTout => continue,
                }
            }
            // Unfortunately, the uart's rx-timeout counter counts up whenever there is
            // data in the fifo, even if the interrupt is disabled and the status bit
            // cleared. Since we do not drain the fifo in the interrupt handler, we need to
            // reset the counter here, after draining the fifo.
            self.register_block()
                .int_clr()
                .write(|w| w.rxfifo_tout().clear_bit_by_one());

            if read_bytes > 0 {
                return Ok(read_bytes);
            }
        }
    }
}

#[cfg(any(doc, feature = "unstable"))]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
impl<T> embedded_io_async::Read for Uart<'_, Async, T>
where
    T: Instance,
{
    /// In contrast to the documentation of embedded_io_async::Read, this
    /// method blocks until an uart interrupt occurs.
    /// See UartRx::read_async for more details.
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.read_async(buf).await
    }
}

#[cfg(any(doc, feature = "unstable"))]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
impl<T> embedded_io_async::Read for UartRx<'_, Async, T>
where
    T: Instance,
{
    /// In contrast to the documentation of embedded_io_async::Read, this
    /// method blocks until an uart interrupt occurs.
    /// See UartRx::read_async for more details.
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.read_async(buf).await
    }
}

#[cfg(any(doc, feature = "unstable"))]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
impl<T> embedded_io_async::Write for Uart<'_, Async, T>
where
    T: Instance,
{
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.write_async(buf).await
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        self.flush_async().await
    }
}

#[cfg(any(doc, feature = "unstable"))]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
impl<T> embedded_io_async::Write for UartTx<'_, Async, T>
where
    T: Instance,
{
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
    let interrupts = uart.register_block().int_st().read();
    let interrupt_bits = interrupts.bits(); // = int_raw & int_ena
    let rx_wake = interrupts.rxfifo_full().bit_is_set()
        || interrupts.rxfifo_ovf().bit_is_set()
        || interrupts.rxfifo_tout().bit_is_set()
        || interrupts.at_cmd_char_det().bit_is_set()
        || interrupts.glitch_det().bit_is_set()
        || interrupts.frm_err().bit_is_set()
        || interrupts.parity_err().bit_is_set();
    let tx_wake = interrupts.tx_done().bit_is_set() || interrupts.txfifo_empty().bit_is_set();
    uart.register_block()
        .int_clr()
        .write(|w| unsafe { w.bits(interrupt_bits) });
    uart.register_block()
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
pub mod lp_uart {
    use crate::{
        gpio::lp_io::{LowPowerInput, LowPowerOutput},
        peripherals::{LP_CLKRST, LP_UART},
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
        // TODO: CTS and RTS pins
        pub fn new(
            uart: LP_UART,
            config: Config,
            _tx: LowPowerOutput<'_, 5>,
            _rx: LowPowerInput<'_, 4>,
        ) -> Self {
            let lp_io = unsafe { crate::peripherals::LP_IO::steal() };
            let lp_aon = unsafe { crate::peripherals::LP_AON::steal() };

            // FIXME: use GPIO APIs to configure pins
            lp_aon
                .gpio_mux()
                .modify(|r, w| unsafe { w.sel().bits(r.sel().bits() | 1 << 4) });
            lp_aon
                .gpio_mux()
                .modify(|r, w| unsafe { w.sel().bits(r.sel().bits() | 1 << 5) });

            lp_io.gpio(4).modify(|_, w| unsafe { w.mcu_sel().bits(1) });
            lp_io.gpio(5).modify(|_, w| unsafe { w.mcu_sel().bits(1) });

            let mut me = Self { uart };

            // Set UART mode - do nothing for LP

            // Disable UART parity
            // 8-bit world
            // 1-bit stop bit
            me.uart.conf0().modify(|_, w| unsafe {
                w.parity().clear_bit();
                w.parity_en().clear_bit();
                w.bit_num().bits(0x3);
                w.stop_bit_num().bits(0x1)
            });
            // Set tx idle
            me.uart
                .idle_conf()
                .modify(|_, w| unsafe { w.tx_idle_num().bits(0) });
            // Disable hw-flow control
            me.uart
                .hwfc_conf()
                .modify(|_, w| w.rx_flow_en().clear_bit());

            // Get source clock frequency
            // default == SOC_MOD_CLK_RTC_FAST == 2

            // LP_CLKRST.lpperi.lp_uart_clk_sel = 0;
            unsafe { LP_CLKRST::steal() }
                .lpperi()
                .modify(|_, w| w.lp_uart_clk_sel().clear_bit());

            // Override protocol parameters from the configuration
            // uart_hal_set_baudrate(&hal, cfg->uart_proto_cfg.baud_rate, sclk_freq);
            me.change_baud_internal(config.baudrate, config.clock_source);
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
            self.uart.conf0().modify(|_, w| w.rxfifo_rst().set_bit());
            self.update();

            self.uart.conf0().modify(|_, w| w.rxfifo_rst().clear_bit());
            self.update();
        }

        fn txfifo_reset(&mut self) {
            self.uart.conf0().modify(|_, w| w.txfifo_rst().set_bit());
            self.update();

            self.uart.conf0().modify(|_, w| w.txfifo_rst().clear_bit());
            self.update();
        }

        fn update(&mut self) {
            self.uart
                .reg_update()
                .modify(|_, w| w.reg_update().set_bit());
            while self.uart.reg_update().read().reg_update().bit_is_set() {
                // wait
            }
        }

        fn change_baud_internal(&mut self, baudrate: u32, clock_source: super::ClockSource) {
            // TODO: Currently it's not possible to use XtalD2Clk
            let clk = 16_000_000_u32;
            let max_div = 0b1111_1111_1111 - 1;
            let clk_div = clk.div_ceil(max_div * baudrate);

            self.uart.clk_conf().modify(|_, w| unsafe {
                w.sclk_div_a().bits(0);
                w.sclk_div_b().bits(0);
                w.sclk_div_num().bits(clk_div as u8 - 1);
                w.sclk_sel().bits(match clock_source {
                    super::ClockSource::Xtal => 3,
                    super::ClockSource::RcFast => 2,
                    super::ClockSource::Apb => panic!("Wrong clock source for LP_UART"),
                });
                w.sclk_en().set_bit()
            });

            let clk = clk / clk_div;
            let divider = clk / baudrate;
            let divider = divider as u16;

            self.uart
                .clkdiv()
                .write(|w| unsafe { w.clkdiv().bits(divider).frag().bits(0) });

            self.update();
        }

        /// Modify UART baud rate and reset TX/RX fifo.
        pub fn change_baud(&mut self, baudrate: u32, clock_source: super::ClockSource) {
            self.change_baud_internal(baudrate, clock_source);
            self.txfifo_reset();
            self.rxfifo_reset();
        }

        fn change_parity(&mut self, parity: Parity) -> &mut Self {
            if parity != Parity::ParityNone {
                self.uart
                    .conf0()
                    .modify(|_, w| w.parity().bit((parity as u8 & 0x1) != 0));
            }

            self.uart.conf0().modify(|_, w| match parity {
                Parity::ParityNone => w.parity_en().clear_bit(),
                Parity::ParityEven => w.parity_en().set_bit().parity().clear_bit(),
                Parity::ParityOdd => w.parity_en().set_bit().parity().set_bit(),
            });

            self
        }

        fn change_data_bits(&mut self, data_bits: DataBits) -> &mut Self {
            self.uart
                .conf0()
                .modify(|_, w| unsafe { w.bit_num().bits(data_bits as u8) });

            self.update();

            self
        }

        fn change_stop_bits(&mut self, stop_bits: StopBits) -> &mut Self {
            self.uart
                .conf0()
                .modify(|_, w| unsafe { w.stop_bit_num().bits(stop_bits as u8) });

            self.update();
            self
        }

        fn change_tx_idle(&mut self, idle_num: u16) -> &mut Self {
            self.uart
                .idle_conf()
                .modify(|_, w| unsafe { w.tx_idle_num().bits(idle_num) });

            self.update();
            self
        }
    }
}

/// UART Peripheral Instance
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
    pub fn register_block(&self) -> &RegisterBlock {
        unsafe { &*self.register_block }
    }

    /// Listen for the given interrupts
    fn enable_listen(&self, interrupts: EnumSet<UartInterrupt>, enable: bool) {
        let reg_block = self.register_block();

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
        let reg_block = self.register_block();

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
        let reg_block = self.register_block();

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
        self.set_rx_fifo_full_threshold(config.rx_fifo_full_threshold)?;
        self.set_rx_timeout(config.rx_timeout, config.symbol_length())?;
        self.change_baud(config.baudrate, config.clock_source);
        self.change_data_bits(config.data_bits);
        self.change_parity(config.parity);
        self.change_stop_bits(config.stop_bits);
        self.change_disable_rx_input_checks(config.disable_rx_input_checks);

        // Reset Tx/Rx FIFOs
        self.rxfifo_reset();
        self.txfifo_reset();

        Ok(())
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

        self.register_block()
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

        let register_block = self.register_block();

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

    #[cfg(any(esp32c2, esp32c3, esp32s3))]
    fn change_baud(&self, baudrate: u32, clock_source: ClockSource) {
        let clocks = Clocks::get();
        let clk = match clock_source {
            ClockSource::Apb => clocks.apb_clock.to_Hz(),
            ClockSource::Xtal => clocks.xtal_clock.to_Hz(),
            ClockSource::RcFast => crate::soc::constants::RC_FAST_CLK.to_Hz(),
        };

        if clock_source == ClockSource::RcFast {
            unsafe { crate::peripherals::RTC_CNTL::steal() }
                .clk_conf()
                .modify(|_, w| w.dig_clk8m_en().variant(true));
            // esp_rom_delay_us(SOC_DELAY_RC_FAST_DIGI_SWITCH);
            crate::rom::ets_delay_us(5);
        }

        let max_div = 0b1111_1111_1111 - 1;
        let clk_div = clk.div_ceil(max_div * baudrate);
        self.register_block().clk_conf().write(|w| unsafe {
            w.sclk_sel().bits(match clock_source {
                ClockSource::Apb => 1,
                ClockSource::RcFast => 2,
                ClockSource::Xtal => 3,
            });
            w.sclk_div_a().bits(0);
            w.sclk_div_b().bits(0);
            w.sclk_div_num().bits(clk_div as u8 - 1);
            w.rx_sclk_en().bit(true);
            w.tx_sclk_en().bit(true)
        });

        let divider = (clk << 4) / (baudrate * clk_div);
        let divider_integer = (divider >> 4) as u16;
        let divider_frag = (divider & 0xf) as u8;
        self.register_block()
            .clkdiv()
            .write(|w| unsafe { w.clkdiv().bits(divider_integer).frag().bits(divider_frag) });
    }

    fn is_instance(&self, other: impl Instance) -> bool {
        self == other.info()
    }

    fn sync_regs(&self) {
        sync_regs(self.register_block());
    }

    #[cfg(any(esp32c6, esp32h2))]
    fn change_baud(&self, baudrate: u32, clock_source: ClockSource) {
        let clocks = Clocks::get();
        let clk = match clock_source {
            ClockSource::Apb => clocks.apb_clock.to_Hz(),
            ClockSource::Xtal => clocks.xtal_clock.to_Hz(),
            ClockSource::RcFast => crate::soc::constants::RC_FAST_CLK.to_Hz(),
        };

        let max_div = 0b1111_1111_1111 - 1;
        let clk_div = clk.div_ceil(max_div * baudrate);

        // UART clocks are configured via PCR
        let pcr = unsafe { crate::peripherals::PCR::steal() };

        if self.is_instance(unsafe { crate::peripherals::UART0::steal() }) {
            pcr.uart0_conf()
                .modify(|_, w| w.uart0_rst_en().clear_bit().uart0_clk_en().set_bit());

            pcr.uart0_sclk_conf().modify(|_, w| unsafe {
                w.uart0_sclk_div_a().bits(0);
                w.uart0_sclk_div_b().bits(0);
                w.uart0_sclk_div_num().bits(clk_div as u8 - 1);
                w.uart0_sclk_sel().bits(match clock_source {
                    ClockSource::Apb => 1,
                    ClockSource::RcFast => 2,
                    ClockSource::Xtal => 3,
                });
                w.uart0_sclk_en().set_bit()
            });
        } else {
            pcr.uart1_conf()
                .modify(|_, w| w.uart1_rst_en().clear_bit().uart1_clk_en().set_bit());

            pcr.uart1_sclk_conf().modify(|_, w| unsafe {
                w.uart1_sclk_div_a().bits(0);
                w.uart1_sclk_div_b().bits(0);
                w.uart1_sclk_div_num().bits(clk_div as u8 - 1);
                w.uart1_sclk_sel().bits(match clock_source {
                    ClockSource::Apb => 1,
                    ClockSource::RcFast => 2,
                    ClockSource::Xtal => 3,
                });
                w.uart1_sclk_en().set_bit()
            });
        }

        let clk = clk / clk_div;
        let divider = clk / baudrate;
        let divider = divider as u16;

        self.register_block()
            .clkdiv()
            .write(|w| unsafe { w.clkdiv().bits(divider).frag().bits(0) });

        self.sync_regs();
    }

    #[cfg(any(esp32, esp32s2))]
    fn change_baud(&self, baudrate: u32, clock_source: ClockSource) {
        let clk = match clock_source {
            ClockSource::Apb => Clocks::get().apb_clock.to_Hz(),
            // ESP32(/-S2) TRM, section 3.2.4.2 (6.2.4.2 for S2)
            ClockSource::RefTick => crate::soc::constants::REF_TICK.to_Hz(),
        };

        self.register_block()
            .conf0()
            .modify(|_, w| w.tick_ref_always_on().bit(clock_source == ClockSource::Apb));

        let divider = clk / baudrate;

        self.register_block()
            .clkdiv()
            .write(|w| unsafe { w.clkdiv().bits(divider).frag().bits(0) });
    }

    fn change_data_bits(&self, data_bits: DataBits) {
        self.register_block()
            .conf0()
            .modify(|_, w| unsafe { w.bit_num().bits(data_bits as u8) });
    }

    fn change_parity(&self, parity: Parity) {
        self.register_block().conf0().modify(|_, w| match parity {
            Parity::ParityNone => w.parity_en().clear_bit(),
            Parity::ParityEven => w.parity_en().set_bit().parity().clear_bit(),
            Parity::ParityOdd => w.parity_en().set_bit().parity().set_bit(),
        });
    }

    fn change_stop_bits(&self, stop_bits: StopBits) {
        #[cfg(esp32)]
        {
            // workaround for hardware issue, when UART stop bit set as 2-bit mode.
            if stop_bits == StopBits::Stop2 {
                self.register_block()
                    .rs485_conf()
                    .modify(|_, w| w.dl1_en().bit(stop_bits == StopBits::Stop2));

                self.register_block().conf0().modify(|_, w| {
                    if stop_bits == StopBits::Stop2 {
                        unsafe { w.stop_bit_num().bits(1) }
                    } else {
                        unsafe { w.stop_bit_num().bits(stop_bits as u8) }
                    }
                });
            }
        }

        #[cfg(not(esp32))]
        self.register_block()
            .conf0()
            .modify(|_, w| unsafe { w.stop_bit_num().bits(stop_bits as u8) });
    }

    fn change_disable_rx_input_checks(&self, disable: bool) {
        self.register_block()
            .conf0()
            .modify(|_, w| w.rx_filter_en().bit(!disable));
    }

    fn rxfifo_reset(&self) {
        fn rxfifo_rst(reg_block: &RegisterBlock, enable: bool) {
            reg_block.conf0().modify(|_, w| w.rxfifo_rst().bit(enable));
            sync_regs(reg_block);
        }

        rxfifo_rst(self.register_block(), true);
        rxfifo_rst(self.register_block(), false);
    }

    fn txfifo_reset(&self) {
        fn txfifo_rst(reg_block: &RegisterBlock, enable: bool) {
            reg_block.conf0().modify(|_, w| w.txfifo_rst().bit(enable));
            sync_regs(reg_block);
        }

        txfifo_rst(self.register_block(), true);
        txfifo_rst(self.register_block(), false);
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
                #[crate::macros::handler]
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
