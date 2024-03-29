//! Universal Asynchronous Receiver/Transmitter (UART)
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
//! configured. Additionally, the transmit (TX) and receive (RX) pins can be
//! specified.
//!
//! ```no_run
//! let config = Config {
//!     baudrate: 115_200,
//!     data_bits: DataBits::DataBits8,
//!     parity: Parity::ParityNone,
//!     stop_bits: StopBits::STOP1,
//! };
//!
//! let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
//! let pins = TxRxPins::new_tx_rx(
//!     io.pins.gpio1.into_push_pull_output(),
//!     io.pins.gpio2.into_floating_input(),
//! );
//!
//! let mut uart1 = Uart::new_with_config(peripherals.UART1, config, Some(pins), &clocks);
//! ```
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
//! ### Examples
//!
//! #### Sending and Receiving Data
//!
//! ```no_run
//! // Write bytes out over the UART:
//! uart1.write_bytes("Hello, world!".as_bytes())?;
//! ```
//!
//! #### Splitting the UART into TX and RX Components
//!
//! ```no_run
//! // The UART can be split into separate Transmit and Receive components:
//! let (mut tx, rx) = uart1.split();
//!
//! // Each component can be used individually to interact with the UART:
//! tx.write_bytes(&[42u8])?;
//! let byte = rx.read_byte()?;
//! ```
//!
//! [embedded-hal]: https://docs.rs/embedded-hal/latest/embedded_hal/
//! [embedded-io]: https://docs.rs/embedded-io/latest/embedded_io/
//! [embedded-hal-async]: https://docs.rs/embedded-hal-async/latest/embedded_hal_async/
//! [embedded-io-async]: https://docs.rs/embedded-io-async/latest/embedded_io_async/

use core::marker::PhantomData;

use self::config::Config;
use crate::{
    clock::Clocks,
    gpio::{InputPin, InputSignal, NoPinType, OutputPin, OutputSignal},
    interrupt::InterruptHandler,
    peripheral::{Peripheral, PeripheralRef},
    peripherals::{
        uart0::{fifo::FIFO_SPEC, RegisterBlock},
        Interrupt,
    },
    system::PeripheralClockControl,
    Blocking,
    Mode,
};

const CONSOLE_UART_NUM: usize = 0;
const UART_FIFO_SIZE: u16 = 128;

/// UART Error
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// An invalid configuration argument was provided
    InvalidArgument,
    /// The RX FIFO overflowed
    #[cfg(feature = "async")]
    RxFifoOvf,
}

#[cfg(feature = "embedded-hal")]
impl embedded_hal_nb::serial::Error for Error {
    fn kind(&self) -> embedded_hal_nb::serial::ErrorKind {
        embedded_hal_nb::serial::ErrorKind::Other
    }
}

#[cfg(feature = "embedded-io")]
impl embedded_io::Error for Error {
    fn kind(&self) -> embedded_io::ErrorKind {
        embedded_io::ErrorKind::Other
    }
}

/// UART Configuration
pub mod config {
    /// Number of data bits
    #[derive(PartialEq, Eq, Copy, Clone, Debug)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum DataBits {
        DataBits5 = 0,
        DataBits6 = 1,
        DataBits7 = 2,
        DataBits8 = 3,
    }

    /// Parity check
    #[derive(PartialEq, Eq, Copy, Clone, Debug)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum Parity {
        ParityNone,
        ParityEven,
        ParityOdd,
    }

    /// Number of stop bits
    #[derive(PartialEq, Eq, Copy, Clone, Debug)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum StopBits {
        /// 1 stop bit
        STOP1   = 1,
        /// 1.5 stop bits
        STOP1P5 = 2,
        /// 2 stop bits
        STOP2   = 3,
    }

    /// UART Configuration
    #[derive(Debug, Copy, Clone)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub struct Config {
        pub baudrate: u32,
        pub data_bits: DataBits,
        pub parity: Parity,
        pub stop_bits: StopBits,
    }

    impl Config {
        pub fn baudrate(mut self, baudrate: u32) -> Self {
            self.baudrate = baudrate;
            self
        }

        pub fn parity_none(mut self) -> Self {
            self.parity = Parity::ParityNone;
            self
        }

        pub fn parity_even(mut self) -> Self {
            self.parity = Parity::ParityEven;
            self
        }

        pub fn parity_odd(mut self) -> Self {
            self.parity = Parity::ParityOdd;
            self
        }

        pub fn data_bits(mut self, data_bits: DataBits) -> Self {
            self.data_bits = data_bits;
            self
        }

        pub fn stop_bits(mut self, stop_bits: StopBits) -> Self {
            self.stop_bits = stop_bits;
            self
        }

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
                StopBits::STOP1 => 1,
                _ => 2, // esp-idf also counts 2 bits for settings 1.5 and 2 stop bits
            };
            length
        }
    }

    impl Default for Config {
        fn default() -> Config {
            Config {
                baudrate: 115_200,
                data_bits: DataBits::DataBits8,
                parity: Parity::ParityNone,
                stop_bits: StopBits::STOP1,
            }
        }
    }

    /// Configuration for the AT-CMD detection functionality
    pub struct AtCmdConfig {
        pub pre_idle_count: Option<u16>,
        pub post_idle_count: Option<u16>,
        pub gap_timeout: Option<u16>,
        pub cmd_char: u8,
        pub char_num: Option<u8>,
    }

    impl AtCmdConfig {
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
}

/// Pins used by the UART interface
pub trait UartPins {
    fn configure_pins(
        &mut self,
        tx_signal: OutputSignal,
        rx_signal: InputSignal,
        cts_signal: InputSignal,
        rts_signal: OutputSignal,
    );
}

/// All UART pins (TX, RX, CTS, RTS)
pub struct AllPins<'d, TX: OutputPin, RX: InputPin, CTS: InputPin, RTS: OutputPin> {
    pub(crate) tx: Option<PeripheralRef<'d, TX>>,
    pub(crate) rx: Option<PeripheralRef<'d, RX>>,
    pub(crate) cts: Option<PeripheralRef<'d, CTS>>,
    pub(crate) rts: Option<PeripheralRef<'d, RTS>>,
}

impl<'d, TX: OutputPin, RX: InputPin, CTS: InputPin, RTS: OutputPin> AllPins<'d, TX, RX, CTS, RTS> {
    pub fn new(
        tx: impl Peripheral<P = TX> + 'd,
        rx: impl Peripheral<P = RX> + 'd,
        cts: impl Peripheral<P = CTS> + 'd,
        rts: impl Peripheral<P = RTS> + 'd,
    ) -> AllPins<'d, TX, RX, CTS, RTS> {
        crate::into_ref!(tx, rx, cts, rts);
        AllPins {
            tx: Some(tx),
            rx: Some(rx),
            cts: Some(cts),
            rts: Some(rts),
        }
    }
}

impl<TX: OutputPin, RX: InputPin, CTS: InputPin, RTS: OutputPin> UartPins
    for AllPins<'_, TX, RX, CTS, RTS>
{
    fn configure_pins(
        &mut self,
        tx_signal: OutputSignal,
        rx_signal: InputSignal,
        cts_signal: InputSignal,
        rts_signal: OutputSignal,
    ) {
        if let Some(ref mut tx) = self.tx {
            tx.set_to_push_pull_output()
                .connect_peripheral_to_output(tx_signal);
        }

        if let Some(ref mut rx) = self.rx {
            rx.set_to_input().connect_input_to_peripheral(rx_signal);
        }

        if let Some(ref mut cts) = self.cts {
            cts.set_to_input().connect_input_to_peripheral(cts_signal);
        }

        if let Some(ref mut rts) = self.rts {
            rts.set_to_push_pull_output()
                .connect_peripheral_to_output(rts_signal);
        }
    }
}

/// TX and RX pins
pub struct TxRxPins<'d, TX: OutputPin, RX: InputPin> {
    pub tx: Option<PeripheralRef<'d, TX>>,
    pub rx: Option<PeripheralRef<'d, RX>>,
}

impl<'d, TX: OutputPin, RX: InputPin> TxRxPins<'d, TX, RX> {
    pub fn new_tx_rx(
        tx: impl Peripheral<P = TX> + 'd,
        rx: impl Peripheral<P = RX> + 'd,
    ) -> TxRxPins<'d, TX, RX> {
        crate::into_ref!(tx, rx);
        TxRxPins {
            tx: Some(tx),
            rx: Some(rx),
        }
    }
}

impl<TX: OutputPin, RX: InputPin> UartPins for TxRxPins<'_, TX, RX> {
    fn configure_pins(
        &mut self,
        tx_signal: OutputSignal,
        rx_signal: InputSignal,
        _cts_signal: InputSignal,
        _rts_signal: OutputSignal,
    ) {
        if let Some(ref mut tx) = self.tx {
            tx.set_to_push_pull_output()
                .connect_peripheral_to_output(tx_signal);
        }

        if let Some(ref mut rx) = self.rx {
            rx.set_to_input().connect_input_to_peripheral(rx_signal);
        }
    }
}

/// UART (Full-duplex)
pub struct Uart<'d, T, M> {
    #[cfg(not(esp32))]
    symbol_len: u8,
    tx: UartTx<'d, T, M>,
    rx: UartRx<'d, T, M>,
}

/// UART (Transmit)
pub struct UartTx<'d, T, M> {
    phantom: PhantomData<(&'d mut T, M)>,
}

/// UART (Receive)
pub struct UartRx<'d, T, M> {
    phantom: PhantomData<(&'d mut T, M)>,
    at_cmd_config: Option<config::AtCmdConfig>,
    rx_timeout_config: Option<u8>,
}

impl<'d, T, M> UartTx<'d, T, M>
where
    T: Instance,
    M: Mode,
{
    fn new_inner() -> Self {
        Self {
            phantom: PhantomData,
        }
    }

    /// Writes bytes
    pub fn write_bytes(&mut self, data: &[u8]) -> Result<usize, Error> {
        let count = data.len();

        data.iter()
            .try_for_each(|c| nb::block!(self.write_byte(*c)))?;

        Ok(count)
    }

    fn write_byte(&mut self, word: u8) -> nb::Result<(), Error> {
        if T::get_tx_fifo_count() < UART_FIFO_SIZE {
            T::register_block()
                .fifo()
                .write(|w| unsafe { w.rxfifo_rd_byte().bits(word) });

            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    fn flush_tx(&self) -> nb::Result<(), Error> {
        if T::is_tx_idle() {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

impl<'d, T, M> UartRx<'d, T, M>
where
    T: Instance,
    M: Mode,
{
    fn new_inner() -> Self {
        Self {
            phantom: PhantomData,
            at_cmd_config: None,
            rx_timeout_config: None,
        }
    }

    fn read_byte(&mut self) -> nb::Result<u8, Error> {
        #[allow(unused_variables)]
        let offset = 0;

        // on ESP32-S2 we need to use PeriBus2 to read the FIFO
        #[cfg(esp32s2)]
        let offset = 0x20c00000;

        if T::get_rx_fifo_count() > 0 {
            let value = unsafe {
                let fifo = (T::register_block().fifo().as_ptr() as *mut u8).offset(offset)
                    as *mut crate::peripherals::generic::Reg<FIFO_SPEC>;
                (*fifo).read().rxfifo_rd_byte().bits()
            };

            Ok(value)
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    /// Read all available bytes from the RX FIFO into the provided buffer and
    /// returns the number of read bytes. Never blocks
    pub fn drain_fifo(&mut self, buf: &mut [u8]) -> usize {
        #[allow(unused_variables)]
        let offset = 0;

        // on ESP32-S2 we need to use PeriBus2 to read the FIFO
        #[cfg(esp32s2)]
        let offset = 0x20c00000;

        let mut count = 0;
        while T::get_rx_fifo_count() > 0 && count < buf.len() {
            let value = unsafe {
                let fifo = (T::register_block().fifo().as_ptr() as *mut u8).offset(offset)
                    as *mut crate::peripherals::generic::Reg<FIFO_SPEC>;
                (*fifo).read().rxfifo_rd_byte().bits()
            };
            buf[count] = value;
            count += 1;
        }
        count
    }
}

impl<'d, T> Uart<'d, T, Blocking>
where
    T: Instance + 'd,
{
    /// Create a new UART instance with configuration options in [`Blocking`]
    /// mode.
    pub fn new_with_config<P>(
        uart: impl Peripheral<P = T> + 'd,
        config: Config,
        pins: Option<P>,
        clocks: &Clocks,
        interrupt: Option<InterruptHandler>,
    ) -> Self
    where
        P: UartPins,
    {
        Self::new_with_config_inner(uart, config, pins, clocks, interrupt)
    }

    /// Create a new UART instance with defaults in [`Blocking`] mode.
    pub fn new(uart: impl Peripheral<P = T> + 'd, clocks: &Clocks) -> Self {
        Self::new_inner(uart, clocks)
    }
}

impl<'d, T, M> Uart<'d, T, M>
where
    T: Instance + 'd,
    M: Mode,
{
    fn new_with_config_inner<P>(
        _uart: impl Peripheral<P = T> + 'd,
        config: Config,
        mut pins: Option<P>,
        clocks: &Clocks,
        interrupt: Option<InterruptHandler>,
    ) -> Self
    where
        P: UartPins,
    {
        Self::init();

        if let Some(ref mut pins) = pins {
            pins.configure_pins(
                T::tx_signal(),
                T::rx_signal(),
                T::cts_signal(),
                T::rts_signal(),
            );
        }

        let mut serial = Uart {
            tx: UartTx::new_inner(),
            rx: UartRx::new_inner(),
            #[cfg(not(esp32))]
            symbol_len: config.symbol_length(),
        };

        serial.change_baud_internal(config.baudrate, clocks);
        serial.change_data_bits(config.data_bits);
        serial.change_parity(config.parity);
        serial.change_stop_bits(config.stop_bits);

        if let Some(interrupt) = interrupt {
            unsafe {
                crate::interrupt::bind_interrupt(T::interrupt(), interrupt.handler());
                crate::interrupt::enable(T::interrupt(), interrupt.priority()).unwrap();
            }
        }

        // Reset Tx/Rx FIFOs
        serial.txfifo_reset();
        serial.rxfifo_reset();
        crate::rom::ets_delay_us(15);

        serial
    }

    fn new_inner(uart: impl Peripheral<P = T> + 'd, clocks: &Clocks) -> Self {
        Self::new_with_config_inner(
            uart,
            Default::default(),
            None::<TxRxPins<'_, NoPinType, NoPinType>>,
            clocks,
            None,
        )
    }

    /// Split the UART into a transmitter and receiver
    ///
    /// This is particularly useful when having two tasks correlating to
    /// transmitting and receiving.
    pub fn split(self) -> (UartTx<'d, T, M>, UartRx<'d, T, M>) {
        (self.tx, self.rx)
    }

    /// Write bytes out over the UART
    pub fn write_bytes(&mut self, data: &[u8]) -> Result<usize, Error> {
        self.tx.write_bytes(data)
    }

    /// Configures the AT-CMD detection settings
    #[allow(clippy::useless_conversion)]
    pub fn set_at_cmd(&mut self, config: config::AtCmdConfig) {
        #[cfg(not(any(esp32, esp32s2)))]
        T::register_block()
            .clk_conf()
            .modify(|_, w| w.sclk_en().clear_bit());

        T::register_block().at_cmd_char().write(|w| unsafe {
            w.at_cmd_char()
                .bits(config.cmd_char)
                .char_num()
                .bits(config.char_num.unwrap_or(1))
        });

        if let Some(pre_idle_count) = config.pre_idle_count {
            T::register_block()
                .at_cmd_precnt()
                .write(|w| unsafe { w.pre_idle_num().bits(pre_idle_count.into()) });
        }

        if let Some(post_idle_count) = config.post_idle_count {
            T::register_block()
                .at_cmd_postcnt()
                .write(|w| unsafe { w.post_idle_num().bits(post_idle_count.into()) });
        }

        if let Some(gap_timeout) = config.gap_timeout {
            T::register_block()
                .at_cmd_gaptout()
                .write(|w| unsafe { w.rx_gap_tout().bits(gap_timeout.into()) });
        }

        #[cfg(not(any(esp32, esp32s2)))]
        T::register_block()
            .clk_conf()
            .modify(|_, w| w.sclk_en().set_bit());

        self.sync_regs();
        self.rx.at_cmd_config = Some(config);
    }

    /// Configures the Receive Timeout detection setting
    ///
    /// # Arguments
    /// `timeout` - the number of symbols ("bytes") to wait for before
    /// triggering a timeout. Pass None to disable the timeout.
    ///
    ///  # Errors
    /// `Err(Error::InvalidArgument)` if the provided value exceeds the maximum
    /// value for SOC :
    /// - `esp32`: Symbol size is fixed to 8, do not pass a value > **0x7F**.
    /// - `esp32c2`, `esp32c3`, `esp32c6`, `esp32h2`, esp32s2`, esp32s3`: The
    ///   value you pass times the symbol size must be <= **0x3FF**
    pub fn set_rx_timeout(&mut self, timeout: Option<u8>) -> Result<(), Error> {
        #[cfg(esp32)]
        const MAX_THRHD: u8 = 0x7F; // 7 bits
        #[cfg(any(esp32c2, esp32c3, esp32c6, esp32h2, esp32s2, esp32s3))]
        const MAX_THRHD: u16 = 0x3FF; // 10 bits

        #[cfg(esp32)]
        let reg_thrhd = &T::register_block().conf1();
        #[cfg(any(esp32c6, esp32h2))]
        let reg_thrhd = &T::register_block().tout_conf();
        #[cfg(any(esp32c2, esp32c3, esp32s2, esp32s3))]
        let reg_thrhd = &T::register_block().mem_conf();

        #[cfg(any(esp32c6, esp32h2))]
        let reg_en = &T::register_block().tout_conf();
        #[cfg(any(esp32, esp32c2, esp32c3, esp32s2, esp32s3))]
        let reg_en = &T::register_block().conf1();

        match timeout {
            None => {
                reg_en.modify(|_, w| w.rx_tout_en().clear_bit());
            }
            Some(timeout) => {
                // the esp32 counts directly in number of symbols (symbol len fixed to 8)
                #[cfg(esp32)]
                let timeout_reg = timeout;
                // all other count in bits, so we need to multiply by the symbol len.
                #[cfg(not(esp32))]
                let timeout_reg = timeout as u16 * self.symbol_len as u16;

                if timeout_reg > MAX_THRHD {
                    return Err(Error::InvalidArgument);
                }

                reg_thrhd.modify(|_, w| unsafe { w.rx_tout_thrhd().bits(timeout_reg) });
                reg_en.modify(|_, w| w.rx_tout_en().set_bit());
            }
        }

        self.rx.rx_timeout_config = timeout;

        self.sync_regs();
        Ok(())
    }

    /// Configures the RX-FIFO threshold
    ///
    /// # Errors
    /// `Err(Error::InvalidArgument)` if provided value exceeds maximum value
    /// for SOC :
    /// - `esp32` **0x7F**
    /// - `esp32c6`, `esp32h2` **0xFF**
    /// - `esp32c3`, `esp32c2`, `esp32s2` **0x1FF**
    /// - `esp32s3` **0x3FF**
    pub fn set_rx_fifo_full_threshold(&mut self, threshold: u16) -> Result<(), Error> {
        #[cfg(esp32)]
        const MAX_THRHD: u16 = 0x7F;
        #[cfg(any(esp32c6, esp32h2))]
        const MAX_THRHD: u16 = 0xFF;
        #[cfg(any(esp32c3, esp32c2, esp32s2))]
        const MAX_THRHD: u16 = 0x1FF;
        #[cfg(esp32s3)]
        const MAX_THRHD: u16 = 0x3FF;

        if threshold > MAX_THRHD {
            return Err(Error::InvalidArgument);
        }

        #[cfg(any(esp32, esp32c6, esp32h2))]
        let threshold: u8 = threshold as u8;

        T::register_block()
            .conf1()
            .modify(|_, w| unsafe { w.rxfifo_full_thrhd().bits(threshold) });

        Ok(())
    }

    /// Listen for AT-CMD interrupts
    pub fn listen_at_cmd(&mut self) {
        T::register_block()
            .int_ena()
            .modify(|_, w| w.at_cmd_char_det().set_bit());
    }

    /// Stop listening for AT-CMD interrupts
    pub fn unlisten_at_cmd(&mut self) {
        T::register_block()
            .int_ena()
            .modify(|_, w| w.at_cmd_char_det().clear_bit());
    }

    /// Listen for TX-DONE interrupts
    pub fn listen_tx_done(&mut self) {
        T::register_block()
            .int_ena()
            .modify(|_, w| w.tx_done().set_bit());
    }

    /// Stop listening for TX-DONE interrupts
    pub fn unlisten_tx_done(&mut self) {
        T::register_block()
            .int_ena()
            .modify(|_, w| w.tx_done().clear_bit());
    }

    /// Listen for RX-FIFO-FULL interrupts
    pub fn listen_rx_fifo_full(&mut self) {
        T::register_block()
            .int_ena()
            .modify(|_, w| w.rxfifo_full().set_bit());
    }

    /// Stop listening for RX-FIFO-FULL interrupts
    pub fn unlisten_rx_fifo_full(&mut self) {
        T::register_block()
            .int_ena()
            .modify(|_, w| w.rxfifo_full().clear_bit());
    }

    /// Checks if AT-CMD interrupt is set
    pub fn at_cmd_interrupt_set(&self) -> bool {
        T::register_block()
            .int_raw()
            .read()
            .at_cmd_char_det()
            .bit_is_set()
    }

    /// Checks if TX-DONE interrupt is set
    pub fn tx_done_interrupt_set(&self) -> bool {
        T::register_block().int_raw().read().tx_done().bit_is_set()
    }

    /// Checks if RX-FIFO-FULL interrupt is set
    pub fn rx_fifo_full_interrupt_set(&self) -> bool {
        T::register_block()
            .int_raw()
            .read()
            .rxfifo_full()
            .bit_is_set()
    }

    /// Reset AT-CMD interrupt
    pub fn reset_at_cmd_interrupt(&self) {
        T::register_block()
            .int_clr()
            .write(|w| w.at_cmd_char_det().clear_bit_by_one());
    }

    /// Reset TX-DONE interrupt
    pub fn reset_tx_done_interrupt(&self) {
        T::register_block()
            .int_clr()
            .write(|w| w.tx_done().clear_bit_by_one());
    }

    /// Reset RX-FIFO-FULL interrupt
    pub fn reset_rx_fifo_full_interrupt(&self) {
        T::register_block()
            .int_clr()
            .write(|w| w.rxfifo_full().clear_bit_by_one());
    }

    /// Write a byte out over the UART
    pub fn write_byte(&mut self, word: u8) -> nb::Result<(), Error> {
        self.tx.write_byte(word)
    }

    /// Flush the transmit buffer of the UART
    pub fn flush_tx(&self) -> nb::Result<(), Error> {
        self.tx.flush_tx()
    }

    /// Read a byte from the UART
    pub fn read_byte(&mut self) -> nb::Result<u8, Error> {
        self.rx.read_byte()
    }

    /// Change the number of stop bits
    pub fn change_stop_bits(&mut self, stop_bits: config::StopBits) -> &mut Self {
        // workaround for hardware issue, when UART stop bit set as 2-bit mode.
        #[cfg(esp32)]
        if stop_bits == config::StopBits::STOP2 {
            T::register_block()
                .rs485_conf()
                .modify(|_, w| w.dl1_en().bit(true));

            T::register_block()
                .conf0()
                .modify(|_, w| unsafe { w.stop_bit_num().bits(1) });
        } else {
            T::register_block()
                .rs485_conf()
                .modify(|_, w| w.dl1_en().bit(false));

            T::register_block()
                .conf0()
                .modify(|_, w| unsafe { w.stop_bit_num().bits(stop_bits as u8) });
        }

        #[cfg(not(esp32))]
        T::register_block()
            .conf0()
            .modify(|_, w| unsafe { w.stop_bit_num().bits(stop_bits as u8) });

        self
    }

    fn change_data_bits(&mut self, data_bits: config::DataBits) -> &mut Self {
        T::register_block()
            .conf0()
            .modify(|_, w| unsafe { w.bit_num().bits(data_bits as u8) });

        self
    }

    fn change_parity(&mut self, parity: config::Parity) -> &mut Self {
        T::register_block().conf0().modify(|_, w| match parity {
            config::Parity::ParityNone => w.parity_en().clear_bit(),
            config::Parity::ParityEven => w.parity_en().set_bit().parity().clear_bit(),
            config::Parity::ParityOdd => w.parity_en().set_bit().parity().set_bit(),
        });

        self
    }

    #[cfg(any(esp32c2, esp32c3, esp32s3))]
    fn change_baud_internal(&self, baudrate: u32, clocks: &Clocks) {
        // we force the clock source to be APB and don't use the decimal part of the
        // divider
        let clk = clocks.apb_clock.to_Hz();
        let max_div = 0b1111_1111_1111 - 1;
        let clk_div = ((clk) + (max_div * baudrate) - 1) / (max_div * baudrate);

        T::register_block().clk_conf().write(|w| unsafe {
            w.sclk_sel()
                .bits(1) // APB
                .sclk_div_a()
                .bits(0)
                .sclk_div_b()
                .bits(0)
                .sclk_div_num()
                .bits(clk_div as u8 - 1)
                .rx_sclk_en()
                .bit(true)
                .tx_sclk_en()
                .bit(true)
        });

        let clk = clk / clk_div;
        let divider = clk / baudrate;
        let divider = divider as u16;

        T::register_block()
            .clkdiv()
            .write(|w| unsafe { w.clkdiv().bits(divider).frag().bits(0) });
    }

    #[cfg(any(esp32c6, esp32h2))]
    fn change_baud_internal(&self, baudrate: u32, clocks: &Clocks) {
        // we force the clock source to be XTAL and don't use the decimal part of
        // the divider
        let clk = clocks.xtal_clock.to_Hz();
        let max_div = 0b1111_1111_1111 - 1;
        let clk_div = ((clk) + (max_div * baudrate) - 1) / (max_div * baudrate);

        // UART clocks are configured via PCR
        let pcr = unsafe { &*crate::peripherals::PCR::PTR };

        match T::uart_number() {
            0 => {
                pcr.uart0_conf()
                    .modify(|_, w| w.uart0_rst_en().clear_bit().uart0_clk_en().set_bit());

                pcr.uart0_sclk_conf().modify(|_, w| unsafe {
                    w.uart0_sclk_div_a()
                        .bits(0)
                        .uart0_sclk_div_b()
                        .bits(0)
                        .uart0_sclk_div_num()
                        .bits(clk_div as u8 - 1)
                        .uart0_sclk_sel()
                        .bits(0x3) // TODO: this probably shouldn't be hard-coded
                        .uart0_sclk_en()
                        .set_bit()
                });
            }
            1 => {
                pcr.uart1_conf()
                    .modify(|_, w| w.uart1_rst_en().clear_bit().uart1_clk_en().set_bit());

                pcr.uart1_sclk_conf().modify(|_, w| unsafe {
                    w.uart1_sclk_div_a()
                        .bits(0)
                        .uart1_sclk_div_b()
                        .bits(0)
                        .uart1_sclk_div_num()
                        .bits(clk_div as u8 - 1)
                        .uart1_sclk_sel()
                        .bits(0x3) // TODO: this probably shouldn't be hard-coded
                        .uart1_sclk_en()
                        .set_bit()
                });
            }
            _ => unreachable!(), // ESP32-C6 only has 2 UART instances
        }

        let clk = clk / clk_div;
        let divider = clk / baudrate;
        let divider = divider as u16;

        T::register_block()
            .clkdiv()
            .write(|w| unsafe { w.clkdiv().bits(divider).frag().bits(0) });

        self.sync_regs();
    }

    #[cfg(any(esp32, esp32s2))]
    fn change_baud_internal(&self, baudrate: u32, clocks: &Clocks) {
        // we force the clock source to be APB and don't use the decimal part of the
        // divider
        let clk = clocks.apb_clock.to_Hz();

        T::register_block()
            .conf0()
            .modify(|_, w| w.tick_ref_always_on().bit(true));
        let divider = clk / baudrate;

        T::register_block()
            .clkdiv()
            .write(|w| unsafe { w.clkdiv().bits(divider).frag().bits(0) });
    }

    #[cfg(any(esp32c2, esp32c3, esp32s3))]
    #[inline(always)]
    fn init() {
        let system = unsafe { crate::peripherals::SYSTEM::steal() };
        if !system.perip_clk_en0().read().uart_mem_clk_en().bit() {
            system
                .perip_clk_en0()
                .modify(|_, w| w.uart_mem_clk_en().set_bit());
        }

        // initialize peripheral by setting clk_enable and clearing uart_reset bits
        T::enable_peripheral();
        Self::uart_peripheral_reset();
        T::disable_rx_interrupts();
        T::disable_tx_interrupts();
    }

    /// Modify UART baud rate and reset TX/RX fifo.
    pub fn change_baud(&mut self, baudrate: u32, clocks: &Clocks) {
        self.change_baud_internal(baudrate, clocks);
        self.txfifo_reset();
        self.rxfifo_reset();
    }

    #[cfg(any(esp32c6, esp32h2))]
    #[inline(always)]
    fn init() {
        T::register_block()
            .conf0()
            .modify(|_, w| w.mem_clk_en().set_bit());

        // initialize peripheral by setting clk_enable and clearing uart_reset bits
        T::enable_peripheral();
        Self::uart_peripheral_reset();
        T::disable_rx_interrupts();
        T::disable_tx_interrupts();
    }

    #[cfg(any(esp32, esp32s2))]
    #[inline(always)]
    fn init() {
        T::enable_peripheral();
        Self::uart_peripheral_reset();
        T::disable_rx_interrupts();
        T::disable_tx_interrupts();
    }

    #[inline(always)]
    fn uart_peripheral_reset() {
        // don't reset the console UART - this will cause trouble (i.e. the UART will
        // start to transmit garbage)
        //
        // We should only reset the console UART if it was absolutely unused before.
        // Apparently the bootloader (and maybe the ROM code) writing to the UART is
        // already enough to make this a no-go. (i.e. one needs to mute the ROM
        // code via efuse / strapping pin AND use a silent bootloader)
        //
        // Ideally this should be configurable once we have a solution for https://github.com/esp-rs/esp-hal/issues/1111
        // see https://github.com/espressif/esp-idf/blob/5f4249357372f209fdd57288265741aaba21a2b1/components/esp_driver_uart/src/uart.c#L179
        if T::uart_number() != CONSOLE_UART_NUM {
            #[cfg(not(any(esp32, esp32s2)))]
            T::register_block()
                .clk_conf()
                .modify(|_, w| w.rst_core().set_bit());

            // reset peripheral
            T::reset_peripheral();

            #[cfg(not(any(esp32, esp32s2)))]
            T::register_block()
                .clk_conf()
                .modify(|_, w| w.rst_core().clear_bit());
        }
    }

    #[cfg(any(esp32c3, esp32c6, esp32h2, esp32s3))] // TODO introduce a cfg symbol for this
    #[inline(always)]
    fn sync_regs(&self) {
        #[cfg(any(esp32c6, esp32h2))]
        let update_reg = T::register_block().reg_update();

        #[cfg(any(esp32c3, esp32s3))]
        let update_reg = T::register_block().id();

        update_reg.modify(|_, w| w.reg_update().set_bit());

        while update_reg.read().reg_update().bit_is_set() {
            // wait
        }
    }

    #[cfg(not(any(esp32c3, esp32c6, esp32h2, esp32s3)))]
    #[inline(always)]
    fn sync_regs(&mut self) {}

    fn rxfifo_reset(&mut self) {
        T::register_block()
            .conf0()
            .modify(|_, w| w.rxfifo_rst().set_bit());
        self.sync_regs();

        T::register_block()
            .conf0()
            .modify(|_, w| w.rxfifo_rst().clear_bit());
        self.sync_regs();
    }

    fn txfifo_reset(&mut self) {
        T::register_block()
            .conf0()
            .modify(|_, w| w.txfifo_rst().set_bit());
        self.sync_regs();

        T::register_block()
            .conf0()
            .modify(|_, w| w.txfifo_rst().clear_bit());
        self.sync_regs();
    }
}

/// UART Peripheral Instance
pub trait Instance: crate::private::Sealed {
    fn register_block() -> &'static RegisterBlock;
    fn uart_number() -> usize;
    fn interrupt() -> Interrupt;

    fn disable_tx_interrupts() {
        Self::register_block().int_clr().write(|w| {
            w.txfifo_empty()
                .clear_bit_by_one()
                .tx_brk_done()
                .clear_bit_by_one()
                .tx_brk_idle_done()
                .clear_bit_by_one()
                .tx_done()
                .clear_bit_by_one()
        });

        Self::register_block().int_ena().write(|w| {
            w.txfifo_empty()
                .clear_bit()
                .tx_brk_done()
                .clear_bit()
                .tx_brk_idle_done()
                .clear_bit()
                .tx_done()
                .clear_bit()
        });
    }

    fn disable_rx_interrupts() {
        Self::register_block().int_clr().write(|w| {
            w.rxfifo_full()
                .clear_bit_by_one()
                .rxfifo_ovf()
                .clear_bit_by_one()
                .rxfifo_tout()
                .clear_bit_by_one()
                .at_cmd_char_det()
                .clear_bit_by_one()
        });

        Self::register_block().int_ena().write(|w| {
            w.rxfifo_full()
                .clear_bit()
                .rxfifo_ovf()
                .clear_bit()
                .rxfifo_tout()
                .clear_bit()
                .at_cmd_char_det()
                .clear_bit()
        });
    }

    #[allow(clippy::useless_conversion)]
    fn get_tx_fifo_count() -> u16 {
        Self::register_block()
            .status()
            .read()
            .txfifo_cnt()
            .bits()
            .into()
    }

    #[allow(clippy::useless_conversion)]
    fn get_rx_fifo_count() -> u16 {
        let fifo_cnt: u16 = Self::register_block()
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
            let rd_addr = Self::register_block()
                .mem_rx_status()
                .read()
                .mem_rx_rd_addr()
                .bits();
            let wr_addr = Self::register_block()
                .mem_rx_status()
                .read()
                .mem_rx_wr_addr()
                .bits();

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

    fn is_tx_idle() -> bool {
        #[cfg(esp32)]
        let idle = Self::register_block().status().read().st_utx_out().bits() == 0x0u8;
        #[cfg(not(esp32))]
        let idle = Self::register_block()
            .fsm_status()
            .read()
            .st_utx_out()
            .bits()
            == 0x0u8;

        idle
    }

    fn is_rx_idle() -> bool {
        #[cfg(esp32)]
        let idle = Self::register_block().status().read().st_urx_out().bits() == 0x0u8;
        #[cfg(not(esp32))]
        let idle = Self::register_block()
            .fsm_status()
            .read()
            .st_urx_out()
            .bits()
            == 0x0u8;

        idle
    }

    fn tx_signal() -> OutputSignal;
    fn rx_signal() -> InputSignal;
    fn cts_signal() -> InputSignal;
    fn rts_signal() -> OutputSignal;
    fn enable_peripheral();
    fn reset_peripheral();
}

macro_rules! impl_instance {
    ($inst:ident, $num:expr, $txd:ident, $rxd:ident, $cts:ident, $rts:ident, $peri:ident) => {
        impl Instance for crate::peripherals::$inst {
            #[inline(always)]
            fn register_block() -> &'static RegisterBlock {
                unsafe { &*crate::peripherals::$inst::PTR }
            }

            #[inline(always)]
            fn uart_number() -> usize {
                $num
            }

            #[inline(always)]
            fn interrupt() -> Interrupt {
                Interrupt::$inst
            }

            fn tx_signal() -> OutputSignal {
                OutputSignal::$txd
            }

            fn rx_signal() -> InputSignal {
                InputSignal::$rxd
            }

            fn cts_signal() -> InputSignal {
                InputSignal::$cts
            }

            fn rts_signal() -> OutputSignal {
                OutputSignal::$rts
            }

            fn enable_peripheral() {
                PeripheralClockControl::enable(crate::system::Peripheral::$peri);
            }

            fn reset_peripheral() {
                PeripheralClockControl::reset(crate::system::Peripheral::$peri);
            }
        }
    };
}

impl_instance!(UART0, 0, U0TXD, U0RXD, U0CTS, U0RTS, Uart0);
impl_instance!(UART1, 1, U1TXD, U1RXD, U1CTS, U1RTS, Uart1);
#[cfg(uart2)]
impl_instance!(UART2, 2, U2TXD, U2RXD, U2CTS, U2RTS, Uart2);

#[cfg(feature = "ufmt")]
impl<T, M> ufmt_write::uWrite for Uart<'_, T, M>
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

#[cfg(feature = "ufmt")]
impl<T, M> ufmt_write::uWrite for UartTx<'_, T, M>
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

impl<T, M> core::fmt::Write for Uart<'_, T, M>
where
    T: Instance,
    M: Mode,
{
    #[inline]
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.tx.write_str(s)
    }
}

impl<T, M> core::fmt::Write for UartTx<'_, T, M>
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

#[cfg(feature = "embedded-hal-02")]
impl<T, M> embedded_hal_02::serial::Write<u8> for Uart<'_, T, M>
where
    T: Instance,
    M: Mode,
{
    type Error = Error;

    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        self.tx.write(word)
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        self.tx.flush()
    }
}

#[cfg(feature = "embedded-hal-02")]
impl<T, M> embedded_hal_02::serial::Write<u8> for UartTx<'_, T, M>
where
    T: Instance,
    M: Mode,
{
    type Error = Error;

    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        self.write_byte(word)
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        self.flush_tx()
    }
}

#[cfg(feature = "embedded-hal-02")]
impl<T, M> embedded_hal_02::serial::Read<u8> for Uart<'_, T, M>
where
    T: Instance,
    M: Mode,
{
    type Error = Error;

    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        self.rx.read()
    }
}

#[cfg(feature = "embedded-hal-02")]
impl<T, M> embedded_hal_02::serial::Read<u8> for UartRx<'_, T, M>
where
    T: Instance,
    M: Mode,
{
    type Error = Error;

    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        self.read_byte()
    }
}

#[cfg(feature = "embedded-hal")]
impl<T, M> embedded_hal_nb::serial::ErrorType for Uart<'_, T, M> {
    type Error = Error;
}

#[cfg(feature = "embedded-hal")]
impl<T, M> embedded_hal_nb::serial::ErrorType for UartTx<'_, T, M> {
    type Error = Error;
}

#[cfg(feature = "embedded-hal")]
impl<T, M> embedded_hal_nb::serial::ErrorType for UartRx<'_, T, M> {
    type Error = Error;
}

#[cfg(feature = "embedded-hal")]
impl<T, M> embedded_hal_nb::serial::Read for Uart<'_, T, M>
where
    T: Instance,
    M: Mode,
{
    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        self.read_byte()
    }
}

#[cfg(feature = "embedded-hal")]
impl<T, M> embedded_hal_nb::serial::Read for UartRx<'_, T, M>
where
    T: Instance,
    M: Mode,
{
    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        self.read_byte()
    }
}

#[cfg(feature = "embedded-hal")]
impl<T, M> embedded_hal_nb::serial::Write for Uart<'_, T, M>
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

#[cfg(feature = "embedded-hal")]
impl<T, M> embedded_hal_nb::serial::Write for UartTx<'_, T, M>
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

#[cfg(feature = "embedded-io")]
impl<T, M> embedded_io::ErrorType for Uart<'_, T, M> {
    type Error = Error;
}

#[cfg(feature = "embedded-io")]
impl<T, M> embedded_io::ErrorType for UartTx<'_, T, M> {
    type Error = Error;
}

#[cfg(feature = "embedded-io")]
impl<T, M> embedded_io::ErrorType for UartRx<'_, T, M> {
    type Error = Error;
}

#[cfg(feature = "embedded-io")]
impl<T, M> embedded_io::Read for Uart<'_, T, M>
where
    T: Instance,
    M: Mode,
{
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.rx.read(buf)
    }
}

#[cfg(feature = "embedded-io")]
impl<T, M> embedded_io::Read for UartRx<'_, T, M>
where
    T: Instance,
    M: Mode,
{
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        if buf.len() == 0 {
            return Ok(0);
        }

        while T::get_rx_fifo_count() == 0 {
            // Block until we received at least one byte
        }

        Ok(self.drain_fifo(buf))
    }
}

#[cfg(feature = "embedded-io")]
impl<T, M> embedded_io::Write for Uart<'_, T, M>
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

#[cfg(feature = "embedded-io")]
impl<T, M> embedded_io::Write for UartTx<'_, T, M>
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

#[cfg(feature = "async")]
mod asynch {
    use core::task::Poll;

    use cfg_if::cfg_if;
    use embassy_sync::waitqueue::AtomicWaker;
    use enumset::{EnumSet, EnumSetType};
    use procmacros::handler;

    use super::*;
    use crate::Async;

    cfg_if! {
        if #[cfg(all(uart0, uart1, uart2))] {
            const NUM_UART: usize = 3;
        } else if #[cfg(all(uart0, uart1))] {
            const NUM_UART: usize = 2;
        } else if #[cfg(uart0)] {
            const NUM_UART: usize = 1;
        }
    }

    const INIT: AtomicWaker = AtomicWaker::new();
    static TX_WAKERS: [AtomicWaker; NUM_UART] = [INIT; NUM_UART];
    static RX_WAKERS: [AtomicWaker; NUM_UART] = [INIT; NUM_UART];

    #[derive(EnumSetType, Debug)]
    pub(crate) enum TxEvent {
        TxDone,
        TxFiFoEmpty,
    }
    #[derive(EnumSetType, Debug)]
    pub(crate) enum RxEvent {
        RxFifoFull,
        RxCmdCharDetected,
        RxFifoOvf,
        RxFifoTout,
    }

    /// A future that resolves when the passed interrupt is triggered,
    /// or has been triggered in the meantime (flag set in INT_RAW).
    /// Upon construction the future enables the passed interrupt and when it
    /// is dropped it disables the interrupt again. The future returns the event
    /// that was initially passed, when it resolves.
    pub(crate) struct UartRxFuture<'d, T: Instance> {
        events: EnumSet<RxEvent>,
        phantom: PhantomData<&'d mut T>,
        registered: bool,
    }
    pub(crate) struct UartTxFuture<'d, T: Instance> {
        events: EnumSet<TxEvent>,
        phantom: PhantomData<&'d mut T>,
        registered: bool,
    }

    impl<'d, T: Instance> UartRxFuture<'d, T> {
        pub fn new(events: EnumSet<RxEvent>) -> Self {
            Self {
                events,
                phantom: PhantomData,
                registered: false,
            }
        }

        fn event_bit_is_clear(&self) -> bool {
            let interrupts_enabled = T::register_block().int_ena().read();
            let mut event_triggered = false;
            for event in self.events {
                event_triggered |= match event {
                    RxEvent::RxFifoFull => interrupts_enabled.rxfifo_full().bit_is_clear(),
                    RxEvent::RxCmdCharDetected => {
                        interrupts_enabled.at_cmd_char_det().bit_is_clear()
                    }

                    RxEvent::RxFifoOvf => interrupts_enabled.rxfifo_ovf().bit_is_clear(),
                    RxEvent::RxFifoTout => interrupts_enabled.rxfifo_tout().bit_is_clear(),
                }
            }
            event_triggered
        }
    }

    impl<'d, T: Instance> core::future::Future for UartRxFuture<'d, T> {
        type Output = ();

        fn poll(
            mut self: core::pin::Pin<&mut Self>,
            cx: &mut core::task::Context<'_>,
        ) -> core::task::Poll<Self::Output> {
            if !self.registered {
                RX_WAKERS[T::uart_number()].register(cx.waker());
                T::register_block().int_ena().modify(|_, w| {
                    for event in self.events {
                        match event {
                            RxEvent::RxFifoFull => w.rxfifo_full().set_bit(),
                            RxEvent::RxCmdCharDetected => w.at_cmd_char_det().set_bit(),
                            RxEvent::RxFifoOvf => w.rxfifo_ovf().set_bit(),
                            RxEvent::RxFifoTout => w.rxfifo_tout().set_bit(),
                        };
                    }
                    w
                });
                self.registered = true;
            }
            if self.event_bit_is_clear() {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        }
    }

    impl<'d, T: Instance> Drop for UartRxFuture<'d, T> {
        fn drop(&mut self) {
            // Although the isr disables the interrupt that occurred directly, we need to
            // disable the other interrupts (= the ones that did not occur), as
            // soon as this future goes out of scope.
            let int_ena = &T::register_block().int_ena();
            for event in self.events {
                match event {
                    RxEvent::RxFifoFull => int_ena.modify(|_, w| w.rxfifo_full().clear_bit()),
                    RxEvent::RxCmdCharDetected => {
                        int_ena.modify(|_, w| w.at_cmd_char_det().clear_bit())
                    }
                    RxEvent::RxFifoOvf => int_ena.modify(|_, w| w.rxfifo_ovf().clear_bit()),
                    RxEvent::RxFifoTout => int_ena.modify(|_, w| w.rxfifo_tout().clear_bit()),
                }
            }
        }
    }

    impl<'d, T: Instance> UartTxFuture<'d, T> {
        pub fn new(events: EnumSet<TxEvent>) -> Self {
            Self {
                events,
                phantom: PhantomData,
                registered: false,
            }
        }

        fn event_bit_is_clear(&self) -> bool {
            let interrupts_enabled = T::register_block().int_ena().read();
            let mut event_triggered = false;
            for event in self.events {
                event_triggered |= match event {
                    TxEvent::TxDone => interrupts_enabled.tx_done().bit_is_clear(),
                    TxEvent::TxFiFoEmpty => interrupts_enabled.txfifo_empty().bit_is_clear(),
                }
            }
            event_triggered
        }
    }

    impl<'d, T: Instance> core::future::Future for UartTxFuture<'d, T> {
        type Output = ();

        fn poll(
            mut self: core::pin::Pin<&mut Self>,
            cx: &mut core::task::Context<'_>,
        ) -> core::task::Poll<Self::Output> {
            if !self.registered {
                TX_WAKERS[T::uart_number()].register(cx.waker());
                T::register_block().int_ena().modify(|_, w| {
                    for event in self.events {
                        match event {
                            TxEvent::TxDone => w.tx_done().set_bit(),
                            TxEvent::TxFiFoEmpty => w.txfifo_empty().set_bit(),
                        };
                    }
                    w
                });
                self.registered = true;
            }

            if self.event_bit_is_clear() {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        }
    }

    impl<'d, T: Instance> Drop for UartTxFuture<'d, T> {
        fn drop(&mut self) {
            // Although the isr disables the interrupt that occurred directly, we need to
            // disable the other interrupts (= the ones that did not occur), as
            // soon as this future goes out of scope.
            let int_ena = &T::register_block().int_ena();
            for event in self.events {
                match event {
                    TxEvent::TxDone => int_ena.modify(|_, w| w.tx_done().clear_bit()),
                    TxEvent::TxFiFoEmpty => int_ena.modify(|_, w| w.txfifo_empty().clear_bit()),
                }
            }
        }
    }

    impl<'d, T> Uart<'d, T, Async>
    where
        T: Instance + 'd,
    {
        /// Create a new UART instance with configuration options in [`Async`]
        /// mode.
        pub fn new_async_with_config<P>(
            uart: impl Peripheral<P = T> + 'd,
            config: Config,
            pins: Option<P>,
            clocks: &Clocks,
        ) -> Self
        where
            P: UartPins,
        {
            Self::new_with_config_inner(
                uart,
                config,
                pins,
                clocks,
                Some(match T::uart_number() {
                    #[cfg(uart0)]
                    0 => uart0,
                    #[cfg(uart1)]
                    1 => uart1,
                    #[cfg(uart2)]
                    2 => uart2,
                    _ => unreachable!(),
                }),
            )
        }

        /// Create a new UART instance with defaults in [`Async`] mode.
        pub fn new_async(uart: impl Peripheral<P = T> + 'd, clocks: &Clocks) -> Self {
            Self::new_async_with_config(
                uart,
                Default::default(),
                None::<TxRxPins<'_, NoPinType, NoPinType>>,
                clocks,
            )
        }
    }

    impl<T> Uart<'_, T, Async>
    where
        T: Instance,
    {
        /// See [`UartRx::read_async`]
        pub async fn read_async(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
            self.rx.read_async(buf).await
        }

        pub async fn write_async(&mut self, words: &[u8]) -> Result<usize, Error> {
            self.tx.write_async(words).await
        }

        pub async fn flush_async(&mut self) -> Result<(), Error> {
            self.tx.flush_async().await
        }
    }

    impl<T> UartTx<'_, T, Async>
    where
        T: Instance,
    {
        pub async fn write_async(&mut self, words: &[u8]) -> Result<usize, Error> {
            let mut count = 0;
            let mut offset: usize = 0;
            loop {
                let mut next_offset = offset + (UART_FIFO_SIZE - T::get_tx_fifo_count()) as usize;
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
                UartTxFuture::<T>::new(TxEvent::TxFiFoEmpty.into()).await;
            }

            Ok(count)
        }

        pub async fn flush_async(&mut self) -> Result<(), Error> {
            let count = T::get_tx_fifo_count();
            if count > 0 {
                UartTxFuture::<T>::new(TxEvent::TxDone.into()).await;
            }

            Ok(())
        }
    }

    impl<T> UartRx<'_, T, Async>
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
        /// This method will never return Ok(0), unless buf.len() == 0.
        pub async fn read_async(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
            if buf.len() == 0 {
                return Ok(0);
            }

            loop {
                let mut events = RxEvent::RxFifoFull | RxEvent::RxFifoOvf;

                if self.at_cmd_config.is_some() {
                    events |= RxEvent::RxCmdCharDetected;
                }

                if self.rx_timeout_config.is_some() {
                    events |= RxEvent::RxFifoTout;
                }
                UartRxFuture::<T>::new(events).await;

                let read_bytes = self.drain_fifo(buf);
                if read_bytes > 0 {
                    // Unfortunately, the uart's rx-timeout counter counts up whenever there is
                    // data in the fifo, even if the interrupt is disabled and the status bit
                    // cleared. Since we do not drain the fifo in the interrupt handler, we need to
                    // reset the counter here, after draining the fifo.
                    T::register_block()
                        .int_clr()
                        .write(|w| w.rxfifo_tout().clear_bit_by_one());

                    return Ok(read_bytes);
                }
            }
        }
    }

    impl<T> embedded_io_async::Read for Uart<'_, T, Async>
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

    impl<T> embedded_io_async::Read for UartRx<'_, T, Async>
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

    impl<T> embedded_io_async::Write for Uart<'_, T, Async>
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

    impl<T> embedded_io_async::Write for UartTx<'_, T, Async>
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
    fn intr_handler(uart: &RegisterBlock) -> (bool, bool) {
        let interrupts = uart.int_st().read();
        let interrupt_bits = interrupts.bits(); // = int_raw & int_ena
        if interrupt_bits == 0 {
            return (false, false);
        }
        let rx_wake = interrupts.rxfifo_full().bit_is_set()
            || interrupts.rxfifo_ovf().bit_is_set()
            || interrupts.rxfifo_tout().bit_is_set()
            || interrupts.at_cmd_char_det().bit_is_set();
        let tx_wake = interrupts.tx_done().bit_is_set() || interrupts.txfifo_empty().bit_is_set();
        uart.int_clr().write(|w| unsafe { w.bits(interrupt_bits) });
        uart.int_ena()
            .modify(|r, w| unsafe { w.bits(r.bits() & !interrupt_bits) });

        (rx_wake, tx_wake)
    }

    #[cfg(uart0)]
    #[handler]
    fn uart0() {
        let uart = unsafe { &*crate::peripherals::UART0::ptr() };
        let (rx, tx) = intr_handler(uart);
        if rx {
            RX_WAKERS[0].wake();
        }
        if tx {
            TX_WAKERS[0].wake();
        }
    }

    #[cfg(uart1)]
    #[handler]
    fn uart1() {
        let uart = unsafe { &*crate::peripherals::UART1::ptr() };
        let (rx, tx) = intr_handler(uart);
        if rx {
            RX_WAKERS[1].wake();
        }
        if tx {
            TX_WAKERS[1].wake();
        }
    }

    #[cfg(uart2)]
    #[handler]
    fn uart2() {
        let uart = unsafe { &*crate::peripherals::UART2::ptr() };
        let (rx, tx) = intr_handler(uart);
        if rx {
            RX_WAKERS[2].wake();
        }
        if tx {
            TX_WAKERS[2].wake();
        }
    }
}

/// Low-power UART
#[cfg(lp_uart)]
pub mod lp_uart {
    use crate::{
        gpio::{lp_gpio::LowPowerPin, Floating, Input, Output, PushPull},
        peripherals::{LP_CLKRST, LP_UART},
        uart::{config, config::Config},
    };
    /// UART driver
    pub struct LpUart {
        uart: LP_UART,
    }

    impl LpUart {
        /// Initialize the UART driver using the default configuration
        // TODO: CTS and RTS pins
        pub fn new(
            uart: LP_UART,
            _tx: LowPowerPin<Output<PushPull>, 5>,
            _rx: LowPowerPin<Input<Floating>, 4>,
        ) -> Self {
            let lp_io = unsafe { &*crate::peripherals::LP_IO::PTR };
            let lp_aon = unsafe { &*crate::peripherals::LP_AON::PTR };

            lp_aon
                .gpio_mux()
                .modify(|r, w| w.sel().variant(r.sel().bits() | 1 << 4));
            lp_aon
                .gpio_mux()
                .modify(|r, w| w.sel().variant(r.sel().bits() | 1 << 5));

            lp_io.gpio4().modify(|_, w| w.mcu_sel().variant(1));
            lp_io.gpio5().modify(|_, w| w.mcu_sel().variant(1));

            Self::new_with_config(uart, Config::default())
        }

        /// Initialize the UART driver using the provided configuration
        pub fn new_with_config(uart: LP_UART, config: Config) -> Self {
            let mut me = Self { uart };

            // Set UART mode - do nothing for LP

            // Disable UART parity
            // 8-bit world
            // 1-bit stop bit
            me.uart.conf0().modify(|_, w| unsafe {
                w.parity()
                    .clear_bit()
                    .parity_en()
                    .clear_bit()
                    .bit_num()
                    .bits(0x3)
                    .stop_bit_num()
                    .bits(0x1)
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
            unsafe { &*LP_CLKRST::PTR }
                .lpperi()
                .modify(|_, w| w.lp_uart_clk_sel().clear_bit());

            // Override protocol parameters from the configuration
            // uart_hal_set_baudrate(&hal, cfg->uart_proto_cfg.baud_rate, sclk_freq);
            me.change_baud_internal(config.baudrate);
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

        fn change_baud_internal(&mut self, baudrate: u32) {
            // we force the clock source to be XTAL and don't use the decimal part of
            // the divider
            // TODO: Currently it's not possible to use XtalD2Clk
            let clk = 16_000_000;
            let max_div = 0b1111_1111_1111 - 1;
            let clk_div = ((clk) + (max_div * baudrate) - 1) / (max_div * baudrate);

            self.uart.clk_conf().modify(|_, w| unsafe {
                w.sclk_div_a()
                    .bits(0)
                    .sclk_div_b()
                    .bits(0)
                    .sclk_div_num()
                    .bits(clk_div as u8 - 1)
                    .sclk_sel()
                    .bits(0x3) // TODO: this probably shouldn't be hard-coded
                    .sclk_en()
                    .set_bit()
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
        pub fn change_baud(&mut self, baudrate: u32) {
            self.change_baud_internal(baudrate);
            self.txfifo_reset();
            self.rxfifo_reset();
        }

        fn change_parity(&mut self, parity: config::Parity) -> &mut Self {
            if parity != config::Parity::ParityNone {
                self.uart
                    .conf0()
                    .modify(|_, w| w.parity().bit((parity as u8 & 0x1) != 0));
            }

            self.uart.conf0().modify(|_, w| match parity {
                config::Parity::ParityNone => w.parity_en().clear_bit(),
                config::Parity::ParityEven => w.parity_en().set_bit().parity().clear_bit(),
                config::Parity::ParityOdd => w.parity_en().set_bit().parity().set_bit(),
            });

            self
        }

        fn change_data_bits(&mut self, data_bits: config::DataBits) -> &mut Self {
            self.uart
                .conf0()
                .modify(|_, w| unsafe { w.bit_num().bits(data_bits as u8) });

            self.update();

            self
        }

        fn change_stop_bits(&mut self, stop_bits: config::StopBits) -> &mut Self {
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
