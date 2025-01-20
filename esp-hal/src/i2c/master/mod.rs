//! # Inter-Integrated Circuit (I2C) - Master mode
//!
//! ## Overview
//!
//! In this mode, the I2C acts as master and initiates the I2C communication by
//! generating a START condition. Note that only one master is allowed to occupy
//! the bus to access one salve at the same time.
//!
//! ## Configuration
//!
//! Each I2C Master controller is individually configurable, and the usual
//! setting such as frequency, timeout, and SDA/SCL pins can easily be
//! configured.
//!
//! ## Usage
//!
//! The I2C driver implements a number of third-party traits, with the
//! intention of making the HAL inter-compatible with various device drivers
//! from the community, including the [`embedded-hal`].
//!
//! ## Examples
//!
//! ### Read Data from a BMP180 Sensor
//!
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::i2c::master::{Config, I2c};
//!
//! // Create a new peripheral object with the described wiring
//! // and standard I2C clock speed.
//! let mut i2c = I2c::new(
//!     peripherals.I2C0,
//!     Config::default(),
//! )
//! .unwrap()
//! .with_sda(peripherals.GPIO1)
//! .with_scl(peripherals.GPIO2);
//!
//! loop {
//!     let mut data = [0u8; 22];
//!     i2c.write_read(0x77, &[0xaa], &mut data).ok();
//! }
//! # }
//! ```
//! 
//! [`embedded-hal`]:embedded_hal

use core::marker::PhantomData;
#[cfg(not(esp32))]
use core::{
    pin::Pin,
    task::{Context, Poll},
};

#[cfg(any(doc, feature = "unstable"))]
use embassy_embedded_hal::SetConfig;
use embedded_hal::i2c::Operation as EhalOperation;
use enumset::{EnumSet, EnumSetType};
use fugit::HertzU32;

use crate::{
    asynch::AtomicWaker,
    clock::Clocks,
    gpio::{interconnect::PeripheralOutput, InputSignal, OutputSignal, Pull},
    interrupt::{InterruptConfigurable, InterruptHandler},
    pac::i2c0::{RegisterBlock, COMD},
    peripheral::{Peripheral, PeripheralRef},
    peripherals::Interrupt,
    private,
    system::{PeripheralClockControl, PeripheralGuard},
    Async,
    Blocking,
    DriverMode,
};

cfg_if::cfg_if! {
    if #[cfg(esp32s2)] {
        const I2C_LL_INTR_MASK: u32 = 0x1ffff;
    } else {
        const I2C_LL_INTR_MASK: u32 = 0x3ffff;
    }
}

// Chunk writes/reads by this size
#[cfg(any(esp32, esp32s2))]
const I2C_CHUNK_SIZE: usize = 32;

#[cfg(not(any(esp32, esp32s2)))]
const I2C_CHUNK_SIZE: usize = 254;

// on ESP32 there is a chance to get trapped in `wait_for_completion` forever
const MAX_ITERATIONS: u32 = 1_000_000;

/// Representation of I2C address.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum I2cAddress {
    /// 7-bit address mode type.
    ///
    /// Note that 7-bit addresses defined by drivers should be specified in
    /// **right-aligned** form, e.g. in the range `0x00..=0x7F`.
    ///
    /// For example, a device that has the seven bit address of `0b011_0010`,
    /// and therefore is addressed on the wire using:
    ///
    /// * `0b0110010_0` or `0x64` for *writes*
    /// * `0b0110010_1` or `0x65` for *reads*
    SevenBit(u8),
}

impl From<u8> for I2cAddress {
    fn from(value: u8) -> Self {
        I2cAddress::SevenBit(value)
    }
}

/// I2C SCL timeout period.
///
/// When the level of SCL remains unchanged for more than `timeout` bus
/// clock cycles, the bus goes to idle state.
///
/// Default value is `BusCycles(10)`.
#[doc = ""]
#[cfg_attr(
    not(esp32),
    doc = "Note that the effective timeout may be longer than the value configured here."
)]
#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash, strum::Display)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
// TODO: when supporting interrupts, document that SCL = high also triggers an
// interrupt.
pub enum BusTimeout {
    /// Use the maximum timeout value.
    Maximum,

    /// Disable timeout control.
    #[cfg(not(any(esp32, esp32s2)))]
    Disabled,

    /// Timeout in bus clock cycles.
    BusCycles(u32),
}

impl BusTimeout {
    fn cycles(&self) -> u32 {
        match self {
            #[cfg(esp32)]
            BusTimeout::Maximum => 0xF_FFFF,

            #[cfg(esp32s2)]
            BusTimeout::Maximum => 0xFF_FFFF,

            #[cfg(not(any(esp32, esp32s2)))]
            BusTimeout::Maximum => 0x1F,

            #[cfg(not(any(esp32, esp32s2)))]
            BusTimeout::Disabled => 1,

            BusTimeout::BusCycles(cycles) => *cycles,
        }
    }

    #[cfg(not(esp32))]
    fn is_set(&self) -> bool {
        matches!(self, BusTimeout::BusCycles(_) | BusTimeout::Maximum)
    }
}

/// I2C-specific transmission errors
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// The transmission exceeded the FIFO size.
    FifoExceeded,
    /// The acknowledgment check failed.
    AcknowledgeCheckFailed(AcknowledgeCheckFailedReason),
    /// A timeout occurred during transmission.
    Timeout,
    /// The arbitration for the bus was lost.
    ArbitrationLost,
    /// The execution of the I2C command was incomplete.
    ExecutionIncomplete,
    /// The number of commands issued exceeded the limit.
    CommandNumberExceeded,
    /// Zero length read or write operation.
    ZeroLengthInvalid,
}

/// I2C no acknowledge error reason.
///
/// Consider this as a hint and make sure to always handle all cases.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum AcknowledgeCheckFailedReason {
    /// The device did not acknowledge its address. The device may be missing.
    Address,

    /// The device did not acknowledge the data. It may not be ready to process
    /// requests at the moment.
    Data,

    /// Either the device did not acknowledge its address or the data, but it is
    /// unknown which.
    Unknown,
}

impl core::fmt::Display for AcknowledgeCheckFailedReason {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            AcknowledgeCheckFailedReason::Address => write!(f, "Address"),
            AcknowledgeCheckFailedReason::Data => write!(f, "Data"),
            AcknowledgeCheckFailedReason::Unknown => write!(f, "Unknown"),
        }
    }
}

impl From<&AcknowledgeCheckFailedReason> for embedded_hal::i2c::NoAcknowledgeSource {
    fn from(value: &AcknowledgeCheckFailedReason) -> Self {
        match value {
            AcknowledgeCheckFailedReason::Address => {
                embedded_hal::i2c::NoAcknowledgeSource::Address
            }
            AcknowledgeCheckFailedReason::Data => embedded_hal::i2c::NoAcknowledgeSource::Data,
            AcknowledgeCheckFailedReason::Unknown => {
                embedded_hal::i2c::NoAcknowledgeSource::Unknown
            }
        }
    }
}

impl core::error::Error for Error {}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Error::FifoExceeded => write!(f, "The transmission exceeded the FIFO size"),
            Error::AcknowledgeCheckFailed(reason) => {
                write!(f, "The acknowledgment check failed. Reason: {}", reason)
            }
            Error::Timeout => write!(f, "A timeout occurred during transmission"),
            Error::ArbitrationLost => write!(f, "The arbitration for the bus was lost"),
            Error::ExecutionIncomplete => {
                write!(f, "The execution of the I2C command was incomplete")
            }
            Error::CommandNumberExceeded => {
                write!(f, "The number of commands issued exceeded the limit")
            }
            Error::ZeroLengthInvalid => write!(f, "Zero length read or write operation"),
        }
    }
}

/// I2C-specific configuration errors
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum ConfigError {
    /// Provided bus frequency is invalid for the current configuration.
    FrequencyInvalid,
    /// Provided timeout is invalid for the current configuration.
    TimeoutInvalid,
}

impl core::error::Error for ConfigError {}

impl core::fmt::Display for ConfigError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            ConfigError::FrequencyInvalid => write!(
                f,
                "Provided bus frequency is invalid for the current configuration"
            ),
            ConfigError::TimeoutInvalid => write!(
                f,
                "Provided timeout is invalid for the current configuration"
            ),
        }
    }
}

// This enum is used to keep track of the last/next operation that was/will be
// performed in an embedded-hal(-async) I2c::transaction. It is used to
// determine whether a START condition should be issued at the start of the
// current operation and whether a read needs an ack or a nack for the final
// byte.
#[derive(PartialEq)]
enum OpKind {
    Write,
    Read,
}

/// I2C operation.
///
/// Several operations can be combined as part of a transaction.
#[derive(Debug, PartialEq, Eq, Hash, strum::Display)]
pub enum Operation<'a> {
    /// Write data from the provided buffer.
    Write(&'a [u8]),

    /// Read data into the provided buffer.
    Read(&'a mut [u8]),
}

impl<'a, 'b> From<&'a mut embedded_hal::i2c::Operation<'b>> for Operation<'a> {
    fn from(value: &'a mut embedded_hal::i2c::Operation<'b>) -> Self {
        match value {
            embedded_hal::i2c::Operation::Write(buffer) => Operation::Write(buffer),
            embedded_hal::i2c::Operation::Read(buffer) => Operation::Read(buffer),
        }
    }
}

impl<'a, 'b> From<&'a mut Operation<'b>> for Operation<'a> {
    fn from(value: &'a mut Operation<'b>) -> Self {
        match value {
            Operation::Write(buffer) => Operation::Write(buffer),
            Operation::Read(buffer) => Operation::Read(buffer),
        }
    }
}

impl Operation<'_> {
    fn is_write(&self) -> bool {
        matches!(self, Operation::Write(_))
    }

    fn kind(&self) -> OpKind {
        match self {
            Operation::Write(_) => OpKind::Write,
            Operation::Read(_) => OpKind::Read,
        }
    }

    fn is_empty(&self) -> bool {
        match self {
            Operation::Write(buffer) => buffer.is_empty(),
            Operation::Read(buffer) => buffer.is_empty(),
        }
    }
}

impl embedded_hal::i2c::Error for Error {
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        use embedded_hal::i2c::ErrorKind;

        match self {
            Self::FifoExceeded => ErrorKind::Overrun,
            Self::ArbitrationLost => ErrorKind::ArbitrationLoss,
            Self::AcknowledgeCheckFailed(reason) => ErrorKind::NoAcknowledge(reason.into()),
            _ => ErrorKind::Other,
        }
    }
}

/// A generic I2C Command
#[cfg_attr(feature = "debug", derive(Debug))]
enum Command {
    Start,
    Stop,
    End,
    Write {
        /// This bit is to set an expected ACK value for the transmitter.
        ack_exp: Ack,
        /// Enables checking the ACK value received against the ack_exp value.
        ack_check_en: bool,
        /// Length of data (in bytes) to be written. The maximum length is 255,
        /// while the minimum is 1.
        length: u8,
    },
    Read {
        /// Indicates whether the receiver will send an ACK after this byte has
        /// been received.
        ack_value: Ack,
        /// Length of data (in bytes) to be read. The maximum length is 255,
        /// while the minimum is 1.
        length: u8,
    },
}

enum OperationType {
    Write = 0,
    Read  = 1,
}

#[derive(Eq, PartialEq, Copy, Clone)]
#[cfg_attr(feature = "debug", derive(Debug))]
enum Ack {
    Ack  = 0,
    Nack = 1,
}

impl From<u32> for Ack {
    fn from(ack: u32) -> Self {
        match ack {
            0 => Ack::Ack,
            1 => Ack::Nack,
            _ => unreachable!(),
        }
    }
}

impl From<Ack> for u32 {
    fn from(ack: Ack) -> u32 {
        ack as u32
    }
}

/// I2C driver configuration
#[derive(Debug, Clone, Copy, PartialEq, Eq, procmacros::BuilderLite)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Config {
    /// The I2C clock frequency.
    pub frequency: HertzU32,

    /// I2C SCL timeout period.
    pub timeout: BusTimeout,
}

impl core::hash::Hash for Config {
    fn hash<H: core::hash::Hasher>(&self, state: &mut H) {
        self.frequency.to_Hz().hash(state); // `HertzU32` doesn't implement `Hash`
        self.timeout.hash(state);
    }
}

impl Default for Config {
    fn default() -> Self {
        use fugit::RateExtU32;
        Config {
            frequency: 100.kHz(),
            timeout: BusTimeout::BusCycles(10),
        }
    }
}

/// I2C driver
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct I2c<'d, Dm: DriverMode> {
    i2c: PeripheralRef<'d, AnyI2c>,
    phantom: PhantomData<Dm>,
    config: Config,
    guard: PeripheralGuard,
}

#[cfg(any(doc, feature = "unstable"))]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
impl<Dm: DriverMode> SetConfig for I2c<'_, Dm> {
    type Config = Config;
    type ConfigError = ConfigError;

    fn set_config(&mut self, config: &Self::Config) -> Result<(), Self::ConfigError> {
        self.apply_config(config)
    }
}

impl<Dm: DriverMode> embedded_hal::i2c::ErrorType for I2c<'_, Dm> {
    type Error = Error;
}

impl<Dm: DriverMode> embedded_hal::i2c::I2c for I2c<'_, Dm> {
    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [embedded_hal::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        self.transaction_impl(
            I2cAddress::SevenBit(address),
            operations.iter_mut().map(Operation::from),
        )
        .inspect_err(|_| self.internal_recover())
    }
}

impl<'d, Dm: DriverMode> I2c<'d, Dm> {
    fn driver(&self) -> Driver<'_> {
        Driver {
            info: self.i2c.info(),
            state: self.i2c.state(),
        }
    }

    fn internal_recover(&self) {
        PeripheralClockControl::disable(self.driver().info.peripheral);
        PeripheralClockControl::enable(self.driver().info.peripheral);
        PeripheralClockControl::reset(self.driver().info.peripheral);

        // We know the configuration is valid, we can ignore the result.
        _ = self.driver().setup(&self.config);
    }

    /// Applies a new configuration.
    pub fn apply_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        self.driver().setup(config)?;
        self.config = *config;
        Ok(())
    }

    fn transaction_impl<'a>(
        &mut self,
        address: I2cAddress,
        operations: impl Iterator<Item = Operation<'a>>,
    ) -> Result<(), Error> {
        let mut last_op: Option<OpKind> = None;
        // filter out 0 length read operations
        let mut op_iter = operations
            .filter(|op| op.is_write() || !op.is_empty())
            .peekable();

        while let Some(op) = op_iter.next() {
            let next_op = op_iter.peek().map(|v| v.kind());
            let kind = op.kind();
            match op {
                Operation::Write(buffer) => {
                    // execute a write operation:
                    // - issue START/RSTART if op is different from previous
                    // - issue STOP if op is the last one
                    self.driver().write_blocking(
                        address,
                        buffer,
                        !matches!(last_op, Some(OpKind::Write)),
                        next_op.is_none(),
                    )?;
                }
                Operation::Read(buffer) => {
                    // execute a read operation:
                    // - issue START/RSTART if op is different from previous
                    // - issue STOP if op is the last one
                    // - will_continue is true if there is another read operation next
                    self.driver().read_blocking(
                        address,
                        buffer,
                        !matches!(last_op, Some(OpKind::Read)),
                        next_op.is_none(),
                        matches!(next_op, Some(OpKind::Read)),
                    )?;
                }
            }

            last_op = Some(kind);
        }

        Ok(())
    }

    /// Connect a pin to the I2C SDA signal.
    pub fn with_sda(self, sda: impl Peripheral<P = impl PeripheralOutput> + 'd) -> Self {
        let info = self.driver().info;
        let input = info.sda_input;
        let output = info.sda_output;
        self.with_pin(sda, input, output)
    }

    /// Connect a pin to the I2C SCL signal.
    pub fn with_scl(self, scl: impl Peripheral<P = impl PeripheralOutput> + 'd) -> Self {
        let info = self.driver().info;
        let input = info.scl_input;
        let output = info.scl_output;
        self.with_pin(scl, input, output)
    }

    fn with_pin(
        self,
        pin: impl Peripheral<P = impl PeripheralOutput> + 'd,
        input: InputSignal,
        output: OutputSignal,
    ) -> Self {
        crate::into_mapped_ref!(pin);
        // avoid the pin going low during configuration
        pin.set_output_high(true);

        pin.set_to_open_drain_output();
        pin.enable_input(true);
        pin.pull_direction(Pull::Up);

        input.connect_to(&mut pin);
        output.connect_to(&mut pin);

        self
    }
}

impl<'d> I2c<'d, Blocking> {
    /// Create a new I2C instance
    /// This will enable the peripheral but the peripheral won't get
    /// automatically disabled when this gets dropped.
    pub fn new(
        i2c: impl Peripheral<P = impl Instance> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        crate::into_mapped_ref!(i2c);

        let guard = PeripheralGuard::new(i2c.info().peripheral);

        let i2c = I2c {
            i2c,
            phantom: PhantomData,
            config,
            guard,
        };

        i2c.driver().setup(&i2c.config)?;

        Ok(i2c)
    }

    /// Listen for the given interrupts
    #[instability::unstable]
    pub fn listen(&mut self, interrupts: impl Into<EnumSet<Event>>) {
        self.i2c.info().enable_listen(interrupts.into(), true)
    }

    /// Unlisten the given interrupts
    #[instability::unstable]
    pub fn unlisten(&mut self, interrupts: impl Into<EnumSet<Event>>) {
        self.i2c.info().enable_listen(interrupts.into(), false)
    }

    /// Gets asserted interrupts
    #[instability::unstable]
    pub fn interrupts(&mut self) -> EnumSet<Event> {
        self.i2c.info().interrupts()
    }

    /// Resets asserted interrupts
    #[instability::unstable]
    pub fn clear_interrupts(&mut self, interrupts: EnumSet<Event>) {
        self.i2c.info().clear_interrupts(interrupts)
    }

    /// Configures the I2C peripheral to operate in asynchronous mode.
    pub fn into_async(mut self) -> I2c<'d, Async> {
        self.set_interrupt_handler(self.driver().info.async_handler);

        I2c {
            i2c: self.i2c,
            phantom: PhantomData,
            config: self.config,
            guard: self.guard,
        }
    }

    /// Writes bytes to slave with address `address`
    pub fn write<A: Into<I2cAddress>>(&mut self, address: A, buffer: &[u8]) -> Result<(), Error> {
        self.driver()
            .write_blocking(address.into(), buffer, true, true)
            .inspect_err(|_| self.internal_recover())
    }

    /// Reads enough bytes from slave with `address` to fill `buffer`
    pub fn read<A: Into<I2cAddress>>(
        &mut self,
        address: A,
        buffer: &mut [u8],
    ) -> Result<(), Error> {
        self.driver()
            .read_blocking(address.into(), buffer, true, true, false)
            .inspect_err(|_| self.internal_recover())
    }

    /// Writes bytes to slave with address `address` and then reads enough bytes
    /// to fill `buffer` *in a single transaction*
    pub fn write_read<A: Into<I2cAddress>>(
        &mut self,
        address: A,
        write_buffer: &[u8],
        read_buffer: &mut [u8],
    ) -> Result<(), Error> {
        let address = address.into();

        self.driver()
            .write_blocking(address, write_buffer, true, read_buffer.is_empty())
            .inspect_err(|_| self.internal_recover())?;

        self.driver()
            .read_blocking(address, read_buffer, true, true, false)
            .inspect_err(|_| self.internal_recover())?;

        Ok(())
    }

    /// Execute the provided operations on the I2C bus.
    ///
    /// Transaction contract:
    /// - Before executing the first operation an ST is sent automatically. This
    ///   is followed by SAD+R/W as appropriate.
    /// - Data from adjacent operations of the same type are sent after each
    ///   other without an SP or SR.
    /// - Between adjacent operations of a different type an SR and SAD+R/W is
    ///   sent.
    /// - After executing the last operation an SP is sent automatically.
    /// - If the last operation is a `Read` the master does not send an
    ///   acknowledge for the last byte.
    ///
    /// - `ST` = start condition
    /// - `SAD+R/W` = slave address followed by bit 1 to indicate reading or 0
    ///   to indicate writing
    /// - `SR` = repeated start condition
    /// - `SP` = stop condition
    pub fn transaction<'a, A: Into<I2cAddress>>(
        &mut self,
        address: A,
        operations: impl IntoIterator<Item = &'a mut Operation<'a>>,
    ) -> Result<(), Error> {
        self.transaction_impl(address.into(), operations.into_iter().map(Operation::from))
            .inspect_err(|_| self.internal_recover())
    }
}

impl private::Sealed for I2c<'_, Blocking> {}

impl InterruptConfigurable for I2c<'_, Blocking> {
    fn set_interrupt_handler(&mut self, handler: crate::interrupt::InterruptHandler) {
        self.i2c.info().set_interrupt_handler(handler);
    }
}

#[cfg_attr(esp32, allow(dead_code))]
#[derive(Debug, EnumSetType)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
#[instability::unstable]
pub enum Event {
    /// Triggered when op_code of the master indicates an END command and an END
    /// condition is detected.
    EndDetect,

    /// Triggered when the I2C controller detects a STOP bit.
    TxComplete,

    /// Triggered when the TX FIFO watermark check is enabled and the TX fifo
    /// falls below the configured watermark.
    #[cfg(not(any(esp32, esp32s2)))]
    TxFifoWatermark,
}

#[cfg(not(esp32))]
#[must_use = "futures do nothing unless you `.await` or poll them"]
struct I2cFuture<'a> {
    event: Event,
    info: &'a Info,
    state: &'a State,
}

#[cfg(not(esp32))]
impl<'a> I2cFuture<'a> {
    pub fn new(event: Event, info: &'a Info, state: &'a State) -> Self {
        info.register_block().int_ena().modify(|_, w| {
            let w = match event {
                Event::EndDetect => w.end_detect().set_bit(),
                Event::TxComplete => w.trans_complete().set_bit(),
                #[cfg(not(any(esp32, esp32s2)))]
                Event::TxFifoWatermark => w.txfifo_wm().set_bit(),
            };

            w.arbitration_lost().set_bit();
            w.time_out().set_bit();
            w.nack().set_bit();

            w
        });

        Self { event, state, info }
    }

    fn event_bit_is_clear(&self) -> bool {
        let r = self.info.register_block().int_ena().read();

        match self.event {
            Event::EndDetect => r.end_detect().bit_is_clear(),
            Event::TxComplete => r.trans_complete().bit_is_clear(),
            #[cfg(not(any(esp32, esp32s2)))]
            Event::TxFifoWatermark => r.txfifo_wm().bit_is_clear(),
        }
    }

    fn check_error(&self) -> Result<(), Error> {
        let r = self.info.register_block().int_raw().read();

        if r.arbitration_lost().bit_is_set() {
            return Err(Error::ArbitrationLost);
        }

        if r.time_out().bit_is_set() {
            return Err(Error::Timeout);
        }

        if r.nack().bit_is_set() {
            return Err(Error::AcknowledgeCheckFailed(estimate_ack_failed_reason(
                self.info.register_block(),
            )));
        }

        #[cfg(not(esp32))]
        if r.trans_complete().bit_is_set()
            && self
                .info
                .register_block()
                .sr()
                .read()
                .resp_rec()
                .bit_is_clear()
        {
            return Err(Error::AcknowledgeCheckFailed(
                AcknowledgeCheckFailedReason::Data,
            ));
        }

        Ok(())
    }
}

#[cfg(not(esp32))]
impl core::future::Future for I2cFuture<'_> {
    type Output = Result<(), Error>;

    fn poll(self: Pin<&mut Self>, ctx: &mut Context<'_>) -> Poll<Self::Output> {
        self.state.waker.register(ctx.waker());

        let error = self.check_error();

        if error.is_err() {
            return Poll::Ready(error);
        }

        if self.event_bit_is_clear() {
            Poll::Ready(Ok(()))
        } else {
            Poll::Pending
        }
    }
}

impl<'d> I2c<'d, Async> {
    /// Configure the I2C peripheral to operate in blocking mode.
    pub fn into_blocking(self) -> I2c<'d, Blocking> {
        self.i2c.info().disable_interrupts();

        I2c {
            i2c: self.i2c,
            phantom: PhantomData,
            config: self.config,
            guard: self.guard,
        }
    }

    /// Writes bytes to slave with address `address`
    pub async fn write<A: Into<I2cAddress>>(
        &mut self,
        address: A,
        buffer: &[u8],
    ) -> Result<(), Error> {
        self.driver()
            .write(address.into(), buffer, true, true)
            .await
            .inspect_err(|_| self.internal_recover())
    }

    /// Reads enough bytes from slave with `address` to fill `buffer`
    pub async fn read<A: Into<I2cAddress>>(
        &mut self,
        address: A,
        buffer: &mut [u8],
    ) -> Result<(), Error> {
        self.driver()
            .read(address.into(), buffer, true, true, false)
            .await
            .inspect_err(|_| self.internal_recover())
    }

    /// Writes bytes to slave with address `address` and then reads enough
    /// bytes to fill `buffer` *in a single transaction*
    pub async fn write_read<A: Into<I2cAddress>>(
        &mut self,
        address: A,
        write_buffer: &[u8],
        read_buffer: &mut [u8],
    ) -> Result<(), Error> {
        let address = address.into();

        self.driver()
            .write(address, write_buffer, true, read_buffer.is_empty())
            .await
            .inspect_err(|_| self.internal_recover())?;

        self.driver()
            .read(address, read_buffer, true, true, false)
            .await
            .inspect_err(|_| self.internal_recover())?;

        Ok(())
    }

    /// Execute the provided operations on the I2C bus as a single
    /// transaction.
    ///
    /// Transaction contract:
    /// - Before executing the first operation an ST is sent automatically. This
    ///   is followed by SAD+R/W as appropriate.
    /// - Data from adjacent operations of the same type are sent after each
    ///   other without an SP or SR.
    /// - Between adjacent operations of a different type an SR and SAD+R/W is
    ///   sent.
    /// - After executing the last operation an SP is sent automatically.
    /// - If the last operation is a `Read` the master does not send an
    ///   acknowledge for the last byte.
    ///
    /// - `ST` = start condition
    /// - `SAD+R/W` = slave address followed by bit 1 to indicate reading or 0
    ///   to indicate writing
    /// - `SR` = repeated start condition
    /// - `SP` = stop condition
    pub async fn transaction<'a, A: Into<I2cAddress>>(
        &mut self,
        address: A,
        operations: impl IntoIterator<Item = &'a mut Operation<'a>>,
    ) -> Result<(), Error> {
        self.transaction_impl_async(address.into(), operations.into_iter().map(Operation::from))
            .await
            .inspect_err(|_| self.internal_recover())
    }

    async fn transaction_impl_async<'a>(
        &mut self,
        address: I2cAddress,
        operations: impl Iterator<Item = Operation<'a>>,
    ) -> Result<(), Error> {
        let mut last_op: Option<OpKind> = None;
        // filter out 0 length read operations
        let mut op_iter = operations
            .filter(|op| op.is_write() || !op.is_empty())
            .peekable();

        while let Some(op) = op_iter.next() {
            let next_op = op_iter.peek().map(|v| v.kind());
            let kind = op.kind();
            match op {
                Operation::Write(buffer) => {
                    // execute a write operation:
                    // - issue START/RSTART if op is different from previous
                    // - issue STOP if op is the last one
                    self.driver()
                        .write(
                            address,
                            buffer,
                            !matches!(last_op, Some(OpKind::Write)),
                            next_op.is_none(),
                        )
                        .await?;
                }
                Operation::Read(buffer) => {
                    // execute a read operation:
                    // - issue START/RSTART if op is different from previous
                    // - issue STOP if op is the last one
                    // - will_continue is true if there is another read operation next
                    self.driver()
                        .read(
                            address,
                            buffer,
                            !matches!(last_op, Some(OpKind::Read)),
                            next_op.is_none(),
                            matches!(next_op, Some(OpKind::Read)),
                        )
                        .await?;
                }
            }

            last_op = Some(kind);
        }

        Ok(())
    }
}

impl embedded_hal_async::i2c::I2c for I2c<'_, Async> {
    async fn transaction(
        &mut self,
        address: u8,
        operations: &mut [EhalOperation<'_>],
    ) -> Result<(), Self::Error> {
        self.transaction_impl_async(address.into(), operations.iter_mut().map(Operation::from))
            .await
            .inspect_err(|_| self.internal_recover())
    }
}

fn async_handler(info: &Info, state: &State) {
    let regs = info.register_block();
    regs.int_ena().modify(|_, w| {
        w.end_detect().clear_bit();
        w.trans_complete().clear_bit();
        w.arbitration_lost().clear_bit();
        w.time_out().clear_bit();

        #[cfg(not(any(esp32, esp32s2)))]
        w.txfifo_wm().clear_bit();

        w.nack().clear_bit()
    });

    state.waker.wake();
}

/// Sets the filter with a supplied threshold in clock cycles for which a
/// pulse must be present to pass the filter
fn set_filter(
    register_block: &RegisterBlock,
    sda_threshold: Option<u8>,
    scl_threshold: Option<u8>,
) {
    cfg_if::cfg_if! {
        if #[cfg(any(esp32, esp32s2))] {
            register_block.sda_filter_cfg().modify(|_, w| {
                if let Some(threshold) = sda_threshold {
                    unsafe { w.sda_filter_thres().bits(threshold) };
                }
                w.sda_filter_en().bit(sda_threshold.is_some())
            });
            register_block.scl_filter_cfg().modify(|_, w| {
                if let Some(threshold) = scl_threshold {
                    unsafe { w.scl_filter_thres().bits(threshold) };
                }
                w.scl_filter_en().bit(scl_threshold.is_some())
            });
        } else {
            register_block.filter_cfg().modify(|_, w| {
                if let Some(threshold) = sda_threshold {
                    unsafe { w.sda_filter_thres().bits(threshold) };
                }
                if let Some(threshold) = scl_threshold {
                    unsafe { w.scl_filter_thres().bits(threshold) };
                }
                w.sda_filter_en().bit(sda_threshold.is_some());
                w.scl_filter_en().bit(scl_threshold.is_some())
            });
        }
    }
}

#[allow(clippy::too_many_arguments, unused)]
/// Configures the clock and timing parameters for the I2C peripheral.
fn configure_clock(
    register_block: &RegisterBlock,
    sclk_div: u32,
    scl_low_period: u32,
    scl_high_period: u32,
    scl_wait_high_period: u32,
    sda_hold_time: u32,
    sda_sample_time: u32,
    scl_rstart_setup_time: u32,
    scl_stop_setup_time: u32,
    scl_start_hold_time: u32,
    scl_stop_hold_time: u32,
    timeout: BusTimeout,
) -> Result<(), ConfigError> {
    unsafe {
        // divider
        #[cfg(any(esp32c2, esp32c3, esp32c6, esp32h2, esp32s3))]
        register_block.clk_conf().modify(|_, w| {
            w.sclk_sel().clear_bit();
            w.sclk_div_num().bits((sclk_div - 1) as u8)
        });

        // scl period
        register_block
            .scl_low_period()
            .write(|w| w.scl_low_period().bits(scl_low_period as u16));

        #[cfg(not(esp32))]
        let scl_wait_high_period = scl_wait_high_period
            .try_into()
            .map_err(|_| ConfigError::FrequencyInvalid)?;

        register_block.scl_high_period().write(|w| {
            #[cfg(not(esp32))] // ESP32 does not have a wait_high field
            w.scl_wait_high_period().bits(scl_wait_high_period);
            w.scl_high_period().bits(scl_high_period as u16)
        });

        // sda sample
        register_block
            .sda_hold()
            .write(|w| w.time().bits(sda_hold_time as u16));
        register_block
            .sda_sample()
            .write(|w| w.time().bits(sda_sample_time as u16));

        // setup
        register_block
            .scl_rstart_setup()
            .write(|w| w.time().bits(scl_rstart_setup_time as u16));
        register_block
            .scl_stop_setup()
            .write(|w| w.time().bits(scl_stop_setup_time as u16));

        // hold
        register_block
            .scl_start_hold()
            .write(|w| w.time().bits(scl_start_hold_time as u16));
        register_block
            .scl_stop_hold()
            .write(|w| w.time().bits(scl_stop_hold_time as u16));

        // The ESP32 variant does not have an enable flag for the
        // timeout mechanism
        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
                register_block
                    .to()
                    .write(|w| w.time_out().bits(timeout.cycles()));
            } else {
                register_block
                    .to()
                    .write(|w| w.time_out_en().bit(timeout.is_set())
                    .time_out_value()
                    .bits(timeout.cycles() as _)
                );
            }
        }
    }
    Ok(())
}

/// Peripheral data describing a particular I2C instance.
#[doc(hidden)]
#[derive(Debug)]
#[non_exhaustive]
pub struct Info {
    /// Pointer to the register block for this I2C instance.
    ///
    /// Use [Self::register_block] to access the register block.
    pub register_block: *const RegisterBlock,

    /// System peripheral marker.
    pub peripheral: crate::system::Peripheral,

    /// Interrupt handler for the asynchronous operations of this I2C instance.
    pub async_handler: InterruptHandler,

    /// Interrupt for this I2C instance.
    pub interrupt: Interrupt,

    /// SCL output signal.
    pub scl_output: OutputSignal,

    /// SCL input signal.
    pub scl_input: InputSignal,

    /// SDA output signal.
    pub sda_output: OutputSignal,

    /// SDA input signal.
    pub sda_input: InputSignal,
}

impl Info {
    /// Returns the register block for this I2C instance.
    pub fn register_block(&self) -> &RegisterBlock {
        unsafe { &*self.register_block }
    }

    /// Listen for the given interrupts
    fn enable_listen(&self, interrupts: EnumSet<Event>, enable: bool) {
        let reg_block = self.register_block();

        reg_block.int_ena().modify(|_, w| {
            for interrupt in interrupts {
                match interrupt {
                    Event::EndDetect => w.end_detect().bit(enable),
                    Event::TxComplete => w.trans_complete().bit(enable),
                    #[cfg(not(any(esp32, esp32s2)))]
                    Event::TxFifoWatermark => w.txfifo_wm().bit(enable),
                };
            }
            w
        });
    }

    fn interrupts(&self) -> EnumSet<Event> {
        let mut res = EnumSet::new();
        let reg_block = self.register_block();

        let ints = reg_block.int_raw().read();

        if ints.end_detect().bit_is_set() {
            res.insert(Event::EndDetect);
        }
        if ints.trans_complete().bit_is_set() {
            res.insert(Event::TxComplete);
        }
        #[cfg(not(any(esp32, esp32s2)))]
        if ints.txfifo_wm().bit_is_set() {
            res.insert(Event::TxFifoWatermark);
        }

        res
    }

    fn clear_interrupts(&self, interrupts: EnumSet<Event>) {
        let reg_block = self.register_block();

        reg_block.int_clr().write(|w| {
            for interrupt in interrupts {
                match interrupt {
                    Event::EndDetect => w.end_detect().clear_bit_by_one(),
                    Event::TxComplete => w.trans_complete().clear_bit_by_one(),
                    #[cfg(not(any(esp32, esp32s2)))]
                    Event::TxFifoWatermark => w.txfifo_wm().clear_bit_by_one(),
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
}

impl PartialEq for Info {
    fn eq(&self, other: &Self) -> bool {
        self.register_block == other.register_block
    }
}

unsafe impl Sync for Info {}

#[allow(dead_code)] // Some versions don't need `state`
struct Driver<'a> {
    info: &'a Info,
    state: &'a State,
}

impl Driver<'_> {
    fn register_block(&self) -> &RegisterBlock {
        self.info.register_block()
    }

    /// Configures the I2C peripheral with the specified frequency, clocks, and
    /// optional timeout.
    fn setup(&self, config: &Config) -> Result<(), ConfigError> {
        self.register_block().ctr().write(|w| {
            // Set I2C controller to master mode
            w.ms_mode().set_bit();
            // Use open drain output for SDA and SCL
            w.sda_force_out().set_bit();
            w.scl_force_out().set_bit();
            // Use Most Significant Bit first for sending and receiving data
            w.tx_lsb_first().clear_bit();
            w.rx_lsb_first().clear_bit();
            // Ensure that clock is enabled
            w.clk_en().set_bit()
        });

        #[cfg(esp32s2)]
        self.register_block()
            .ctr()
            .modify(|_, w| w.ref_always_on().set_bit());

        // Configure filter
        // FIXME if we ever change this we need to adapt `set_frequency` for ESP32
        set_filter(self.register_block(), Some(7), Some(7));

        // Configure frequency
        let clocks = Clocks::get();
        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
                let clock = clocks.i2c_clock.convert();
            } else if #[cfg(esp32s2)] {
                let clock = clocks.apb_clock.convert();
            } else {
                let clock = clocks.xtal_clock.convert();
            }
        }
        self.set_frequency(clock, config.frequency, config.timeout)?;

        self.update_config();

        // Reset entire peripheral (also resets fifo)
        self.reset();

        Ok(())
    }

    /// Resets the I2C controller (FIFO + FSM + command list)
    fn reset(&self) {
        // Reset the FSM
        // (the option to reset the FSM is not available
        // for the ESP32)
        #[cfg(not(esp32))]
        self.register_block()
            .ctr()
            .modify(|_, w| w.fsm_rst().set_bit());

        // Clear all I2C interrupts
        self.register_block()
            .int_clr()
            .write(|w| unsafe { w.bits(I2C_LL_INTR_MASK) });

        // Reset fifo
        self.reset_fifo();

        // Reset the command list
        self.reset_command_list();
    }

    /// Resets the I2C peripheral's command registers
    fn reset_command_list(&self) {
        // Confirm that all commands that were configured were actually executed
        for cmd in self.register_block().comd_iter() {
            cmd.reset();
        }
    }

    #[cfg(esp32)]
    /// Sets the frequency of the I2C interface by calculating and applying the
    /// associated timings - corresponds to i2c_ll_cal_bus_clk and
    /// i2c_ll_set_bus_timing in ESP-IDF
    fn set_frequency(
        &self,
        source_clk: HertzU32,
        bus_freq: HertzU32,
        timeout: BusTimeout,
    ) -> Result<(), ConfigError> {
        let source_clk = source_clk.raw();
        let bus_freq = bus_freq.raw();

        let half_cycle: u32 = source_clk / bus_freq / 2;
        let scl_low = half_cycle;
        let scl_high = half_cycle;
        let sda_hold = half_cycle / 2;
        let sda_sample = scl_high / 2;
        let setup = half_cycle;
        let hold = half_cycle;
        let timeout = BusTimeout::BusCycles(match timeout {
            BusTimeout::Maximum => 0xF_FFFF,
            BusTimeout::BusCycles(cycles) => check_timeout(cycles * 2 * half_cycle, 0xF_FFFF)?,
        });

        // SCL period. According to the TRM, we should always subtract 1 to SCL low
        // period
        let scl_low = scl_low - 1;
        // Still according to the TRM, if filter is not enbled, we have to subtract 7,
        // if SCL filter is enabled, we have to subtract:
        //   8 if SCL filter is between 0 and 2 (included)
        //   6 + SCL threshold if SCL filter is between 3 and 7 (included)
        // to SCL high period
        let mut scl_high = scl_high;
        // In the "worst" case, we will subtract 13, make sure the result will still be
        // correct

        // FIXME since we always set the filter threshold to 7 we don't need conditional
        // code here once that changes we need the conditional code here
        scl_high -= 7 + 6;

        // if (filter_cfg_en) {
        //     if (thres <= 2) {
        //         scl_high -= 8;
        //     } else {
        //         assert(hw->scl_filter_cfg.thres <= 7);
        //         scl_high -= thres + 6;
        //     }
        // } else {
        //    scl_high -= 7;
        //}

        let scl_high_period = scl_high;
        let scl_low_period = scl_low;
        // sda sample
        let sda_hold_time = sda_hold;
        let sda_sample_time = sda_sample;
        // setup
        let scl_rstart_setup_time = setup;
        let scl_stop_setup_time = setup;
        // hold
        let scl_start_hold_time = hold;
        let scl_stop_hold_time = hold;

        configure_clock(
            self.register_block(),
            0,
            scl_low_period,
            scl_high_period,
            0,
            sda_hold_time,
            sda_sample_time,
            scl_rstart_setup_time,
            scl_stop_setup_time,
            scl_start_hold_time,
            scl_stop_hold_time,
            timeout,
        )?;

        Ok(())
    }

    #[cfg(esp32s2)]
    /// Sets the frequency of the I2C interface by calculating and applying the
    /// associated timings - corresponds to i2c_ll_cal_bus_clk and
    /// i2c_ll_set_bus_timing in ESP-IDF
    fn set_frequency(
        &self,
        source_clk: HertzU32,
        bus_freq: HertzU32,
        timeout: BusTimeout,
    ) -> Result<(), ConfigError> {
        let source_clk = source_clk.raw();
        let bus_freq = bus_freq.raw();

        let half_cycle: u32 = source_clk / bus_freq / 2;
        // SCL
        let scl_low = half_cycle;
        // default, scl_wait_high < scl_high
        let scl_high = half_cycle / 2 + 2;
        let scl_wait_high = half_cycle - scl_high;
        let sda_hold = half_cycle / 2;
        // scl_wait_high < sda_sample <= scl_high
        let sda_sample = half_cycle / 2 - 1;
        let setup = half_cycle;
        let hold = half_cycle;

        // scl period
        let scl_low_period = scl_low - 1;
        let scl_high_period = scl_high;
        let scl_wait_high_period = scl_wait_high;
        // sda sample
        let sda_hold_time = sda_hold;
        let sda_sample_time = sda_sample;
        // setup
        let scl_rstart_setup_time = setup;
        let scl_stop_setup_time = setup;
        // hold
        let scl_start_hold_time = hold - 1;
        let scl_stop_hold_time = hold;

        let timeout = BusTimeout::BusCycles(match timeout {
            BusTimeout::Maximum => 0xFF_FFFF,
            BusTimeout::BusCycles(cycles) => check_timeout(cycles * 2 * half_cycle, 0xFF_FFFF)?,
        });

        configure_clock(
            self.register_block(),
            0,
            scl_low_period,
            scl_high_period,
            scl_wait_high_period,
            sda_hold_time,
            sda_sample_time,
            scl_rstart_setup_time,
            scl_stop_setup_time,
            scl_start_hold_time,
            scl_stop_hold_time,
            timeout,
        )?;

        Ok(())
    }

    #[cfg(any(esp32c2, esp32c3, esp32c6, esp32h2, esp32s3))]
    /// Sets the frequency of the I2C interface by calculating and applying the
    /// associated timings - corresponds to i2c_ll_cal_bus_clk and
    /// i2c_ll_set_bus_timing in ESP-IDF
    fn set_frequency(
        &self,
        source_clk: HertzU32,
        bus_freq: HertzU32,
        timeout: BusTimeout,
    ) -> Result<(), ConfigError> {
        let source_clk = source_clk.raw();
        let bus_freq = bus_freq.raw();

        let clkm_div: u32 = source_clk / (bus_freq * 1024) + 1;
        let sclk_freq: u32 = source_clk / clkm_div;
        let half_cycle: u32 = sclk_freq / bus_freq / 2;
        // SCL
        let scl_low = half_cycle;
        // default, scl_wait_high < scl_high
        // Make 80KHz as a boundary here, because when working at lower frequency, too
        // much scl_wait_high will faster the frequency according to some
        // hardware behaviors.
        let scl_wait_high = if bus_freq >= 80 * 1000 {
            half_cycle / 2 - 2
        } else {
            half_cycle / 4
        };
        let scl_high = half_cycle - scl_wait_high;
        let sda_hold = half_cycle / 4;
        let sda_sample = half_cycle / 2 + scl_wait_high;
        let setup = half_cycle;
        let hold = half_cycle;

        // According to the Technical Reference Manual, the following timings must be
        // subtracted by 1. However, according to the practical measurement and
        // some hardware behaviour, if wait_high_period and scl_high minus one.
        // The SCL frequency would be a little higher than expected. Therefore, the
        // solution here is not to minus scl_high as well as scl_wait high, and
        // the frequency will be absolutely accurate to all frequency
        // to some extent.
        let scl_low_period = scl_low - 1;
        let scl_high_period = scl_high;
        let scl_wait_high_period = scl_wait_high;
        // sda sample
        let sda_hold_time = sda_hold - 1;
        let sda_sample_time = sda_sample - 1;
        // setup
        let scl_rstart_setup_time = setup - 1;
        let scl_stop_setup_time = setup - 1;
        // hold
        let scl_start_hold_time = hold - 1;
        let scl_stop_hold_time = hold - 1;

        let timeout = match timeout {
            BusTimeout::Maximum => BusTimeout::BusCycles(0x1F),
            BusTimeout::Disabled => BusTimeout::Disabled,
            BusTimeout::BusCycles(cycles) => {
                let to_peri = (cycles * 2 * half_cycle).max(1);
                let log2 = to_peri.ilog2();
                // Round up so that we don't shorten timeouts.
                let raw = if to_peri != 1 << log2 { log2 + 1 } else { log2 };
                BusTimeout::BusCycles(check_timeout(raw, 0x1F)?)
            }
        };

        configure_clock(
            self.register_block(),
            clkm_div,
            scl_low_period,
            scl_high_period,
            scl_wait_high_period,
            sda_hold_time,
            sda_sample_time,
            scl_rstart_setup_time,
            scl_stop_setup_time,
            scl_start_hold_time,
            scl_stop_hold_time,
            timeout,
        )?;

        Ok(())
    }

    #[cfg(any(esp32, esp32s2))]
    async fn read_all_from_fifo(&self, buffer: &mut [u8]) -> Result<(), Error> {
        if buffer.len() > 32 {
            return Err(Error::FifoExceeded);
        }

        self.wait_for_completion(false).await?;

        for byte in buffer.iter_mut() {
            *byte = read_fifo(self.register_block());
        }

        Ok(())
    }

    #[cfg(not(any(esp32, esp32s2)))]
    async fn read_all_from_fifo(&self, buffer: &mut [u8]) -> Result<(), Error> {
        self.read_all_from_fifo_blocking(buffer)
    }

    /// Configures the I2C peripheral for a write operation.
    /// - `addr` is the address of the slave device.
    /// - `bytes` is the data two be sent.
    /// - `start` indicates whether the operation should start by a START
    ///   condition and sending the address.
    /// - `cmd_iterator` is an iterator over the command registers.
    fn setup_write<'a, I>(
        &self,
        addr: I2cAddress,
        bytes: &[u8],
        start: bool,
        cmd_iterator: &mut I,
    ) -> Result<(), Error>
    where
        I: Iterator<Item = &'a COMD>,
    {
        // if start is true we can only send 254 additional bytes with the address as
        // the first
        let max_len = if start { 254usize } else { 255usize };
        if bytes.len() > max_len {
            // we could support more by adding multiple write operations
            return Err(Error::FifoExceeded);
        }

        let write_len = if start { bytes.len() + 1 } else { bytes.len() };
        // don't issue write if there is no data to write
        if write_len > 0 {
            // WRITE command
            add_cmd(
                cmd_iterator,
                Command::Write {
                    ack_exp: Ack::Ack,
                    ack_check_en: true,
                    length: write_len as u8,
                },
            )?;
        }

        self.update_config();

        if start {
            // Load address and R/W bit into FIFO
            match addr {
                I2cAddress::SevenBit(addr) => {
                    write_fifo(
                        self.register_block(),
                        (addr << 1) | OperationType::Write as u8,
                    );
                }
            }
        }
        Ok(())
    }

    /// Configures the I2C peripheral for a read operation.
    /// - `addr` is the address of the slave device.
    /// - `buffer` is the buffer to store the read data.
    /// - `start` indicates whether the operation should start by a START
    ///   condition and sending the address.
    /// - `will_continue` indicates whether there is another read operation
    ///   following this one and we should not nack the last byte.
    /// - `cmd_iterator` is an iterator over the command registers.
    fn setup_read<'a, I>(
        &self,
        addr: I2cAddress,
        buffer: &mut [u8],
        start: bool,
        will_continue: bool,
        cmd_iterator: &mut I,
    ) -> Result<(), Error>
    where
        I: Iterator<Item = &'a COMD>,
    {
        if buffer.is_empty() {
            return Err(Error::ZeroLengthInvalid);
        }
        let (max_len, initial_len) = if will_continue {
            (255usize, buffer.len())
        } else {
            (254usize, buffer.len() - 1)
        };
        if buffer.len() > max_len {
            // we could support more by adding multiple read operations
            return Err(Error::FifoExceeded);
        }

        if start {
            // WRITE command
            add_cmd(
                cmd_iterator,
                Command::Write {
                    ack_exp: Ack::Ack,
                    ack_check_en: true,
                    length: 1,
                },
            )?;
        }

        if initial_len > 0 {
            // READ command
            add_cmd(
                cmd_iterator,
                Command::Read {
                    ack_value: Ack::Ack,
                    length: initial_len as u8,
                },
            )?;
        }

        if !will_continue {
            // this is the last read so we need to nack the last byte
            // READ w/o ACK
            add_cmd(
                cmd_iterator,
                Command::Read {
                    ack_value: Ack::Nack,
                    length: 1,
                },
            )?;
        }

        self.update_config();

        if start {
            // Load address and R/W bit into FIFO
            match addr {
                I2cAddress::SevenBit(addr) => {
                    write_fifo(
                        self.register_block(),
                        (addr << 1) | OperationType::Read as u8,
                    );
                }
            }
        }
        Ok(())
    }

    #[cfg(not(any(esp32, esp32s2)))]
    /// Reads all bytes from the RX FIFO.
    fn read_all_from_fifo_blocking(&self, buffer: &mut [u8]) -> Result<(), Error> {
        // Read bytes from FIFO
        // FIXME: Handle case where less data has been provided by the slave than
        // requested? Or is this prevented from a protocol perspective?
        for byte in buffer.iter_mut() {
            loop {
                self.check_errors()?;

                let reg = self.register_block().fifo_st().read();
                if reg.rxfifo_raddr().bits() != reg.rxfifo_waddr().bits() {
                    break;
                }
            }

            *byte = read_fifo(self.register_block());
        }

        Ok(())
    }

    #[cfg(any(esp32, esp32s2))]
    /// Reads all bytes from the RX FIFO.
    fn read_all_from_fifo_blocking(&self, buffer: &mut [u8]) -> Result<(), Error> {
        // on ESP32/ESP32-S2 we currently don't support I2C transactions larger than the
        // FIFO apparently it would be possible by using non-fifo mode
        // see https://github.com/espressif/arduino-esp32/blob/7e9afe8c5ed7b5bf29624a5cd6e07d431c027b97/cores/esp32/esp32-hal-i2c.c#L615

        if buffer.len() > 32 {
            return Err(Error::FifoExceeded);
        }

        // wait for completion - then we can just read the data from FIFO
        // once we change to non-fifo mode to support larger transfers that
        // won't work anymore
        self.wait_for_completion_blocking(false)?;

        // Read bytes from FIFO
        // FIXME: Handle case where less data has been provided by the slave than
        // requested? Or is this prevented from a protocol perspective?
        for byte in buffer.iter_mut() {
            *byte = read_fifo(self.register_block());
        }

        Ok(())
    }

    /// Clears all pending interrupts for the I2C peripheral.
    fn clear_all_interrupts(&self) {
        self.register_block()
            .int_clr()
            .write(|w| unsafe { w.bits(I2C_LL_INTR_MASK) });
    }

    #[cfg(any(esp32, esp32s2))]
    async fn write_remaining_tx_fifo(&self, start_index: usize, bytes: &[u8]) -> Result<(), Error> {
        if start_index >= bytes.len() {
            return Ok(());
        }

        for b in bytes {
            write_fifo(self.register_block(), *b);
            self.check_errors()?;
        }

        Ok(())
    }

    #[cfg(not(any(esp32, esp32s2)))]
    async fn write_remaining_tx_fifo(&self, start_index: usize, bytes: &[u8]) -> Result<(), Error> {
        let mut index = start_index;
        loop {
            self.check_errors()?;

            I2cFuture::new(Event::TxFifoWatermark, self.info, self.state).await?;

            self.register_block()
                .int_clr()
                .write(|w| w.txfifo_wm().clear_bit_by_one());

            I2cFuture::new(Event::TxFifoWatermark, self.info, self.state).await?;

            if index >= bytes.len() {
                break Ok(());
            }

            write_fifo(self.register_block(), bytes[index]);
            index += 1;
        }
    }

    #[cfg(not(esp32))]
    async fn wait_for_completion(&self, end_only: bool) -> Result<(), Error> {
        self.check_errors()?;

        if end_only {
            I2cFuture::new(Event::EndDetect, self.info, self.state).await?;
        } else {
            let res = embassy_futures::select::select(
                I2cFuture::new(Event::TxComplete, self.info, self.state),
                I2cFuture::new(Event::EndDetect, self.info, self.state),
            )
            .await;

            match res {
                embassy_futures::select::Either::First(res) => res?,
                embassy_futures::select::Either::Second(res) => res?,
            }
        }
        self.check_all_commands_done()?;

        Ok(())
    }

    #[cfg(esp32)]
    async fn wait_for_completion(&self, end_only: bool) -> Result<(), Error> {
        // for ESP32 we need a timeout here but wasting a timer seems unnecessary
        // given the short time we spend here

        let mut tout = MAX_ITERATIONS / 10; // adjust the timeout because we are yielding in the loop
        loop {
            let interrupts = self.register_block().int_raw().read();

            self.check_errors()?;

            // Handle completion cases
            // A full transmission was completed (either a STOP condition or END was
            // processed)
            if (!end_only && interrupts.trans_complete().bit_is_set())
                || interrupts.end_detect().bit_is_set()
            {
                break;
            }

            tout -= 1;
            if tout == 0 {
                return Err(Error::Timeout);
            }

            embassy_futures::yield_now().await;
        }
        self.check_all_commands_done()?;
        Ok(())
    }

    /// Waits for the completion of an I2C transaction.
    fn wait_for_completion_blocking(&self, end_only: bool) -> Result<(), Error> {
        let mut tout = MAX_ITERATIONS;
        loop {
            let interrupts = self.register_block().int_raw().read();

            self.check_errors()?;

            // Handle completion cases
            // A full transmission was completed (either a STOP condition or END was
            // processed)
            if (!end_only && interrupts.trans_complete().bit_is_set())
                || interrupts.end_detect().bit_is_set()
            {
                break;
            }

            tout -= 1;
            if tout == 0 {
                return Err(Error::Timeout);
            }
        }
        self.check_all_commands_done()?;
        Ok(())
    }

    /// Checks whether all I2C commands have completed execution.
    fn check_all_commands_done(&self) -> Result<(), Error> {
        // NOTE: on esp32 executing the end command generates the end_detect interrupt
        //       but does not seem to clear the done bit! So we don't check the done
        //       status of an end command
        for cmd_reg in self.register_block().comd_iter() {
            let cmd = cmd_reg.read();

            if cmd.bits() != 0x0 && !cmd.opcode().is_end() && !cmd.command_done().bit_is_set() {
                return Err(Error::ExecutionIncomplete);
            }
        }

        Ok(())
    }

    /// Checks for I2C transmission errors and handles them.
    ///
    /// This function inspects specific I2C-related interrupts to detect errors
    /// during communication, such as timeouts, failed acknowledgments, or
    /// arbitration loss. If an error is detected, the function handles it
    /// by resetting the I2C peripheral to clear the error condition and then
    /// returns an appropriate error.
    fn check_errors(&self) -> Result<(), Error> {
        let interrupts = self.register_block().int_raw().read();

        // The ESP32 variant has a slightly different interrupt naming
        // scheme!
        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
                // Handle error cases
                let retval = if interrupts.time_out().bit_is_set() {
                    Err(Error::Timeout)
                } else if interrupts.nack().bit_is_set() {
                    Err(Error::AcknowledgeCheckFailed(estimate_ack_failed_reason(self.register_block())))
                } else if interrupts.arbitration_lost().bit_is_set() {
                    Err(Error::ArbitrationLost)
                } else {
                    Ok(())
                };
            } else {
                // Handle error cases
                let retval = if interrupts.time_out().bit_is_set() {
                    Err(Error::Timeout)
                } else if interrupts.nack().bit_is_set() {
                    Err(Error::AcknowledgeCheckFailed(estimate_ack_failed_reason(self.register_block())))
                } else if interrupts.arbitration_lost().bit_is_set() {
                    Err(Error::ArbitrationLost)
                } else if interrupts.trans_complete().bit_is_set() && self.register_block().sr().read().resp_rec().bit_is_clear() {
                    Err(Error::AcknowledgeCheckFailed(AcknowledgeCheckFailedReason::Data))
                } else {
                    Ok(())
                };
            }
        }

        if retval.is_err() {
            self.reset();
        }

        retval
    }

    /// Updates the configuration of the I2C peripheral.
    ///
    /// This function ensures that the configuration values, such as clock
    /// settings, SDA/SCL filtering, timeouts, and other operational
    /// parameters, which are configured in other functions, are properly
    /// propagated to the I2C hardware. This step is necessary to synchronize
    /// the software-configured settings with the peripheral's internal
    /// registers, ensuring that the hardware behaves according to the
    /// current configuration.
    fn update_config(&self) {
        // Ensure that the configuration of the peripheral is correctly propagated
        // (only necessary for C2, C3, C6, H2 and S3 variant)
        #[cfg(any(esp32c2, esp32c3, esp32c6, esp32h2, esp32s3))]
        self.register_block()
            .ctr()
            .modify(|_, w| w.conf_upgate().set_bit());
    }

    /// Starts an I2C transmission.
    fn start_transmission(&self) {
        // Start transmission
        self.register_block()
            .ctr()
            .modify(|_, w| w.trans_start().set_bit());
    }

    #[cfg(not(any(esp32, esp32s2)))]
    /// Fills the TX FIFO with data from the provided slice.
    fn fill_tx_fifo(&self, bytes: &[u8]) -> Result<usize, Error> {
        let mut index = 0;
        while index < bytes.len()
            && !self
                .register_block()
                .int_raw()
                .read()
                .txfifo_ovf()
                .bit_is_set()
        {
            write_fifo(self.register_block(), bytes[index]);
            index += 1;
        }
        if self
            .register_block()
            .int_raw()
            .read()
            .txfifo_ovf()
            .bit_is_set()
        {
            index -= 1;
            self.register_block()
                .int_clr()
                .write(|w| w.txfifo_ovf().clear_bit_by_one());
        }
        Ok(index)
    }

    #[cfg(not(any(esp32, esp32s2)))]
    /// Writes remaining data from byte slice to the TX FIFO from the specified
    /// index.
    fn write_remaining_tx_fifo_blocking(
        &self,
        start_index: usize,
        bytes: &[u8],
    ) -> Result<(), Error> {
        let mut index = start_index;
        loop {
            self.check_errors()?;

            while !self
                .register_block()
                .int_raw()
                .read()
                .txfifo_wm()
                .bit_is_set()
            {
                self.check_errors()?;
            }

            self.register_block()
                .int_clr()
                .write(|w| w.txfifo_wm().clear_bit_by_one());

            while !self
                .register_block()
                .int_raw()
                .read()
                .txfifo_wm()
                .bit_is_set()
            {
                self.check_errors()?;
            }

            if index >= bytes.len() {
                break Ok(());
            }

            write_fifo(self.register_block(), bytes[index]);
            index += 1;
        }
    }

    #[cfg(any(esp32, esp32s2))]
    /// Fills the TX FIFO with data from the provided slice.
    fn fill_tx_fifo(&self, bytes: &[u8]) -> Result<usize, Error> {
        // on ESP32/ESP32-S2 we currently don't support I2C transactions larger than the
        // FIFO apparently it would be possible by using non-fifo mode
        // see  https://github.com/espressif/arduino-esp32/blob/7e9afe8c5ed7b5bf29624a5cd6e07d431c027b97/cores/esp32/esp32-hal-i2c.c#L615

        if bytes.len() > 31 {
            return Err(Error::FifoExceeded);
        }

        for b in bytes {
            write_fifo(self.register_block(), *b);
        }

        Ok(bytes.len())
    }

    #[cfg(any(esp32, esp32s2))]
    /// Writes remaining data from byte slice to the TX FIFO from the specified
    /// index.
    fn write_remaining_tx_fifo_blocking(
        &self,
        start_index: usize,
        bytes: &[u8],
    ) -> Result<(), Error> {
        // on ESP32/ESP32-S2 we currently don't support I2C transactions larger than the
        // FIFO apparently it would be possible by using non-fifo mode
        // see  https://github.com/espressif/arduino-esp32/blob/7e9afe8c5ed7b5bf29624a5cd6e07d431c027b97/cores/esp32/esp32-hal-i2c.c#L615

        if start_index >= bytes.len() {
            return Ok(());
        }

        // this is only possible when writing the I2C address in release mode
        // from [perform_write_read]
        for b in bytes {
            write_fifo(self.register_block(), *b);
            self.check_errors()?;
        }

        Ok(())
    }

    /// Resets the transmit and receive FIFO buffers
    #[cfg(not(esp32))]
    fn reset_fifo(&self) {
        // First, reset the fifo buffers
        self.register_block().fifo_conf().modify(|_, w| unsafe {
            w.tx_fifo_rst().set_bit();
            w.rx_fifo_rst().set_bit();
            w.nonfifo_en().clear_bit();
            w.fifo_prt_en().set_bit();
            w.rxfifo_wm_thrhd().bits(1);
            w.txfifo_wm_thrhd().bits(8)
        });

        self.register_block().fifo_conf().modify(|_, w| {
            w.tx_fifo_rst().clear_bit();
            w.rx_fifo_rst().clear_bit()
        });

        self.register_block().int_clr().write(|w| {
            w.rxfifo_wm().clear_bit_by_one();
            w.txfifo_wm().clear_bit_by_one()
        });

        self.update_config();
    }

    /// Resets the transmit and receive FIFO buffers
    #[cfg(esp32)]
    fn reset_fifo(&self) {
        // First, reset the fifo buffers
        self.register_block().fifo_conf().modify(|_, w| unsafe {
            w.tx_fifo_rst().set_bit();
            w.rx_fifo_rst().set_bit();
            w.nonfifo_en().clear_bit();
            w.nonfifo_rx_thres().bits(1);
            w.nonfifo_tx_thres().bits(32)
        });

        self.register_block().fifo_conf().modify(|_, w| {
            w.tx_fifo_rst().clear_bit();
            w.rx_fifo_rst().clear_bit()
        });

        self.register_block()
            .int_clr()
            .write(|w| w.rxfifo_full().clear_bit_by_one());
    }

    fn start_write_operation(
        &self,
        address: I2cAddress,
        bytes: &[u8],
        start: bool,
        stop: bool,
    ) -> Result<usize, Error> {
        self.reset_fifo();
        self.reset_command_list();
        let cmd_iterator = &mut self.register_block().comd_iter();

        if start {
            add_cmd(cmd_iterator, Command::Start)?;
        }

        self.setup_write(address, bytes, start, cmd_iterator)?;

        add_cmd(
            cmd_iterator,
            if stop { Command::Stop } else { Command::End },
        )?;
        let index = self.fill_tx_fifo(bytes)?;
        self.start_transmission();

        Ok(index)
    }

    /// Executes an I2C read operation.
    /// - `addr` is the address of the slave device.
    /// - `buffer` is the buffer to store the read data.
    /// - `start` indicates whether the operation should start by a START
    ///   condition and sending the address.
    /// - `stop` indicates whether the operation should end with a STOP
    ///   condition.
    /// - `will_continue` indicates whether there is another read operation
    ///   following this one and we should not nack the last byte.
    /// - `cmd_iterator` is an iterator over the command registers.
    fn start_read_operation(
        &self,
        address: I2cAddress,
        buffer: &mut [u8],
        start: bool,
        stop: bool,
        will_continue: bool,
    ) -> Result<(), Error> {
        self.reset_fifo();
        self.reset_command_list();

        let cmd_iterator = &mut self.register_block().comd_iter();

        if start {
            add_cmd(cmd_iterator, Command::Start)?;
        }

        self.setup_read(address, buffer, start, will_continue, cmd_iterator)?;

        add_cmd(
            cmd_iterator,
            if stop { Command::Stop } else { Command::End },
        )?;
        self.start_transmission();
        Ok(())
    }

    /// Executes an I2C write operation.
    /// - `addr` is the address of the slave device.
    /// - `bytes` is the data two be sent.
    /// - `start` indicates whether the operation should start by a START
    ///   condition and sending the address.
    /// - `stop` indicates whether the operation should end with a STOP
    ///   condition.
    /// - `cmd_iterator` is an iterator over the command registers.
    fn write_operation_blocking(
        &self,
        address: I2cAddress,
        bytes: &[u8],
        start: bool,
        stop: bool,
    ) -> Result<(), Error> {
        self.clear_all_interrupts();

        // Short circuit for zero length writes without start or end as that would be an
        // invalid operation write lengths in the TRM (at least for ESP32-S3) are 1-255
        if bytes.is_empty() && !start && !stop {
            return Ok(());
        }

        let index = self.start_write_operation(address, bytes, start, stop)?;
        // Fill the FIFO with the remaining bytes:
        self.write_remaining_tx_fifo_blocking(index, bytes)?;
        self.wait_for_completion_blocking(!stop)?;
        Ok(())
    }

    /// Executes an I2C read operation.
    /// - `addr` is the address of the slave device.
    /// - `buffer` is the buffer to store the read data.
    /// - `start` indicates whether the operation should start by a START
    ///   condition and sending the address.
    /// - `stop` indicates whether the operation should end with a STOP
    ///   condition.
    /// - `will_continue` indicates whether there is another read operation
    ///   following this one and we should not nack the last byte.
    /// - `cmd_iterator` is an iterator over the command registers.
    fn read_operation_blocking(
        &self,
        address: I2cAddress,
        buffer: &mut [u8],
        start: bool,
        stop: bool,
        will_continue: bool,
    ) -> Result<(), Error> {
        self.clear_all_interrupts();

        // Short circuit for zero length reads as that would be an invalid operation
        // read lengths in the TRM (at least for ESP32-S3) are 1-255
        if buffer.is_empty() {
            return Ok(());
        }

        self.start_read_operation(address, buffer, start, stop, will_continue)?;
        self.read_all_from_fifo_blocking(buffer)?;
        self.wait_for_completion_blocking(!stop)?;
        Ok(())
    }

    /// Executes an async I2C write operation.
    /// - `addr` is the address of the slave device.
    /// - `bytes` is the data two be sent.
    /// - `start` indicates whether the operation should start by a START
    ///   condition and sending the address.
    /// - `stop` indicates whether the operation should end with a STOP
    ///   condition.
    /// - `cmd_iterator` is an iterator over the command registers.
    async fn write_operation(
        &self,
        address: I2cAddress,
        bytes: &[u8],
        start: bool,
        stop: bool,
    ) -> Result<(), Error> {
        self.clear_all_interrupts();

        // Short circuit for zero length writes without start or end as that would be an
        // invalid operation write lengths in the TRM (at least for ESP32-S3) are 1-255
        if bytes.is_empty() && !start && !stop {
            return Ok(());
        }

        let index = self.start_write_operation(address, bytes, start, stop)?;
        // Fill the FIFO with the remaining bytes:
        self.write_remaining_tx_fifo(index, bytes).await?;
        self.wait_for_completion(!stop).await?;
        Ok(())
    }

    /// Executes an async I2C read operation.
    /// - `addr` is the address of the slave device.
    /// - `buffer` is the buffer to store the read data.
    /// - `start` indicates whether the operation should start by a START
    ///   condition and sending the address.
    /// - `stop` indicates whether the operation should end with a STOP
    ///   condition.
    /// - `will_continue` indicates whether there is another read operation
    ///   following this one and we should not nack the last byte.
    /// - `cmd_iterator` is an iterator over the command registers.
    async fn read_operation(
        &self,
        address: I2cAddress,
        buffer: &mut [u8],
        start: bool,
        stop: bool,
        will_continue: bool,
    ) -> Result<(), Error> {
        self.clear_all_interrupts();

        // Short circuit for zero length reads as that would be an invalid operation
        // read lengths in the TRM (at least for ESP32-S3) are 1-255
        if buffer.is_empty() {
            return Ok(());
        }

        self.start_read_operation(address, buffer, start, stop, will_continue)?;
        self.read_all_from_fifo(buffer).await?;
        self.wait_for_completion(!stop).await?;
        Ok(())
    }

    fn read_blocking(
        &self,
        address: I2cAddress,
        buffer: &mut [u8],
        start: bool,
        stop: bool,
        will_continue: bool,
    ) -> Result<(), Error> {
        let chunk_count = buffer.len().div_ceil(I2C_CHUNK_SIZE);
        for (idx, chunk) in buffer.chunks_mut(I2C_CHUNK_SIZE).enumerate() {
            self.read_operation_blocking(
                address,
                chunk,
                start && idx == 0,
                stop && idx == chunk_count - 1,
                will_continue || idx < chunk_count - 1,
            )?;
        }

        Ok(())
    }

    fn write_blocking(
        &self,
        address: I2cAddress,
        buffer: &[u8],
        start: bool,
        stop: bool,
    ) -> Result<(), Error> {
        if buffer.is_empty() {
            return self.write_operation_blocking(address, &[], start, stop);
        }
        let chunk_count = buffer.len().div_ceil(I2C_CHUNK_SIZE);
        for (idx, chunk) in buffer.chunks(I2C_CHUNK_SIZE).enumerate() {
            self.write_operation_blocking(
                address,
                chunk,
                start && idx == 0,
                stop && idx == chunk_count - 1,
            )?;
        }

        Ok(())
    }

    async fn read(
        &self,
        address: I2cAddress,
        buffer: &mut [u8],
        start: bool,
        stop: bool,
        will_continue: bool,
    ) -> Result<(), Error> {
        let chunk_count = buffer.len().div_ceil(I2C_CHUNK_SIZE);
        for (idx, chunk) in buffer.chunks_mut(I2C_CHUNK_SIZE).enumerate() {
            self.read_operation(
                address,
                chunk,
                start && idx == 0,
                stop && idx == chunk_count - 1,
                will_continue || idx < chunk_count - 1,
            )
            .await?;
        }

        Ok(())
    }

    async fn write(
        &self,
        address: I2cAddress,
        buffer: &[u8],
        start: bool,
        stop: bool,
    ) -> Result<(), Error> {
        if buffer.is_empty() {
            return self.write_operation(address, &[], start, stop).await;
        }
        let chunk_count = buffer.len().div_ceil(I2C_CHUNK_SIZE);
        for (idx, chunk) in buffer.chunks(I2C_CHUNK_SIZE).enumerate() {
            self.write_operation(
                address,
                chunk,
                start && idx == 0,
                stop && idx == chunk_count - 1,
            )
            .await?;
        }

        Ok(())
    }
}

fn check_timeout(v: u32, max: u32) -> Result<u32, ConfigError> {
    if v <= max {
        Ok(v)
    } else {
        Err(ConfigError::TimeoutInvalid)
    }
}

/// Peripheral state for an I2C instance.
#[doc(hidden)]
#[non_exhaustive]
pub struct State {
    /// Waker for the asynchronous operations.
    pub waker: AtomicWaker,
}

/// I2C Peripheral Instance
#[doc(hidden)]
pub trait Instance: Peripheral<P = Self> + Into<AnyI2c> + 'static {
    /// Returns the peripheral data and state describing this instance.
    fn parts(&self) -> (&Info, &State);

    /// Returns the peripheral data describing this instance.
    #[inline(always)]
    fn info(&self) -> &Info {
        self.parts().0
    }

    /// Returns the peripheral state for this instance.
    #[inline(always)]
    fn state(&self) -> &State {
        self.parts().1
    }
}

/// Adds a command to the I2C command sequence.
fn add_cmd<'a, I>(cmd_iterator: &mut I, command: Command) -> Result<(), Error>
where
    I: Iterator<Item = &'a COMD>,
{
    let cmd = cmd_iterator.next().ok_or(Error::CommandNumberExceeded)?;

    cmd.write(|w| match command {
        Command::Start => w.opcode().rstart(),
        Command::Stop => w.opcode().stop(),
        Command::End => w.opcode().end(),
        Command::Write {
            ack_exp,
            ack_check_en,
            length,
        } => unsafe {
            w.opcode().write();
            w.ack_exp().bit(ack_exp == Ack::Nack);
            w.ack_check_en().bit(ack_check_en);
            w.byte_num().bits(length);
            w
        },
        Command::Read { ack_value, length } => unsafe {
            w.opcode().read();
            w.ack_value().bit(ack_value == Ack::Nack);
            w.byte_num().bits(length);
            w
        },
    });

    Ok(())
}

#[cfg(not(esp32s2))]
fn read_fifo(register_block: &RegisterBlock) -> u8 {
    register_block.data().read().fifo_rdata().bits()
}

#[cfg(not(esp32))]
fn write_fifo(register_block: &RegisterBlock, data: u8) {
    register_block
        .data()
        .write(|w| unsafe { w.fifo_rdata().bits(data) });
}

#[cfg(esp32s2)]
fn read_fifo(register_block: &RegisterBlock) -> u8 {
    let base_addr = register_block.scl_low_period().as_ptr();
    let fifo_ptr = (if base_addr as u32 == 0x3f413000 {
        0x6001301c
    } else {
        0x6002701c
    }) as *mut u32;
    unsafe { (fifo_ptr.read_volatile() & 0xff) as u8 }
}

#[cfg(esp32)]
fn write_fifo(register_block: &RegisterBlock, data: u8) {
    let base_addr = register_block.scl_low_period().as_ptr();
    let fifo_ptr = (if base_addr as u32 == 0x3FF53000 {
        0x6001301c
    } else {
        0x6002701c
    }) as *mut u32;
    unsafe {
        fifo_ptr.write_volatile(data as u32);
    }
}

// Estimate the reason for an acknowledge check failure on a best effort basis.
// When in doubt it's better to return `Unknown` than to return a wrong reason.
fn estimate_ack_failed_reason(_register_block: &RegisterBlock) -> AcknowledgeCheckFailedReason {
    cfg_if::cfg_if! {
        if #[cfg(any(esp32, esp32s2, esp32c2, esp32c3))] {
            AcknowledgeCheckFailedReason::Unknown
        } else {
            // this is based on observations rather than documented behavior
            if _register_block.fifo_st().read().txfifo_raddr().bits() <= 1 {
                AcknowledgeCheckFailedReason::Address
            } else {
                AcknowledgeCheckFailedReason::Data
            }
        }
    }
}

macro_rules! instance {
    ($inst:ident, $peri:ident, $scl:ident, $sda:ident, $interrupt:ident) => {
        impl Instance for crate::peripherals::$inst {
            fn parts(&self) -> (&Info, &State) {
                #[crate::handler]
                pub(super) fn irq_handler() {
                    async_handler(&PERIPHERAL, &STATE);
                }

                static STATE: State = State {
                    waker: AtomicWaker::new(),
                };

                static PERIPHERAL: Info = Info {
                    register_block: crate::peripherals::$inst::ptr(),
                    peripheral: crate::system::Peripheral::$peri,
                    async_handler: irq_handler,
                    interrupt: Interrupt::$interrupt,
                    scl_output: OutputSignal::$scl,
                    scl_input: InputSignal::$scl,
                    sda_output: OutputSignal::$sda,
                    sda_input: InputSignal::$sda,
                };
                (&PERIPHERAL, &STATE)
            }
        }
    };
}

#[cfg(i2c0)]
instance!(I2C0, I2cExt0, I2CEXT0_SCL, I2CEXT0_SDA, I2C_EXT0);
#[cfg(i2c1)]
instance!(I2C1, I2cExt1, I2CEXT1_SCL, I2CEXT1_SDA, I2C_EXT1);

crate::any_peripheral! {
    /// Represents any I2C peripheral.
    pub peripheral AnyI2c {
        #[cfg(i2c0)]
        I2c0(crate::peripherals::I2C0),
        #[cfg(i2c1)]
        I2c1(crate::peripherals::I2C1),
    }
}

impl Instance for AnyI2c {
    delegate::delegate! {
        to match &self.0 {
            AnyI2cInner::I2c0(i2c) => i2c,
            #[cfg(i2c1)]
            AnyI2cInner::I2c1(i2c) => i2c,
        } {
            fn parts(&self) -> (&Info, &State);
        }
    }
}
