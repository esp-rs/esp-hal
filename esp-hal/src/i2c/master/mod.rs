#![cfg_attr(docsrs, procmacros::doc_replace)]
//! # Inter-Integrated Circuit (I2C) - Master mode
//!
//! ## Overview
//!
//! This driver implements the I2C Master mode. In this mode, the MCU initiates
//! and controls the I2C communication with one or more slave devices. Slave
//! devices are identified by their unique I2C addresses.
//!
//! ## Configuration
//!
//! The driver can be configured using the [`Config`] struct. To create a
//! configuration, you can use the [`Config::default()`] method, and then modify
//! the individual settings as needed, by calling `with_*` methods on the
//! [`Config`] struct.
//!
//! ```rust, no_run
//! # {before_snippet}
//! use esp_hal::{i2c::master::Config, time::Rate};
//!
//! let config = Config::default().with_frequency(Rate::from_khz(100));
//! # {after_snippet}
//! ```
//!
//! You will then need to pass the configuration to [`I2c::new`], and you can
//! also change the configuration later by calling [`I2c::apply_config`].
//!
//! You will also need to specify the SDA and SCL pins when you create the
//! driver instance.
//! ```rust, no_run
//! # {before_snippet}
//! use esp_hal::i2c::master::I2c;
//! # use esp_hal::{i2c::master::Config, time::Rate};
//! #
//! # let config = Config::default();
//! #
//! // You need to configure the driver during initialization:
//! let mut i2c = I2c::new(peripherals.I2C0, config)?
//!     .with_sda(peripherals.GPIO2)
//!     .with_scl(peripherals.GPIO3);
//!
//! // You can change the configuration later:
//! let new_config = config.with_frequency(Rate::from_khz(400));
//! i2c.apply_config(&new_config)?;
//! # {after_snippet}
//! ```
//!
//! ## Usage
//!
//! The master communicates with slave devices using I2C transactions. A
//! transaction can be a write, a read, or a combination of both. The
//! [`I2c`] driver provides methods for performing these transactions:
//! ```rust, no_run
//! # {before_snippet}
//! # use esp_hal::i2c::master::{I2c, Config, Operation};
//! # let config = Config::default();
//! # let mut i2c = I2c::new(peripherals.I2C0, config)?;
//! #
//! // `u8` is automatically converted to `I2cAddress::SevenBit`. The device
//! // address does not contain the `R/W` bit!
//! const DEVICE_ADDR: u8 = 0x77;
//! let write_buffer = [0xAA];
//! let mut read_buffer = [0u8; 22];
//!
//! i2c.write(DEVICE_ADDR, &write_buffer)?;
//! i2c.write_read(DEVICE_ADDR, &write_buffer, &mut read_buffer)?;
//! i2c.read(DEVICE_ADDR, &mut read_buffer)?;
//! i2c.transaction(
//!     DEVICE_ADDR,
//!     &mut [
//!         Operation::Write(&write_buffer),
//!         Operation::Read(&mut read_buffer),
//!     ],
//! )?;
//! # {after_snippet}
//! ```
//! If you configure the driver to `async` mode, the driver also provides
//! asynchronous versions of these methods:
//! ```rust, no_run
//! # {before_snippet}
//! # use esp_hal::i2c::master::{I2c, Config, Operation};
//! # let config = Config::default();
//! # let mut i2c = I2c::new(peripherals.I2C0, config)?;
//! #
//! # const DEVICE_ADDR: u8 = 0x77;
//! # let write_buffer = [0xAA];
//! # let mut read_buffer = [0u8; 22];
//! #
//! // Reconfigure the driver to use async mode.
//! let mut i2c = i2c.into_async();
//!
//! i2c.write_async(DEVICE_ADDR, &write_buffer).await?;
//! i2c.write_read_async(DEVICE_ADDR, &write_buffer, &mut read_buffer)
//!     .await?;
//! i2c.read_async(DEVICE_ADDR, &mut read_buffer).await?;
//! i2c.transaction_async(
//!     DEVICE_ADDR,
//!     &mut [
//!         Operation::Write(&write_buffer),
//!         Operation::Read(&mut read_buffer),
//!     ],
//! )
//! .await?;
//!
//! // You should still be able to use the blocking methods, if you need to:
//! i2c.write(DEVICE_ADDR, &write_buffer)?;
//!
//! # {after_snippet}
//! ```
//!
//! The I2C driver also implements [embedded-hal] and [embedded-hal-async]
//! traits, so you can use it with any crate that supports these traits.
//!
//! [embedded-hal]: embedded_hal::i2c
//! [embedded-hal-async]: embedded_hal_async::i2c

use core::{
    marker::PhantomData,
    pin::Pin,
    task::{Context, Poll},
};

use embedded_hal::i2c::Operation as EhalOperation;
use enumset::{EnumSet, EnumSetType};

use crate::{
    Async,
    Blocking,
    DriverMode,
    asynch::AtomicWaker,
    clock::Clocks,
    gpio::{
        DriveMode,
        InputSignal,
        OutputConfig,
        OutputSignal,
        PinGuard,
        Pull,
        interconnect::{self, PeripheralOutput},
    },
    handler,
    interrupt::{self, InterruptHandler},
    pac::i2c0::{COMD, RegisterBlock},
    private,
    ram,
    system::PeripheralGuard,
    time::{Duration, Instant, Rate},
};

const I2C_FIFO_SIZE: usize = property!("i2c_master.fifo_size");
// Chunk writes/reads by this size
const I2C_CHUNK_SIZE: usize = I2C_FIFO_SIZE - 1;
const CLEAR_BUS_TIMEOUT_MS: Duration = Duration::from_millis(50);

/// Representation of I2C address.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum I2cAddress {
    /// 7-bit address mode type.
    ///
    /// Note that 7-bit addresses are specified in **right-aligned** form, e.g.
    /// in the range `0x00..=0x7F`.
    ///
    /// For example, a device that has the seven bit address of `0b011_0010`,
    /// and therefore is addressed on the wire using:
    ///
    /// * `0b0110010_0` or `0x64` for *writes*
    /// * `0b0110010_1` or `0x65` for *reads*
    ///
    /// The above address is specified as 0b0011_0010 or 0x32, NOT 0x64 or 0x65.
    SevenBit(u8),
}

impl I2cAddress {
    fn validate(&self) -> Result<(), Error> {
        match self {
            I2cAddress::SevenBit(addr) => {
                if *addr > 0x7F {
                    return Err(Error::AddressInvalid(*self));
                }
            }
        }

        Ok(())
    }
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
    i2c_master_bus_timeout_is_exponential,
    doc = "Note that the effective timeout may be longer than the value configured here."
)]
#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash, strum::Display)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
#[instability::unstable]
pub enum BusTimeout {
    /// Use the maximum timeout value.
    Maximum,

    /// Disable timeout control.
    #[cfg(i2c_master_has_bus_timeout_enable)]
    Disabled,

    /// Timeout in bus clock cycles.
    BusCycles(u32),
}

impl BusTimeout {
    /// Returns the timeout in APB cycles, or `None` if the timeout is disabled.
    ///
    /// Newer devices only support power-of-two timeouts, so we'll have to take
    /// the logarithm of the timeout value. This may cause considerably
    /// longer (at most ~double) timeouts than configured. We may provide an
    /// `ApbCycles` variant in the future to allow specifying the timeout in
    /// APB cycles directly.
    fn apb_cycles(self, half_bus_cycle: u32) -> Result<Option<u32>, ConfigError> {
        match self {
            BusTimeout::Maximum => Ok(Some(property!("i2c_master.max_bus_timeout"))),

            #[cfg(i2c_master_has_bus_timeout_enable)]
            BusTimeout::Disabled => Ok(None),

            BusTimeout::BusCycles(cycles) => {
                let raw = if cfg!(i2c_master_bus_timeout_is_exponential) {
                    let to_peri = (cycles * 2 * half_bus_cycle).max(1);
                    let log2 = to_peri.ilog2();
                    // If not a power of 2, round up so that we don't shorten timeouts.
                    if to_peri != 1 << log2 { log2 + 1 } else { log2 }
                } else {
                    cycles * 2 * half_bus_cycle
                };

                if raw <= property!("i2c_master.max_bus_timeout") {
                    Ok(Some(raw))
                } else {
                    Err(ConfigError::TimeoutTooLong)
                }
            }
        }
    }
}

/// Software timeout for I2C operations.
///
/// This timeout is used to limit the duration of I2C operations in software.
/// Note that using this in conjunction with `async` operations will cause the
/// task to be woken up continuously until the operation completes or the
/// timeout is reached. You should prefer using an asynchronous
/// timeout mechanism (like [`embassy_time::with_timeout`]) for better
/// efficiency.
///
/// [`embassy_time::with_timeout`]: https://docs.rs/embassy-time/0.4.0/embassy_time/fn.with_timeout.html
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SoftwareTimeout {
    /// No software timeout is set.
    None,

    /// Define a fixed timeout for I2C operations.
    Transaction(Duration),

    /// Define a data length dependent timeout for I2C operations.
    ///
    /// The applied timeout is calculated as `data_length * duration_per_byte`.
    /// In [`I2c::transaction`] and [`I2c::transaction_async`], the timeout is
    /// applied separately for each operation.
    PerByte(Duration),
}

/// When the FSM remains unchanged for more than the 2^ the given amount of bus
/// clock cycles a timeout will be triggered.
///
/// The default value is 23 (2^23 clock cycles).
#[instability::unstable]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg(i2c_master_has_fsm_timeouts)]
pub struct FsmTimeout {
    value: u8,
}

#[cfg(i2c_master_has_fsm_timeouts)]
impl FsmTimeout {
    const FSM_TIMEOUT_MAX: u8 = 23;

    /// Creates a new timeout.
    ///
    /// The meaning of the value and the allowed range of values is different
    /// for different chips.
    #[instability::unstable]
    pub const fn new_const<const VALUE: u8>() -> Self {
        const {
            core::assert!(VALUE <= Self::FSM_TIMEOUT_MAX, "Invalid timeout value");
        }
        Self { value: VALUE }
    }

    /// Creates a new timeout.
    ///
    /// The meaning of the value and the allowed range of values is different
    /// for different chips.
    #[instability::unstable]
    pub fn new(value: u8) -> Result<Self, ConfigError> {
        if value > Self::FSM_TIMEOUT_MAX {
            return Err(ConfigError::TimeoutTooLong);
        }

        Ok(Self { value })
    }

    fn value(&self) -> u8 {
        self.value
    }
}

#[cfg(i2c_master_has_fsm_timeouts)]
impl Default for FsmTimeout {
    fn default() -> Self {
        Self::new_const::<{ Self::FSM_TIMEOUT_MAX }>()
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
    /// The given address is invalid.
    AddressInvalid(I2cAddress),
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
                write!(f, "The acknowledgment check failed. Reason: {reason}")
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
            Error::AddressInvalid(address) => {
                write!(f, "The given address ({address:?}) is invalid")
            }
        }
    }
}

/// I2C-specific configuration errors
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum ConfigError {
    /// Provided bus frequency is not valid for the current configuration.
    FrequencyOutOfRange,
    /// Provided timeout is not valid for the current configuration.
    TimeoutTooLong,
}

impl core::error::Error for ConfigError {}

impl core::fmt::Display for ConfigError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            ConfigError::FrequencyOutOfRange => write!(
                f,
                "Provided bus frequency is invalid for the current configuration"
            ),
            ConfigError::TimeoutTooLong => write!(
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
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
#[derive(Debug)]
enum Command {
    Start,
    Stop,
    End,
    Write {
        /// This bit is to set an expected ACK value for the transmitter.
        ack_exp: Ack,
        /// Enables checking the ACK value received against the ack_exp value.
        ack_check_en: bool,
        /// Length of data (in bytes) to be written. The maximum length is
        #[doc = property!("i2c_master.fifo_size", str)]
        /// , while the minimum is 1.
        length: u8,
    },
    Read {
        /// Indicates whether the receiver will send an ACK after this byte has
        /// been received.
        ack_value: Ack,
        /// Length of data (in bytes) to be written. The maximum length is
        #[doc = property!("i2c_master.fifo_size", str)]
        /// , while the minimum is 1.
        length: u8,
    },
}

enum OperationType {
    Write = 0,
    Read  = 1,
}

#[derive(Eq, PartialEq, Copy, Clone, Debug)]
enum Ack {
    Ack  = 0,
    Nack = 1,
}

/// I2C driver configuration
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, procmacros::BuilderLite)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Config {
    /// The I2C clock frequency.
    ///
    /// Default value: 100 kHz.
    frequency: Rate,

    /// I2C SCL timeout period.
    ///
    /// Default value:
    #[cfg_attr(i2c_master_has_bus_timeout_enable, doc = "disabled")]
    #[cfg_attr(not(i2c_master_has_bus_timeout_enable), doc = concat!(property!("i2c_master.max_bus_timeout", str), " bus cycles"))]
    #[builder_lite(unstable)]
    timeout: BusTimeout,

    /// Software timeout.
    ///
    /// Default value: disabled.
    software_timeout: SoftwareTimeout,

    /// Sets the threshold value for the unchanged period of the SCL_FSM.
    ///
    /// Default value: 16.
    #[cfg(i2c_master_has_fsm_timeouts)]
    #[builder_lite(unstable)]
    scl_st_timeout: FsmTimeout,

    /// Sets the threshold for the unchanged duration of the SCL_MAIN_FSM.
    ///
    /// Default value: 16.
    #[cfg(i2c_master_has_fsm_timeouts)]
    #[builder_lite(unstable)]
    scl_main_st_timeout: FsmTimeout,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            frequency: Rate::from_khz(100),

            #[cfg(i2c_master_has_bus_timeout_enable)]
            timeout: BusTimeout::Disabled,
            #[cfg(not(i2c_master_has_bus_timeout_enable))]
            timeout: BusTimeout::Maximum,

            software_timeout: SoftwareTimeout::None,

            #[cfg(i2c_master_has_fsm_timeouts)]
            scl_st_timeout: Default::default(),
            #[cfg(i2c_master_has_fsm_timeouts)]
            scl_main_st_timeout: Default::default(),
        }
    }
}

#[procmacros::doc_replace]
/// I2C driver
///
/// ## Example
///
/// ```rust, no_run
/// # {before_snippet}
/// use esp_hal::i2c::master::{Config, I2c};
/// # const DEVICE_ADDR: u8 = 0x77;
/// let mut i2c = I2c::new(peripherals.I2C0, Config::default())?
///     .with_sda(peripherals.GPIO1)
///     .with_scl(peripherals.GPIO2);
///
/// let mut data = [0u8; 22];
/// i2c.write_read(DEVICE_ADDR, &[0xaa], &mut data)?;
/// # {after_snippet}
/// ```
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct I2c<'d, Dm: DriverMode> {
    i2c: AnyI2c<'d>,
    phantom: PhantomData<Dm>,
    guard: PeripheralGuard,
    config: DriverConfig,
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct DriverConfig {
    config: Config,
    sda_pin: PinGuard,
    scl_pin: PinGuard,
}

#[instability::unstable]
impl<Dm: DriverMode> embassy_embedded_hal::SetConfig for I2c<'_, Dm> {
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
        self.driver()
            .transaction_impl(
                I2cAddress::SevenBit(address),
                operations.iter_mut().map(Operation::from),
            )
            .inspect_err(|error| self.internal_recover(error))
    }
}

impl<'d> I2c<'d, Blocking> {
    #[procmacros::doc_replace]
    /// Create a new I2C instance.
    ///
    /// ## Errors
    ///
    /// A [`ConfigError`] variant will be returned if bus frequency or timeout
    /// passed in config is invalid.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::i2c::master::{Config, I2c};
    /// let i2c = I2c::new(peripherals.I2C0, Config::default())?
    ///     .with_sda(peripherals.GPIO1)
    ///     .with_scl(peripherals.GPIO2);
    /// # {after_snippet}
    /// ```
    pub fn new(i2c: impl Instance + 'd, config: Config) -> Result<Self, ConfigError> {
        let guard = PeripheralGuard::new(i2c.info().peripheral);

        let sda_pin = PinGuard::new_unconnected(i2c.info().sda_output);
        let scl_pin = PinGuard::new_unconnected(i2c.info().scl_output);

        let mut i2c = I2c {
            i2c: i2c.degrade(),
            phantom: PhantomData,
            guard,
            config: DriverConfig {
                config,
                sda_pin,
                scl_pin,
            },
        };

        i2c.apply_config(&config)?;

        Ok(i2c)
    }

    /// Reconfigures the driver to operate in [`Async`] mode.
    ///
    /// See the [`Async`] documentation for an example on how to use this
    /// method.
    pub fn into_async(mut self) -> I2c<'d, Async> {
        self.set_interrupt_handler(self.driver().info.async_handler);

        I2c {
            i2c: self.i2c,
            phantom: PhantomData,
            guard: self.guard,
            config: self.config,
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
    /// You can restore the default/unhandled interrupt handler by passing
    /// [DEFAULT_INTERRUPT_HANDLER][crate::interrupt::DEFAULT_INTERRUPT_HANDLER].
    ///
    /// # Panics
    ///
    /// Panics if passed interrupt handler is invalid (e.g. has priority
    /// `None`)
    #[instability::unstable]
    pub fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        self.i2c.set_interrupt_handler(handler);
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
}

impl private::Sealed for I2c<'_, Blocking> {}

#[instability::unstable]
impl crate::interrupt::InterruptConfigurable for I2c<'_, Blocking> {
    fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        self.i2c.set_interrupt_handler(handler);
    }
}

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
    #[cfg(i2c_master_has_tx_fifo_watermark)]
    TxFifoWatermark,
}

#[must_use = "futures do nothing unless you `.await` or poll them"]
struct I2cFuture<'a> {
    events: EnumSet<Event>,
    driver: Driver<'a>,
    deadline: Option<Instant>,
    /// True if the Future has been polled to completion.
    finished: bool,
}

impl<'a> I2cFuture<'a> {
    pub fn new(events: EnumSet<Event>, driver: Driver<'a>, deadline: Option<Instant>) -> Self {
        driver.regs().int_ena().modify(|_, w| {
            for event in events {
                match event {
                    Event::EndDetect => w.end_detect().set_bit(),
                    Event::TxComplete => w.trans_complete().set_bit(),
                    #[cfg(i2c_master_has_tx_fifo_watermark)]
                    Event::TxFifoWatermark => w.txfifo_wm().set_bit(),
                };
            }

            w.arbitration_lost().set_bit();
            w.time_out().set_bit();
            w.nack().set_bit();
            #[cfg(i2c_master_has_fsm_timeouts)]
            {
                w.scl_main_st_to().set_bit();
                w.scl_st_to().set_bit();
            }

            w
        });

        Self::new_blocking(events, driver, deadline)
    }

    pub fn new_blocking(
        events: EnumSet<Event>,
        driver: Driver<'a>,
        deadline: Option<Instant>,
    ) -> Self {
        Self {
            events,
            driver,
            deadline,
            finished: false,
        }
    }

    fn is_done(&self) -> bool {
        !self.driver.info.interrupts().is_disjoint(self.events)
    }

    fn poll_completion(&mut self) -> Poll<Result<(), Error>> {
        // Grab the current time before doing anything. This will ensure that a long
        // interruption still allows the peripheral sufficient time to complete the
        // operation (i.e. it ensures that the deadline is "at least", not "at most").
        let now = if self.deadline.is_some() {
            Instant::now()
        } else {
            Instant::EPOCH
        };
        let error = self.driver.check_errors();

        let result = if self.is_done() {
            // Even though we are done, we have to check for NACK and arbitration loss.
            let result = if error == Err(Error::Timeout) {
                // We are both done, and timed out. Likely the transaction has completed, but we
                // checked too late?
                Ok(())
            } else {
                error
            };
            Poll::Ready(result)
        } else if error.is_err() {
            Poll::Ready(error)
        } else if let Some(deadline) = self.deadline
            && now > deadline
        {
            // If the deadline is reached, we return an error.
            Poll::Ready(Err(Error::Timeout))
        } else {
            Poll::Pending
        };

        if result.is_ready() {
            self.finished = true;
        }

        result
    }
}

impl core::future::Future for I2cFuture<'_> {
    type Output = Result<(), Error>;

    fn poll(mut self: Pin<&mut Self>, ctx: &mut Context<'_>) -> Poll<Self::Output> {
        self.driver.state.waker.register(ctx.waker());

        let result = self.poll_completion();

        if result.is_pending() && self.deadline.is_some() {
            ctx.waker().wake_by_ref();
        }

        result
    }
}

impl Drop for I2cFuture<'_> {
    fn drop(&mut self) {
        if !self.finished {
            let result = self.poll_completion();
            if result.is_pending() || result == Poll::Ready(Err(Error::Timeout)) {
                self.driver.reset_fsm(true);
            }
        }
    }
}

impl<'d> I2c<'d, Async> {
    /// Reconfigures the driver to operate in [`Blocking`] mode.
    ///
    /// See the [`Blocking`] documentation for an example on how to use this
    /// method.
    pub fn into_blocking(self) -> I2c<'d, Blocking> {
        self.i2c.disable_peri_interrupt();

        I2c {
            i2c: self.i2c,
            phantom: PhantomData,
            guard: self.guard,
            config: self.config,
        }
    }

    #[procmacros::doc_replace]
    /// Writes bytes to slave with given `address`.
    ///
    /// Note that dropping the returned Future will abort the transfer, but doing so will
    /// block while the driver is finishing clearing and releasing the bus.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::i2c::master::{Config, I2c};
    /// const DEVICE_ADDR: u8 = 0x77;
    /// let mut i2c = I2c::new(peripherals.I2C0, Config::default())?
    ///     .with_sda(peripherals.GPIO1)
    ///     .with_scl(peripherals.GPIO2)
    ///     .into_async();
    ///
    /// i2c.write_async(DEVICE_ADDR, &[0xaa]).await?;
    /// # {after_snippet}
    /// ```
    pub async fn write_async<A: Into<I2cAddress>>(
        &mut self,
        address: A,
        buffer: &[u8],
    ) -> Result<(), Error> {
        self.transaction_async(address, &mut [Operation::Write(buffer)])
            .await
    }

    #[procmacros::doc_replace]
    /// Reads enough bytes from slave with `address` to fill `buffer`.
    ///
    /// Note that dropping the returned Future will abort the transfer, but doing so will
    /// block while the driver is finishing clearing and releasing the bus.
    ///
    /// ## Errors
    ///
    /// The corresponding error variant from [`Error`] will be returned if the
    /// passed buffer has zero length.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::i2c::master::{Config, I2c};
    /// const DEVICE_ADDR: u8 = 0x77;
    /// let mut i2c = I2c::new(peripherals.I2C0, Config::default())?
    ///     .with_sda(peripherals.GPIO1)
    ///     .with_scl(peripherals.GPIO2)
    ///     .into_async();
    ///
    /// let mut data = [0u8; 22];
    /// i2c.read_async(DEVICE_ADDR, &mut data).await?;
    /// # {after_snippet}
    /// ```
    pub async fn read_async<A: Into<I2cAddress>>(
        &mut self,
        address: A,
        buffer: &mut [u8],
    ) -> Result<(), Error> {
        self.transaction_async(address, &mut [Operation::Read(buffer)])
            .await
    }

    #[procmacros::doc_replace]
    /// Writes bytes to slave with given `address` and then reads enough
    /// bytes to fill `buffer` *in a single transaction*.
    ///
    /// Note that dropping the returned Future will abort the transfer, but doing so will
    /// block while the driver is finishing clearing and releasing the bus.
    ///
    /// ## Errors
    ///
    /// The corresponding error variant from [`Error`] will be returned if the
    /// passed buffer has zero length.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::i2c::master::{Config, I2c};
    /// const DEVICE_ADDR: u8 = 0x77;
    /// let mut i2c = I2c::new(peripherals.I2C0, Config::default())?
    ///     .with_sda(peripherals.GPIO1)
    ///     .with_scl(peripherals.GPIO2)
    ///     .into_async();
    ///
    /// let mut data = [0u8; 22];
    /// i2c.write_read_async(DEVICE_ADDR, &[0xaa], &mut data)
    ///     .await?;
    /// # {after_snippet}
    /// ```
    pub async fn write_read_async<A: Into<I2cAddress>>(
        &mut self,
        address: A,
        write_buffer: &[u8],
        read_buffer: &mut [u8],
    ) -> Result<(), Error> {
        self.transaction_async(
            address,
            &mut [Operation::Write(write_buffer), Operation::Read(read_buffer)],
        )
        .await
    }

    #[procmacros::doc_replace]
    /// Execute the provided operations on the I2C bus as a single transaction.
    ///
    /// Note that dropping the returned Future will abort the transfer, but doing so will
    /// block while the driver is finishing clearing and releasing the bus.
    ///
    /// Transaction contract:
    /// - Before executing the first operation an ST is sent automatically. This is followed by
    ///   SAD+R/W as appropriate.
    /// - Data from adjacent operations of the same type are sent after each other without an SP or
    ///   SR.
    /// - Between adjacent operations of a different type an SR and SAD+R/W is sent.
    /// - After executing the last operation an SP is sent automatically.
    /// - If the last operation is a `Read` the master does not send an acknowledge for the last
    ///   byte.
    ///
    /// - `ST` = start condition
    /// - `SAD+R/W` = slave address followed by bit 1 to indicate reading or 0 to indicate writing
    /// - `SR` = repeated start condition
    /// - `SP` = stop condition
    #[cfg_attr(
        any(esp32, esp32s2),
        doc = "\n\nOn ESP32 and ESP32-S2 there might be issues combining large read/write operations with small (<3 bytes) read/write operations.\n\n"
    )]
    /// ## Errors
    ///
    /// The corresponding error variant from [`Error`] will be returned if the
    /// buffer passed to an [`Operation`] has zero length.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::i2c::master::{Config, I2c, Operation};
    /// const DEVICE_ADDR: u8 = 0x77;
    /// let mut i2c = I2c::new(peripherals.I2C0, Config::default())?
    ///     .with_sda(peripherals.GPIO1)
    ///     .with_scl(peripherals.GPIO2)
    ///     .into_async();
    ///
    /// let mut data = [0u8; 22];
    /// i2c.transaction_async(
    ///     DEVICE_ADDR,
    ///     &mut [Operation::Write(&[0xaa]), Operation::Read(&mut data)],
    /// )
    /// .await?;
    /// # {after_snippet}
    /// ```
    pub async fn transaction_async<'a, A: Into<I2cAddress>>(
        &mut self,
        address: A,
        operations: impl IntoIterator<Item = &'a mut Operation<'a>>,
    ) -> Result<(), Error> {
        self.driver()
            .transaction_impl_async(address.into(), operations.into_iter().map(Operation::from))
            .await
            .inspect_err(|error| self.internal_recover(error))
    }
}

impl<'d, Dm> I2c<'d, Dm>
where
    Dm: DriverMode,
{
    fn driver(&self) -> Driver<'_> {
        Driver {
            info: self.i2c.info(),
            state: self.i2c.state(),
            config: &self.config,
        }
    }

    fn internal_recover(&self, error: &Error) {
        // Timeout errors mean our hardware is (possibly) working when it gets reset. Clear the bus
        // in this case, to prevent leaving the I2C device mid-transfer.
        self.driver().reset_fsm(*error == Error::Timeout)
    }

    /// Connect a pin to the I2C SDA signal.
    ///
    /// This will replace previous pin assignments for this signal.
    pub fn with_sda(mut self, sda: impl PeripheralOutput<'d>) -> Self {
        let info = self.driver().info;
        let input = info.sda_input;
        let output = info.sda_output;
        Driver::connect_pin(sda.into(), input, output, &mut self.config.sda_pin);

        self
    }

    #[procmacros::doc_replace]
    /// Connect a pin to the I2C SCL signal.
    ///
    /// This will replace previous pin assignments for this signal.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::i2c::master::{Config, I2c};
    /// const DEVICE_ADDR: u8 = 0x77;
    /// let i2c = I2c::new(peripherals.I2C0, Config::default())?.with_scl(peripherals.GPIO2);
    /// # {after_snippet}
    /// ```
    pub fn with_scl(mut self, scl: impl PeripheralOutput<'d>) -> Self {
        let info = self.driver().info;
        let input = info.scl_input;
        let output = info.scl_output;
        Driver::connect_pin(scl.into(), input, output, &mut self.config.scl_pin);

        self
    }

    #[procmacros::doc_replace]
    /// Writes bytes to slave with given `address`
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::i2c::master::{Config, I2c};
    /// # let mut i2c = I2c::new(
    /// #   peripherals.I2C0,
    /// #   Config::default(),
    /// # )?;
    /// # const DEVICE_ADDR: u8 = 0x77;
    /// i2c.write(DEVICE_ADDR, &[0xaa])?;
    /// # {after_snippet}
    /// ```
    pub fn write<A: Into<I2cAddress>>(&mut self, address: A, buffer: &[u8]) -> Result<(), Error> {
        self.transaction(address, &mut [Operation::Write(buffer)])
    }

    #[procmacros::doc_replace]
    /// Reads enough bytes from slave with `address` to fill `buffer`
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::i2c::master::{Config, I2c};
    /// # let mut i2c = I2c::new(
    /// #   peripherals.I2C0,
    /// #   Config::default(),
    /// # )?;
    /// # const DEVICE_ADDR: u8 = 0x77;
    /// let mut data = [0u8; 22];
    /// i2c.read(DEVICE_ADDR, &mut data)?;
    /// # {after_snippet}
    /// ```
    ///
    /// ## Errors
    ///
    /// The corresponding error variant from [`Error`] will be returned if the passed buffer has
    /// zero length.
    pub fn read<A: Into<I2cAddress>>(
        &mut self,
        address: A,
        buffer: &mut [u8],
    ) -> Result<(), Error> {
        self.transaction(address, &mut [Operation::Read(buffer)])
    }

    #[procmacros::doc_replace]
    /// Writes bytes to slave with given `address` and then reads enough bytes
    /// to fill `buffer` *in a single transaction*
    ///
    /// ## Errors
    ///
    /// The corresponding error variant from [`Error`] will be returned if the passed buffer has
    /// zero length.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::i2c::master::{Config, I2c};
    /// # let mut i2c = I2c::new(
    /// #   peripherals.I2C0,
    /// #   Config::default(),
    /// # )?;
    /// # const DEVICE_ADDR: u8 = 0x77;
    /// let mut data = [0u8; 22];
    /// i2c.write_read(DEVICE_ADDR, &[0xaa], &mut data)?;
    /// # {after_snippet}
    /// ```
    pub fn write_read<A: Into<I2cAddress>>(
        &mut self,
        address: A,
        write_buffer: &[u8],
        read_buffer: &mut [u8],
    ) -> Result<(), Error> {
        self.transaction(
            address,
            &mut [Operation::Write(write_buffer), Operation::Read(read_buffer)],
        )
    }

    #[procmacros::doc_replace]
    /// Execute the provided operations on the I2C bus.
    ///
    /// Transaction contract:
    /// - Before executing the first operation an ST is sent automatically. This is followed by
    ///   SAD+R/W as appropriate.
    /// - Data from adjacent operations of the same type are sent after each other without an SP or
    ///   SR.
    /// - Between adjacent operations of a different type an SR and SAD+R/W is sent.
    /// - After executing the last operation an SP is sent automatically.
    /// - If the last operation is a `Read` the master does not send an acknowledge for the last
    ///   byte.
    ///
    /// - `ST` = start condition
    /// - `SAD+R/W` = slave address followed by bit 1 to indicate reading or 0 to indicate writing
    /// - `SR` = repeated start condition
    /// - `SP` = stop condition
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::i2c::master::{Config, I2c, Operation};
    /// # let mut i2c = I2c::new(
    /// #   peripherals.I2C0,
    /// #   Config::default(),
    /// # )?;
    /// # const DEVICE_ADDR: u8 = 0x77;
    /// let mut data = [0u8; 22];
    /// i2c.transaction(
    ///     DEVICE_ADDR,
    ///     &mut [Operation::Write(&[0xaa]), Operation::Read(&mut data)],
    /// )?;
    /// # {after_snippet}
    /// ```
    #[cfg_attr(
        any(esp32, esp32s2),
        doc = "\n\nOn ESP32 and ESP32-S2 it is advisable to not combine large read/write operations with small (<3 bytes) read/write operations.\n\n"
    )]
    /// ## Errors
    ///
    /// The corresponding error variant from [`Error`] will be returned if the
    /// buffer passed to an [`Operation`] has zero length.
    pub fn transaction<'a, A: Into<I2cAddress>>(
        &mut self,
        address: A,
        operations: impl IntoIterator<Item = &'a mut Operation<'a>>,
    ) -> Result<(), Error> {
        self.driver()
            .transaction_impl(address.into(), operations.into_iter().map(Operation::from))
            .inspect_err(|error| self.internal_recover(error))
    }

    #[procmacros::doc_replace]
    /// Applies a new configuration.
    ///
    /// ## Errors
    ///
    /// A [`ConfigError`] variant will be returned if bus frequency or timeout
    /// passed in config is invalid.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::i2c::master::{Config, I2c};
    /// let mut i2c = I2c::new(peripherals.I2C0, Config::default())?;
    ///
    /// i2c.apply_config(&Config::default().with_frequency(Rate::from_khz(400)))?;
    /// # {after_snippet}
    /// ```
    pub fn apply_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        self.config.config = *config;
        self.driver().setup(config)?;
        self.driver().reset_fsm(false);
        Ok(())
    }
}

impl embedded_hal_async::i2c::I2c for I2c<'_, Async> {
    async fn transaction(
        &mut self,
        address: u8,
        operations: &mut [EhalOperation<'_>],
    ) -> Result<(), Self::Error> {
        self.driver()
            .transaction_impl_async(address.into(), operations.iter_mut().map(Operation::from))
            .await
            .inspect_err(|error| self.internal_recover(error))
    }
}

#[ram]
fn async_handler(info: &Info, state: &State) {
    // Disable all interrupts. The I2C Future will check events based on the
    // interrupt status bits.
    info.regs().int_ena().write(|w| unsafe { w.bits(0) });

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
        if #[cfg(i2c_master_separate_filter_config_registers)] {
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

#[expect(clippy::too_many_arguments)]
#[allow(unused)]
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
    timeout: Option<u32>,
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
            .map_err(|_| ConfigError::FrequencyOutOfRange)?;

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

        cfg_if::cfg_if! {
            if #[cfg(i2c_master_has_bus_timeout_enable)] {
                register_block.to().write(|w| {
                    w.time_out_en().bit(timeout.is_some());
                    w.time_out_value().bits(timeout.unwrap_or(1) as _)
                });
            } else {
                register_block
                    .to()
                    .write(|w| w.time_out().bits(timeout.unwrap_or(1)));
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
    pub fn regs(&self) -> &RegisterBlock {
        unsafe { &*self.register_block }
    }

    /// Listen for the given interrupts
    fn enable_listen(&self, interrupts: EnumSet<Event>, enable: bool) {
        let reg_block = self.regs();

        reg_block.int_ena().modify(|_, w| {
            for interrupt in interrupts {
                match interrupt {
                    Event::EndDetect => w.end_detect().bit(enable),
                    Event::TxComplete => w.trans_complete().bit(enable),
                    #[cfg(i2c_master_has_tx_fifo_watermark)]
                    Event::TxFifoWatermark => w.txfifo_wm().bit(enable),
                };
            }
            w
        });
    }

    fn interrupts(&self) -> EnumSet<Event> {
        let mut res = EnumSet::new();
        let reg_block = self.regs();

        let ints = reg_block.int_raw().read();

        if ints.end_detect().bit_is_set() {
            res.insert(Event::EndDetect);
        }
        if ints.trans_complete().bit_is_set() {
            res.insert(Event::TxComplete);
        }
        #[cfg(i2c_master_has_tx_fifo_watermark)]
        if ints.txfifo_wm().bit_is_set() {
            res.insert(Event::TxFifoWatermark);
        }

        res
    }

    fn clear_interrupts(&self, interrupts: EnumSet<Event>) {
        let reg_block = self.regs();

        reg_block.int_clr().write(|w| {
            for interrupt in interrupts {
                match interrupt {
                    Event::EndDetect => w.end_detect().clear_bit_by_one(),
                    Event::TxComplete => w.trans_complete().clear_bit_by_one(),
                    #[cfg(i2c_master_has_tx_fifo_watermark)]
                    Event::TxFifoWatermark => w.txfifo_wm().clear_bit_by_one(),
                };
            }
            w
        });
    }
}

impl PartialEq for Info {
    fn eq(&self, other: &Self) -> bool {
        core::ptr::eq(self.register_block, other.register_block)
    }
}

unsafe impl Sync for Info {}

#[derive(Clone, Copy)]
enum Deadline {
    None,
    Fixed(Instant),
    PerByte(Duration),
}

impl Deadline {
    fn start(self, data_len: usize) -> Option<Instant> {
        match self {
            Deadline::None => None,
            Deadline::Fixed(deadline) => Some(deadline),
            Deadline::PerByte(duration) => Some(Instant::now() + duration * data_len as u32),
        }
    }
}

#[allow(dead_code)] // Some versions don't need `state`
#[derive(Clone, Copy)]
struct Driver<'a> {
    info: &'a Info,
    state: &'a State,
    config: &'a DriverConfig,
}

impl Driver<'_> {
    fn regs(&self) -> &RegisterBlock {
        self.info.regs()
    }

    fn connect_pin(
        pin: crate::gpio::interconnect::OutputSignal<'_>,
        input: InputSignal,
        output: OutputSignal,
        guard: &mut PinGuard,
    ) {
        // avoid the pin going low during configuration
        pin.set_output_high(true);

        pin.apply_output_config(
            &OutputConfig::default()
                .with_drive_mode(DriveMode::OpenDrain)
                .with_pull(Pull::Up),
        );
        pin.set_output_enable(true);
        pin.set_input_enable(true);

        input.connect_to(&pin);

        *guard = interconnect::OutputSignal::connect_with_guard(pin, output);
    }

    fn init_master(&self) {
        self.regs().ctr().write(|w| {
            // Set I2C controller to master mode
            w.ms_mode().set_bit();
            // Use open drain output for SDA and SCL
            w.sda_force_out().set_bit();
            w.scl_force_out().set_bit();
            // Use Most Significant Bit first for sending and receiving data
            w.tx_lsb_first().clear_bit();
            w.rx_lsb_first().clear_bit();

            #[cfg(i2c_master_has_arbitration_en)]
            w.arbitration_en().clear_bit();

            #[cfg(esp32s2)]
            w.ref_always_on().set_bit();

            // Ensure that clock is enabled
            w.clk_en().set_bit()
        });
    }

    /// Configures the I2C peripheral with the specified frequency, clocks, and
    /// optional timeout.
    fn setup(&self, config: &Config) -> Result<(), ConfigError> {
        self.init_master();

        // Configure filter
        // FIXME if we ever change this we need to adapt `set_frequency` for ESP32
        set_filter(self.regs(), Some(7), Some(7));

        // Configure frequency
        self.set_frequency(config)?;

        // Configure additional timeouts
        #[cfg(i2c_master_has_fsm_timeouts)]
        {
            self.regs()
                .scl_st_time_out()
                .write(|w| unsafe { w.scl_st_to().bits(config.scl_st_timeout.value()) });
            self.regs()
                .scl_main_st_time_out()
                .write(|w| unsafe { w.scl_main_st_to().bits(config.scl_main_st_timeout.value()) });
        }

        self.update_registers();

        Ok(())
    }

    fn do_fsm_reset(&self) {
        cfg_if::cfg_if! {
            if #[cfg(i2c_master_has_reliable_fsm_reset)] {
                // Device has a working FSM reset mechanism
                self.regs().ctr().modify(|_, w| w.fsm_rst().set_bit());
            } else {
                // Even though C2 and C3 have a FSM reset bit, esp-idf does not
                // define SOC_I2C_SUPPORT_HW_FSM_RST for them, so include them in the fallback impl.

                crate::system::PeripheralClockControl::reset(self.info.peripheral);

                // Restore configuration. This operation has succeeded once, so we can
                // assume that the config is valid and we can ignore the result.
                self.setup(&self.config.config).ok();
            }
        }
    }

    /// Resets the I2C controller (FIFO + FSM + command list)
    // This function implements esp-idf's `s_i2c_hw_fsm_reset`
    // https://github.com/espressif/esp-idf/blob/27d68f57e6bdd3842cd263585c2c352698a9eda2/components/esp_driver_i2c/i2c_master.c#L115
    //
    // Make sure you don't call this function in the middle of a transaction. If the
    // first command in the command list is not a START, the hardware will hang
    // with no timeouts.
    fn reset_fsm(&self, clear_bus: bool) {
        if clear_bus {
            self.clear_bus_blocking(true);
        } else {
            self.do_fsm_reset();
        }
    }

    fn bus_busy(&self) -> bool {
        self.regs().sr().read().bus_busy().bit_is_set()
    }

    fn ensure_idle_blocking(&self) {
        if self.bus_busy() {
            // If the bus is busy, we need to clear it.
            self.clear_bus_blocking(false);
        }
    }

    async fn ensure_idle(&self) {
        if self.bus_busy() {
            // If the bus is busy, we need to clear it.
            self.clear_bus().await;
        }
    }

    fn reset_before_transmission(&self) {
        // Clear all I2C interrupts
        self.clear_all_interrupts();

        // Reset fifo
        self.reset_fifo();

        // Reset the command list
        self.reset_command_list();
    }

    /// Implements s_i2c_master_clear_bus
    ///
    /// If a transaction ended incorrectly for some reason, the slave may drive
    /// SDA indefinitely. This function forces the slave to release the
    /// bus by sending 9 clock pulses.
    fn clear_bus_blocking(&self, reset_fsm: bool) {
        let mut future = ClearBusFuture::new(*self, reset_fsm);
        let start = Instant::now();
        while future.poll_completion().is_pending() {
            if start.elapsed() > CLEAR_BUS_TIMEOUT_MS {
                break;
            }
        }
    }

    async fn clear_bus(&self) {
        let clear_bus = ClearBusFuture::new(*self, true);
        let start = Instant::now();

        embassy_futures::select::select(clear_bus, async {
            while start.elapsed() < CLEAR_BUS_TIMEOUT_MS {
                embassy_futures::yield_now().await;
            }
        })
        .await;
    }

    /// Resets the I2C peripheral's command registers.
    fn reset_command_list(&self) {
        for cmd in self.regs().comd_iter() {
            cmd.reset();
        }
    }

    #[cfg(esp32)]
    /// Sets the frequency of the I2C interface by calculating and applying the
    /// associated timings - corresponds to i2c_ll_cal_bus_clk and
    /// i2c_ll_set_bus_timing in ESP-IDF
    fn set_frequency(&self, clock_config: &Config) -> Result<(), ConfigError> {
        let timeout = clock_config.timeout;

        let clocks = Clocks::get();
        let source_clk = clocks.i2c_clock.as_hz();
        let bus_freq = clock_config.frequency.as_hz();

        let half_cycle: u32 = source_clk / bus_freq / 2;
        let scl_low = half_cycle;
        let scl_high = half_cycle;
        let sda_hold = half_cycle / 2;
        let sda_sample = scl_high / 2;
        let setup = half_cycle;
        let hold = half_cycle;

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
            self.regs(),
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
            timeout.apb_cycles(half_cycle)?,
        )?;

        Ok(())
    }

    #[cfg(esp32s2)]
    /// Sets the frequency of the I2C interface by calculating and applying the
    /// associated timings - corresponds to i2c_ll_cal_bus_clk and
    /// i2c_ll_set_bus_timing in ESP-IDF
    fn set_frequency(&self, clock_config: &Config) -> Result<(), ConfigError> {
        let timeout = clock_config.timeout;

        let clocks = Clocks::get();
        let source_clk = clocks.apb_clock.as_hz();
        let bus_freq = clock_config.frequency.as_hz();

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

        configure_clock(
            self.regs(),
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
            timeout.apb_cycles(half_cycle)?,
        )?;

        Ok(())
    }

    #[cfg(any(esp32c2, esp32c3, esp32c6, esp32h2, esp32s3))]
    /// Sets the frequency of the I2C interface by calculating and applying the
    /// associated timings - corresponds to i2c_ll_cal_bus_clk and
    /// i2c_ll_set_bus_timing in ESP-IDF
    fn set_frequency(&self, clock_config: &Config) -> Result<(), ConfigError> {
        let timeout = clock_config.timeout;

        let clocks = Clocks::get();
        let source_clk = clocks.xtal_clock.as_hz();
        let bus_freq = clock_config.frequency.as_hz();

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

        configure_clock(
            self.regs(),
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
            timeout.apb_cycles(half_cycle)?,
        )?;

        Ok(())
    }

    /// Configures the I2C peripheral for a write operation.
    /// - `addr` is the address of the slave device.
    /// - `bytes` is the data two be sent.
    /// - `start` indicates whether the operation should start by a START condition and sending the
    ///   address.
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
        // If start is true we need to send the address, too, which takes up a data
        // byte.
        let max_len = if start {
            I2C_CHUNK_SIZE
        } else {
            I2C_CHUNK_SIZE + 1
        };
        if bytes.len() > max_len {
            return Err(Error::FifoExceeded);
        }

        if start {
            add_cmd(cmd_iterator, Command::Start)?;
        }

        let write_len = if start { bytes.len() + 1 } else { bytes.len() };
        // don't issue write if there is no data to write
        if write_len > 0 {
            if cfg!(any(esp32, esp32s2)) {
                // try to place END at the same index
                if write_len < 2 {
                    add_cmd(
                        cmd_iterator,
                        Command::Write {
                            ack_exp: Ack::Ack,
                            ack_check_en: true,
                            length: write_len as u8,
                        },
                    )?;
                } else if start {
                    add_cmd(
                        cmd_iterator,
                        Command::Write {
                            ack_exp: Ack::Ack,
                            ack_check_en: true,
                            length: (write_len as u8) - 1,
                        },
                    )?;
                    add_cmd(
                        cmd_iterator,
                        Command::Write {
                            ack_exp: Ack::Ack,
                            ack_check_en: true,
                            length: 1,
                        },
                    )?;
                } else {
                    add_cmd(
                        cmd_iterator,
                        Command::Write {
                            ack_exp: Ack::Ack,
                            ack_check_en: true,
                            length: (write_len as u8) - 2,
                        },
                    )?;
                    add_cmd(
                        cmd_iterator,
                        Command::Write {
                            ack_exp: Ack::Ack,
                            ack_check_en: true,
                            length: 1,
                        },
                    )?;
                    add_cmd(
                        cmd_iterator,
                        Command::Write {
                            ack_exp: Ack::Ack,
                            ack_check_en: true,
                            length: 1,
                        },
                    )?;
                }
            } else {
                add_cmd(
                    cmd_iterator,
                    Command::Write {
                        ack_exp: Ack::Ack,
                        ack_check_en: true,
                        length: write_len as u8,
                    },
                )?;
            }
        }

        if start {
            // Load address and R/W bit into FIFO
            match addr {
                I2cAddress::SevenBit(addr) => {
                    write_fifo(self.regs(), (addr << 1) | OperationType::Write as u8);
                }
            }
        }
        for b in bytes {
            write_fifo(self.regs(), *b);
        }

        Ok(())
    }

    /// Configures the I2C peripheral for a read operation.
    /// - `addr` is the address of the slave device.
    /// - `buffer` is the buffer to store the read data.
    /// - `start` indicates whether the operation should start by a START condition and sending the
    ///   address.
    /// - `will_continue` indicates whether there is another read operation following this one and
    ///   we should not nack the last byte.
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
            (I2C_CHUNK_SIZE + 1, buffer.len())
        } else {
            (I2C_CHUNK_SIZE, buffer.len() - 1)
        };
        if buffer.len() > max_len {
            return Err(Error::FifoExceeded);
        }

        if start {
            add_cmd(cmd_iterator, Command::Start)?;
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
            if cfg!(any(esp32, esp32s2)) {
                // try to place END at the same index
                if initial_len < 2 || start {
                    add_cmd(
                        cmd_iterator,
                        Command::Read {
                            ack_value: Ack::Ack,
                            length: initial_len as u8,
                        },
                    )?;
                } else if !will_continue {
                    add_cmd(
                        cmd_iterator,
                        Command::Read {
                            ack_value: Ack::Ack,
                            length: (initial_len as u8) - 1,
                        },
                    )?;
                    add_cmd(
                        cmd_iterator,
                        Command::Read {
                            ack_value: Ack::Ack,
                            length: 1,
                        },
                    )?;
                } else {
                    add_cmd(
                        cmd_iterator,
                        Command::Read {
                            ack_value: Ack::Ack,
                            length: (initial_len as u8) - 2,
                        },
                    )?;
                    add_cmd(
                        cmd_iterator,
                        Command::Read {
                            ack_value: Ack::Ack,
                            length: 1,
                        },
                    )?;
                    add_cmd(
                        cmd_iterator,
                        Command::Read {
                            ack_value: Ack::Ack,
                            length: 1,
                        },
                    )?;
                }
            } else {
                add_cmd(
                    cmd_iterator,
                    Command::Read {
                        ack_value: Ack::Ack,
                        length: initial_len as u8,
                    },
                )?;
            }
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

        self.update_registers();

        if start {
            // Load address and R/W bit into FIFO
            match addr {
                I2cAddress::SevenBit(addr) => {
                    write_fifo(self.regs(), (addr << 1) | OperationType::Read as u8);
                }
            }
        }
        Ok(())
    }

    /// Reads from RX FIFO into the given buffer.
    fn read_all_from_fifo(&self, buffer: &mut [u8]) -> Result<(), Error> {
        if self.regs().sr().read().rxfifo_cnt().bits() < buffer.len() as u8 {
            return Err(Error::ExecutionIncomplete);
        }

        // Read bytes from FIFO
        for byte in buffer.iter_mut() {
            *byte = read_fifo(self.regs());
        }

        // The RX FIFO should be empty now. If it is not, it means we queued up reading
        // more data than we read, which is an error.
        debug_assert!(self.regs().sr().read().rxfifo_cnt().bits() == 0);

        Ok(())
    }

    /// Clears all pending interrupts for the I2C peripheral.
    fn clear_all_interrupts(&self) {
        self.regs()
            .int_clr()
            .write(|w| unsafe { w.bits(property!("i2c_master.ll_intr_mask")) });
    }

    async fn wait_for_completion(&self, deadline: Option<Instant>) -> Result<(), Error> {
        I2cFuture::new(Event::TxComplete | Event::EndDetect, *self, deadline).await?;
        self.check_all_commands_done(deadline).await
    }

    /// Waits for the completion of an I2C transaction.
    fn wait_for_completion_blocking(&self, deadline: Option<Instant>) -> Result<(), Error> {
        let mut future =
            I2cFuture::new_blocking(Event::TxComplete | Event::EndDetect, *self, deadline);
        loop {
            if let Poll::Ready(result) = future.poll_completion() {
                result?;
                return self.check_all_commands_done_blocking(deadline);
            }
        }
    }

    fn all_commands_done(&self, deadline: Option<Instant>) -> Result<bool, Error> {
        // NOTE: on esp32 executing the end command generates the end_detect interrupt
        //       but does not seem to clear the done bit! So we don't check the done
        //       status of an end command
        let now = if deadline.is_some() {
            Instant::now()
        } else {
            Instant::EPOCH
        };

        for cmd_reg in self.regs().comd_iter() {
            let cmd = cmd_reg.read();

            // if there is a valid command which is not END, check if it's marked as done
            if cmd.bits() != 0x0 && !cmd.opcode().is_end() && !cmd.command_done().bit_is_set() {
                // Let's retry
                if let Some(deadline) = deadline
                    && now > deadline
                {
                    return Err(Error::ExecutionIncomplete);
                }

                return Ok(false);
            }

            // once we hit END or STOP we can break the loop
            if cmd.opcode().is_end() {
                break;
            }
            if cmd.opcode().is_stop() {
                #[cfg(esp32)]
                // wait for STOP - apparently on ESP32 we otherwise miss the ACK error for an
                // empty write
                if self.regs().sr().read().scl_state_last() == 6 {
                    self.check_errors()?;
                } else {
                    continue;
                }
                break;
            }
        }
        Ok(true)
    }

    /// Checks whether all I2C commands have completed execution.
    fn check_all_commands_done_blocking(&self, deadline: Option<Instant>) -> Result<(), Error> {
        // loop until commands are actually done
        while !self.all_commands_done(deadline)? {}
        self.check_errors()?;

        Ok(())
    }

    /// Checks whether all I2C commands have completed execution.
    async fn check_all_commands_done(&self, deadline: Option<Instant>) -> Result<(), Error> {
        // loop until commands are actually done
        while !self.all_commands_done(deadline)? {
            embassy_futures::yield_now().await;
        }
        self.check_errors()?;

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
        let r = self.regs().int_raw().read();

        // Handle error cases
        if r.nack().bit_is_set() {
            return Err(Error::AcknowledgeCheckFailed(estimate_ack_failed_reason(
                self.regs(),
            )));
        }
        if r.arbitration_lost().bit_is_set() {
            return Err(Error::ArbitrationLost);
        }

        #[cfg(not(esp32))]
        if r.trans_complete().bit_is_set() && self.regs().sr().read().resp_rec().bit_is_clear() {
            return Err(Error::AcknowledgeCheckFailed(
                AcknowledgeCheckFailedReason::Data,
            ));
        }

        #[cfg(i2c_master_has_fsm_timeouts)]
        {
            if r.scl_st_to().bit_is_set() {
                return Err(Error::Timeout);
            }
            if r.scl_main_st_to().bit_is_set() {
                return Err(Error::Timeout);
            }
        }
        if r.time_out().bit_is_set() {
            return Err(Error::Timeout);
        }

        Ok(())
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
    fn update_registers(&self) {
        // Ensure that the configuration of the peripheral is correctly propagated
        // (only necessary for C2, C3, C6, H2 and S3 variant)
        #[cfg(i2c_master_has_conf_update)]
        self.regs().ctr().modify(|_, w| w.conf_upgate().set_bit());
    }

    /// Starts an I2C transmission.
    fn start_transmission(&self) {
        // Start transmission
        self.regs().ctr().modify(|_, w| w.trans_start().set_bit());
    }

    /// Resets the transmit and receive FIFO buffers
    #[cfg(not(esp32))]
    fn reset_fifo(&self) {
        // First, reset the fifo buffers
        self.regs().fifo_conf().write(|w| unsafe {
            w.tx_fifo_rst().set_bit();
            w.rx_fifo_rst().set_bit();
            w.nonfifo_en().clear_bit();
            w.fifo_prt_en().set_bit();
            w.rxfifo_wm_thrhd().bits(1);
            w.txfifo_wm_thrhd().bits(8)
        });

        self.regs().fifo_conf().modify(|_, w| {
            w.tx_fifo_rst().clear_bit();
            w.rx_fifo_rst().clear_bit()
        });

        self.regs().int_clr().write(|w| {
            w.rxfifo_wm().clear_bit_by_one();
            w.txfifo_wm().clear_bit_by_one()
        });

        self.update_registers();
    }

    /// Resets the transmit and receive FIFO buffers
    #[cfg(esp32)]
    fn reset_fifo(&self) {
        // First, reset the fifo buffers
        self.regs().fifo_conf().write(|w| unsafe {
            w.tx_fifo_rst().set_bit();
            w.rx_fifo_rst().set_bit();
            w.nonfifo_en().clear_bit();
            w.nonfifo_rx_thres().bits(1);
            w.nonfifo_tx_thres().bits(32)
        });

        self.regs().fifo_conf().modify(|_, w| {
            w.tx_fifo_rst().clear_bit();
            w.rx_fifo_rst().clear_bit()
        });

        self.regs()
            .int_clr()
            .write(|w| w.rxfifo_full().clear_bit_by_one());
    }

    fn start_write_operation(
        &self,
        address: I2cAddress,
        bytes: &[u8],
        start: bool,
        stop: bool,
        deadline: Deadline,
    ) -> Result<Option<Instant>, Error> {
        let cmd_iterator = &mut self.regs().comd_iter();

        self.setup_write(address, bytes, start, cmd_iterator)?;

        if stop {
            add_cmd(cmd_iterator, Command::Stop)?;
        } else {
            add_cmd(cmd_iterator, Command::End)?;
        }
        self.start_transmission();
        let deadline = deadline.start(bytes.len() + start as usize);

        Ok(deadline)
    }

    /// Executes an I2C read operation.
    /// - `addr` is the address of the slave device.
    /// - `buffer` is the buffer to store the read data.
    /// - `start` indicates whether the operation should start by a START condition and sending the
    ///   address.
    /// - `stop` indicates whether the operation should end with a STOP condition.
    /// - `will_continue` indicates whether there is another read operation following this one and
    ///   we should not nack the last byte.
    /// - `cmd_iterator` is an iterator over the command registers.
    fn start_read_operation(
        &self,
        address: I2cAddress,
        buffer: &mut [u8],
        start: bool,
        will_continue: bool,
        stop: bool,
        deadline: Deadline,
    ) -> Result<Option<Instant>, Error> {
        // We don't support single I2C reads larger than the FIFO. This should be
        // enforced by `VariableChunkIterMut` used in `Driver::read` and
        // `Driver::read_async`.
        debug_assert!(buffer.len() <= I2C_FIFO_SIZE);

        let cmd_iterator = &mut self.regs().comd_iter();

        self.setup_read(address, buffer, start, will_continue, cmd_iterator)?;

        if stop {
            add_cmd(cmd_iterator, Command::Stop)?;
        } else {
            add_cmd(cmd_iterator, Command::End)?;
        }
        self.start_transmission();
        let deadline = deadline.start(buffer.len() + start as usize);

        Ok(deadline)
    }

    /// Executes an I2C write operation.
    /// - `addr` is the address of the slave device.
    /// - `bytes` is the data two be sent.
    /// - `start` indicates whether the operation should start by a START condition and sending the
    ///   address.
    /// - `stop` indicates whether the operation should end with a STOP condition.
    /// - `cmd_iterator` is an iterator over the command registers.
    fn write_operation_blocking(
        &self,
        address: I2cAddress,
        bytes: &[u8],
        start: bool,
        stop: bool,
        deadline: Deadline,
    ) -> Result<(), Error> {
        address.validate()?;
        self.reset_before_transmission();

        // Short circuit for zero length writes without start or end as that would be an
        // invalid operation write lengths in the TRM (at least for ESP32-S3) are 1-255
        if bytes.is_empty() && !start && !stop {
            return Ok(());
        }

        let deadline = self.start_write_operation(address, bytes, start, stop, deadline)?;
        self.wait_for_completion_blocking(deadline)?;

        Ok(())
    }

    /// Executes an I2C read operation.
    /// - `addr` is the address of the slave device.
    /// - `buffer` is the buffer to store the read data.
    /// - `start` indicates whether the operation should start by a START condition and sending the
    ///   address.
    /// - `stop` indicates whether the operation should end with a STOP condition.
    /// - `will_continue` indicates whether there is another read operation following this one and
    ///   we should not nack the last byte.
    /// - `cmd_iterator` is an iterator over the command registers.
    fn read_operation_blocking(
        &self,
        address: I2cAddress,
        buffer: &mut [u8],
        start: bool,
        stop: bool,
        will_continue: bool,
        deadline: Deadline,
    ) -> Result<(), Error> {
        address.validate()?;
        self.reset_before_transmission();

        // Short circuit for zero length reads as that would be an invalid operation
        // read lengths in the TRM (at least for ESP32-S3) are 1-255
        if buffer.is_empty() {
            return Ok(());
        }

        let deadline =
            self.start_read_operation(address, buffer, start, will_continue, stop, deadline)?;
        self.wait_for_completion_blocking(deadline)?;
        self.read_all_from_fifo(buffer)?;

        Ok(())
    }

    /// Executes an async I2C write operation.
    /// - `addr` is the address of the slave device.
    /// - `bytes` is the data two be sent.
    /// - `start` indicates whether the operation should start by a START condition and sending the
    ///   address.
    /// - `stop` indicates whether the operation should end with a STOP condition.
    /// - `cmd_iterator` is an iterator over the command registers.
    async fn write_operation(
        &self,
        address: I2cAddress,
        bytes: &[u8],
        start: bool,
        stop: bool,
        deadline: Deadline,
    ) -> Result<(), Error> {
        address.validate()?;
        self.reset_before_transmission();

        // Short circuit for zero length writes without start or end as that would be an
        // invalid operation write lengths in the TRM (at least for ESP32-S3) are 1-255
        if bytes.is_empty() && !start && !stop {
            return Ok(());
        }

        let deadline = self.start_write_operation(address, bytes, start, stop, deadline)?;
        self.wait_for_completion(deadline).await?;

        Ok(())
    }

    /// Executes an async I2C read operation.
    /// - `addr` is the address of the slave device.
    /// - `buffer` is the buffer to store the read data.
    /// - `start` indicates whether the operation should start by a START condition and sending the
    ///   address.
    /// - `stop` indicates whether the operation should end with a STOP condition.
    /// - `will_continue` indicates whether there is another read operation following this one and
    ///   we should not nack the last byte.
    /// - `cmd_iterator` is an iterator over the command registers.
    async fn read_operation(
        &self,
        address: I2cAddress,
        buffer: &mut [u8],
        start: bool,
        stop: bool,
        will_continue: bool,
        deadline: Deadline,
    ) -> Result<(), Error> {
        address.validate()?;
        self.reset_before_transmission();

        // Short circuit for zero length reads as that would be an invalid operation
        // read lengths in the TRM (at least for ESP32-S3) are 1-255
        if buffer.is_empty() {
            return Ok(());
        }

        let deadline =
            self.start_read_operation(address, buffer, start, will_continue, stop, deadline)?;
        self.wait_for_completion(deadline).await?;
        self.read_all_from_fifo(buffer)?;

        Ok(())
    }

    fn read_blocking(
        &self,
        address: I2cAddress,
        buffer: &mut [u8],
        start: bool,
        stop: bool,
        will_continue: bool,
        deadline: Deadline,
    ) -> Result<(), Error> {
        let chunk_count = VariableChunkIterMut::new(buffer).count();
        for (idx, chunk) in VariableChunkIterMut::new(buffer).enumerate() {
            self.read_operation_blocking(
                address,
                chunk,
                start && idx == 0,
                stop && idx == chunk_count - 1,
                will_continue || idx < chunk_count - 1,
                deadline,
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
        deadline: Deadline,
    ) -> Result<(), Error> {
        if buffer.is_empty() {
            return self.write_operation_blocking(address, &[], start, stop, deadline);
        }

        let chunk_count = VariableChunkIter::new(buffer).count();
        for (idx, chunk) in VariableChunkIter::new(buffer).enumerate() {
            self.write_operation_blocking(
                address,
                chunk,
                start && idx == 0,
                stop && idx == chunk_count - 1,
                deadline,
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
        deadline: Deadline,
    ) -> Result<(), Error> {
        let chunk_count = VariableChunkIterMut::new(buffer).count();
        for (idx, chunk) in VariableChunkIterMut::new(buffer).enumerate() {
            self.read_operation(
                address,
                chunk,
                start && idx == 0,
                stop && idx == chunk_count - 1,
                will_continue || idx < chunk_count - 1,
                deadline,
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
        deadline: Deadline,
    ) -> Result<(), Error> {
        if buffer.is_empty() {
            return self
                .write_operation(address, &[], start, stop, deadline)
                .await;
        }

        let chunk_count = VariableChunkIter::new(buffer).count();
        for (idx, chunk) in VariableChunkIter::new(buffer).enumerate() {
            self.write_operation(
                address,
                chunk,
                start && idx == 0,
                stop && idx == chunk_count - 1,
                deadline,
            )
            .await?;
        }

        Ok(())
    }

    fn transaction_impl<'a>(
        &self,
        address: I2cAddress,
        operations: impl Iterator<Item = Operation<'a>>,
    ) -> Result<(), Error> {
        address.validate()?;
        self.ensure_idle_blocking();

        let mut deadline = Deadline::None;

        if let SoftwareTimeout::Transaction(timeout) = self.config.config.software_timeout {
            deadline = Deadline::Fixed(Instant::now() + timeout);
        }

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
                    if let SoftwareTimeout::PerByte(timeout) = self.config.config.software_timeout {
                        deadline = Deadline::PerByte(timeout);
                    }
                    self.write_blocking(
                        address,
                        buffer,
                        !matches!(last_op, Some(OpKind::Write)),
                        next_op.is_none(),
                        deadline,
                    )?;
                }
                Operation::Read(buffer) => {
                    if let SoftwareTimeout::PerByte(timeout) = self.config.config.software_timeout {
                        deadline = Deadline::PerByte(timeout);
                    }
                    // execute a read operation:
                    // - issue START/RSTART if op is different from previous
                    // - issue STOP if op is the last one
                    // - will_continue is true if there is another read operation next
                    self.read_blocking(
                        address,
                        buffer,
                        !matches!(last_op, Some(OpKind::Read)),
                        next_op.is_none(),
                        matches!(next_op, Some(OpKind::Read)),
                        deadline,
                    )?;
                }
            }

            last_op = Some(kind);
        }

        Ok(())
    }

    async fn transaction_impl_async<'a>(
        &self,
        address: I2cAddress,
        operations: impl Iterator<Item = Operation<'a>>,
    ) -> Result<(), Error> {
        address.validate()?;
        self.ensure_idle().await;

        let mut deadline = Deadline::None;

        if let SoftwareTimeout::Transaction(timeout) = self.config.config.software_timeout {
            deadline = Deadline::Fixed(Instant::now() + timeout);
        }

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
                    if let SoftwareTimeout::PerByte(timeout) = self.config.config.software_timeout {
                        deadline = Deadline::PerByte(timeout);
                    }
                    // execute a write operation:
                    // - issue START/RSTART if op is different from previous
                    // - issue STOP if op is the last one
                    self.write(
                        address,
                        buffer,
                        !matches!(last_op, Some(OpKind::Write)),
                        next_op.is_none(),
                        deadline,
                    )
                    .await?;
                }
                Operation::Read(buffer) => {
                    if let SoftwareTimeout::PerByte(timeout) = self.config.config.software_timeout {
                        deadline = Deadline::PerByte(timeout);
                    }
                    // execute a read operation:
                    // - issue START/RSTART if op is different from previous
                    // - issue STOP if op is the last one
                    // - will_continue is true if there is another read operation next
                    self.read(
                        address,
                        buffer,
                        !matches!(last_op, Some(OpKind::Read)),
                        next_op.is_none(),
                        matches!(next_op, Some(OpKind::Read)),
                        deadline,
                    )
                    .await?;
                }
            }

            last_op = Some(kind);
        }

        Ok(())
    }
}

/// Chunks a slice by I2C_CHUNK_SIZE in a way to avoid the last chunk being
/// sized smaller than 2
struct VariableChunkIterMut<'a, T> {
    buffer: &'a mut [T],
}

impl<'a, T> VariableChunkIterMut<'a, T> {
    fn new(buffer: &'a mut [T]) -> Self {
        Self { buffer }
    }
}

impl<'a, T> Iterator for VariableChunkIterMut<'a, T> {
    type Item = &'a mut [T];

    fn next(&mut self) -> Option<Self::Item> {
        if self.buffer.is_empty() {
            return None;
        }

        let s = calculate_chunk_size(self.buffer.len());
        let (chunk, remaining) = core::mem::take(&mut self.buffer).split_at_mut(s);
        self.buffer = remaining;
        Some(chunk)
    }
}

/// Chunks a slice by I2C_CHUNK_SIZE in a way to avoid the last chunk being
/// sized smaller than 2
struct VariableChunkIter<'a, T> {
    buffer: &'a [T],
}

impl<'a, T> VariableChunkIter<'a, T> {
    fn new(buffer: &'a [T]) -> Self {
        Self { buffer }
    }
}

impl<'a, T> Iterator for VariableChunkIter<'a, T> {
    type Item = &'a [T];

    fn next(&mut self) -> Option<Self::Item> {
        if self.buffer.is_empty() {
            return None;
        }

        let s = calculate_chunk_size(self.buffer.len());
        let (chunk, remaining) = core::mem::take(&mut self.buffer).split_at(s);
        self.buffer = remaining;
        Some(chunk)
    }
}

fn calculate_chunk_size(remaining: usize) -> usize {
    if remaining <= I2C_CHUNK_SIZE {
        remaining
    } else if remaining > I2C_CHUNK_SIZE + 2 {
        I2C_CHUNK_SIZE
    } else {
        I2C_CHUNK_SIZE - 2
    }
}

#[cfg(i2c_master_has_hw_bus_clear)]
mod bus_clear {
    use esp_rom_sys::rom::ets_delay_us;

    use super::*;

    pub struct ClearBusFuture<'a> {
        driver: Driver<'a>,
    }

    impl<'a> ClearBusFuture<'a> {
        // Number of SCL pulses to clear the bus
        const BUS_CLEAR_BITS: u8 = 9;
        const DELAY_US: u32 = 5; // 5us -> 100kHz

        pub fn new(driver: Driver<'a>, reset_fsm: bool) -> Self {
            // If we have a HW implementation, reset FSM to make sure it's not trying to transmit
            // while we clear the bus.
            if reset_fsm {
                // Resetting the FSM may still generate a short SCL pulse, but I don't know how to
                // work around it - just waiting doesn't solve anything if the hardware is running.
                driver.do_fsm_reset();
            }

            let mut this = Self { driver };

            // Prevent SCL from going low immediately after FSM reset/previous operation has set
            // it high
            ets_delay_us(Self::DELAY_US);

            this.configure(Self::BUS_CLEAR_BITS);
            this
        }

        fn configure(&mut self, bits: u8) {
            self.driver.regs().scl_sp_conf().modify(|_, w| {
                unsafe { w.scl_rst_slv_num().bits(bits) };
                w.scl_rst_slv_en().bit(bits > 0)
            });
            self.driver.update_registers();
        }

        fn is_done(&self) -> bool {
            self.driver
                .regs()
                .scl_sp_conf()
                .read()
                .scl_rst_slv_en()
                .bit_is_clear()
        }

        pub fn poll_completion(&mut self) -> Poll<()> {
            if self.is_done() {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        }
    }

    impl Drop for ClearBusFuture<'_> {
        fn drop(&mut self) {
            use crate::gpio::AnyPin;
            if !self.is_done() {
                self.configure(0);
            }

            // Generate a stop condition
            let sda = self
                .driver
                .config
                .sda_pin
                .pin_number()
                .map(|n| unsafe { AnyPin::steal(n) });
            let scl = self
                .driver
                .config
                .scl_pin
                .pin_number()
                .map(|n| unsafe { AnyPin::steal(n) });

            if let (Some(sda), Some(scl)) = (sda, scl) {
                // Prevent short SCL pulse right after HW clearing completes
                ets_delay_us(Self::DELAY_US);

                sda.set_output_high(true);
                scl.set_output_high(false);

                self.driver.info.scl_output.disconnect_from(&scl);
                self.driver.info.sda_output.disconnect_from(&sda);

                // Set SDA low - whatever state it was in, we need a low -> high transition.
                sda.set_output_high(false);
                ets_delay_us(Self::DELAY_US);

                // Set SCL high to prepare for STOP condition
                scl.set_output_high(true);
                ets_delay_us(Self::DELAY_US);

                // STOP
                sda.set_output_high(true);
                ets_delay_us(Self::DELAY_US);

                self.driver.info.sda_output.connect_to(&sda);
                self.driver.info.scl_output.connect_to(&scl);
            }

            // We don't care about errors during bus clearing
            self.driver.clear_all_interrupts();
        }
    }
}

#[cfg(not(i2c_master_has_hw_bus_clear))]
mod bus_clear {
    use super::*;
    use crate::gpio::AnyPin;

    /// State of the bus clearing operation.
    ///
    /// Pins are changed on the start of the state, and a wait is scheduled
    /// for the end of the state. At the end of the wait, the state is
    /// updated to the next state.
    enum State {
        Idle,
        SendStop,

        // Number of SCL pulses left to send, and the last SCL level.
        //
        // Our job is to send 9 high->low SCL transitions, followed by a STOP condition.
        SendClock(u8, bool),
    }

    pub struct ClearBusFuture<'a> {
        driver: Driver<'a>,
        wait: Instant,
        state: State,
        reset_fsm: bool,
        pins: Option<(AnyPin<'static>, AnyPin<'static>)>,
    }

    impl<'a> ClearBusFuture<'a> {
        // Number of SCL pulses to clear the bus (max 8 data bits sent by the device, + NACK)
        const BUS_CLEAR_BITS: u8 = 9;
        // use standard 100kHz data rate
        const SCL_DELAY: Duration = Duration::from_micros(5);

        pub fn new(driver: Driver<'a>, reset_fsm: bool) -> Self {
            let sda = driver
                .config
                .sda_pin
                .pin_number()
                .map(|n| unsafe { AnyPin::steal(n) });
            let scl = driver
                .config
                .scl_pin
                .pin_number()
                .map(|n| unsafe { AnyPin::steal(n) });

            let (Some(sda), Some(scl)) = (sda, scl) else {
                // If we don't have the pins, we can't clear the bus.
                if reset_fsm {
                    driver.do_fsm_reset();
                }
                return Self {
                    driver,
                    wait: Instant::now(),
                    state: State::Idle,
                    reset_fsm: false,
                    pins: None,
                };
            };

            sda.set_output_high(true);
            scl.set_output_high(false);

            driver.info.scl_output.disconnect_from(&scl);
            driver.info.sda_output.disconnect_from(&sda);

            // Starting from (9, false), becase:
            // - we start with SCL low
            // - a complete SCL cycle consists of a high period and a low period
            // - we decrement the remaining counter at the beginning of a cycle, which gives us 9
            //   complete SCL cycles.
            let state = State::SendClock(Self::BUS_CLEAR_BITS, false);

            Self {
                driver,
                wait: Instant::now() + Self::SCL_DELAY,
                state,
                reset_fsm,
                pins: Some((sda, scl)),
            }
        }
    }

    impl ClearBusFuture<'_> {
        pub fn poll_completion(&mut self) -> Poll<()> {
            let now = Instant::now();

            match self.state {
                State::Idle => {
                    if let Some((sda, _scl)) = self.pins.as_ref() {
                        // Pins are disconnected from the peripheral, we can't use `bus_busy`.
                        if !sda.is_input_high() {
                            return Poll::Pending;
                        }
                    }
                    return Poll::Ready(());
                }
                _ if now < self.wait => {
                    // Still waiting for the end of the SCL pulse
                    return Poll::Pending;
                }
                State::SendStop => {
                    if let Some((sda, _scl)) = self.pins.as_ref() {
                        sda.set_output_high(true); // STOP, SDA low -> high while SCL is HIGH
                    }
                    self.state = State::Idle;
                    return Poll::Pending;
                }
                State::SendClock(0, false) => {
                    if let Some((sda, scl)) = self.pins.as_ref() {
                        // Set up for STOP condition
                        sda.set_output_high(false);
                        scl.set_output_high(true);
                    }
                    self.state = State::SendStop;
                }
                State::SendClock(n, false) => {
                    if let Some((sda, scl)) = self.pins.as_ref() {
                        scl.set_output_high(true);
                        if sda.is_input_high() {
                            sda.set_output_high(false);
                            // The device has released SDA, we can move on to generating a STOP
                            // condition
                            self.wait = Instant::now() + Self::SCL_DELAY;
                            self.state = State::SendStop;
                            return Poll::Pending;
                        }
                    }
                    self.state = State::SendClock(n - 1, true);
                }
                State::SendClock(n, true) => {
                    if let Some((_sda, scl)) = self.pins.as_ref() {
                        scl.set_output_high(false);
                    }
                    self.state = State::SendClock(n, false);
                }
            }
            self.wait = Instant::now() + Self::SCL_DELAY;

            Poll::Pending
        }
    }

    impl Drop for ClearBusFuture<'_> {
        fn drop(&mut self) {
            if let Some((sda, scl)) = self.pins.take() {
                // Make sure _we_ release the bus.
                scl.set_output_high(true);
                sda.set_output_high(true);

                // If we don't have a HW implementation, reset the peripheral after clearing the
                // bus, but before we reconnect the pins in Drop. This should prevent glitches.
                if self.reset_fsm {
                    self.driver.do_fsm_reset();
                }

                self.driver.info.sda_output.connect_to(&sda);
                self.driver.info.scl_output.connect_to(&scl);

                // We don't care about errors during bus clearing. There shouldn't be any,
                // anyway.
                self.driver.clear_all_interrupts();
            }
        }
    }
}

use bus_clear::ClearBusFuture;

impl Future for ClearBusFuture<'_> {
    type Output = ();

    fn poll(mut self: Pin<&mut Self>, ctx: &mut Context<'_>) -> Poll<Self::Output> {
        let pending = self.poll_completion();
        if pending.is_pending() {
            ctx.waker().wake_by_ref();
        }
        pending
    }
}

/// Peripheral state for an I2C instance.
#[doc(hidden)]
#[non_exhaustive]
pub struct State {
    /// Waker for the asynchronous operations.
    pub waker: AtomicWaker,
}

/// A peripheral singleton compatible with the I2C master driver.
pub trait Instance: crate::private::Sealed + any::Degrade {
    #[doc(hidden)]
    /// Returns the peripheral data and state describing this instance.
    fn parts(&self) -> (&Info, &State);

    /// Returns the peripheral data describing this instance.
    #[doc(hidden)]
    #[inline(always)]
    fn info(&self) -> &Info {
        self.parts().0
    }

    /// Returns the peripheral state for this instance.
    #[doc(hidden)]
    #[inline(always)]
    fn state(&self) -> &State {
        self.parts().1
    }
}

/// Adds a command to the I2C command sequence.
///
/// Make sure the first command after a FSM reset is a START, otherwise
/// the hardware will hang with no timeouts.
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

fn read_fifo(register_block: &RegisterBlock) -> u8 {
    cfg_if::cfg_if! {
        if #[cfg(esp32s2)] {
            // Apparently the ESO can read just fine using DPORT,
            // so use this workaround on S2 only.
            let peri_offset = register_block as *const _ as usize - crate::peripherals::I2C0::ptr() as usize;
            let fifo_ptr = (property!("i2c_master.i2c0_data_register_ahb_address") + peri_offset) as *mut u32;
            unsafe { (fifo_ptr.read_volatile() & 0xff) as u8 }
        } else {
            register_block.data().read().fifo_rdata().bits()
        }
    }
}

fn write_fifo(register_block: &RegisterBlock, data: u8) {
    cfg_if::cfg_if! {
        if #[cfg(any(esp32, esp32s2))] {
            let peri_offset = register_block as *const _ as usize - crate::peripherals::I2C0::ptr() as usize;
            let fifo_ptr = (property!("i2c_master.i2c0_data_register_ahb_address") + peri_offset) as *mut u32;
            unsafe {
                fifo_ptr.write_volatile(data as u32);
            }
        } else {
            register_block
                .data()
                .write(|w| unsafe { w.fifo_rdata().bits(data) });
        }
    }
}

// Estimate the reason for an acknowledge check failure on a best effort basis.
// When in doubt it's better to return `Unknown` than to return a wrong reason.
fn estimate_ack_failed_reason(_register_block: &RegisterBlock) -> AcknowledgeCheckFailedReason {
    cfg_if::cfg_if! {
        if #[cfg(i2c_master_can_estimate_nack_reason)] {
            // this is based on observations rather than documented behavior
            if _register_block.fifo_st().read().txfifo_raddr().bits() <= 1 {
                AcknowledgeCheckFailedReason::Address
            } else {
                AcknowledgeCheckFailedReason::Data
            }
        } else {
            AcknowledgeCheckFailedReason::Unknown
        }
    }
}

for_each_i2c_master!(
    ($inst:ident, $peri:ident, $scl:ident, $sda:ident) => {
        impl Instance for crate::peripherals::$inst<'_> {
            fn parts(&self) -> (&Info, &State) {
                #[handler]
                #[ram]
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
                    scl_output: OutputSignal::$scl,
                    scl_input: InputSignal::$scl,
                    sda_output: OutputSignal::$sda,
                    sda_input: InputSignal::$sda,
                };
                (&PERIPHERAL, &STATE)
            }
        }
    };
);

crate::any_peripheral! {
    /// Any I2C peripheral.
    pub peripheral AnyI2c<'d> {
        #[cfg(i2c_master_i2c0)]
        I2c0(crate::peripherals::I2C0<'d>),
        #[cfg(i2c_master_i2c1)]
        I2c1(crate::peripherals::I2C1<'d>),
    }
}

impl Instance for AnyI2c<'_> {
    fn parts(&self) -> (&Info, &State) {
        any::delegate!(self, i2c => { i2c.parts() })
    }
}

impl AnyI2c<'_> {
    fn bind_peri_interrupt(&self, handler: interrupt::IsrCallback) {
        any::delegate!(self, i2c => { i2c.bind_peri_interrupt(handler) })
    }

    fn disable_peri_interrupt(&self) {
        any::delegate!(self, i2c => { i2c.disable_peri_interrupt() })
    }

    fn enable_peri_interrupt(&self, priority: crate::interrupt::Priority) {
        any::delegate!(self, i2c => { i2c.enable_peri_interrupt(priority) })
    }

    fn set_interrupt_handler(&self, handler: InterruptHandler) {
        self.disable_peri_interrupt();

        self.info().enable_listen(EnumSet::all(), false);
        self.info().clear_interrupts(EnumSet::all());

        self.bind_peri_interrupt(handler.handler());
        self.enable_peri_interrupt(handler.priority());
    }
}
