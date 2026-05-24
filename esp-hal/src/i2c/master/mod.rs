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

use enumset::{EnumSet, EnumSetType};

use crate::{
    Async,
    Blocking,
    DriverMode,
    asynch::AtomicWaker,
    gpio::{
        DriveMode,
        InputSignal,
        OutputConfig,
        OutputSignal,
        PinGuard,
        Pull,
        interconnect::{self, PeripheralInput, PeripheralOutput},
    },
    handler,
    interrupt::InterruptHandler,
    pac::i2c0::{COMD, RegisterBlock},
    private,
    ram,
    system::PeripheralGuard,
    time::{Duration, Instant, Rate},
};

mod eh;
mod low_level;

use low_level::Driver;
pub use low_level::{AnyI2c, Instance};

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

    fn bytes(self) -> usize {
        match self {
            I2cAddress::SevenBit(_) => 1,
        }
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
        /// Length of data (in bytes) to be read. The maximum length is
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

        let sda_pin = PinGuard::new_unconnected();
        let scl_pin = PinGuard::new_unconnected();

        let i2c = I2c {
            i2c: i2c.degrade(),
            phantom: PhantomData,
            guard,
            config: DriverConfig {
                config,
                sda_pin,
                scl_pin,
            },
        };

        // Make sure inputs are well-defined.
        let i2c = i2c.with_scl(crate::gpio::Level::High);
        let mut i2c = i2c.with_sda(crate::gpio::Level::High);

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

impl<'d> I2c<'d, Async> {
    /// Reconfigures the driver to operate in [`Blocking`] mode.
    ///
    /// See the [`Blocking`] documentation for an example on how to use this
    /// method.
    pub fn into_blocking(self) -> I2c<'d, Blocking> {
        self.i2c.disable_peri_interrupt_on_all_cores();

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

    #[procmacros::doc_replace]
    /// Connect a pin to the I2C SDA signal.
    ///
    /// If this function is called with a pin singleton (e.g. `GPIO2`), the pin will be configured
    /// to use the internal pull-up resistor. If this is undesired, call this function with a fully
    /// configured [`Flex`][crate::gpio::Flex] pin driver. Note that if you use `Flex`, the I2C
    /// driver will not change the pin's configuration in any way.
    ///
    /// This will replace previous pin assignments for this signal.
    ///
    /// ## Examples
    ///
    /// Basic usage
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::i2c::master::{Config, I2c};
    ///
    /// let i2c = I2c::new(peripherals.I2C0, Config::default())?.with_sda(peripherals.GPIO2);
    /// # {after_snippet}
    /// ```
    ///
    /// Using `Flex` to configure the pin
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::{
    ///     gpio::{DriveMode, Flex, OutputConfig},
    ///     i2c::master::{Config, I2c},
    /// };
    ///
    /// let mut sda = Flex::new(peripherals.GPIO2);
    ///
    /// // The default pullup setting is `Pull::None`.
    /// sda.apply_output_config(&OutputConfig::default().with_drive_mode(DriveMode::OpenDrain));
    /// sda.set_input_enable(true);
    /// sda.set_output_enable(true);
    /// // Initial pin state to avoid the pin to go low during peripheral configuration.
    /// sda.set_high();
    ///
    /// let i2c = I2c::new(peripherals.I2C0, Config::default())?.with_sda(sda);
    /// # {after_snippet}
    /// ```
    pub fn with_sda(mut self, sda: impl PeripheralInput<'d> + PeripheralOutput<'d>) -> Self {
        let info = self.driver().info;
        let input = info.sda_input;
        let output = info.sda_output;
        Driver::connect_pin(sda.into(), input, output, &mut self.config.sda_pin);

        self
    }

    #[procmacros::doc_replace]
    /// Connect a pin to the I2C SCL signal.
    ///
    /// If this function is called with a pin singleton (e.g. `GPIO2`), the pin will be configured
    /// to use the internal pull-up resistor. If this is undesired, call this function with a fully
    /// configured [`Flex`][crate::gpio::Flex] pin driver. Note that if you use `Flex`, the I2C
    /// driver will not change the pin's configuration in any way.
    ///
    /// This will replace previous pin assignments for this signal.
    ///
    /// ## Examples
    ///
    /// Basic usage
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::i2c::master::{Config, I2c};
    ///
    /// let i2c = I2c::new(peripherals.I2C0, Config::default())?.with_scl(peripherals.GPIO2);
    /// # {after_snippet}
    /// ```
    ///
    /// Using `Flex` to configure the pin
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::{
    ///     gpio::{DriveMode, Flex, OutputConfig},
    ///     i2c::master::{Config, I2c},
    /// };
    ///
    /// let mut scl = Flex::new(peripherals.GPIO2);
    ///
    /// // The default pullup setting is `Pull::None`.
    /// scl.apply_output_config(&OutputConfig::default().with_drive_mode(DriveMode::OpenDrain));
    /// scl.set_input_enable(true);
    /// scl.set_output_enable(true);
    /// // Initial pin state to avoid the pin to go low during peripheral configuration.
    /// scl.set_high();
    ///
    /// let i2c = I2c::new(peripherals.I2C0, Config::default())?.with_scl(scl);
    /// # {after_snippet}
    /// ```
    pub fn with_scl(mut self, scl: impl PeripheralInput<'d> + PeripheralOutput<'d>) -> Self {
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
