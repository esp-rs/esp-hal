//! # Inter-Integrated Circuit (I2C) - Master Mode
//!
//! I2C is a serial, synchronous, multi-device, half-duplex communication
//! protocol that allows co-existence of multiple masters and slaves on the
//! same bus. I2C uses two bidirectional open-drain lines: serial data line
//! (SDA) and serial clock line (SCL), pulled up by resistors.
//!
//! Espressif devices sometimes have more than one I2C controller, responsible
//! for handling communication on the I2C bus. A single I2C controller can be
//! a master or a slave.
//!
//! Typically, an I2C slave device has a 7-bit address or 10-bit address.
//! Devices supports both I2C Standard-mode (Sm) and Fast-mode (Fm) which can
//! go up to 100KHz and 400KHz respectively.
//!
//! ## Configuration
//!
//! Each I2C controller is individually configurable, and the usual setting
//! such as frequency, timeout, and SDA/SCL pins can easily be configured.
//!
//! ## Usage
//!
//! The I2C driver implements a number of third-party traits, with the
//! intention of making the HAL inter-compatible with various device drivers
//! from the community. This includes the [`embedded-hal`] for both 0.2.x and
//! 1.0.x versions.
//!
//! ## Examples
//!
//! ### Read Data from a BMP180 Sensor
//!
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::i2c::master::{Config, I2c};
//! # use esp_hal::gpio::Io;
//! let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
//!
//! // Create a new peripheral object with the described wiring
//! // and standard I2C clock speed.
//! let mut i2c = I2c::new_with_config(peripherals.I2C0, Config {
//!     frequency: 100.kHz(),
//!     timeout: None,
//! })
//! .with_sda(io.pins.gpio1)
//! .with_scl(io.pins.gpio2);
//!
//! loop {
//!     let mut data = [0u8; 22];
//!     i2c.write_read(0x77, &[0xaa], &mut data).ok();
//! }
//! # }
//! ```
//! [`embedded-hal`]: https://crates.io/crates/embedded-hal

mod support;
mod version;

use core::marker::PhantomData;

use embassy_embedded_hal::SetConfig;
use embassy_sync::waitqueue::AtomicWaker;
use fugit::HertzU32;
use version::{I2C_CHUNK_SIZE, I2C_LL_INTR_MASK};

use crate::{
    clock::Clocks,
    dma::PeripheralMarker,
    gpio::{interconnect::PeripheralOutput, InputSignal, OutputSignal, Pull},
    interrupt::InterruptHandler,
    peripheral::{Peripheral, PeripheralRef},
    peripherals::{
        i2c0::{RegisterBlock, COMD},
        Interrupt,
    },
    private,
    system::PeripheralClockControl,
    Async,
    Blocking,
    Cpu,
    InterruptConfigurable,
    Mode,
};

// on ESP32 there is a chance to get trapped in `wait_for_completion` forever
const MAX_ITERATIONS: u32 = 1_000_000;

/// I2C-specific transmission errors
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// The transmission exceeded the FIFO size.
    ExceedingFifo,
    /// The acknowledgment check failed.
    AckCheckFailed,
    /// A timeout occurred during transmission.
    TimeOut,
    /// The arbitration for the bus was lost.
    ArbitrationLost,
    /// The execution of the I2C command was incomplete.
    ExecIncomplete,
    /// The number of commands issued exceeded the limit.
    CommandNrExceeded,
    /// Zero length read or write operation.
    InvalidZeroLength,
}

/// I2C-specific configuration errors
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ConfigError {}

#[derive(PartialEq)]
// This enum is used to keep track of the last/next operation that was/will be
// performed in an embedded-hal(-async) I2c::transaction. It is used to
// determine whether a START condition should be issued at the start of the
// current operation and whether a read needs an ack or a nack for the final
// byte.
enum OpKind {
    Write,
    Read,
}

/// I2C operation.
///
/// Several operations can be combined as part of a transaction.
pub enum Operation<'a> {
    /// Write data from the provided buffer.
    Write(&'a [u8]),

    /// Read data into the provided buffer.
    Read(&'a mut [u8]),
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
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    /// The I2C clock frequency.
    pub frequency: HertzU32,

    /// I2C SCL timeout period.
    ///
    /// When the level of SCL remains unchanged for more than `timeout` bus
    /// clock cycles, the bus goes to idle state.
    ///
    /// The default value is about 10 bus clock cycles.
    #[doc = ""]
    #[cfg_attr(
        not(esp32),
        doc = "Note that the effective timeout may be longer than the value configured here."
    )]
    #[cfg_attr(not(esp32), doc = "Configuring `None` disables timeout control.")]
    #[cfg_attr(esp32, doc = "Configuring `None` equals to the maximum timeout value.")]
    // TODO: when supporting interrupts, document that SCL = high also triggers an interrupt.
    pub timeout: Option<u32>,
}

impl Default for Config {
    fn default() -> Self {
        use fugit::RateExtU32;
        Config {
            frequency: 100.kHz(),
            timeout: Some(10),
        }
    }
}

/// I2C driver
pub struct I2c<'d, DM: Mode, T = AnyI2c> {
    i2c: PeripheralRef<'d, T>,
    phantom: PhantomData<DM>,
    config: Config,
}

impl<T: Instance, DM: Mode> SetConfig for I2c<'_, DM, T> {
    type Config = Config;
    type ConfigError = ConfigError;

    fn set_config(&mut self, config: &Self::Config) -> Result<(), Self::ConfigError> {
        self.apply_config(config)
    }
}

impl<'d, T, DM: Mode> I2c<'d, DM, T>
where
    T: Instance,
{
    fn driver(&self) -> Driver<'_> {
        Driver {
            info: self.i2c.info(),
            state: self.i2c.state(),
        }
    }

    fn internal_recover(&self) {
        PeripheralClockControl::reset(self.i2c.peripheral());
        PeripheralClockControl::enable(self.i2c.peripheral());

        // It's okay to ignore the result, we've already validated the config
        // either in `apply_config` or in `new_typed_with_config`.
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
        address: u8,
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
                    let chunk_count = buffer.len().div_ceil(I2C_CHUNK_SIZE);
                    for (idx, chunk) in buffer.chunks(I2C_CHUNK_SIZE).enumerate() {
                        let last_chunk = idx == chunk_count - 1;
                        self.driver()
                            .write_operation_blocking(
                                address,
                                chunk,
                                idx == 0 && !matches!(last_op, Some(OpKind::Write)),
                                last_chunk && next_op.is_none(),
                            )
                            .inspect_err(|_| self.internal_recover())?;
                    }
                }
                Operation::Read(buffer) => {
                    // execute a read operation:
                    // - issue START/RSTART if op is different from previous
                    // - issue STOP if op is the last one
                    // - will_continue is true if there is another read operation next
                    let chunk_count = buffer.len().div_ceil(I2C_CHUNK_SIZE);
                    for (idx, chunk) in buffer.chunks_mut(I2C_CHUNK_SIZE).enumerate() {
                        let last_chunk = idx == chunk_count - 1;
                        self.driver()
                            .read_operation_blocking(
                                address,
                                chunk,
                                idx == 0 && !matches!(last_op, Some(OpKind::Read)),
                                last_chunk && next_op.is_none(),
                                matches!(next_op, Some(OpKind::Read)) || !last_chunk,
                            )
                            .inspect_err(|_| self.internal_recover())?;
                    }
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
        pin.set_output_high(true, private::Internal);

        pin.set_to_open_drain_output(private::Internal);
        pin.enable_input(true, private::Internal);
        pin.pull_direction(Pull::Up, private::Internal);

        input.connect_to(&mut pin);
        output.connect_to(&mut pin);

        self
    }
}

impl<'d> I2c<'d, Blocking> {
    /// Creates a new I2C instance.
    ///
    /// This will enable the peripheral but the peripheral won't get
    /// automatically disabled when this gets dropped.
    pub fn new(i2c: impl Peripheral<P = impl Instance> + 'd) -> Self {
        Self::new_with_config(i2c.map_into(), Config::default())
    }

    /// Creates a new I2C instance with a given configuration.
    ///
    /// This will enable the peripheral but the peripheral won't get
    /// automatically disabled when this gets dropped.
    pub fn new_with_config(i2c: impl Peripheral<P = impl Instance> + 'd, config: Config) -> Self {
        Self::new_typed_with_config(i2c.map_into(), config)
    }
}

impl<'d, T> I2c<'d, Blocking, T>
where
    T: Instance,
{
    /// Creates a new I2C instance with a given configuration.
    ///
    /// This will enable the peripheral but the peripheral won't get
    /// automatically disabled when this gets dropped.
    pub fn new_typed(i2c: impl Peripheral<P = T> + 'd) -> Self {
        Self::new_typed_with_config(i2c, Config::default())
    }

    /// Creates a new I2C instance with a given configuration.
    ///
    /// This will enable the peripheral but the peripheral won't get
    /// automatically disabled when this gets dropped.
    pub fn new_typed_with_config(i2c: impl Peripheral<P = T> + 'd, config: Config) -> Self {
        crate::into_ref!(i2c);

        let i2c = I2c {
            i2c,
            phantom: PhantomData,
            config,
        };

        PeripheralClockControl::reset(i2c.i2c.peripheral());
        PeripheralClockControl::enable(i2c.i2c.peripheral());

        unwrap!(i2c.driver().setup(&i2c.config));
        i2c
    }

    // TODO: missing interrupt APIs

    /// Configures the I2C peripheral to operate in asynchronous mode.
    pub fn into_async(mut self) -> I2c<'d, Async, T> {
        self.set_interrupt_handler(self.driver().info.async_handler);

        I2c {
            i2c: self.i2c,
            phantom: PhantomData,
            config: self.config,
        }
    }

    /// Reads enough bytes from slave with `address` to fill `buffer`
    pub fn read(&mut self, address: u8, buffer: &mut [u8]) -> Result<(), Error> {
        let chunk_count = buffer.len().div_ceil(I2C_CHUNK_SIZE);
        for (idx, chunk) in buffer.chunks_mut(I2C_CHUNK_SIZE).enumerate() {
            self.driver()
                .read_operation_blocking(
                    address,
                    chunk,
                    idx == 0,
                    idx == chunk_count - 1,
                    idx < chunk_count - 1,
                )
                .inspect_err(|_| self.internal_recover())?;
        }

        Ok(())
    }

    /// Writes bytes to slave with address `address`
    pub fn write(&mut self, address: u8, buffer: &[u8]) -> Result<(), Error> {
        let chunk_count = buffer.len().div_ceil(I2C_CHUNK_SIZE);
        for (idx, chunk) in buffer.chunks(I2C_CHUNK_SIZE).enumerate() {
            self.driver()
                .write_operation_blocking(address, chunk, idx == 0, idx == chunk_count - 1)
                .inspect_err(|_| self.internal_recover())?;
        }

        Ok(())
    }

    /// Writes bytes to slave with address `address` and then reads enough bytes
    /// to fill `buffer` *in a single transaction*
    pub fn write_read(
        &mut self,
        address: u8,
        write_buffer: &[u8],
        read_buffer: &mut [u8],
    ) -> Result<(), Error> {
        let write_count = write_buffer.len().div_ceil(I2C_CHUNK_SIZE);
        let read_count = read_buffer.len().div_ceil(I2C_CHUNK_SIZE);

        for (idx, chunk) in write_buffer.chunks(I2C_CHUNK_SIZE).enumerate() {
            self.driver()
                .write_operation_blocking(
                    address,
                    chunk,
                    idx == 0,
                    idx == write_count - 1 && read_count == 0,
                )
                .inspect_err(|_| self.internal_recover())?;
        }

        for (idx, chunk) in read_buffer.chunks_mut(I2C_CHUNK_SIZE).enumerate() {
            self.driver()
                .read_operation_blocking(
                    address,
                    chunk,
                    idx == 0,
                    idx == read_count - 1,
                    idx < read_count - 1,
                )
                .inspect_err(|_| self.internal_recover())?;
        }

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
    pub fn transaction<'a>(
        &mut self,
        address: u8,
        operations: impl IntoIterator<Item = &'a mut Operation<'a>>,
    ) -> Result<(), Error> {
        self.transaction_impl(address, operations.into_iter().map(Operation::from))
    }
}

impl<T> private::Sealed for I2c<'_, Blocking, T> where T: Instance {}

impl<T> InterruptConfigurable for I2c<'_, Blocking, T>
where
    T: Instance,
{
    fn set_interrupt_handler(&mut self, handler: crate::interrupt::InterruptHandler) {
        let interrupt = self.driver().info.interrupt;
        for core in Cpu::other() {
            crate::interrupt::disable(core, interrupt);
        }
        unsafe { crate::interrupt::bind_interrupt(interrupt, handler.handler()) };
        unwrap!(crate::interrupt::enable(interrupt, handler.priority()));
    }
}

impl<'d, T> I2c<'d, Async, T>
where
    T: Instance,
{
    /// Configure the I2C peripheral to operate in blocking mode.
    pub fn into_blocking(self) -> I2c<'d, Blocking, T> {
        let interrupt = self.driver().info.interrupt;
        crate::interrupt::disable(Cpu::current(), interrupt);

        I2c {
            i2c: self.i2c,
            phantom: PhantomData,
            config: self.config,
        }
    }

    /// Writes bytes to slave with address `address`
    pub async fn write(&mut self, address: u8, buffer: &[u8]) -> Result<(), Error> {
        let chunk_count = buffer.len().div_ceil(I2C_CHUNK_SIZE);
        for (idx, chunk) in buffer.chunks(I2C_CHUNK_SIZE).enumerate() {
            self.driver()
                .write_operation(address, chunk, idx == 0, idx == chunk_count - 1)
                .await?;
        }

        Ok(())
    }

    /// Reads enough bytes from slave with `address` to fill `buffer`
    pub async fn read(&mut self, address: u8, buffer: &mut [u8]) -> Result<(), Error> {
        let chunk_count = buffer.len().div_ceil(I2C_CHUNK_SIZE);
        for (idx, chunk) in buffer.chunks_mut(I2C_CHUNK_SIZE).enumerate() {
            self.driver()
                .read_operation(
                    address,
                    chunk,
                    idx == 0,
                    idx == chunk_count - 1,
                    idx < chunk_count - 1,
                )
                .await?;
        }

        Ok(())
    }

    /// Writes bytes to slave with address `address` and then reads enough
    /// bytes to fill `buffer` *in a single transaction*
    pub async fn write_read(
        &mut self,
        address: u8,
        write_buffer: &[u8],
        read_buffer: &mut [u8],
    ) -> Result<(), Error> {
        let write_count = write_buffer.len().div_ceil(I2C_CHUNK_SIZE);
        let read_count = read_buffer.len().div_ceil(I2C_CHUNK_SIZE);
        for (idx, chunk) in write_buffer.chunks(I2C_CHUNK_SIZE).enumerate() {
            self.driver()
                .write_operation(
                    address,
                    chunk,
                    idx == 0,
                    idx == write_count - 1 && read_count == 0,
                )
                .await?;
        }

        for (idx, chunk) in read_buffer.chunks_mut(I2C_CHUNK_SIZE).enumerate() {
            self.driver()
                .read_operation(
                    address,
                    chunk,
                    idx == 0,
                    idx == read_count - 1,
                    idx < read_count - 1,
                )
                .await?;
        }

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
    pub async fn transaction<'a>(
        &mut self,
        address: u8,
        operations: impl IntoIterator<Item = &'a mut Operation<'a>>,
    ) -> Result<(), Error> {
        self.transaction_impl_async(address, operations.into_iter().map(Operation::from))
            .await
    }

    async fn transaction_impl_async<'a>(
        &mut self,
        address: u8,
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
                    let chunk_count = buffer.len().div_ceil(I2C_CHUNK_SIZE);
                    for (idx, chunk) in buffer.chunks(I2C_CHUNK_SIZE).enumerate() {
                        let last_chunk = idx == chunk_count - 1;
                        self.driver()
                            .write_operation(
                                address,
                                chunk,
                                idx == 0 && !matches!(last_op, Some(OpKind::Write)),
                                last_chunk && next_op.is_none(),
                            )
                            .await?;
                    }
                }
                Operation::Read(buffer) => {
                    // execute a read operation:
                    // - issue START/RSTART if op is different from previous
                    // - issue STOP if op is the last one
                    // - will_continue is true if there is another read operation next
                    let chunk_count = buffer.len().div_ceil(I2C_CHUNK_SIZE);
                    for (idx, chunk) in buffer.chunks_mut(I2C_CHUNK_SIZE).enumerate() {
                        let last_chunk = idx == chunk_count - 1;
                        self.driver()
                            .read_operation(
                                address,
                                chunk,
                                idx == 0 && !matches!(last_op, Some(OpKind::Read)),
                                last_chunk && next_op.is_none(),
                                matches!(next_op, Some(OpKind::Read)) || !last_chunk,
                            )
                            .await?;
                    }
                }
            }

            last_op = Some(kind);
        }

        Ok(())
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
    timeout: Option<u32>,
) {
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

        register_block.scl_high_period().write(|w| {
            #[cfg(not(esp32))] // ESP32 does not have a wait_high field
            w.scl_wait_high_period()
                .bits(scl_wait_high_period.try_into().unwrap());
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
                    .write(|w| w.time_out().bits(unwrap!(timeout)));
            } else {
                register_block
                    .to()
                    .write(|w| w.time_out_en().bit(timeout.is_some())
                    .time_out_value()
                    .bits(timeout.unwrap_or(1) as _)
                );
            }
        }
    }
}

/// Peripheral data describing a particular I2C instance.
#[non_exhaustive]
pub struct Info {
    /// Pointer to the register block for this I2C instance.
    ///
    /// Use [Self::register_block] to access the register block.
    pub register_block: *const RegisterBlock,

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
}

#[allow(dead_code)] // Some versions don't need `state`
struct Driver<'a> {
    info: &'a Info,
    state: &'a State,
}

impl Driver<'_> {
    /// Configures the I2C peripheral with the specified frequency, clocks, and
    /// optional timeout.
    fn setup(&self, config: &Config) -> Result<(), ConfigError> {
        self.info.register_block().ctr().write(|w| {
            // Set I2C controller to master mode
            w.ms_mode().set_bit();
            // Use open drain output for SDA and SCL
            w.sda_force_out().set_bit();
            w.scl_force_out().set_bit();
            // Use Most Significant Bit first for sending and receiving data
            w.tx_lsb_first().clear_bit();
            w.rx_lsb_first().clear_bit();
            // Ensure that clock is enabled
            #[cfg(esp32s2)]
            w.ref_always_on().set_bit();
            w.clk_en().set_bit()
        });

        // Configure filter
        // FIXME if we ever change this we need to adapt `set_frequency` for ESP32
        self.set_filter(Some(7), Some(7));

        // Configure frequency
        self.set_frequency(config.frequency, config.timeout);

        self.update_config();

        // Reset entire peripheral (also resets fifo)
        self.reset();

        Ok(())
    }

    /// Resets the I2C peripheral's command registers
    fn reset_command_list(&self) {
        // Confirm that all commands that were configured were actually executed
        for cmd in self.info.register_block().comd_iter() {
            cmd.reset();
        }
    }

    /// Configures the I2C peripheral for a write operation.
    /// - `addr` is the address of the slave device.
    /// - `bytes` is the data two be sent.
    /// - `start` indicates whether the operation should start by a START
    ///   condition and sending the address.
    /// - `cmd_iterator` is an iterator over the command registers.
    fn setup_write<'a, I>(
        &self,
        addr: u8,
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
            return Err(Error::ExceedingFifo);
        }

        let write_len = if start { bytes.len() + 1 } else { bytes.len() };
        // don't issue write if there is no data to write
        if write_len > 0 {
            // WRITE command
            self.add_cmd(
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
            self.write_fifo(addr << 1 | OperationType::Write as u8);
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
        addr: u8,
        buffer: &mut [u8],
        start: bool,
        will_continue: bool,
        cmd_iterator: &mut I,
    ) -> Result<(), Error>
    where
        I: Iterator<Item = &'a COMD>,
    {
        if buffer.is_empty() {
            return Err(Error::InvalidZeroLength);
        }
        let (max_len, initial_len) = if will_continue {
            (255usize, buffer.len())
        } else {
            (254usize, buffer.len() - 1)
        };
        if buffer.len() > max_len {
            // we could support more by adding multiple read operations
            return Err(Error::ExceedingFifo);
        }

        if start {
            // WRITE command
            self.add_cmd(
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
            self.add_cmd(
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
            self.add_cmd(
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
            self.write_fifo(addr << 1 | OperationType::Read as u8);
        }
        Ok(())
    }

    /// Clears all pending interrupts for the I2C peripheral.
    fn clear_all_interrupts(&self) {
        self.info
            .register_block()
            .int_clr()
            .write(|w| unsafe { w.bits(I2C_LL_INTR_MASK) });
    }

    /// Waits for the completion of an I2C transaction.
    fn wait_for_completion_blocking(&self, end_only: bool) -> Result<(), Error> {
        let mut tout = MAX_ITERATIONS;
        loop {
            let interrupts = self.info.register_block().int_raw().read();

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
                return Err(Error::TimeOut);
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
        for cmd_reg in self.info.register_block().comd_iter() {
            let cmd = cmd_reg.read();

            if cmd.bits() != 0x0 && !cmd.opcode().is_end() && !cmd.command_done().bit_is_set() {
                return Err(Error::ExecIncomplete);
            }
        }

        Ok(())
    }

    /// Starts an I2C transmission.
    fn start_transmission(&self) {
        // Start transmission
        self.info
            .register_block()
            .ctr()
            .modify(|_, w| w.trans_start().set_bit());
    }

    /// Executes an I2C write operation.
    /// - `addr` is the address of the slave device.
    /// - `bytes` is the data two be sent.
    /// - `start` indicates whether the operation should start by a START
    ///   condition and sending the address.
    /// - `stop` indicates whether the operation should end with a STOP
    ///   condition.
    fn write_operation_blocking(
        &self,
        address: u8,
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

    /// Executes an I2C write operation.
    /// - `addr` is the address of the slave device.
    /// - `bytes` is the data two be sent.
    /// - `start` indicates whether the operation should start by a START
    ///   condition and sending the address.
    /// - `stop` indicates whether the operation should end with a STOP
    ///   condition.
    async fn write_operation(
        &self,
        address: u8,
        bytes: &[u8],
        start: bool,
        stop: bool,
    ) -> Result<(), Error> {
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

    /// Executes an I2C read operation.
    /// - `addr` is the address of the slave device.
    /// - `buffer` is the buffer to store the read data.
    /// - `start` indicates whether the operation should start by a START
    ///   condition and sending the address.
    /// - `stop` indicates whether the operation should end with a STOP
    ///   condition.
    /// - `will_continue` indicates whether there is another read operation
    ///   following this one and we should not nack the last byte.
    fn read_operation_blocking(
        &self,
        address: u8,
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

    /// Executes an async I2C read operation.
    /// - `addr` is the address of the slave device.
    /// - `buffer` is the buffer to store the read data.
    /// - `start` indicates whether the operation should start by a START
    ///   condition and sending the address.
    /// - `stop` indicates whether the operation should end with a STOP
    ///   condition.
    /// - `will_continue` indicates whether there is another read operation
    ///   following this one and we should not nack the last byte.
    async fn read_operation(
        &self,
        address: u8,
        buffer: &mut [u8],
        start: bool,
        stop: bool,
        will_continue: bool,
    ) -> Result<(), Error> {
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

    fn start_read_operation(
        &self,
        address: u8,
        buffer: &mut [u8],
        start: bool,
        stop: bool,
        will_continue: bool,
    ) -> Result<(), Error> {
        // Reset FIFO and command list
        self.reset_fifo();
        self.reset_command_list();
        let cmd_iterator = &mut self.info.register_block().comd_iter();

        if start {
            self.add_cmd(cmd_iterator, Command::Start)?;
        }

        self.setup_read(address, buffer, start, will_continue, cmd_iterator)?;

        self.add_cmd(
            cmd_iterator,
            if stop { Command::Stop } else { Command::End },
        )?;
        self.start_transmission();

        Ok(())
    }

    fn start_write_operation(
        &self,
        address: u8,
        bytes: &[u8],
        start: bool,
        stop: bool,
    ) -> Result<usize, Error> {
        // Reset FIFO and command list
        self.reset_fifo();
        self.reset_command_list();
        let cmd_iterator = &mut self.info.register_block().comd_iter();

        if start {
            self.add_cmd(cmd_iterator, Command::Start)?;
        }
        self.setup_write(address, bytes, start, cmd_iterator)?;
        self.add_cmd(
            cmd_iterator,
            if stop { Command::Stop } else { Command::End },
        )?;
        let index = self.fill_tx_fifo(bytes);
        self.start_transmission();

        Ok(index)
    }

    /// Adds a command to the I2C command sequence.
    fn add_cmd<'a, I>(&self, cmd_iterator: &mut I, command: Command) -> Result<(), Error>
    where
        I: Iterator<Item = &'a COMD>,
    {
        let cmd = cmd_iterator.next().ok_or(Error::CommandNrExceeded)?;

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
}

impl PartialEq for Info {
    fn eq(&self, other: &Self) -> bool {
        self.register_block == other.register_block
    }
}

unsafe impl Sync for Info {}

/// Peripheral state for an I2C instance.
#[non_exhaustive]
pub struct State {
    /// Waker for the asynchronous operations.
    pub waker: AtomicWaker,
}

/// I2C Peripheral Instance
#[doc(hidden)]
pub trait Instance: Peripheral<P = Self> + PeripheralMarker + Into<AnyI2c> + 'static {
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

macro_rules! instance {
    ($inst:ident, $scl:ident, $sda:ident, $interrupt:ident) => {
        impl Instance for crate::peripherals::$inst {
            fn parts(&self) -> (&Info, &State) {
                #[crate::macros::handler]
                pub(super) fn irq_handler() {
                    async_handler(&PERIPHERAL, &STATE);
                }

                static STATE: State = State {
                    waker: AtomicWaker::new(),
                };

                static PERIPHERAL: Info = Info {
                    register_block: crate::peripherals::$inst::ptr(),
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
instance!(I2C0, I2CEXT0_SCL, I2CEXT0_SDA, I2C_EXT0);
#[cfg(i2c1)]
instance!(I2C1, I2CEXT1_SCL, I2CEXT1_SDA, I2C_EXT1);

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
