//! # Inter-Integrated Circuit (I2C) - Slave mode
//!
//! ## Overview
//!
//! This driver implements the I2C Slave mode. In this mode, the MCU responds to
//! and communicates with one or more master devices. The MCU acts as a slave
//! device on the I2C bus, identified by its unique I2C address.
//!
//! ## Implementation
//!
//! This driver supports both Blocking and Async modes, using hardware SCL clock stretching
//! to handle flow control. The async implementation runs a callback-free waker model, waking
//! the task waker when stretching or transaction completeness interrupts occur.

use core::{
    future::Future,
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
    i2c::master::I2cAddress,
    interrupt::InterruptHandler,
    pac::i2c0::RegisterBlock,
    private,
    ram,
    system::PeripheralGuard,
};

/// FIFO size of the I2C peripheral.
pub const FIFO_SIZE: usize = property!("i2c_master.fifo_size");

/// I2C Slave error
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// A timeout occurred during transmission.
    Timeout,
    /// The arbitration for the bus was lost.
    ArbitrationLost,
}

impl embedded_hal::i2c::Error for Error {
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        match self {
            Self::ArbitrationLost => embedded_hal::i2c::ErrorKind::ArbitrationLoss,
            _ => embedded_hal::i2c::ErrorKind::Other,
        }
    }
}

/// I2C Slave configuration
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, procmacros::BuilderLite)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Config {
    /// The I2C slave address.
    address: I2cAddress,
}

impl Config {
    /// Creates a new I2C slave configuration with the specified address.
    pub fn new(address: impl Into<I2cAddress>) -> Self {
        Config {
            address: address.into(),
        }
    }
}

/// I2C slave configuration errors
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum ConfigError {
    /// Provided address is not valid.
    AddressInvalid,
}

impl core::error::Error for ConfigError {}

impl core::fmt::Display for ConfigError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            ConfigError::AddressInvalid => write!(f, "Provided address is invalid"),
        }
    }
}

/// Received command from the master
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Command {
    /// The master is requesting data from the slave.
    Read,
    /// The master wrote the specified number of bytes to the slave.
    Write(usize),
}

/// Possible responses to responding to a read request
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ReadStatus {
    /// Transaction Complete; master completed reading all provided bytes.
    Done,
    /// Transaction Incomplete; master is trying to read more bytes than were provided.
    NeedMoreBytes,
    /// Transaction Complete, but master stopped reading before all bytes were consumed.
    LeftoverBytes(u16),
}

/// I2C Slave driver
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct I2cSlave<'d, Dm: DriverMode> {
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

impl<Dm: DriverMode> embedded_hal::i2c::ErrorType for I2cSlave<'_, Dm> {
    type Error = Error;
}

impl<'d> I2cSlave<'d, Blocking> {
    /// Create a new I2C slave instance.
    pub fn new(i2c: impl Instance + 'd, config: Config) -> Result<Self, ConfigError> {
        let guard = PeripheralGuard::new(i2c.info().peripheral);

        let sda_pin = PinGuard::new_unconnected();
        let scl_pin = PinGuard::new_unconnected();

        let mut slave = I2cSlave {
            i2c: i2c.degrade(),
            phantom: PhantomData,
            guard,
            config: DriverConfig {
                config,
                sda_pin,
                scl_pin,
            },
        };

        slave.apply_config(&config)?;
        Ok(slave)
    }

    /// Reconfigures the driver to operate in [`Async`] mode.
    pub fn into_async(self) -> I2cSlave<'d, Async> {
        self.i2c
            .set_interrupt_handler(self.driver().info.async_handler);

        I2cSlave {
            i2c: self.i2c,
            phantom: PhantomData,
            guard: self.guard,
            config: self.config,
        }
    }
}

impl<'d> I2cSlave<'d, Async> {
    /// Reconfigures the driver to operate in [`Blocking`] mode.
    pub fn into_blocking(self) -> I2cSlave<'d, Blocking> {
        self.i2c.disable_peri_interrupt();

        I2cSlave {
            i2c: self.i2c,
            phantom: PhantomData,
            guard: self.guard,
            config: self.config,
        }
    }
}

impl<'d, Dm: DriverMode> I2cSlave<'d, Dm> {
    fn driver(&self) -> Driver<'_> {
        Driver {
            info: self.i2c.info(),
            state: self.i2c.state(),
            config: &self.config,
        }
    }

    /// Connect a pin to the I2C SDA signal.
    pub fn with_sda(mut self, sda: impl PeripheralInput<'d> + PeripheralOutput<'d>) -> Self {
        let info = self.driver().info;
        let input = info.sda_input;
        let output = info.sda_output;
        Driver::connect_pin(sda.into(), input, output, &mut self.config.sda_pin);
        self
    }

    /// Connect a pin to the I2C SCL signal.
    pub fn with_scl(mut self, scl: impl PeripheralInput<'d> + PeripheralOutput<'d>) -> Self {
        let info = self.driver().info;
        let input = info.scl_input;
        let output = info.scl_output;
        Driver::connect_pin(scl.into(), input, output, &mut self.config.scl_pin);
        self
    }

    /// Applies a new configuration.
    pub fn apply_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        self.config.config = *config;
        self.driver().setup(config)?;
        Ok(())
    }
}

impl<'d, Dm: DriverMode> I2cSlave<'d, Dm> {
    /// Listens for a request from the master.
    ///
    /// If the master writes to the slave, this method reads data into the provided
    /// `buffer` and returns `Command::Write(len)`.
    /// If the master is requesting a read, it returns `Command::Read` with the SCL line
    /// stretched (held low).
    pub fn listen(&mut self, buffer: &mut [u8]) -> Result<Command, Error> {
        let regs = self.driver().info.regs();

        self.driver().reset_fifo();

        regs.scl_stretch_conf().modify(|_, w| unsafe {
            w.stretch_protect_num().bits(0x3ff);
            w.slave_scl_stretch_en().set_bit()
        });

        regs.int_clr().write(|w| unsafe { w.bits(0x3fff) });
        regs.scl_stretch_conf()
            .modify(|_, w| w.slave_scl_stretch_clr().set_bit());
        self.driver().update_registers();

        let mut bytes_read = 0;
        loop {
            self.driver().check_errors()?;
            if let Some(res) = self.driver().listen_step(buffer, &mut bytes_read) {
                return res;
            }
        }
    }

    /// Responds to a master read request.
    ///
    /// Writes the contents of `buffer` to the TX FIFO, clears the clock stretch,
    /// and waits for the transaction to complete.
    pub fn respond_to_read(&mut self, buffer: &[u8]) -> Result<ReadStatus, Error> {
        let regs = self.driver().info.regs();
        let mut bytes_written = 0;
        let initial_write = core::cmp::min(buffer.len(), FIFO_SIZE);
        for &byte in buffer.iter().take(initial_write) {
            regs.data().write(|w| unsafe { w.fifo_rdata().bits(byte) });
        }
        bytes_written += initial_write;

        regs.scl_stretch_conf()
            .modify(|_, w| w.slave_scl_stretch_clr().set_bit());
        regs.int_clr().write(|w| unsafe { w.bits(0x3fff) });
        self.driver().update_registers();

        loop {
            self.driver().check_errors()?;
            if let Some(res) = self
                .driver()
                .respond_to_read_step(buffer, &mut bytes_written)
            {
                return res;
            }
        }
    }
}

impl<'d> I2cSlave<'d, Async> {
    /// Listens for a request from the master asynchronously.
    pub async fn listen_async(&mut self, buffer: &mut [u8]) -> Result<Command, Error> {
        let regs = self.driver().info.regs();
        let state = self.i2c.state();

        self.driver().reset_fifo();

        regs.scl_stretch_conf().modify(|_, w| unsafe {
            w.stretch_protect_num().bits(0x3ff);
            w.slave_scl_stretch_en().set_bit()
        });

        regs.int_clr().write(|w| unsafe { w.bits(0x3fff) });
        regs.scl_stretch_conf()
            .modify(|_, w| w.slave_scl_stretch_clr().set_bit());
        self.driver().update_registers();

        let mut bytes_read = 0;
        loop {
            I2cSlaveFuture::new(regs, state, Event::SlaveStretch | Event::TransComplete).await?;
            if let Some(res) = self.driver().listen_step(buffer, &mut bytes_read) {
                return res;
            }
        }
    }

    /// Responds to a master read request asynchronously.
    pub async fn respond_to_read_async(&mut self, buffer: &[u8]) -> Result<ReadStatus, Error> {
        let regs = self.driver().info.regs();
        let state = self.i2c.state();

        let mut bytes_written = 0;
        let initial_write = core::cmp::min(buffer.len(), FIFO_SIZE);
        for &byte in buffer.iter().take(initial_write) {
            regs.data().write(|w| unsafe { w.fifo_rdata().bits(byte) });
        }
        bytes_written += initial_write;

        regs.scl_stretch_conf()
            .modify(|_, w| w.slave_scl_stretch_clr().set_bit());
        regs.int_clr().write(|w| unsafe { w.bits(0x3fff) });
        self.driver().update_registers();

        loop {
            I2cSlaveFuture::new(
                regs,
                state,
                Event::SlaveStretch | Event::TransComplete | Event::Nack,
            )
            .await?;

            if let Some(res) = self
                .driver()
                .respond_to_read_step(buffer, &mut bytes_written)
            {
                return res;
            }
        }
    }
}

impl<Dm: DriverMode> private::Sealed for I2cSlave<'_, Dm> {}

#[derive(Debug, EnumSetType)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub(crate) enum Event {
    SlaveStretch,
    TransComplete,
    Nack,
}

#[must_use = "futures do nothing unless you `.await` or poll them"]
struct I2cSlaveFuture<'a> {
    regs: &'a RegisterBlock,
    state: &'a State,
    events: EnumSet<Event>,
}

impl<'a> I2cSlaveFuture<'a> {
    pub fn new(regs: &'a RegisterBlock, state: &'a State, events: EnumSet<Event>) -> Self {
        regs.int_ena().modify(|_, w| {
            if events.contains(Event::SlaveStretch) {
                w.slave_stretch().set_bit();
            }
            if events.contains(Event::TransComplete) {
                w.trans_complete().set_bit();
            }
            if events.contains(Event::Nack) {
                w.nack().set_bit();
            }
            w.arbitration_lost().set_bit();
            w.time_out().set_bit();
            w
        });

        Self {
            regs,
            state,
            events,
        }
    }
}

impl Future for I2cSlaveFuture<'_> {
    type Output = Result<(), Error>;

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        self.state.waker.register(cx.waker());

        let raw = self.regs.int_raw().read();

        if raw.arbitration_lost().bit_is_set() {
            return Poll::Ready(Err(Error::ArbitrationLost));
        }
        if raw.time_out().bit_is_set() {
            return Poll::Ready(Err(Error::Timeout));
        }

        let mut triggered = false;
        if self.events.contains(Event::SlaveStretch) && raw.slave_stretch().bit_is_set() {
            triggered = true;
        }
        if self.events.contains(Event::TransComplete) && raw.trans_complete().bit_is_set() {
            triggered = true;
        }
        if self.events.contains(Event::Nack) && raw.nack().bit_is_set() {
            triggered = true;
        }

        if triggered {
            Poll::Ready(Ok(()))
        } else {
            self.regs.int_ena().modify(|_, w| {
                if self.events.contains(Event::SlaveStretch) {
                    w.slave_stretch().set_bit();
                }
                if self.events.contains(Event::TransComplete) {
                    w.trans_complete().set_bit();
                }
                if self.events.contains(Event::Nack) {
                    w.nack().set_bit();
                }
                w.arbitration_lost().set_bit();
                w.time_out().set_bit();
                w
            });
            Poll::Pending
        }
    }
}

impl Drop for I2cSlaveFuture<'_> {
    fn drop(&mut self) {
        self.regs.int_ena().write(|w| unsafe { w.bits(0) });
    }
}

#[ram]
fn async_handler(info: &Info, state: &State) {
    info.regs().int_ena().write(|w| unsafe { w.bits(0) });
    state.waker.wake();
}

/// I2C Slave state.
#[non_exhaustive]
pub struct State {
    pub(crate) waker: AtomicWaker,
}

impl core::fmt::Debug for State {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("State")
            .field("waker", &"<AtomicWaker>")
            .finish()
    }
}

/// I2C Slave peripheral information.
#[derive(Debug)]
#[non_exhaustive]
pub struct Info {
    pub(crate) register_block: *const RegisterBlock,
    pub(crate) peripheral: crate::system::Peripheral,
    pub(crate) async_handler: InterruptHandler,
    pub(crate) scl_output: OutputSignal,
    pub(crate) scl_input: InputSignal,
    pub(crate) sda_output: OutputSignal,
    pub(crate) sda_input: InputSignal,
}

impl Info {
    pub(crate) fn regs(&self) -> &RegisterBlock {
        unsafe { &*self.register_block }
    }
}

unsafe impl Sync for Info {}

/// I2C Slave peripheral instance.
pub trait Instance: crate::private::Sealed + any::Degrade {
    #[doc(hidden)]
    fn parts(&self) -> (&Info, &State);

    #[doc(hidden)]
    #[inline(always)]
    fn info(&self) -> &Info {
        self.parts().0
    }

    #[doc(hidden)]
    #[inline(always)]
    fn state(&self) -> &State {
        self.parts().1
    }
}

#[allow(dead_code)]
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

    fn init_slave(&self) {
        self.regs().ctr().write(|w| {
            w.ms_mode().clear_bit(); // Slave mode
            w.sda_force_out().set_bit();
            w.scl_force_out().set_bit();
            #[cfg(i2c_master_has_arbitration_en)]
            w.arbitration_en().clear_bit();
            w.tx_lsb_first().clear_bit();
            w.rx_lsb_first().clear_bit();
            w
        });

        self.reset_fifo();

        self.regs().fifo_conf().modify(|_, w| {
            w.nonfifo_en().clear_bit();
            w
        });
    }

    fn set_slave_addr(&self, address: I2cAddress) -> Result<(), ConfigError> {
        match address {
            I2cAddress::SevenBit(addr) if addr <= 0x7F => {
                self.regs().slave_addr().write(|w| unsafe {
                    w.slave_addr().bits(addr as u16);
                    w.addr_10bit_en().bit(false)
                });
                Ok(())
            }
            _ => Err(ConfigError::AddressInvalid),
        }
    }

    fn update_registers(&self) {
        #[cfg(i2c_master_has_conf_update)]
        self.regs().ctr().modify(|_, w| w.conf_upgate().set_bit());
    }

    fn setup(&self, config: &Config) -> Result<(), ConfigError> {
        self.init_slave();
        self.set_slave_addr(config.address)?;

        let regs = self.regs();
        regs.to().modify(|_, w| unsafe {
            w.time_out_en().set_bit();
            w.time_out_value().bits(14);
            w
        });

        regs.clk_conf().modify(|_, w| w.sclk_sel().clear_bit());
        self.set_slave_addr(config.address)?;

        regs.ctr()
            .modify(|_, w| w.addr_broadcasting_en().clear_bit());

        regs.fifo_conf().modify(|_, w| {
            w.fifo_prt_en().set_bit();
            unsafe { w.txfifo_wm_thrhd().bits(0) }
        });

        regs.fifo_conf().modify(|_, w| w.fifo_prt_en().set_bit());
        regs.ctr().write(|w| w.rx_full_ack_level().clear_bit());

        regs.fifo_conf()
            .modify(|_, w| unsafe { w.rxfifo_wm_thrhd().bits(0x1F) });

        regs.sda_hold().write(|w| unsafe { w.time().bits(10u16) });
        regs.sda_sample().write(|w| unsafe { w.time().bits(10u16) });

        regs.int_ena().write(|w| unsafe { w.bits(0) });
        regs.int_clr().write(|w| unsafe { w.bits(0x3fff) });

        regs.scl_stretch_conf().modify(|_, w| unsafe {
            w.stretch_protect_num().bits(0x3ff);
            w.slave_scl_stretch_en().set_bit()
        });

        regs.scl_stretch_conf()
            .modify(|_, w| w.slave_scl_stretch_clr().set_bit());
        self.update_registers();

        Ok(())
    }

    fn reset_fifo(&self) {
        self.regs()
            .fifo_conf()
            .modify(|_, w| w.tx_fifo_rst().set_bit());
        self.regs()
            .fifo_conf()
            .modify(|_, w| w.tx_fifo_rst().clear_bit());
        self.regs()
            .fifo_conf()
            .modify(|_, w| w.rx_fifo_rst().set_bit());
        self.regs()
            .fifo_conf()
            .modify(|_, w| w.rx_fifo_rst().clear_bit());
    }

    fn reset_fsm(&self) {
        #[cfg(i2c_master_has_reliable_fsm_reset)]
        {
            self.regs().ctr().modify(|_, w| w.fsm_rst().set_bit());
        }
        #[cfg(not(i2c_master_has_reliable_fsm_reset))]
        {
            crate::system::PeripheralClockControl::reset(self.info.peripheral);
            self.setup(&self.config.config).ok();
        }
    }

    fn check_errors(&self) -> Result<(), Error> {
        let r = self.regs().int_raw().read();

        if r.arbitration_lost().bit_is_set() {
            return Err(Error::ArbitrationLost);
        }
        if r.time_out().bit_is_set() {
            return Err(Error::Timeout);
        }
        Ok(())
    }

    fn listen_step(
        &self,
        buffer: &mut [u8],
        bytes_read: &mut usize,
    ) -> Option<Result<Command, Error>> {
        let regs = self.regs();
        let ints = regs.int_raw().read();

        if ints.slave_stretch().bit_is_set() {
            let cause = regs.sr().read().stretch_cause().bits();
            match cause {
                0 => {
                    let rw = regs.sr().read().slave_rw().bit_is_set();
                    if rw {
                        regs.int_clr()
                            .write(|w| w.slave_stretch().clear_bit_by_one());
                        return Some(Ok(Command::Read));
                    } else {
                        regs.scl_stretch_conf()
                            .modify(|_, w| w.slave_scl_stretch_clr().set_bit());
                        regs.int_clr()
                            .write(|w| w.slave_stretch().clear_bit_by_one());
                    }
                }
                2 => {
                    while regs.sr().read().rxfifo_cnt().bits() > 0 && *bytes_read < buffer.len() {
                        buffer[*bytes_read] = regs.data().read().fifo_rdata().bits();
                        *bytes_read += 1;
                    }
                    regs.scl_stretch_conf()
                        .modify(|_, w| w.slave_scl_stretch_clr().set_bit());
                    regs.int_clr()
                        .write(|w| w.slave_stretch().clear_bit_by_one());
                }
                _ => {
                    regs.scl_stretch_conf()
                        .modify(|_, w| w.slave_scl_stretch_clr().set_bit());
                    regs.int_clr()
                        .write(|w| w.slave_stretch().clear_bit_by_one());
                }
            }
        }

        while regs.sr().read().rxfifo_cnt().bits() > 0 && *bytes_read < buffer.len() {
            buffer[*bytes_read] = regs.data().read().fifo_rdata().bits();
            *bytes_read += 1;
        }

        if ints.trans_complete().bit_is_set() {
            regs.int_clr()
                .write(|w| w.trans_complete().clear_bit_by_one());
            return Some(Ok(Command::Write(*bytes_read)));
        }

        None
    }

    fn respond_to_read_step(
        &self,
        buffer: &[u8],
        bytes_written: &mut usize,
    ) -> Option<Result<ReadStatus, Error>> {
        let regs = self.regs();
        let ints = regs.int_raw().read();

        if ints.trans_complete().bit_is_set() || ints.nack().bit_is_set() {
            let is_nack = ints.nack().bit_is_set();
            regs.int_clr().write(|w| {
                w.trans_complete().clear_bit_by_one();
                w.nack().clear_bit_by_one()
            });
            if is_nack {
                self.reset_fifo();
                self.reset_fsm();
            }

            if *bytes_written > buffer.len() {
                return Some(Ok(ReadStatus::NeedMoreBytes));
            } else if *bytes_written < buffer.len() {
                let rem = (buffer.len() - *bytes_written) as u16;
                return Some(Ok(ReadStatus::LeftoverBytes(rem)));
            } else {
                return Some(Ok(ReadStatus::Done));
            }
        }

        if ints.slave_stretch().bit_is_set() {
            let cause = regs.sr().read().stretch_cause().bits();
            if cause == 1 {
                if *bytes_written < buffer.len() {
                    let txfifo_cnt = regs.sr().read().txfifo_cnt().bits() as usize;
                    let free = FIFO_SIZE - txfifo_cnt;
                    let to_write = core::cmp::min(buffer.len() - *bytes_written, free);
                    for i in 0..to_write {
                        regs.data()
                            .write(|w| unsafe { w.fifo_rdata().bits(buffer[*bytes_written + i]) });
                    }
                    *bytes_written += to_write;

                    regs.scl_stretch_conf()
                        .modify(|_, w| w.slave_scl_stretch_clr().set_bit());
                    regs.int_clr()
                        .write(|w| w.slave_stretch().clear_bit_by_one());
                    self.update_registers();
                } else {
                    regs.data().write(|w| unsafe { w.fifo_rdata().bits(0xFF) });
                    *bytes_written += 1;
                    regs.scl_stretch_conf()
                        .modify(|_, w| w.slave_scl_stretch_clr().set_bit());
                    regs.int_clr()
                        .write(|w| w.slave_stretch().clear_bit_by_one());
                    self.update_registers();
                }
            } else {
                regs.scl_stretch_conf()
                    .modify(|_, w| w.slave_scl_stretch_clr().set_bit());
                regs.int_clr()
                    .write(|w| w.slave_stretch().clear_bit_by_one());
                self.update_registers();
            }
        }

        None
    }
}

#[cfg(i2c_slave_i2c0)]
impl crate::i2c::slave::Instance for crate::peripherals::I2C0<'_> {
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
            register_block: crate::peripherals::I2C0::ptr(),
            peripheral: crate::system::Peripheral::I2cExt0,
            async_handler: irq_handler,
            scl_output: OutputSignal::I2CEXT0_SCL,
            scl_input: InputSignal::I2CEXT0_SCL,
            sda_output: OutputSignal::I2CEXT0_SDA,
            sda_input: InputSignal::I2CEXT0_SDA,
        };
        (&PERIPHERAL, &STATE)
    }
}

#[cfg(i2c_slave_i2c1)]
impl crate::i2c::slave::Instance for crate::peripherals::I2C1<'_> {
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
            register_block: crate::peripherals::I2C1::ptr(),
            peripheral: crate::system::Peripheral::I2cExt1,
            async_handler: irq_handler,
            scl_output: OutputSignal::I2CEXT1_SCL,
            scl_input: InputSignal::I2CEXT1_SCL,
            sda_output: OutputSignal::I2CEXT1_SDA,
            sda_input: InputSignal::I2CEXT1_SDA,
        };
        (&PERIPHERAL, &STATE)
    }
}

crate::any_peripheral! {
    /// Any I2C peripheral in slave mode.
    pub peripheral AnyI2c<'d> {
        #[cfg(i2c_slave_i2c0)]
        I2c0(crate::peripherals::I2C0<'d>),
        #[cfg(i2c_slave_i2c1)]
        I2c1(crate::peripherals::I2C1<'d>),
    }
}

impl crate::i2c::slave::Instance for AnyI2c<'_> {
    fn parts(&self) -> (&Info, &State) {
        any::delegate!(self, i2c => { crate::i2c::slave::Instance::parts(i2c) })
    }
}

impl AnyI2c<'_> {
    fn bind_peri_interrupt(&self, handler: InterruptHandler) {
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

        self.info().regs().int_ena().write(|w| unsafe { w.bits(0) });
        self.info()
            .regs()
            .int_clr()
            .write(|w| unsafe { w.bits(0x3fff) });

        self.bind_peri_interrupt(handler);
        self.enable_peri_interrupt(handler.priority());
    }
}
