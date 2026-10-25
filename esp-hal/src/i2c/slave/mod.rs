//! # Inter-Integrated Circuit (I2C) - Slave mode
//!
//! ## Overview
//!
//! Receives writes and responds to read requests at a 7-bit I2C address.
//! The driver uses SCL clock stretching for flow control in [`Blocking`] and [`Async`] modes.
//! Writes stretch SCL at each data byte's ACK until the driver consumes that byte.
//!
//! ## Timeouts
//!
//! Operations have no software time limit and can wait indefinitely.
//! [`Error::Timeout`] reports a detected hardware bus timeout.
//! A later request's clock stretch can hide an earlier bus timeout.
//!
//! Operations reset the controller before returning [`Error::Timeout`] or
//! [`Error::ArbitrationLost`]. This releases the slave's outputs and discards pending requests.
//! The master must retry these requests.

use core::{
    future::Future,
    marker::PhantomData,
    pin::Pin,
    sync::atomic::{AtomicBool, Ordering},
    task::{Context, Poll},
};

use enumset::{EnumSet, EnumSetType};

use crate::{
    Async,
    Blocking,
    DriverMode,
    asynch::AtomicWaker,
    clock::ll::{ClockTree, I2cFunctionClockConfig, I2cInstance},
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
    rtc_cntl::WakeLock,
    system::PeripheralGuard,
};

/// FIFO size of the I2C peripheral.
pub const FIFO_SIZE: usize = property!("i2c_master.fifo_size");

/// I2C slave errors.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// A hardware bus timeout is detected.
    Timeout,
    /// Bus arbitration is lost.
    ArbitrationLost,
    /// The receive buffer is too small, or the RX FIFO overflows.
    Overflow,
    /// The unread response exceeds 65,535 bytes and cannot fit in [`ReadStatus::LeftoverBytes`].
    ResponseTooLong,
    /// The master ends a read without a NACK.
    ReadAborted,
}

impl core::error::Error for Error {}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(match self {
            Self::Timeout => "Hardware bus timeout",
            Self::ArbitrationLost => "Bus arbitration lost",
            Self::Overflow => "Receive buffer or RX FIFO overflow",
            Self::ResponseTooLong => "Unread response length exceeds 65,535 bytes",
            Self::ReadAborted => "Read ended without a NACK",
        })
    }
}

impl embedded_hal::i2c::Error for Error {
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        match self {
            Self::ArbitrationLost => embedded_hal::i2c::ErrorKind::ArbitrationLoss,
            Self::Overflow => embedded_hal::i2c::ErrorKind::Overrun,
            Self::ReadAborted => embedded_hal::i2c::ErrorKind::Bus,
            _ => embedded_hal::i2c::ErrorKind::Other,
        }
    }
}

/// I2C slave configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, procmacros::BuilderLite)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Config {
    /// The I2C slave address. Only 7-bit addresses are supported.
    address: I2cAddress,
}

impl Config {
    /// Creates a new I2C slave configuration with the specified address.
    ///
    /// The driver validates the address during construction or reconfiguration.
    pub fn new(address: impl Into<I2cAddress>) -> Self {
        Config {
            address: address.into(),
        }
    }
}

/// I2C slave configuration errors.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum ConfigError {
    /// The address is not a valid 7-bit address.
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

/// Request received from the master.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Command {
    /// The master requests data from the slave.
    Read,
    /// A write request with the number of received bytes.
    Write(usize),
}

/// Result of a response to a master read request.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ReadStatus {
    /// The master reads exactly the provided number of bytes.
    Done,
    /// The master reads beyond the provided data and receives `0xff` padding.
    NeedMoreBytes,
    /// The number of provided bytes left unread.
    LeftoverBytes(u16),
}

/// I2C slave driver.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct I2cSlave<'d, Dm: DriverMode> {
    i2c: AnyI2c<'d>,
    phantom: PhantomData<Dm>,
    // Release the function clock before disabling peripheral register access.
    clock_guard: I2cClockGuard,
    guard: PeripheralGuard,
    sda_pin: PinGuard,
    scl_pin: PinGuard,
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct I2cClockGuard {
    clock: I2cInstance,
}

impl I2cClockGuard {
    fn new(clock: I2cInstance) -> Self {
        ClockTree::with(|clocks| {
            let config = I2cFunctionClockConfig::new(
                Default::default(),
                #[cfg(any(i2c_master_version = "3", i2c_master_version = "4"))]
                0,
            );
            clock.configure_function_clock(clocks, config);
            clock.request_function_clock(clocks);
        });
        Self { clock }
    }
}

impl Drop for I2cClockGuard {
    fn drop(&mut self) {
        ClockTree::with(|clocks| self.clock.release_function_clock(clocks));
    }
}

impl<Dm: DriverMode> embedded_hal::i2c::ErrorType for I2cSlave<'_, Dm> {
    type Error = Error;
}

impl<'d> I2cSlave<'d, Blocking> {
    /// Creates a new I2C slave instance in [`Blocking`] mode.
    ///
    /// # Errors
    ///
    /// [`ConfigError::AddressInvalid`] when the address is not a valid 7-bit address.
    pub fn new(i2c: impl Instance + 'd, config: Config) -> Result<Self, ConfigError> {
        let guard = PeripheralGuard::new(i2c.info().peripheral);
        let clock_guard = I2cClockGuard::new(i2c.info().clock_instance);

        let sda_pin = PinGuard::new_unconnected();
        let scl_pin = PinGuard::new_unconnected();

        let mut slave = I2cSlave {
            i2c: i2c.degrade(),
            phantom: PhantomData,
            clock_guard,
            guard,
            sda_pin,
            scl_pin,
        };

        slave.apply_config(&config)?;
        Ok(slave)
    }

    /// Reconfigures the driver to operate in [`Async`] mode.
    ///
    /// Pending requests are preserved.
    pub fn into_async(self) -> I2cSlave<'d, Async> {
        self.i2c
            .set_interrupt_handler(self.driver().info.async_handler);

        I2cSlave {
            i2c: self.i2c,
            phantom: PhantomData,
            clock_guard: self.clock_guard,
            guard: self.guard,
            sda_pin: self.sda_pin,
            scl_pin: self.scl_pin,
        }
    }
}

impl<'d> I2cSlave<'d, Async> {
    /// Reconfigures the driver to operate in [`Blocking`] mode.
    ///
    /// Pending requests are preserved.
    pub fn into_blocking(self) -> I2cSlave<'d, Blocking> {
        self.i2c.disable_peri_interrupt();

        I2cSlave {
            i2c: self.i2c,
            phantom: PhantomData,
            clock_guard: self.clock_guard,
            guard: self.guard,
            sda_pin: self.sda_pin,
            scl_pin: self.scl_pin,
        }
    }

    /// Listens for a request from the master asynchronously.
    ///
    /// See [`Self::listen`] for the receive behavior and errors.
    pub async fn listen_async(&mut self, buffer: &mut [u8]) -> Result<Command, Error> {
        let driver = self.driver();
        let regs = driver.regs();
        let state = self.i2c.state();

        driver.start_listen();

        let mut bytes_read = 0;
        loop {
            I2cSlaveFuture::new(
                regs,
                state,
                Event::SlaveStretch
                    | Event::TransComplete
                    | Event::StartDetected
                    | Event::ReceiveBoundary,
            )
            .await;
            if let Some(res) = driver.listen_step(buffer, &mut bytes_read) {
                return res;
            }
        }
    }

    /// Responds to a master read request asynchronously.
    ///
    /// See [`Self::respond_to_read`] for the response behavior and errors.
    pub async fn respond_to_read_async(&mut self, buffer: &[u8]) -> Result<ReadStatus, Error> {
        let driver = self.driver();
        let regs = driver.regs();
        let state = self.i2c.state();

        let mut bytes_written = 1;
        driver.start_read(buffer);

        loop {
            I2cSlaveFuture::new(
                regs,
                state,
                Event::SlaveStretch | Event::Nack | Event::TransComplete | Event::StartDetected,
            )
            .await;

            if let Some(status) = driver.respond_to_read_step(buffer, &mut bytes_written) {
                return status;
            }
        }
    }
}

impl<'d, Dm: DriverMode> I2cSlave<'d, Dm> {
    fn driver(&self) -> Driver<'_> {
        let (info, state) = self.i2c.parts();
        Driver { info, state }
    }

    /// Connects a pin to the I2C SDA signal.
    pub fn with_sda(mut self, sda: impl PeripheralInput<'d> + PeripheralOutput<'d>) -> Self {
        let info = self.driver().info;
        let input = info.sda_input;
        let output = info.sda_output;
        Driver::connect_pin(sda.into(), input, output, &mut self.sda_pin);
        self
    }

    /// Connects a pin to the I2C SCL signal.
    pub fn with_scl(mut self, scl: impl PeripheralInput<'d> + PeripheralOutput<'d>) -> Self {
        let info = self.driver().info;
        let input = info.scl_input;
        let output = info.scl_output;
        Driver::connect_pin(scl.into(), input, output, &mut self.scl_pin);
        self
    }

    /// Applies a new configuration.
    ///
    /// A valid configuration resets the controller and discards pending requests.
    /// The master must retry these requests.
    /// Invalid configurations preserve the current settings and FIFO contents.
    ///
    /// # Errors
    ///
    /// [`ConfigError::AddressInvalid`] when the address is not a valid 7-bit address.
    pub fn apply_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        self.driver().setup(config)
    }

    /// Listens for a request from the master.
    ///
    /// Receives writes into `buffer` and returns [`Command::Write`] with the number
    /// of received bytes at STOP or repeated START. Address-only write probes are ignored.
    /// Extra bytes are discarded if `buffer` is too small.
    ///
    /// Returns [`Command::Read`] for a read request and holds SCL low until
    /// [`Self::respond_to_read`] supplies the response.
    /// Pending receive data is preserved between calls.
    /// The controller stretches SCL at each write byte's ACK until this method consumes the byte.
    ///
    /// If a bus error is detected when this method starts, it resets the controller
    /// and discards pending data.
    ///
    /// # Errors
    ///
    /// [`Error::Overflow`] when the receive buffer is too small or the RX FIFO overflows.
    /// The error is reported after STOP or repeated START.
    ///
    /// [`Error::Timeout`] when a hardware bus timeout is detected.
    /// [`Error::ArbitrationLost`] when arbitration is lost.
    pub fn listen(&mut self, buffer: &mut [u8]) -> Result<Command, Error> {
        let driver = self.driver();
        driver.start_listen();

        let mut bytes_read = 0;
        loop {
            if let Some(res) = driver.listen_step(buffer, &mut bytes_read) {
                return res;
            }
        }
    }

    /// Responds to a master read request.
    ///
    /// A listen operation must first return [`Command::Read`].
    /// Sends data from `buffer` and releases SCL clock stretching.
    /// Completes when the master sends a NACK.
    /// Data is padded with `0xff` if the master reads past the end of `buffer`.
    ///
    /// # Errors
    ///
    /// [`Error::ResponseTooLong`] when more than 65,535 provided bytes remain unread.
    /// The response completes, but the remaining count cannot fit in [`ReadStatus::LeftoverBytes`].
    ///
    /// [`Error::ReadAborted`] when the master ends the read without a NACK.
    /// The controller resets and discards pending requests.
    /// The master must retry these requests.
    ///
    /// [`Error::Timeout`] when a hardware bus timeout is detected.
    /// [`Error::ArbitrationLost`] when arbitration is lost.
    pub fn respond_to_read(&mut self, buffer: &[u8]) -> Result<ReadStatus, Error> {
        let driver = self.driver();
        let mut bytes_written = 1;
        driver.start_read(buffer);

        loop {
            if let Some(status) = driver.respond_to_read_step(buffer, &mut bytes_written) {
                return status;
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
    StartDetected,
    ReceiveBoundary,
}

#[must_use = "futures do nothing unless you `.await` or poll them"]
struct I2cSlaveFuture<'a> {
    regs: &'a RegisterBlock,
    state: &'a State,
    events: EnumSet<Event>,
    _wake_lock: WakeLock,
}

impl<'a> I2cSlaveFuture<'a> {
    fn new(regs: &'a RegisterBlock, state: &'a State, events: EnumSet<Event>) -> Self {
        Self {
            regs,
            state,
            events,
            _wake_lock: WakeLock::new(),
        }
    }
}

impl Future for I2cSlaveFuture<'_> {
    type Output = ();

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        self.state.waker.register(cx.waker());

        let raw = self.regs.int_raw().read();
        let triggered = (self.events.contains(Event::SlaveStretch)
            && raw.slave_stretch().bit_is_set())
            || (self.events.contains(Event::TransComplete) && raw.trans_complete().bit_is_set())
            || (self.events.contains(Event::Nack) && raw.nack().bit_is_set())
            || (self.events.contains(Event::StartDetected) && raw.det_start().bit_is_set());

        let boundary_pending = self.events.contains(Event::ReceiveBoundary)
            && self.state.receive_boundary().is_pending()
            && {
                let status = self.regs.sr().read();
                status.slave_addressed().bit_is_clear()
                    || status.slave_rw().bit_is_set()
                    || Driver::address_pending(&status)
            };

        if triggered || boundary_pending || Driver::check_errors(self.regs).is_err() {
            Poll::Ready(())
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
                if self.events.contains(Event::StartDetected) {
                    w.det_start().set_bit();
                }
                w.arbitration_lost().set_bit();
                w.time_out().set_bit();
                w.scl_st_to().set_bit();
                w.scl_main_st_to().set_bit();
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

#[derive(Debug, Default, Clone, Copy)]
struct ReceiveBoundary {
    start: bool,
    stop: bool,
}

impl ReceiveBoundary {
    fn is_pending(self) -> bool {
        self.start || self.stop
    }
}

// Public for the sealed instance interface.
#[doc(hidden)]
#[non_exhaustive]
pub struct State {
    pub(crate) waker: AtomicWaker,
    rx_start_pending: AtomicBool,
    rx_stop_pending: AtomicBool,
}

impl State {
    fn receive_boundary(&self) -> ReceiveBoundary {
        ReceiveBoundary {
            start: self.rx_start_pending.load(Ordering::Relaxed),
            stop: self.rx_stop_pending.load(Ordering::Relaxed),
        }
    }

    fn set_receive_boundary(&self, boundary: ReceiveBoundary) {
        self.rx_start_pending
            .store(boundary.start, Ordering::Relaxed);
        self.rx_stop_pending.store(boundary.stop, Ordering::Relaxed);
    }
}

impl core::fmt::Debug for State {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("State")
            .field("waker", &"<AtomicWaker>")
            .field("receive_boundary", &self.receive_boundary())
            .finish()
    }
}

// Public for the sealed instance interface.
#[doc(hidden)]
#[derive(Debug)]
#[non_exhaustive]
pub struct Info {
    pub(crate) register_block: *const RegisterBlock,
    pub(crate) peripheral: crate::system::Peripheral,
    pub(crate) async_handler: InterruptHandler,
    pub(crate) clock_instance: I2cInstance,
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

// Public constructor bound; implementations are sealed.
#[doc(hidden)]
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

struct Driver<'a> {
    info: &'a Info,
    state: &'a State,
}

impl Driver<'_> {
    // Stretch-cause encodings for ESP32-C3 and ESP32-C6:
    // https://github.com/espressif/esp-idf/blob/d445e947156f276dfe0db5804e4edf6ae90fe6fb/components/esp_hal_i2c/esp32c3/include/hal/i2c_ll.h#L363-L377
    const STRETCH_ADDRESS_MATCH: u8 = 0;
    const STRETCH_TX_EMPTY: u8 = 1;
    const STRETCH_RX_FULL: u8 = 2;

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
            w.ms_mode().clear_bit();
            w.rx_full_ack_level().clear_bit();
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

    fn update_registers(&self) {
        #[cfg(i2c_master_has_conf_update)]
        self.regs().ctr().modify(|_, w| w.conf_upgate().set_bit());
    }

    fn setup(&self, config: &Config) -> Result<(), ConfigError> {
        let address = match config.address {
            I2cAddress::SevenBit(addr) if addr <= 0x7F => addr,
            _ => return Err(ConfigError::AddressInvalid),
        };

        self.state.set_receive_boundary(ReceiveBoundary::default());
        // FIFO resets do not clear an active transaction.
        crate::system::PeripheralClockControl::reset(self.info.peripheral);
        self.init_slave();

        let regs = self.regs();
        regs.slave_addr().write(|w| unsafe {
            w.slave_addr().bits(address as u16);
            w.addr_10bit_en().clear_bit()
        });
        regs.to().modify(|_, w| unsafe {
            w.time_out_en().set_bit();
            w.time_out_value().bits(14);
            w
        });

        regs.clk_conf().modify(|_, w| w.sclk_sel().clear_bit());

        regs.ctr()
            .modify(|_, w| w.addr_broadcasting_en().clear_bit());

        regs.fifo_conf().modify(|_, w| {
            w.fifo_prt_en().set_bit();
            unsafe { w.txfifo_wm_thrhd().bits(0) }
        });

        regs.fifo_conf()
            .modify(|_, w| unsafe { w.rxfifo_wm_thrhd().bits(0x1F) });

        // SDA hold/sample periods match ESP-IDF.
        // https://github.com/espressif/esp-idf/blob/d445e947156f276dfe0db5804e4edf6ae90fe6fb/components/esp_driver_i2c/i2c_slave.c#L303
        regs.sda_hold().write(|w| unsafe { w.time().bits(10u16) });
        regs.sda_sample().write(|w| unsafe { w.time().bits(10u16) });

        regs.int_ena().write(|w| unsafe { w.bits(0) });
        regs.int_clr().write(|w| unsafe { w.bits(0x3fff) });

        regs.scl_stretch_conf().modify(|_, w| unsafe {
            w.stretch_protect_num().bits(0x3ff);
            // Service each byte before the master can cross a transaction boundary.
            w.slave_byte_ack_ctl_en().set_bit();
            w.slave_byte_ack_lvl().clear_bit();
            w.slave_scl_stretch_en().set_bit()
        });

        regs.scl_stretch_conf()
            .modify(|_, w| w.slave_scl_stretch_clr().set_bit());
        self.update_registers();

        Ok(())
    }

    fn reset_tx_fifo(&self) {
        let regs = self.regs();
        regs.fifo_conf().modify(|_, w| w.tx_fifo_rst().set_bit());
        regs.fifo_conf().modify(|_, w| w.tx_fifo_rst().clear_bit());
    }

    fn reset_fifo(&self) {
        self.reset_tx_fifo();
        let regs = self.regs();
        regs.fifo_conf().modify(|_, w| w.rx_fifo_rst().set_bit());
        regs.fifo_conf().modify(|_, w| w.rx_fifo_rst().clear_bit());
    }

    fn check_errors(regs: &RegisterBlock) -> Result<(), Error> {
        let r = regs.int_raw().read();

        if r.arbitration_lost().bit_is_set() {
            return Err(Error::ArbitrationLost);
        }
        // The timeout counter also runs while the slave holds SCL low.
        // A later stretch can hide an earlier timeout; preserve the pending request.
        if r.time_out().bit_is_set()
            || r.scl_st_to().bit_is_set()
            || r.scl_main_st_to().bit_is_set()
        {
            let status = regs.sr().read();
            // ACK stretching can retain the previous TX stretch cause.
            let tx_finished = status.slave_rw().bit_is_set()
                && status.stretch_cause().bits() == Self::STRETCH_TX_EMPTY
                && r.nack().bit_is_set();
            if r.slave_stretch().bit_is_clear() || tx_finished {
                return Err(Error::Timeout);
            }
        }
        Ok(())
    }

    fn release_stretch(&self) {
        let regs = self.regs();
        // Restart timeout counting when the master regains control of SCL.
        regs.to().modify(|_, w| w.time_out_en().clear_bit());
        self.update_registers();
        regs.int_clr().write(|w| {
            w.slave_stretch().clear_bit_by_one();
            w.scl_st_to().clear_bit_by_one();
            w.scl_main_st_to().clear_bit_by_one();
            w.time_out().clear_bit_by_one()
        });
        regs.scl_stretch_conf()
            .modify(|_, w| w.slave_scl_stretch_clr().set_bit());
        regs.to().modify(|_, w| w.time_out_en().set_bit());
        self.update_registers();
    }

    fn recover(&self) {
        let address = self.regs().slave_addr().read().slave_addr().bits() as u8;
        self.setup(&Config::new(address)).ok();
    }

    fn start_listen(&self) {
        if Self::check_errors(self.regs()).is_err() {
            self.recover();
        } else {
            self.reset_tx_fifo();
        }
    }

    fn address_pending(status: &crate::pac::i2c0::sr::R) -> bool {
        // SLAVE_ADDRESSED retains the preceding match during address shift/ACK.
        matches!(status.scl_main_state_last().bits(), 1 | 2)
    }

    fn take_receive_events(&self, boundary: &mut ReceiveBoundary) -> crate::pac::i2c0::int_raw::R {
        let regs = self.regs();
        let ints = regs.int_raw().read();
        boundary.start |= ints.det_start().bit_is_set();
        boundary.stop |= ints.trans_complete().bit_is_set();
        // Acknowledge boundaries before sampling the FIFO they delimit.
        regs.int_clr().write(|w| {
            w.trans_complete().bit(ints.trans_complete().bit());
            w.det_start().bit(ints.det_start().bit())
        });
        ints
    }

    fn listen_step(
        &self,
        buffer: &mut [u8],
        bytes_read: &mut usize,
    ) -> Option<Result<Command, Error>> {
        let regs = self.regs();
        let error = Self::check_errors(regs).err();
        let mut boundary = self.state.receive_boundary();
        let mut ints = self.take_receive_events(&mut boundary);
        let mut status = regs.sr().read();
        // An ACK stretch can start between the interrupt and FIFO snapshots.
        if ints.slave_stretch().bit_is_clear() && regs.int_raw().read().slave_stretch().bit_is_set()
        {
            ints = self.take_receive_events(&mut boundary);
            status = regs.sr().read();
        }

        let fifo_overflow = ints.rxfifo_ovf().bit_is_set();
        let stretched = ints.slave_stretch().bit_is_set();
        let read_pending = stretched
            && status.stretch_cause().bits() == Self::STRETCH_ADDRESS_MATCH
            && status.slave_rw().bit_is_set();
        let fifo_count = status.rxfifo_cnt().bits() as usize;
        let addressed = status.slave_addressed().bit_is_set() && !Self::address_pending(&status);
        let active_write = addressed && status.slave_rw().bit_is_clear();
        let next_write_byte = stretched
            && active_write
            // Byte-ACK stretching can retain an earlier RX-full cause.
            && (fifo_count < FIFO_SIZE || status.stretch_cause().bits() != Self::STRETCH_RX_FULL);
        // FIFO insertion precedes ACK stretching. Until ACK, a buffered byte
        // may belong to either side of a previously observed boundary.
        let wait_for_ack = boundary.is_pending() && fifo_count > 0 && active_write && !stretched;
        let previous_bytes = fifo_count.saturating_sub(usize::from(next_write_byte));
        let write_ended = !wait_for_ack
            && ((boundary.stop && (*bytes_read > 0 || previous_bytes > 0 || fifo_overflow))
                || (boundary.start
                    && (*bytes_read > 0
                        || (previous_bytes > 0 && (next_write_byte || !addressed)))));

        // A following request's timeout must not discard a completed write.
        if let Some(error) = error {
            if error != Error::Timeout || !write_ended {
                self.recover();
                return Some(Err(error));
            }
        }

        if wait_for_ack {
            self.state.set_receive_boundary(boundary);
            return None;
        }
        self.state.set_receive_boundary(ReceiveBoundary::default());

        if stretched || write_ended || fifo_overflow {
            let count = if write_ended {
                previous_bytes
            } else {
                fifo_count
            };
            for _ in 0..count {
                let byte = regs.data().read().fifo_rdata().bits();
                if let Some(dest) = buffer.get_mut(*bytes_read) {
                    *dest = byte;
                }
                *bytes_read = bytes_read.saturating_add(1);
            }
        }

        let overflow = *bytes_read > buffer.len() || fifo_overflow;
        let write_pending = *bytes_read > 0 || overflow;
        let write_complete = read_pending || write_ended;
        // Keep the next request stretched until the next listen call.
        if write_pending && write_complete {
            regs.int_clr().write(|w| w.rxfifo_ovf().clear_bit_by_one());
            if fifo_overflow && status.stretch_cause().bits() == Self::STRETCH_RX_FULL {
                self.release_stretch();
            }
            return Some(if overflow {
                Err(Error::Overflow)
            } else {
                Ok(Command::Write(*bytes_read))
            });
        }

        if stretched {
            if read_pending {
                regs.int_clr()
                    .write(|w| w.slave_stretch().clear_bit_by_one());
                return Some(Ok(Command::Read));
            }
            self.release_stretch();
        }

        None
    }

    fn start_read(&self, buffer: &[u8]) {
        let regs = self.regs();
        // Queue one byte at a time to count short reads correctly.
        regs.data()
            .write(|w| unsafe { w.fifo_rdata().bits(buffer.first().copied().unwrap_or(0xff)) });

        // Clear the current START before releasing the address stretch.
        regs.int_clr().write(|w| unsafe {
            w.bits(0x3fff);
            w.det_start().clear_bit_by_one()
        });
        self.release_stretch();
    }

    fn respond_to_read_step(
        &self,
        buffer: &[u8],
        bytes_written: &mut usize,
    ) -> Option<Result<ReadStatus, Error>> {
        let regs = self.regs();
        let error = Self::check_errors(regs).err();
        let ints = regs.int_raw().read();

        // A later timeout must not replace a completed read.
        if let Some(error) = error {
            if error != Error::Timeout || ints.nack().bit_is_clear() {
                self.recover();
                return Some(Err(error));
            }
        }

        if ints.nack().bit_is_set()
            || ints.trans_complete().bit_is_set()
            || ints.det_start().bit_is_set()
        {
            if ints.nack().bit_is_clear() {
                self.recover();
                return Some(Err(Error::ReadAborted));
            }
            let bytes_read =
                bytes_written.saturating_sub(regs.sr().read().txfifo_cnt().bits() as usize);
            // START and STOP can already belong to the next request.
            regs.int_clr().write(|w| w.nack().clear_bit_by_one());
            self.reset_tx_fifo();
            let status = if bytes_read > buffer.len() {
                Ok(ReadStatus::NeedMoreBytes)
            } else if bytes_read < buffer.len() {
                u16::try_from(buffer.len() - bytes_read)
                    .map(ReadStatus::LeftoverBytes)
                    .map_err(|_| Error::ResponseTooLong)
            } else {
                Ok(ReadStatus::Done)
            };
            return Some(status);
        }

        if ints.slave_stretch().bit_is_set() {
            if regs.sr().read().stretch_cause().bits() == Self::STRETCH_TX_EMPTY {
                let byte = buffer.get(*bytes_written).copied().unwrap_or(0xff);
                regs.data().write(|w| unsafe { w.fifo_rdata().bits(byte) });
                *bytes_written = bytes_written.saturating_add(1);
            }
            self.release_stretch();
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
            rx_start_pending: AtomicBool::new(false),
            rx_stop_pending: AtomicBool::new(false),
        };

        static PERIPHERAL: Info = Info {
            register_block: crate::peripherals::I2C0::ptr(),
            peripheral: crate::system::Peripheral::I2cExt0,
            async_handler: irq_handler,
            clock_instance: I2cInstance::I2c0,
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
            rx_start_pending: AtomicBool::new(false),
            rx_stop_pending: AtomicBool::new(false),
        };

        static PERIPHERAL: Info = Info {
            register_block: crate::peripherals::I2C1::ptr(),
            peripheral: crate::system::Peripheral::I2cExt1,
            async_handler: irq_handler,
            clock_instance: I2cInstance::I2c1,
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

        self.bind_peri_interrupt(handler);
        self.enable_peri_interrupt(handler.priority());
    }
}
