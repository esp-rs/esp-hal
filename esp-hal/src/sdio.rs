//! # Secure Digital I/O - Slave Mode
//!
//! ## Overiew
//!
//! The peripheral can be used to transfer data over the SDIO bus in `Slave`
//! mode.

use embedded_hal_sdmmc::{
    CardMode,
    CardType,
    Common,
    Device,
    FifoStatus,
    Reset,
    command::Command,
    response::Response,
    tuning::{TuningMode, TuningWidth},
};

mod pins;
mod slc;
mod slchost;
mod state;

pub use pins::Pins;
pub use slc::{AnySlc, SlcInfo, SlcInstance};
pub use slchost::{AnySlchost, SlchostInfo, SlchostInstance};
pub use state::State;

/// SDIO peripheral instance.
pub trait PeripheralInstance: crate::private::Sealed {
    /// Represents the peripheral information type containing the register
    /// block.
    type Info;

    /// Gets a static shared reference to the peripheral information.
    fn info(&self) -> &'static Self::Info;
}

/// Represents the transmission modes for the SDIO peripheral.
#[repr(u8)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Mode {
    /// SPI transmission mode.
    ///
    /// Uses the following I/O signals:
    ///
    /// - `SCLK`
    /// - `MOSI`
    /// - `MISO`
    /// - `IRQ`
    /// - `#CS`
    Spi,
    /// SD 1-bit transmission mode.
    ///
    /// Uses the following I/O signals:
    ///
    /// - `CLK`
    /// - `CMD`
    /// - `DAT0`
    /// - `IRQ`
    Sd1bit,
    /// SD 4-bit transmission mode.
    ///
    /// Uses the following I/O signals:
    ///
    /// - `CLK`
    /// - `CMD`
    /// - `DAT0`
    /// - `DAT1`
    /// - `DAT2`
    /// - `DAT3`
    Sd4bit,
}

/// Represents the SDIO 2.0 peripheral for the microcontroller.
#[derive(Debug)]
pub struct Sdio<'d> {
    slc: AnySlc<'d>,
    slchost: AnySlchost<'d>,
    pins: Pins<'d>,
    state: State,
}

impl<'d> Sdio<'d> {
    /// Creates a new [Sdio].
    ///
    /// # Example
    #[doc = crate::before_snippet!()]
    /// ```rust, no_run
    /// use esp_hal::sdio::{Mode, Pins, Sdio};
    ///
    /// let pins = Pins::new(
    ///     Mode::Sd4bit,
    ///     peripherals.GPIO19, // CLK/SCLK
    ///     peripherals.GPIO18, // CMD/MOSI
    ///     peripherals.GPIO20, // DAT0/MISO
    ///     peripherals.GPIO21, // DAT1/IRQ
    ///     peripherals.GPIO22, // DAT2
    ///     peripherals.GPIO23, // DAT3/#CS
    /// );
    ///
    /// let _sdio = Sdio::new(peripherals.slc, peripherals.slchost, pins);
    /// ```
    pub fn new(
        slc: impl SlcInstance + 'd,
        slchost: impl SlchostInstance + 'd,
        pins: Pins<'d>,
    ) -> Self {
        Self {
            slc: slc.degrade(),
            slchost: slchost.degrade(),
            pins,
            state: State::new(),
        }
    }

    /// Gets a static reference to the SLC information.
    pub fn slc(&self) -> &'static SlcInfo {
        self.slc.info()
    }

    /// Gets a static reference to the SLCHOST information.
    pub fn slchost(&self) -> &'static SlchostInfo {
        self.slchost.info()
    }

    /// Gets a reference to the [Pins] information.
    pub const fn pins(&self) -> &Pins<'_> {
        &self.pins
    }

    /// Gets the bus mode of the SDIO peripheral.
    pub const fn bus_mode(&self) -> Mode {
        self.pins.mode()
    }

    /// Gets the current [State] of the SDIO peripheral.
    pub const fn state(&self) -> State {
        self.state
    }

    /// Transitions the SDIO peripheral to the requested [State].
    pub(crate) fn state_transition(&mut self, state: State) -> Result<(), Error> {
        self.state
            .valid_transition(state)
            .map(|_| self.state = state)
    }
}

/// Represents the error variants for SDIO peripherals.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Error {
    /// Indicates a general error occured.
    General,
    /// Indicates use of an illegal command.
    IllegalCommand,
    /// Indicates a CRC error from the previous command.
    Crc,
    /// The function and/or type is unimplemented.
    Unimplemented,
    /// Indicates an invalid state transition.
    InvalidTransition {
        /// Represents the current state.
        from: State,
        /// Represents the transition state.
        to: State,
    },
}

impl Error {
    /// Creates a new [Error].
    pub const fn new() -> Self {
        Self::Unimplemented
    }

    /// Creates an general [Error].
    #[inline]
    pub const fn general() -> Self {
        Self::General
    }

    /// Creates an illegal command [Error].
    #[inline]
    pub const fn illegal_command() -> Self {
        Self::IllegalCommand
    }

    /// Creates an crc [Error].
    #[inline]
    pub const fn crc() -> Self {
        Self::Crc
    }

    /// Creates an unimplemented [Error].
    #[inline]
    pub const fn unimplemented() -> Self {
        Self::Unimplemented
    }

    /// Creates an invalid state transition [Error].
    #[inline]
    pub const fn invalid_transition(from: State, to: State) -> Self {
        Self::InvalidTransition { from, to }
    }
}

impl Default for Error {
    fn default() -> Self {
        Self::new()
    }
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::General => write!(f, "general"),
            Self::IllegalCommand => write!(f, "illegal command"),
            Self::Crc => write!(f, "CRC"),
            Self::Unimplemented => write!(f, "unimplemented"),
            Self::InvalidTransition { from, to } => {
                write!(f, "invalid state transition, from: {from}, to: {to}")
            }
        }
    }
}

impl core::error::Error for Error {}

impl Common for Sdio<'_> {
    type Error = Error;

    fn card_type(&self) -> CardType {
        CardType::Sd
    }

    fn card_mode(&self) -> CardMode {
        CardMode::Sdio
    }

    fn setup_bus(&mut self) -> Result<(), Error> {
        Err(Error::unimplemented())
    }

    fn init(&mut self) -> Result<(), Error> {
        // TODO: perform peripheral configuration
        self.state_transition(State::Standby)?;

        Err(Error::unimplemented())
    }

    fn set_sample_phase(&mut self, _sample_phase: u8) {}

    fn fifo_ready(&self, _fifo_status: FifoStatus) -> Result<(), Error> {
        Err(Error::unimplemented())
    }

    fn wait_for_reset(&mut self, _reset: Reset, _timeout: u64) -> Result<(), Error> {
        Err(Error::unimplemented())
    }

    fn wait_while_busy(&mut self, _timout_us: u64) -> Result<(), Error> {
        Err(Error::unimplemented())
    }

    fn read_data(&mut self, _data: &mut [u8]) -> Result<(), Error> {
        Err(Error::unimplemented())
    }

    fn write_data(&mut self, _data: &[u8]) -> Result<(), Error> {
        Err(Error::unimplemented())
    }

    fn send_tuning(&mut self, _mode: TuningMode, _width: TuningWidth) -> Result<(), Error> {
        Err(Error::unimplemented())
    }

    fn interrupt(&self) -> u32 {
        0
    }

    fn set_interrupt(&mut self, _int: u32) {}

    fn clear_all_interrupt(&mut self) {}

    fn response_interrupt(&self) -> u32 {
        0
    }

    fn set_response_interrupt(&mut self, _int: u32) {}

    fn clear_all_response_interrupt(&mut self) {}
}

impl Device for Sdio<'_> {
    fn read_command<C: Command>(&mut self) -> Result<C, Error> {
        Err(Error::unimplemented())
    }

    fn write_response<R: Response>(&mut self, _response: &R) -> Result<(), Error> {
        Err(Error::unimplemented())
    }
}
