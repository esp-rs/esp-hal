//! # Secure Digital I/O - Slave Mode
//!
//! ## Overiew
//!
//! The peripheral can be used to transfer data over the SDIO bus in `Slave`
//! mode.

use embedded_hal::mmc::{
    CardMode,
    CardType,
    FifoStatus,
    MmcCommon,
    MmcDevice,
    Reset,
    command::MmcCommand,
    response::MmcResponse,
    tuning::{TuningMode, TuningWidth},
};

mod slc;
mod slchost;

pub use slc::*;
pub use slchost::*;

/// SDIO peripheral instance.
pub trait PeripheralInstance: crate::private::Sealed {
    /// Represents the peripheral information type containing the register
    /// block.
    type Info;

    /// Gets a static shared reference to the peripheral information.
    fn info(&self) -> &'static Self::Info;
}

/// Represents the SDIO 2.0 peripheral for the microcontroller.
#[derive(Debug)]
pub struct Sdio<'d> {
    slc: AnySlc<'d>,
    slchost: AnySlchost<'d>,
}

impl<'d> Sdio<'d> {
    /// Creates a new [Sdio].
    ///
    /// # Example
    ///
    /// ```rust,no_run
    /// use esp_hal::sdio::Sdio;
    ///
    /// use crate::peripheral::Peripherals;
    ///
    /// let dp = Peripheral::take().unwrap();
    /// let _sdio = Sdio::new(dp.slc, dp.slchost);
    /// ```
    pub fn new(slc: impl SlcInstance + 'd, slchost: impl SlchostInstance + 'd) -> Self {
        Self {
            slc: slc.degrade(),
            slchost: slchost.degrade(),
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
        }
    }
}

impl core::error::Error for Error {}

impl<'d> MmcCommon for Sdio<'d> {
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

<<<<<<< Updated upstream
    fn init(&mut self) -> Result<(), Self::Error> {
=======
    fn init(&mut self) -> Result<(), Error> {
        // TODO: perform peripheral configuration
        self.state_transition(State::Standby)?;

>>>>>>> Stashed changes
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

impl<'d> MmcDevice for Sdio<'d> {
    fn read_command<C: MmcCommand>(&mut self) -> Result<C, Error> {
        Err(Error::unimplemented())
    }

    fn write_response<R: MmcResponse>(&mut self, _response: &R) -> Result<(), Error> {
        Err(Error::unimplemented())
    }
}
