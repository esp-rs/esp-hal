//! # Secure Digital I/O - Slave Mode
//!
//! ## Overiew
//!
//! The peripheral can be used to transfer data over the SDIO bus in `Slave` mode.

use embedded_hal::mmc::command::MmcCommand;
use embedded_hal::mmc::response::MmcResponse;
use embedded_hal::mmc::tuning::{TuningMode, TuningWidth};
use embedded_hal::mmc::{CardMode, CardType, FifoStatus, MmcOps, Reset};

pub mod error;

use error::Error;

mod slc;
mod slchost;

pub use slc::*;
pub use slchost::*;

/// SDIO peripheral instance.
pub trait PeripheralInstance: crate::private::Sealed {
    /// Represents the peripheral information type containing the register block.
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
    /// use crate::peripheral::Peripherals;
    /// use esp_hal::sdio::Sdio;
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

impl<'d> MmcOps for Sdio<'d> {
    type Error = Error;

    fn card_type(&self) -> CardType {
        CardType::Sd
    }

    fn card_mode(&self) -> CardMode {
        CardMode::Sdio
    }

    fn setup_bus(&mut self) -> Result<(), Self::Error> {
        Err(Error::unimplemented())
    }

    fn init(&mut self) -> Result<(), Self::Error> {
        Err(Error::unimplemented())
    }

    fn set_sample_phase(&mut self, _sample_phase: u8) {}

    fn fifo_ready(&self, _fifo_status: FifoStatus) -> Result<(), Self::Error> {
        Err(Error::unimplemented())
    }

    fn wait_for_reset(&mut self, _reset: Reset, _timeout: u64) -> Result<(), Self::Error> {
        Err(Error::unimplemented())
    }

    fn wait_while_busy(&mut self, _timout_us: u64) -> Result<(), Self::Error> {
        Err(Error::unimplemented())
    }

    fn write_command<C: MmcCommand>(&mut self, _cmd: &C) -> Result<(), Self::Error> {
        Err(Error::unimplemented())
    }

    fn read_response<C: MmcCommand, R: MmcResponse>(&mut self, _cmd: &C) -> Result<R, Self::Error> {
        Err(Error::unimplemented())
    }

    fn response_bytes<const N: usize>(&mut self, _exp_crc: bool) -> Result<[u8; N], Self::Error> {
        Err(Error::unimplemented())
    }

    fn read_data(&mut self, _data: &mut [u8]) -> Result<(), Self::Error> {
        Err(Error::unimplemented())
    }

    fn write_data(&mut self, _data: &[u8]) -> Result<(), Self::Error> {
        Err(Error::unimplemented())
    }

    fn send_tuning(&mut self, _mode: TuningMode, _width: TuningWidth) -> Result<(), Self::Error> {
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
