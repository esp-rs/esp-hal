use super::Timing;
pub use crate::spi::Mode as SpiMode;

/// Represents SDIO configuration parameters.
#[repr(C)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Config {
    hs: bool,
    timing: Timing,
    spi_mode: SpiMode,
}

impl Config {
    /// Creates a new [Config].
    pub const fn new() -> Self {
        Self {
            hs: false,
            timing: Timing::new(),
            spi_mode: SpiMode::_0,
        }
    }

    /// Gets the highspeed enable setting.
    pub const fn hs(&self) -> bool {
        self.hs
    }

    /// Sets the highspeed enable setting.
    pub fn set_hs(&mut self, hs: bool) {
        self.hs = hs;
    }

    /// Builder funciton that sets the highspeed enable setting.
    pub fn with_hs(self, hs: bool) -> Self {
        Self { hs, ..self }
    }

    /// Gets the timing setting.
    pub const fn timing(&self) -> Timing {
        self.timing
    }

    /// Sets the timing setting.
    pub fn set_timing(&mut self, timing: Timing) {
        self.timing = timing;
    }

    /// Builder funciton that sets the timing setting.
    pub fn with_timing(self, timing: Timing) -> Self {
        Self { timing, ..self }
    }

    /// Gets the SPI mode setting.
    pub const fn spi_mode(&self) -> SpiMode {
        self.spi_mode
    }

    /// Sets the SPI mode seting.
    pub fn set_spi_mode(&mut self, spi_mode: SpiMode) {
        self.spi_mode = spi_mode;
    }

    /// Builder funciton that sets the SPI mode setting.
    pub fn with_spi_mode(self, spi_mode: SpiMode) -> Self {
        Self { spi_mode, ..self }
    }
}

impl Default for Config {
    fn default() -> Self {
        Self::new()
    }
}
