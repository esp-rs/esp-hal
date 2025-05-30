use super::{Error, Mode};
use crate::gpio::{Flex, Level, Output, OutputConfig, OutputPin, Pin};

/// Represents the GPIO pins used for SDIO communication.
///
/// The pins are configurable for the SPI + SD (1-bit, 4-bit) operation modes.
///
/// The CLK/SCLK pin is configured as an output, all other pins are input/output
/// `Flex` pins.
#[derive(Debug)]
pub struct Pins<'d> {
    mode: Mode,
    clk_sclk: Output<'d>,
    cmd_mosi: Flex<'d>,
    dat0_miso: Flex<'d>,
    dat1_irq: Flex<'d>,
    dat2: Flex<'d>,
    dat3_cs: Flex<'d>,
}

impl<'d> Pins<'d> {
    /// Creates a new [Pins] from the provided GPIO pins.
    ///
    /// ```rust, no_run
    #[doc = crate::before_snippet!()]
    /// use esp_hal::sdio::{Mode, Pins};
    ///
    /// let _pins = Pins::new(
    ///     Mode::Sd4bit,
    ///     peripherals.GPIO19, // CLK/SCLK
    ///     peripherals.GPIO18, // CMD/MOSI
    ///     peripherals.GPIO20, // DAT0/MISO
    ///     peripherals.GPIO21, // DAT1/IRQ
    ///     peripherals.GPIO22, // DAT2
    ///     peripherals.GPIO23, // DAT3/#CS
    /// );
    /// ```
    pub fn new(
        mode: Mode,
        clk_sclk: impl OutputPin + 'd,
        cmd_mosi: impl Pin + 'd,
        dat0_miso: impl Pin + 'd,
        dat1_irq: impl Pin + 'd,
        dat2: impl Pin + 'd,
        dat3_cs: impl Pin + 'd,
    ) -> Self {
        Self {
            mode,
            clk_sclk: Output::new(clk_sclk, Level::Low, OutputConfig::default()),
            cmd_mosi: Flex::new(cmd_mosi),
            dat0_miso: Flex::new(dat0_miso),
            dat1_irq: Flex::new(dat1_irq),
            dat2: Flex::new(dat2),
            dat3_cs: Flex::new(dat3_cs),
        }
    }

    /// Gets the [Mode] of the [Pins].
    pub const fn mode(&self) -> Mode {
        self.mode
    }

    /// Sets the [Mode] of the [Pins].
    pub fn set_mode(&mut self, mode: Mode) {
        self.mode = mode;
    }

    /// Gets the CLK signal for the [Pins].
    ///
    /// Returns an [Error] if the [Mode] is SPI.
    pub const fn clk(&self) -> Result<&Output<'_>, Error> {
        match self.mode {
            Mode::Sd1bit | Mode::Sd4bit => Ok(&self.clk_sclk),
            Mode::Spi => Err(Error::General),
        }
    }

    /// Gets the SCLK signal for the [Pins].
    ///
    /// Returns an [Error] if the [Mode] is not SPI.
    pub const fn sclk(&self) -> Result<&Output<'_>, Error> {
        match self.mode {
            Mode::Spi => Ok(&self.clk_sclk),
            Mode::Sd1bit | Mode::Sd4bit => Err(Error::General),
        }
    }

    /// Gets the CMD signal for the [Pins].
    ///
    /// Returns an [Error] if the [Mode] is SPI.
    pub const fn cmd(&self) -> Result<&Flex<'_>, Error> {
        match self.mode {
            Mode::Sd1bit | Mode::Sd4bit => Ok(&self.cmd_mosi),
            Mode::Spi => Err(Error::General),
        }
    }

    /// Gets the MOSI signal for the [Pins].
    ///
    /// Returns an [Error] if the [Mode] is not SPI.
    pub const fn mosi(&self) -> Result<&Flex<'_>, Error> {
        match self.mode {
            Mode::Spi => Ok(&self.cmd_mosi),
            Mode::Sd1bit | Mode::Sd4bit => Err(Error::General),
        }
    }

    /// Gets the DAT0 signal for the [Pins].
    ///
    /// Returns an [Error] if the [Mode] is SPI.
    pub const fn dat0(&self) -> Result<&Flex<'_>, Error> {
        match self.mode {
            Mode::Sd1bit | Mode::Sd4bit => Ok(&self.dat0_miso),
            Mode::Spi => Err(Error::General),
        }
    }

    /// Gets the MISO signal for the [Pins].
    ///
    /// Returns an [Error] if the [Mode] is not SPI.
    pub const fn miso(&self) -> Result<&Flex<'_>, Error> {
        match self.mode {
            Mode::Spi => Ok(&self.cmd_mosi),
            Mode::Sd1bit | Mode::Sd4bit => Err(Error::General),
        }
    }

    /// Gets the DAT1 signal for the [Pins].
    ///
    /// Returns an [Error] if the [Mode] is not SD 4-bit.
    pub const fn dat1(&self) -> Result<&Flex<'_>, Error> {
        match self.mode {
            Mode::Sd4bit => Ok(&self.dat1_irq),
            Mode::Sd1bit | Mode::Spi => Err(Error::General),
        }
    }

    /// Gets the IRQ signal for the [Pins].
    ///
    /// Returns an [Error] if the [Mode] is SD 4-bit.
    pub const fn irq(&self) -> Result<&Flex<'_>, Error> {
        match self.mode {
            Mode::Sd1bit | Mode::Spi => Ok(&self.dat1_irq),
            Mode::Sd4bit => Err(Error::General),
        }
    }

    /// Gets the DAT2 signal for the [Pins].
    ///
    /// Returns an [Error] if the [Mode] is not SD 4-bit.
    pub const fn dat2(&self) -> Result<&Flex<'_>, Error> {
        match self.mode {
            Mode::Sd4bit => Ok(&self.dat2),
            Mode::Sd1bit | Mode::Spi => Err(Error::General),
        }
    }

    /// Gets the DAT3 signal for the [Pins].
    ///
    /// Returns an [Error] if the [Mode] is SPI.
    pub const fn dat3(&self) -> Result<&Flex<'_>, Error> {
        match self.mode {
            Mode::Sd4bit => Ok(&self.dat3_cs),
            Mode::Sd1bit | Mode::Spi => Err(Error::General),
        }
    }

    /// Gets the CS signal for the [Pins].
    ///
    /// Returns an [Error] if the [Mode] is not SPI.
    pub const fn cs(&self) -> Result<&Flex<'_>, Error> {
        match self.mode {
            Mode::Spi => Ok(&self.dat3_cs),
            Mode::Sd1bit | Mode::Sd4bit => Err(Error::General),
        }
    }
}
