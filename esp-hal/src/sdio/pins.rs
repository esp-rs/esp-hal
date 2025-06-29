use super::{Error, Mode};
use crate::gpio::{
    AnyPin,
    DriveMode,
    InputSignal,
    OutputConfig,
    OutputSignal,
    PinGuard,
    Pull,
    interconnect,
};

/// Represents SDIO pin signals.
pub(crate) enum SdioPin {
    /// Represents the SDIO CLK signal.
    Clk   = 19,
    /// Represents the SDIO CMD signal.
    Cmd   = 18,
    /// Represents the SDIO DATA0 signal.
    Data0 = 20,
    /// Represents the SDIO DATA1 signal.
    Data1 = 21,
    /// Represents the SDIO DATA2 signal.
    Data2 = 22,
    /// Represents the SDIO DATA3 signal.
    Data3 = 23,
}

/// Represents the GPIO pins used for SDIO communication.
///
/// The pins are configurable for the SPI + SD (1-bit, 4-bit) operation modes.
///
/// The CLK/SCLK pin is configured as an output, all other pins are input/output
/// `Flex` pins.
#[derive(Debug)]
pub struct Pins {
    mode: Mode,
    pub(crate) clk_sclk: PinGuard,
    pub(crate) cmd_mosi: PinGuard,
    pub(crate) dat0_miso: PinGuard,
    pub(crate) dat1_irq: PinGuard,
    pub(crate) dat2: PinGuard,
    pub(crate) dat3_cs: PinGuard,
}

impl Pins {
    /// Creates a new [Pins] from the provided GPIO pins.
    ///
    /// ## Example
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
    /// # Ok(())
    /// # }
    /// ```
    pub fn new<'clk, 'cmd, 'd0, 'd1, 'd2, 'd3, CLK, CMD, DATA0, DATA1, DATA2, DATA3>(
        mode: Mode,
        clk_sclk: CLK,
        cmd_mosi: CMD,
        dat0_miso: DATA0,
        dat1_irq: DATA1,
        dat2: DATA2,
        dat3_cs: DATA3,
    ) -> Self
    where
        CLK: Into<interconnect::OutputSignal<'clk>>,
        CMD: Into<interconnect::OutputSignal<'cmd>>,
        DATA0: Into<interconnect::OutputSignal<'d0>>,
        DATA1: Into<interconnect::OutputSignal<'d1>>,
        DATA2: Into<interconnect::OutputSignal<'d2>>,
        DATA3: Into<interconnect::OutputSignal<'d3>>,
    {
        Self {
            mode,
            clk_sclk: Self::connect_pin(clk_sclk.into(), OutputSignal::SDIO_CLK, None),
            cmd_mosi: Self::connect_pin(
                cmd_mosi.into(),
                OutputSignal::SDIO_CMD,
                Some(InputSignal::SDIO_CMD),
            ),
            dat0_miso: Self::connect_pin(
                dat0_miso.into(),
                OutputSignal::SDIO_DATA0,
                Some(InputSignal::SDIO_DATA0),
            ),
            dat1_irq: Self::connect_pin(
                dat1_irq.into(),
                OutputSignal::SDIO_DATA1,
                Some(InputSignal::SDIO_DATA1),
            ),
            dat2: Self::connect_pin(
                dat2.into(),
                OutputSignal::SDIO_DATA2,
                Some(InputSignal::SDIO_DATA2),
            ),
            dat3_cs: Self::connect_pin(
                dat3_cs.into(),
                OutputSignal::SDIO_DATA3,
                Some(InputSignal::SDIO_DATA3),
            ),
        }
    }

    fn connect_pin(
        pin: interconnect::OutputSignal<'_>,
        output: OutputSignal,
        input: Option<InputSignal>,
    ) -> PinGuard {
        pin.set_output_high(true);

        pin.apply_output_config(
            &OutputConfig::default()
                .with_drive_mode(DriveMode::OpenDrain)
                .with_pull(Pull::Up),
        );

        pin.set_output_enable(true);

        if let Some(in_signal) = input.as_ref() {
            pin.set_input_enable(true);
            in_signal.connect_to(&pin);
        } else {
            pin.set_input_enable(false);
        }

        interconnect::OutputSignal::connect_with_guard(pin, output)
    }

    fn unguard_pin(&self, pin: SdioPin) -> Result<AnyPin<'_>, Error> {
        let pin = match pin {
            SdioPin::Clk => &self.clk_sclk,
            SdioPin::Cmd => &self.cmd_mosi,
            SdioPin::Data0 => &self.dat0_miso,
            SdioPin::Data1 => &self.dat1_irq,
            SdioPin::Data2 => &self.dat2,
            SdioPin::Data3 => &self.dat3_cs,
        };

        pin.pin_number()
            .map(|n| unsafe { AnyPin::steal(n) })
            .ok_or(Error::General)
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
    pub fn clk(&self) -> Result<AnyPin<'_>, Error> {
        match self.mode {
            Mode::Sd1bit | Mode::Sd4bit => self.unguard_pin(SdioPin::Clk),
            Mode::Spi => Err(Error::General),
        }
    }

    /// Gets the SCLK signal for the [Pins].
    ///
    /// Returns an [Error] if the [Mode] is not SPI.
    pub fn sclk(&self) -> Result<AnyPin<'_>, Error> {
        match self.mode {
            Mode::Spi => self.unguard_pin(SdioPin::Clk),
            Mode::Sd1bit | Mode::Sd4bit => Err(Error::General),
        }
    }

    /// Gets the CMD signal for the [Pins].
    ///
    /// Returns an [Error] if the [Mode] is SPI.
    pub fn cmd(&self) -> Result<AnyPin<'_>, Error> {
        match self.mode {
            Mode::Sd1bit | Mode::Sd4bit => self.unguard_pin(SdioPin::Cmd),
            Mode::Spi => Err(Error::General),
        }
    }

    /// Gets the MOSI signal for the [Pins].
    ///
    /// Returns an [Error] if the [Mode] is not SPI.
    pub fn mosi(&self) -> Result<AnyPin<'_>, Error> {
        match self.mode {
            Mode::Spi => self.unguard_pin(SdioPin::Cmd),
            Mode::Sd1bit | Mode::Sd4bit => Err(Error::General),
        }
    }

    /// Gets the DAT0 signal for the [Pins].
    ///
    /// Returns an [Error] if the [Mode] is SPI.
    pub fn dat0(&self) -> Result<AnyPin<'_>, Error> {
        match self.mode {
            Mode::Sd1bit | Mode::Sd4bit => self.unguard_pin(SdioPin::Data0),
            Mode::Spi => Err(Error::General),
        }
    }

    /// Gets the MISO signal for the [Pins].
    ///
    /// Returns an [Error] if the [Mode] is not SPI.
    pub fn miso(&self) -> Result<AnyPin<'_>, Error> {
        match self.mode {
            Mode::Spi => self.unguard_pin(SdioPin::Data0),
            Mode::Sd1bit | Mode::Sd4bit => Err(Error::General),
        }
    }

    /// Gets the DAT1 signal for the [Pins].
    ///
    /// Returns an [Error] if the [Mode] is not SD 4-bit.
    pub fn dat1(&self) -> Result<AnyPin<'_>, Error> {
        match self.mode {
            Mode::Sd1bit | Mode::Sd4bit => self.unguard_pin(SdioPin::Data1),
            Mode::Spi => Err(Error::General),
        }
    }

    /// Gets the IRQ signal for the [Pins].
    ///
    /// Returns an [Error] if the [Mode] is SD 4-bit.
    pub fn irq(&self) -> Result<AnyPin<'_>, Error> {
        match self.mode {
            Mode::Spi => self.unguard_pin(SdioPin::Data1),
            Mode::Sd1bit | Mode::Sd4bit => Err(Error::General),
        }
    }

    /// Gets the DAT2 signal for the [Pins].
    ///
    /// Returns an [Error] if the [Mode] is not SD 4-bit.
    pub fn dat2(&self) -> Result<AnyPin<'_>, Error> {
        match self.mode {
            Mode::Sd1bit | Mode::Sd4bit => self.unguard_pin(SdioPin::Data2),
            Mode::Spi => Err(Error::General),
        }
    }

    /// Gets the DAT3 signal for the [Pins].
    ///
    /// Returns an [Error] if the [Mode] is SPI.
    pub fn dat3(&self) -> Result<AnyPin<'_>, Error> {
        match self.mode {
            Mode::Sd1bit | Mode::Sd4bit => self.unguard_pin(SdioPin::Data3),
            Mode::Spi => Err(Error::General),
        }
    }

    /// Gets the CS signal for the [Pins].
    ///
    /// Returns an [Error] if the [Mode] is not SPI.
    pub fn cs(&self) -> Result<AnyPin<'_>, Error> {
        match self.mode {
            Mode::Spi => self.unguard_pin(SdioPin::Data3),
            Mode::Sd1bit | Mode::Sd4bit => Err(Error::General),
        }
    }
}
