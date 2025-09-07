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
    SdBusWidth as BusWidth,
    command::Command,
    response::Response,
    tuning::{TuningMode, TuningWidth},
};

use crate::pac;

pub mod command;
mod config;
mod direction;
pub mod dma;
mod hinf;
mod interrupt;
mod io_ocr;
mod pins;
mod slc;
mod slchost;
mod state;
mod timing;

pub use config::Config;
pub use direction::Direction;
pub use hinf::{AnyHinf, HinfInfo, HinfInstance};
pub use interrupt::{DeviceInterrupt, HostInterrupt};
pub use io_ocr::IoOcr;
pub use pins::Pins;
pub use slc::{AnySlc, SlcInfo, SlcInstance};
pub use slchost::{AnySlchost, SlchostInfo, SlchostInstance};
pub use state::State;
pub use timing::Timing;

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
    hinf: AnyHinf<'d>,
    pins: Pins,
    config: Config,
    state: State,
}

impl<'d> Sdio<'d> {
    /// Creates a new [Sdio].
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    #[doc = crate::before_snippet!()]
    /// use esp_hal::sdio::{Config, Mode, Pins, Sdio};
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
    /// let config = Config::new();
    ///
    /// let _sdio = Sdio::new(
    ///     peripherals.SLC,
    ///     peripherals.SLCHOST,
    ///     peripherals.HINF,
    ///     pins,
    ///     config,
    /// );
    /// # Ok(())
    /// # }
    /// ```
    pub fn new(
        slc: impl SlcInstance + 'd,
        slchost: impl SlchostInstance + 'd,
        hinf: impl HinfInstance + 'd,
        pins: Pins,
        config: Config,
    ) -> Self {
        Self {
            slc: slc.degrade(),
            slchost: slchost.degrade(),
            hinf: hinf.degrade(),
            pins,
            config,
            state: State::new(),
        }
    }

    /// Gets a static reference to the SLC information.
    pub fn slc(&self) -> &'static SlcInfo {
        self.slc.info()
    }

    /// Convenience function to get a reference to the SLC register block.
    fn slc_block(&self) -> &'_ pac::slc::RegisterBlock {
        unsafe { &*self.slc().register_block }
    }

    /// Gets a static reference to the SLCHOST information.
    pub fn slchost(&self) -> &'static SlchostInfo {
        self.slchost.info()
    }

    /// Convenience function to get a reference to the SLCHOST register block.
    fn slchost_block(&self) -> &'_ pac::slchost::RegisterBlock {
        unsafe { &*self.slchost().register_block }
    }

    /// Gets a static reference to the HINF information.
    pub fn hinf(&self) -> &'static HinfInfo {
        self.hinf.info()
    }

    /// Convenience function to get a reference to the HINF register block.
    fn hinf_block(&self) -> &'_ pac::hinf::RegisterBlock {
        unsafe { &*self.hinf().register_block }
    }

    /// Gets a reference to the [Pins] information.
    pub const fn pins(&self) -> &Pins {
        &self.pins
    }

    /// Gets a reference to the [Config] information.
    pub const fn config(&self) -> &Config {
        &self.config
    }

    /// Gets the bus mode of the SDIO peripheral.
    pub const fn bus_mode(&self) -> Mode {
        self.pins.mode()
    }

    /// Gets the bus width of the SDIO peripheral.
    pub const fn bus_width(&self) -> BusWidth {
        match self.bus_mode() {
            Mode::Spi | Mode::Sd1bit => BusWidth::_1bit,
            Mode::Sd4bit => BusWidth::_4bit,
        }
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

    /// Performs final low-level HAL hardware initialization.
    pub(crate) fn hardware_init(&mut self) -> Result<(), Error> {
        self.pac_init()?;
        self.pac_enable_hs()?;
        self.pac_set_timing()?;
        self.pac_dev_interrupt_enable(enumset::EnumSet::all())
    }

    /// Performs low-level initialization of the SDIO peripheral.
    #[cfg(esp32)]
    fn pac_init(&mut self) -> Result<(), Error> {
        let slc = self.slc_block();

        slc.conf0().modify(|_, w| {
            w.slc0_rx_auto_wrback().set_bit();
            w.slc0_token_auto_clr().clear_bit();

            w.slc0_rx_loop_test().clear_bit();
            w.slc0_tx_loop_test().clear_bit()
        });

        slc.conf1().modify(|_, w| {
            w.slc0_rx_stitch_en().clear_bit();
            w.slc0_tx_stitch_en().clear_bit();

            w.slc0_len_auto_clr().clear_bit()
        });

        slc.rx_dscr_conf()
            .modify(|_, w| w.slc0_token_no_replace().set_bit());

        Ok(())
    }

    /// Performs low-level initialization of the SDIO peripheral.
    #[cfg(esp32c6)]
    fn pac_init(&mut self) -> Result<(), Error> {
        let slc = self.slc_block();

        slc.slcconf0().modify(|_, w| {
            w.sdio_slc0_rx_auto_wrback().set_bit();
            w.sdio_slc0_token_auto_clr().clear_bit();

            w.sdio_slc0_rx_loop_test().clear_bit();
            w.sdio_slc0_tx_loop_test().clear_bit()
        });

        slc.slcconf1().modify(|_, w| {
            w.sdio_slc0_rx_stitch_en().clear_bit();
            w.sdio_slc0_tx_stitch_en().clear_bit();

            w.sdio_slc0_len_auto_clr().clear_bit()
        });

        slc.slc_rx_dscr_conf()
            .modify(|_, w| w.sdio_slc0_token_no_replace().set_bit());

        Ok(())
    }

    /// Sets the high-speed supported bit to be read by the host.
    #[cfg(any(esp32, esp32c6))]
    fn pac_enable_hs(&mut self) -> Result<(), Error> {
        self.hinf_block()
            .cfg_data1()
            .modify(|_, w| w.highspeed_enable().variant(self.config.hs()));

        Ok(())
    }

    /// Sets the communication timing.
    fn pac_set_timing(&mut self) -> Result<(), Error> {
        match self.config.timing() {
            Timing::PsendPsample => {
                self.set_timing_registers(0x1f, 0x0, 0x1f, 0x0);
            }
            Timing::PsendNsample => {
                self.set_timing_registers(0x1f, 0x0, 0x0, 0x1f);
            }
            Timing::NsendPsample => {
                self.set_timing_registers(0x0, 0x1f, 0x1f, 0x0);
            }
            Timing::NsendNsample => {
                self.set_timing_registers(0x0, 0x1f, 0x0, 0x1f);
            }
        }

        Ok(())
    }

    #[cfg(esp32)]
    fn set_timing_registers(&self, sdio20: u8, sdio11: u8, pos_samp: u8, neg_samp: u8) {
        self.slchost_block()
            .host_slchost_conf()
            .modify(|_, w| unsafe {
                w.host_frc_sdio20().bits(sdio20);
                w.host_frc_sdio11().bits(sdio11);
                w.host_frc_pos_samp().bits(pos_samp);
                w.host_frc_neg_samp().bits(neg_samp)
            });
    }

    #[cfg(esp32c6)]
    fn set_timing_registers(&self, sdio20: u8, sdio11: u8, pos_samp: u8, neg_samp: u8) {
        self.slchost_block().conf().modify(|_, w| unsafe {
            w.frc_sdio20().bits(sdio20);
            w.frc_sdio11().bits(sdio11);
            w.frc_pos_samp().bits(pos_samp);
            w.frc_neg_samp().bits(neg_samp)
        });
    }

    /// Sets which device interrupts to enable based on the provided mask.
    #[cfg(esp32)]
    fn pac_dev_interrupt_enable(
        &self,
        mask: enumset::EnumSet<DeviceInterrupt>,
    ) -> Result<(), Error> {
        self.slc_block()._0int_ena().modify(|_, w| {
            w.frhost_bit0_int_ena()
                .variant(mask.contains(DeviceInterrupt::General0));
            w.frhost_bit1_int_ena()
                .variant(mask.contains(DeviceInterrupt::General1));
            w.frhost_bit2_int_ena()
                .variant(mask.contains(DeviceInterrupt::General2));
            w.frhost_bit3_int_ena()
                .variant(mask.contains(DeviceInterrupt::General3));
            w.frhost_bit4_int_ena()
                .variant(mask.contains(DeviceInterrupt::General4));
            w.frhost_bit5_int_ena()
                .variant(mask.contains(DeviceInterrupt::General5));
            w.frhost_bit6_int_ena()
                .variant(mask.contains(DeviceInterrupt::General6));
            w.frhost_bit7_int_ena()
                .variant(mask.contains(DeviceInterrupt::General7))
        });

        Ok(())
    }

    /// Sets which device interrupts to enable based on the provided mask.
    #[cfg(esp32c6)]
    fn pac_dev_interrupt_enable(
        &self,
        mask: enumset::EnumSet<DeviceInterrupt>,
    ) -> Result<(), Error> {
        self.slc_block().slc0int_ena().modify(|_, w| {
            w.sdio_slc_frhost_bit0_int_ena()
                .variant(mask.contains(DeviceInterrupt::General0));
            w.sdio_slc_frhost_bit1_int_ena()
                .variant(mask.contains(DeviceInterrupt::General1));
            w.sdio_slc_frhost_bit2_int_ena()
                .variant(mask.contains(DeviceInterrupt::General2));
            w.sdio_slc_frhost_bit3_int_ena()
                .variant(mask.contains(DeviceInterrupt::General3));
            w.sdio_slc_frhost_bit4_int_ena()
                .variant(mask.contains(DeviceInterrupt::General4));
            w.sdio_slc_frhost_bit5_int_ena()
                .variant(mask.contains(DeviceInterrupt::General5));
            w.sdio_slc_frhost_bit6_int_ena()
                .variant(mask.contains(DeviceInterrupt::General6));
            w.sdio_slc_frhost_bit7_int_ena()
                .variant(mask.contains(DeviceInterrupt::General7))
        });

        Ok(())
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
        // This implementation is based on the `esp-idf` driver:
        // <https://github.com/espressif/esp-idf/blob/release/v5.5/components/esp_driver_sdio/src/sdio_slave.c#L299>
        self.hardware_init()?;

        self.state_transition(State::Standby)
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
