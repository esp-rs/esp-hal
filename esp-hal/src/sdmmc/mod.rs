//! # Secure Digital / MultiMedia Card host (SDMMC / SDIO)
//!
//! ## Overview
//!
//! Driver for the SDMMC/SDIO host controller (`SDHOST`). The controller exposes
//! up to two independent card slots that share a single transfer engine.
//!
//! This module is under active development. ESP32-S3 is the current bring-up
//! target; ESP32 and ESP32-P4 support is added in later milestones.

#[cfg(feature = "__sdmmc")]
use sdio as _;

/// Selects one of the controller's card slots.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum SlotId {
    /// Slot 0.
    _0,
    /// Slot 1.
    _1,
}

impl SlotId {
    /// Zero-based slot index.
    #[cfg_attr(not(esp32s3), allow(dead_code))]
    fn index(self) -> u8 {
        match self {
            SlotId::_0 => 0,
            SlotId::_1 => 1,
        }
    }
}

/// Card clock source feeding the controller's divider.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ClockSource {
    /// 160 MHz PLL.
    Pll160m,
    /// Crystal oscillator.
    Xtal,
}

/// Clock input sampling phase used for high-speed tuning.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum DelayPhase {
    /// 0°.
    #[default]
    _0,
    /// 90°.
    _1,
    /// 180°.
    _2,
    /// 270°.
    _3,
}

/// Card data bus width.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum BusWidth {
    /// 1-bit bus (DAT0 only).
    #[default]
    Bit1,
    /// 4-bit bus (DAT0–DAT3).
    Bit4,
}

/// Slot configuration.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(not(esp32s3), allow(dead_code))]
pub struct Config {
    clock_source: ClockSource,
    input_delay_phase: DelayPhase,
    frequency_hz: u32,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            clock_source: ClockSource::Pll160m,
            input_delay_phase: DelayPhase::_0,
            frequency_hz: 400_000,
        }
    }
}

impl Config {
    /// Sets the card clock source.
    pub fn with_clock_source(mut self, source: ClockSource) -> Self {
        self.clock_source = source;
        self
    }

    /// Sets the input sampling delay phase.
    pub fn with_input_delay_phase(mut self, phase: DelayPhase) -> Self {
        self.input_delay_phase = phase;
        self
    }

    /// Sets the target card clock frequency in Hz.
    pub fn with_frequency(mut self, hz: u32) -> Self {
        self.frequency_hz = hz;
        self
    }
}

/// Error returned by host operations.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[non_exhaustive]
pub enum Error {
    /// A hardware operation did not complete in time.
    Timeout,
}

/// Error returned when applying a [`Config`].
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[non_exhaustive]
pub enum ConfigError {}

#[cfg(esp32s3)]
pub use imp::SdHostController;
#[cfg(esp32s3)]
pub use imp::Slot;

#[cfg(esp32s3)]
mod imp {
    use core::marker::PhantomData;

    use super::{BusWidth, ClockSource, Config, Error, SlotId};
    use crate::{
        Async,
        Blocking,
        DriverMode,
        gpio::{
            InputSignal,
            OutputConfig,
            OutputSignal,
            PinGuard,
            Pull,
            interconnect::{self, PeripheralInput, PeripheralOutput},
        },
        pac::sdhost::RegisterBlock,
        peripherals::SDHOST,
        system::{Peripheral, PeripheralGuard},
    };

    /// Module clock: 160 MHz PLL divided by the module divider.
    const MODULE_DIV: u32 = 2;
    /// Resulting module clock in Hz (160 MHz / `MODULE_DIV`).
    const MODULE_CLOCK_HZ: u32 = 160_000_000 / MODULE_DIV;
    /// Spin-poll bound for self-clearing hardware bits.
    const POLL_LIMIT: u32 = 1_000_000;

    fn regs() -> &'static RegisterBlock {
        SDHOST::regs()
    }

    /// SDMMC / SDIO host controller driver.
    ///
    /// Owns the shared transfer engine and hands out per-slot handles.
    pub struct SdHostController<'d> {
        _peri: SDHOST<'d>,
        _guard: PeripheralGuard,
    }

    impl<'d> SdHostController<'d> {
        /// Creates the controller, enabling its bus clock and configuring the
        /// shared module clock.
        pub fn new(peri: SDHOST<'d>) -> Self {
            let guard = PeripheralGuard::new(Peripheral::SdioHost);
            let this = Self {
                _peri: peri,
                _guard: guard,
            };

            // DesignWare reset, then module clock, then quiesce interrupts.
            let _ = reset_engine();
            set_module_clock(ClockSource::Pll160m, MODULE_DIV);
            let r = regs();
            r.tmout().write(|w| unsafe {
                w.response_timeout()
                    .bits(0xFF)
                    .data_timeout()
                    .bits(0xFF_FFFF)
            });
            r.rintsts().write(|w| unsafe { w.bits(0xFFFF_FFFF) });
            r.ctrl().modify(|_, w| w.int_enable().clear_bit());

            this
        }

        /// Returns a builder for the given slot in blocking mode.
        pub fn slot<'a>(&'a mut self, id: SlotId, config: Config) -> Slot<'a, Blocking> {
            Slot {
                id,
                config,
                width: BusWidth::Bit1,
                data_pins: 0,
                cd_connected: false,
                wp_connected: false,
                _guard: PeripheralGuard::new(Peripheral::SdioHost),
                clk: PinGuard::new_unconnected(),
                cmd: PinGuard::new_unconnected(),
                data: [
                    PinGuard::new_unconnected(),
                    PinGuard::new_unconnected(),
                    PinGuard::new_unconnected(),
                    PinGuard::new_unconnected(),
                ],
                power: None,
                cd: None,
                wp: None,
                _pd: PhantomData,
            }
        }
    }

    /// A configured card slot, generic over the driver mode.
    pub struct Slot<'d, Dm: DriverMode> {
        id: SlotId,
        config: Config,
        width: BusWidth,
        data_pins: u8,
        cd_connected: bool,
        wp_connected: bool,
        _guard: PeripheralGuard,
        clk: PinGuard,
        cmd: PinGuard,
        data: [PinGuard; 4],
        power: Option<interconnect::OutputSignal<'d>>,
        cd: Option<interconnect::InputSignal<'d>>,
        wp: Option<interconnect::InputSignal<'d>>,
        _pd: PhantomData<(&'d mut (), Dm)>,
    }

    impl<'d, Dm: DriverMode> Slot<'d, Dm> {
        /// Connects the card clock output.
        pub fn with_clk(mut self, clk: impl PeripheralOutput<'d>) -> Self {
            let pin = clk.into();
            pin.apply_output_config(&OutputConfig::default());
            pin.set_output_enable(true);
            self.clk = interconnect::OutputSignal::connect_with_guard(pin, out_clk(self.id));
            self
        }

        /// Connects the bidirectional command line.
        pub fn with_cmd(mut self, cmd: impl PeripheralInput<'d> + PeripheralOutput<'d>) -> Self {
            self.cmd = connect_bidir(cmd.into(), in_cmd(self.id), out_cmd(self.id));
            self
        }

        /// Connects data line 0 (required for any transfer).
        pub fn with_data0(mut self, d0: impl PeripheralInput<'d> + PeripheralOutput<'d>) -> Self {
            self.data[0] = connect_bidir(d0.into(), in_data(self.id, 0), out_data(self.id, 0));
            self.note_data_pin(1);
            self
        }

        /// Connects data line 1.
        pub fn with_data1(mut self, d1: impl PeripheralInput<'d> + PeripheralOutput<'d>) -> Self {
            self.data[1] = connect_bidir(d1.into(), in_data(self.id, 1), out_data(self.id, 1));
            self.note_data_pin(2);
            self
        }

        /// Connects data line 2.
        pub fn with_data2(mut self, d2: impl PeripheralInput<'d> + PeripheralOutput<'d>) -> Self {
            self.data[2] = connect_bidir(d2.into(), in_data(self.id, 2), out_data(self.id, 2));
            self.note_data_pin(3);
            self
        }

        /// Connects data line 3 (completes the 4-bit bus).
        pub fn with_data3(mut self, d3: impl PeripheralInput<'d> + PeripheralOutput<'d>) -> Self {
            self.data[3] = connect_bidir(d3.into(), in_data(self.id, 3), out_data(self.id, 3));
            self.note_data_pin(4);
            self
        }

        /// Connects the card-detect input.
        pub fn with_card_detect(mut self, cd: impl PeripheralInput<'d>) -> Self {
            let pin = cd.into();
            pin.set_input_enable(true);
            in_card_detect(self.id).connect_to(&pin);
            self.cd = Some(pin);
            self.cd_connected = true;
            self
        }

        /// Connects the write-protect input.
        pub fn with_write_protect(mut self, wp: impl PeripheralInput<'d>) -> Self {
            let pin = wp.into();
            pin.set_input_enable(true);
            in_write_protect(self.id).connect_to(&pin);
            self.wp = Some(pin);
            self.wp_connected = true;
            self
        }

        /// Connects a GPIO used to switch card power. Driven high here.
        pub fn with_power_enable(mut self, power: impl PeripheralOutput<'d>) -> Self {
            let pin = power.into();
            pin.set_output_high(true);
            pin.apply_output_config(&OutputConfig::default());
            pin.set_output_enable(true);
            self.power = Some(pin);
            self
        }

        /// Returns `true` if a card is detected, or if no card-detect pin is
        /// wired (assume present).
        pub fn is_card_present(&self) -> bool {
            if !self.cd_connected {
                return true;
            }
            (regs().cdetect().read().card_detect_n().bits() & (1 << self.id.index())) == 0
        }

        /// Returns `true` if the card reports write protection. Returns `false`
        /// if no write-protect pin is wired.
        pub fn is_write_protected(&self) -> bool {
            if !self.wp_connected {
                return false;
            }
            (regs().wrtprt().read().write_protect().bits() & (1 << self.id.index())) != 0
        }

        /// Applies bus width + card clock for the slot. Used during card init.
        pub fn set_bus_low_level(&mut self, width: BusWidth, hz: u32) -> Result<(), Error> {
            set_module_clock(self.config.clock_source, MODULE_DIV);
            set_card_width(self.id, width);
            self.width = width;
            let div = freq_to_card_div(hz);
            set_card_clock(self.id, div)
        }

        /// Issues the 80-clock SD init sequence (no command index).
        pub fn send_init_sequence(&mut self) -> Result<(), Error> {
            let r = regs();
            r.cmdarg().write(|w| unsafe { w.bits(0) });
            r.cmd().write(|w| unsafe {
                w.send_initialization().set_bit();
                w.wait_prvdata_complete().set_bit();
                w.card_number().bits(self.id.index() as u8);
                w.start_cmd().set_bit()
            });
            wait_command_accepted()
        }

        fn note_data_pin(&mut self, count: u8) {
            self.data_pins = self.data_pins.max(count);
            self.width = if self.data_pins >= 4 {
                BusWidth::Bit4
            } else {
                BusWidth::Bit1
            };
        }
    }

    impl<'d> Slot<'d, Blocking> {
        /// Reconfigures the slot to operate in [`Async`] mode.
        pub fn into_async(self) -> Slot<'d, Async> {
            Slot {
                id: self.id,
                config: self.config,
                width: self.width,
                data_pins: self.data_pins,
                cd_connected: self.cd_connected,
                wp_connected: self.wp_connected,
                _guard: self._guard,
                clk: self.clk,
                cmd: self.cmd,
                data: self.data,
                power: self.power,
                cd: self.cd,
                wp: self.wp,
                _pd: PhantomData,
            }
        }
    }

    fn connect_bidir(
        pin: interconnect::OutputSignal<'_>,
        input: InputSignal,
        output: OutputSignal,
    ) -> PinGuard {
        pin.set_output_high(true);
        pin.apply_output_config(&OutputConfig::default().with_pull(Pull::Up));
        pin.set_output_enable(true);
        pin.set_input_enable(true);
        input.connect_to(&pin);
        interconnect::OutputSignal::connect_with_guard(pin, output)
    }

    /// Resets the controller, FIFO and DMA blocks, waiting for self-clear.
    fn reset_engine() -> Result<(), Error> {
        let r = regs();
        r.ctrl().modify(|_, w| {
            w.controller_reset().set_bit();
            w.fifo_reset().set_bit();
            w.dma_reset().set_bit()
        });
        for _ in 0..POLL_LIMIT {
            let c = r.ctrl().read();
            if !c.controller_reset().bit_is_set()
                && !c.fifo_reset().bit_is_set()
                && !c.dma_reset().bit_is_set()
            {
                return Ok(());
            }
        }
        Err(Error::Timeout)
    }

    /// Programs the shared module clock register (divider, source, phases).
    fn set_module_clock(source: ClockSource, div: u32) {
        // l: clock period, h: high pulse, n == l (see TRM / sdmmc_ll).
        let l = (div - 1) as u8;
        let h = (div / 2).saturating_sub(1) as u8;
        let n = l;
        regs().clk_edge_sel().write(|w| unsafe {
            w.cclkin_edge_drv_sel().bits(1); // output phase 90°
            w.cclkin_edge_sam_sel().bits(0); // input phase 0°
            w.cclkin_edge_slf_sel().bits(0); // core phase 0°
            w.ccllkin_edge_h().bits(h);
            w.ccllkin_edge_l().bits(l);
            w.ccllkin_edge_n().bits(n);
            w.cclk_en().bit(matches!(source, ClockSource::Pll160m))
        });
    }

    /// Sets a slot's card clock divider and (re-)enables the card clock.
    fn set_card_clock(id: SlotId, card_div: u8) -> Result<(), Error> {
        let slot = id.index();
        let r = regs();

        // Disable card clock before changing the divider.
        r.clkena().modify(|rd, w| unsafe {
            w.cclk_enable().bits(rd.cclk_enable().bits() & !(1 << slot))
        });
        apply_clock_update(id)?;

        // Select per-slot divider register and program the divider value.
        r.clksrc().modify(|rd, w| unsafe {
            let mut v = rd.clksrc().bits();
            match id {
                SlotId::_0 => v &= !0b11,
                SlotId::_1 => {
                    v &= !0b1100;
                    v |= 1 << 2;
                }
            }
            w.clksrc().bits(v)
        });
        r.clkdiv().modify(|_, w| unsafe {
            match id {
                SlotId::_0 => w.clk_divider0().bits(card_div),
                SlotId::_1 => w.clk_divider1().bits(card_div),
            }
        });

        // Re-enable card clock with low-power gating, then commit.
        r.clkena().modify(|rd, w| unsafe {
            w.cclk_enable().bits(rd.cclk_enable().bits() | (1 << slot));
            w.lp_enable().bits(rd.lp_enable().bits() | (1 << slot))
        });
        apply_clock_update(id)
    }

    /// Commits clock-register changes via an `update_clock_registers_only`
    /// command.
    fn apply_clock_update(id: SlotId) -> Result<(), Error> {
        let r = regs();
        r.cmdarg().write(|w| unsafe { w.bits(0) });
        r.cmd().write(|w| unsafe {
            w.update_clock_registers_only().set_bit();
            w.wait_prvdata_complete().set_bit();
            w.card_number().bits(id.index() as u8);
            w.start_cmd().set_bit()
        });
        wait_command_accepted()
    }

    /// Spins until the CIU accepts the command (`start_cmd` self-clears).
    fn wait_command_accepted() -> Result<(), Error> {
        let r = regs();
        for _ in 0..POLL_LIMIT {
            if !r.cmd().read().start_cmd().bit_is_set() {
                return Ok(());
            }
            // Hardware-locked error: clear and report.
            if (r.rintsts().read().bits() & (1 << 12)) != 0 {
                r.rintsts().write(|w| unsafe { w.bits(1 << 12) });
                return Err(Error::Timeout);
            }
        }
        Err(Error::Timeout)
    }

    /// Sets the bus width bits for a slot.
    fn set_card_width(id: SlotId, width: BusWidth) {
        let slot = id.index();
        regs().ctype().modify(|rd, w| unsafe {
            let mut w4 = rd.card_width4().bits();
            let w8 = rd.card_width8().bits() & !(1 << slot);
            match width {
                BusWidth::Bit4 => w4 |= 1 << slot,
                BusWidth::Bit1 => w4 &= !(1 << slot),
            }
            w.card_width4().bits(w4);
            w.card_width8().bits(w8)
        });
    }

    /// Computes the per-card divider for a target frequency.
    ///
    /// `card_clk = module / (2 * div)`; `div == 0` bypasses the divider.
    fn freq_to_card_div(target_hz: u32) -> u8 {
        if target_hz == 0 || MODULE_CLOCK_HZ <= target_hz {
            return 0;
        }
        let div = MODULE_CLOCK_HZ.div_ceil(2 * target_hz);
        div.clamp(1, 255) as u8
    }

    fn out_clk(id: SlotId) -> OutputSignal {
        match id {
            SlotId::_0 => OutputSignal::SDHOST_CCLK_OUT_1,
            SlotId::_1 => OutputSignal::SDHOST_CCLK_OUT_2,
        }
    }

    fn out_cmd(id: SlotId) -> OutputSignal {
        match id {
            SlotId::_0 => OutputSignal::SDHOST_CCMD_OUT_1,
            SlotId::_1 => OutputSignal::SDHOST_CCMD_OUT_2,
        }
    }

    fn in_cmd(id: SlotId) -> InputSignal {
        match id {
            SlotId::_0 => InputSignal::SDHOST_CCMD_IN_1,
            SlotId::_1 => InputSignal::SDHOST_CCMD_IN_2,
        }
    }

    fn out_data(id: SlotId, line: u8) -> OutputSignal {
        match (id, line) {
            (SlotId::_0, 0) => OutputSignal::SDHOST_CDATA_OUT_10,
            (SlotId::_0, 1) => OutputSignal::SDHOST_CDATA_OUT_11,
            (SlotId::_0, 2) => OutputSignal::SDHOST_CDATA_OUT_12,
            (SlotId::_0, _) => OutputSignal::SDHOST_CDATA_OUT_13,
            (SlotId::_1, 0) => OutputSignal::SDHOST_CDATA_OUT_20,
            (SlotId::_1, 1) => OutputSignal::SDHOST_CDATA_OUT_21,
            (SlotId::_1, 2) => OutputSignal::SDHOST_CDATA_OUT_22,
            (SlotId::_1, _) => OutputSignal::SDHOST_CDATA_OUT_23,
        }
    }

    fn in_data(id: SlotId, line: u8) -> InputSignal {
        match (id, line) {
            (SlotId::_0, 0) => InputSignal::SDHOST_CDATA_IN_10,
            (SlotId::_0, 1) => InputSignal::SDHOST_CDATA_IN_11,
            (SlotId::_0, 2) => InputSignal::SDHOST_CDATA_IN_12,
            (SlotId::_0, _) => InputSignal::SDHOST_CDATA_IN_13,
            (SlotId::_1, 0) => InputSignal::SDHOST_CDATA_IN_20,
            (SlotId::_1, 1) => InputSignal::SDHOST_CDATA_IN_21,
            (SlotId::_1, 2) => InputSignal::SDHOST_CDATA_IN_22,
            (SlotId::_1, _) => InputSignal::SDHOST_CDATA_IN_23,
        }
    }

    fn in_card_detect(id: SlotId) -> InputSignal {
        match id {
            SlotId::_0 => InputSignal::SDHOST_CARD_DETECT_N_1,
            SlotId::_1 => InputSignal::SDHOST_CARD_DETECT_N_2,
        }
    }

    fn in_write_protect(id: SlotId) -> InputSignal {
        match id {
            SlotId::_0 => InputSignal::SDHOST_CARD_WRITE_PRT_1,
            SlotId::_1 => InputSignal::SDHOST_CARD_WRITE_PRT_2,
        }
    }
}
