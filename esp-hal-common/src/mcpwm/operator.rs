//! # MCPWM peripheral - operator module
//!
//! ## Overview
//! The `operator` module is part of the `MCPWM` peripheral driver for
//! `ESP` chips. It is responsible for
//! generating `PWM (Pulse Width Modulation)` signals and handling various
//! aspects related to `PWM` signal generation.
//!
//! This module provides flexibility in configuring the PWM outputs. Its
//! implementation allows for motor control and other applications that demand
//! accurate pulse timing and sophisticated modulation techniques.

use core::marker::PhantomData;

use crate::{
    gpio::OutputPin,
    mcpwm::{timer::Timer, PwmPeripheral},
    peripheral::{Peripheral, PeripheralRef},
};

/// Input/Output Stream descriptor for each channel
pub enum PWMStream {
    /// PWM Stream A
    PWMA,
    /// PWM Stream B
    PWMB,
}

/// Configuration for MCPWM Operator DeadTime
/// It's recommended to reference the technical manual for configuration
pub struct DeadTimeCfg {
    cfg_reg: u32,
    rising_edge_delay: u16,
    falling_edge_delay: u16,
}

impl DeadTimeCfg {
    // NOTE: it's a bit difficult to make this typestate
    // due to the different interconnections (FED/RED vs PWMxA/PWMxB) and
    // the many mode of operation

    /// Uses the following configuration:
    /// * Clock: PWM_clk
    /// * Bypass: A & B
    /// * Inputs: A->A, B->B (InSel)
    /// * Outputs: A->A, B->B (OutSwap)
    /// * No Dual-edge B
    /// * No Invert
    /// * FED/RED update mode = immediate
    /// * FED/RED = 0
    pub fn new_bypass() -> DeadTimeCfg {
        DeadTimeCfg {
            cfg_reg: 0b0_11_00_00_00_0_000_000,
            rising_edge_delay: 0,
            falling_edge_delay: 0,
        }
    }

    /// Active High Complementary (AHC) from Technical Reference manual
    ///
    /// Will generate a PWM from input PWMA, such that output PWMA & PWMB are
    /// each others complement Except during a transition in which they will
    /// be both off (as deadtime) such that they should never overlap, useful
    /// for H-Bridge type scenarios
    ///
    /// Default delay on both rising (red) and falling (fed) edge is 16 cycles
    pub fn new_ahc(red_delay: Option<u16>, fed_delay: Option<u16>) -> DeadTimeCfg {
        DeadTimeCfg {
            cfg_reg: 0b0_00_10_00_00_0_000_000,
            rising_edge_delay: red_delay.unwrap_or(16u16),
            falling_edge_delay: fed_delay.unwrap_or(16u16),
        }
    }
    // TODO: Add some common configurations ~AHC~,ALC,AH,AC

    fn set_flag(&mut self, offset: u8, val: bool) {
        let mask = !(1 << offset);
        self.cfg_reg = self.cfg_reg & mask | ((val as u32) << offset);
    }

    /// Sets the delay for the FED/RED module
    pub fn set_delay(&mut self, rising_edge: u16, falling_edge: u16) {
        self.rising_edge_delay = rising_edge;
        self.falling_edge_delay = falling_edge;
    }

    /// Sets FED/RED output inverter
    /// Inverts the output of the FED/RED module (excl DEB mode feedback)
    pub fn invert_output(&mut self, fed: bool, red: bool) {
        self.set_flag(13, fed);
        self.set_flag(14, red);
    }

    /// Swaps the output of a PWM Stream
    /// i.e. If streams have output_swap enabled, the output of the module
    /// is swapped, while if only one is enabled that one 'copies' from the
    /// other stream
    pub fn set_output_swap(&mut self, stream: PWMStream, swap: bool) {
        self.set_flag(
            match stream {
                PWMStream::PWMA => 9,
                PWMStream::PWMB => 10,
            },
            swap,
        );
    }

    /// Set PWMA/PWMB stream to bypass everything except output_swap
    /// This means no deadtime is applied when enabled
    pub fn set_bypass(&mut self, stream: PWMStream, enable: bool) {
        self.set_flag(
            match stream {
                PWMStream::PWMA => 15,
                PWMStream::PWMB => 16,
            },
            enable,
        );
    }

    /// Select Between PWMClk & PT_Clk
    pub fn select_clock(&mut self, pwm_clock: bool) {
        self.set_flag(17, pwm_clock);
    }

    /// Select which stream is used for the input of FED/RED
    pub fn select_input(&mut self, fed: PWMStream, red: PWMStream) {
        self.set_flag(
            12,
            match fed {
                PWMStream::PWMA => false,
                PWMStream::PWMB => true,
            },
        );
        self.set_flag(
            11,
            match red {
                PWMStream::PWMA => false,
                PWMStream::PWMB => true,
            },
        );
    }
}

#[cfg(feature = "esp32s3")]
fn dt_cfg<const OP: u8, PWM: PwmPeripheral>() -> &'static crate::peripherals::mcpwm0::DB0_CFG {
    let block = unsafe { &*PWM::block() };
    match OP {
        0 => &block.db0_cfg,
        1 => unsafe { &*(&block.db1_cfg as *const _ as *const _) },
        2 => unsafe { &*(&block.db2_cfg as *const _ as *const _) },
        _ => unreachable!(),
    }
}
#[cfg(feature = "esp32s3")]
fn dt_fed<const OP: u8, PWM: PwmPeripheral>() -> &'static crate::peripherals::mcpwm0::DB0_FED_CFG {
    let block = unsafe { &*PWM::block() };
    match OP {
        0 => &block.db0_fed_cfg,
        1 => unsafe { &*(&block.db1_fed_cfg as *const _ as *const _) },
        2 => unsafe { &*(&block.db2_fed_cfg as *const _ as *const _) },
        _ => unreachable!(),
    }
}
#[cfg(feature = "esp32s3")]
fn dt_red<const OP: u8, PWM: PwmPeripheral>() -> &'static crate::peripherals::mcpwm0::DB0_RED_CFG {
    let block = unsafe { &*PWM::block() };
    match OP {
        0 => &block.db0_red_cfg,
        1 => unsafe { &*(&block.db1_red_cfg as *const _ as *const _) },
        2 => unsafe { &*(&block.db2_red_cfg as *const _ as *const _) },
        _ => unreachable!(),
    }
}

// TODO: dt_cfg, dt_fed, dt_red (and similar functions in mcpwm can be made safe
// by patching PACS)
#[cfg(not(feature = "esp32s3"))]
fn dt_cfg<const OP: u8, PWM: PwmPeripheral>() -> &'static crate::peripherals::mcpwm0::DT0_CFG {
    let block = unsafe { &*PWM::block() };
    match OP {
        0 => &block.dt0_cfg(),
        1 => unsafe { &*(&block.dt1_cfg() as *const _ as *const _) },
        2 => unsafe { &*(&block.dt2_cfg() as *const _ as *const _) },
        _ => unreachable!(),
    }
}

#[cfg(not(feature = "esp32s3"))]
fn dt_fed<const OP: u8, PWM: PwmPeripheral>() -> &'static crate::peripherals::mcpwm0::DT0_FED_CFG {
    let block = unsafe { &*PWM::block() };
    match OP {
        0 => &block.dt0_fed_cfg(),
        1 => unsafe { &*(&block.dt1_fed_cfg() as *const _ as *const _) },
        2 => unsafe { &*(&block.dt2_fed_cfg() as *const _ as *const _) },
        _ => unreachable!(),
    }
}
#[cfg(not(feature = "esp32s3"))]
fn dt_red<const OP: u8, PWM: PwmPeripheral>() -> &'static crate::peripherals::mcpwm0::DT0_RED_CFG {
    let block = unsafe { &*PWM::block() };
    match OP {
        0 => &block.dt0_red_cfg(),
        1 => unsafe { &*(&block.dt1_red_cfg() as *const _ as *const _) },
        2 => unsafe { &*(&block.dt2_red_cfg() as *const _ as *const _) },
        _ => unreachable!(),
    }
}

/// A MCPWM operator
///
/// The PWM Operator submodule has the following functions:
/// * Generates a PWM signal pair, based on timing references obtained from the
///   corresponding PWM timer.
/// * Each signal out of the PWM signal pair includes a specific pattern of dead
///   time. (Not yet implemented)
/// * Superimposes a carrier on the PWM signal, if configured to do so. (Not yet
///   implemented)
/// * Handles response under fault conditions. (Not yet implemented)
pub struct Operator<const OP: u8, PWM> {
    phantom: PhantomData<PWM>,
}

impl<const OP: u8, PWM: PwmPeripheral> Operator<OP, PWM> {
    pub(super) fn new() -> Self {
        // Side note:
        // It would have been nice to deselect any timer reference on peripheral
        // initialization.
        // However experimentation (ESP32-S3) showed that writing `3` to timersel
        // will not disable the timer reference but instead act as though `2` was
        // written.
        Operator {
            phantom: PhantomData,
        }
    }

    /// Select a [`Timer`] to be the timing reference for this operator
    ///
    /// ### Note:
    /// By default TIMER0 is used
    pub fn set_timer<const TIM: u8>(&mut self, timer: &Timer<TIM, PWM>) {
        let _ = timer;
        // SAFETY:
        // We only write to our OPERATORx_TIMERSEL register
        let block = unsafe { &*PWM::block() };
        block.operator_timersel().modify(|_, w| match OP {
            0 => w.operator0_timersel().variant(TIM),
            1 => w.operator1_timersel().variant(TIM),
            2 => w.operator2_timersel().variant(TIM),
            _ => {
                unreachable!()
            }
        });
    }

    /// Configures deadtime for this operator
    pub fn set_deadtime(&mut self, cfg: &DeadTimeCfg) {
        dt_fed::<OP, PWM>().write(|w| unsafe { w.bits(cfg.falling_edge_delay as u32) });
        dt_red::<OP, PWM>().write(|w| unsafe { w.bits(cfg.rising_edge_delay as u32) });
        dt_cfg::<OP, PWM>().write(|w| unsafe { w.bits(cfg.cfg_reg) });
    }

    /// Use the A output with the given pin and configuration
    pub fn with_pin_a<'d, Pin: OutputPin>(
        self,
        pin: impl Peripheral<P = Pin> + 'd,
        config: PwmPinConfig<true>,
    ) -> PwmPin<'d, Pin, PWM, OP, true> {
        PwmPin::new(pin, config)
    }

    /// Use the B output with the given pin and configuration
    pub fn with_pin_b<'d, Pin: OutputPin>(
        self,
        pin: impl Peripheral<P = Pin> + 'd,
        config: PwmPinConfig<false>,
    ) -> PwmPin<'d, Pin, PWM, OP, false> {
        PwmPin::new(pin, config)
    }

    /// Use both the A and the B output with the given pins and configurations
    pub fn with_pins<'d, PinA: OutputPin, PinB: OutputPin>(
        self,
        pin_a: impl Peripheral<P = PinA> + 'd,
        config_a: PwmPinConfig<true>,
        pin_b: impl Peripheral<P = PinB> + 'd,
        config_b: PwmPinConfig<false>,
    ) -> (
        PwmPin<'d, PinA, PWM, OP, true>,
        PwmPin<'d, PinB, PWM, OP, false>,
    ) {
        (PwmPin::new(pin_a, config_a), PwmPin::new(pin_b, config_b))
    }
}

/// Configuration describing how the operator generates a signal on a connected
/// pin
pub struct PwmPinConfig<const IS_A: bool> {
    actions: PwmActions<IS_A>,
    update_method: PwmUpdateMethod,
}

impl<const IS_A: bool> PwmPinConfig<IS_A> {
    /// A configuration using [`PwmActions::UP_ACTIVE_HIGH`] and
    /// [`PwmUpdateMethod::SYNC_ON_ZERO`]
    pub const UP_ACTIVE_HIGH: Self =
        Self::new(PwmActions::UP_ACTIVE_HIGH, PwmUpdateMethod::SYNC_ON_ZERO);
    /// A configuration using [`PwmActions::UP_DOWN_ACTIVE_HIGH`] and
    /// [`PwmUpdateMethod::SYNC_ON_ZERO`]
    pub const UP_DOWN_ACTIVE_HIGH: Self = Self::new(
        PwmActions::UP_DOWN_ACTIVE_HIGH,
        PwmUpdateMethod::SYNC_ON_ZERO,
    );

    /// Get a configuration using the given `PwmActions` and `PwmUpdateMethod`
    pub const fn new(actions: PwmActions<IS_A>, update_method: PwmUpdateMethod) -> Self {
        PwmPinConfig {
            actions,
            update_method,
        }
    }
}

/// A pin driven by an MCPWM operator
pub struct PwmPin<'d, Pin, PWM, const OP: u8, const IS_A: bool> {
    _pin: PeripheralRef<'d, Pin>,
    phantom: PhantomData<PWM>,
}

impl<'d, Pin: OutputPin, PWM: PwmPeripheral, const OP: u8, const IS_A: bool>
    PwmPin<'d, Pin, PWM, OP, IS_A>
{
    fn new(pin: impl Peripheral<P = Pin> + 'd, config: PwmPinConfig<IS_A>) -> Self {
        crate::into_ref!(pin);
        let output_signal = PWM::output_signal::<OP, IS_A>();
        pin.enable_output(true)
            .connect_peripheral_to_output(output_signal);
        let mut pin = PwmPin {
            _pin: pin,
            phantom: PhantomData,
        };
        pin.set_actions(config.actions);
        pin.set_update_method(config.update_method);
        pin
    }

    /// Updates dead-time FED register
    ///
    /// WARNING: FED is connected to the operator, and could be connected to
    /// another pin
    #[inline]
    pub fn update_fed(&self, cycles: u16) {
        dt_fed::<OP, PWM>().write(|w| unsafe { w.bits(cycles as u32) });
    }

    /// Updates dead-time RED register
    ///
    /// WARNING: RED is connected to the operator, and could be connected to
    /// another pin
    #[inline]
    pub fn update_red(&self, cycles: u16) {
        dt_red::<OP, PWM>().write(|w| unsafe { w.bits(cycles as u32) });
    }

    /// Configure what actions should be taken on timing events
    pub fn set_actions(&mut self, value: PwmActions<IS_A>) {
        // SAFETY:
        // We only write to our GENx_x register
        let block = unsafe { &*PWM::block() };

        let bits = value.0;

        // SAFETY:
        // `bits` is a valid bit pattern
        unsafe {
            match (OP, IS_A) {
                (0, true) => block.gen0_a().write(|w| w.bits(bits)),
                (1, true) => block.gen1_a().write(|w| w.bits(bits)),
                (2, true) => block.gen2_a().write(|w| w.bits(bits)),
                (0, false) => block.gen0_b().write(|w| w.bits(bits)),
                (1, false) => block.gen1_b().write(|w| w.bits(bits)),
                (2, false) => block.gen2_b().write(|w| w.bits(bits)),
                _ => unreachable!(),
            }
        }
    }

    /// Set how a new timestamp syncs with the timer
    #[cfg(esp32)]
    pub fn set_update_method(&mut self, update_method: PwmUpdateMethod) {
        // SAFETY:
        // We only write to our GENx_x_UPMETHOD register
        let block = unsafe { &*PWM::block() };
        let bits = update_method.0;
        match (OP, IS_A) {
            (0, true) => block
                .gen0_stmp_cfg()
                .modify(|_, w| w.gen0_a_upmethod().variant(bits)),
            (1, true) => block
                .gen1_stmp_cfg()
                .modify(|_, w| w.gen1_a_upmethod().variant(bits)),
            (2, true) => block
                .gen2_stmp_cfg()
                .modify(|_, w| w.gen2_a_upmethod().variant(bits)),
            (0, false) => block
                .gen0_stmp_cfg()
                .modify(|_, w| w.gen0_b_upmethod().variant(bits)),
            (1, false) => block
                .gen1_stmp_cfg()
                .modify(|_, w| w.gen1_b_upmethod().variant(bits)),
            (2, false) => block
                .gen2_stmp_cfg()
                .modify(|_, w| w.gen2_b_upmethod().variant(bits)),
            _ => {
                unreachable!()
            }
        }
    }

    /// Set how a new timestamp syncs with the timer
    #[cfg(esp32s3)]
    pub fn set_update_method(&mut self, update_method: PwmUpdateMethod) {
        // SAFETY:
        // We only write to our GENx_x_UPMETHOD register
        let block = unsafe { &*PWM::block() };
        let bits = update_method.0;
        match (OP, IS_A) {
            (0, true) => block
                .cmpr0_cfg
                .modify(|_, w| w.cmpr0_a_upmethod().variant(bits)),
            (1, true) => block
                .cmpr1_cfg
                .modify(|_, w| w.cmpr1_a_upmethod().variant(bits)),
            (2, true) => block
                .cmpr2_cfg
                .modify(|_, w| w.cmpr2_a_upmethod().variant(bits)),
            (0, false) => block
                .cmpr0_cfg
                .modify(|_, w| w.cmpr0_b_upmethod().variant(bits)),
            (1, false) => block
                .cmpr1_cfg
                .modify(|_, w| w.cmpr1_b_upmethod().variant(bits)),
            (2, false) => block
                .cmpr2_cfg
                .modify(|_, w| w.cmpr2_b_upmethod().variant(bits)),
            _ => {
                unreachable!()
            }
        }
    }

    /// Set how a new timestamp syncs with the timer
    #[cfg(any(esp32c6, esp32h2))]
    pub fn set_update_method(&mut self, update_method: PwmUpdateMethod) {
        // SAFETY:
        // We only write to our GENx_x_UPMETHOD register
        let block = unsafe { &*PWM::block() };
        let bits = update_method.0;
        match (OP, IS_A) {
            (0, true) => block
                .gen0_stmp_cfg()
                .modify(|_, w| w.cmpr0_a_upmethod().variant(bits)),
            (1, true) => block
                .gen1_stmp_cfg()
                .modify(|_, w| w.cmpr1_a_upmethod().variant(bits)),
            (2, true) => block
                .gen2_stmp_cfg()
                .modify(|_, w| w.cmpr2_a_upmethod().variant(bits)),
            (0, false) => block
                .gen0_stmp_cfg()
                .modify(|_, w| w.cmpr0_b_upmethod().variant(bits)),
            (1, false) => block
                .gen1_stmp_cfg()
                .modify(|_, w| w.cmpr1_b_upmethod().variant(bits)),
            (2, false) => block
                .gen2_stmp_cfg()
                .modify(|_, w| w.cmpr2_b_upmethod().variant(bits)),
            _ => {
                unreachable!()
            }
        }
    }

    /// Write a new timestamp.
    /// The written value will take effect according to the set
    /// [`PwmUpdateMethod`].
    #[cfg(esp32)]
    pub fn set_timestamp(&mut self, value: u16) {
        // SAFETY:
        // We only write to our GENx_TSTMP_x register
        let block = unsafe { &*PWM::block() };
        match (OP, IS_A) {
            (0, true) => block.gen0_tstmp_a().write(|w| w.gen0_a().variant(value)),
            (1, true) => block.gen1_tstmp_a().write(|w| w.gen1_a().variant(value)),
            (2, true) => block.gen2_tstmp_a().write(|w| w.gen2_a().variant(value)),
            (0, false) => block.gen0_tstmp_b().write(|w| w.gen0_b().variant(value)),
            (1, false) => block.gen1_tstmp_b().write(|w| w.gen1_b().variant(value)),
            (2, false) => block.gen2_tstmp_b().write(|w| w.gen2_b().variant(value)),
            _ => {
                unreachable!()
            }
        }
    }

    /// Get the old timestamp.
    /// The value of the timestamp will take effect according to the set
    /// [`PwmUpdateMethod`].
    #[cfg(esp32)]
    pub fn get_timestamp(&self) -> u16 {
        // SAFETY:
        // We only read to our GENx_TSTMP_x register
        let block = unsafe { &*PWM::block() };
        match (OP, IS_A) {
            (0, true) => block.gen0_tstmp_a().read().gen0_a().bits(),
            (1, true) => block.gen1_tstmp_a().read().gen1_a().bits(),
            (2, true) => block.gen2_tstmp_a().read().gen2_a().bits(),
            (0, false) => block.gen0_tstmp_b().read().gen0_b().bits(),
            (1, false) => block.gen1_tstmp_b().read().gen1_b().bits(),
            (2, false) => block.gen2_tstmp_b().read().gen2_b().bits(),
            _ => {
                unreachable!()
            }
        }
    }

    /// Write a new timestamp.
    /// The written value will take effect according to the set
    /// [`PwmUpdateMethod`].
    #[cfg(esp32s3)]
    pub fn set_timestamp(&mut self, value: u16) {
        // SAFETY:
        // We only write to our CMPRx_VALUEx register
        let block = unsafe { &*PWM::block() };
        match (OP, IS_A) {
            (0, true) => block.cmpr0_value0.write(|w| w.cmpr0_a().variant(value)),
            (1, true) => block.cmpr1_value0.write(|w| w.cmpr1_a().variant(value)),
            (2, true) => block.cmpr2_value0.write(|w| w.cmpr2_a().variant(value)),
            (0, false) => block.cmpr0_value1.write(|w| w.cmpr0_b().variant(value)),
            (1, false) => block.cmpr1_value1.write(|w| w.cmpr1_b().variant(value)),
            (2, false) => block.cmpr2_value1.write(|w| w.cmpr2_b().variant(value)),
            _ => {
                unreachable!()
            }
        }
    }

    /// Get the old timestamp.
    /// The value of the timestamp will take effect according to the set
    /// [`PwmUpdateMethod`].
    #[cfg(esp32s3)]
    pub fn get_timestamp(&self) -> u16 {
        // SAFETY:
        // We only read to our GENx_TSTMP_x register
        let block = unsafe { &*PWM::block() };
        match (OP, IS_A) {
            (0, true) => block.cmpr0_value0.read().cmpr0_a().bits(),
            (1, true) => block.cmpr1_value0.read().cmpr1_a().bits(),
            (2, true) => block.cmpr2_value0.read().cmpr2_a().bits(),
            (0, false) => block.cmpr0_value1.read().cmpr0_b().bits(),
            (1, false) => block.cmpr1_value1.read().cmpr1_b().bits(),
            (2, false) => block.cmpr2_value1.read().cmpr2_b().bits(),
            _ => {
                unreachable!()
            }
        }
    }

    /// Write a new timestamp.
    /// The written value will take effect according to the set
    /// [`PwmUpdateMethod`].
    #[cfg(any(esp32c6, esp32h2))]
    pub fn set_timestamp(&mut self, value: u16) {
        // SAFETY:
        // We only write to our GENx_TSTMP_x register
        let block = unsafe { &*PWM::block() };
        match (OP, IS_A) {
            (0, true) => block.gen0_tstmp_a().write(|w| w.cmpr0_a().variant(value)),
            (1, true) => block.gen1_tstmp_a().write(|w| w.cmpr1_a().variant(value)),
            (2, true) => block.gen2_tstmp_a().write(|w| w.cmpr2_a().variant(value)),
            (0, false) => block.gen0_tstmp_b().write(|w| w.cmpr0_b().variant(value)),
            (1, false) => block.gen1_tstmp_b().write(|w| w.cmpr1_b().variant(value)),
            (2, false) => block.gen2_tstmp_b().write(|w| w.cmpr2_b().variant(value)),
            _ => {
                unreachable!()
            }
        }
    }

    /// Get the old timestamp.
    /// The value of the timestamp will take effect according to the set
    /// [`PwmUpdateMethod`].
    #[cfg(any(esp32c6, esp32h2))]
    pub fn get_timestamp(&self) -> u16 {
        // SAFETY:
        // We only read to our GENx_TSTMP_x register
        let block = unsafe { &*PWM::block() };
        match (OP, IS_A) {
            (0, true) => block.gen0_tstmp_a().read().cmpr0_a().bits(),
            (1, true) => block.gen1_tstmp_a().read().cmpr1_a().bits(),
            (2, true) => block.gen2_tstmp_a().read().cmpr2_a().bits(),
            (0, false) => block.gen0_tstmp_b().read().cmpr0_b().bits(),
            (1, false) => block.gen1_tstmp_b().read().cmpr1_b().bits(),
            (2, false) => block.gen2_tstmp_b().read().cmpr2_b().bits(),
            _ => {
                unreachable!()
            }
        }
    }

    /// Get the period of the timer.
    fn get_period(&self) -> u16 {
        // SAFETY:
        // We only grant access to our CFG0 register with the lifetime of &mut self
        let block = unsafe { &*PWM::block() };

        let tim_select = block.operator_timersel().read();
        let tim = match OP {
            0 => tim_select.operator0_timersel().bits(),
            1 => tim_select.operator1_timersel().bits(),
            2 => tim_select.operator2_timersel().bits(),
            _ => {
                unreachable!()
            }
        };

        // SAFETY:
        // The CFG0 registers are identical for all timers so we can pretend they're
        // TIMER0_CFG0
        let timer0_cfg = &block.timer0_cfg0();
        let timer0_cfg = match tim {
            0 => timer0_cfg,
            1 => unsafe { &*(&block.timer1_cfg0() as *const _ as *const _) },
            2 => unsafe { &*(&block.timer2_cfg0() as *const _ as *const _) },
            _ => unreachable!(),
        };

        timer0_cfg.read().timer0_period().bits()
    }
}

impl<'d, Pin: OutputPin, PWM: PwmPeripheral, const OP: u8, const IS_A: bool> embedded_hal::PwmPin
    for PwmPin<'d, Pin, PWM, OP, IS_A>
{
    type Duty = u16;

    /// This only set the timestamp to 0, if you want to disable the PwmPin,
    /// it must be done on the timer itself.
    fn disable(&mut self) {
        self.set_timestamp(0);
    }

    /// This only set the timestamp to the maximum, if you want to disable the
    /// PwmPin, it must be done on the timer itself.
    fn enable(&mut self) {
        self.set_timestamp(u16::MAX);
    }

    /// Get the duty of the pin
    fn get_duty(&self) -> Self::Duty {
        self.get_timestamp()
    }

    /// Get the max duty of the pin
    fn get_max_duty(&self) -> Self::Duty {
        self.get_period()
    }

    /// Set the duty of the pin
    fn set_duty(&mut self, duty: Self::Duty) {
        self.set_timestamp(duty);
    }
}

/// Implement no error type for the PwmPin because the method are infallible
#[cfg(feature = "eh1")]
impl<'d, Pin: OutputPin, PWM: PwmPeripheral, const OP: u8, const IS_A: bool>
    embedded_hal_1::pwm::ErrorType for &mut PwmPin<'d, Pin, PWM, OP, IS_A>
{
    type Error = core::convert::Infallible;
}

/// Implement the trait SetDutyCycle for PwmPin
#[cfg(feature = "eh1")]
impl<'d, Pin: OutputPin, PWM: PwmPeripheral, const OP: u8, const IS_A: bool>
    embedded_hal_1::pwm::SetDutyCycle for &mut PwmPin<'d, Pin, PWM, OP, IS_A>
{
    /// Get the max duty of the PwmPin
    fn get_max_duty_cycle(&self) -> u16 {
        self.get_period()
    }
    /// Set the max duty of the PwmPin
    fn set_duty_cycle(&mut self, duty: u16) -> Result<(), core::convert::Infallible> {
        self.set_timestamp(duty);
        Ok(())
    }
}

/// An action the operator applies to an output
#[non_exhaustive]
#[repr(u32)]
pub enum UpdateAction {
    /// Clear the output by setting it to a low level.
    SetLow  = 1,
    /// Set the to a high level.
    SetHigh = 2,
    /// Change the current output level to the opposite value.
    /// If it is currently pulled high, pull it low, or vice versa.
    Toggle  = 3,
}

/// Settings for what actions should be taken on timing events
///
/// ### Note:
/// The hardware supports using a timestamp A event to trigger an action on
/// output B or vice versa. For clearer ownership semantics this HAL does not
/// support such configurations.
pub struct PwmActions<const IS_A: bool>(u32);

impl<const IS_A: bool> PwmActions<IS_A> {
    /// Using this setting together with a timer configured with
    /// [`PwmWorkingMode::Increase`](super::timer::PwmWorkingMode::Increase)
    /// will set the output high for a duration proportional to the set
    /// timestamp.
    pub const UP_ACTIVE_HIGH: Self = Self::empty()
        .on_up_counting_timer_equals_zero(UpdateAction::SetHigh)
        .on_up_counting_timer_equals_timestamp(UpdateAction::SetLow);

    /// Using this setting together with a timer configured with
    /// [`PwmWorkingMode::UpDown`](super::timer::PwmWorkingMode::UpDown) will
    /// set the output high for a duration proportional to the set
    /// timestamp.
    pub const UP_DOWN_ACTIVE_HIGH: Self = Self::empty()
        .on_down_counting_timer_equals_timestamp(UpdateAction::SetHigh)
        .on_up_counting_timer_equals_timestamp(UpdateAction::SetLow);

    /// `PwmActions` with no `UpdateAction`s set
    pub const fn empty() -> Self {
        PwmActions(0)
    }

    /// Choose an `UpdateAction` for an `UTEZ` event
    pub const fn on_up_counting_timer_equals_zero(self, action: UpdateAction) -> Self {
        self.with_value_at_offset(action as u32, 0)
    }

    /// Choose an `UpdateAction` for an `UTEP` event
    pub const fn on_up_counting_timer_equals_period(self, action: UpdateAction) -> Self {
        self.with_value_at_offset(action as u32, 2)
    }

    /// Choose an `UpdateAction` for an `UTEA`/`UTEB` event
    pub const fn on_up_counting_timer_equals_timestamp(self, action: UpdateAction) -> Self {
        match IS_A {
            true => self.with_value_at_offset(action as u32, 4),
            false => self.with_value_at_offset(action as u32, 6),
        }
    }

    /// Choose an `UpdateAction` for an `UTEA`/`UTEB` event where you can
    /// specify which of the A/B to use
    pub const fn on_up_counting_timer_equals_ch_timestamp<const CH_A: bool>(
        self,
        action: UpdateAction,
    ) -> Self {
        match CH_A {
            true => self.with_value_at_offset(action as u32, 4),
            false => self.with_value_at_offset(action as u32, 6),
        }
    }

    /// Choose an `UpdateAction` for an `DTEZ` event
    pub const fn on_down_counting_timer_equals_zero(self, action: UpdateAction) -> Self {
        self.with_value_at_offset(action as u32, 12)
    }

    /// Choose an `UpdateAction` for an `DTEP` event
    pub const fn on_down_counting_timer_equals_period(self, action: UpdateAction) -> Self {
        self.with_value_at_offset(action as u32, 14)
    }

    /// Choose an `UpdateAction` for an `DTEA`/`DTEB` event
    pub const fn on_down_counting_timer_equals_timestamp(self, action: UpdateAction) -> Self {
        match IS_A {
            true => self.with_value_at_offset(action as u32, 16),
            false => self.with_value_at_offset(action as u32, 18),
        }
    }

    /// Choose an `UpdateAction` for an `DTEA`/`DTEB` event where you can
    /// specify which of the A/B to use
    pub const fn on_down_counting_timer_equals_ch_timestamp<const CH_A: bool>(
        self,
        action: UpdateAction,
    ) -> Self {
        match CH_A {
            true => self.with_value_at_offset(action as u32, 16),
            false => self.with_value_at_offset(action as u32, 18),
        }
    }

    const fn with_value_at_offset(self, value: u32, offset: u32) -> Self {
        let mask = !(0b11 << offset);
        let value = (self.0 & mask) | (value << offset);
        PwmActions(value)
    }
}

/// Settings for when [`PwmPin::set_timestamp`] takes effect
///
/// Multiple syncing triggers can be set.
pub struct PwmUpdateMethod(u8);

impl PwmUpdateMethod {
    /// New timestamp will be applied immediately
    pub const SYNC_IMMEDIATLY: Self = Self::empty();
    /// New timestamp will be applied when timer is equal to zero
    pub const SYNC_ON_ZERO: Self = Self::empty().sync_on_timer_equals_zero();
    /// New timestamp will be applied when timer is equal to period
    pub const SYNC_ON_PERIOD: Self = Self::empty().sync_on_timer_equals_period();

    /// `PwmUpdateMethod` with no sync triggers.
    /// Corresponds to syncing immediately
    pub const fn empty() -> Self {
        PwmUpdateMethod(0)
    }

    /// Enable syncing new timestamp values when timer is equal to zero
    pub const fn sync_on_timer_equals_zero(mut self) -> Self {
        self.0 |= 0b0001;
        self
    }

    /// Enable syncing new timestamp values when timer is equal to period
    pub const fn sync_on_timer_equals_period(mut self) -> Self {
        self.0 |= 0b0010;
        self
    }
}
