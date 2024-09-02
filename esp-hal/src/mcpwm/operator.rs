//! # MCPWM Operator Module
//!
//! ## Overview
//! The `operator` is responsible for generating `PWM (Pulse Width Modulation)`
//! signals and handling various aspects related to `PWM` signal generation.
//!
//! ## Configuration
//! This module provides flexibility in configuring the PWM outputs. Its
//! implementation allows for motor control and other applications that demand
//! accurate pulse timing and sophisticated modulation techniques.

use core::marker::PhantomData;

use crate::{
    gpio::OutputPin,
    mcpwm::{timer::Timer, PwmPeripheral},
    peripheral::{Peripheral, PeripheralRef},
    private,
};

/// Input/Output Stream descriptor for each channel
#[derive(Copy, Clone)]
pub enum PWMStream {
    /// PWM Stream A
    PWMA,
    /// PWM Stream B
    PWMB,
}

/// Configuration for MCPWM Operator DeadTime
/// It's recommended to reference the technical manual for configuration
#[derive(Copy, Clone)]
pub struct DeadTimeCfg {
    cfg_reg: u32,
}

#[allow(clippy::unusual_byte_groupings)]
impl DeadTimeCfg {
    // NOTE: it's a bit difficult to make this typestate
    // due to the different interconnections (FED/RED vs PWMxA/PWMxB) and
    // the many modes of operation

    /// B_OUTBYPASS
    const S0: u32 = 0b01_0000_0000_0000_0000;
    /// A_OUTBYPASS
    const S1: u32 = 0b00_1000_0000_0000_0000;
    /// RED_OUTINVERT
    const S2: u32 = 0b00_0010_0000_0000_0000;
    /// FED_OUTINVERT
    const S3: u32 = 0b00_0100_0000_0000_0000;
    /// RED_INSEL
    const S4: u32 = 0b00_0000_1000_0000_0000;
    /// FED_INSEL
    const S5: u32 = 0b00_0001_0000_0000_0000;
    /// A_OUTSWAP
    const S6: u32 = 0b00_0000_0010_0000_0000;
    /// B_OUTSWAP
    const S7: u32 = 0b00_0000_0100_0000_0000;
    /// DEB_MODE
    const _S8: u32 = 0b00_0000_0001_0000_0000;
    /// Use PT_clk instead of PWM_clk
    const CLK_SEL: u32 = 0b10_0000_0000_0000_0000;

    /// Uses the following configuration:
    /// * Clock: PWM_clk
    /// * Bypass: A & B
    /// * Inputs: A->A, B->B (InSel)
    /// * Outputs: A->A, B->B (OutSwap)
    /// * No Dual-edge B
    /// * No Invert
    /// * FED/RED update mode = immediate
    pub const fn new_bypass() -> DeadTimeCfg {
        DeadTimeCfg {
            cfg_reg: Self::S0 | Self::S1,
        }
    }

    /// Active High Complementary (AHC) from Technical Reference manual
    ///
    /// Will generate a PWM from input PWMA, such that output PWMA & PWMB are
    /// each others complement except during a transition in which they will
    /// be both off (as deadtime) such that they should never overlap, useful
    /// for H-Bridge type scenarios
    pub const fn new_ahc() -> DeadTimeCfg {
        DeadTimeCfg { cfg_reg: Self::S3 }
    }
    // TODO: Add some common configurations ~AHC~,ALC,AH,AC

    #[must_use]
    const fn set_flag(mut self, flag: u32, val: bool) -> Self {
        if val {
            self.cfg_reg |= flag;
        } else {
            self.cfg_reg &= !flag;
        }
        self
    }

    /// Sets FED/RED output inverter
    /// Inverts the output of the FED/RED module (excl DEB mode feedback)
    #[must_use]
    pub const fn invert_output(self, fed: bool, red: bool) -> Self {
        self.set_flag(Self::S3, fed).set_flag(Self::S2, red)
    }

    /// Swaps the output of a PWM Stream
    /// i.e. If both streams have output_swap enabled, the output of the module
    /// is swapped, while if only one is enabled that one 'copies' from the
    /// other stream
    #[must_use]
    pub const fn set_output_swap(self, stream: PWMStream, swap: bool) -> Self {
        self.set_flag(
            match stream {
                PWMStream::PWMA => Self::S6,
                PWMStream::PWMB => Self::S7,
            },
            swap,
        )
    }

    /// Set PWMA/PWMB stream to bypass everything except output_swap
    /// This means no deadtime is applied when enabled
    #[must_use]
    pub const fn set_bypass(self, stream: PWMStream, enable: bool) -> Self {
        self.set_flag(
            match stream {
                PWMStream::PWMA => Self::S1,
                PWMStream::PWMB => Self::S0,
            },
            enable,
        )
    }

    /// Select Between PWMClk & PT_Clk
    #[must_use]
    pub const fn select_clock(self, pwm_clock: bool) -> Self {
        self.set_flag(Self::CLK_SEL, pwm_clock)
    }

    /// Select which stream is used for the input of FED/RED
    #[must_use]
    pub const fn select_input(self, fed: PWMStream, red: PWMStream) -> Self {
        self.set_flag(
            Self::S5,
            match fed {
                PWMStream::PWMA => false,
                PWMStream::PWMB => true,
            },
        )
        .set_flag(
            Self::S4,
            match red {
                PWMStream::PWMA => false,
                PWMStream::PWMB => true,
            },
        )
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
            0 => unsafe { w.operator0_timersel().bits(TIM) },
            1 => unsafe { w.operator1_timersel().bits(TIM) },
            2 => unsafe { w.operator2_timersel().bits(TIM) },
            _ => {
                unreachable!()
            }
        });
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

    /// Link two pins using the deadtime generator
    ///
    /// This is useful for complementary or mirrored signals with or without
    /// configured deadtime
    pub fn with_linked_pins<'d, PinA: OutputPin, PinB: OutputPin>(
        self,
        pin_a: impl Peripheral<P = PinA> + 'd,
        config_a: PwmPinConfig<true>,
        pin_b: impl Peripheral<P = PinB> + 'd,
        config_b: PwmPinConfig<false>,
        config_dt: DeadTimeCfg,
    ) -> LinkedPins<'d, PinA, PinB, PWM, OP> {
        LinkedPins::new(pin_a, config_a, pin_b, config_b, config_dt)
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
    /// A configuration using [`PwmActions::empty`] and
    /// [`PwmUpdateMethod::empty`]
    pub const EMPTY: Self = Self::new(PwmActions::empty(), PwmUpdateMethod::empty());

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
    pin: PeripheralRef<'d, Pin>,
    phantom: PhantomData<PWM>,
}

impl<'d, Pin: OutputPin, PWM: PwmPeripheral, const OP: u8, const IS_A: bool>
    PwmPin<'d, Pin, PWM, OP, IS_A>
{
    fn new(pin: impl Peripheral<P = Pin> + 'd, config: PwmPinConfig<IS_A>) -> Self {
        crate::into_ref!(pin);
        let mut pin = PwmPin {
            pin,
            phantom: PhantomData,
        };
        pin.set_actions(config.actions);
        pin.set_update_method(config.update_method);

        let output_signal = PWM::output_signal::<OP, IS_A>();
        pin.pin
            .connect_peripheral_to_output(output_signal, private::Internal);
        pin.pin.enable_output(true, private::Internal);
        pin
    }

    /// Configure what actions should be taken on timing events
    pub fn set_actions(&mut self, value: PwmActions<IS_A>) {
        // SAFETY:
        // We only write to our GENx_x register
        let ch = unsafe { Self::ch() };
        let bits = value.0;

        // SAFETY:
        // `bits` is a valid bit pattern
        ch.gen((!IS_A) as usize).write(|w| unsafe { w.bits(bits) })
    }

    /// Set how a new timestamp syncs with the timer
    pub fn set_update_method(&mut self, update_method: PwmUpdateMethod) {
        // SAFETY:
        // We only write to our GENx_x_UPMETHOD register
        let ch = unsafe { Self::ch() };
        let bits = update_method.0;

        #[cfg(esp32s3)]
        let cfg = ch.cmpr_cfg();
        #[cfg(any(esp32, esp32c6, esp32h2))]
        let cfg = ch.gen_stmp_cfg();

        cfg.modify(|_, w| unsafe {
            if IS_A {
                w.a_upmethod().bits(bits)
            } else {
                w.b_upmethod().bits(bits)
            }
        })
    }

    /// Write a new timestamp.
    /// The written value will take effect according to the set
    /// [`PwmUpdateMethod`].
    pub fn set_timestamp(&mut self, value: u16) {
        // SAFETY:
        // We only write to our GENx_TSTMP_x register
        let ch = unsafe { Self::ch() };

        #[cfg(esp32s3)]
        if IS_A {
            ch.cmpr_value0().write(|w| unsafe { w.a().bits(value) })
        } else {
            ch.cmpr_value1().write(|w| unsafe { w.b().bits(value) })
        }

        #[cfg(any(esp32, esp32c6, esp32h2))]
        if IS_A {
            ch.gen_tstmp_a().write(|w| unsafe { w.a().bits(value) })
        } else {
            ch.gen_tstmp_b().write(|w| unsafe { w.b().bits(value) })
        }
    }

    /// Get the old timestamp.
    /// The value of the timestamp will take effect according to the set
    /// [`PwmUpdateMethod`].
    pub fn get_timestamp(&self) -> u16 {
        // SAFETY:
        // We only read to our GENx_TSTMP_x register
        let ch = unsafe { Self::ch() };

        #[cfg(esp32s3)]
        if IS_A {
            ch.cmpr_value0().read().a().bits()
        } else {
            ch.cmpr_value1().read().b().bits()
        }

        #[cfg(any(esp32, esp32c6, esp32h2))]
        if IS_A {
            ch.gen_tstmp_a().read().a().bits()
        } else {
            ch.gen_tstmp_b().read().b().bits()
        }
    }

    /// Get the period of the timer.
    pub fn get_period(&self) -> u16 {
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
        block.timer(tim as usize).cfg0().read().period().bits()
    }

    unsafe fn ch() -> &'static crate::peripherals::mcpwm0::CH {
        let block = unsafe { &*PWM::block() };
        block.ch(OP as usize)
    }
}

#[cfg(feature = "embedded-hal-02")]
impl<'d, Pin: OutputPin, PWM: PwmPeripheral, const OP: u8, const IS_A: bool> embedded_hal_02::PwmPin
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
#[cfg(feature = "embedded-hal")]
impl<'d, Pin: OutputPin, PWM: PwmPeripheral, const OP: u8, const IS_A: bool>
    embedded_hal::pwm::ErrorType for PwmPin<'d, Pin, PWM, OP, IS_A>
{
    type Error = core::convert::Infallible;
}

/// Implement the trait SetDutyCycle for PwmPin
#[cfg(feature = "embedded-hal")]
impl<'d, Pin: OutputPin, PWM: PwmPeripheral, const OP: u8, const IS_A: bool>
    embedded_hal::pwm::SetDutyCycle for PwmPin<'d, Pin, PWM, OP, IS_A>
{
    /// Get the max duty of the PwmPin
    fn max_duty_cycle(&self) -> u16 {
        self.get_period()
    }

    /// Set the max duty of the PwmPin
    fn set_duty_cycle(&mut self, duty: u16) -> Result<(), core::convert::Infallible> {
        self.set_timestamp(duty);
        Ok(())
    }
}

/// Two pins driven by the same timer and operator
///
/// Useful for complementary or mirrored signals with or without
/// configured deadtime.
///
/// # H-Bridge example
///
/// ```rust, no_run
#[doc = crate::before_snippet!()]
/// # use esp_hal::{mcpwm, prelude::*};
/// # use esp_hal::mcpwm::{McPwm, PeripheralClockConfig};
/// # use esp_hal::mcpwm::operator::{DeadTimeCfg, PwmPinConfig, PWMStream};
/// # use esp_hal::gpio::Io;
/// # let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
/// // active high complementary using PWMA input
/// let bridge_active = DeadTimeCfg::new_ahc();
/// // use PWMB as input for both outputs
/// let bridge_off = DeadTimeCfg::new_bypass().set_output_swap(PWMStream::PWMA,
/// true);
#[cfg_attr(
    esp32h2,
    doc = "let clock_cfg = PeripheralClockConfig::with_frequency(&clocks, 40.MHz()).unwrap();"
)]
#[cfg_attr(
    not(esp32h2),
    doc = "let clock_cfg = PeripheralClockConfig::with_frequency(&clocks, 32.MHz()).unwrap();"
)]
/// let mut mcpwm = McPwm::new(peripherals.MCPWM0, clock_cfg);
///
/// let mut pins = mcpwm.operator0.with_linked_pins(
///     io.pins.gpio0,
///     PwmPinConfig::UP_DOWN_ACTIVE_HIGH, // use PWMA as our main input
///     io.pins.gpio1,
///     PwmPinConfig::EMPTY, // keep PWMB "low"
///     bridge_off,
/// );
///
/// pins.set_falling_edge_deadtime(5);
/// pins.set_rising_edge_deadtime(5);
/// // pin_a: ________________________________________
/// // pin_b: ________________________________________
/// pins.set_timestamp_a(40); // 40% duty cycle if period configured to 100
/// pins.set_deadtime_cfg(bridge_active);
/// // pin_a: _______-------_____________-------______
/// // pin_b: ------_________-----------_________-----
/// # }
/// ```
pub struct LinkedPins<'d, PinA, PinB, PWM, const OP: u8> {
    pin_a: PwmPin<'d, PinA, PWM, OP, true>,
    pin_b: PwmPin<'d, PinB, PWM, OP, false>,
}

impl<'d, PinA: OutputPin, PinB: OutputPin, PWM: PwmPeripheral, const OP: u8>
    LinkedPins<'d, PinA, PinB, PWM, OP>
{
    fn new(
        pin_a: impl Peripheral<P = PinA> + 'd,
        config_a: PwmPinConfig<true>,
        pin_b: impl Peripheral<P = PinB> + 'd,
        config_b: PwmPinConfig<false>,
        config_dt: DeadTimeCfg,
    ) -> Self {
        // setup deadtime config before enabling the pins
        #[cfg(esp32s3)]
        let dt_cfg = unsafe { Self::ch() }.db_cfg();
        #[cfg(not(esp32s3))]
        let dt_cfg = unsafe { Self::ch() }.dt_cfg();
        dt_cfg.write(|w| unsafe { w.bits(config_dt.cfg_reg) });

        let pin_a = PwmPin::new(pin_a, config_a);
        let pin_b = PwmPin::new(pin_b, config_b);

        LinkedPins { pin_a, pin_b }
    }

    /// Configure what actions should be taken on timing events
    pub fn set_actions_a(&mut self, value: PwmActions<true>) {
        self.pin_a.set_actions(value)
    }
    /// Configure what actions should be taken on timing events
    pub fn set_actions_b(&mut self, value: PwmActions<false>) {
        self.pin_b.set_actions(value)
    }

    /// Set how a new timestamp syncs with the timer
    pub fn set_update_method_a(&mut self, update_method: PwmUpdateMethod) {
        self.pin_a.set_update_method(update_method)
    }
    /// Set how a new timestamp syncs with the timer
    pub fn set_update_method_b(&mut self, update_method: PwmUpdateMethod) {
        self.pin_b.set_update_method(update_method)
    }

    /// Write a new timestamp.
    /// The written value will take effect according to the set
    /// [`PwmUpdateMethod`].
    pub fn set_timestamp_a(&mut self, value: u16) {
        self.pin_a.set_timestamp(value)
    }
    /// Write a new timestamp.
    /// The written value will take effect according to the set
    /// [`PwmUpdateMethod`].
    pub fn set_timestamp_b(&mut self, value: u16) {
        self.pin_a.set_timestamp(value)
    }

    /// Configure the deadtime generator
    pub fn set_deadtime_cfg(&mut self, config: DeadTimeCfg) {
        #[cfg(esp32s3)]
        let dt_cfg = unsafe { Self::ch() }.db_cfg();
        #[cfg(not(esp32s3))]
        let dt_cfg = unsafe { Self::ch() }.dt_cfg();
        dt_cfg.write(|w| unsafe { w.bits(config.cfg_reg) });
    }

    /// Set the deadtime generator rising edge delay
    pub fn set_rising_edge_deadtime(&mut self, dead_time: u16) {
        #[cfg(esp32s3)]
        let dt_red = unsafe { Self::ch() }.db_red_cfg();
        #[cfg(not(esp32s3))]
        let dt_red = unsafe { Self::ch() }.dt_red_cfg();
        dt_red.write(|w| unsafe { w.red().bits(dead_time) });
    }
    /// Set the deadtime generator falling edge delay
    pub fn set_falling_edge_deadtime(&mut self, dead_time: u16) {
        #[cfg(esp32s3)]
        let dt_fed = unsafe { Self::ch() }.db_fed_cfg();
        #[cfg(not(esp32s3))]
        let dt_fed = unsafe { Self::ch() }.dt_fed_cfg();
        dt_fed.write(|w| unsafe { w.fed().bits(dead_time) });
    }

    unsafe fn ch() -> &'static crate::peripherals::mcpwm0::CH {
        let block = unsafe { &*PWM::block() };
        block.ch(OP as usize)
    }
}

/// An action the operator applies to an output
#[non_exhaustive]
#[repr(u32)]
pub enum UpdateAction {
    /// Clear the output by setting it to a low level.
    SetLow  = 1,
    /// Set the output to a high level.
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
