use core::marker::PhantomData;

use crate::{
    gpio::OutputPin,
    mcpwm::{timer::Timer, PwmPeripheral},
    peripheral::{Peripheral, PeripheralRef},
};

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
        block.operator_timersel.modify(|_, w| match OP {
            0 => w.operator0_timersel().variant(TIM),
            1 => w.operator1_timersel().variant(TIM),
            2 => w.operator2_timersel().variant(TIM),
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
                (0, true) => block.gen0_a.write(|w| w.bits(bits)),
                (1, true) => block.gen1_a.write(|w| w.bits(bits)),
                (2, true) => block.gen2_a.write(|w| w.bits(bits)),
                (0, false) => block.gen0_b.write(|w| w.bits(bits)),
                (1, false) => block.gen1_b.write(|w| w.bits(bits)),
                (2, false) => block.gen2_b.write(|w| w.bits(bits)),
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
                .gen0_stmp_cfg
                .modify(|_, w| w.gen0_a_upmethod().variant(bits)),
            (1, true) => block
                .gen1_stmp_cfg
                .modify(|_, w| w.gen1_a_upmethod().variant(bits)),
            (2, true) => block
                .gen2_stmp_cfg
                .modify(|_, w| w.gen2_a_upmethod().variant(bits)),
            (0, false) => block
                .gen0_stmp_cfg
                .modify(|_, w| w.gen0_b_upmethod().variant(bits)),
            (1, false) => block
                .gen1_stmp_cfg
                .modify(|_, w| w.gen1_b_upmethod().variant(bits)),
            (2, false) => block
                .gen2_stmp_cfg
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
    #[cfg(esp32c6)]
    pub fn set_update_method(&mut self, update_method: PwmUpdateMethod) {
        // SAFETY:
        // We only write to our GENx_x_UPMETHOD register
        let block = unsafe { &*PWM::block() };
        let bits = update_method.0;
        match (OP, IS_A) {
            (0, true) => block
                .gen0_stmp_cfg
                .modify(|_, w| w.cmpr0_a_upmethod().variant(bits)),
            (1, true) => block
                .gen1_stmp_cfg
                .modify(|_, w| w.cmpr1_a_upmethod().variant(bits)),
            (2, true) => block
                .gen2_stmp_cfg
                .modify(|_, w| w.cmpr2_a_upmethod().variant(bits)),
            (0, false) => block
                .gen0_stmp_cfg
                .modify(|_, w| w.cmpr0_b_upmethod().variant(bits)),
            (1, false) => block
                .gen1_stmp_cfg
                .modify(|_, w| w.cmpr1_b_upmethod().variant(bits)),
            (2, false) => block
                .gen2_stmp_cfg
                .modify(|_, w| w.cmpr2_b_upmethod().variant(bits)),
            _ => {
                unreachable!()
            }
        }
    }

    /// Set how a new timestamp syncs with the timer.
    /// The written value will take effect according to the set
    /// [`PwmUpdateMethod`].
    #[cfg(esp32)]
    pub fn set_timestamp(&mut self, value: u16) {
        // SAFETY:
        // We only write to our GENx_TSTMP_x register
        let block = unsafe { &*PWM::block() };
        match (OP, IS_A) {
            (0, true) => block.gen0_tstmp_a.write(|w| w.gen0_a().variant(value)),
            (1, true) => block.gen1_tstmp_a.write(|w| w.gen1_a().variant(value)),
            (2, true) => block.gen2_tstmp_a.write(|w| w.gen2_a().variant(value)),
            (0, false) => block.gen0_tstmp_b.write(|w| w.gen0_b().variant(value)),
            (1, false) => block.gen1_tstmp_b.write(|w| w.gen1_b().variant(value)),
            (2, false) => block.gen2_tstmp_b.write(|w| w.gen2_b().variant(value)),
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
        // We only write to our GENx_TSTMP_x register
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

    /// Write a new timestamp.
    /// The written value will take effect according to the set
    /// [`PwmUpdateMethod`].
    #[cfg(esp32c6)]
    pub fn set_timestamp(&mut self, value: u16) {
        // SAFETY:
        // We only write to our GENx_TSTMP_x register
        let block = unsafe { &*PWM::block() };
        match (OP, IS_A) {
            (0, true) => block.gen0_tstmp_a.write(|w| w.cmpr0_a().variant(value)),
            (1, true) => block.gen1_tstmp_a.write(|w| w.cmpr1_a().variant(value)),
            (2, true) => block.gen2_tstmp_a.write(|w| w.cmpr2_a().variant(value)),
            (0, false) => block.gen0_tstmp_b.write(|w| w.cmpr0_b().variant(value)),
            (1, false) => block.gen1_tstmp_b.write(|w| w.cmpr1_b().variant(value)),
            (2, false) => block.gen2_tstmp_b.write(|w| w.cmpr2_b().variant(value)),
            _ => {
                unreachable!()
            }
        }
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
