use core::marker::PhantomData;

use crate::{
    mcpwm::{timer::Timer, PwmPeripheral},
    OutputPin,
};

pub struct Operator<const O: u8, PWM> {
    phantom: PhantomData<PWM>,
}

impl<const O: u8, PWM: PwmPeripheral> Operator<O, PWM> {
    pub(super) fn new() -> Self {
        // TODO maybe set timersel to 3 to disable?

        Operator {
            phantom: PhantomData,
        }
    }

    /// Select which [`Timer`] is the timing reference for this operator
    ///
    /// ### Note:
    /// By default TIMER0 is used
    pub fn set_timer<const T: u8>(&mut self, timer: &Timer<T, PWM>) {
        let _ = timer;
        let block = unsafe { &*PWM::block() };
        block.operator_timersel.modify(|_, w| match O {
            0 => w.operator0_timersel().variant(T),
            1 => w.operator1_timersel().variant(T),
            2 => w.operator2_timersel().variant(T),
            _ => {
                unreachable!()
            }
        });
    }

    pub fn with_a_pin<Pin: OutputPin>(
        self,
        pin: Pin,
        actions: PwmActions<true>,
    ) -> PwmPin<Pin, PWM, O, true> {
        let mut pin = PwmPin::new(pin);
        pin.set_actions(actions);
        pin
    }
    pub fn with_b_pin<Pin: OutputPin>(
        self,
        pin: Pin,
        actions: PwmActions<false>,
    ) -> PwmPin<Pin, PWM, O, false> {
        let mut pin = PwmPin::new(pin);
        pin.set_actions(actions);
        pin
    }
    pub fn with_pins<PinA: OutputPin, PinB: OutputPin>(
        self,
        pin_a: PinA,
        actions_a: PwmActions<true>,
        pin_b: PinB,
        actions_b: PwmActions<false>,
    ) -> (PwmPin<PinA, PWM, O, true>, PwmPin<PinB, PWM, O, false>) {
        let self2 = Self::new();
        (
            self.with_a_pin(pin_a, actions_a),
            self2.with_b_pin(pin_b, actions_b),
        )
    }
}

pub struct PwmPin<Pin, PWM, const O: u8, const IS_A: bool> {
    _pin: Pin,
    phantom: PhantomData<PWM>,
}

impl<Pin: OutputPin, PWM: PwmPeripheral, const O: u8, const IS_A: bool> PwmPin<Pin, PWM, O, IS_A> {
    fn new(mut pin: Pin) -> Self {
        let output_signal = PWM::output_signal::<O, IS_A>();
        pin.enable_output(true)
            .connect_peripheral_to_output(output_signal);
        PwmPin {
            _pin: pin,
            phantom: PhantomData,
        }
    }

    // TODO mention that using A on B and vice versa is not supported
    // TODO should this be public?
    pub fn set_actions(&mut self, value: PwmActions<IS_A>) {
        // SAFETY:
        // We only grant access to our GENx_x register with the lifetime of &mut self
        let block = unsafe { &*PWM::block() };

        let bits = value.0;

        // SAFETY:
        // `bits` is a valid bit pattern
        unsafe {
            match (O, IS_A) {
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

    #[cfg(esp32)]
    pub fn set_timestamp(&mut self, value: u16) {
        let block = unsafe { &*PWM::block() };
        unsafe {
            match (O, IS_A) {
                (0, true) => &block.gen0_tstmp_a.write(|w| w.gen0_a().bits(value)),
                (1, true) => &block.gen1_tstmp_a.write(|w| w.gen1_a().bits(value)),
                (2, true) => &block.gen2_tstmp_a.write(|w| w.gen2_a().bits(value)),
                (0, false) => &block.gen0_tstmp_b.write(|w| w.gen0_b().bits(value)),
                (1, false) => &block.gen1_tstmp_b.write(|w| w.gen1_b().bits(value)),
                (2, false) => &block.gen2_tstmp_b.write(|w| w.gen2_b().bits(value)),
                _ => {
                    unreachable!()
                }
            }
        };
    }

    #[cfg(esp32s3)]
    pub fn set_timestamp(&mut self, value: u16) {
        let block = unsafe { &*PWM::block() };
        unsafe {
            match (O, IS_A) {
                (0, true) => &block.cmpr0_value0.write(|w| w.cmpr0_a().bits(value)),
                (1, true) => &block.cmpr1_value0.write(|w| w.cmpr1_a().bits(value)),
                (2, true) => &block.cmpr2_value0.write(|w| w.cmpr2_a().bits(value)),
                (0, false) => &block.cmpr0_value1.write(|w| w.cmpr0_b().bits(value)),
                (1, false) => &block.cmpr1_value1.write(|w| w.cmpr1_b().bits(value)),
                (2, false) => &block.cmpr2_value1.write(|w| w.cmpr2_b().bits(value)),
                _ => {
                    unreachable!()
                }
            }
        };
    }
}

#[repr(u32)]
pub enum UpdateAction {
    None    = 0,
    SetLow  = 1,
    SetHigh = 2,
    Toggle  = 3,
}

pub struct PwmActions<const IS_A: bool>(u32);

impl<const IS_A: bool> PwmActions<IS_A> {
    pub const UP_ACTIVE_HIGH: Self = Self::empty()
        .on_up_counting_timer_equals_zero(UpdateAction::SetHigh)
        .on_up_counting_timer_equals_timestamp(UpdateAction::SetLow);

    pub const UP_DOWN_ACTIVE_HIGH: Self = Self::empty()
        .on_down_counting_timer_equals_timestamp(UpdateAction::SetHigh)
        .on_up_counting_timer_equals_timestamp(UpdateAction::SetLow);

    pub const fn empty() -> Self {
        PwmActions(0)
    }
    pub const fn on_up_counting_timer_equals_zero(self, action: UpdateAction) -> Self {
        self.with_value_at_offset(action as u32, 0)
    }
    pub const fn on_up_counting_timer_equals_period(self, action: UpdateAction) -> Self {
        self.with_value_at_offset(action as u32, 2)
    }
    pub const fn on_up_counting_timer_equals_timestamp(self, action: UpdateAction) -> Self {
        match IS_A {
            true => self.with_value_at_offset(action as u32, 4),
            false => self.with_value_at_offset(action as u32, 6),
        }
    }

    pub const fn on_down_counting_timer_equals_zero(self, action: UpdateAction) -> Self {
        self.with_value_at_offset(action as u32, 12)
    }
    pub const fn on_down_counting_timer_equals_period(self, action: UpdateAction) -> Self {
        self.with_value_at_offset(action as u32, 14)
    }
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
