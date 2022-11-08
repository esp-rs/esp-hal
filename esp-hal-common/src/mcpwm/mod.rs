use core::marker::PhantomData;

use fugit::HertzU32;
use operator::Operator;
use timer::Timer;

use crate::{
    clock::Clocks,
    system::{Peripheral, PeripheralClockControl},
    types::OutputSignal,
};

pub mod operator;
pub mod timer;

#[non_exhaustive]
pub struct MCPWM<'a, PWM> {
    pub pwm_clk: PwmClock<'a>,
    pub timer0: Timer<0, PWM>,
    pub timer1: Timer<1, PWM>,
    pub timer2: Timer<2, PWM>,
    pub operator0: Operator<0, PWM>,
    pub operator1: Operator<1, PWM>,
    pub operator2: Operator<2, PWM>,
}

impl<'a, PWM: PwmPeripheral> MCPWM<'a, PWM> {
    /// `pwm_clk = clocks.crypto_pwm_clock / (prescaler + 1)`
    // clocks.crypto_pwm_clock normally is 160 MHz
    pub fn new(
        peripheral: PWM,
        clocks: &'a Clocks,
        prescaler: u8,
        system: &mut PeripheralClockControl,
    ) -> Self {
        let _ = peripheral;

        PWM::enable(system);

        let block = unsafe { &*PWM::block() };
        // set prescaler
        block.clk_cfg.write(|w| w.clk_prescale().variant(prescaler));
        // enable clock
        block.clk.write(|w| w.en().set_bit());

        MCPWM {
            pwm_clk: PwmClock::new(clocks, prescaler),
            timer0: Timer::new(),
            timer1: Timer::new(),
            timer2: Timer::new(),
            operator0: Operator::new(),
            operator1: Operator::new(),
            operator2: Operator::new(),
        }
    }
}

pub struct PwmClock<'a> {
    pwm_clock: HertzU32,
    phantom: PhantomData<&'a Clocks>,
}

impl<'a> PwmClock<'a> {
    pub fn clock_frequency(&self) -> HertzU32 {
        self.pwm_clock
    }

    pub fn timer_frequency(
        &self,
        mode: timer::PwmWorkingMode,
        prescaler: u8,
        period: u16,
    ) -> HertzU32 {
        let period = match mode {
            timer::PwmWorkingMode::Increase | timer::PwmWorkingMode::Decrease => period as u32 + 1,
            // The reference manual seems to provide an incorrect formula for UpDown
            timer::PwmWorkingMode::UpDown => period as u32 * 2,
        };
        self.pwm_clock / (prescaler as u32 + 1) / period
    }

    #[cfg(esp32)]
    fn new(clocks: &'a Clocks, prescaler: u8) -> Self {
        // TODO Docs are unclear here, need to test this

        PwmClock {
            pwm_clock: clocks.apb_clock / (prescaler as u32 + 1),
            phantom: PhantomData,
        }
    }

    #[cfg(esp32s3)]
    fn new(clocks: &'a Clocks, prescaler: u8) -> Self {
        PwmClock {
            pwm_clock: clocks.crypto_pwm_clock / (prescaler as u32 + 1),
            phantom: PhantomData,
        }
    }
}

/// A MCPWM peripheral
pub unsafe trait PwmPeripheral {
    /// Enable peripheral
    fn enable(system: &mut PeripheralClockControl);
    /// Get a pointer to the peripheral RegisterBlock
    fn block() -> *const crate::pac::pwm0::RegisterBlock;
    /// Get operator GPIO mux output signal
    fn output_signal<const O: u8, const IS_A: bool>() -> OutputSignal;
}

unsafe impl PwmPeripheral for crate::pac::PWM0 {
    fn enable(system: &mut PeripheralClockControl) {
        system.enable(Peripheral::Mcpwm0)
    }

    fn block() -> *const crate::pac::pwm0::RegisterBlock {
        Self::ptr()
    }

    fn output_signal<const O: u8, const IS_A: bool>() -> OutputSignal {
        match (O, IS_A) {
            (0, true) => OutputSignal::PWM0_0A,
            (1, true) => OutputSignal::PWM0_1A,
            (2, true) => OutputSignal::PWM0_1A,
            (0, false) => OutputSignal::PWM0_0B,
            (1, false) => OutputSignal::PWM0_1B,
            (2, false) => OutputSignal::PWM0_1B,
            _ => unreachable!(),
        }
    }
}

unsafe impl PwmPeripheral for crate::pac::PWM1 {
    fn enable(system: &mut PeripheralClockControl) {
        system.enable(Peripheral::Mcpwm1)
    }

    fn block() -> *const crate::pac::pwm0::RegisterBlock {
        Self::ptr()
    }

    fn output_signal<const O: u8, const IS_A: bool>() -> OutputSignal {
        match (O, IS_A) {
            (0, true) => OutputSignal::PWM1_0A,
            (1, true) => OutputSignal::PWM1_1A,
            (2, true) => OutputSignal::PWM1_1A,
            (0, false) => OutputSignal::PWM1_0B,
            (1, false) => OutputSignal::PWM1_1B,
            (2, false) => OutputSignal::PWM1_1B,
            _ => unreachable!(),
        }
    }
}
