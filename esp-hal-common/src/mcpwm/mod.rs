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
    pub peripheral_clock: PeripheralClockConfig<'a>,
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
        peripheral_clock: PeripheralClockConfig<'a>,
        system: &mut PeripheralClockControl,
    ) -> Self {
        let _ = peripheral;

        PWM::enable(system);

        let block = unsafe { &*PWM::block() };
        // set prescaler
        block
            .clk_cfg
            .write(|w| w.clk_prescale().variant(peripheral_clock.prescaler));
        // enable clock
        block.clk.write(|w| w.en().set_bit());

        MCPWM {
            peripheral_clock,
            timer0: Timer::new(),
            timer1: Timer::new(),
            timer2: Timer::new(),
            operator0: Operator::new(),
            operator1: Operator::new(),
            operator2: Operator::new(),
        }
    }
}

#[derive(Copy, Clone)]
pub struct PeripheralClockConfig<'a> {
    frequency: HertzU32,
    prescaler: u8,
    phantom: PhantomData<&'a Clocks>,
}

impl<'a> PeripheralClockConfig<'a> {
    pub fn with_prescaler(clocks: &'a Clocks, prescaler: u8) -> Self {
        #[cfg(esp32)]
        let source_clock = clocks.pwm_clock;
        #[cfg(esp32s3)]
        let source_clock = clocks.crypto_pwm_clock;

        PeripheralClockConfig {
            frequency: source_clock / (prescaler as u32 + 1),
            prescaler,
            phantom: PhantomData,
        }
    }

    pub fn with_frequency(
        clocks: &'a Clocks,
        target_freq: HertzU32,
    ) -> Result<Self, FrequencyError> {
        #[cfg(esp32)]
        let source_clock = clocks.pwm_clock;
        #[cfg(esp32s3)]
        let source_clock = clocks.crypto_pwm_clock;

        if target_freq.raw() == 0 || target_freq > source_clock {
            return Err(FrequencyError);
        }
        let prescaler = source_clock / target_freq - 1;
        if prescaler > u8::MAX as u32 {
            return Err(FrequencyError);
        }
        Ok(Self::with_prescaler(clocks, prescaler as u8))
    }

    pub fn frequency(&self) -> HertzU32 {
        self.frequency
    }

    pub fn timer_clock_with_prescaler(
        &self,
        period: u16,
        mode: timer::PwmWorkingMode,
        prescaler: u8,
    ) -> timer::TimerClockConfig<'a> {
        timer::TimerClockConfig::with_prescaler(self, period, mode, prescaler)
    }

    pub fn timer_clock_with_frequency(
        &self,
        period: u16,
        mode: timer::PwmWorkingMode,
        target_freq: HertzU32,
    ) -> Result<timer::TimerClockConfig<'a>, FrequencyError> {
        timer::TimerClockConfig::with_frequency(self, period, mode, target_freq)
    }
}

#[derive(Debug)]
pub struct FrequencyError;

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
