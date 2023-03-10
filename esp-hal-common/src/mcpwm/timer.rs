use core::marker::PhantomData;

use fugit::HertzU32;

use crate::{
    clock::Clocks,
    mcpwm::{FrequencyError, PeripheralClockConfig, PwmPeripheral},
    peripherals::mcpwm0::{TIMER0_CFG0, TIMER0_CFG1},
};

/// A MCPWM timer
///
/// Every timer of a particular [`MCPWM`](super::MCPWM) peripheral can be used
/// as a timing reference for every
/// [`Operator`](super::operator::Operator) of that peripheral
pub struct Timer<const TIM: u8, PWM> {
    pub(super) phantom: PhantomData<PWM>,
}

impl<const TIM: u8, PWM: PwmPeripheral> Timer<TIM, PWM> {
    pub(super) fn new() -> Self {
        Timer {
            phantom: PhantomData,
        }
    }

    /// Apply the given timer configuration.
    ///
    /// ### Note:
    /// The prescalar and period configuration will be applied immediately and
    /// before setting the [`PwmWorkingMode`].
    /// If the timer is already running you might want to call [`Timer::stop`]
    /// and/or [`Timer::set_counter`] first
    /// (if the new period is larger than the current counter value this will
    /// cause weird behavior).
    ///
    /// The hardware supports writing these settings in sync with certain timer
    /// events but this HAL does not expose these for now.
    pub fn start(&mut self, timer_config: TimerClockConfig) {
        // write prescaler and period with immediate update method
        self.cfg0().write(|w| {
            w.timer0_prescale()
                .variant(timer_config.prescaler)
                .timer0_period()
                .variant(timer_config.period)
                .timer0_period_upmethod()
                .variant(0)
        });

        // set timer to continuously run and set the timer working mode
        self.cfg1().write(|w| {
            w.timer0_start()
                .variant(2)
                .timer0_mod()
                .variant(timer_config.mode as u8)
        });
    }

    /// Stop the timer in its current state
    pub fn stop(&mut self) {
        // freeze the timer
        self.cfg1().write(|w| w.timer0_mod().variant(0));
    }

    /// Set the timer counter to the provided value
    pub fn set_counter(&mut self, phase: u16, direction: CounterDirection) {
        // SAFETY:
        // We only write to our TIMERx_SYNC register
        let block = unsafe { &*PWM::block() };

        match TIM {
            0 => {
                let sw = block.timer0_sync.read().sw().bit_is_set();
                block.timer0_sync.write(|w| {
                    w.timer0_phase_direction()
                        .variant(direction as u8 != 0)
                        .timer0_phase()
                        .variant(phase)
                        .sw()
                        .variant(!sw)
                });
            }
            1 => {
                let sw = block.timer1_sync.read().sw().bit_is_set();
                block.timer1_sync.write(|w| {
                    w.timer1_phase_direction()
                        .variant(direction as u8 != 0)
                        .timer1_phase()
                        .variant(phase)
                        .sw()
                        .variant(!sw)
                });
            }
            2 => {
                let sw = block.timer2_sync.read().sw().bit_is_set();
                block.timer2_sync.write(|w| {
                    w.timer2_phase_direction()
                        .variant(direction as u8 != 0)
                        .timer2_phase()
                        .variant(phase)
                        .sw()
                        .variant(!sw)
                });
            }
            _ => unreachable!(),
        }
    }

    /// Read the counter value and counter direction of the timer
    pub fn status(&self) -> (u16, CounterDirection) {
        // SAFETY:
        // We only read from our TIMERx_STATUS register
        let block = unsafe { &*PWM::block() };

        match TIM {
            0 => {
                let reg = block.timer0_status.read();
                (
                    reg.timer0_value().bits(),
                    reg.timer0_direction().bit_is_set().into(),
                )
            }
            1 => {
                let reg = block.timer1_status.read();
                (
                    reg.timer1_value().bits(),
                    reg.timer1_direction().bit_is_set().into(),
                )
            }
            2 => {
                let reg = block.timer2_status.read();
                (
                    reg.timer2_value().bits(),
                    reg.timer2_direction().bit_is_set().into(),
                )
            }
            _ => unreachable!(),
        }
    }

    fn cfg0(&mut self) -> &TIMER0_CFG0 {
        // SAFETY:
        // We only grant access to our CFG0 register with the lifetime of &mut self
        let block = unsafe { &*PWM::block() };

        // SAFETY:
        // The CFG0 registers are identical for all timers so we can pretend they're
        // TIMER0_CFG0
        match TIM {
            0 => &block.timer0_cfg0,
            1 => unsafe { &*(&block.timer1_cfg0 as *const _ as *const _) },
            2 => unsafe { &*(&block.timer2_cfg0 as *const _ as *const _) },
            _ => unreachable!(),
        }
    }

    fn cfg1(&mut self) -> &TIMER0_CFG1 {
        // SAFETY:
        // We only grant access to our CFG1 register with the lifetime of &mut self
        let block = unsafe { &*PWM::block() };

        // SAFETY:
        // The CFG1 registers are identical for all timers so we can pretend they're
        // TIMER0_CFG1
        match TIM {
            0 => &block.timer0_cfg1,
            1 => unsafe { &*(&block.timer1_cfg1 as *const _ as *const _) },
            2 => unsafe { &*(&block.timer2_cfg1 as *const _ as *const _) },
            _ => unreachable!(),
        }
    }
}

/// Clock configuration of a MCPWM timer
///
/// Use [`PeripheralClockConfig::timer_clock_with_prescaler`](super::PeripheralClockConfig::timer_clock_with_prescaler) or
/// [`PeripheralClockConfig::timer_clock_with_frequency`](super::PeripheralClockConfig::timer_clock_with_frequency) to it.
#[derive(Copy, Clone)]
pub struct TimerClockConfig<'a> {
    frequency: HertzU32,
    period: u16,
    prescaler: u8,
    mode: PwmWorkingMode,
    phantom: PhantomData<&'a Clocks<'a>>,
}

impl<'a> TimerClockConfig<'a> {
    pub(super) fn with_prescaler(
        clock: &PeripheralClockConfig<'a>,
        period: u16,
        mode: PwmWorkingMode,
        prescaler: u8,
    ) -> Self {
        let cycle_period = match mode {
            PwmWorkingMode::Increase | PwmWorkingMode::Decrease => period as u32 + 1,
            // The reference manual seems to provide an incorrect formula for UpDown
            PwmWorkingMode::UpDown => period as u32 * 2,
        };
        let frequency = clock.frequency / (prescaler as u32 + 1) / cycle_period;

        TimerClockConfig {
            frequency,
            prescaler,
            period,
            mode,
            phantom: PhantomData,
        }
    }

    pub(super) fn with_frequency(
        clock: &PeripheralClockConfig<'a>,
        period: u16,
        mode: PwmWorkingMode,
        target_freq: HertzU32,
    ) -> Result<Self, FrequencyError> {
        let cycle_period = match mode {
            PwmWorkingMode::Increase | PwmWorkingMode::Decrease => period as u32 + 1,
            // The reference manual seems to provide an incorrect formula for UpDown
            PwmWorkingMode::UpDown => period as u32 * 2,
        };
        let target_timer_frequency = target_freq
            .raw()
            .checked_mul(cycle_period)
            .ok_or(FrequencyError)?;
        if target_timer_frequency == 0 || target_freq > clock.frequency {
            return Err(FrequencyError);
        }
        let prescaler = clock.frequency.raw() / target_timer_frequency - 1;
        if prescaler > u8::MAX as u32 {
            return Err(FrequencyError);
        }
        let frequency = clock.frequency / (prescaler + 1) / cycle_period;

        Ok(TimerClockConfig {
            frequency,
            prescaler: prescaler as u8,
            period,
            mode,
            phantom: PhantomData,
        })
    }

    /// Get the timer clock frequency.
    ///
    /// ### Note:
    /// The actual value is rounded down to the nearest `u32` value
    pub fn frequency(&self) -> HertzU32 {
        self.frequency
    }
}

/// PWM working mode
#[derive(Copy, Clone)]
#[repr(u8)]
pub enum PwmWorkingMode {
    /// In this mode, the PWM timer increments from zero until reaching the
    /// value configured in the period field. Once done, the PWM timer
    /// returns to zero and starts increasing again. PWM period is equal to the
    /// value of the period field + 1.
    Increase = 1,
    /// The PWM timer decrements to zero, starting from the value configured in
    /// the period field. After reaching zero, it is set back to the period
    /// value. Then it starts to decrement again. In this case, the PWM period
    /// is also equal to the value of period field + 1.
    Decrease = 2,
    /// This is a combination of the two modes mentioned above. The PWM timer
    /// starts increasing from zero until the period value is reached. Then,
    /// the timer decreases back to zero. This pattern is then repeated. The
    /// PWM period is the result of the value of the period field × 2.
    UpDown   = 3,
}

/// The direction the timer counter is changing
#[derive(Debug)]
#[repr(u8)]
pub enum CounterDirection {
    /// The timer counter is increasing
    Increasing = 0,
    /// The timer counter is decreasing
    Decreasing = 1,
}

impl From<bool> for CounterDirection {
    fn from(bit: bool) -> Self {
        match bit {
            false => CounterDirection::Increasing,
            true => CounterDirection::Decreasing,
        }
    }
}
