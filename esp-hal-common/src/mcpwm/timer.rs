use core::marker::PhantomData;

use crate::mcpwm::PwmPeripheral;

/// PWM working mode
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
    /// PWM period is the result of (the value of the period field × 2 + 1).
    UpDown   = 3,
}

pub struct Timer<const T: u8, PWM> {
    pub(super) phantom: PhantomData<PWM>,
}

impl<const T: u8, PWM: PwmPeripheral> Timer<T, PWM> {
    pub(super) fn new() -> Self {
        Timer {
            phantom: PhantomData,
        }
    }

    /// Set the prescaler, period and then the mode
    ///
    /// ### Note:
    /// `prescaler` and `period` will be applied immediately.
    /// If the timer is already running you might want to call [`stop`] first.
    ///
    /// Note also that the hardware supports writing these settings
    /// in sync with certain timer events but this HAL does not expose these for
    /// now.
    pub fn start(&mut self, prescaler: u8, period: u16, mode: PwmWorkingMode) {
        // write prescaler and period with immediate update method
        unsafe {
            self.cfg0().write(|w| {
                w.timer0_prescale()
                    .bits(prescaler)
                    .timer0_period()
                    .bits(period)
                    .timer0_period_upmethod()
                    .variant(0)
            });
        }

        // set timer to continuously run and set the timer working mode
        self.cfg1()
            .write(|w| w.timer0_start().variant(2).timer0_mod().variant(mode as u8));
    }

    pub fn stop(&mut self) {
        // freeze the timer
        self.cfg1().write(|w| w.timer0_mod().variant(0));
    }

    pub fn status(&self) -> (u16, CounterDirection) {
        let block = unsafe { &*PWM::block() };

        match T {
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
            _ => {
                unreachable!()
            }
        }
    }

    fn cfg0(&mut self) -> &crate::pac::pwm0::TIMER0_CFG0 {
        // SAFETY:
        // We only grant access to our CFG0 register with the lifetime of &mut self
        let block = unsafe { &*PWM::block() };

        // SAFETY:
        // The CFG0 registers are identical for all timers so we can pretend they're
        // TIMER0_CFG0
        match T {
            0 => &block.timer0_cfg0,
            1 => unsafe { &*(&block.timer1_cfg0 as *const _ as *const _) },
            2 => unsafe { &*(&block.timer2_cfg0 as *const _ as *const _) },
            _ => {
                unreachable!()
            }
        }
    }
    fn cfg1(&mut self) -> &crate::pac::pwm0::TIMER0_CFG1 {
        // SAFETY:
        // We only grant access to our CFG1 register with the lifetime of &mut self
        let block = unsafe { &*PWM::block() };

        // SAFETY:
        // The CFG1 registers are identical for all timers so we can pretend they're
        // TIMER0_CFG1
        match T {
            0 => &block.timer0_cfg1,
            1 => unsafe { &*(&block.timer1_cfg1 as *const _ as *const _) },
            2 => unsafe { &*(&block.timer2_cfg1 as *const _ as *const _) },
            _ => {
                unreachable!()
            }
        }
    }
}

#[derive(Debug)]
pub enum CounterDirection {
    Increasing,
    Decreasing,
}

impl From<bool> for CounterDirection {
    fn from(bit: bool) -> Self {
        match bit {
            false => CounterDirection::Increasing,
            true => CounterDirection::Decreasing,
        }
    }
}
