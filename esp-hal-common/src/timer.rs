//! General-purpose timers

use embedded_hal::{
    timer::{Cancel, CountDown, Periodic},
    watchdog::WatchdogDisable,
};
use void::Void;

use crate::pac::{timg0::RegisterBlock, TIMG0, TIMG1};

/// Custom timer error type
#[derive(Debug)]
pub enum Error {
    TimerActive,
    TimerInactive,
    AlarmInactive,
}

/// General-purpose timer
pub struct Timer<T> {
    timg: T,
}

/// Timer driver
impl<T> Timer<T>
where
    T: Instance,
{
    /// Create a new timer instance
    pub fn new(timg: T) -> Self {
        Self { timg }
    }

    /// Return the raw interface to the underlying timer instance
    pub fn free(self) -> T {
        self.timg
    }
}

/// Timer peripheral instance
pub trait Instance {
    fn register_block(&self) -> &RegisterBlock;

    fn reset_counter(&mut self) {
        let reg_block = self.register_block();

        reg_block
            .t0loadlo
            .write(|w| unsafe { w.t0_load_lo().bits(0) });

        reg_block
            .t0loadhi
            .write(|w| unsafe { w.t0_load_hi().bits(0) });

        reg_block.t0load.write(|w| unsafe { w.t0_load().bits(1) });
    }

    fn set_counter_active(&mut self, state: bool) {
        self.register_block()
            .t0config
            .modify(|_, w| w.t0_en().bit(state));
    }

    fn is_counter_active(&mut self) -> bool {
        self.register_block().t0config.read().t0_en().bit_is_set()
    }

    fn set_counter_decrementing(&mut self, decrementing: bool) {
        self.register_block()
            .t0config
            .modify(|_, w| w.t0_increase().bit(!decrementing));
    }

    fn set_auto_reload(&mut self, auto_reload: bool) {
        self.register_block()
            .t0config
            .modify(|_, w| w.t0_autoreload().bit(auto_reload));
    }

    fn set_alarm_active(&mut self, state: bool) {
        self.register_block()
            .t0config
            .modify(|_, w| w.t0_alarm_en().bit(state));
    }

    fn is_alarm_active(&mut self) -> bool {
        self.register_block()
            .t0config
            .read()
            .t0_alarm_en()
            .bit_is_set()
    }

    fn load_alarm_value(&mut self, value: u64) {
        let value = value & 0x3F_FFFF_FFFF_FFFF;
        let high = (value >> 32) as u32;
        let low = (value & 0xFFFF_FFFF) as u32;

        let reg_block = self.register_block();

        reg_block
            .t0alarmlo
            .write(|w| unsafe { w.t0_alarm_lo().bits(low) });

        reg_block
            .t0alarmhi
            .write(|w| unsafe { w.t0_alarm_hi().bits(high) });
    }

    fn set_wdt_enabled(&mut self, enabled: bool) {
        let reg_block = self.register_block();

        reg_block
            .wdtwprotect
            .write(|w| unsafe { w.wdt_wkey().bits(0x50D8_3AA1u32) });

        if !enabled {
            reg_block.wdtconfig0.write(|w| unsafe { w.bits(0) });
        } else {
            reg_block.wdtconfig0.write(|w| w.wdt_en().bit(true));
        }

        reg_block
            .wdtwprotect
            .write(|w| unsafe { w.wdt_wkey().bits(0u32) });
    }
}

impl Instance for TIMG0 {
    #[inline(always)]
    fn register_block(&self) -> &RegisterBlock {
        self
    }
}

impl Instance for TIMG1 {
    #[inline(always)]
    fn register_block(&self) -> &RegisterBlock {
        self
    }
}

impl<T> CountDown for Timer<T>
where
    T: Instance,
{
    type Time = u64;

    fn start<Time>(&mut self, timeout: Time)
    where
        Time: Into<u64>,
    {
        self.timg.set_counter_active(false);
        self.timg.set_alarm_active(false);

        self.timg.reset_counter();
        self.timg.load_alarm_value(timeout.into());

        self.timg.set_counter_decrementing(false);
        self.timg.set_auto_reload(true);
        self.timg.set_counter_active(true);
        self.timg.set_alarm_active(true);
    }

    fn wait(&mut self) -> nb::Result<(), Void> {
        if !self.timg.is_counter_active() {
            panic!("Called wait on an inactive timer!")
        }

        let reg_block = self.timg.register_block();

        if reg_block.int_raw_timers.read().t0_int_raw().bit_is_set() {
            reg_block.int_clr_timers.write(|w| w.t0_int_clr().set_bit());
            self.timg.set_alarm_active(true);

            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

impl<T> Cancel for Timer<T>
where
    T: Instance,
{
    type Error = Error;

    fn cancel(&mut self) -> Result<(), Error> {
        if !self.timg.is_counter_active() {
            return Err(Error::TimerInactive);
        } else if !self.timg.is_alarm_active() {
            return Err(Error::AlarmInactive);
        }

        self.timg.set_counter_active(false);

        Ok(())
    }
}

impl<T> Periodic for Timer<T> where T: Instance {}

impl<T> WatchdogDisable for Timer<T>
where
    T: Instance,
{
    fn disable(&mut self) {
        self.timg.set_wdt_enabled(false);
    }
}
