use embedded_hal::{
    timer::{Cancel, CountDown, Periodic},
    watchdog::WatchdogDisable,
};
use void::Void;

use crate::pac::{timg0::RegisterBlock, TIMG0, TIMG1};

pub struct Timer<T> {
    timg: T,
}

pub enum Error {
    /// Report that the timer is active and certain management
    /// operations cannot be performed safely
    TimerActive,
    /// Report that the timer is inactive and thus not
    /// ever reaching any potentially configured alarm value
    TimerInactive,
    /// Report that the alarm functionality is disabled
    AlarmInactive,
}

impl<T> Timer<T>
where
    T: Instance,
{
    pub fn new(timg: T) -> Self {
        Self { timg }
    }
}

pub trait Instance {
    fn as_timg0(&self) -> &RegisterBlock;

    fn reset_counter(&mut self) {
        self.as_timg0()
            .t0loadhi
            .write(|w| unsafe { w.t0_load_hi().bits(0) });

        self.as_timg0()
            .t0load
            .write(|w| unsafe { w.t0_load().bits(1) });
    }

    fn set_counter_active(&mut self, state: bool) {
        self.as_timg0().t0config.modify(|_, w| w.t0_en().bit(state));
    }

    fn is_counter_active(&mut self) -> bool {
        self.as_timg0().t0config.read().t0_en().bit_is_set()
    }

    fn set_counter_decrementing(&mut self, decrementing: bool) {
        self.as_timg0()
            .t0config
            .modify(|_, w| w.t0_increase().bit(!decrementing));
    }

    fn set_auto_reload(&mut self, auto_reload: bool) {
        self.as_timg0()
            .t0config
            .modify(|_, w| w.t0_autoreload().bit(auto_reload));
    }

    fn set_alarm_active(&mut self, state: bool) {
        self.as_timg0()
            .t0config
            .modify(|_, w| w.t0_alarm_en().bit(state));
    }

    fn is_alarm_active(&mut self) -> bool {
        self.as_timg0().t0config.read().t0_alarm_en().bit_is_set()
    }

    fn load_alarm_value(&mut self, value: u64) {
        let value = value & 0x3F_FFFF_FFFF_FFFF;
        let high = (value >> 32) as u32;
        let low = (value & 0xFFFF_FFFF) as u32;

        self.as_timg0()
            .t0alarmlo
            .write(|w| unsafe { w.t0_alarm_lo().bits(low) });
        self.as_timg0()
            .t0alarmhi
            .write(|w| unsafe { w.t0_alarm_hi().bits(high) });
    }

    fn set_wdt_enabled(&mut self, enabled: bool) {
        self.as_timg0()
            .wdtwprotect
            .write(|w| unsafe { w.wdt_wkey().bits(0u32) });

        self.as_timg0()
            .wdtconfig0
            .write(|w| w.wdt_en().bit(enabled));

        self.as_timg0()
            .wdtwprotect
            .write(|w| unsafe { w.wdt_wkey().bits(0x50D8_3AA1u32) });
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
            panic!("Called wait on an inactive timer!");
        }

        let int_raw_is_clear = self
            .timg
            .as_timg0()
            .int_raw_timers
            .read()
            .t0_int_raw()
            .bit_is_clear();

        if int_raw_is_clear {
            Err(nb::Error::WouldBlock)
        } else {
            self.timg
                .as_timg0()
                .int_clr_timers
                .write(|w| w.t0_int_clr().set_bit());

            self.timg.set_alarm_active(true);

            Ok(())
        }
    }
}

impl<T> Periodic for Timer<T> where T: Instance {}

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

impl<T> WatchdogDisable for Timer<T>
where
    T: Instance,
{
    fn disable(&mut self) {
        self.timg.set_wdt_enabled(false);
    }
}

impl Instance for TIMG0 {
    #[inline(always)]
    fn as_timg0(&self) -> &RegisterBlock {
        self
    }
}

impl Instance for TIMG1 {
    #[inline(always)]
    fn as_timg0(&self) -> &RegisterBlock {
        self
    }
}
