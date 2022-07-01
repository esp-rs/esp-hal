//! General-purpose timers

use embedded_hal::{
    timer::{Cancel, CountDown, Periodic},
    watchdog::WatchdogDisable,
};
use fugit::{MegahertzU32, MicrosDurationU64};
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
    apb_clk_freq: MegahertzU32,
}

/// Timer driver
impl<T> Timer<T>
where
    T: Instance,
{
    /// Create a new timer instance
    pub fn new(timg: T, apb_clk_freq: MegahertzU32) -> Self {
        // TODO: this currently assumes APB_CLK is being used, as we don't yet have a
        //       way to select the XTAL_CLK.
        Self { timg, apb_clk_freq }
    }

    /// Return the raw interface to the underlying timer instance
    pub fn free(self) -> T {
        self.timg
    }

    /// Listen for interrupt
    pub fn listen(&mut self) {
        self.timg.listen();
    }

    /// Stop listening for interrupt
    pub fn unlisten(&mut self) {
        self.timg.unlisten();
    }

    /// Clear intterupt status
    pub fn clear_interrupt(&mut self) {
        self.timg.clear_interrupt();
    }

    /// Read current raw timer value in timer ticks
    pub fn read_raw(&self) -> u64 {
        self.timg.read_raw()
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

    fn is_counter_active(&self) -> bool {
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

    fn is_alarm_active(&self) -> bool {
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

    fn listen(&mut self) {
        // always use level interrupt
        #[cfg(any(feature = "esp32", feature = "esp32s2"))]
        self.register_block()
            .t0config
            .modify(|_, w| w.t0_level_int_en().set_bit());

        self.register_block()
            .int_ena_timers
            .modify(|_, w| w.t0_int_ena().set_bit());
    }

    fn unlisten(&mut self) {
        self.register_block()
            .int_ena_timers
            .modify(|_, w| w.t0_int_ena().clear_bit());
    }

    fn clear_interrupt(&mut self) {
        self.register_block()
            .int_clr_timers
            .write(|w| w.t0_int_clr().set_bit());
    }

    fn read_raw(&self) -> u64 {
        self.register_block()
            .t0update
            .write(|w| unsafe { w.bits(0) });

        let value_lo = self.register_block().t0lo.read().bits() as u64;
        let value_hi = (self.register_block().t0hi.read().bits() as u64) << 32;

        (value_lo | value_hi) as u64
    }

    fn divider(&self) -> u32 {
        // From the ESP32 TRM, "11.2.1 16­-bit Prescaler and Clock Selection":
        //
        // "The prescaler can divide the APB clock by a factor from 2 to 65536.
        // Specifically, when TIMGn_Tx_DIVIDER is either 1 or 2, the clock divisor is 2;
        // when TIMGn_Tx_DIVIDER is 0, the clock divisor is 65536. Any other value will
        // cause the clock to be divided by exactly that value."
        match self.register_block().t0config.read().t0_divider().bits() {
            0 => 65536,
            1 | 2 => 2,
            n => n as u32,
        }
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

fn timeout_to_ticks<T, F>(timeout: T, clock: F, divider: u32) -> u64
where
    T: Into<MicrosDurationU64>,
    F: Into<MegahertzU32>,
{
    let timeout: MicrosDurationU64 = timeout.into();
    let micros = timeout.to_micros();

    let clock: MegahertzU32 = clock.into();

    // TODO can we get this to not use doubles/floats
    let period = 1_000_000f64 / (clock.to_Hz() as f64 / divider as f64); // micros

    (micros as f64 / period) as u64
}

impl<T> CountDown for Timer<T>
where
    T: Instance,
{
    type Time = MicrosDurationU64;

    fn start<Time>(&mut self, timeout: Time)
    where
        Time: Into<Self::Time>,
    {
        self.timg.set_counter_active(false);
        self.timg.set_alarm_active(false);

        self.timg.reset_counter();

        // TODO: this currently assumes APB_CLK is being used, as we don't yet have a
        //       way to select the XTAL_CLK.
        // TODO: can we cache the divider (only get it on initialization)?
        let ticks = timeout_to_ticks(timeout, self.apb_clk_freq, self.timg.divider());
        self.timg.load_alarm_value(ticks);

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
