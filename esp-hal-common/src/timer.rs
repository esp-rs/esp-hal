//! General-purpose timers

use core::{
    marker::PhantomData,
    ops::{Deref, DerefMut},
};

use embedded_hal::{
    timer::{Cancel, CountDown, Periodic},
    watchdog::{Watchdog, WatchdogDisable, WatchdogEnable},
};
use fugit::{HertzU32, MicrosDurationU64};
use void::Void;

#[cfg(timg1)]
use crate::peripherals::TIMG1;
#[cfg(any(esp32c6, esp32h2))]
use crate::soc::constants::TIMG_DEFAULT_CLK_SRC;
use crate::{
    clock::Clocks,
    peripheral::{Peripheral, PeripheralRef},
    peripherals::{timg0::RegisterBlock, TIMG0},
    system::PeripheralClockControl,
};

/// Custom timer error type
#[derive(Debug)]
pub enum Error {
    TimerActive,
    TimerInactive,
    AlarmInactive,
}

// A timergroup consisting of up to 2 timers (chip dependent) and a watchdog
// timer
pub struct TimerGroup<'d, T>
where
    T: TimerGroupInstance,
{
    _timer_group: PeripheralRef<'d, T>,
    pub timer0: Timer<Timer0<T>>,
    #[cfg(not(any(esp32c2, esp32c3, esp32c6, esp32h2)))]
    pub timer1: Timer<Timer1<T>>,
    pub wdt: Wdt<T>,
}

pub trait TimerGroupInstance {
    fn register_block() -> *const RegisterBlock;
    fn configure_src_clk();
    fn configure_wdt_src_clk();
}

impl TimerGroupInstance for TIMG0 {
    #[inline(always)]
    fn register_block() -> *const RegisterBlock {
        crate::peripherals::TIMG0::PTR
    }
    #[inline(always)]
    #[cfg(any(esp32c6, esp32h2))]
    fn configure_src_clk() {
        unsafe { &*crate::peripherals::PCR::PTR }
            .timergroup0_timer_clk_conf
            .modify(|_, w| unsafe { w.tg0_timer_clk_sel().bits(TIMG_DEFAULT_CLK_SRC) });
    }
    #[inline(always)]
    #[cfg(any(esp32c2, esp32c3, esp32s2, esp32s3))]
    fn configure_src_clk() {
        unsafe {
            (*Self::register_block())
                .t0config
                .modify(|_, w| w.use_xtal().clear_bit())
        };
    }
    #[inline(always)]
    #[cfg(esp32)]
    fn configure_src_clk() {
        // ESP32 has only APB clock source, do nothing
    }
    #[inline(always)]
    #[cfg(any(esp32c2, esp32c3))]
    fn configure_wdt_src_clk() {
        unsafe {
            (*Self::register_block())
                .wdtconfig0
                .modify(|_, w| w.wdt_use_xtal().clear_bit())
        };
    }
    #[inline(always)]
    #[cfg(any(esp32c6, esp32h2))]
    fn configure_wdt_src_clk() {
        unsafe { &*crate::peripherals::PCR::PTR }
            .timergroup0_wdt_clk_conf
            .modify(|_, w| unsafe { w.tg0_wdt_clk_sel().bits(1) });
    }
    #[inline(always)]
    #[cfg(any(esp32, esp32s2, esp32s3))]
    fn configure_wdt_src_clk() {
        // ESP32, ESP32-S2, and ESP32-S3 use only ABP, do nothing
    }
}

#[cfg(timg1)]
impl TimerGroupInstance for TIMG1 {
    #[inline(always)]
    fn register_block() -> *const RegisterBlock {
        crate::peripherals::TIMG1::PTR
    }
    #[inline(always)]
    #[cfg(any(esp32c6, esp32h2))]
    fn configure_src_clk() {
        unsafe { &*crate::peripherals::PCR::PTR }
            .timergroup1_timer_clk_conf
            .modify(|_, w| unsafe { w.tg1_timer_clk_sel().bits(TIMG_DEFAULT_CLK_SRC) });
    }
    #[inline(always)]
    #[cfg(any(esp32s2, esp32s3))]
    fn configure_src_clk() {
        unsafe {
            (*Self::register_block())
                .t1config
                .modify(|_, w| w.use_xtal().clear_bit())
        };
    }
    #[inline(always)]
    #[cfg(any(esp32, esp32c2, esp32c3))]
    fn configure_src_clk() {
        // ESP32 has only APB clock source, do nothing
        // ESP32-C2 and ESP32-C3 don't have t1config only t0config, do nothing
    }
    #[inline(always)]
    #[cfg(any(esp32c6, esp32h2))]
    fn configure_wdt_src_clk() {
        unsafe { &*crate::peripherals::PCR::PTR }
            .timergroup1_wdt_clk_conf
            .modify(|_, w| unsafe { w.tg1_wdt_clk_sel().bits(1) });
    }
    #[inline(always)]
    #[cfg(any(esp32, esp32s2, esp32s3, esp32c2, esp32c3))]
    fn configure_wdt_src_clk() {
        // ESP32-C2 and ESP32-C3 don't have t1config only t0config, do nothing
        // ESP32, ESP32-S2, and ESP32-S3 use only ABP, do nothing
    }
}

impl<'d, T> TimerGroup<'d, T>
where
    T: TimerGroupInstance,
{
    pub fn new(
        timer_group: impl Peripheral<P = T> + 'd,
        clocks: &Clocks,
        peripheral_clock_control: &mut PeripheralClockControl,
    ) -> Self {
        crate::into_ref!(timer_group);

        T::configure_src_clk();

        // ESP32-H2 is using PLL_48M_CLK source instead of APB_CLK
        let timer0 = Timer::new(
            Timer0 {
                phantom: PhantomData::default(),
            },
            #[cfg(not(esp32h2))]
            clocks.apb_clock,
            #[cfg(esp32h2)]
            clocks.pll_48m_clock,
            peripheral_clock_control,
        );

        #[cfg(not(any(esp32c2, esp32c3, esp32c6, esp32h2)))]
        let timer1 = Timer::new(
            Timer1 {
                phantom: PhantomData::default(),
            },
            clocks.apb_clock,
            peripheral_clock_control,
        );

        let wdt = Wdt::new(peripheral_clock_control);

        Self {
            _timer_group: timer_group,
            timer0,
            #[cfg(not(any(esp32c2, esp32c3, esp32c6, esp32h2)))]
            timer1,
            wdt,
        }
    }
}

/// General-purpose Timer driver
pub struct Timer<T> {
    timg: T,
    apb_clk_freq: HertzU32,
}

impl<T> Timer<T>
where
    T: Instance,
{
    /// Create a new timer instance
    pub fn new(
        timg: T,
        apb_clk_freq: HertzU32,
        peripheral_clock_control: &mut PeripheralClockControl,
    ) -> Self {
        // TODO: this currently assumes APB_CLK is being used, as we don't yet have a
        //       way to select the XTAL_CLK.

        timg.enable_peripheral(peripheral_clock_control);
        Self { timg, apb_clk_freq }
    }

    /// Return the raw interface to the underlying timer instance
    pub fn free(self) -> T {
        self.timg
    }
}

impl<T> Deref for Timer<T>
where
    T: Instance,
{
    type Target = T;

    fn deref(&self) -> &Self::Target {
        &self.timg
    }
}

impl<T> DerefMut for Timer<T>
where
    T: Instance,
{
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.timg
    }
}

/// Timer peripheral instance
pub trait Instance {
    fn reset_counter(&mut self);

    fn set_counter_active(&mut self, state: bool);

    fn is_counter_active(&self) -> bool;

    fn set_counter_decrementing(&mut self, decrementing: bool);

    fn set_auto_reload(&mut self, auto_reload: bool);

    fn set_alarm_active(&mut self, state: bool);

    fn is_alarm_active(&self) -> bool;

    fn load_alarm_value(&mut self, value: u64);

    fn listen(&mut self);

    fn unlisten(&mut self);

    fn clear_interrupt(&mut self);

    fn now(&self) -> u64;

    fn divider(&self) -> u32;

    fn set_divider(&mut self, divider: u16);

    fn is_interrupt_set(&self) -> bool;

    fn enable_peripheral(&self, peripheral_clock_control: &mut PeripheralClockControl);
}

pub struct Timer0<TG> {
    phantom: PhantomData<TG>,
}

impl<TG> Timer0<TG>
where
    TG: TimerGroupInstance,
{
    #[cfg(feature = "embassy-time-timg0")]
    pub(crate) unsafe fn steal() -> Self {
        Self {
            phantom: PhantomData,
        }
    }
}

/// Timer peripheral instance
impl<TG> Instance for Timer0<TG>
where
    TG: TimerGroupInstance,
{
    fn reset_counter(&mut self) {
        let reg_block = unsafe { &*TG::register_block() };

        reg_block.t0loadlo.write(|w| unsafe { w.load_lo().bits(0) });

        reg_block.t0loadhi.write(|w| unsafe { w.load_hi().bits(0) });

        reg_block.t0load.write(|w| unsafe { w.load().bits(1) });
    }

    fn set_counter_active(&mut self, state: bool) {
        let reg_block = unsafe { &*TG::register_block() };

        reg_block.t0config.modify(|_, w| w.en().bit(state));
    }

    fn is_counter_active(&self) -> bool {
        let reg_block = unsafe { &*TG::register_block() };

        reg_block.t0config.read().en().bit_is_set()
    }

    fn set_counter_decrementing(&mut self, decrementing: bool) {
        let reg_block = unsafe { &*TG::register_block() };

        reg_block
            .t0config
            .modify(|_, w| w.increase().bit(!decrementing));
    }

    fn set_auto_reload(&mut self, auto_reload: bool) {
        let reg_block = unsafe { &*TG::register_block() };

        reg_block
            .t0config
            .modify(|_, w| w.autoreload().bit(auto_reload));
    }

    fn set_alarm_active(&mut self, state: bool) {
        let reg_block = unsafe { &*TG::register_block() };

        reg_block.t0config.modify(|_, w| w.alarm_en().bit(state));
    }

    fn is_alarm_active(&self) -> bool {
        let reg_block = unsafe { &*TG::register_block() };

        reg_block.t0config.read().alarm_en().bit_is_set()
    }

    fn load_alarm_value(&mut self, value: u64) {
        let value = value & 0x3F_FFFF_FFFF_FFFF;
        let high = (value >> 32) as u32;
        let low = (value & 0xFFFF_FFFF) as u32;

        let reg_block = unsafe { &*TG::register_block() };

        reg_block
            .t0alarmlo
            .write(|w| unsafe { w.alarm_lo().bits(low) });

        reg_block
            .t0alarmhi
            .write(|w| unsafe { w.alarm_hi().bits(high) });
    }

    fn listen(&mut self) {
        let reg_block = unsafe { &*TG::register_block() };

        // always use level interrupt
        #[cfg(any(esp32, esp32s2))]
        reg_block.t0config.modify(|_, w| w.level_int_en().set_bit());

        reg_block
            .int_ena_timers
            .modify(|_, w| w.t0_int_ena().set_bit());
    }

    fn unlisten(&mut self) {
        let reg_block = unsafe { &*TG::register_block() };

        reg_block
            .int_ena_timers
            .modify(|_, w| w.t0_int_ena().clear_bit());
    }

    fn clear_interrupt(&mut self) {
        let reg_block = unsafe { &*TG::register_block() };

        reg_block.int_clr_timers.write(|w| w.t0_int_clr().set_bit());
    }

    fn now(&self) -> u64 {
        let reg_block = unsafe { &*TG::register_block() };

        reg_block.t0update.write(|w| unsafe { w.bits(0) });

        let value_lo = reg_block.t0lo.read().bits() as u64;
        let value_hi = (reg_block.t0hi.read().bits() as u64) << 32;

        (value_lo | value_hi) as u64
    }

    fn divider(&self) -> u32 {
        let reg_block = unsafe { &*TG::register_block() };

        // From the ESP32 TRM, "11.2.1 16­-bit Prescaler and Clock Selection":
        //
        // "The prescaler can divide the APB clock by a factor from 2 to 65536.
        // Specifically, when TIMGn_Tx_DIVIDER is either 1 or 2, the clock divisor is 2;
        // when TIMGn_Tx_DIVIDER is 0, the clock divisor is 65536. Any other value will
        // cause the clock to be divided by exactly that value."
        match reg_block.t0config.read().divider().bits() {
            0 => 65536,
            1 | 2 => 2,
            n => n as u32,
        }
    }

    fn is_interrupt_set(&self) -> bool {
        let reg_block = unsafe { &*TG::register_block() };

        reg_block.int_raw_timers.read().t0_int_raw().bit_is_set()
    }

    fn set_divider(&mut self, divider: u16) {
        let reg_block = unsafe { &*TG::register_block() };

        reg_block
            .t0config
            .modify(|_, w| unsafe { w.divider().bits(divider) })
    }

    fn enable_peripheral(&self, peripheral_clock_control: &mut PeripheralClockControl) {
        peripheral_clock_control.enable(crate::system::Peripheral::Timg0);
    }
}

#[cfg(not(any(esp32c2, esp32c3, esp32c6, esp32h2)))]
pub struct Timer1<TG> {
    phantom: PhantomData<TG>,
}

#[cfg(not(any(esp32c2, esp32c3, esp32c6, esp32h2)))]
impl<TG> Timer1<TG>
where
    TG: TimerGroupInstance,
{
    #[cfg(feature = "embassy-time-timg1")]
    pub(crate) unsafe fn steal() -> Self {
        Self {
            phantom: PhantomData,
        }
    }
}

/// Timer peripheral instance
#[cfg(not(any(esp32c2, esp32c3, esp32c6, esp32h2)))]
impl<TG> Instance for Timer1<TG>
where
    TG: TimerGroupInstance,
{
    fn reset_counter(&mut self) {
        let reg_block = unsafe { &*TG::register_block() };

        reg_block.t1loadlo.write(|w| unsafe { w.load_lo().bits(0) });

        reg_block.t1loadhi.write(|w| unsafe { w.load_hi().bits(0) });

        reg_block.t1load.write(|w| unsafe { w.load().bits(1) });
    }

    fn set_counter_active(&mut self, state: bool) {
        let reg_block = unsafe { &*TG::register_block() };

        reg_block.t1config.modify(|_, w| w.en().bit(state));
    }

    fn is_counter_active(&self) -> bool {
        let reg_block = unsafe { &*TG::register_block() };

        reg_block.t1config.read().en().bit_is_set()
    }

    fn set_counter_decrementing(&mut self, decrementing: bool) {
        let reg_block = unsafe { &*TG::register_block() };

        reg_block
            .t1config
            .modify(|_, w| w.increase().bit(!decrementing));
    }

    fn set_auto_reload(&mut self, auto_reload: bool) {
        let reg_block = unsafe { &*TG::register_block() };

        reg_block
            .t1config
            .modify(|_, w| w.autoreload().bit(auto_reload));
    }

    fn set_alarm_active(&mut self, state: bool) {
        let reg_block = unsafe { &*TG::register_block() };

        reg_block.t1config.modify(|_, w| w.alarm_en().bit(state));
    }

    fn is_alarm_active(&self) -> bool {
        let reg_block = unsafe { &*TG::register_block() };

        reg_block.t1config.read().alarm_en().bit_is_set()
    }

    fn load_alarm_value(&mut self, value: u64) {
        let value = value & 0x3F_FFFF_FFFF_FFFF;
        let high = (value >> 32) as u32;
        let low = (value & 0xFFFF_FFFF) as u32;

        let reg_block = unsafe { &*TG::register_block() };

        reg_block
            .t1alarmlo
            .write(|w| unsafe { w.alarm_lo().bits(low) });

        reg_block
            .t1alarmhi
            .write(|w| unsafe { w.alarm_hi().bits(high) });
    }

    fn listen(&mut self) {
        let reg_block = unsafe { &*TG::register_block() };

        // always use level interrupt
        #[cfg(any(esp32, esp32s2))]
        reg_block.t1config.modify(|_, w| w.level_int_en().set_bit());

        reg_block
            .int_ena_timers
            .modify(|_, w| w.t1_int_ena().set_bit());
    }

    fn unlisten(&mut self) {
        let reg_block = unsafe { &*TG::register_block() };

        reg_block
            .int_ena_timers
            .modify(|_, w| w.t1_int_ena().clear_bit());
    }

    fn clear_interrupt(&mut self) {
        let reg_block = unsafe { &*TG::register_block() };

        reg_block.int_clr_timers.write(|w| w.t1_int_clr().set_bit());
    }

    fn now(&self) -> u64 {
        let reg_block = unsafe { &*TG::register_block() };

        reg_block.t1update.write(|w| unsafe { w.bits(0) });

        let value_lo = reg_block.t1lo.read().bits() as u64;
        let value_hi = (reg_block.t1hi.read().bits() as u64) << 32;

        (value_lo | value_hi) as u64
    }

    fn divider(&self) -> u32 {
        let reg_block = unsafe { &*TG::register_block() };

        // From the ESP32 TRM, "11.2.1 16­-bit Prescaler and Clock Selection":
        //
        // "The prescaler can divide the APB clock by a factor from 2 to 65536.
        // Specifically, when TIMGn_Tx_DIVIDER is either 1 or 2, the clock divisor is 2;
        // when TIMGn_Tx_DIVIDER is 0, the clock divisor is 65536. Any other value will
        // cause the clock to be divided by exactly that value."
        match reg_block.t1config.read().divider().bits() {
            0 => 65536,
            1 | 2 => 2,
            n => n as u32,
        }
    }

    fn is_interrupt_set(&self) -> bool {
        let reg_block = unsafe { &*TG::register_block() };

        reg_block.int_raw_timers.read().t1_int_raw().bit_is_set()
    }

    fn set_divider(&mut self, divider: u16) {
        let reg_block = unsafe { &*TG::register_block() };

        reg_block
            .t1config
            .modify(|_, w| unsafe { w.divider().bits(divider) })
    }

    fn enable_peripheral(&self, peripheral_clock_control: &mut PeripheralClockControl) {
        peripheral_clock_control.enable(crate::system::Peripheral::Timg1);
    }
}

fn timeout_to_ticks<T, F>(timeout: T, clock: F, divider: u32) -> u64
where
    T: Into<MicrosDurationU64>,
    F: Into<HertzU32>,
{
    let timeout: MicrosDurationU64 = timeout.into();
    let micros = timeout.to_micros();

    let clock: HertzU32 = clock.into();

    // 1_000_000 is used to get rid of `float` calculations
    let period: u64 = 1_000_000 * 1_000_000 / (clock.to_Hz() as u64 / divider as u64);
    (1_000_000 * micros / period as u64) as u64
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

        if self.timg.is_interrupt_set() {
            self.timg.clear_interrupt();
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

/// Watchdog timer
pub struct Wdt<TG> {
    phantom: PhantomData<TG>,
}

/// Watchdog driver
impl<TG> Wdt<TG>
where
    TG: TimerGroupInstance,
{
    /// Create a new watchdog timer instance
    pub fn new(_peripheral_clock_control: &mut PeripheralClockControl) -> Self {
        #[cfg(lp_wdt)]
        _peripheral_clock_control.enable(crate::system::Peripheral::Wdt);
        TG::configure_wdt_src_clk();
        Self {
            phantom: PhantomData::default(),
        }
    }

    fn set_wdt_enabled(&mut self, enabled: bool) {
        let reg_block = unsafe { &*TG::register_block() };

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

    fn feed(&mut self) {
        let reg_block = unsafe { &*TG::register_block() };

        reg_block
            .wdtwprotect
            .write(|w| unsafe { w.wdt_wkey().bits(0x50D8_3AA1u32) });

        reg_block.wdtfeed.write(|w| unsafe { w.bits(1) });

        reg_block
            .wdtwprotect
            .write(|w| unsafe { w.wdt_wkey().bits(0u32) });
    }

    fn set_timeout(&mut self, timeout: MicrosDurationU64) {
        let timeout_raw = (timeout.to_nanos() * 10 / 125) as u32;

        let reg_block = unsafe { &*TG::register_block() };

        reg_block
            .wdtwprotect
            .write(|w| unsafe { w.wdt_wkey().bits(0x50D8_3AA1u32) });

        reg_block
            .wdtconfig1
            .write(|w| unsafe { w.wdt_clk_prescale().bits(1) });

        reg_block
            .wdtconfig2
            .write(|w| unsafe { w.wdt_stg0_hold().bits(timeout_raw) });

        #[cfg_attr(esp32, allow(unused_unsafe))]
        reg_block.wdtconfig0.write(|w| unsafe {
            w.wdt_en()
                .bit(true)
                .wdt_stg0()
                .bits(3)
                .wdt_cpu_reset_length()
                .bits(1)
                .wdt_sys_reset_length()
                .bits(1)
                .wdt_stg1()
                .bits(0)
                .wdt_stg2()
                .bits(0)
                .wdt_stg3()
                .bits(0)
        });

        #[cfg(any(esp32c2, esp32c3, esp32c6))]
        reg_block
            .wdtconfig0
            .modify(|_, w| w.wdt_conf_update_en().set_bit());

        reg_block
            .wdtwprotect
            .write(|w| unsafe { w.wdt_wkey().bits(0u32) });
    }
}

impl<TG> WatchdogDisable for Wdt<TG>
where
    TG: TimerGroupInstance,
{
    fn disable(&mut self) {
        self.set_wdt_enabled(false);
    }
}

impl<TG> WatchdogEnable for Wdt<TG>
where
    TG: TimerGroupInstance,
{
    type Time = MicrosDurationU64;

    fn start<T>(&mut self, period: T)
    where
        T: Into<Self::Time>,
    {
        self.set_timeout(period.into());
    }
}

impl<TG> Watchdog for Wdt<TG>
where
    TG: TimerGroupInstance,
{
    fn feed(&mut self) {
        self.feed();
    }
}
