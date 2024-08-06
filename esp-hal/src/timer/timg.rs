//! # Timer Group (TIMG)
//!
//! ## Overview
//! The Timer Group (TIMG) peripherals contain one or more general-purpose
//! timers, plus one or more watchdog timers.
//!
//! The general-purpose timers are based on a 16-bit pre-scaler and a 54-bit
//! auto-reload-capable up-down counter.
//!
//! ## Configuration
//! The timers have configurable alarms, which are triggered when the internal
//! counter of the timers reaches a specific target value. The timers are
//! clocked using the APB clock source.
//!
//! Typically, a general-purpose timer can be used in scenarios such as:
//!
//! - Generate period alarms; trigger events periodically
//! - Generate one-shot alarms; trigger events once
//! - Free-running; fetching a high-resolution timestamp on demand
//!
//!
//! ## Examples
//! ### General-purpose Timer
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::timer::timg::TimerGroup;
//! # use crate::esp_hal::prelude::_esp_hal_timer_Timer;
//! # use esp_hal::prelude::*;
//! let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks);
//! let timer0 = timg0.timer0;
//!
//! // Get the current timestamp, in microseconds:
//! let now = timer0.now();
//!
//! // Wait for timeout:
//! timer0.load_value(1.secs());
//! timer0.start();
//!
//! while !timer0.is_interrupt_set() {
//!     // Wait
//! }
//! # }
//! ```
//! 
//! #### Watchdog Timer
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::timer::timg::TimerGroup;
//! # use esp_hal::prelude::*;
//! let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks);
//! let mut wdt = timg0.wdt;
//!
//! wdt.set_timeout(5_000.millis());
//! wdt.enable();
//!
//! loop {
//!     wdt.feed();
//! }
//! # }
//! ```

use core::{
    marker::PhantomData,
    ops::{Deref, DerefMut},
};

use fugit::{HertzU32, Instant, MicrosDurationU64};

use super::Error;
#[cfg(timg1)]
use crate::peripherals::TIMG1;
#[cfg(any(esp32c6, esp32h2))]
use crate::soc::constants::TIMG_DEFAULT_CLK_SRC;
use crate::{
    clock::Clocks,
    interrupt::{self, InterruptHandler},
    peripheral::{Peripheral, PeripheralRef},
    peripherals::{timg0::RegisterBlock, Interrupt, TIMG0},
    private::Sealed,
    system::{Peripheral as PeripheralEnable, PeripheralClockControl},
    Async,
    Blocking,
    InterruptConfigurable,
    Mode,
};

/// A timer group consisting of up to 2 timers (chip dependent) and a watchdog
/// timer.
pub struct TimerGroup<'d, T, DM>
where
    T: TimerGroupInstance,
    DM: Mode,
{
    _timer_group: PeripheralRef<'d, T>,
    /// Timer 0
    pub timer0: Timer<Timer0<T>, DM>,
    /// Timer 1
    #[cfg(timg_timer1)]
    pub timer1: Timer<Timer1<T>, DM>,
    /// Watchdog timer
    pub wdt: Wdt<T, DM>,
}

#[doc(hidden)]
pub trait TimerGroupInstance {
    fn id() -> u8;
    fn register_block() -> *const RegisterBlock;
    fn configure_src_clk();
    fn configure_wdt_src_clk();
}

impl TimerGroupInstance for TIMG0 {
    fn id() -> u8 {
        0
    }

    #[inline(always)]
    fn register_block() -> *const RegisterBlock {
        TIMG0::PTR
    }

    #[inline(always)]
    #[cfg(any(esp32c6, esp32h2))]
    fn configure_src_clk() {
        unsafe { &*crate::peripherals::PCR::PTR }
            .timergroup0_timer_clk_conf()
            .modify(|_, w| unsafe { w.tg0_timer_clk_sel().bits(TIMG_DEFAULT_CLK_SRC) });
    }

    #[inline(always)]
    #[cfg(any(esp32c2, esp32c3, esp32s2, esp32s3))]
    fn configure_src_clk() {
        unsafe {
            (*Self::register_block())
                .t(0)
                .config()
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
                .wdtconfig0()
                .modify(|_, w| w.wdt_use_xtal().clear_bit())
        };
    }

    #[inline(always)]
    #[cfg(any(esp32c6, esp32h2))]
    fn configure_wdt_src_clk() {
        unsafe { &*crate::peripherals::PCR::PTR }
            .timergroup0_wdt_clk_conf()
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
    fn id() -> u8 {
        1
    }

    #[inline(always)]
    fn register_block() -> *const RegisterBlock {
        TIMG1::PTR
    }

    #[inline(always)]
    #[cfg(any(esp32c6, esp32h2))]
    fn configure_src_clk() {
        unsafe { &*crate::peripherals::PCR::PTR }
            .timergroup1_timer_clk_conf()
            .modify(|_, w| unsafe { w.tg1_timer_clk_sel().bits(TIMG_DEFAULT_CLK_SRC) });
    }

    #[inline(always)]
    #[cfg(any(esp32s2, esp32s3))]
    fn configure_src_clk() {
        unsafe {
            (*Self::register_block())
                .t(1)
                .config()
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
            .timergroup1_wdt_clk_conf()
            .modify(|_, w| unsafe { w.tg1_wdt_clk_sel().bits(1) });
    }

    #[inline(always)]
    #[cfg(any(esp32, esp32s2, esp32s3, esp32c2, esp32c3))]
    fn configure_wdt_src_clk() {
        // ESP32-C2 and ESP32-C3 don't have t1config only t0config, do nothing
        // ESP32, ESP32-S2, and ESP32-S3 use only ABP, do nothing
    }
}

impl<'d, T> TimerGroup<'d, T, Blocking>
where
    T: TimerGroupInstance,
{
    /// Construct a new instance of [`TimerGroup`] in blocking mode
    pub fn new(_timer_group: impl Peripheral<P = T> + 'd, clocks: &Clocks<'d>) -> Self {
        crate::into_ref!(_timer_group);

        T::configure_src_clk();

        // ESP32-H2 is using PLL_48M_CLK source instead of APB_CLK
        let timer0 = Timer::new(
            Timer0 {
                phantom: PhantomData,
            },
            #[cfg(not(esp32h2))]
            clocks.apb_clock,
            #[cfg(esp32h2)]
            clocks.pll_48m_clock,
        );

        #[cfg(timg_timer1)]
        let timer1 = Timer::new(
            Timer1 {
                phantom: PhantomData,
            },
            clocks.apb_clock,
        );

        Self {
            _timer_group,
            timer0,
            #[cfg(timg_timer1)]
            timer1,
            wdt: Wdt::new(),
        }
    }
}

impl<'d, T> TimerGroup<'d, T, Async>
where
    T: TimerGroupInstance,
{
    /// Construct a new instance of [`TimerGroup`] in asynchronous mode
    pub fn new_async(_timer_group: impl Peripheral<P = T> + 'd, clocks: &Clocks<'d>) -> Self {
        crate::into_ref!(_timer_group);

        T::configure_src_clk();

        // ESP32-H2 is using PLL_48M_CLK source instead of APB_CLK
        let timer0 = Timer::new(
            Timer0 {
                phantom: PhantomData,
            },
            #[cfg(not(esp32h2))]
            clocks.apb_clock,
            #[cfg(esp32h2)]
            clocks.pll_48m_clock,
        );

        #[cfg(timg_timer1)]
        let timer1 = Timer::new(
            Timer1 {
                phantom: PhantomData,
            },
            clocks.apb_clock,
        );

        Self {
            _timer_group,
            timer0,
            #[cfg(timg_timer1)]
            timer1,
            wdt: Wdt::new(),
        }
    }
}

/// General-purpose timer.
pub struct Timer<T, DM>
where
    DM: Mode,
{
    timg: T,
    apb_clk_freq: HertzU32,
    phantom: PhantomData<DM>,
}

impl<T, DM> Timer<T, DM>
where
    T: Instance,
    DM: Mode,
{
    /// Construct a new instance of [`Timer`]
    pub fn new(timg: T, apb_clk_freq: HertzU32) -> Self {
        timg.enable_peripheral();
        timg.set_counter_active(true);

        Self {
            timg,
            apb_clk_freq,
            phantom: PhantomData,
        }
    }

    /// Check if the timer has elapsed
    pub fn has_elapsed(&mut self) -> bool {
        if !self.timg.is_counter_active() {
            panic!("Called wait on an inactive timer!")
        }

        if self.timg.is_interrupt_set() {
            self.timg.clear_interrupt();
            self.timg.set_alarm_active(true);

            true
        } else {
            false
        }
    }

    /// Block until the timer has elasped.
    pub fn wait(&mut self) {
        while !self.has_elapsed() {}
    }
}

impl<T, DM> Deref for Timer<T, DM>
where
    T: Instance,
    DM: Mode,
{
    type Target = T;

    fn deref(&self) -> &Self::Target {
        &self.timg
    }
}

impl<T, DM> DerefMut for Timer<T, DM>
where
    T: Instance,
    DM: Mode,
{
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.timg
    }
}

impl<T, DM> Sealed for Timer<T, DM>
where
    T: Instance,
    DM: Mode,
{
}

impl<T, DM> super::Timer for Timer<T, DM>
where
    T: Instance,
    DM: Mode,
{
    fn start(&self) {
        self.timg.set_counter_active(false);
        self.timg.set_alarm_active(false);

        self.timg.reset_counter();
        self.timg.set_counter_decrementing(false);

        self.timg.set_counter_active(true);
        self.timg.set_alarm_active(true);
    }

    fn stop(&self) {
        self.timg.set_counter_active(false);
    }

    fn reset(&self) {
        let t = self.register_block().t(self.timer_number().into());

        t.loadlo().write(|w| unsafe { w.load_lo().bits(0) });
        t.loadhi().write(|w| unsafe { w.load_hi().bits(0) });

        t.load().write(|w| unsafe { w.load().bits(1) });
    }

    fn is_running(&self) -> bool {
        self.register_block()
            .t(self.timer_number().into())
            .config()
            .read()
            .en()
            .bit_is_set()
    }

    fn now(&self) -> Instant<u64, 1, 1_000_000> {
        let t = self.register_block().t(self.timer_number().into());

        t.update().write(|w| w.update().set_bit());
        while t.update().read().update().bit_is_set() {
            // Wait for the update to complete
        }

        let value_lo = t.lo().read().bits() as u64;
        let value_hi = t.hi().read().bits() as u64;

        let ticks = (value_hi << 32) | value_lo;
        let micros = ticks_to_timeout(ticks, self.apb_clk_freq, self.timg.divider());

        Instant::<u64, 1, 1_000_000>::from_ticks(micros)
    }

    fn load_value(&self, value: MicrosDurationU64) -> Result<(), Error> {
        let ticks = timeout_to_ticks(value, self.apb_clk_freq, self.timg.divider());

        // The counter is 54-bits wide, so we must ensure that the provided
        // value is not too wide:
        if (ticks & !0x3F_FFFF_FFFF_FFFF) != 0 {
            return Err(Error::InvalidTimeout);
        }

        let high = (ticks >> 32) as u32;
        let low = (ticks & 0xFFFF_FFFF) as u32;

        let t = self.register_block().t(self.timer_number().into());

        t.alarmlo().write(|w| unsafe { w.alarm_lo().bits(low) });
        t.alarmhi().write(|w| unsafe { w.alarm_hi().bits(high) });

        Ok(())
    }

    fn enable_auto_reload(&self, auto_reload: bool) {
        self.register_block()
            .t(self.timer_number().into())
            .config()
            .modify(|_, w| w.autoreload().bit(auto_reload));
    }

    fn enable_interrupt(&self, state: bool) {
        // always use level interrupt
        #[cfg(any(esp32, esp32s2))]
        self.register_block()
            .t(self.timer_number().into())
            .config()
            .modify(|_, w| w.level_int_en().set_bit());

        self.register_block()
            .int_ena_timers()
            .modify(|_, w| w.t(self.timer_number()).bit(state));
    }

    fn clear_interrupt(&self) {
        self.register_block()
            .int_clr_timers()
            .write(|w| w.t(self.timer_number()).clear_bit_by_one());
    }

    fn set_interrupt_handler(&self, handler: InterruptHandler) {
        let interrupt = match (self.timer_group(), self.timer_number()) {
            (0, 0) => Interrupt::TG0_T0_LEVEL,
            #[cfg(timg_timer1)]
            (0, 1) => Interrupt::TG0_T1_LEVEL,
            #[cfg(timg1)]
            (1, 0) => Interrupt::TG1_T0_LEVEL,
            #[cfg(all(timg_timer1, timg1))]
            (1, 1) => Interrupt::TG1_T1_LEVEL,
            _ => unreachable!(),
        };

        unsafe {
            interrupt::bind_interrupt(interrupt, handler.handler());
        }
        interrupt::enable(interrupt, handler.priority()).unwrap();
    }

    fn is_interrupt_set(&self) -> bool {
        self.register_block()
            .int_raw_timers()
            .read()
            .t(self.timer_number())
            .bit_is_set()
    }

    fn set_alarm_active(&self, state: bool) {
        self.register_block()
            .t(self.timer_number().into())
            .config()
            .modify(|_, w| w.alarm_en().bit(state));
    }
}

impl<T> InterruptConfigurable for Timer<T, Blocking>
where
    T: Instance,
{
    fn set_interrupt_handler(&mut self, handler: interrupt::InterruptHandler) {
        <Self as super::Timer>::set_interrupt_handler(self, handler);
    }
}

impl<T, DM> Peripheral for Timer<T, DM>
where
    T: Instance,
    DM: Mode,
{
    type P = Self;

    #[inline]
    unsafe fn clone_unchecked(&mut self) -> Self::P {
        core::ptr::read(self as *const _)
    }
}

#[doc(hidden)]
pub trait Instance: Sealed + Enable {
    fn register_block(&self) -> &RegisterBlock;

    fn timer_group(&self) -> u8;

    fn timer_number(&self) -> u8;

    fn reset_counter(&self);

    fn set_counter_active(&self, state: bool);

    fn is_counter_active(&self) -> bool;

    fn set_counter_decrementing(&self, decrementing: bool);

    fn set_auto_reload(&self, auto_reload: bool);

    fn set_alarm_active(&self, state: bool);

    fn is_alarm_active(&self) -> bool;

    fn load_alarm_value(&self, value: u64);

    fn listen(&self);

    fn unlisten(&self);

    fn clear_interrupt(&self);

    fn now(&self) -> u64;

    fn divider(&self) -> u32;

    fn set_divider(&self, divider: u16);

    fn is_interrupt_set(&self) -> bool;
}

#[doc(hidden)]
pub trait Enable: Sealed {
    fn enable_peripheral(&self);
}

/// A timer within a Timer Group.
pub struct TimerX<TG, const T: u8 = 0> {
    phantom: PhantomData<TG>,
}

/// Timer 0 in the Timer Group.
pub type Timer0<TG> = TimerX<TG, 0>;

/// Timer 1 in the Timer Group.
#[cfg(timg_timer1)]
pub type Timer1<TG> = TimerX<TG, 1>;

impl<TG, const T: u8> Sealed for TimerX<TG, T> {}

impl<TG, const T: u8> TimerX<TG, T>
where
    TG: TimerGroupInstance,
{
    /// Unsafely create an instance of this peripheral out of thin air.
    ///
    /// # Safety
    ///
    /// You must ensure that you're only using one instance of this type at a
    /// time.
    #[allow(unused)]
    pub unsafe fn steal() -> Self {
        Self {
            phantom: PhantomData,
        }
    }

    unsafe fn t() -> &'static crate::peripherals::timg0::T {
        (*TG::register_block()).t(T as usize)
    }
}

/// Timer peripheral instance
impl<TG, const T: u8> Instance for TimerX<TG, T>
where
    TG: TimerGroupInstance,
    Self: Enable,
{
    fn register_block(&self) -> &RegisterBlock {
        unsafe { &*TG::register_block() }
    }

    fn timer_group(&self) -> u8 {
        TG::id()
    }

    fn timer_number(&self) -> u8 {
        T
    }

    fn reset_counter(&self) {
        let t = unsafe { Self::t() };

        t.loadlo().write(|w| unsafe { w.load_lo().bits(0) });
        t.loadhi().write(|w| unsafe { w.load_hi().bits(0) });

        t.load().write(|w| unsafe { w.load().bits(1) });
    }

    fn set_counter_active(&self, state: bool) {
        unsafe { Self::t() }
            .config()
            .modify(|_, w| w.en().bit(state));
    }

    fn is_counter_active(&self) -> bool {
        unsafe { Self::t() }.config().read().en().bit_is_set()
    }

    fn set_counter_decrementing(&self, decrementing: bool) {
        unsafe { Self::t() }
            .config()
            .modify(|_, w| w.increase().bit(!decrementing));
    }

    fn set_auto_reload(&self, auto_reload: bool) {
        unsafe { Self::t() }
            .config()
            .modify(|_, w| w.autoreload().bit(auto_reload));
    }

    fn set_alarm_active(&self, state: bool) {
        unsafe { Self::t() }
            .config()
            .modify(|_, w| w.alarm_en().bit(state));
    }

    fn is_alarm_active(&self) -> bool {
        unsafe { Self::t() }.config().read().alarm_en().bit_is_set()
    }

    fn load_alarm_value(&self, value: u64) {
        let value = value & 0x3F_FFFF_FFFF_FFFF;
        let high = (value >> 32) as u32;
        let low = (value & 0xFFFF_FFFF) as u32;

        let t = unsafe { Self::t() };

        t.alarmlo().write(|w| unsafe { w.alarm_lo().bits(low) });

        t.alarmhi().write(|w| unsafe { w.alarm_hi().bits(high) });
    }

    fn listen(&self) {
        // always use level interrupt
        #[cfg(any(esp32, esp32s2))]
        unsafe { Self::t() }
            .config()
            .modify(|_, w| w.level_int_en().set_bit());

        self.register_block()
            .int_ena_timers()
            .modify(|_, w| w.t(T).set_bit());
    }

    fn unlisten(&self) {
        self.register_block()
            .int_ena_timers()
            .modify(|_, w| w.t(T).clear_bit());
    }

    fn clear_interrupt(&self) {
        self.register_block()
            .int_clr_timers()
            .write(|w| w.t(T).clear_bit_by_one());
    }

    fn now(&self) -> u64 {
        let t = unsafe { Self::t() };

        t.update().write(|w| w.update().set_bit());
        while t.update().read().update().bit_is_set() {}

        let value_lo = t.lo().read().bits() as u64;
        let value_hi = (t.hi().read().bits() as u64) << 32;

        value_lo | value_hi
    }

    fn divider(&self) -> u32 {
        let t = unsafe { Self::t() };

        // From the ESP32 TRM, "11.2.1 16­-bit Prescaler and Clock Selection":
        //
        // "The prescaler can divide the APB clock by a factor from 2 to 65536.
        // Specifically, when TIMGn_Tx_DIVIDER is either 1 or 2, the clock divisor is 2;
        // when TIMGn_Tx_DIVIDER is 0, the clock divisor is 65536. Any other value will
        // cause the clock to be divided by exactly that value."
        match t.config().read().divider().bits() {
            0 => 65536,
            1 | 2 => 2,
            n => n as u32,
        }
    }

    fn is_interrupt_set(&self) -> bool {
        self.register_block()
            .int_raw_timers()
            .read()
            .t(T)
            .bit_is_set()
    }

    fn set_divider(&self, divider: u16) {
        unsafe { Self::t() }
            .config()
            .modify(|_, w| unsafe { w.divider().bits(divider) })
    }
}

impl<TG> Enable for Timer0<TG>
where
    TG: TimerGroupInstance,
{
    fn enable_peripheral(&self) {
        PeripheralClockControl::enable(PeripheralEnable::Timg0);
    }
}

#[cfg(timg_timer1)]
impl<TG> Enable for Timer1<TG>
where
    TG: TimerGroupInstance,
{
    fn enable_peripheral(&self) {
        PeripheralClockControl::enable(PeripheralEnable::Timg1);
    }
}

fn ticks_to_timeout<F>(ticks: u64, clock: F, divider: u32) -> u64
where
    F: Into<HertzU32>,
{
    let clock: HertzU32 = clock.into();

    // 1_000_000 is used to get rid of `float` calculations
    let period: u64 = 1_000_000 * 1_000_000 / (clock.to_Hz() as u64 / divider as u64);

    ticks * period / 1_000_000
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

#[cfg(feature = "embedded-hal-02")]
impl<T, DM> embedded_hal_02::timer::CountDown for Timer<T, DM>
where
    T: Instance + super::Timer,
    DM: Mode,
{
    type Time = MicrosDurationU64;

    fn start<Time>(&mut self, timeout: Time)
    where
        Time: Into<Self::Time>,
    {
        self.timg.load_value(timeout.into()).unwrap();
        self.timg.start();
    }

    fn wait(&mut self) -> nb::Result<(), void::Void> {
        if self.has_elapsed() {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

#[cfg(feature = "embedded-hal-02")]
impl<T, DM> embedded_hal_02::timer::Cancel for Timer<T, DM>
where
    T: Instance + super::Timer,
    DM: Mode,
{
    type Error = super::Error;

    fn cancel(&mut self) -> Result<(), super::Error> {
        if !self.timg.is_counter_active() {
            return Err(super::Error::TimerInactive);
        } else if !self.timg.is_alarm_active() {
            return Err(super::Error::AlarmInactive);
        }

        self.timg.set_counter_active(false);

        Ok(())
    }
}

#[cfg(feature = "embedded-hal-02")]
impl<T, DM> embedded_hal_02::timer::Periodic for Timer<T, DM>
where
    T: Instance + super::Timer,
    DM: Mode,
{
}

/// Watchdog timer
pub struct Wdt<TG, DM> {
    phantom: PhantomData<(TG, DM)>,
}

/// Watchdog driver
impl<TG, DM> Wdt<TG, DM>
where
    TG: TimerGroupInstance,
    DM: Mode,
{
    /// Construct a new instance of [`Wdt`]
    pub fn new() -> Self {
        #[cfg(lp_wdt)]
        PeripheralClockControl::enable(PeripheralEnable::Wdt);

        TG::configure_wdt_src_clk();

        Self {
            phantom: PhantomData,
        }
    }

    /// Enable the watchdog timer instance
    pub fn enable(&mut self) {
        // SAFETY: The `TG` instance being modified is owned by `self`, which is behind
        //         a mutable reference.
        unsafe { Self::set_wdt_enabled(true) };
    }

    /// Disable the watchdog timer instance
    pub fn disable(&mut self) {
        // SAFETY: The `TG` instance being modified is owned by `self`, which is behind
        //         a mutable reference.
        unsafe { Self::set_wdt_enabled(false) };
    }

    /// Forcibly enable or disable the watchdog timer
    ///
    /// # Safety
    ///
    /// This bypasses the usual ownership rules for the peripheral, so users
    /// must take care to ensure that no driver instance is active for the
    /// timer.
    pub unsafe fn set_wdt_enabled(enabled: bool) {
        let reg_block = unsafe { &*TG::register_block() };

        reg_block
            .wdtwprotect()
            .write(|w| unsafe { w.wdt_wkey().bits(0x50D8_3AA1u32) });

        if !enabled {
            reg_block.wdtconfig0().write(|w| unsafe { w.bits(0) });
        } else {
            reg_block.wdtconfig0().write(|w| w.wdt_en().bit(true));
        }

        reg_block
            .wdtwprotect()
            .write(|w| unsafe { w.wdt_wkey().bits(0u32) });
    }

    /// Feed the watchdog timer
    pub fn feed(&mut self) {
        let reg_block = unsafe { &*TG::register_block() };

        reg_block
            .wdtwprotect()
            .write(|w| unsafe { w.wdt_wkey().bits(0x50D8_3AA1u32) });

        reg_block.wdtfeed().write(|w| unsafe { w.bits(1) });

        reg_block
            .wdtwprotect()
            .write(|w| unsafe { w.wdt_wkey().bits(0u32) });
    }

    /// Set the timeout, in microseconds, of the watchdog timer
    pub fn set_timeout(&mut self, timeout: MicrosDurationU64) {
        let timeout_raw = (timeout.to_nanos() * 10 / 125) as u32;

        let reg_block = unsafe { &*TG::register_block() };

        reg_block
            .wdtwprotect()
            .write(|w| unsafe { w.wdt_wkey().bits(0x50D8_3AA1u32) });

        reg_block
            .wdtconfig1()
            .write(|w| unsafe { w.wdt_clk_prescale().bits(1) });

        reg_block
            .wdtconfig2()
            .write(|w| unsafe { w.wdt_stg0_hold().bits(timeout_raw) });

        #[cfg_attr(esp32, allow(unused_unsafe))]
        reg_block.wdtconfig0().write(|w| unsafe {
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
            .wdtconfig0()
            .modify(|_, w| w.wdt_conf_update_en().set_bit());

        reg_block
            .wdtwprotect()
            .write(|w| unsafe { w.wdt_wkey().bits(0u32) });
    }
}

impl<TG, DM> crate::private::Sealed for Wdt<TG, DM>
where
    TG: TimerGroupInstance,
    DM: Mode,
{
}

impl<TG> InterruptConfigurable for Wdt<TG, Blocking>
where
    TG: TimerGroupInstance,
{
    fn set_interrupt_handler(&mut self, handler: interrupt::InterruptHandler) {
        let interrupt = match TG::id() {
            0 => Interrupt::TG0_WDT_LEVEL,
            #[cfg(timg1)]
            1 => Interrupt::TG1_WDT_LEVEL,
            _ => unreachable!(),
        };
        unsafe {
            interrupt::bind_interrupt(interrupt, handler.handler());
            interrupt::enable(interrupt, handler.priority()).unwrap();
        }
    }
}

impl<TG, DM> Default for Wdt<TG, DM>
where
    TG: TimerGroupInstance,
    DM: Mode,
{
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(feature = "embedded-hal-02")]
impl<TG, DM> embedded_hal_02::watchdog::WatchdogDisable for Wdt<TG, DM>
where
    TG: TimerGroupInstance,
    DM: Mode,
{
    fn disable(&mut self) {
        self.disable();
    }
}

#[cfg(feature = "embedded-hal-02")]
impl<TG, DM> embedded_hal_02::watchdog::WatchdogEnable for Wdt<TG, DM>
where
    TG: TimerGroupInstance,
    DM: Mode,
{
    type Time = MicrosDurationU64;

    fn start<T>(&mut self, period: T)
    where
        T: Into<Self::Time>,
    {
        self.enable();
        self.set_timeout(period.into());
    }
}

#[cfg(feature = "embedded-hal-02")]
impl<TG, DM> embedded_hal_02::watchdog::Watchdog for Wdt<TG, DM>
where
    TG: TimerGroupInstance,
    DM: Mode,
{
    fn feed(&mut self) {
        self.feed();
    }
}

/// Event Task Matrix
#[cfg(soc_etm)]
pub mod etm {
    use super::*;
    use crate::etm::{EtmEvent, EtmTask};

    /// Event Task Matrix event for a timer.
    pub struct TimerEtmEvent {
        id: u8,
    }

    /// Event Task Matrix task for a timer.
    pub struct TimerEtmTask {
        id: u8,
    }

    impl EtmEvent for TimerEtmEvent {
        fn id(&self) -> u8 {
            self.id
        }
    }

    impl Sealed for TimerEtmEvent {}

    impl EtmTask for TimerEtmTask {
        fn id(&self) -> u8 {
            self.id
        }
    }

    impl Sealed for TimerEtmTask {}

    /// General purpose timer ETM events.
    pub trait TimerEtmEvents<TG> {
        /// ETM event triggered on alarm
        fn on_alarm(&self) -> TimerEtmEvent;
    }

    /// General purpose timer ETM tasks
    pub trait TimerEtmTasks<TG> {
        /// ETM task to start the counter
        fn cnt_start(&self) -> TimerEtmTask;

        /// ETM task to start the alarm
        fn cnt_stop(&self) -> TimerEtmTask;

        /// ETM task to stop the counter
        fn cnt_reload(&self) -> TimerEtmTask;

        /// ETM task to reload the counter
        fn cnt_cap(&self) -> TimerEtmTask;

        /// ETM task to load the counter with the value stored when the last
        /// `now()` was called
        fn alarm_start(&self) -> TimerEtmTask;
    }

    impl<TG> TimerEtmEvents<TG> for Timer0<TG>
    where
        TG: TimerGroupInstance,
    {
        fn on_alarm(&self) -> TimerEtmEvent {
            TimerEtmEvent { id: 48 + TG::id() }
        }
    }

    impl<TG> TimerEtmTasks<TG> for Timer0<TG>
    where
        TG: TimerGroupInstance,
    {
        fn cnt_start(&self) -> TimerEtmTask {
            TimerEtmTask { id: 88 + TG::id() }
        }

        fn alarm_start(&self) -> TimerEtmTask {
            TimerEtmTask { id: 90 + TG::id() }
        }

        fn cnt_stop(&self) -> TimerEtmTask {
            TimerEtmTask { id: 92 + TG::id() }
        }

        fn cnt_reload(&self) -> TimerEtmTask {
            TimerEtmTask { id: 94 + TG::id() }
        }

        fn cnt_cap(&self) -> TimerEtmTask {
            TimerEtmTask { id: 96 + TG::id() }
        }
    }
}
