//! # Timer Group (TIMG)
//!
//! ## Overview
//!
//! The Timer Group (TIMG) peripherals contain one or more general-purpose
//! timers, plus one or more watchdog timers.
//!
//! The general-purpose timers are based on a 16-bit pre-scaler and a 54-bit
//! auto-reload-capable up-down counter.
//!
//! ## Configuration
//!
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
//! ## Examples
//!
//! ### General-purpose Timer
//!
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::timer::timg::TimerGroup;
//!
//! let timg0 = TimerGroup::new(peripherals.TIMG0);
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
//!
//! timer0.clear_interrupt();
//! # }
//! ```
//! 
//! ### Watchdog Timer
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::timer::timg::TimerGroup;
//! # use esp_hal::timer::timg::MwdtStage;
//!
//! let timg0 = TimerGroup::new(peripherals.TIMG0);
//! let mut wdt = timg0.wdt;
//!
//! wdt.set_timeout(MwdtStage::Stage0, 5_000.millis());
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
#[cfg(any(esp32c6, esp32h2))]
use crate::soc::constants::TIMG_DEFAULT_CLK_SRC;
use crate::{
    clock::Clocks,
    interrupt::{self, InterruptHandler},
    peripheral::{Peripheral, PeripheralRef},
    peripherals::{timg0::RegisterBlock, Interrupt, TIMG0},
    private::Sealed,
    sync::{lock, Lock},
    system::PeripheralClockControl,
    Async,
    Blocking,
    InterruptConfigurable,
    Mode,
};

const NUM_TIMG: usize = 1 + cfg!(timg1) as usize;

static INT_ENA_LOCK: [Lock; NUM_TIMG] = [const { Lock::new() }; NUM_TIMG];

/// A timer group consisting of
#[cfg_attr(not(timg_timer1), doc = "a general purpose timer")]
#[cfg_attr(timg_timer1, doc = "2 timers")]
/// and a watchdog timer.
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
    pub wdt: Wdt<T>,
}

#[doc(hidden)]
pub trait TimerGroupInstance {
    fn id() -> u8;
    fn register_block() -> *const RegisterBlock;
    fn configure_src_clk();
    fn enable_peripheral();
    fn reset_peripheral();
    fn configure_wdt_src_clk();
    fn wdt_interrupt() -> Interrupt;
}

impl TimerGroupInstance for TIMG0 {
    fn id() -> u8 {
        0
    }

    #[inline(always)]
    fn register_block() -> *const RegisterBlock {
        Self::PTR
    }

    fn configure_src_clk() {
        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
                // ESP32 has only APB clock source, do nothing
            } else if #[cfg(any(esp32c2, esp32c3, esp32s2, esp32s3))] {
                unsafe {
                    (*Self::register_block())
                        .t(0)
                        .config()
                        .modify(|_, w| w.use_xtal().clear_bit());
                }
            } else if #[cfg(any(esp32c6, esp32h2))] {
                unsafe { &*crate::peripherals::PCR::PTR }
                    .timergroup0_timer_clk_conf()
                    .modify(|_, w| unsafe { w.tg0_timer_clk_sel().bits(TIMG_DEFAULT_CLK_SRC) });
            }
        }
    }

    fn enable_peripheral() {
        PeripheralClockControl::enable(crate::system::Peripheral::Timg0)
    }

    fn reset_peripheral() {
        // FIXME: for TIMG0 do nothing for now because the reset breaks
        // `time::now`
    }

    fn configure_wdt_src_clk() {
        cfg_if::cfg_if! {
            if #[cfg(any(esp32, esp32s2, esp32s3))] {
                // ESP32, ESP32-S2, and ESP32-S3 use only ABP, do nothing
            } else if #[cfg(any(esp32c2, esp32c3))] {
                unsafe {
                    (*Self::register_block())
                        .wdtconfig0()
                        .modify(|_, w| w.wdt_use_xtal().clear_bit());
                }
            } else if #[cfg(any(esp32c6, esp32h2))] {
                unsafe { &*crate::peripherals::PCR::PTR }
                    .timergroup0_wdt_clk_conf()
                    .modify(|_, w| unsafe { w.tg0_wdt_clk_sel().bits(1) });
            }
        }
    }

    fn wdt_interrupt() -> Interrupt {
        Interrupt::TG0_WDT_LEVEL
    }
}

#[cfg(timg1)]
impl TimerGroupInstance for crate::peripherals::TIMG1 {
    fn id() -> u8 {
        1
    }

    #[inline(always)]
    fn register_block() -> *const RegisterBlock {
        Self::PTR
    }

    fn configure_src_clk() {
        cfg_if::cfg_if! {
            if #[cfg(any(esp32, esp32c2, esp32c3))] {
                // ESP32 has only APB clock source, do nothing
                // ESP32-C2 and ESP32-C3 don't have t1config only t0config, do nothing
            } else if #[cfg(any(esp32c6, esp32h2))] {
                unsafe { &*crate::peripherals::PCR::PTR }
                    .timergroup1_timer_clk_conf()
                    .modify(|_, w| unsafe { w.tg1_timer_clk_sel().bits(TIMG_DEFAULT_CLK_SRC) });
            } else if #[cfg(any(esp32s2, esp32s3))] {
                unsafe {
                    (*Self::register_block())
                        .t(1)
                        .config()
                        .modify(|_, w| w.use_xtal().clear_bit());
                }
            }
        }
    }

    fn enable_peripheral() {
        PeripheralClockControl::enable(crate::system::Peripheral::Timg1)
    }

    fn reset_peripheral() {
        PeripheralClockControl::reset(crate::system::Peripheral::Timg1)
    }

    fn configure_wdt_src_clk() {
        cfg_if::cfg_if! {
            if #[cfg(any(esp32, esp32s2, esp32s3, esp32c2, esp32c3))] {
                // ESP32-C2 and ESP32-C3 don't have t1config only t0config, do nothing
                // ESP32, ESP32-S2, and ESP32-S3 use only ABP, do nothing
            } else if #[cfg(any(esp32c6, esp32h2))] {
                unsafe { &*crate::peripherals::PCR::PTR }
                    .timergroup1_wdt_clk_conf()
                    .modify(|_, w| unsafe { w.tg1_wdt_clk_sel().bits(1) });
            }
        }
    }

    fn wdt_interrupt() -> Interrupt {
        Interrupt::TG1_WDT_LEVEL
    }
}

impl<'d, T, DM> TimerGroup<'d, T, DM>
where
    T: TimerGroupInstance,
    DM: Mode,
{
    /// Construct a new instance of [`TimerGroup`] in blocking mode
    pub fn new_inner(_timer_group: impl Peripheral<P = T> + 'd) -> Self {
        crate::into_ref!(_timer_group);

        T::reset_peripheral();
        T::enable_peripheral();

        T::configure_src_clk();

        let clocks = Clocks::get();
        cfg_if::cfg_if! {
            if #[cfg(esp32h2)] {
                // ESP32-H2 is using PLL_48M_CLK source instead of APB_CLK
                let apb_clk_freq = clocks.pll_48m_clock;
            } else {
                let apb_clk_freq = clocks.apb_clock;
            }
        }

        let timer0 = Timer::new(
            Timer0 {
                phantom: PhantomData,
            },
            apb_clk_freq,
        );

        #[cfg(timg_timer1)]
        let timer1 = Timer::new(
            Timer1 {
                phantom: PhantomData,
            },
            apb_clk_freq,
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

impl<'d, T> TimerGroup<'d, T, Blocking>
where
    T: TimerGroupInstance,
{
    /// Construct a new instance of [`TimerGroup`] in blocking mode
    pub fn new(_timer_group: impl Peripheral<P = T> + 'd) -> Self {
        Self::new_inner(_timer_group)
    }
}

impl<'d, T> TimerGroup<'d, T, Async>
where
    T: TimerGroupInstance,
{
    /// Construct a new instance of [`TimerGroup`] in asynchronous mode
    pub fn new_async(_timer_group: impl Peripheral<P = T> + 'd) -> Self {
        match T::id() {
            0 => {
                use crate::timer::timg::asynch::timg0_timer0_handler;
                unsafe {
                    interrupt::bind_interrupt(
                        Interrupt::TG0_T0_LEVEL,
                        timg0_timer0_handler.handler(),
                    );
                    interrupt::enable(Interrupt::TG0_T0_LEVEL, timg0_timer0_handler.priority())
                        .unwrap();

                    #[cfg(timg_timer1)]
                    {
                        use crate::timer::timg::asynch::timg0_timer1_handler;

                        interrupt::bind_interrupt(
                            Interrupt::TG0_T1_LEVEL,
                            timg0_timer1_handler.handler(),
                        );
                        interrupt::enable(Interrupt::TG0_T1_LEVEL, timg0_timer1_handler.priority())
                            .unwrap();
                    }
                }
            }
            #[cfg(timg1)]
            1 => {
                use crate::timer::timg::asynch::timg1_timer0_handler;
                unsafe {
                    {
                        interrupt::bind_interrupt(
                            Interrupt::TG1_T0_LEVEL,
                            timg1_timer0_handler.handler(),
                        );
                        interrupt::enable(Interrupt::TG1_T0_LEVEL, timg1_timer0_handler.priority())
                            .unwrap();
                    }
                    #[cfg(timg_timer1)]
                    {
                        use crate::timer::timg::asynch::timg1_timer1_handler;
                        interrupt::bind_interrupt(
                            Interrupt::TG1_T1_LEVEL,
                            timg1_timer1_handler.handler(),
                        );
                        interrupt::enable(Interrupt::TG1_T1_LEVEL, timg1_timer1_handler.priority())
                            .unwrap();
                    }
                }
            }
            _ => unreachable!(),
        }

        Self::new_inner(_timer_group)
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

    /// Block until the timer has elapsed.
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

        lock(&INT_ENA_LOCK[self.timer_group() as usize], || {
            self.register_block()
                .int_ena()
                .modify(|_, w| w.t(self.timer_number()).bit(state));
        });
    }

    fn clear_interrupt(&self) {
        self.register_block()
            .int_clr()
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

        for core in crate::Cpu::other() {
            crate::interrupt::disable(core, interrupt);
        }
        unsafe { interrupt::bind_interrupt(interrupt, handler.handler()) };
        unwrap!(interrupt::enable(interrupt, handler.priority()));
    }

    fn is_interrupt_set(&self) -> bool {
        self.register_block()
            .int_raw()
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
    unsafe fn clone_unchecked(&self) -> Self::P {
        core::ptr::read(self as *const _)
    }
}

#[doc(hidden)]
pub trait Instance: Sealed {
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

/// A timer within a Timer Group.
pub struct TimerX<TG, const T: u8 = 0> {
    phantom: PhantomData<TG>,
}

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

        lock(&INT_ENA_LOCK[self.timer_group() as usize], || {
            self.register_block()
                .int_ena()
                .modify(|_, w| w.t(T).set_bit());
        });
    }

    fn unlisten(&self) {
        lock(&INT_ENA_LOCK[self.timer_group() as usize], || {
            self.register_block()
                .int_ena()
                .modify(|_, w| w.t(T).clear_bit());
        });
    }

    fn clear_interrupt(&self) {
        self.register_block()
            .int_clr()
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
        self.register_block().int_raw().read().t(T).bit_is_set()
    }

    fn set_divider(&self, divider: u16) {
        unsafe { Self::t() }
            .config()
            .modify(|_, w| unsafe { w.divider().bits(divider) });
    }
}

/// Timer 0 in the Timer Group.
pub type Timer0<TG> = TimerX<TG, 0>;

/// Timer 1 in the Timer Group.
#[cfg(timg_timer1)]
pub type Timer1<TG> = TimerX<TG, 1>;

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

impl<T, DM> embedded_hal_02::timer::Periodic for Timer<T, DM>
where
    T: Instance + super::Timer,
    DM: Mode,
{
}

/// Behavior of the MWDT stage if it times out.
#[allow(unused)]
#[derive(Debug, Clone, Copy)]
pub enum MwdtStageAction {
    /// No effect on the system.
    Off         = 0,
    /// Trigger an interrupt.
    Interrupt   = 1,
    /// Reset the CPU core.
    ResetCpu    = 2,
    /// Reset the main system, power management unit and RTC peripherals.
    ResetSystem = 3,
}

/// MWDT stages.
///
/// Timer stages allow for a timer to have a series of different timeout values
/// and corresponding expiry action.
#[derive(Debug, Clone, Copy)]
pub enum MwdtStage {
    /// MWDT stage 0.
    Stage0,
    /// MWDT stage 1.
    Stage1,
    /// MWDT stage 2.
    Stage2,
    /// MWDT stage 3.
    Stage3,
}

/// Watchdog timer
pub struct Wdt<TG> {
    phantom: PhantomData<TG>,
}

/// Watchdog driver
impl<TG> Wdt<TG>
where
    TG: TimerGroupInstance,
{
    /// Construct a new instance of [`Wdt`]
    pub fn new() -> Self {
        #[cfg(lp_wdt)]
        PeripheralClockControl::enable(crate::system::Peripheral::Wdt);

        TG::configure_wdt_src_clk();

        Self {
            phantom: PhantomData,
        }
    }

    /// Enable the watchdog timer instance
    pub fn enable(&mut self) {
        // SAFETY: The `TG` instance being modified is owned by `self`, which is behind
        //         a mutable reference.
        unsafe { self.set_wdt_enabled(true) };
    }

    /// Disable the watchdog timer instance
    pub fn disable(&mut self) {
        // SAFETY: The `TG` instance being modified is owned by `self`, which is behind
        //         a mutable reference.
        unsafe { self.set_wdt_enabled(false) };
    }

    /// Forcibly enable or disable the watchdog timer
    ///
    /// # Safety
    ///
    /// This bypasses the usual ownership rules for the peripheral, so users
    /// must take care to ensure that no driver instance is active for the
    /// timer.
    pub unsafe fn set_wdt_enabled(&mut self, enabled: bool) {
        let reg_block = unsafe { &*TG::register_block() };

        self.set_write_protection(false);

        if !enabled {
            reg_block.wdtconfig0().write(|w| unsafe { w.bits(0) });
        } else {
            reg_block.wdtconfig0().write(|w| w.wdt_en().bit(true));

            reg_block
                .wdtconfig0()
                .write(|w| w.wdt_flashboot_mod_en().bit(false));

            #[cfg_attr(esp32, allow(unused_unsafe))]
            reg_block.wdtconfig0().write(|w| unsafe {
                w.wdt_en()
                    .bit(true)
                    .wdt_stg0()
                    .bits(MwdtStageAction::ResetSystem as u8)
                    .wdt_cpu_reset_length()
                    .bits(7)
                    .wdt_sys_reset_length()
                    .bits(7)
                    .wdt_stg1()
                    .bits(MwdtStageAction::Off as u8)
                    .wdt_stg2()
                    .bits(MwdtStageAction::Off as u8)
                    .wdt_stg3()
                    .bits(MwdtStageAction::Off as u8)
            });

            #[cfg(any(esp32c2, esp32c3, esp32c6))]
            reg_block
                .wdtconfig0()
                .modify(|_, w| w.wdt_conf_update_en().set_bit());
        }

        self.set_write_protection(true);
    }

    /// Feed the watchdog timer
    pub fn feed(&mut self) {
        let reg_block = unsafe { &*TG::register_block() };

        self.set_write_protection(false);

        reg_block.wdtfeed().write(|w| unsafe { w.bits(1) });

        self.set_write_protection(true);
    }

    fn set_write_protection(&mut self, enable: bool) {
        let reg_block = unsafe { &*TG::register_block() };

        let wkey = if enable { 0u32 } else { 0x50D8_3AA1u32 };

        reg_block
            .wdtwprotect()
            .write(|w| unsafe { w.wdt_wkey().bits(wkey) });
    }

    /// Set the timeout, in microseconds, of the watchdog timer
    pub fn set_timeout(&mut self, stage: MwdtStage, timeout: MicrosDurationU64) {
        let timeout_raw = (timeout.to_nanos() * 10 / 125) as u32;

        let reg_block = unsafe { &*TG::register_block() };

        self.set_write_protection(false);

        reg_block
            .wdtconfig1()
            .write(|w| unsafe { w.wdt_clk_prescale().bits(1) });

        unsafe {
            match stage {
                MwdtStage::Stage0 => reg_block
                    .wdtconfig2()
                    .write(|w| w.wdt_stg0_hold().bits(timeout_raw)),
                MwdtStage::Stage1 => reg_block
                    .wdtconfig3()
                    .write(|w| w.wdt_stg1_hold().bits(timeout_raw)),
                MwdtStage::Stage2 => reg_block
                    .wdtconfig4()
                    .write(|w| w.wdt_stg2_hold().bits(timeout_raw)),
                MwdtStage::Stage3 => reg_block
                    .wdtconfig5()
                    .write(|w| w.wdt_stg3_hold().bits(timeout_raw)),
            };
        }

        #[cfg(any(esp32c2, esp32c3, esp32c6))]
        reg_block
            .wdtconfig0()
            .modify(|_, w| w.wdt_conf_update_en().set_bit());

        self.set_write_protection(true);
    }

    /// Set the stage action of the MWDT for a specific stage.
    ///
    /// This function modifies MWDT behavior only if a custom bootloader with
    /// the following modifications is used:
    /// - `ESP_TASK_WDT_EN` parameter **disabled**
    /// - `ESP_INT_WDT` parameter **disabled**
    pub fn set_stage_action(&mut self, stage: MwdtStage, action: MwdtStageAction) {
        let reg_block = unsafe { &*TG::register_block() };

        self.set_write_protection(false);

        match stage {
            MwdtStage::Stage0 => {
                reg_block
                    .wdtconfig0()
                    .modify(|_, w| unsafe { w.wdt_stg0().bits(action as u8) });
            }
            MwdtStage::Stage1 => {
                reg_block
                    .wdtconfig0()
                    .modify(|_, w| unsafe { w.wdt_stg1().bits(action as u8) });
            }
            MwdtStage::Stage2 => {
                reg_block
                    .wdtconfig0()
                    .modify(|_, w| unsafe { w.wdt_stg2().bits(action as u8) });
            }
            MwdtStage::Stage3 => {
                reg_block
                    .wdtconfig0()
                    .modify(|_, w| unsafe { w.wdt_stg3().bits(action as u8) });
            }
        }

        self.set_write_protection(true);
    }
}

impl<TG> crate::private::Sealed for Wdt<TG> where TG: TimerGroupInstance {}

impl<TG> InterruptConfigurable for Wdt<TG>
where
    TG: TimerGroupInstance,
{
    fn set_interrupt_handler(&mut self, handler: interrupt::InterruptHandler) {
        let interrupt = TG::wdt_interrupt();
        unsafe {
            interrupt::bind_interrupt(interrupt, handler.handler());
            interrupt::enable(interrupt, handler.priority()).unwrap();
        }
    }
}

impl<TG> Default for Wdt<TG>
where
    TG: TimerGroupInstance,
{
    fn default() -> Self {
        Self::new()
    }
}

impl<TG> embedded_hal_02::watchdog::WatchdogDisable for Wdt<TG>
where
    TG: TimerGroupInstance,
{
    fn disable(&mut self) {
        self.disable();
    }
}

impl<TG> embedded_hal_02::watchdog::WatchdogEnable for Wdt<TG>
where
    TG: TimerGroupInstance,
{
    type Time = MicrosDurationU64;

    fn start<T>(&mut self, period: T)
    where
        T: Into<Self::Time>,
    {
        self.enable();
        self.set_timeout(MwdtStage::Stage0, period.into());
    }
}

impl<TG> embedded_hal_02::watchdog::Watchdog for Wdt<TG>
where
    TG: TimerGroupInstance,
{
    fn feed(&mut self) {
        self.feed();
    }
}

// Async functionality of the timer groups.
mod asynch {
    use core::{
        pin::Pin,
        task::{Context, Poll},
    };

    use embassy_sync::waitqueue::AtomicWaker;
    use procmacros::handler;

    use super::*;

    cfg_if::cfg_if! {
        if #[cfg(all(timg1, timg_timer1))] {
            const NUM_WAKERS: usize = 4;
        } else if #[cfg(timg1)] {
            const NUM_WAKERS: usize = 2;
        } else {
            const NUM_WAKERS: usize = 1;
        }
    }

    static WAKERS: [AtomicWaker; NUM_WAKERS] = [const { AtomicWaker::new() }; NUM_WAKERS];

    pub(crate) struct TimerFuture<'a, T>
    where
        T: Instance,
    {
        timer: &'a Timer<T, crate::Async>,
    }

    impl<'a, T> TimerFuture<'a, T>
    where
        T: Instance,
    {
        pub(crate) fn new(timer: &'a Timer<T, crate::Async>) -> Self {
            use crate::timer::Timer;

            timer.enable_interrupt(true);

            Self { timer }
        }

        fn event_bit_is_clear(&self) -> bool {
            self.timer
                .register_block()
                .int_ena()
                .read()
                .t(self.timer.timer_number())
                .bit_is_clear()
        }
    }

    impl<T> core::future::Future for TimerFuture<'_, T>
    where
        T: Instance,
    {
        type Output = ();

        fn poll(self: Pin<&mut Self>, ctx: &mut Context<'_>) -> Poll<Self::Output> {
            let index = (self.timer.timer_number() << 1) | self.timer.timer_group();
            WAKERS[index as usize].register(ctx.waker());

            if self.event_bit_is_clear() {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        }
    }

    impl<T> Drop for TimerFuture<'_, T>
    where
        T: Instance,
    {
        fn drop(&mut self) {
            self.timer.clear_interrupt();
        }
    }

    impl<T> embedded_hal_async::delay::DelayNs for Timer<T, crate::Async>
    where
        T: Instance,
    {
        async fn delay_ns(&mut self, ns: u32) {
            use crate::timer::Timer as _;

            let period = MicrosDurationU64::from_ticks(ns.div_ceil(1000) as u64);
            self.load_value(period).unwrap();
            self.start();
            self.listen();

            TimerFuture::new(self).await;
        }
    }

    // INT_ENA means that when the interrupt occurs, it will show up in the INT_ST.
    // Clearing INT_ENA that it won't show up on INT_ST but if interrupt is
    // already there, it won't clear it - that's why we need to clear the INT_CLR as
    // well.
    #[handler]
    pub(crate) fn timg0_timer0_handler() {
        lock(&INT_ENA_LOCK[0], || {
            unsafe { &*crate::peripherals::TIMG0::PTR }
                .int_ena()
                .modify(|_, w| w.t(0).clear_bit())
        });

        unsafe { &*crate::peripherals::TIMG0::PTR }
            .int_clr()
            .write(|w| w.t(0).clear_bit_by_one());

        WAKERS[0].wake();
    }

    #[cfg(timg1)]
    #[handler]
    pub(crate) fn timg1_timer0_handler() {
        lock(&INT_ENA_LOCK[1], || {
            unsafe { &*crate::peripherals::TIMG1::PTR }
                .int_ena()
                .modify(|_, w| w.t(0).clear_bit())
        });
        unsafe { &*crate::peripherals::TIMG1::PTR }
            .int_clr()
            .write(|w| w.t(0).clear_bit_by_one());

        WAKERS[1].wake();
    }

    #[cfg(timg_timer1)]
    #[handler]
    pub(crate) fn timg0_timer1_handler() {
        lock(&INT_ENA_LOCK[0], || {
            unsafe { &*crate::peripherals::TIMG0::PTR }
                .int_ena()
                .modify(|_, w| w.t(1).clear_bit())
        });
        unsafe { &*crate::peripherals::TIMG0::PTR }
            .int_clr()
            .write(|w| w.t(1).clear_bit_by_one());

        WAKERS[2].wake();
    }

    #[cfg(all(timg1, timg_timer1))]
    #[handler]
    pub(crate) fn timg1_timer1_handler() {
        lock(&INT_ENA_LOCK[1], || {
            unsafe { &*crate::peripherals::TIMG1::PTR }
                .int_ena()
                .modify(|_, w| w.t(1).clear_bit())
        });

        unsafe { &*crate::peripherals::TIMG1::PTR }
            .int_clr()
            .write(|w| w.t(1).clear_bit_by_one());

        WAKERS[3].wake();
    }
}

/// Event Task Matrix
#[cfg(soc_etm)]
pub mod etm {
    use super::*;
    use crate::etm::{EtmEvent, EtmTask};

    /// Event Task Matrix event for a timer.
    pub struct Event {
        id: u8,
    }

    /// Event Task Matrix task for a timer.
    pub struct Task {
        id: u8,
    }

    impl EtmEvent for Event {
        fn id(&self) -> u8 {
            self.id
        }
    }

    impl Sealed for Event {}

    impl EtmTask for Task {
        fn id(&self) -> u8 {
            self.id
        }
    }

    impl Sealed for Task {}

    /// General purpose timer ETM events.
    pub trait Events<TG> {
        /// ETM event triggered on alarm
        fn on_alarm(&self) -> Event;
    }

    /// General purpose timer ETM tasks
    pub trait Tasks<TG> {
        /// ETM task to start the counter
        fn cnt_start(&self) -> Task;

        /// ETM task to start the alarm
        fn cnt_stop(&self) -> Task;

        /// ETM task to stop the counter
        fn cnt_reload(&self) -> Task;

        /// ETM task to reload the counter
        fn cnt_cap(&self) -> Task;

        /// ETM task to load the counter with the value stored when the last
        /// `now()` was called
        fn alarm_start(&self) -> Task;
    }

    impl<TG> Events<TG> for Timer0<TG>
    where
        TG: TimerGroupInstance,
    {
        fn on_alarm(&self) -> Event {
            Event { id: 48 + TG::id() }
        }
    }

    impl<TG> Tasks<TG> for Timer0<TG>
    where
        TG: TimerGroupInstance,
    {
        fn cnt_start(&self) -> Task {
            Task { id: 88 + TG::id() }
        }

        fn alarm_start(&self) -> Task {
            Task { id: 90 + TG::id() }
        }

        fn cnt_stop(&self) -> Task {
            Task { id: 92 + TG::id() }
        }

        fn cnt_reload(&self) -> Task {
            Task { id: 94 + TG::id() }
        }

        fn cnt_cap(&self) -> Task {
            Task { id: 96 + TG::id() }
        }
    }
}
