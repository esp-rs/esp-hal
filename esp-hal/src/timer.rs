//! # General-purpose timers
//!
//! ## Overview
//! The `general-purpose timer` peripheral consists of a timer group, which can
//! have up to two timers (depending on the chip) and a watchdog timer. The
//! timer group allows for the management of multiple timers and synchronization
//! between them.
//!
//! This peripheral can be used to perform a variety of
//! tasks, such as triggering an interrupt after a particular interval
//! (periodically and aperiodically), precisely time an interval, act as a
//! hardware clock and so on.
//!
//! Each timer group consists of two general purpose timers and one Main System
//! Watchdog Timer(MSWDT). All general purpose timers are based on 16-bit
//! prescalers and 54-bit auto-reload-capable up-down counters.
//!
//! The driver uses APB as it's clock source.
//!
//! ## Example
//!
//! ```no_run
//! let mut rtc = Rtc::new(peripherals.LPWR, None);
//!
//! // Create timer groups
//! let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
//! // Get watchdog timers of timer groups
//! let mut wdt0 = timer_group0.wdt;
//! let timer_group1 = TimerGroup::new(peripherals.TIMG1, &clocks);
//! let mut wdt1 = timer_group1.wdt;
//!
//! // Disable watchdog timers
//! rtc.swd.disable();
//! rtc.rwdt.disable();
//! wdt0.disable();
//! wdt1.disable();
//! ```

use core::{
    marker::PhantomData,
    ops::{Deref, DerefMut},
};

use fugit::{HertzU32, MicrosDurationU64};

#[cfg(timg1)]
use crate::peripherals::TIMG1;
#[cfg(any(esp32c6, esp32h2))]
use crate::soc::constants::TIMG_DEFAULT_CLK_SRC;
use crate::{
    clock::Clocks,
    interrupt::InterruptHandler,
    peripheral::{Peripheral, PeripheralRef},
    peripherals::{timg0::RegisterBlock, TIMG0},
    system::PeripheralClockControl,
};

/// Custom timer error type
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    TimerActive,
    TimerInactive,
    AlarmInactive,
}

/// Interrupts which can be registered in [crate::Blocking] mode
#[derive(Debug, Default)]
pub struct TimerInterrupts {
    pub timer0_t0: Option<InterruptHandler>,
    pub timer0_t1: Option<InterruptHandler>,
    pub timer0_wdt: Option<InterruptHandler>,
    #[cfg(not(any(esp32c2, esp32c3, esp32c6, esp32h2)))]
    pub timer1_t0: Option<InterruptHandler>,
    #[cfg(not(any(esp32c2, esp32c3, esp32c6, esp32h2)))]
    pub timer1_t1: Option<InterruptHandler>,
    #[cfg(not(any(esp32c2, esp32c3, esp32c6, esp32h2)))]
    pub timer1_wdt: Option<InterruptHandler>,
}

// A timergroup consisting of up to 2 timers (chip dependent) and a watchdog
// timer
pub struct TimerGroup<'d, T, DM>
where
    T: TimerGroupInstance,
    DM: crate::Mode,
{
    _timer_group: PeripheralRef<'d, T>,
    pub timer0: Timer<Timer0<T>, DM>,
    #[cfg(not(any(esp32c2, esp32c3, esp32c6, esp32h2)))]
    pub timer1: Timer<Timer1<T>, DM>,
    pub wdt: Wdt<T, DM>,
}

pub trait TimerGroupInstance {
    fn register_block() -> *const RegisterBlock;
    fn configure_src_clk();
    fn configure_wdt_src_clk();
    fn id() -> u8;
}

impl TimerGroupInstance for TIMG0 {
    fn id() -> u8 {
        0
    }
    #[inline(always)]
    fn register_block() -> *const RegisterBlock {
        crate::peripherals::TIMG0::PTR
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
        crate::peripherals::TIMG1::PTR
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

impl<'d, T> TimerGroup<'d, T, crate::Blocking>
where
    T: TimerGroupInstance,
{
    pub fn new(
        timer_group: impl Peripheral<P = T> + 'd,
        clocks: &Clocks,
        isr: Option<TimerInterrupts>,
    ) -> Self {
        crate::into_ref!(timer_group);

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

        #[cfg(not(any(esp32c2, esp32c3, esp32c6, esp32h2)))]
        let timer1 = Timer::new(
            Timer1 {
                phantom: PhantomData,
            },
            clocks.apb_clock,
        );

        if let Some(isr) = isr {
            if let Some(handler) = isr.timer0_t0 {
                unsafe {
                    crate::interrupt::bind_interrupt(
                        crate::peripherals::Interrupt::TG0_T0_LEVEL,
                        handler.handler(),
                    );
                    crate::interrupt::enable(
                        crate::peripherals::Interrupt::TG0_T0_LEVEL,
                        handler.priority(),
                    )
                    .unwrap();
                }
            }

            #[cfg(any(esp32, esp32s2, esp32s3))]
            if let Some(handler) = isr.timer0_t1 {
                unsafe {
                    crate::interrupt::bind_interrupt(
                        crate::peripherals::Interrupt::TG0_T1_LEVEL,
                        handler.handler(),
                    );
                    crate::interrupt::enable(
                        crate::peripherals::Interrupt::TG0_T1_LEVEL,
                        handler.priority(),
                    )
                    .unwrap();
                }
            }

            if let Some(handler) = isr.timer0_wdt {
                unsafe {
                    crate::interrupt::bind_interrupt(
                        crate::peripherals::Interrupt::TG0_WDT_LEVEL,
                        handler.handler(),
                    );
                    crate::interrupt::enable(
                        crate::peripherals::Interrupt::TG0_WDT_LEVEL,
                        handler.priority(),
                    )
                    .unwrap();
                }
            }

            #[cfg(not(any(esp32c2, esp32c3, esp32c6, esp32h2)))]
            {
                if let Some(handler) = isr.timer1_t0 {
                    unsafe {
                        crate::interrupt::bind_interrupt(
                            crate::peripherals::Interrupt::TG1_T0_LEVEL,
                            handler.handler(),
                        );
                        crate::interrupt::enable(
                            crate::peripherals::Interrupt::TG1_T0_LEVEL,
                            handler.priority(),
                        )
                        .unwrap();
                    }
                }

                #[cfg(any(esp32, esp32s2, esp32s3))]
                if let Some(handler) = isr.timer1_t1 {
                    unsafe {
                        crate::interrupt::bind_interrupt(
                            crate::peripherals::Interrupt::TG1_T1_LEVEL,
                            handler.handler(),
                        );
                        crate::interrupt::enable(
                            crate::peripherals::Interrupt::TG1_T1_LEVEL,
                            handler.priority(),
                        )
                        .unwrap();
                    }
                }

                if let Some(handler) = isr.timer1_wdt {
                    unsafe {
                        crate::interrupt::bind_interrupt(
                            crate::peripherals::Interrupt::TG1_WDT_LEVEL,
                            handler.handler(),
                        );
                        crate::interrupt::enable(
                            crate::peripherals::Interrupt::TG1_WDT_LEVEL,
                            handler.priority(),
                        )
                        .unwrap();
                    }
                }
            }
        }

        Self {
            _timer_group: timer_group,
            timer0,
            #[cfg(not(any(esp32c2, esp32c3, esp32c6, esp32h2)))]
            timer1,
            wdt: Wdt::new(),
        }
    }
}

impl<'d, T> TimerGroup<'d, T, crate::Async>
where
    T: TimerGroupInstance,
{
    pub fn new_async(timer_group: impl Peripheral<P = T> + 'd, clocks: &Clocks) -> Self {
        crate::into_ref!(timer_group);

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

        #[cfg(not(any(esp32c2, esp32c3, esp32c6, esp32h2)))]
        let timer1 = Timer::new(
            Timer1 {
                phantom: PhantomData,
            },
            clocks.apb_clock,
        );

        Self {
            _timer_group: timer_group,
            timer0,
            #[cfg(not(any(esp32c2, esp32c3, esp32c6, esp32h2)))]
            timer1,
            wdt: Wdt::new(),
        }
    }
}

/// General-purpose Timer driver
pub struct Timer<T, DM: crate::Mode> {
    timg: T,
    apb_clk_freq: HertzU32,
    phantom: PhantomData<DM>,
}

impl<T, DM: crate::Mode> Timer<T, DM>
where
    T: Instance,
{
    /// Create a new timer instance.
    pub fn new(timg: T, apb_clk_freq: HertzU32) -> Self {
        timg.enable_peripheral();

        Self {
            timg,
            apb_clk_freq,
            phantom: PhantomData,
        }
    }

    /// Start the timer with the given time period.
    pub fn start(&mut self, timeout: MicrosDurationU64) {
        self.timg.set_counter_active(false);
        self.timg.set_alarm_active(false);

        self.timg.reset_counter();

        // TODO: can we cache the divider (only get it on initialization)?
        let ticks = timeout_to_ticks(timeout, self.apb_clk_freq, self.timg.divider());
        self.timg.load_alarm_value(ticks);

        self.timg.set_counter_decrementing(false);
        self.timg.set_auto_reload(true);
        self.timg.set_counter_active(true);
        self.timg.set_alarm_active(true);
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

impl<T, DM: crate::Mode> Deref for Timer<T, DM>
where
    T: Instance,
{
    type Target = T;

    fn deref(&self) -> &Self::Target {
        &self.timg
    }
}

impl<T, DM: crate::Mode> DerefMut for Timer<T, DM>
where
    T: Instance,
{
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.timg
    }
}

/// Timer peripheral instance
pub trait Instance: crate::private::Sealed + Enable {
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
}

pub trait Enable: crate::private::Sealed {
    fn enable_peripheral(&self);
}

pub struct TimerX<TG, const T: u8 = 0> {
    phantom: PhantomData<TG>,
}

pub type Timer0<TG> = TimerX<TG, 0>;

#[cfg(not(any(esp32c2, esp32c3, esp32c6, esp32h2)))]
pub type Timer1<TG> = TimerX<TG, 1>;

impl<TG, const T: u8> crate::private::Sealed for TimerX<TG, T> {}

impl<TG, const T: u8> TimerX<TG, T>
where
    TG: TimerGroupInstance,
{
    #[allow(unused)]
    pub(crate) unsafe fn steal() -> Self {
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
    fn reset_counter(&mut self) {
        let t = unsafe { Self::t() };

        t.loadlo().write(|w| unsafe { w.load_lo().bits(0) });

        t.loadhi().write(|w| unsafe { w.load_hi().bits(0) });

        t.load().write(|w| unsafe { w.load().bits(1) });
    }

    fn set_counter_active(&mut self, state: bool) {
        unsafe { Self::t() }
            .config()
            .modify(|_, w| w.en().bit(state));
    }

    fn is_counter_active(&self) -> bool {
        unsafe { Self::t() }.config().read().en().bit_is_set()
    }

    fn set_counter_decrementing(&mut self, decrementing: bool) {
        unsafe { Self::t() }
            .config()
            .modify(|_, w| w.increase().bit(!decrementing));
    }

    fn set_auto_reload(&mut self, auto_reload: bool) {
        unsafe { Self::t() }
            .config()
            .modify(|_, w| w.autoreload().bit(auto_reload));
    }

    fn set_alarm_active(&mut self, state: bool) {
        unsafe { Self::t() }
            .config()
            .modify(|_, w| w.alarm_en().bit(state));
    }

    fn is_alarm_active(&self) -> bool {
        unsafe { Self::t() }.config().read().alarm_en().bit_is_set()
    }

    fn load_alarm_value(&mut self, value: u64) {
        let value = value & 0x3F_FFFF_FFFF_FFFF;
        let high = (value >> 32) as u32;
        let low = (value & 0xFFFF_FFFF) as u32;

        let t = unsafe { Self::t() };

        t.alarmlo().write(|w| unsafe { w.alarm_lo().bits(low) });

        t.alarmhi().write(|w| unsafe { w.alarm_hi().bits(high) });
    }

    fn listen(&mut self) {
        let reg_block = unsafe { &*TG::register_block() };

        // always use level interrupt
        #[cfg(any(esp32, esp32s2))]
        unsafe { Self::t() }
            .config()
            .modify(|_, w| w.level_int_en().set_bit());

        reg_block.int_ena_timers().modify(|_, w| w.t(T).set_bit());
    }

    fn unlisten(&mut self) {
        let reg_block = unsafe { &*TG::register_block() };

        reg_block.int_ena_timers().modify(|_, w| w.t(T).clear_bit());
    }

    fn clear_interrupt(&mut self) {
        let reg_block = unsafe { &*TG::register_block() };

        reg_block
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
        let reg_block = unsafe { &*TG::register_block() };

        reg_block.int_raw_timers().read().t(T).bit_is_set()
    }

    fn set_divider(&mut self, divider: u16) {
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
        PeripheralClockControl::enable(crate::system::Peripheral::Timg0);
    }
}

#[cfg(not(any(esp32c2, esp32c3, esp32c6, esp32h2)))]
impl<TG> Enable for Timer1<TG>
where
    TG: TimerGroupInstance,
{
    fn enable_peripheral(&self) {
        PeripheralClockControl::enable(crate::system::Peripheral::Timg1);
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

#[cfg(feature = "embedded-hal-02")]
impl<T, DM> embedded_hal_02::timer::CountDown for Timer<T, DM>
where
    T: Instance,
    DM: crate::Mode,
{
    type Time = MicrosDurationU64;

    fn start<Time>(&mut self, timeout: Time)
    where
        Time: Into<Self::Time>,
    {
        (*self).start(timeout.into())
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
    T: Instance,
    DM: crate::Mode,
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

#[cfg(feature = "embedded-hal-02")]
impl<T, DM> embedded_hal_02::timer::Periodic for Timer<T, DM>
where
    T: Instance,
    DM: crate::Mode,
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
    DM: crate::Mode,
{
    /// Create a new watchdog timer instance
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

impl<TG, DM> Default for Wdt<TG, DM>
where
    TG: TimerGroupInstance,
    DM: crate::Mode,
{
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(feature = "embedded-hal-02")]
impl<TG, DM> embedded_hal_02::watchdog::WatchdogDisable for Wdt<TG, DM>
where
    TG: TimerGroupInstance,
    DM: crate::Mode,
{
    fn disable(&mut self) {
        self.disable();
    }
}

#[cfg(feature = "embedded-hal-02")]
impl<TG, DM> embedded_hal_02::watchdog::WatchdogEnable for Wdt<TG, DM>
where
    TG: TimerGroupInstance,
    DM: crate::Mode,
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
    DM: crate::Mode,
{
    fn feed(&mut self) {
        self.feed();
    }
}

#[cfg(soc_etm)]
pub mod etm {
    use super::*;
    use crate::{
        etm::{EtmEvent, EtmTask},
        private::Sealed,
    };

    pub struct TimerEtmEvent {
        id: u8,
    }

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

    /// General purpose timer ETM events
    pub trait TimerEtmEvents<TG> {
        fn on_alarm(&self) -> TimerEtmEvent;
    }

    /// General purpose timer ETM tasks
    pub trait TimerEtmTasks<TG> {
        fn cnt_start(&self) -> TimerEtmTask;
        fn cnt_stop(&self) -> TimerEtmTask;
        fn cnt_reload(&self) -> TimerEtmTask;
        fn cnt_cap(&self) -> TimerEtmTask;
        fn alarm_start(&self) -> TimerEtmTask;
    }

    impl<TG> TimerEtmEvents<TG> for Timer0<TG>
    where
        TG: TimerGroupInstance,
    {
        /// ETM event triggered on alarm
        fn on_alarm(&self) -> TimerEtmEvent {
            TimerEtmEvent { id: 48 + TG::id() }
        }
    }

    impl<TG> TimerEtmTasks<TG> for Timer0<TG>
    where
        TG: TimerGroupInstance,
    {
        /// ETM task to start the counter
        fn cnt_start(&self) -> TimerEtmTask {
            TimerEtmTask { id: 88 + TG::id() }
        }

        /// ETM task to start the alarm
        fn alarm_start(&self) -> TimerEtmTask {
            TimerEtmTask { id: 90 + TG::id() }
        }

        /// ETM task to stop the counter
        fn cnt_stop(&self) -> TimerEtmTask {
            TimerEtmTask { id: 92 + TG::id() }
        }

        /// ETM task to reload the counter
        fn cnt_reload(&self) -> TimerEtmTask {
            TimerEtmTask { id: 94 + TG::id() }
        }

        /// ETM task to load the counter with the value stored when the last
        /// `now()` was called
        fn cnt_cap(&self) -> TimerEtmTask {
            TimerEtmTask { id: 96 + TG::id() }
        }
    }
}
