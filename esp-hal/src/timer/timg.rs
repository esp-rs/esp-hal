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
//! use esp_hal::timer::timg::TimerGroup;
//! use esp_hal::timer::Timer;
//!
//! let timg0 = TimerGroup::new(peripherals.TIMG0);
//! let timer0 = timg0.timer0;
//!
//! // Get the current timestamp, in microseconds:
//! let now = timer0.now();
//!
//! // Wait for timeout:
//! timer0.load_value(Duration::from_secs(1));
//! timer0.start();
//!
//! while !timer0.is_interrupt_set() {
//!     // Wait
//! }
//!
//! timer0.clear_interrupt();
//! # Ok(())
//! # }
//! ```
//! 
//! ### Watchdog Timer
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! use esp_hal::timer::timg::TimerGroup;
//! use esp_hal::timer::timg::MwdtStage;
//! use esp_hal::timer::Timer;
//!
//! let timg0 = TimerGroup::new(peripherals.TIMG0);
//! let mut wdt = timg0.wdt;
//!
//! wdt.set_timeout(MwdtStage::Stage0, Duration::from_millis(5_000));
//! wdt.enable();
//!
//! loop {
//!     wdt.feed();
//! }
//! # }
//! ```
use core::marker::PhantomData;

use super::Error;
#[cfg(timg1)]
use crate::peripherals::TIMG1;
#[cfg(any(esp32c6, esp32h2))]
use crate::soc::constants::TIMG_DEFAULT_CLK_SRC;
use crate::{
    clock::Clocks,
    interrupt::{self, InterruptConfigurable, InterruptHandler},
    pac::timg0::RegisterBlock,
    peripheral::Peripheral,
    peripherals::{Interrupt, TIMG0},
    private::Sealed,
    system::PeripheralClockControl,
    time::{Duration, Instant, Rate},
};

const NUM_TIMG: usize = 1 + cfg!(timg1) as usize;

cfg_if::cfg_if! {
    // We need no locks when a TIMG has a single timer, and we don't need locks for ESP32
    // and S2 where the effective interrupt enable register (config) is not shared between
    // the timers.
    if #[cfg(all(timg_timer1, not(any(esp32, esp32s2))))] {
        use crate::sync::{lock, RawMutex};
        static INT_ENA_LOCK: [RawMutex; NUM_TIMG] = [const { RawMutex::new() }; NUM_TIMG];
    }
}

/// A timer group consisting of
#[cfg_attr(not(timg_timer1), doc = "a general purpose timer")]
#[cfg_attr(timg_timer1, doc = "2 timers")]
/// and a watchdog timer.
pub struct TimerGroup<T>
where
    T: TimerGroupInstance,
{
    _timer_group: PhantomData<T>,
    /// Timer 0
    pub timer0: Timer,
    /// Timer 1
    #[cfg(timg_timer1)]
    pub timer1: Timer,
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
        Self::regs()
    }

    fn configure_src_clk() {
        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
                // ESP32 has only APB clock source, do nothing
            } else if #[cfg(any(esp32c2, esp32c3, esp32s2, esp32s3))] {
                unsafe {
                    (*<Self as TimerGroupInstance>::register_block())
                        .t(0)
                        .config()
                        .modify(|_, w| w.use_xtal().clear_bit());
                }
            } else if #[cfg(any(esp32c6, esp32h2))] {
                crate::peripherals::PCR::regs()
                    .timergroup0_timer_clk_conf()
                    .modify(|_, w| unsafe { w.tg0_timer_clk_sel().bits(TIMG_DEFAULT_CLK_SRC) });
            }
        }
    }

    fn enable_peripheral() {
        PeripheralClockControl::enable(crate::system::Peripheral::Timg0);
    }

    fn reset_peripheral() {
        // FIXME: for TIMG0 do nothing for now because the reset breaks
        // `time::Instant::now`
    }

    fn configure_wdt_src_clk() {
        cfg_if::cfg_if! {
            if #[cfg(any(esp32, esp32s2, esp32s3))] {
                // ESP32, ESP32-S2, and ESP32-S3 use only ABP, do nothing
            } else if #[cfg(any(esp32c2, esp32c3))] {
                unsafe {
                    (*<Self as TimerGroupInstance>::register_block())
                        .wdtconfig0()
                        .modify(|_, w| w.wdt_use_xtal().clear_bit());
                }
            } else if #[cfg(any(esp32c6, esp32h2))] {
                crate::peripherals::PCR::regs()
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
        Self::regs()
    }

    fn configure_src_clk() {
        cfg_if::cfg_if! {
            if #[cfg(any(esp32, esp32c2, esp32c3))] {
                // ESP32 has only APB clock source, do nothing
                // ESP32-C2 and ESP32-C3 don't have t1config only t0config, do nothing
            } else if #[cfg(any(esp32c6, esp32h2))] {
                crate::peripherals::PCR::regs()
                    .timergroup1_timer_clk_conf()
                    .modify(|_, w| unsafe { w.tg1_timer_clk_sel().bits(TIMG_DEFAULT_CLK_SRC) });
            } else if #[cfg(any(esp32s2, esp32s3))] {
                unsafe {
                    (*<Self as TimerGroupInstance>::register_block())
                        .t(1)
                        .config()
                        .modify(|_, w| w.use_xtal().clear_bit());
                }
            }
        }
    }

    fn enable_peripheral() {
        PeripheralClockControl::enable(crate::system::Peripheral::Timg1);
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
                crate::peripherals::PCR::regs()
                    .timergroup1_wdt_clk_conf()
                    .modify(|_, w| unsafe { w.tg1_wdt_clk_sel().bits(TIMG_DEFAULT_CLK_SRC) });
            }
        }
    }

    fn wdt_interrupt() -> Interrupt {
        Interrupt::TG1_WDT_LEVEL
    }
}

impl<T> TimerGroup<T>
where
    T: TimerGroupInstance,
{
    /// Construct a new instance of [`TimerGroup`] in blocking mode
    pub fn new(_timer_group: T) -> Self {
        T::reset_peripheral();
        T::enable_peripheral();

        T::configure_src_clk();

        Self {
            _timer_group: PhantomData,
            timer0: Timer {
                timer: 0,
                tg: T::id(),
                register_block: T::register_block(),
            },
            #[cfg(timg_timer1)]
            timer1: Timer {
                timer: 1,
                tg: T::id(),
                register_block: T::register_block(),
            },
            wdt: Wdt::new(),
        }
    }
}

impl super::Timer for Timer {
    fn start(&self) {
        self.set_counter_active(false);
        self.set_alarm_active(false);

        self.reset_counter();
        self.set_counter_decrementing(false);

        self.set_counter_active(true);
        self.set_alarm_active(true);
    }

    fn stop(&self) {
        self.set_counter_active(false);
    }

    fn reset(&self) {
        self.reset_counter()
    }

    fn is_running(&self) -> bool {
        self.is_counter_active()
    }

    fn now(&self) -> Instant {
        self.now()
    }

    fn load_value(&self, value: Duration) -> Result<(), Error> {
        self.load_value(value)
    }

    fn enable_auto_reload(&self, auto_reload: bool) {
        self.set_auto_reload(auto_reload)
    }

    fn enable_interrupt(&self, state: bool) {
        self.set_interrupt_enabled(state);
    }

    fn clear_interrupt(&self) {
        self.clear_interrupt()
    }

    fn is_interrupt_set(&self) -> bool {
        self.is_interrupt_set()
    }

    async fn wait(&self) {
        asynch::TimerFuture::new(self).await
    }

    fn async_interrupt_handler(&self) -> InterruptHandler {
        match (self.timer_group(), self.timer_number()) {
            (0, 0) => asynch::timg0_timer0_handler,
            #[cfg(timg_timer1)]
            (0, 1) => asynch::timg0_timer1_handler,
            #[cfg(timg1)]
            (1, 0) => asynch::timg1_timer0_handler,
            #[cfg(all(timg_timer1, timg1))]
            (1, 1) => asynch::timg1_timer1_handler,
            _ => unreachable!(),
        }
    }

    fn peripheral_interrupt(&self) -> Interrupt {
        match (self.timer_group(), self.timer_number()) {
            (0, 0) => Interrupt::TG0_T0_LEVEL,
            #[cfg(timg_timer1)]
            (0, 1) => Interrupt::TG0_T1_LEVEL,
            #[cfg(timg1)]
            (1, 0) => Interrupt::TG1_T0_LEVEL,
            #[cfg(all(timg_timer1, timg1))]
            (1, 1) => Interrupt::TG1_T1_LEVEL,
            _ => unreachable!(),
        }
    }

    fn set_interrupt_handler(&self, handler: InterruptHandler) {
        self.set_interrupt_handler(handler)
    }
}

impl Peripheral for Timer {
    type P = Self;

    #[inline]
    unsafe fn clone_unchecked(&self) -> Self::P {
        core::ptr::read(self as *const _)
    }
}

/// A timer within a Timer Group.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Timer {
    register_block: *const RegisterBlock,
    timer: u8,
    tg: u8,
}

impl Sealed for Timer {}
unsafe impl Send for Timer {}

/// Timer peripheral instance
impl Timer {
    pub(crate) fn set_interrupt_handler(&self, handler: InterruptHandler) {
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

    fn register_block(&self) -> &RegisterBlock {
        unsafe { &*self.register_block }
    }

    fn timer_group(&self) -> u8 {
        self.tg
    }

    fn timer_number(&self) -> u8 {
        self.timer
    }

    fn t(&self) -> &crate::pac::timg0::T {
        self.register_block().t(self.timer_number().into())
    }

    fn reset_counter(&self) {
        let t = self.t();

        t.loadlo().write(|w| unsafe { w.load_lo().bits(0) });
        t.loadhi().write(|w| unsafe { w.load_hi().bits(0) });

        t.load().write(|w| unsafe { w.load().bits(1) });
    }

    fn set_counter_active(&self, state: bool) {
        self.t().config().modify(|_, w| w.en().bit(state));
    }

    fn is_counter_active(&self) -> bool {
        self.t().config().read().en().bit_is_set()
    }

    fn set_counter_decrementing(&self, decrementing: bool) {
        self.t()
            .config()
            .modify(|_, w| w.increase().bit(!decrementing));
    }

    fn set_auto_reload(&self, auto_reload: bool) {
        self.t()
            .config()
            .modify(|_, w| w.autoreload().bit(auto_reload));
    }

    fn set_alarm_active(&self, state: bool) {
        self.t().config().modify(|_, w| w.alarm_en().bit(state));
    }

    fn load_value(&self, value: Duration) -> Result<(), Error> {
        cfg_if::cfg_if! {
            if #[cfg(esp32h2)] {
                // ESP32-H2 is using PLL_48M_CLK source instead of APB_CLK
                let clk_src = Clocks::get().pll_48m_clock;
            } else {
                let clk_src = Clocks::get().apb_clock;
            }
        }
        let ticks = timeout_to_ticks(value, clk_src, self.divider());

        // The counter is 54-bits wide, so we must ensure that the provided
        // value is not too wide:
        if (ticks & !0x3F_FFFF_FFFF_FFFF) != 0 {
            return Err(Error::InvalidTimeout);
        }

        let high = (ticks >> 32) as u32;
        let low = (ticks & 0xFFFF_FFFF) as u32;

        let t = self.t();

        t.alarmlo().write(|w| unsafe { w.alarm_lo().bits(low) });
        t.alarmhi().write(|w| unsafe { w.alarm_hi().bits(high) });

        Ok(())
    }

    fn clear_interrupt(&self) {
        self.register_block()
            .int_clr()
            .write(|w| w.t(self.timer).clear_bit_by_one());
        let periodic = self.t().config().read().autoreload().bit_is_set();
        self.set_alarm_active(periodic);
    }

    fn now(&self) -> Instant {
        let t = self.t();

        t.update().write(|w| w.update().set_bit());
        while t.update().read().update().bit_is_set() {
            // Wait for the update to complete
        }

        let value_lo = t.lo().read().bits() as u64;
        let value_hi = t.hi().read().bits() as u64;

        let ticks = (value_hi << 32) | value_lo;
        cfg_if::cfg_if! {
            if #[cfg(esp32h2)] {
                // ESP32-H2 is using PLL_48M_CLK source instead of APB_CLK
                let clk_src = Clocks::get().pll_48m_clock;
            } else {
                let clk_src = Clocks::get().apb_clock;
            }
        }
        let micros = ticks_to_timeout(ticks, clk_src, self.divider());

        Instant::from_ticks(micros)
    }

    fn divider(&self) -> u32 {
        let t = self.t();

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
            .int_raw()
            .read()
            .t(self.timer)
            .bit_is_set()
    }

    fn set_interrupt_enabled(&self, state: bool) {
        cfg_if::cfg_if! {
            if #[cfg(any(esp32, esp32s2))] {
                self.register_block()
                    .t(self.timer as usize)
                    .config()
                    .modify(|_, w| w.level_int_en().bit(state));
            } else if #[cfg(timg_timer1)] {
                lock(&INT_ENA_LOCK[self.timer_group() as usize], || {
                    self.register_block()
                        .int_ena()
                        .modify(|_, w| w.t(self.timer_number()).bit(state));
                });
            } else {
                self.register_block()
                    .int_ena()
                    .modify(|_, w| w.t(0).bit(state));
            }
        }
    }
}

fn ticks_to_timeout(ticks: u64, clock: Rate, divider: u32) -> u64 {
    // 1_000_000 is used to get rid of `float` calculations
    let period: u64 = 1_000_000 * 1_000_000 / (clock.as_hz() as u64 / divider as u64);

    ticks * period / 1_000_000
}

fn timeout_to_ticks(timeout: Duration, clock: Rate, divider: u32) -> u64 {
    let micros = timeout.as_micros();

    // 1_000_000 is used to get rid of `float` calculations
    let period: u64 = 1_000_000 * 1_000_000 / ((clock.as_hz() / divider) as u64);

    (1_000_000 * micros) / period
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
    pub fn set_timeout(&mut self, stage: MwdtStage, timeout: Duration) {
        let timeout_raw = (timeout.as_micros() * 10_000 / 125) as u32;

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

// Async functionality of the timer groups.
mod asynch {
    use core::{
        pin::Pin,
        task::{Context, Poll},
    };

    use procmacros::handler;

    use super::*;
    use crate::asynch::AtomicWaker;

    const NUM_WAKERS: usize = {
        let timer_per_group = 1 + cfg!(timg_timer1) as usize;
        NUM_TIMG * timer_per_group
    };

    static WAKERS: [AtomicWaker; NUM_WAKERS] = [const { AtomicWaker::new() }; NUM_WAKERS];

    fn waker(timer: &Timer) -> &AtomicWaker {
        let index = (timer.timer_number() << 1) | timer.timer_group();
        &WAKERS[index as usize]
    }

    pub(crate) struct TimerFuture<'a> {
        timer: &'a Timer,
    }

    impl<'a> TimerFuture<'a> {
        pub(crate) fn new(timer: &'a Timer) -> Self {
            use crate::timer::Timer;

            timer.enable_interrupt(true);

            Self { timer }
        }

        fn is_done(&self) -> bool {
            self.timer.is_interrupt_set()
        }
    }

    impl core::future::Future for TimerFuture<'_> {
        type Output = ();

        fn poll(self: Pin<&mut Self>, ctx: &mut Context<'_>) -> Poll<Self::Output> {
            waker(self.timer).register(ctx.waker());

            if self.is_done() {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        }
    }

    impl Drop for TimerFuture<'_> {
        fn drop(&mut self) {
            self.timer.clear_interrupt();
            self.timer.set_interrupt_enabled(false);
        }
    }

    #[inline]
    fn handle_irq(timer: Timer) {
        timer.set_interrupt_enabled(false);
        waker(&timer).wake();
    }

    // INT_ENA means that when the interrupt occurs, it will show up in the
    // INT_ST. Clearing INT_ENA that it won't show up on INT_ST but if
    // interrupt is already there, it won't clear it - that's why we need to
    // clear the INT_CLR as well.
    #[handler]
    pub(crate) fn timg0_timer0_handler() {
        handle_irq(Timer {
            register_block: TIMG0::regs(),
            timer: 0,
            tg: 0,
        });
    }

    #[cfg(timg1)]
    #[handler]
    pub(crate) fn timg1_timer0_handler() {
        handle_irq(Timer {
            register_block: TIMG1::regs(),
            timer: 0,
            tg: 1,
        });
    }

    #[cfg(timg_timer1)]
    #[handler]
    pub(crate) fn timg0_timer1_handler() {
        handle_irq(Timer {
            register_block: TIMG0::regs(),
            timer: 1,
            tg: 0,
        });
    }

    #[cfg(all(timg1, timg_timer1))]
    #[handler]
    pub(crate) fn timg1_timer1_handler() {
        handle_irq(Timer {
            register_block: TIMG1::regs(),
            timer: 1,
            tg: 1,
        });
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
    pub trait Events {
        /// ETM event triggered on alarm
        fn on_alarm(&self) -> Event;
    }

    /// General purpose timer ETM tasks
    pub trait Tasks {
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

    impl Events for Timer {
        fn on_alarm(&self) -> Event {
            Event {
                id: 48 + self.timer_group(),
            }
        }
    }

    impl Tasks for Timer {
        fn cnt_start(&self) -> Task {
            Task {
                id: 88 + self.timer_group(),
            }
        }

        fn alarm_start(&self) -> Task {
            Task {
                id: 90 + self.timer_group(),
            }
        }

        fn cnt_stop(&self) -> Task {
            Task {
                id: 92 + self.timer_group(),
            }
        }

        fn cnt_reload(&self) -> Task {
            Task {
                id: 94 + self.timer_group(),
            }
        }

        fn cnt_cap(&self) -> Task {
            Task {
                id: 96 + self.timer_group(),
            }
        }
    }
}
