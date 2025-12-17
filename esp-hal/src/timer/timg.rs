#![cfg_attr(docsrs, procmacros::doc_replace)]
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
//! # {before_snippet}
//! use esp_hal::timer::{Timer, timg::TimerGroup};
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
//! # {after_snippet}
//! ```
//!
//! ### Watchdog Timer
//! ```rust, no_run
//! # {before_snippet}
//! use esp_hal::timer::{
//!     Timer,
//!     timg::{MwdtStage, TimerGroup},
//! };
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
#[cfg(timergroup_timg1)]
use crate::peripherals::TIMG1;
#[cfg(soc_has_clock_node_timg0_function_clock)]
use crate::soc::clocks::Timg0FunctionClockConfig;
#[cfg(soc_has_clock_node_timg0_wdt_clock)]
use crate::soc::clocks::Timg0WdtClockConfig;
use crate::{
    asynch::AtomicWaker,
    interrupt::{self, InterruptConfigurable, InterruptHandler},
    pac::timg0::RegisterBlock,
    peripherals::{Interrupt, TIMG0},
    private::Sealed,
    soc::clocks::ClockTree,
    system::PeripheralClockControl,
    time::{Duration, Instant, Rate},
};

const NUM_TIMG: usize = 1 + cfg!(timergroup_timg1) as usize;

cfg_if::cfg_if! {
    // We need no locks when a TIMG has a single timer, and we don't need locks for ESP32
    // and S2 where the effective interrupt enable register (config) is not shared between
    // the timers.
    if #[cfg(all(timergroup_timg_has_timer1, not(any(esp32, esp32s2))))] {
        use esp_sync::RawMutex;
        static INT_ENA_LOCK: [RawMutex; NUM_TIMG] = [const { RawMutex::new() }; NUM_TIMG];
    }
}

#[procmacros::doc_replace(
    "timers" => {
        cfg(timergroup_timg_has_timer1) => "2 timers",
        _ => "a general purpose timer",
    }
)]
/// A timer group consisting of
/// # {timers}
/// and a watchdog timer.
pub struct TimerGroup<'d, T>
where
    T: TimerGroupInstance + 'd,
{
    _timer_group: PhantomData<T>,
    /// Timer 0
    pub timer0: Timer<'d>,
    /// Timer 1
    #[cfg(timergroup_timg_has_timer1)]
    pub timer1: Timer<'d>,
    /// Watchdog timer
    pub wdt: Wdt<T>,
}

#[doc(hidden)]
pub trait TimerGroupInstance {
    fn id() -> u8;
    fn register_block() -> *const RegisterBlock;
    #[cfg(soc_has_clock_node_timg0_function_clock)]
    fn configure_src_clk(src: Timg0FunctionClockConfig);
    fn enable_peripheral();
    fn reset_peripheral();
    #[cfg(soc_has_clock_node_timg0_wdt_clock)]
    fn configure_wdt_src_clk(src: Timg0WdtClockConfig);
    #[cfg(soc_has_clock_node_timg0_wdt_clock)]
    fn gate_wdt_src_clk(enable: bool);
    fn wdt_src_frequency() -> Rate;
    fn wdt_interrupt() -> Interrupt;
}

#[cfg(timergroup_timg0)]
impl TimerGroupInstance for TIMG0<'_> {
    fn id() -> u8 {
        0
    }

    #[inline(always)]
    fn register_block() -> *const RegisterBlock {
        Self::regs()
    }

    #[cfg(soc_has_clock_node_timg0_function_clock)]
    fn configure_src_clk(src: Timg0FunctionClockConfig) {
        crate::soc::clocks::ClockTree::with(|clocks| {
            crate::soc::clocks::configure_timg0_function_clock(clocks, src);
            crate::soc::clocks::request_timg0_function_clock(clocks);
        });
    }

    fn enable_peripheral() {
        PeripheralClockControl::enable(crate::system::Peripheral::Timg0);
    }

    fn reset_peripheral() {
        // FIXME: for TIMG0 do nothing for now because the reset breaks
        // `time::Instant::now`
    }

    #[cfg(soc_has_clock_node_timg0_wdt_clock)]
    fn configure_wdt_src_clk(src: Timg0WdtClockConfig) {
        crate::soc::clocks::ClockTree::with(|clocks| {
            crate::soc::clocks::configure_timg0_wdt_clock(clocks, src)
        });
    }

    #[cfg(soc_has_clock_node_timg0_wdt_clock)]
    fn gate_wdt_src_clk(enable: bool) {
        crate::soc::clocks::ClockTree::with(|clocks| {
            if enable {
                crate::soc::clocks::request_timg0_wdt_clock(clocks)
            } else {
                crate::soc::clocks::release_timg0_wdt_clock(clocks)
            }
        });
    }

    fn wdt_src_frequency() -> Rate {
        crate::soc::clocks::ClockTree::with(|clocks| {
            cfg_if::cfg_if! {
                if #[cfg(soc_has_clock_node_timg0_wdt_clock)] {
                    Rate::from_hz(crate::soc::clocks::timg0_wdt_clock_frequency(clocks))
                } else {
                    Rate::from_hz(crate::soc::clocks::apb_clk_frequency(clocks))
                }
            }
        })
    }

    fn wdt_interrupt() -> Interrupt {
        Interrupt::TG0_WDT_LEVEL
    }
}

#[cfg(timergroup_timg1)]
impl TimerGroupInstance for crate::peripherals::TIMG1<'_> {
    fn id() -> u8 {
        1
    }

    #[inline(always)]
    fn register_block() -> *const RegisterBlock {
        Self::regs()
    }

    #[cfg(soc_has_clock_node_timg0_function_clock)]
    fn configure_src_clk(src: Timg0FunctionClockConfig) {
        crate::soc::clocks::ClockTree::with(|clocks| {
            crate::soc::clocks::configure_timg1_function_clock(clocks, src);
            crate::soc::clocks::request_timg1_function_clock(clocks);
        });
    }

    fn enable_peripheral() {
        PeripheralClockControl::enable(crate::system::Peripheral::Timg1);
    }

    fn reset_peripheral() {
        PeripheralClockControl::reset(crate::system::Peripheral::Timg1);
    }

    #[cfg(soc_has_clock_node_timg0_wdt_clock)]
    fn configure_wdt_src_clk(src: Timg0WdtClockConfig) {
        crate::soc::clocks::ClockTree::with(|clocks| {
            crate::soc::clocks::configure_timg1_wdt_clock(clocks, src)
        });
    }

    #[cfg(soc_has_clock_node_timg0_wdt_clock)]
    fn gate_wdt_src_clk(enable: bool) {
        crate::soc::clocks::ClockTree::with(|clocks| {
            if enable {
                crate::soc::clocks::request_timg1_wdt_clock(clocks)
            } else {
                crate::soc::clocks::release_timg1_wdt_clock(clocks)
            }
        });
    }

    fn wdt_src_frequency() -> Rate {
        crate::soc::clocks::ClockTree::with(|clocks| {
            cfg_if::cfg_if! {
                if #[cfg(soc_has_clock_node_timg1_wdt_clock)] {
                    Rate::from_hz(crate::soc::clocks::timg1_wdt_clock_frequency(clocks))
                } else {
                    Rate::from_hz(crate::soc::clocks::apb_clk_frequency(clocks))
                }
            }
        })
    }

    fn wdt_interrupt() -> Interrupt {
        Interrupt::TG1_WDT_LEVEL
    }
}

impl<'d, T> TimerGroup<'d, T>
where
    T: TimerGroupInstance + 'd,
{
    /// Construct a new instance of [`TimerGroup`] in blocking mode
    pub fn new(_timer_group: T) -> Self {
        T::reset_peripheral();
        T::enable_peripheral();

        #[cfg(soc_has_clock_node_timg0_function_clock)]
        T::configure_src_clk(Timg0FunctionClockConfig::default());

        Self {
            _timer_group: PhantomData,
            timer0: Timer {
                timer: TimerId::Timer0,
                tg: T::id(),
                register_block: T::register_block(),
                _lifetime: PhantomData,
            },
            #[cfg(timergroup_timg_has_timer1)]
            timer1: Timer {
                timer: TimerId::Timer1,
                tg: T::id(),
                register_block: T::register_block(),
                _lifetime: PhantomData,
            },
            wdt: Wdt::new(),
        }
    }
}

impl super::Timer for Timer<'_> {
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

    fn async_interrupt_handler(&self) -> InterruptHandler {
        match (self.timer_group(), self.timer_number()) {
            (0, 0) => asynch::timg0_timer0_handler,
            #[cfg(timergroup_timg_has_timer1)]
            (0, 1) => asynch::timg0_timer1_handler,
            #[cfg(timergroup_timg1)]
            (1, 0) => asynch::timg1_timer0_handler,
            #[cfg(all(timergroup_timg_has_timer1, timergroup_timg1))]
            (1, 1) => asynch::timg1_timer1_handler,
            _ => unreachable!(),
        }
    }

    fn peripheral_interrupt(&self) -> Interrupt {
        match (self.timer_group(), self.timer_number()) {
            (0, 0) => Interrupt::TG0_T0_LEVEL,
            #[cfg(timergroup_timg_has_timer1)]
            (0, 1) => Interrupt::TG0_T1_LEVEL,
            #[cfg(timergroup_timg1)]
            (1, 0) => Interrupt::TG1_T0_LEVEL,
            #[cfg(all(timergroup_timg_has_timer1, timergroup_timg1))]
            (1, 1) => Interrupt::TG1_T1_LEVEL,
            _ => unreachable!(),
        }
    }

    fn set_interrupt_handler(&self, handler: InterruptHandler) {
        self.set_interrupt_handler(handler)
    }

    fn waker(&self) -> &AtomicWaker {
        asynch::waker(self)
    }
}

/// A timer within a Timer Group.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Timer<'d> {
    register_block: *const RegisterBlock,
    _lifetime: PhantomData<&'d mut ()>,
    timer: TimerId,
    tg: u8,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
enum TimerId {
    Timer0,
    #[cfg(timergroup_timg_has_timer1)]
    Timer1,
}

impl Sealed for Timer<'_> {}
unsafe impl Send for Timer<'_> {}

/// Timer peripheral instance
impl Timer<'_> {
    /// Unsafely clone this peripheral reference.
    ///
    /// # Safety
    ///
    /// You must ensure that you're only using one instance of this type at a
    /// time.
    pub unsafe fn clone_unchecked(&self) -> Self {
        Self {
            register_block: self.register_block,
            timer: self.timer,
            tg: self.tg,
            _lifetime: PhantomData,
        }
    }

    /// Creates a new peripheral reference with a shorter lifetime.
    ///
    /// Use this method if you would like to keep working with the peripheral
    /// after you dropped the driver that consumes this.
    ///
    /// See [Peripheral singleton] section for more information.
    ///
    /// [Peripheral singleton]: crate#peripheral-singletons
    pub fn reborrow(&mut self) -> Timer<'_> {
        unsafe { self.clone_unchecked() }
    }

    pub(crate) fn set_interrupt_handler(&self, handler: InterruptHandler) {
        let interrupt = match (self.timer_group(), self.timer_number()) {
            (0, 0) => Interrupt::TG0_T0_LEVEL,
            #[cfg(timergroup_timg_has_timer1)]
            (0, 1) => Interrupt::TG0_T1_LEVEL,
            #[cfg(timergroup_timg1)]
            (1, 0) => Interrupt::TG1_T0_LEVEL,
            #[cfg(all(timergroup_timg_has_timer1, timergroup_timg1))]
            (1, 1) => Interrupt::TG1_T1_LEVEL,
            _ => unreachable!(),
        };

        for core in crate::system::Cpu::other() {
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
        self.timer as u8
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

    fn source_frequency(&self) -> Rate {
        let hz = ClockTree::with(|clocks| {
            cfg_if::cfg_if! {
                if #[cfg(soc_has_clock_node_timg0_function_clock)] {
                    match self.timer_group() {
                        0 => crate::soc::clocks::timg0_function_clock_frequency(clocks),
                        #[cfg(soc_has_clock_node_timg1_function_clock)]
                        1 => crate::soc::clocks::timg1_function_clock_frequency(clocks),
                        _ => unreachable!()
                    }
                } else {
                    crate::soc::clocks::apb_clk_frequency(clocks)
                }
            }
        });
        Rate::from_hz(hz)
    }

    fn load_value(&self, value: Duration) -> Result<(), Error> {
        let clk_src = self.source_frequency();
        let Some(ticks) = timeout_to_ticks(value, clk_src, self.divider()) else {
            return Err(Error::InvalidTimeout);
        };

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
            .write(|w| w.t(self.timer as _).clear_bit_by_one());
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
        let clk_src = self.source_frequency();
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
            .t(self.timer as _)
            .bit_is_set()
    }

    fn set_interrupt_enabled(&self, state: bool) {
        cfg_if::cfg_if! {
            if #[cfg(any(esp32, esp32s2))] {
                // On ESP32 and S2, the `int_ena` register is ineffective - interrupts fire even
                // without int_ena enabling them. We use level interrupts so that we have a status
                // bit available.
                self.t()
                    .config()
                    .modify(|_, w| w.level_int_en().bit(state));
            } else if #[cfg(timergroup_timg_has_timer1)] {
                INT_ENA_LOCK[self.timer_group() as usize].lock(|| {
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

fn timeout_to_ticks(timeout: Duration, clock: Rate, divider: u32) -> Option<u64> {
    let micros = timeout.as_micros();
    let ticks_per_sec = (clock.as_hz() / divider) as u64;

    micros.checked_mul(ticks_per_sec).map(|n| n / 1_000_000)
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
        let mut this = Self {
            phantom: PhantomData,
        };

        this.set_write_protection(false);
        #[cfg(soc_has_clock_node_timg0_wdt_clock)]
        TG::configure_wdt_src_clk(Timg0WdtClockConfig::default());
        this.set_write_protection(true);

        this
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

        #[cfg(soc_has_clock_node_timg0_wdt_clock)]
        if enabled {
            TG::gate_wdt_src_clk(true);
        }

        reg_block
            .wdtconfig0()
            .modify(|_, w| w.wdt_en().bit(enabled));
        if enabled {
            reg_block.wdtconfig0().modify(|_, w| unsafe {
                w.wdt_flashboot_mod_en().bit(false);
                w.wdt_stg0().bits(MwdtStageAction::ResetSystem as u8);
                w.wdt_cpu_reset_length().bits(7);
                w.wdt_sys_reset_length().bits(7);
                w.wdt_stg1().bits(MwdtStageAction::Off as u8);
                w.wdt_stg2().bits(MwdtStageAction::Off as u8);
                w.wdt_stg3().bits(MwdtStageAction::Off as u8)
            });

            #[cfg(any(esp32c2, esp32c3, esp32c6))]
            reg_block
                .wdtconfig0()
                .modify(|_, w| w.wdt_conf_update_en().set_bit());
        }

        #[cfg(soc_has_clock_node_timg0_wdt_clock)]
        if !enabled {
            TG::gate_wdt_src_clk(false);
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
        let clk_src = TG::wdt_src_frequency();
        let timeout_ticks = timeout.as_micros() * clk_src.as_mhz() as u64;

        let reg_block = unsafe { &*TG::register_block() };

        let (prescaler, timeout) = if timeout_ticks > u32::MAX as u64 {
            let prescaler = timeout_ticks
                .div_ceil(u32::MAX as u64 + 1)
                .min(u16::MAX as u64) as u16;
            let timeout = timeout_ticks
                .div_ceil(prescaler as u64)
                .min(u32::MAX as u64);
            (prescaler, timeout as u32)
        } else {
            (1, timeout_ticks as u32)
        };

        self.set_write_protection(false);

        reg_block.wdtconfig1().write(|w| unsafe {
            #[cfg(timergroup_timg_has_divcnt_rst)]
            w.wdt_divcnt_rst().set_bit();
            w.wdt_clk_prescale().bits(prescaler)
        });

        let config_register = match stage {
            MwdtStage::Stage0 => reg_block.wdtconfig2(),
            MwdtStage::Stage1 => reg_block.wdtconfig3(),
            MwdtStage::Stage2 => reg_block.wdtconfig4(),
            MwdtStage::Stage3 => reg_block.wdtconfig5(),
        };

        config_register.write(|w| unsafe { w.hold().bits(timeout) });

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

        reg_block.wdtconfig0().modify(|_, w| unsafe {
            match stage {
                MwdtStage::Stage0 => w.wdt_stg0().bits(action as u8),
                MwdtStage::Stage1 => w.wdt_stg1().bits(action as u8),
                MwdtStage::Stage2 => w.wdt_stg2().bits(action as u8),
                MwdtStage::Stage3 => w.wdt_stg3().bits(action as u8),
            }
        });

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
    use procmacros::handler;

    use super::*;
    use crate::asynch::AtomicWaker;

    const NUM_WAKERS: usize = {
        let timer_per_group = 1 + cfg!(timergroup_timg_has_timer1) as usize;
        NUM_TIMG * timer_per_group
    };

    static WAKERS: [AtomicWaker; NUM_WAKERS] = [const { AtomicWaker::new() }; NUM_WAKERS];

    pub(super) fn waker(timer: &Timer<'_>) -> &'static AtomicWaker {
        let index = (timer.timer_number() << 1) | timer.timer_group();
        &WAKERS[index as usize]
    }

    #[inline]
    fn handle_irq(timer: Timer<'_>) {
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
            _lifetime: PhantomData,
            timer: TimerId::Timer0,
            tg: 0,
        });
    }

    #[cfg(timergroup_timg1)]
    #[handler]
    pub(crate) fn timg1_timer0_handler() {
        handle_irq(Timer {
            register_block: TIMG1::regs(),
            _lifetime: PhantomData,
            timer: TimerId::Timer0,
            tg: 1,
        });
    }

    #[cfg(timergroup_timg_has_timer1)]
    #[handler]
    pub(crate) fn timg0_timer1_handler() {
        handle_irq(Timer {
            register_block: TIMG0::regs(),
            _lifetime: PhantomData,
            timer: TimerId::Timer1,
            tg: 0,
        });
    }

    #[cfg(all(timergroup_timg1, timergroup_timg_has_timer1))]
    #[handler]
    pub(crate) fn timg1_timer1_handler() {
        handle_irq(Timer {
            register_block: TIMG1::regs(),
            _lifetime: PhantomData,
            timer: TimerId::Timer1,
            tg: 1,
        });
    }
}

/// Event Task Matrix
#[cfg(soc_has_etm)]
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

    impl Events for Timer<'_> {
        fn on_alarm(&self) -> Event {
            Event {
                id: 48 + self.timer_group(),
            }
        }
    }

    impl Tasks for Timer<'_> {
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
