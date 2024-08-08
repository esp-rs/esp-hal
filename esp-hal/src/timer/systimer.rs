//! # System Timer (SYSTIMER)
//!
//! ## Overview
//! The System Timer is a
#![cfg_attr(esp32s2, doc = "64-bit")]
#![cfg_attr(not(esp32s2), doc = "52-bit")]
//! timer which can be used, for example, to generate tick interrupts for an
//! operating system, or simply as a general-purpose timer.
//!
//! ## Configuration
//! The timer consists of two counters, `UNIT0` and `UNIT1`. The counter values
//! can be monitored by 3 comparators, `COMP0`, `COMP1`, and `COMP2`.
//!
//! [Alarm]s can be configured in two modes: [Target] (one-shot) and [Periodic].
//!
//! ## Examples
//! ### General-purpose Timer
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::timer::systimer::SystemTimer;
//! # use esp_hal::timer::timg::TimerGroup;
//! # use crate::esp_hal::prelude::_esp_hal_timer_Timer;
//! # use esp_hal::prelude::*;
//! let systimer = SystemTimer::new(peripherals.SYSTIMER);
//!
//! // Get the current timestamp, in microseconds:
//! let now = SystemTimer::now();
//!
//! let timg0 = TimerGroup::new(
//!     peripherals.TIMG0,
//!     &clocks,
//! );
//!
//! let mut timer0 = timg0.timer0;
//! timer0.set_interrupt_handler(tg0_t0_level);
//!
//! // Wait for timeout:
//! timer0.load_value(1.secs());
//! timer0.start();
//!
//! while !timer0.is_interrupt_set() {
//!     // Wait
//! }
//! # }
//!
//!
//! # use core::cell::RefCell;
//! # use critical_section::Mutex;
//! # use procmacros::handler;
//! # use esp_hal::interrupt::InterruptHandler;
//! # use esp_hal::interrupt;
//! # use esp_hal::peripherals::TIMG0;
//! # use esp_hal::timer::timg::{Timer, Timer0};
//! # use crate::esp_hal::prelude::_esp_hal_timer_Timer;
//! # static TIMER0: Mutex<RefCell<Option<Timer<Timer0<TIMG0>, esp_hal::Blocking>>>> = Mutex::new(RefCell::new(None));
//! #[handler]
//! fn tg0_t0_level() {
//!     critical_section::with(|cs| {
//!     let mut timer0 = TIMER0.borrow_ref_mut(cs);
//!     let timer0 = timer0.as_mut().unwrap();
//!
//!     timer0.clear_interrupt();
//!
//!     // Counter value should be a very small number as the alarm triggered a
//!     // counter reload to 0 and ETM stopped the counter quickly after
//!     // esp_println::println!("counter in interrupt: {}", timer0.now());
//!     });
//! }
//! ```

use core::{
    fmt::{Debug, Formatter},
    marker::PhantomData,
    ptr::addr_of_mut,
};

use fugit::{Instant, MicrosDurationU32, MicrosDurationU64};

use super::{Error, Timer as _};
use crate::{
    interrupt::{self, InterruptHandler},
    peripheral::Peripheral,
    peripherals::{Interrupt, SYSTIMER},
    Async,
    Blocking,
    Cpu,
    InterruptConfigurable,
    Mode,
};

/// System Timer driver.
pub struct SystemTimer<'d> {
    /// Unit 0
    pub unit0: Unit<'d, 0>,

    #[cfg(not(esp32s2))]
    /// Unit 1
    pub unit1: Unit<'d, 1>,

    /// Comparator 0.
    pub comparator0: Comparator<'d, 0>,

    /// Comparator 1.
    pub comparator1: Comparator<'d, 1>,

    /// Comparator 2.
    pub comparator2: Comparator<'d, 2>,
}

impl<'d> SystemTimer<'d> {
    cfg_if::cfg_if! {
        if #[cfg(esp32s2)] {
            /// Bitmask to be applied to the raw register value.
            pub const BIT_MASK: u64 = u64::MAX;
            /// The ticks per second the underlying peripheral uses.
            pub const TICKS_PER_SECOND: u64 = 80_000_000;
            // Bitmask to be applied to the raw period register value.
            const PERIOD_MASK: u64 = 0x1FFF_FFFF;
        } else {
            /// Bitmask to be applied to the raw register value.
            pub const BIT_MASK: u64 = 0xF_FFFF_FFFF_FFFF;
            /// The ticks per second the underlying peripheral uses.
            pub const TICKS_PER_SECOND: u64 = 16_000_000;
            // Bitmask to be applied to the raw period register value.
            const PERIOD_MASK: u64 = 0x3FF_FFFF;
        }
    }

    /// Create a new instance.
    pub fn new(_systimer: impl Peripheral<P = SYSTIMER> + 'd) -> Self {
        #[cfg(soc_etm)]
        etm::enable_etm();

        Self {
            unit0: Unit::new(),
            #[cfg(not(esp32s2))]
            unit1: Unit::new(),
            comparator0: Comparator::new(),
            comparator1: Comparator::new(),
            comparator2: Comparator::new(),
        }
    }

    /// Get the current count of Unit 0 in the System Timer.
    pub fn now() -> u64 {
        // This should be safe to access from multiple contexts
        // worst case scenario the second accessor ends up reading
        // an older time stamp

        let unit = unsafe { Unit::<'_, 0>::conjure() };

        unit.update();
        loop {
            if let Some(value) = unit.poll_count() {
                break value;
            }
        }
    }
}

impl SystemTimer<'static> {
    /// Split the System Timer into three alarms.
    ///
    /// This is a convenience method to create `'static` alarms of the same
    /// type. You are encouraged to use [Alarm::new] over this very specific
    /// helper.
    pub fn split<MODE>(self) -> SysTimerAlarms<MODE, Blocking> {
        static mut UNIT0: Option<Unit<'static, 0>> = None;
        let unit0 = unsafe { &mut *addr_of_mut!(UNIT0) };

        let unit0 = unit0.insert(self.unit0);
        let unit = FrozenUnit::new(unit0);

        SysTimerAlarms {
            alarm0: Alarm::new(self.comparator0, &unit),
            alarm1: Alarm::new(self.comparator1, &unit),
            alarm2: Alarm::new(self.comparator2, &unit),
            #[cfg(not(esp32s2))]
            unit1: self.unit1,
        }
    }

    /// Split the System Timer into three alarms.
    ///
    /// This is a convenience method to create `'static` alarms of the same
    /// type. You are encouraged to use [Alarm::new_async] over this very
    /// specific helper.
    pub fn split_async<MODE>(self) -> SysTimerAlarms<MODE, Async> {
        static mut UNIT0: Option<Unit<'static, 0>> = None;
        let unit0 = unsafe { &mut *addr_of_mut!(UNIT0) };

        let unit0 = unit0.insert(self.unit0);
        let unit = FrozenUnit::new(unit0);

        SysTimerAlarms {
            alarm0: Alarm::new_async(self.comparator0, &unit),
            alarm1: Alarm::new_async(self.comparator1, &unit),
            alarm2: Alarm::new_async(self.comparator2, &unit),
            #[cfg(not(esp32s2))]
            unit1: self.unit1,
        }
    }
}

/// A
#[cfg_attr(esp32s2, doc = "64-bit")]
#[cfg_attr(not(esp32s2), doc = "52-bit")]
/// counter.
#[derive(Debug)]
pub struct Unit<'d, const CHANNEL: u8>(PhantomData<&'d ()>);

impl<'d, const CHANNEL: u8> Unit<'d, CHANNEL> {
    fn new() -> Self {
        Self(PhantomData)
    }

    #[cfg(not(esp32s2))]
    /// Configures when this counter can run.
    /// It can be configured to stall or continue running when CPU stalls
    /// or enters on-chip-debugging mode
    pub fn configure(&self, config: UnitConfig) {
        let systimer = unsafe { &*SYSTIMER::ptr() };
        let conf = systimer.conf();

        critical_section::with(|_| {
            conf.modify(|_, w| match config {
                UnitConfig::Disabled => match CHANNEL {
                    0 => w.timer_unit0_work_en().clear_bit(),
                    1 => w.timer_unit1_work_en().clear_bit(),
                    _ => unreachable!(),
                },
                UnitConfig::DisabledIfCpuIsStalled(cpu) => match CHANNEL {
                    0 => w
                        .timer_unit0_work_en()
                        .set_bit()
                        .timer_unit0_core0_stall_en()
                        .bit(cpu == Cpu::ProCpu)
                        .timer_unit0_core1_stall_en()
                        .bit(cpu != Cpu::ProCpu),
                    1 => w
                        .timer_unit1_work_en()
                        .set_bit()
                        .timer_unit1_core0_stall_en()
                        .bit(cpu == Cpu::ProCpu)
                        .timer_unit1_core1_stall_en()
                        .bit(cpu != Cpu::ProCpu),
                    _ => unreachable!(),
                },
                UnitConfig::Enabled => match CHANNEL {
                    0 => w
                        .timer_unit0_work_en()
                        .set_bit()
                        .timer_unit0_core0_stall_en()
                        .clear_bit()
                        .timer_unit0_core1_stall_en()
                        .clear_bit(),
                    1 => w
                        .timer_unit1_work_en()
                        .set_bit()
                        .timer_unit1_core0_stall_en()
                        .clear_bit()
                        .timer_unit1_core1_stall_en()
                        .clear_bit(),
                    _ => unreachable!(),
                },
            });
        });
    }

    /// Set the value of the counter immediately. If the unit is at work,
    /// the counter will continue to count up from the new reloaded value.
    ///
    /// This can be used to load back the sleep time recorded by RTC timer
    /// via software after Light-sleep
    pub fn set_count(&self, value: u64) {
        let systimer = unsafe { &*SYSTIMER::ptr() };
        #[cfg(not(esp32s2))]
        {
            let unitload = systimer.unitload(CHANNEL as _);
            let unit_load = systimer.unit_load(CHANNEL as _);

            unitload.hi().write(|w| w.load_hi().set((value << 32) as _));
            unitload
                .lo()
                .write(|w| w.load_lo().set((value & 0xFFFF_FFFF) as _));

            unit_load.write(|w| w.load().set_bit());
        }
        #[cfg(esp32s2)]
        {
            systimer
                .load_hi()
                .write(|w| w.load_hi().set((value << 32) as _));
            systimer
                .load_lo()
                .write(|w| w.load_lo().set((value & 0xFFFF_FFFF) as _));

            systimer.load().write(|w| w.load().set_bit());
        }
    }

    /// Update the value returned by [Self::poll_count] to be the current value
    /// of the counter.
    ///
    /// This can be used to read the current value of the timer.
    pub fn update(&self) {
        let systimer = unsafe { &*SYSTIMER::ptr() };
        systimer
            .unit_op(CHANNEL as _)
            .modify(|_, w| w.update().set_bit());
    }

    /// Return the count value at the time of the last call to [Self::update].
    ///
    /// Returns None if the update isn't ready to read if update has never been
    /// called.
    pub fn poll_count(&self) -> Option<u64> {
        let systimer = unsafe { &*SYSTIMER::ptr() };
        if systimer
            .unit_op(CHANNEL as _)
            .read()
            .value_valid()
            .bit_is_set()
        {
            let unit_value = systimer.unit_value(CHANNEL as _);

            let lo = unit_value.lo().read().bits();
            let hi = unit_value.hi().read().bits();

            Some(((hi as u64) << 32) | lo as u64)
        } else {
            None
        }
    }

    /// Convenience method to call [Self::update] and [Self::poll_count].
    pub fn read_count(&self) -> u64 {
        // This can be a shared reference as long as this type isn't Sync.

        self.update();
        loop {
            if let Some(count) = self.poll_count() {
                break count;
            }
        }
    }
}

/// A comparator that can generate alarms/interrupts based on values of a unit.
#[derive(Debug)]
pub struct Comparator<'d, const CHANNEL: u8>(PhantomData<&'d ()>);

impl<'d, const CHANNEL: u8> Comparator<'d, CHANNEL> {
    fn new() -> Self {
        Self(PhantomData)
    }

    /// Enables/disables the comparator. If enabled, this means
    /// it will generate interrupt based on its configuration.
    pub fn set_enable(&self, enable: bool) {
        let systimer = unsafe { &*SYSTIMER::ptr() };

        #[cfg(not(esp32s2))]
        critical_section::with(|_| {
            systimer.conf().modify(|_, w| match CHANNEL {
                0 => w.target0_work_en().bit(enable),
                1 => w.target1_work_en().bit(enable),
                2 => w.target2_work_en().bit(enable),
                _ => unreachable!(),
            });
        });

        #[cfg(esp32s2)]
        systimer
            .target_conf(CHANNEL as usize)
            .modify(|_r, w| w.work_en().bit(enable));
    }

    /// Returns true if the comparator has been enabled. This means
    /// it will generate interrupt based on its configuration.
    pub fn is_enabled(&self) -> bool {
        #[cfg(not(esp32s2))]
        {
            let systimer = unsafe { &*SYSTIMER::ptr() };
            let conf = systimer.conf().read();
            match CHANNEL {
                0 => conf.target0_work_en().bit(),
                1 => conf.target1_work_en().bit(),
                2 => conf.target2_work_en().bit(),
                _ => unreachable!(),
            }
        }

        #[cfg(esp32s2)]
        {
            let tconf = unsafe {
                let systimer = &*SYSTIMER::ptr();
                systimer.target_conf(CHANNEL as usize)
            };
            tconf.read().work_en().bit()
        }
    }

    /// Sets the unit this comparator uses as a reference count.
    #[cfg(not(esp32s2))]
    pub fn set_unit(&self, is_unit0: bool) {
        let tconf = unsafe {
            let systimer = &*SYSTIMER::ptr();
            systimer.target_conf(CHANNEL as usize)
        };
        tconf.modify(|_, w| w.timer_unit_sel().bit(is_unit0));
    }

    /// Set the mode of the comparator to be either target or periodic.
    pub fn set_mode(&self, mode: ComparatorMode) {
        let tconf = unsafe {
            let systimer = &*SYSTIMER::ptr();
            systimer.target_conf(CHANNEL as usize)
        };
        let is_period_mode = match mode {
            ComparatorMode::Period => true,
            ComparatorMode::Target => false,
        };
        tconf.modify(|_, w| w.period_mode().bit(is_period_mode));
    }

    /// Get the current mode of the comparator, which is either target or
    /// periodic.
    pub fn get_mode(&self) -> ComparatorMode {
        let tconf = unsafe {
            let systimer = &*SYSTIMER::ptr();
            systimer.target_conf(CHANNEL as usize)
        };
        if tconf.read().period_mode().bit() {
            ComparatorMode::Period
        } else {
            ComparatorMode::Target
        }
    }

    /// Set how often the comparator should generate an interrupt when in
    /// periodic mode.
    pub fn set_period(&self, value: u32) {
        unsafe {
            let systimer = &*SYSTIMER::ptr();
            let tconf = systimer.target_conf(CHANNEL as usize);
            tconf.modify(|_, w| w.period().bits(value));
            #[cfg(not(esp32s2))]
            {
                let comp_load = systimer.comp_load(CHANNEL as usize);
                comp_load.write(|w| w.load().set_bit());
            }
        }
    }

    /// Set when the comparator should generate an interrupt in target mode.
    pub fn set_target(&self, value: u64) {
        let systimer = unsafe { &*SYSTIMER::ptr() };
        let target = systimer.trgt(CHANNEL as usize);
        target.hi().write(|w| w.hi().set((value >> 32) as u32));
        target
            .lo()
            .write(|w| w.lo().set((value & 0xFFFF_FFFF) as u32));
        #[cfg(not(esp32s2))]
        {
            let comp_load = systimer.comp_load(CHANNEL as usize);
            comp_load.write(|w| w.load().set_bit());
        }
    }

    /// Get the actual target value of the comparator.
    pub fn get_actual_target(&self) -> u64 {
        let target = unsafe {
            let systimer = &*SYSTIMER::ptr();
            systimer.trgt(CHANNEL as usize)
        };
        let hi = target.hi().read().hi().bits();
        let lo = target.lo().read().lo().bits();

        ((hi as u64) << 32) | (lo as u64)
    }

    /// Set the interrupt handler for this comparator.
    pub fn set_interrupt_handler(&self, handler: InterruptHandler) {
        let interrupt = match CHANNEL {
            0 => Interrupt::SYSTIMER_TARGET0,
            1 => Interrupt::SYSTIMER_TARGET1,
            2 => Interrupt::SYSTIMER_TARGET2,
            _ => unreachable!(),
        };
        unsafe {
            interrupt::bind_interrupt(interrupt, handler.handler());
        }
        unwrap!(interrupt::enable(interrupt, handler.priority()));
    }
}

/// The configuration of a unit.
#[derive(Copy, Clone)]
pub enum UnitConfig {
    /// Unit is not counting.
    Disabled,

    /// Unit is counting unless the Cpu is stalled.
    DisabledIfCpuIsStalled(Cpu),

    /// Unit is counting.
    Enabled,
}

/// The modes of a comparator.
#[derive(Copy, Clone)]
pub enum ComparatorMode {
    /// The comparator will generate interrupts periodically.
    Period,

    /// The comparator will generate an interrupt when the unit reaches the
    /// target.
    Target,
}

impl Unit<'static, 0> {
    /// Conjure a system timer unit out of thin air.
    ///
    /// # Safety
    ///
    /// Users must take care to ensure that only one reference to the unit is
    /// in scope at any given time.
    pub const unsafe fn conjure() -> Self {
        Self(PhantomData)
    }
}

#[cfg(not(esp32s2))]
impl Unit<'static, 1> {
    /// Conjure a system timer unit out of thin air.
    ///
    /// # Safety
    ///
    /// Users must take care to ensure that only one reference to the unit is
    /// in scope at any given time.
    pub const unsafe fn conjure() -> Self {
        Self(PhantomData)
    }
}

/// A unit whose value cannot be updated.
pub struct FrozenUnit<'d, const CHANNEL: u8>(&'d Unit<'d, CHANNEL>);

impl<'d, const CHANNEL: u8> FrozenUnit<'d, CHANNEL> {
    /// Creates a frozen unit. You will no longer be allowed
    /// direct access to this unit until all the alarms created
    /// from the unit are dropped.
    pub fn new(unit: &'d mut Unit<'d, CHANNEL>) -> Self {
        Self(unit)
    }

    fn borrow(&self) -> &'d Unit<'d, CHANNEL> {
        self.0
    }
}

/// Alarms created from the System Timer peripheral.
pub struct SysTimerAlarms<MODE, DM: Mode> {
    /// Alarm 0
    pub alarm0: Alarm<'static, MODE, DM, 0, 0>,
    /// Alarm 1
    pub alarm1: Alarm<'static, MODE, DM, 1, 0>,
    /// Alarm 2
    pub alarm2: Alarm<'static, MODE, DM, 2, 0>,

    /// Unit 1
    ///
    /// Leftover unit which wasn't used to create the three alarms.
    #[cfg(not(esp32s2))]
    pub unit1: Unit<'static, 1>,
}

/// A marker for a [Alarm] in target mode.
#[derive(Debug)]
pub struct Target;

/// A marker for a [Alarm] in periodic mode.
#[derive(Debug)]
pub struct Periodic;

/// A single alarm.
pub struct Alarm<'d, MODE, DM, const COMP: u8, const UNIT: u8>
where
    DM: Mode,
{
    comparator: Comparator<'d, COMP>,
    unit: &'d Unit<'d, UNIT>,
    _pd: PhantomData<(MODE, DM)>,
}

impl<'d, T, DM, const COMP: u8, const UNIT: u8> Debug for Alarm<'d, T, DM, COMP, UNIT>
where
    DM: Mode,
{
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Alarm")
            .field("comparator", &COMP)
            .field("unit", &UNIT)
            .finish()
    }
}

impl<'d, T, const COMP: u8, const UNIT: u8> Alarm<'d, T, Blocking, COMP, UNIT> {
    /// Creates a new alarm from a comparator and unit, in blocking mode.
    pub fn new(comparator: Comparator<'d, COMP>, unit: &FrozenUnit<'d, UNIT>) -> Self {
        Self {
            comparator,
            unit: unit.borrow(),
            _pd: PhantomData,
        }
    }
}

impl<'d, T, const COMP: u8, const UNIT: u8> Alarm<'d, T, Async, COMP, UNIT> {
    /// Creates a new alarm from a comparator and unit, in async mode.
    pub fn new_async(comparator: Comparator<'d, COMP>, unit: &FrozenUnit<'d, UNIT>) -> Self {
        Self {
            comparator,
            unit: unit.0,
            _pd: PhantomData,
        }
    }
}

impl<'d, T, const COMP: u8, const UNIT: u8> InterruptConfigurable
    for Alarm<'d, T, Blocking, COMP, UNIT>
{
    fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        self.comparator.set_interrupt_handler(handler)
    }
}

impl<'d, DM, const COMP: u8, const UNIT: u8> Alarm<'d, Target, DM, COMP, UNIT>
where
    DM: Mode,
{
    /// Set the target value of this [Alarm]
    pub fn set_target(&self, timestamp: u64) {
        #[cfg(esp32s2)]
        unsafe {
            let systimer = &*SYSTIMER::ptr();
            // run at XTAL freq, not 80 * XTAL freq
            systimer.step().write(|w| w.xtal_step().bits(0x1));
        }

        self.comparator.set_mode(ComparatorMode::Target);
        self.comparator.set_target(timestamp);
        self.comparator.set_enable(true);
    }

    /// Block waiting until the timer reaches the `timestamp`
    pub fn wait_until(&self, timestamp: u64) {
        self.clear_interrupt();
        self.set_target(timestamp);

        let r = unsafe { &*crate::peripherals::SYSTIMER::PTR }.int_raw();
        loop {
            if r.read().target(COMP).bit_is_set() {
                break;
            }
        }
    }

    /// Converts this [Alarm] into [Periodic] mode
    pub fn into_periodic(self) -> Alarm<'d, Periodic, DM, COMP, UNIT> {
        Alarm {
            comparator: self.comparator,
            unit: self.unit,
            _pd: PhantomData,
        }
    }
}

impl<'d, DM, const COMP: u8, const UNIT: u8> Alarm<'d, Periodic, DM, COMP, UNIT>
where
    DM: Mode,
{
    /// Set the period of this [Alarm]
    pub fn set_period(&self, period: MicrosDurationU32) {
        #[cfg(esp32s2)]
        unsafe {
            let systimer = &*SYSTIMER::ptr();
            // run at XTAL freq, not 80 * XTAL freq
            systimer.step().write(|w| w.xtal_step().bits(0x1));
        }

        let us = period.ticks();
        let ticks = us * (SystemTimer::TICKS_PER_SECOND / 1_000_000) as u32;

        self.comparator.set_mode(ComparatorMode::Period);
        self.comparator.set_period(ticks);
        self.comparator.set_enable(true);
    }

    /// Converts this [Alarm] into [Target] mode
    pub fn into_target(self) -> Alarm<'d, Target, DM, COMP, UNIT> {
        Alarm {
            comparator: self.comparator,
            unit: self.unit,
            _pd: PhantomData,
        }
    }
}

impl<'d, T, DM, const COMP: u8, const UNIT: u8> crate::private::Sealed
    for Alarm<'d, T, DM, COMP, UNIT>
where
    DM: Mode,
{
}

impl<'d, T, DM, const COMP: u8, const UNIT: u8> super::Timer for Alarm<'d, T, DM, COMP, UNIT>
where
    DM: Mode,
{
    fn start(&self) {
        self.comparator.set_enable(true);
    }

    fn stop(&self) {
        self.comparator.set_enable(false);
    }

    fn reset(&self) {
        let systimer = unsafe { &*SYSTIMER::PTR };

        #[cfg(esp32s2)]
        // Run at XTAL freq, not 80 * XTAL freq:
        systimer
            .step()
            .modify(|_, w| unsafe { w.xtal_step().bits(0x1) });

        #[cfg(not(esp32s2))]
        {
            systimer
                .conf()
                .modify(|_, w| w.timer_unit0_core0_stall_en().clear_bit());
        }
    }

    fn is_running(&self) -> bool {
        self.comparator.is_enabled()
    }

    fn now(&self) -> Instant<u64, 1, 1_000_000> {
        // This should be safe to access from multiple contexts; worst case
        // scenario the second accessor ends up reading an older time stamp.

        self.unit.update();

        let ticks = loop {
            if let Some(value) = self.unit.poll_count() {
                break value;
            }
        };

        let us = ticks / (SystemTimer::TICKS_PER_SECOND / 1_000_000);

        Instant::<u64, 1, 1_000_000>::from_ticks(us)
    }

    fn load_value(&self, value: MicrosDurationU64) -> Result<(), Error> {
        let mode = self.comparator.get_mode();

        let us = value.ticks();
        let ticks = us * (SystemTimer::TICKS_PER_SECOND / 1_000_000);

        if matches!(mode, ComparatorMode::Period) {
            // Period mode

            // The `SYSTIMER_TARGETx_PERIOD` field is 26-bits wide (or
            // 29-bits on the ESP32-S2), so we must ensure that the provided
            // value is not too wide:
            if (ticks & !SystemTimer::PERIOD_MASK) != 0 {
                return Err(Error::InvalidTimeout);
            }

            self.comparator.set_period(ticks as u32);

            // Clear and then set SYSTIMER_TARGETx_PERIOD_MODE to configure COMPx into
            // period mode
            self.comparator.set_mode(ComparatorMode::Target);
            self.comparator.set_mode(ComparatorMode::Period);
        } else {
            // Target mode

            self.unit.update();
            while self.unit.poll_count().is_none() {
                // Wait for value registers to update
            }

            // The counters/comparators are 52-bits wide (except on ESP32-S2,
            // which is 64-bits), so we must ensure that the provided value
            // is not too wide:
            #[cfg(not(esp32s2))]
            if (ticks & !SystemTimer::BIT_MASK) != 0 {
                return Err(Error::InvalidTimeout);
            }

            let v = self.unit.poll_count().unwrap();
            let t = v + ticks;

            self.comparator.set_target(t);
        }

        Ok(())
    }

    fn enable_auto_reload(&self, auto_reload: bool) {
        // If `auto_reload` is true use Period Mode, otherwise use Target Mode:
        let mode = if auto_reload {
            ComparatorMode::Period
        } else {
            ComparatorMode::Target
        };
        self.comparator.set_mode(mode)
    }

    fn enable_interrupt(&self, state: bool) {
        unsafe { &*SYSTIMER::PTR }
            .int_ena()
            .modify(|_, w| w.target(COMP).bit(state));
    }

    fn clear_interrupt(&self) {
        unsafe { &*SYSTIMER::PTR }
            .int_clr()
            .write(|w| w.target(COMP).clear_bit_by_one());
    }

    fn is_interrupt_set(&self) -> bool {
        unsafe { &*SYSTIMER::PTR }
            .int_raw()
            .read()
            .target(COMP)
            .bit_is_set()
    }

    fn set_alarm_active(&self, _active: bool) {
        // Nothing to do
    }

    fn set_interrupt_handler(&self, handler: InterruptHandler) {
        self.comparator.set_interrupt_handler(handler);
    }
}

impl<'d, T, DM, const COMP: u8, const UNIT: u8> Peripheral for Alarm<'d, T, DM, COMP, UNIT>
where
    DM: Mode,
{
    type P = Self;

    #[inline]
    unsafe fn clone_unchecked(&mut self) -> Self::P {
        core::ptr::read(self as *const _)
    }
}

// Async functionality of the system timer.
#[cfg(feature = "async")]
mod asynch {
    use core::{
        pin::Pin,
        task::{Context, Poll},
    };

    use embassy_sync::waitqueue::AtomicWaker;
    use procmacros::handler;

    use super::*;

    const NUM_ALARMS: usize = 3;

    #[allow(clippy::declare_interior_mutable_const)]
    const INIT: AtomicWaker = AtomicWaker::new();
    static WAKERS: [AtomicWaker; NUM_ALARMS] = [INIT; NUM_ALARMS];

    pub(crate) struct AlarmFuture<'a, const COMP: u8, const UNIT: u8> {
        phantom: PhantomData<&'a Alarm<'a, Periodic, crate::Async, COMP, UNIT>>,
    }

    impl<'a, const COMP: u8, const UNIT: u8> AlarmFuture<'a, COMP, UNIT> {
        pub(crate) fn new(alarm: &'a Alarm<Periodic, crate::Async, COMP, UNIT>) -> Self {
            alarm.clear_interrupt();

            let (interrupt, handler) = match COMP {
                0 => (Interrupt::SYSTIMER_TARGET0, target0_handler),
                1 => (Interrupt::SYSTIMER_TARGET1, target1_handler),
                _ => (Interrupt::SYSTIMER_TARGET2, target2_handler),
            };

            unsafe {
                interrupt::bind_interrupt(interrupt, handler.handler());
                interrupt::enable(interrupt, handler.priority()).unwrap();
            }

            alarm.enable_interrupt(true);

            Self {
                phantom: PhantomData,
            }
        }

        fn event_bit_is_clear(&self) -> bool {
            unsafe { &*crate::peripherals::SYSTIMER::PTR }
                .int_ena()
                .read()
                .target(COMP)
                .bit_is_clear()
        }
    }

    impl<'a, const COMP: u8, const UNIT: u8> core::future::Future for AlarmFuture<'a, COMP, UNIT> {
        type Output = ();

        fn poll(self: Pin<&mut Self>, ctx: &mut Context<'_>) -> Poll<Self::Output> {
            WAKERS[COMP as usize].register(ctx.waker());

            if self.event_bit_is_clear() {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        }
    }

    impl<'d, const COMP: u8, const UNIT: u8> embedded_hal_async::delay::DelayNs
        for Alarm<'d, Periodic, crate::Async, COMP, UNIT>
    {
        async fn delay_ns(&mut self, ns: u32) {
            let period = MicrosDurationU32::from_ticks(ns / 1000);
            self.set_period(period);

            AlarmFuture::new(self).await;
        }

        async fn delay_ms(&mut self, ms: u32) {
            for _ in 0..ms {
                self.delay_us(1000).await;
            }
        }
    }

    #[handler]
    fn target0_handler() {
        unsafe { &*crate::peripherals::SYSTIMER::PTR }
            .int_ena()
            .modify(|_, w| w.target0().clear_bit());

        WAKERS[0].wake();
    }

    #[handler]
    fn target1_handler() {
        unsafe { &*crate::peripherals::SYSTIMER::PTR }
            .int_ena()
            .modify(|_, w| w.target1().clear_bit());

        WAKERS[1].wake();
    }

    #[handler]
    fn target2_handler() {
        unsafe { &*crate::peripherals::SYSTIMER::PTR }
            .int_ena()
            .modify(|_, w| w.target2().clear_bit());

        WAKERS[2].wake();
    }
}

#[cfg(soc_etm)]
pub mod etm {
    //! # Event Task Matrix Function
    //!
    //! ## Overview
    //!
    //!    The system timer supports the Event Task Matrix (ETM) function, which
    //! allows the system timer’s    ETM events to trigger any
    //!    peripherals’ ETM tasks.
    //!
    //!    The system timer can generate the following ETM events:
    //!    - SYSTIMER_EVT_CNT_CMPx: Indicates the alarm pulses generated by
    //!      COMPx
    //!
    //! ## Example
    //! ```rust, no_run
    #![doc = crate::before_snippet!()]
    //! # use esp_hal::timer::systimer::{etm::SysTimerEtmEvent, SystemTimer};
    //! # use fugit::ExtU32;
    //! let syst = SystemTimer::new(peripherals.SYSTIMER);
    //! let syst_alarms = syst.split();
    //! let mut alarm0 = syst_alarms.alarm0.into_periodic();
    //! alarm0.set_period(1.secs());
    //!
    //! let timer_event = SysTimerEtmEvent::new(&mut alarm0);
    //! # }
    //! ```

    use super::*;

    /// An ETM controlled SYSTIMER event
    pub struct SysTimerEtmEvent<'a, 'd, M, DM: crate::Mode, const COMP: u8> {
        alarm: &'a mut Alarm<'d, M, DM, COMP, 0>,
    }

    impl<'a, 'd, M, DM: crate::Mode, const COMP: u8> SysTimerEtmEvent<'a, 'd, M, DM, COMP> {
        /// Creates an ETM event from the given [Alarm]
        pub fn new(alarm: &'a mut Alarm<'d, M, DM, COMP, 0>) -> Self {
            Self { alarm }
        }

        /// Execute closure f with mutable access to the wrapped [Alarm].
        pub fn with<R>(&self, f: impl FnOnce(&&'a mut Alarm<'d, M, DM, COMP, 0>) -> R) -> R {
            let alarm = &self.alarm;
            f(alarm)
        }
    }

    impl<'a, 'd, M, DM: crate::Mode, const COMP: u8> crate::private::Sealed
        for SysTimerEtmEvent<'a, 'd, M, DM, COMP>
    {
    }

    impl<'a, 'd, M, DM: crate::Mode, const COMP: u8> crate::etm::EtmEvent
        for SysTimerEtmEvent<'a, 'd, M, DM, COMP>
    {
        fn id(&self) -> u8 {
            50 + COMP
        }
    }

    pub(super) fn enable_etm() {
        let syst = unsafe { crate::peripherals::SYSTIMER::steal() };
        syst.conf().modify(|_, w| w.etm_en().set_bit());
    }
}
