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
//!
//! The timer consists of two counters, `Unit0` and `Unit1`. The counter values
//! can be monitored by 3 [`Alarm`]s
//!
//! It is recommended to pass the [`Alarm`]s into a high level driver like
//! [`OneShotTimer`](super::OneShotTimer) and
//! [`PeriodicTimer`](super::PeriodicTimer). Using the System timer directly is
//! only possible through the low level [`Timer`](crate::timer::Timer) trait.

use core::{fmt::Debug, marker::PhantomData, num::NonZeroU32};

use esp_sync::RawMutex;

use super::{Error, Timer as _};
use crate::{
    asynch::AtomicWaker,
    interrupt::{self, InterruptHandler},
    peripherals::{Interrupt, SYSTIMER},
    soc::clocks::ClockTree,
    system::{Cpu, Peripheral as PeripheralEnable, PeripheralClockControl},
    time::{Duration, Instant},
};

// System Timer is only clocked by XTAL divided by 2 or 2.5 (or RC_FAST_CLK which is not supported
// yet). Some XTAL options (most, in fact) divided by this value may not be an integer multiple of
// 1_000_000. Because the timer API works with microseconds, we need to correct for this. To avoid
// u64 division as much as possible, we use the two highest bits of the divisor to determine the
// division method.
// - If these flags are 0b00, we divide by the divisor.
// - If these flags are 0b01, we multiply by a constant before dividing by the divisor to improve
//   accuracy.
// - If these flags are 0b10, we shift the timestamp by the lower bits.
//
// Apart from the S2, the System Timer clock divider outputs a 16MHz timer clock when using
// the "default" XTAL configuration, so this method will commonly use the shifting based
// division.
//
// On a 26MHz C2, the divider outputs 10.4MHz. On a 32MHz C3, the divider outputs 12.8MHz.
//
// Time is unreliable before `init_timestamp_scaler` is called.
//
// Because a single crate version can have "rt" enabled, `ESP_HAL_SYSTIMER_CORRECTION` needs
// to be shared between versions. This aspect of this driver must be therefore kept stable.
#[unsafe(no_mangle)]
#[cfg(feature = "rt")]
static mut ESP_HAL_SYSTIMER_CORRECTION: NonZeroU32 = NonZeroU32::new(SHIFT_TIMESTAMP_FLAG).unwrap(); // Shift-by-0 = no-op

#[cfg(not(feature = "rt"))]
unsafe extern "Rust" {
    static mut ESP_HAL_SYSTIMER_CORRECTION: NonZeroU32;
}

const SHIFT_TIMESTAMP_FLAG: u32 = 0x8000_0000;
const SHIFT_MASK: u32 = 0x0000_FFFF;
// If the tick rate is not an integer number of microseconds: Since the divider is 2.5,
// and we assume XTAL is an integer number of MHz, we can multiply by 5, then divide by
// 5 to improve accuracy. On H2 the divider is 2, so we can multiply by 2, then divide by
// 2.
const UNEVEN_DIVIDER_FLAG: u32 = 0x4000_0000;
const UNEVEN_MULTIPLIER: u32 = if cfg!(esp32h2) { 2 } else { 5 };
const UNEVEN_DIVIDER_MASK: u32 = 0x0000_FFFF;

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

/// System Timer driver.
pub struct SystemTimer<'d> {
    /// Alarm 0.
    pub alarm0: Alarm<'d>,

    /// Alarm 1.
    pub alarm1: Alarm<'d>,

    /// Alarm 2.
    pub alarm2: Alarm<'d>,
}

impl<'d> SystemTimer<'d> {
    cfg_if::cfg_if! {
        if #[cfg(esp32s2)] {
            /// Bitmask to be applied to the raw register value.
            pub const BIT_MASK: u64 = u64::MAX;
            // Bitmask to be applied to the raw period register value.
            const PERIOD_MASK: u64 = 0x1FFF_FFFF;
        } else {
            /// Bitmask to be applied to the raw register value.
            pub const BIT_MASK: u64 = 0xF_FFFF_FFFF_FFFF;
            // Bitmask to be applied to the raw period register value.
            const PERIOD_MASK: u64 = 0x3FF_FFFF;
        }
    }

    /// One-time initialization for the timestamp conversion/scaling.
    #[cfg(feature = "rt")]
    pub(crate) fn init_timestamp_scaler() {
        // Maximum tick rate is 80MHz (S2), which fits in a u32, so let's narrow the type.
        let systimer_rate = Self::ticks_per_second();

        // Select the optimal way to divide timestamps.
        let packed_rate_and_method = if systimer_rate.is_multiple_of(1_000_000) {
            let ticks_per_us = systimer_rate as u32 / 1_000_000;
            if ticks_per_us.is_power_of_two() {
                // Turn the division into a shift
                SHIFT_TIMESTAMP_FLAG | (ticks_per_us.ilog2() & SHIFT_MASK)
            } else {
                // We need to divide by an integer :(
                ticks_per_us
            }
        } else {
            // The rate is not a multiple of 1 MHz, we need to scale it up to prevent precision
            // loss.
            let multiplied_ticks_per_us = (systimer_rate * UNEVEN_MULTIPLIER as u64) / 1_000_000;
            UNEVEN_DIVIDER_FLAG | (multiplied_ticks_per_us as u32)
        };

        // Safety: we only ever write ESP_HAL_SYSTIMER_CORRECTION in `init_timestamp_scaler`, which
        // is called once and only once during startup, from `time_init`.
        unsafe {
            let correction_ptr = &raw mut ESP_HAL_SYSTIMER_CORRECTION;
            *correction_ptr = unwrap!(NonZeroU32::new(packed_rate_and_method));
        }
    }

    #[inline]
    pub(crate) fn ticks_to_us(ticks: u64) -> u64 {
        // Safety: we only ever write ESP_HAL_SYSTIMER_CORRECTION in `init_timestamp_scaler`, which
        // is called once and only once during startup.
        let correction = unsafe { ESP_HAL_SYSTIMER_CORRECTION };

        let correction = correction.get();
        match correction & (SHIFT_TIMESTAMP_FLAG | UNEVEN_DIVIDER_FLAG) {
            v if v == SHIFT_TIMESTAMP_FLAG => ticks >> (correction & SHIFT_MASK),
            v if v == UNEVEN_DIVIDER_FLAG => {
                // Not only that, but we need to multiply the timestamp first otherwise
                // we'd count slower than the timer.
                let multiplied = if UNEVEN_MULTIPLIER.is_power_of_two() {
                    ticks << UNEVEN_MULTIPLIER.ilog2()
                } else {
                    ticks * UNEVEN_MULTIPLIER as u64
                };

                let divider = correction & UNEVEN_DIVIDER_MASK;
                multiplied / divider as u64
            }
            _ => ticks / correction as u64,
        }
    }

    #[inline]
    fn us_to_ticks(us: u64) -> u64 {
        // Safety: we only ever write ESP_HAL_SYSTIMER_CORRECTION in `init_timestamp_scaler`, which
        // is called once and only once during startup.
        let correction = unsafe { ESP_HAL_SYSTIMER_CORRECTION };

        let correction = correction.get();
        match correction & (SHIFT_TIMESTAMP_FLAG | UNEVEN_DIVIDER_FLAG) {
            v if v == SHIFT_TIMESTAMP_FLAG => us << (correction & SHIFT_MASK),
            v if v == UNEVEN_DIVIDER_FLAG => {
                let multiplier = correction & UNEVEN_DIVIDER_MASK;
                let multiplied = us * multiplier as u64;

                // Not only that, but we need to divide the timestamp first otherwise
                // we'd return a slightly too-big value.
                if UNEVEN_MULTIPLIER.is_power_of_two() {
                    multiplied >> UNEVEN_MULTIPLIER.ilog2()
                } else {
                    multiplied / UNEVEN_MULTIPLIER as u64
                }
            }
            _ => us * correction as u64,
        }
    }

    /// Returns the tick frequency of the underlying timer unit.
    #[inline]
    pub fn ticks_per_second() -> u64 {
        // FIXME: this requires a critical section. We can probably do better, if we can formulate
        // invariants well.
        ClockTree::with(|clocks| {
            cfg_if::cfg_if! {
                if #[cfg(esp32s2)] {
                    crate::soc::clocks::apb_clk_frequency(clocks) as u64
                } else if #[cfg(esp32h2)] {
                    // The counters and comparators are driven using `XTAL_CLK`.
                    // The average clock frequency is fXTAL_CLK/2, (16 MHz for XTAL = 32 MHz)
                    (crate::soc::clocks::xtal_clk_frequency(clocks) / 2) as u64
                } else {
                    // The counters and comparators are driven using `XTAL_CLK`.
                    // The average clock frequency is fXTAL_CLK/2.5 (16 MHz for XTAL = 40 MHz)
                    (crate::soc::clocks::xtal_clk_frequency(clocks) * 10 / 25) as u64
                }
            }
        })
    }

    /// Create a new instance.
    pub fn new(_systimer: SYSTIMER<'d>) -> Self {
        // Don't reset Systimer as it will break `time::Instant::now`, only enable it
        PeripheralClockControl::enable(PeripheralEnable::Systimer);

        #[cfg(soc_has_etm)]
        etm::enable_etm();

        Self {
            alarm0: Alarm::new(Comparator::Comparator0),
            alarm1: Alarm::new(Comparator::Comparator1),
            alarm2: Alarm::new(Comparator::Comparator2),
        }
    }

    /// Get the current count of the given unit in the System Timer.
    #[inline]
    pub fn unit_value(unit: Unit) -> u64 {
        // This should be safe to access from multiple contexts
        // worst case scenario the second accessor ends up reading
        // an older time stamp

        unit.read_count()
    }

    #[cfg(not(esp32s2))]
    /// Configures when this counter can run.
    /// It can be configured to stall or continue running when CPU stalls
    /// or enters on-chip-debugging mode.
    ///
    /// # Safety
    ///
    /// - Disabling a `Unit` whilst [`Alarm`]s are using it will affect the [`Alarm`]s operation.
    /// - Disabling Unit0 will affect [`Instant::now`].
    pub unsafe fn configure_unit(unit: Unit, config: UnitConfig) {
        unit.configure(config)
    }

    /// Set the value of the counter immediately. If the unit is at work,
    /// the counter will continue to count up from the new reloaded value.
    ///
    /// This can be used to load back the sleep time recorded by RTC timer
    /// via software after Light-sleep
    ///
    /// # Safety
    ///
    /// - Modifying a unit's count whilst [`Alarm`]s are using it may cause unexpected behaviour
    /// - Any modification of the unit0 count will affect [`Instant::now`]
    pub unsafe fn set_unit_value(unit: Unit, value: u64) {
        unit.set_count(value)
    }
}

/// A
#[cfg_attr(esp32s2, doc = "64-bit")]
#[cfg_attr(not(esp32s2), doc = "52-bit")]
/// counter.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Unit {
    /// Unit 0
    Unit0 = 0,
    #[cfg(not(esp32s2))]
    /// Unit 1
    Unit1 = 1,
}

impl Unit {
    #[inline]
    fn channel(&self) -> u8 {
        *self as _
    }

    #[cfg(not(esp32s2))]
    fn configure(&self, config: UnitConfig) {
        CONF_LOCK.lock(|| {
            SYSTIMER::regs().conf().modify(|_, w| match config {
                UnitConfig::Disabled => match self.channel() {
                    0 => w.timer_unit0_work_en().clear_bit(),
                    1 => w.timer_unit1_work_en().clear_bit(),
                    _ => unreachable!(),
                },
                UnitConfig::DisabledIfCpuIsStalled(cpu) => match self.channel() {
                    0 => {
                        w.timer_unit0_work_en().set_bit();
                        w.timer_unit0_core0_stall_en().bit(cpu == Cpu::ProCpu);
                        w.timer_unit0_core1_stall_en().bit(cpu != Cpu::ProCpu)
                    }
                    1 => {
                        w.timer_unit1_work_en().set_bit();
                        w.timer_unit1_core0_stall_en().bit(cpu == Cpu::ProCpu);
                        w.timer_unit1_core1_stall_en().bit(cpu != Cpu::ProCpu)
                    }
                    _ => unreachable!(),
                },
                UnitConfig::Enabled => match self.channel() {
                    0 => {
                        w.timer_unit0_work_en().set_bit();
                        w.timer_unit0_core0_stall_en().clear_bit();
                        w.timer_unit0_core1_stall_en().clear_bit()
                    }
                    1 => {
                        w.timer_unit1_work_en().set_bit();
                        w.timer_unit1_core0_stall_en().clear_bit();
                        w.timer_unit1_core1_stall_en().clear_bit()
                    }
                    _ => unreachable!(),
                },
            });
        });
    }

    fn set_count(&self, value: u64) {
        let systimer = SYSTIMER::regs();
        #[cfg(not(esp32s2))]
        {
            let unitload = systimer.unitload(self.channel() as _);
            let unit_load = systimer.unit_load(self.channel() as _);

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

    #[inline]
    fn read_count(&self) -> u64 {
        // This can be a shared reference as long as this type isn't Sync.

        let channel = self.channel() as usize;
        let systimer = SYSTIMER::regs();

        systimer.unit_op(channel).write(|w| w.update().set_bit());
        while !systimer.unit_op(channel).read().value_valid().bit_is_set() {}

        // Read LO, HI, then LO again, check that LO returns the same value.
        // This accounts for the case when an interrupt may happen between reading
        // HI and LO values (or the other core updates the counter mid-read), and this
        // function may get called from the ISR. In this case, the repeated read
        // will return consistent values.
        let unit_value = systimer.unit_value(channel);
        let mut lo_prev = unit_value.lo().read().bits();
        loop {
            let lo = lo_prev;
            let hi = unit_value.hi().read().bits();
            lo_prev = unit_value.lo().read().bits();

            if lo == lo_prev {
                return ((hi as u64) << 32) | lo as u64;
            }
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
enum Comparator {
    Comparator0,
    Comparator1,
    Comparator2,
}

/// An alarm unit
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Alarm<'d> {
    comp: Comparator,
    unit: Unit,
    _lifetime: PhantomData<&'d mut ()>,
}

impl Alarm<'_> {
    const fn new(comp: Comparator) -> Self {
        Alarm {
            comp,
            unit: Unit::Unit0,
            _lifetime: PhantomData,
        }
    }

    /// Unsafely clone this peripheral reference.
    ///
    /// # Safety
    ///
    /// You must ensure that you're only using one instance of this type at a
    /// time.
    pub unsafe fn clone_unchecked(&self) -> Self {
        Self {
            comp: self.comp,
            unit: self.unit,
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
    pub fn reborrow(&mut self) -> Alarm<'_> {
        unsafe { self.clone_unchecked() }
    }

    /// Returns the comparator's number.
    #[inline]
    fn channel(&self) -> u8 {
        self.comp as u8
    }

    /// Enables/disables the comparator. If enabled, this means
    /// it will generate interrupt based on its configuration.
    fn set_enable(&self, enable: bool) {
        CONF_LOCK.lock(|| {
            #[cfg(not(esp32s2))]
            SYSTIMER::regs().conf().modify(|_, w| match self.channel() {
                0 => w.target0_work_en().bit(enable),
                1 => w.target1_work_en().bit(enable),
                2 => w.target2_work_en().bit(enable),
                _ => unreachable!(),
            });
        });

        // Note: The ESP32-S2 doesn't require a lock because each
        // comparator's enable bit in a different register.
        #[cfg(esp32s2)]
        SYSTIMER::regs()
            .target_conf(self.channel() as usize)
            .modify(|_r, w| w.work_en().bit(enable));
    }

    /// Returns true if the comparator has been enabled. This means
    /// it will generate interrupt based on its configuration.
    fn is_enabled(&self) -> bool {
        #[cfg(not(esp32s2))]
        match self.channel() {
            0 => SYSTIMER::regs().conf().read().target0_work_en().bit(),
            1 => SYSTIMER::regs().conf().read().target1_work_en().bit(),
            2 => SYSTIMER::regs().conf().read().target2_work_en().bit(),
            _ => unreachable!(),
        }

        #[cfg(esp32s2)]
        SYSTIMER::regs()
            .target_conf(self.channel() as usize)
            .read()
            .work_en()
            .bit()
    }

    /// Sets the unit this comparator uses as a reference count.
    #[cfg(not(esp32s2))]
    pub fn set_unit(&self, unit: Unit) {
        SYSTIMER::regs()
            .target_conf(self.channel() as usize)
            .modify(|_, w| w.timer_unit_sel().bit(matches!(unit, Unit::Unit1)));
    }

    /// Set the mode of the comparator to be either target or periodic.
    fn set_mode(&self, mode: ComparatorMode) {
        let is_period_mode = match mode {
            ComparatorMode::Period => true,
            ComparatorMode::Target => false,
        };
        SYSTIMER::regs()
            .target_conf(self.channel() as usize)
            .modify(|_, w| w.period_mode().bit(is_period_mode));
    }

    /// Get the current mode of the comparator, which is either target or
    /// periodic.
    fn mode(&self) -> ComparatorMode {
        if SYSTIMER::regs()
            .target_conf(self.channel() as usize)
            .read()
            .period_mode()
            .bit()
        {
            ComparatorMode::Period
        } else {
            ComparatorMode::Target
        }
    }

    /// Set how often the comparator should generate an interrupt when in
    /// periodic mode.
    fn set_period(&self, value: u32) {
        let systimer = SYSTIMER::regs();
        let tconf = systimer.target_conf(self.channel() as usize);
        unsafe { tconf.modify(|_, w| w.period().bits(value)) };
        #[cfg(not(esp32s2))]
        {
            let comp_load = systimer.comp_load(self.channel() as usize);
            comp_load.write(|w| w.load().set_bit());
        }
    }

    /// Set when the comparator should generate an interrupt in target mode.
    fn set_target(&self, value: u64) {
        let systimer = SYSTIMER::regs();
        let target = systimer.trgt(self.channel() as usize);
        target.hi().write(|w| w.hi().set((value >> 32) as u32));
        target
            .lo()
            .write(|w| w.lo().set((value & 0xFFFF_FFFF) as u32));
        #[cfg(not(esp32s2))]
        {
            let comp_load = systimer.comp_load(self.channel() as usize);
            comp_load.write(|w| w.load().set_bit());
        }
    }

    /// Set the interrupt handler for this comparator.
    fn set_interrupt_handler(&self, handler: InterruptHandler) {
        let interrupt = match self.channel() {
            0 => Interrupt::SYSTIMER_TARGET0,
            1 => Interrupt::SYSTIMER_TARGET1,
            2 => Interrupt::SYSTIMER_TARGET2,
            _ => unreachable!(),
        };

        for core in crate::system::Cpu::other() {
            crate::interrupt::disable(core, interrupt);
        }

        #[cfg(not(esp32s2))]
        unsafe {
            interrupt::bind_interrupt(interrupt, handler.handler());
        }

        #[cfg(esp32s2)]
        {
            // ESP32-S2 Systimer interrupts are edge triggered. Our interrupt
            // handler calls each of the handlers, regardless of which one triggered the
            // interrupt. This mess registers an intermediate handler that
            // checks if an interrupt is active before calling the associated
            // handler functions.

            static mut HANDLERS: [Option<crate::interrupt::IsrCallback>; 3] = [None, None, None];

            #[crate::ram]
            extern "C" fn _handle_interrupt<const CH: u8>() {
                if SYSTIMER::regs().int_raw().read().target(CH).bit_is_set() {
                    let handler = unsafe { HANDLERS[CH as usize] };
                    if let Some(handler) = handler {
                        (handler.aligned_ptr())();
                    }
                }
            }

            unsafe {
                HANDLERS[self.channel() as usize] = Some(handler.handler());
                let handler = match self.channel() {
                    0 => _handle_interrupt::<0>,
                    1 => _handle_interrupt::<1>,
                    2 => _handle_interrupt::<2>,
                    _ => unreachable!(),
                };
                interrupt::bind_interrupt(interrupt, crate::interrupt::IsrCallback::new(handler));
            }
        }
        unwrap!(interrupt::enable(interrupt, handler.priority()));
    }
}

/// The modes of a comparator.
#[derive(Copy, Clone)]
enum ComparatorMode {
    /// The comparator will generate interrupts periodically.
    Period,

    /// The comparator will generate an interrupt when the unit reaches the
    /// target.
    Target,
}

impl super::Timer for Alarm<'_> {
    fn start(&self) {
        self.set_enable(true);
    }

    fn stop(&self) {
        self.set_enable(false);
    }

    fn reset(&self) {
        #[cfg(esp32s2)]
        // Run at XTAL freq, not 80 * XTAL freq:
        SYSTIMER::regs()
            .step()
            .modify(|_, w| unsafe { w.xtal_step().bits(0x1) });

        #[cfg(not(esp32s2))]
        SYSTIMER::regs()
            .conf()
            .modify(|_, w| w.timer_unit0_core0_stall_en().clear_bit());
    }

    fn is_running(&self) -> bool {
        self.is_enabled()
    }

    fn now(&self) -> Instant {
        // This should be safe to access from multiple contexts; worst case
        // scenario the second accessor ends up reading an older time stamp.

        let ticks = self.unit.read_count();

        let us = SystemTimer::ticks_to_us(ticks);

        Instant::from_ticks(us)
    }

    fn load_value(&self, value: Duration) -> Result<(), Error> {
        let mode = self.mode();

        let us = value.as_micros();
        let ticks = SystemTimer::us_to_ticks(us);

        if matches!(mode, ComparatorMode::Period) {
            // Period mode

            // The `SYSTIMER_TARGETx_PERIOD` field is 26-bits wide (or
            // 29-bits on the ESP32-S2), so we must ensure that the provided
            // value is not too wide:
            if (ticks & !SystemTimer::PERIOD_MASK) != 0 {
                return Err(Error::InvalidTimeout);
            }

            self.set_period(ticks as u32);

            // Clear and then set SYSTIMER_TARGETx_PERIOD_MODE to configure COMPx into
            // period mode
            self.set_mode(ComparatorMode::Target);
            self.set_mode(ComparatorMode::Period);
        } else {
            // Target mode

            // The counters/comparators are 52-bits wide (except on ESP32-S2,
            // which is 64-bits), so we must ensure that the provided value
            // is not too wide:
            #[cfg(not(esp32s2))]
            if (ticks & !SystemTimer::BIT_MASK) != 0 {
                return Err(Error::InvalidTimeout);
            }

            let v = self.unit.read_count();
            let t = v + ticks;

            self.set_target(t);
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
        self.set_mode(mode)
    }

    fn enable_interrupt(&self, state: bool) {
        INT_ENA_LOCK.lock(|| {
            SYSTIMER::regs()
                .int_ena()
                .modify(|_, w| w.target(self.channel()).bit(state));
        });
    }

    fn clear_interrupt(&self) {
        SYSTIMER::regs()
            .int_clr()
            .write(|w| w.target(self.channel()).clear_bit_by_one());
    }

    fn is_interrupt_set(&self) -> bool {
        SYSTIMER::regs()
            .int_raw()
            .read()
            .target(self.channel())
            .bit_is_set()
    }

    fn async_interrupt_handler(&self) -> InterruptHandler {
        match self.channel() {
            0 => asynch::target0_handler,
            1 => asynch::target1_handler,
            2 => asynch::target2_handler,
            _ => unreachable!(),
        }
    }

    fn peripheral_interrupt(&self) -> Interrupt {
        match self.channel() {
            0 => Interrupt::SYSTIMER_TARGET0,
            1 => Interrupt::SYSTIMER_TARGET1,
            2 => Interrupt::SYSTIMER_TARGET2,
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

impl crate::private::Sealed for Alarm<'_> {}

static CONF_LOCK: RawMutex = RawMutex::new();
static INT_ENA_LOCK: RawMutex = RawMutex::new();

// Async functionality of the system timer.
mod asynch {
    use core::marker::PhantomData;

    use procmacros::handler;

    use super::*;
    use crate::asynch::AtomicWaker;

    const NUM_ALARMS: usize = 3;
    static WAKERS: [AtomicWaker; NUM_ALARMS] = [const { AtomicWaker::new() }; NUM_ALARMS];

    pub(super) fn waker(alarm: &Alarm<'_>) -> &'static AtomicWaker {
        &WAKERS[alarm.channel() as usize]
    }

    #[inline]
    fn handle_alarm(comp: Comparator) {
        Alarm {
            comp,
            unit: Unit::Unit0,
            _lifetime: PhantomData,
        }
        .enable_interrupt(false);

        WAKERS[comp as usize].wake();
    }

    #[handler]
    pub(crate) fn target0_handler() {
        handle_alarm(Comparator::Comparator0);
    }

    #[handler]
    pub(crate) fn target1_handler() {
        handle_alarm(Comparator::Comparator1);
    }

    #[handler]
    pub(crate) fn target2_handler() {
        handle_alarm(Comparator::Comparator2);
    }
}

#[cfg(soc_has_etm)]
pub mod etm {
    #![cfg_attr(docsrs, procmacros::doc_replace)]
    //! # Event Task Matrix Function
    //!
    //! ## Overview
    //!
    //! The system timer supports the Event Task Matrix (ETM) function, which
    //! allows the system timer’s ETM events to trigger any peripherals’ ETM
    //! tasks.
    //!
    //! The system timer can generate the following ETM events:
    //! - SYSTIMER_EVT_CNT_CMPx: Indicates the alarm pulses generated by COMPx
    //! ## Example
    //! ```rust, no_run
    //! # {before_snippet}
    //! # use esp_hal::timer::systimer::{etm::Event, SystemTimer};
    //! # use esp_hal::timer::PeriodicTimer;
    //! # use esp_hal::etm::Etm;
    //! # use esp_hal::gpio::{
    //! #     etm::{Channels, OutputConfig},
    //! #     Level,
    //! #     Pull,
    //! # };
    //! let syst = SystemTimer::new(peripherals.SYSTIMER);
    //! let etm = Etm::new(peripherals.ETM);
    //! let gpio_ext = Channels::new(peripherals.GPIO_SD);
    //! let alarm0 = syst.alarm0;
    //! let mut led = peripherals.GPIO1;
    //!
    //! let timer_event = Event::new(&alarm0);
    //! let led_task = gpio_ext.channel0_task.toggle(
    //!     led,
    //!     OutputConfig {
    //!         open_drain: false,
    //!         pull: Pull::None,
    //!         initial_state: Level::High,
    //!     },
    //! );
    //!
    //! let _configured_etm_channel = etm.channel0.setup(&timer_event, &led_task);
    //!
    //! let timer = PeriodicTimer::new(alarm0);
    //! // configure the timer as usual
    //! // when it fires it will toggle the GPIO
    //! # {after_snippet}
    //! ```

    use super::*;

    /// An ETM controlled SYSTIMER event
    pub struct Event {
        id: u8,
    }

    impl Event {
        /// Creates an ETM event from the given [Alarm]
        pub fn new(alarm: &Alarm<'_>) -> Self {
            Self {
                id: 50 + alarm.channel(),
            }
        }
    }

    impl crate::private::Sealed for Event {}

    impl crate::etm::EtmEvent for Event {
        fn id(&self) -> u8 {
            self.id
        }
    }

    pub(super) fn enable_etm() {
        SYSTIMER::regs().conf().modify(|_, w| w.etm_en().set_bit());
    }
}
