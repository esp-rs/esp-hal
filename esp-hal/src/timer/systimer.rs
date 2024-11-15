//! # System Timer (SYSTIMER)
//!
//! ## Overview
//! The System Timer is a
#![cfg_attr(esp32s2, doc = "64-bit")]
#![cfg_attr(not(esp32s2), doc = "52-bit")]
//! timer which can be used, for example, to generate tick interrupts for an
//! operating system, or simply as a general-purpose timer.

use core::fmt::{Debug, Formatter};

use fugit::{Instant, MicrosDurationU64};

use super::{Error, Timer as _};
use crate::{
    interrupt::{self, InterruptHandler},
    peripheral::{Peripheral, PeripheralRef},
    peripherals::{Interrupt, SYSTIMER},
    sync::{lock, Lock},
    system::{Peripheral as PeripheralEnable, PeripheralClockControl},
    Cpu,
    InterruptConfigurable,
};

/// Systimer's config
#[non_exhaustive]
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct Config {
    #[cfg(not(esp32s2))]
    unit0: UnitConfig,
    #[cfg(not(esp32s2))]
    unit1: UnitConfig,
}

/// System Timer driver.
pub struct SystemTimer {
    /// Comparator 0.
    pub comparator0: AnyComparator,

    /// Comparator 1.
    pub comparator1: AnyComparator,

    /// Comparator 2.
    pub comparator2: AnyComparator,
}

impl SystemTimer {
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

    /// Returns the tick frequency of the underlying timer unit.
    pub fn ticks_per_second() -> u64 {
        cfg_if::cfg_if! {
            if #[cfg(esp32s2)] {
                const MULTIPLIER: u64 = 2_000_000;
            } else if #[cfg(esp32h2)] {
                // The counters and comparators are driven using `XTAL_CLK`.
                // The average clock frequency is fXTAL_CLK/2, which is 16 MHz.
                // The timer counting is incremented by 1/16 μs on each `CNT_CLK` cycle.
                const MULTIPLIER: u64 = 10_000_000 / 20;
            } else {
                // The counters and comparators are driven using `XTAL_CLK`.
                // The average clock frequency is fXTAL_CLK/2.5, which is 16 MHz.
                // The timer counting is incremented by 1/16 μs on each `CNT_CLK` cycle.
                const MULTIPLIER: u64 = 10_000_000 / 25;
            }
        }
        let xtal_freq_mhz = crate::clock::Clocks::xtal_freq().to_MHz();
        xtal_freq_mhz as u64 * MULTIPLIER
    }

    /// Create a new instance.
    pub fn new(_systimer: SYSTIMER, _config: Config) -> Self {
        // Don't reset Systimer as it will break `time::now`, only enable it
        PeripheralClockControl::enable(PeripheralEnable::Systimer);

        #[cfg(soc_etm)]
        etm::enable_etm();

        #[cfg(not(esp32s2))]
        {
            Unit::Unit0.configure(_config.unit0);
            Unit::Unit1.configure(_config.unit1);
        }

        Self {
            comparator0: AnyComparator(0),
            comparator1: AnyComparator(1),
            comparator2: AnyComparator(2),
        }
    }

    /// Get the current count of Unit 0 in the System Timer.
    pub fn now() -> u64 {
        // This should be safe to access from multiple contexts
        // worst case scenario the second accessor ends up reading
        // an older time stamp

        Unit::Unit0.read_count()
    }
}

/// Alarms created from the System Timer peripheral.
pub struct SysTimerAlarms {
    /// Alarm 0
    pub alarm0: Alarm<'static>,
    /// Alarm 1
    pub alarm1: Alarm<'static>,
    /// Alarm 2
    pub alarm2: Alarm<'static>,
}

impl SystemTimer {
    /// Split the System Timer into three alarms.
    ///
    /// This is a convenience method to create `'static` alarms.
    /// You are encouraged to use [Alarm::new] over this very specific
    /// helper.
    pub fn split(self) -> SysTimerAlarms {
        SysTimerAlarms {
            alarm0: Alarm::new(self.comparator0),
            alarm1: Alarm::new(self.comparator1),
            alarm2: Alarm::new(self.comparator2),
        }
    }
}

/// A
#[cfg_attr(esp32s2, doc = "64-bit")]
#[cfg_attr(not(esp32s2), doc = "52-bit")]
/// counter.
#[derive(Debug, Copy, Clone)]
pub enum Unit {
    /// Unit 0
    Unit0 = 0,
    #[cfg(not(esp32s2))]
    /// Unit 1
    Unit1 = 1,
}

impl Unit {
    fn channel(&self) -> u8 {
        *self as _
    }

    #[cfg(not(esp32s2))]
    /// Configures when this counter can run.
    /// It can be configured to stall or continue running when CPU stalls
    /// or enters on-chip-debugging mode
    fn configure(&self, config: UnitConfig) {
        let systimer = unsafe { &*SYSTIMER::ptr() };
        let conf = systimer.conf();

        lock(&CONF_LOCK, || {
            conf.modify(|_, w| match config {
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

    /// Reads the current counter value.
    fn read_count(&self) -> u64 {
        // This can be a shared reference as long as this type isn't Sync.

        let channel = self.channel() as usize;
        let systimer = unsafe { SYSTIMER::steal() };

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

/// Any Comparator
pub struct AnyComparator(u8);

impl AnyComparator {
    fn channel(&self) -> u8 {
        self.0
    }

    /// Enables/disables the comparator. If enabled, this means
    /// it will generate interrupt based on its configuration.
    fn set_enable(&self, enable: bool) {
        let systimer = unsafe { &*SYSTIMER::ptr() };

        lock(&CONF_LOCK, || {
            #[cfg(not(esp32s2))]
            systimer.conf().modify(|_, w| match self.channel() {
                0 => w.target0_work_en().bit(enable),
                1 => w.target1_work_en().bit(enable),
                2 => w.target2_work_en().bit(enable),
                _ => unreachable!(),
            });
        });

        // Note: The ESP32-S2 doesn't require a lock because each
        // comparator's enable bit in a different register.
        #[cfg(esp32s2)]
        systimer
            .target_conf(self.channel() as usize)
            .modify(|_r, w| w.work_en().bit(enable));
    }

    /// Returns true if the comparator has been enabled. This means
    /// it will generate interrupt based on its configuration.
    fn is_enabled(&self) -> bool {
        #[cfg(not(esp32s2))]
        {
            let systimer = unsafe { &*SYSTIMER::ptr() };
            let conf = systimer.conf().read();
            match self.channel() {
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
                systimer.target_conf(self.channel() as usize)
            };
            tconf.read().work_en().bit()
        }
    }

    /// Sets the unit this comparator uses as a reference count.
    #[cfg(not(esp32s2))]
    fn set_unit(&self, is_unit0: bool) {
        let tconf = unsafe {
            let systimer = &*SYSTIMER::ptr();
            systimer.target_conf(self.channel() as usize)
        };
        tconf.modify(|_, w| w.timer_unit_sel().bit(is_unit0));
    }

    /// Set the mode of the comparator to be either target or periodic.
    fn set_mode(&self, mode: ComparatorMode) {
        let tconf = unsafe {
            let systimer = &*SYSTIMER::ptr();
            systimer.target_conf(self.channel() as usize)
        };
        let is_period_mode = match mode {
            ComparatorMode::Period => true,
            ComparatorMode::Target => false,
        };
        tconf.modify(|_, w| w.period_mode().bit(is_period_mode));
    }

    /// Get the current mode of the comparator, which is either target or
    /// periodic.
    fn mode(&self) -> ComparatorMode {
        let tconf = unsafe {
            let systimer = &*SYSTIMER::ptr();
            systimer.target_conf(self.channel() as usize)
        };
        if tconf.read().period_mode().bit() {
            ComparatorMode::Period
        } else {
            ComparatorMode::Target
        }
    }

    /// Set how often the comparator should generate an interrupt when in
    /// periodic mode.
    fn set_period(&self, value: u32) {
        unsafe {
            let systimer = &*SYSTIMER::ptr();
            let tconf = systimer.target_conf(self.channel() as usize);
            tconf.modify(|_, w| w.period().bits(value));
            #[cfg(not(esp32s2))]
            {
                let comp_load = systimer.comp_load(self.channel() as usize);
                comp_load.write(|w| w.load().set_bit());
            }
        }
    }

    /// Set when the comparator should generate an interrupt in target mode.
    fn set_target(&self, value: u64) {
        let systimer = unsafe { &*SYSTIMER::ptr() };
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

        for core in crate::Cpu::other() {
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

            static mut HANDLERS: [Option<extern "C" fn()>; 3] = [None, None, None];

            #[crate::prelude::ram]
            unsafe extern "C" fn _handle_interrupt<const CH: u8>() {
                if unsafe { &*SYSTIMER::PTR }
                    .int_raw()
                    .read()
                    .target(CH)
                    .bit_is_set()
                {
                    let handler = unsafe { HANDLERS[CH as usize] };
                    if let Some(handler) = handler {
                        handler();
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
                interrupt::bind_interrupt(interrupt, handler);
            }
        }
        unwrap!(interrupt::enable(interrupt, handler.priority()));
    }
}

impl crate::private::Sealed for AnyComparator {}

impl Peripheral for AnyComparator {
    type P = Self;

    #[inline]
    unsafe fn clone_unchecked(&self) -> Self::P {
        core::ptr::read(self as *const _)
    }
}

/// The configuration of a unit.
#[derive(Debug, Copy, Clone, PartialEq, Eq, Default)]
pub enum UnitConfig {
    /// Unit is not counting.
    Disabled,

    /// Unit is counting unless the Cpu is stalled.
    DisabledIfCpuIsStalled(Cpu),

    #[default]
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

/// A single alarm.
pub struct Alarm<'d> {
    comparator: PeripheralRef<'d, AnyComparator>,
    unit: Unit,
}

impl Debug for Alarm<'_> {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Alarm")
            .field("comparator", &self.comparator.channel())
            .field("unit", &self.unit.channel())
            .finish()
    }
}

impl<'d> Alarm<'d> {
    /// Creates a new alarm from a comparator and a default unit of `Unit0`.
    pub fn new(comparator: impl Peripheral<P = AnyComparator> + 'd) -> Self {
        Self::new_with_unit(comparator, Unit::Unit0)
    }

    /// Creates a new alarm from a comparator and unit.
    pub fn new_with_unit(comparator: impl Peripheral<P = AnyComparator> + 'd, unit: Unit) -> Self {
        crate::into_ref!(comparator);
        #[cfg(not(esp32s2))]
        comparator.set_unit(unit.channel() == 0);
        Self { comparator, unit }
    }
}

impl InterruptConfigurable for Alarm<'_> {
    fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        self.comparator.set_interrupt_handler(handler)
    }
}

impl crate::private::Sealed for Alarm<'_> {}

impl super::Timer for Alarm<'_> {
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

        let ticks = self.unit.read_count();

        let us = ticks / (SystemTimer::ticks_per_second() / 1_000_000);

        Instant::<u64, 1, 1_000_000>::from_ticks(us)
    }

    fn load_value(&self, value: MicrosDurationU64) -> Result<(), Error> {
        let mode = self.comparator.mode();

        let us = value.ticks();
        let ticks = us * (SystemTimer::ticks_per_second() / 1_000_000);

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

            // The counters/comparators are 52-bits wide (except on ESP32-S2,
            // which is 64-bits), so we must ensure that the provided value
            // is not too wide:
            #[cfg(not(esp32s2))]
            if (ticks & !SystemTimer::BIT_MASK) != 0 {
                return Err(Error::InvalidTimeout);
            }

            let v = self.unit.read_count();
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
        lock(&INT_ENA_LOCK, || {
            unsafe { &*SYSTIMER::PTR }
                .int_ena()
                .modify(|_, w| w.target(self.comparator.channel()).bit(state));
        });
    }

    fn clear_interrupt(&self) {
        unsafe { &*SYSTIMER::PTR }
            .int_clr()
            .write(|w| w.target(self.comparator.channel()).clear_bit_by_one());
    }

    fn is_interrupt_set(&self) -> bool {
        unsafe { &*SYSTIMER::PTR }
            .int_raw()
            .read()
            .target(self.comparator.channel())
            .bit_is_set()
    }

    fn set_alarm_active(&self, _active: bool) {
        // Nothing to do
    }

    fn set_interrupt_handler(&self, handler: InterruptHandler) {
        self.comparator.set_interrupt_handler(handler);
    }

    async fn wait(&self) {
        asynch::AlarmFuture::new(self).await
    }

    fn async_interrupt_handler(&self) -> InterruptHandler {
        match self.comparator.channel() {
            0 => asynch::target0_handler,
            1 => asynch::target1_handler,
            2 => asynch::target2_handler,
            _ => unreachable!(),
        }
    }

    fn peripheral_interrupt(&self) -> Interrupt {
        match self.comparator.channel() {
            0 => Interrupt::SYSTIMER_TARGET0,
            1 => Interrupt::SYSTIMER_TARGET1,
            2 => Interrupt::SYSTIMER_TARGET2,
            _ => unreachable!(),
        }
    }
}

impl Peripheral for Alarm<'_> {
    type P = Self;

    #[inline]
    unsafe fn clone_unchecked(&self) -> Self::P {
        core::ptr::read(self as *const _)
    }
}

static CONF_LOCK: Lock = Lock::new();
static INT_ENA_LOCK: Lock = Lock::new();

// Async functionality of the system timer.
mod asynch {
    use core::{
        pin::Pin,
        task::{Context, Poll},
    };

    use embassy_sync::waitqueue::AtomicWaker;
    use procmacros::handler;

    use super::*;

    const NUM_ALARMS: usize = 3;

    static WAKERS: [AtomicWaker; NUM_ALARMS] = [const { AtomicWaker::new() }; NUM_ALARMS];

    #[must_use = "futures do nothing unless you `.await` or poll them"]
    pub(crate) struct AlarmFuture<'a> {
        alarm: &'a Alarm<'a>,
    }

    impl<'a> AlarmFuture<'a> {
        pub(crate) fn new(alarm: &'a Alarm<'a>) -> Self {
            alarm.clear_interrupt();

            alarm.enable_interrupt(true);

            Self { alarm }
        }

        fn event_bit_is_clear(&self) -> bool {
            unsafe { &*crate::peripherals::SYSTIMER::PTR }
                .int_ena()
                .read()
                .target(self.alarm.comparator.channel())
                .bit_is_clear()
        }
    }

    impl core::future::Future for AlarmFuture<'_> {
        type Output = ();

        fn poll(self: Pin<&mut Self>, ctx: &mut Context<'_>) -> Poll<Self::Output> {
            WAKERS[self.alarm.comparator.channel() as usize].register(ctx.waker());

            if self.event_bit_is_clear() {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        }
    }

    #[handler]
    pub(crate) fn target0_handler() {
        lock(&INT_ENA_LOCK, || {
            unsafe { &*crate::peripherals::SYSTIMER::PTR }
                .int_ena()
                .modify(|_, w| w.target0().clear_bit());
        });

        WAKERS[0].wake();
    }

    #[handler]
    pub(crate) fn target1_handler() {
        lock(&INT_ENA_LOCK, || {
            unsafe { &*crate::peripherals::SYSTIMER::PTR }
                .int_ena()
                .modify(|_, w| w.target1().clear_bit());
        });

        WAKERS[1].wake();
    }

    #[handler]
    pub(crate) fn target2_handler() {
        lock(&INT_ENA_LOCK, || {
            unsafe { &*crate::peripherals::SYSTIMER::PTR }
                .int_ena()
                .modify(|_, w| w.target2().clear_bit());
        });

        WAKERS[2].wake();
    }
}

#[cfg(soc_etm)]
pub mod etm {
    //! # Event Task Matrix Function
    //!
    //! ## Overview
    //!
    //! The system timer supports the Event Task Matrix (ETM) function, which
    //! allows the system timer’s ETM events to trigger any peripherals’ ETM
    //! tasks.
    //!
    //!    The system timer can generate the following ETM events:
    //!    - SYSTIMER_EVT_CNT_CMPx: Indicates the alarm pulses generated by
    //!      COMPx
    //!
    //! ## Example
    //! ```rust, no_run
    #![doc = crate::before_snippet!()]
    //! # use esp_hal::timer::systimer::{etm::Event, SystemTimer};
    //! # use fugit::ExtU32;
    //! let syst = SystemTimer::new(peripherals.SYSTIMER, Default::default());
    //! let syst_alarms = syst.split();
    //! let mut alarm0 = syst_alarms.alarm0;
    //! alarm0.enable_auto_reload(true);
    //! alarm0.load_value(1u64.secs()).unwrap();
    //! alarm0.start();
    //!
    //! let timer_event = Event::new(&mut alarm0);
    //! # }
    //! ```

    use super::*;

    /// An ETM controlled SYSTIMER event
    pub struct Event<'a, 'd> {
        alarm: &'a mut Alarm<'d>,
    }

    impl<'a, 'd> Event<'a, 'd> {
        /// Creates an ETM event from the given [Alarm]
        pub fn new(alarm: &'a mut Alarm<'d>) -> Self {
            Self { alarm }
        }

        /// Execute closure f with mutable access to the wrapped [Alarm].
        pub fn with<R>(&self, f: impl FnOnce(&&'a mut Alarm<'d>) -> R) -> R {
            let alarm = &self.alarm;
            f(alarm)
        }
    }

    impl crate::private::Sealed for Event<'_, '_> {}

    impl crate::etm::EtmEvent for Event<'_, '_> {
        fn id(&self) -> u8 {
            50 + self.alarm.comparator.channel()
        }
    }

    pub(super) fn enable_etm() {
        let syst = unsafe { crate::peripherals::SYSTIMER::steal() };
        syst.conf().modify(|_, w| w.etm_en().set_bit());
    }
}
