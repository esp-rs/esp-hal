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

use core::fmt::Debug;

use fugit::{Instant, MicrosDurationU64};

use super::{Error, Timer as _};
use crate::{
    interrupt::{self, InterruptHandler},
    peripheral::Peripheral,
    peripherals::{Interrupt, SYSTIMER},
    sync::{lock, RawMutex},
    system::{Peripheral as PeripheralEnable, PeripheralClockControl},
    Cpu,
};

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
pub struct SystemTimer {
    /// Alarm 0.
    pub alarm0: Alarm,

    /// Alarm 1.
    pub alarm1: Alarm,

    /// Alarm 2.
    pub alarm2: Alarm,
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
    pub fn new(_systimer: SYSTIMER) -> Self {
        // Don't reset Systimer as it will break `time::now`, only enable it
        PeripheralClockControl::enable(PeripheralEnable::Systimer);

        #[cfg(soc_etm)]
        etm::enable_etm();

        Self {
            alarm0: Alarm::new(0),
            alarm1: Alarm::new(1),
            alarm2: Alarm::new(2),
        }
    }

    /// Get the current count of the given unit in the System Timer.
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
    /// - Disabling a `Unit` whilst [`Alarm`]s are using it will affect the
    ///   [`Alarm`]s operation.
    /// - Disabling Unit0 will affect [`now`](crate::time::now).
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
    /// - Modifying a unit's count whilst [`Alarm`]s are using it may cause
    ///   unexpected behaviour
    /// - Any modification of the unit0 count will affect
    ///   [`now`](crate::time::now).
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
        lock(&CONF_LOCK, || {
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

/// An alarm unit
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Alarm {
    comp: u8,
    unit: Unit,
}

impl Alarm {
    const fn new(comp: u8) -> Self {
        Alarm {
            comp,
            unit: Unit::Unit0,
        }
    }

    /// Returns the comparator's number.
    #[inline]
    fn channel(&self) -> u8 {
        self.comp
    }

    /// Enables/disables the comparator. If enabled, this means
    /// it will generate interrupt based on its configuration.
    fn set_enable(&self, enable: bool) {
        lock(&CONF_LOCK, || {
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

            #[crate::ram]
            unsafe extern "C" fn _handle_interrupt<const CH: u8>() {
                if SYSTIMER::regs().int_raw().read().target(CH).bit_is_set() {
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

/// The modes of a comparator.
#[derive(Copy, Clone)]
enum ComparatorMode {
    /// The comparator will generate interrupts periodically.
    Period,

    /// The comparator will generate an interrupt when the unit reaches the
    /// target.
    Target,
}

impl super::Timer for Alarm {
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

    fn now(&self) -> Instant<u64, 1, 1_000_000> {
        // This should be safe to access from multiple contexts; worst case
        // scenario the second accessor ends up reading an older time stamp.

        let ticks = self.unit.read_count();

        let us = ticks / (SystemTimer::ticks_per_second() / 1_000_000);

        Instant::<u64, 1, 1_000_000>::from_ticks(us)
    }

    fn load_value(&self, value: MicrosDurationU64) -> Result<(), Error> {
        let mode = self.mode();

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
        lock(&INT_ENA_LOCK, || {
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

    async fn wait(&self) {
        asynch::AlarmFuture::new(self).await
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
}

impl Peripheral for Alarm {
    type P = Self;

    #[inline]
    unsafe fn clone_unchecked(&self) -> Self::P {
        Alarm {
            comp: self.comp,
            unit: self.unit,
        }
    }
}

impl crate::private::Sealed for Alarm {}

static CONF_LOCK: RawMutex = RawMutex::new();
static INT_ENA_LOCK: RawMutex = RawMutex::new();

// Async functionality of the system timer.
mod asynch {
    use core::{
        pin::Pin,
        task::{Context, Poll},
    };

    use procmacros::handler;

    use super::*;
    use crate::{asynch::AtomicWaker, peripherals::SYSTIMER};

    const NUM_ALARMS: usize = 3;

    static WAKERS: [AtomicWaker; NUM_ALARMS] = [const { AtomicWaker::new() }; NUM_ALARMS];

    #[must_use = "futures do nothing unless you `.await` or poll them"]
    pub(crate) struct AlarmFuture<'a> {
        alarm: &'a Alarm,
    }

    impl<'a> AlarmFuture<'a> {
        pub(crate) fn new(alarm: &'a Alarm) -> Self {
            alarm.enable_interrupt(true);

            Self { alarm }
        }

        fn event_bit_is_clear(&self) -> bool {
            SYSTIMER::regs()
                .int_ena()
                .read()
                .target(self.alarm.channel())
                .bit_is_clear()
        }
    }

    impl core::future::Future for AlarmFuture<'_> {
        type Output = ();

        fn poll(self: Pin<&mut Self>, ctx: &mut Context<'_>) -> Poll<Self::Output> {
            WAKERS[self.alarm.channel() as usize].register(ctx.waker());

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
            SYSTIMER::regs()
                .int_ena()
                .modify(|_, w| w.target0().clear_bit());
        });

        WAKERS[0].wake();
    }

    #[handler]
    pub(crate) fn target1_handler() {
        lock(&INT_ENA_LOCK, || {
            SYSTIMER::regs()
                .int_ena()
                .modify(|_, w| w.target1().clear_bit());
        });

        WAKERS[1].wake();
    }

    #[handler]
    pub(crate) fn target2_handler() {
        lock(&INT_ENA_LOCK, || {
            SYSTIMER::regs()
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
    //! ## Example
    //! ```rust, no_run
    #![doc = crate::before_snippet!()]
    //! # use esp_hal::timer::systimer::{etm::Event, SystemTimer};
    //! # use esp_hal::timer::PeriodicTimer;
    //! # use esp_hal::etm::Etm;
    //! # use esp_hal::gpio::{
    //! #     etm::{Channels, OutputConfig},
    //! #     Level,
    //! #     Pull,
    //! # };
    //! # use fugit::ExtU32;
    //! let syst = SystemTimer::new(peripherals.SYSTIMER);
    //! let etm = Etm::new(peripherals.SOC_ETM);
    //! let gpio_ext = Channels::new(peripherals.GPIO_SD);
    //! let alarm0 = syst.alarm0;
    //! let mut led = peripherals.GPIO1;
    //!
    //! let timer_event = Event::new(&alarm0);
    //! let led_task = gpio_ext.channel0_task.toggle(
    //!     &mut led,
    //!     OutputConfig {
    //!         open_drain: false,
    //!         pull: Pull::None,
    //!         initial_state: Level::High,
    //!     },
    //! );
    //!
    //! let _configured_etm_channel = etm.channel0.setup(&timer_event,
    //! &led_task);
    //!
    //! let timer = PeriodicTimer::new(alarm0);
    //! // configure the timer as usual
    //! // when it fires it will toggle the GPIO
    //! # Ok(())
    //! # }
    //! ```

    use super::*;

    /// An ETM controlled SYSTIMER event
    pub struct Event {
        id: u8,
    }

    impl Event {
        /// Creates an ETM event from the given [Alarm]
        pub fn new(alarm: &Alarm) -> Self {
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
