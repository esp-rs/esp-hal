//! # System Timer peripheral driver
//!
//! The System Timer is a
#![cfg_attr(esp32s2, doc = "`64-bit`")]
#![cfg_attr(not(esp32s2), doc = "`54-bit`")]
//! timer with three comparators capable of raising an alarm interupt on each.
//!
//! To obtain the current timer value, call [`SystemTimer::now`].
//!
//! [`Alarm`]'s can be configured into two different modes:
//!
//! - [`Target`], for one-shot timer behaviour.
//! - [`Periodic`], for alarm triggers at a repeated interval.
//!
//! ## Example
//! ```no_run
//! let peripherals = Peripherals::take();
//!
//! let systimer = SystemTimer::new(peripherals.SYSTIMER);
//! println!("SYSTIMER Current value = {}", SystemTimer::now());
//!
//! let mut alarm0 = systimer.alarm0;
//! // Block for a second
//! alarm0.wait_until(SystemTimer::now().wrapping_add(SystemTimer::TICKS_PER_SECOND));
//! ```

use core::marker::PhantomData;

use fugit::MicrosDurationU32;

use crate::{
    interrupt::InterruptHandler,
    peripheral::Peripheral,
    peripherals::{
        systimer::{TARGET_CONF, TRGT},
        SYSTIMER,
    },
};

/// The SystemTimer
pub struct SystemTimer<'d, DM: crate::Mode> {
    pub alarm0: Alarm<Target, DM, 0>,
    pub alarm1: Alarm<Target, DM, 1>,
    pub alarm2: Alarm<Target, DM, 2>,
    _phantom: PhantomData<&'d ()>,
}

impl<'d> SystemTimer<'d, crate::Blocking> {
    /// Bitmask to be applied to the raw register value
    #[cfg(esp32s2)]
    pub const BIT_MASK: u64 = u64::MAX;
    #[cfg(not(esp32s2))]
    pub const BIT_MASK: u64 = 0xF_FFFF_FFFF_FFFF;

    /// The ticks per second the underlying peripheral uses
    #[cfg(esp32s2)]
    pub const TICKS_PER_SECOND: u64 = 80_000_000;
    #[cfg(not(esp32s2))]
    pub const TICKS_PER_SECOND: u64 = 16_000_000;

    /// Create a new instance in [crate::Blocking] mode.
    pub fn new(_p: impl Peripheral<P = SYSTIMER> + 'd) -> Self {
        #[cfg(soc_etm)]
        etm::enable_etm();

        Self {
            alarm0: Alarm::new(),
            alarm1: Alarm::new(),
            alarm2: Alarm::new(),
            _phantom: PhantomData,
        }
    }

    /// Get the current count of unit 0 in the system timer.
    pub fn now() -> u64 {
        // This should be safe to access from multiple contexts
        // worst case scenario the second accessor ends up reading
        // an older time stamp
        let systimer = unsafe { &*SYSTIMER::ptr() };
        systimer.unit0_op().modify(|_, w| w.update().set_bit());

        while !systimer.unit0_op().read().value_valid().bit_is_set() {}

        let value_lo = systimer.unit0_value().lo().read().bits();
        let value_hi = systimer.unit0_value().hi().read().bits();

        ((value_hi as u64) << 32) | value_lo as u64
    }
}

impl<'d> SystemTimer<'d, crate::Async> {
    /// Create a new instance in [crate::Async] mode.
    pub fn new_async(_p: impl Peripheral<P = SYSTIMER> + 'd) -> Self {
        #[cfg(soc_etm)]
        etm::enable_etm();

        Self {
            alarm0: Alarm::new(),
            alarm1: Alarm::new(),
            alarm2: Alarm::new(),
            _phantom: PhantomData,
        }
    }
}

/// A marker for a [Alarm] in target mode.
#[derive(Debug)]
pub struct Target;

/// A marker for a [Alarm] in periodic mode.
#[derive(Debug)]
pub struct Periodic;

/// A single alarm.
#[derive(Debug)]
pub struct Alarm<MODE, DM: crate::Mode, const CHANNEL: u8> {
    _pd: PhantomData<(MODE, DM)>,
}

impl<T, DM: crate::Mode, const CHANNEL: u8> Alarm<T, DM, CHANNEL> {
    // private constructor
    fn new() -> Self {
        Self { _pd: PhantomData }
    }

    fn configure(&self, conf: impl FnOnce(&TARGET_CONF, &TRGT)) {
        unsafe {
            let systimer = &*SYSTIMER::ptr();
            let tconf = systimer.target_conf(CHANNEL as usize);
            let target = systimer.trgt(CHANNEL as usize);

            #[cfg(esp32s2)]
            systimer.step().write(|w| w.xtal_step().bits(0x1)); // run at XTAL freq, not 80 * XTAL freq

            #[cfg(not(esp32s2))]
            {
                tconf.write(|w| w.timer_unit_sel().clear_bit()); // default, use unit 0
                systimer
                    .conf()
                    .modify(|_, w| w.timer_unit0_core0_stall_en().clear_bit());
            }

            conf(tconf, target);

            #[cfg(not(esp32s2))]
            {
                systimer
                    .comp_load(CHANNEL as usize)
                    .write(|w| w.load().set_bit());

                systimer.conf().modify(|_r, w| match CHANNEL {
                    0 => w.target0_work_en().set_bit(),
                    1 => w.target1_work_en().set_bit(),
                    2 => w.target2_work_en().set_bit(),
                    _ => unreachable!(),
                });
            }

            #[cfg(esp32s2)]
            tconf.modify(|_r, w| w.work_en().set_bit());
        }
    }

    pub(crate) fn enable_interrupt_internal(&self, val: bool) {
        let systimer = unsafe { &*SYSTIMER::ptr() };
        systimer.int_ena().modify(|_, w| w.target(CHANNEL).bit(val));
    }

    pub(crate) fn clear_interrupt_internal(&self) {
        let systimer = unsafe { &*SYSTIMER::ptr() };
        systimer
            .int_clr()
            .write(|w| w.target(CHANNEL).clear_bit_by_one());
    }

    pub(crate) fn set_target_internal(&self, timestamp: u64) {
        self.configure(|tconf, target| unsafe {
            tconf.write(|w| w.period_mode().clear_bit()); // target mode
            target.hi().write(|w| w.hi().bits((timestamp >> 32) as u32));
            target
                .lo()
                .write(|w| w.lo().set((timestamp & 0xFFFF_FFFF) as u32));
        })
    }

    pub(crate) fn set_period_internal(&self, ticks: u32) {
        self.configure(|tconf, target| {
            tconf.write(|w| unsafe { w.period_mode().set_bit().period().bits(ticks) });
            target.hi().write(|w| w.hi().set(0));
            target.lo().write(|w| w.lo().set(0));
        });
    }
}

impl<T, const CHANNEL: u8> Alarm<T, crate::Blocking, CHANNEL> {
    /// Set the interrupt handler for this alarm.
    pub fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        match CHANNEL {
            0 => unsafe {
                crate::interrupt::bind_interrupt(
                    crate::peripherals::Interrupt::SYSTIMER_TARGET0,
                    handler.handler(),
                );
                unwrap!(crate::interrupt::enable(
                    crate::peripherals::Interrupt::SYSTIMER_TARGET0,
                    handler.priority(),
                ));
            },
            1 => unsafe {
                crate::interrupt::bind_interrupt(
                    crate::peripherals::Interrupt::SYSTIMER_TARGET1,
                    handler.handler(),
                );
                unwrap!(crate::interrupt::enable(
                    crate::peripherals::Interrupt::SYSTIMER_TARGET1,
                    handler.priority(),
                ));
            },
            2 => unsafe {
                crate::interrupt::bind_interrupt(
                    crate::peripherals::Interrupt::SYSTIMER_TARGET2,
                    handler.handler(),
                );
                unwrap!(crate::interrupt::enable(
                    crate::peripherals::Interrupt::SYSTIMER_TARGET2,
                    handler.priority(),
                ));
            },
            _ => unreachable!(),
        }
    }

    /// Enable the interrupt for this alarm.
    pub fn enable_interrupt(&mut self, val: bool) {
        self.enable_interrupt_internal(val);
    }

    /// Enable the interrupt pending status for this alarm.
    pub fn clear_interrupt(&mut self) {
        self.clear_interrupt_internal();
    }
}
impl<DM: crate::Mode, const CHANNEL: u8> Alarm<Target, DM, CHANNEL> {
    /// Set the target value of this [Alarm]
    pub fn set_target(&mut self, timestamp: u64) {
        self.set_target_internal(timestamp);
    }

    /// Block waiting until the timer reaches the `timestamp`
    pub fn wait_until(&mut self, timestamp: u64) {
        self.clear_interrupt_internal();
        self.set_target(timestamp);
        let r = unsafe { &*crate::peripherals::SYSTIMER::PTR }.int_raw();
        loop {
            if r.read().target(CHANNEL).bit_is_set() {
                break;
            }
        }
    }

    /// Converts this [Alarm] into [Periodic] mode
    pub fn into_periodic(self) -> Alarm<Periodic, DM, CHANNEL> {
        Alarm { _pd: PhantomData }
    }
}

impl<DM: crate::Mode, const CHANNEL: u8> Alarm<Periodic, DM, CHANNEL> {
    /// Set the period of this [Alarm]
    pub fn set_period(&mut self, period: MicrosDurationU32) {
        let us = period.ticks();
        let ticks = us * (SystemTimer::TICKS_PER_SECOND / 1_000_000) as u32;

        self.set_period_internal(ticks);
    }

    /// Converts this [Alarm] into [Target] mode
    pub fn into_target(self) -> Alarm<Target, DM, CHANNEL> {
        Alarm { _pd: PhantomData }
    }
}

impl<T, DM: crate::Mode> Alarm<T, DM, 0> {
    /// Conjure an alarm out of thin air.
    ///
    /// # Safety
    ///
    /// Users must take care to ensure that only one reference to the timer is
    /// in scope at any given time.
    pub const unsafe fn conjure() -> Self {
        Self { _pd: PhantomData }
    }
}

impl<T, DM: crate::Mode> Alarm<T, DM, 1> {
    /// Conjure an alarm out of thin air.
    ///
    /// # Safety
    ///
    /// Users must take care to ensure that only one reference to the timer is
    /// in scope at any given time.
    pub const unsafe fn conjure() -> Self {
        Self { _pd: PhantomData }
    }
}

impl<T, DM: crate::Mode> Alarm<T, DM, 2> {
    /// Conjure an alarm out of thin air.
    ///
    /// # Safety
    ///
    /// Users must take care to ensure that only one reference to the timer is
    /// in scope at any given time.
    pub const unsafe fn conjure() -> Self {
        Self { _pd: PhantomData }
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

    const INIT: AtomicWaker = AtomicWaker::new();
    static WAKERS: [AtomicWaker; NUM_ALARMS] = [INIT; NUM_ALARMS];

    pub(crate) struct AlarmFuture<'a, const N: u8> {
        phantom: PhantomData<&'a Alarm<Periodic, crate::Async, N>>,
    }

    impl<'a, const N: u8> AlarmFuture<'a, N> {
        pub(crate) fn new(alarm: &'a Alarm<Periodic, crate::Async, N>) -> Self {
            alarm.clear_interrupt_internal();

            let (interrupt, handler) = match N {
                0 => (
                    crate::peripherals::Interrupt::SYSTIMER_TARGET0,
                    target0_handler,
                ),
                1 => (
                    crate::peripherals::Interrupt::SYSTIMER_TARGET1,
                    target1_handler,
                ),
                _ => (
                    crate::peripherals::Interrupt::SYSTIMER_TARGET2,
                    target2_handler,
                ),
            };

            unsafe {
                crate::interrupt::bind_interrupt(interrupt, handler.handler());
                crate::interrupt::enable(interrupt, handler.priority()).unwrap();
            }

            alarm.enable_interrupt_internal(true);

            Self {
                phantom: PhantomData,
            }
        }

        fn event_bit_is_clear(&self) -> bool {
            unsafe { &*crate::peripherals::SYSTIMER::PTR }
                .int_ena()
                .read()
                .target(N)
                .bit_is_clear()
        }
    }

    impl<'a, const N: u8> core::future::Future for AlarmFuture<'a, N> {
        type Output = ();

        fn poll(self: Pin<&mut Self>, ctx: &mut Context<'_>) -> Poll<Self::Output> {
            WAKERS[N as usize].register(ctx.waker());

            if self.event_bit_is_clear() {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        }
    }

    impl<const CHANNEL: u8> embedded_hal_async::delay::DelayNs
        for Alarm<Periodic, crate::Async, CHANNEL>
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
    //! ```no_run
    //! let syst = SystemTimer::new(peripherals.SYSTIMER);
    //! let mut alarm0 = syst.alarm0.into_periodic();
    //! alarm0.set_period(1.secs());
    //!
    //! let timer_event = SysTimerEtmEvent::new(&mut alarm0);
    //! ```

    use super::*;

    /// An ETM controlled SYSTIMER event
    pub struct SysTimerEtmEvent<'a, M, DM: crate::Mode, const N: u8> {
        alarm: &'a mut Alarm<M, DM, N>,
    }

    impl<'a, M, DM: crate::Mode, const N: u8> SysTimerEtmEvent<'a, M, DM, N> {
        /// Creates an ETM event from the given [Alarm]
        pub fn new(alarm: &'a mut Alarm<M, DM, N>) -> Self {
            Self { alarm }
        }

        /// Execute closure f with mutable access to the wrapped [Alarm].
        pub fn with<R>(&self, f: impl FnOnce(&&'a mut Alarm<M, DM, N>) -> R) -> R {
            let alarm = &self.alarm;
            f(alarm)
        }
    }

    impl<'a, M, DM: crate::Mode, const N: u8> crate::private::Sealed
        for SysTimerEtmEvent<'a, M, DM, N>
    {
    }

    impl<'a, M, DM: crate::Mode, const N: u8> crate::etm::EtmEvent for SysTimerEtmEvent<'a, M, DM, N> {
        fn id(&self) -> u8 {
            50 + N
        }
    }

    pub(super) fn enable_etm() {
        let syst = unsafe { crate::peripherals::SYSTIMER::steal() };
        syst.conf().modify(|_, w| w.etm_en().set_bit());
    }
}
