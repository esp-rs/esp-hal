//! # System Timer peripheral driver
//!
//! ## Overview
//! This software module provides an interface to interact with the system timer
//! (SYSTIMER) peripheral on ESP microcontroller chips.
//!
//! Each ESP chip provides a timer (`52-bit` or `64-bit`, depends on chip),
//! which can be used to generate tick interrupts for operating system, or be
//! used as a general timer to generate periodic interrupts or one-time
//! interrupts. With the help of the RTC timer, system timer can be kept up to
//! date after Light-sleep or Deep-sleep.
//!
//! The driver supports features such as retrieving the current system time,
//! setting alarms for specific time points or periodic intervals, enabling and
//! clearing interrupts, configuring various settings of the system timer.
//!
//! By using the SYSTIMER peripheral driver, you can leverage the system timer
//! functionality of ESP chips for accurate timing measurements, event
//! triggering and synchronization in your applications.
//!
//! ## Example
//! ```no_run
//! let peripherals = Peripherals::take();
//!
//! let syst = SystemTimer::new(peripherals.SYSTIMER);
//! println!("SYSTIMER Current value = {}", SystemTimer::now());
//! ```

use core::{intrinsics::transmute, marker::PhantomData};

use fugit::MicrosDurationU32;

use crate::{
    peripheral::{Peripheral, PeripheralRef},
    peripherals::{
        generic::Reg,
        systimer::{
            target0_conf::TARGET0_CONF_SPEC,
            target0_hi::TARGET0_HI_SPEC,
            target0_lo::TARGET0_LO_SPEC,
        },
        SYSTIMER,
    },
};

// TODO this only handles unit0 of the systimer

pub struct SystemTimer<'d> {
    _inner: PeripheralRef<'d, SYSTIMER>,
    pub alarm0: Alarm<Target, 0>,
    pub alarm1: Alarm<Target, 1>,
    pub alarm2: Alarm<Target, 2>,
}

impl<'d> SystemTimer<'d> {
    #[cfg(esp32s2)]
    pub const BIT_MASK: u64 = u64::MAX;
    #[cfg(not(esp32s2))]
    pub const BIT_MASK: u64 = 0xF_FFFF_FFFF_FFFF;

    #[cfg(esp32s2)]
    pub const TICKS_PER_SECOND: u64 = 80_000_000; // TODO this can change when we have support for changing APB frequency
    #[cfg(not(esp32s2))]
    pub const TICKS_PER_SECOND: u64 = 16_000_000;

    pub fn new(p: impl Peripheral<P = SYSTIMER> + 'd) -> Self {
        crate::into_ref!(p);

        #[cfg(soc_etm)]
        etm::enable_etm();

        Self {
            _inner: p,
            alarm0: Alarm::new(),
            alarm1: Alarm::new(),
            alarm2: Alarm::new(),
        }
    }

    // TODO use fugit types
    pub fn now() -> u64 {
        // This should be safe to access from multiple contexts
        // worst case scenario the second accesor ends up reading
        // an older time stamp
        let systimer = unsafe { &*SYSTIMER::ptr() };
        systimer
            .unit0_op()
            .modify(|_, w| w.timer_unit0_update().set_bit());

        while !systimer
            .unit0_op()
            .read()
            .timer_unit0_value_valid()
            .bit_is_set()
        {}

        let value_lo = systimer.unit0_value_lo().read().bits();
        let value_hi = systimer.unit0_value_hi().read().bits();

        ((value_hi as u64) << 32) | value_lo as u64
    }
}

#[derive(Debug)]
pub struct Target;

#[derive(Debug)]
pub struct Periodic; // TODO, also impl e-h timer traits

#[derive(Debug)]
pub struct Alarm<MODE, const CHANNEL: u8> {
    _pd: PhantomData<MODE>,
}

impl<T, const CHANNEL: u8> Alarm<T, CHANNEL> {
    // private constructor
    fn new() -> Self {
        Self { _pd: PhantomData }
    }

    pub fn enable_interrupt(&self, val: bool) {
        let systimer = unsafe { &*SYSTIMER::ptr() };
        match CHANNEL {
            0 => systimer
                .int_ena()
                .modify(|_, w| w.target0_int_ena().bit(val)),
            1 => systimer
                .int_ena()
                .modify(|_, w| w.target1_int_ena().bit(val)),
            2 => systimer
                .int_ena()
                .modify(|_, w| w.target2_int_ena().bit(val)),
            _ => unreachable!(),
        }
    }

    pub fn clear_interrupt(&self) {
        let systimer = unsafe { &*SYSTIMER::ptr() };
        match CHANNEL {
            0 => systimer.int_clr().write(|w| w.target0_int_clr().set_bit()),
            1 => systimer.int_clr().write(|w| w.target1_int_clr().set_bit()),
            2 => systimer.int_clr().write(|w| w.target2_int_clr().set_bit()),
            _ => unreachable!(),
        }
    }

    fn configure(
        &self,
        conf: impl FnOnce(&Reg<TARGET0_CONF_SPEC>, &Reg<TARGET0_HI_SPEC>, &Reg<TARGET0_LO_SPEC>),
    ) {
        unsafe {
            let systimer = &*SYSTIMER::ptr();
            let (tconf, hi, lo): (
                &Reg<TARGET0_CONF_SPEC>,
                &Reg<TARGET0_HI_SPEC>,
                &Reg<TARGET0_LO_SPEC>,
            ) = match CHANNEL {
                0 => (
                    &systimer.target0_conf(),
                    &systimer.target0_hi(),
                    &systimer.target0_lo(),
                ),
                1 => (
                    transmute(&systimer.target1_conf()),
                    transmute(&systimer.target1_hi()),
                    transmute(&systimer.target1_lo()),
                ),
                2 => (
                    transmute(&systimer.target2_conf()),
                    transmute(&systimer.target2_hi()),
                    transmute(&systimer.target2_lo()),
                ),
                _ => unreachable!(),
            };

            #[cfg(esp32s2)]
            systimer.step().write(|w| w.timer_xtal_step().bits(0x1)); // run at XTAL freq, not 80 * XTAL freq

            #[cfg(any(esp32c2, esp32c3, esp32c6, esp32h2, esp32s3))]
            {
                tconf.write(|w| w.target0_timer_unit_sel().clear_bit()); // default, use unit 0
                systimer
                    .conf()
                    .modify(|_, w| w.timer_unit0_core0_stall_en().clear_bit());
            }

            conf(tconf, hi, lo);

            #[cfg(any(esp32c2, esp32c3, esp32c6, esp32h2, esp32s3))]
            {
                match CHANNEL {
                    0 => systimer
                        .comp0_load()
                        .write(|w| w.timer_comp0_load().set_bit()),
                    1 => systimer
                        .comp1_load()
                        .write(|w| w.timer_comp1_load().set_bit()),
                    2 => systimer
                        .comp2_load()
                        .write(|w| w.timer_comp2_load().set_bit()),
                    _ => unreachable!(),
                }

                systimer.conf().modify(|_r, w| match CHANNEL {
                    0 => w.target0_work_en().set_bit(),
                    1 => w.target1_work_en().set_bit(),
                    2 => w.target2_work_en().set_bit(),
                    _ => unreachable!(),
                });
            }

            #[cfg(esp32s2)]
            tconf.modify(|_r, w| match CHANNEL {
                0 => w.target0_work_en().set_bit(),
                1 => w.target0_work_en().set_bit(),
                2 => w.target0_work_en().set_bit(),
                _ => unreachable!(),
            });
        }
    }
}

impl<const CHANNEL: u8> Alarm<Target, CHANNEL> {
    pub fn set_target(&self, timestamp: u64) {
        self.configure(|tconf, hi, lo| unsafe {
            tconf.write(|w| w.target0_period_mode().clear_bit()); // target mode
            hi.write(|w| w.timer_target0_hi().bits((timestamp >> 32) as u32));
            lo.write(|w| w.timer_target0_lo().bits((timestamp & 0xFFFF_FFFF) as u32));
        })
    }

    pub fn into_periodic(self) -> Alarm<Periodic, CHANNEL> {
        Alarm { _pd: PhantomData }
    }
}

impl<const CHANNEL: u8> Alarm<Periodic, CHANNEL> {
    pub fn set_period(&self, period: MicrosDurationU32) {
        let us = period.ticks();
        let ticks = us * (SystemTimer::TICKS_PER_SECOND / 1_000_000) as u32;

        self.configure(|tconf, hi, lo| unsafe {
            tconf.write(|w| {
                w.target0_period_mode()
                    .set_bit()
                    .target0_period()
                    .bits(ticks)
            });
            hi.write(|w| w.timer_target0_hi().bits(0));
            lo.write(|w| w.timer_target0_lo().bits(0));
        });
    }

    pub fn into_target(self) -> Alarm<Target, CHANNEL> {
        Alarm { _pd: PhantomData }
    }
}

impl<T> Alarm<T, 0> {
    pub const unsafe fn conjure() -> Self {
        Self { _pd: PhantomData }
    }
}

impl<T> Alarm<T, 1> {
    pub const unsafe fn conjure() -> Self {
        Self { _pd: PhantomData }
    }
}

impl<T> Alarm<T, 2> {
    pub const unsafe fn conjure() -> Self {
        Self { _pd: PhantomData }
    }
}

// FIXME: The `embedded_hal_async::delay::DelayUs` trait implementation
//        interferes with the embassy time driver, which also uses the
//        `SYSTIMER` peripheral. Until we come up with a solution, do not
//        implement this trait if the `embassy-time-systick` feature is enabled.
// #[cfg(all(feature = "async", not(feature = "embassy-time-systick")))]
// HACK: disable `asynch` module *always* until we come up with a solution
#[cfg(not(systimer))]
mod asynch {
    use core::{
        pin::Pin,
        task::{Context, Poll},
    };

    use embassy_sync::waitqueue::AtomicWaker;
    use procmacros::interrupt;

    use super::*;

    const NUM_ALARMS: usize = 3;

    const INIT: AtomicWaker = AtomicWaker::new();
    static WAKERS: [AtomicWaker; NUM_ALARMS] = [INIT; NUM_ALARMS];

    pub(crate) struct AlarmFuture<'a, const N: u8> {
        phantom: PhantomData<&'a Alarm<Periodic, N>>,
    }

    impl<'a, const N: u8> AlarmFuture<'a, N> {
        pub(crate) fn new(alarm: &'a Alarm<Periodic, N>) -> Self {
            alarm.clear_interrupt();
            alarm.enable_interrupt(true);

            Self {
                phantom: PhantomData,
            }
        }

        fn event_bit_is_clear(&self) -> bool {
            let r = unsafe { &*crate::peripherals::SYSTIMER::PTR }
                .int_ena
                .read();

            match N {
                0 => r.target0_int_ena().bit_is_clear(),
                1 => r.target1_int_ena().bit_is_clear(),
                2 => r.target2_int_ena().bit_is_clear(),
                _ => unreachable!(),
            }
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

    impl<const CHANNEL: u8> embedded_hal_async::delay::DelayUs for Alarm<Periodic, CHANNEL> {
        async fn delay_us(&mut self, us: u32) {
            let period = MicrosDurationU32::from_ticks(us);
            self.set_period(period);

            AlarmFuture::new(self).await;
        }

        async fn delay_ms(&mut self, ms: u32) {
            for _ in 0..ms {
                self.delay_us(1000).await;
            }
        }
    }

    #[interrupt]
    fn SYSTIMER_TARGET0() {
        unsafe { &*crate::peripherals::SYSTIMER::PTR }
            .int_ena
            .modify(|_, w| w.target0_int_ena().clear_bit());

        WAKERS[0].wake();
    }

    #[interrupt]
    fn SYSTIMER_TARGET1() {
        unsafe { &*crate::peripherals::SYSTIMER::PTR }
            .int_ena
            .modify(|_, w| w.target1_int_ena().clear_bit());

        WAKERS[1].wake();
    }

    #[interrupt]
    fn SYSTIMER_TARGET2() {
        unsafe { &*crate::peripherals::SYSTIMER::PTR }
            .int_ena
            .modify(|_, w| w.target2_int_ena().clear_bit());

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
    //! alarm0.set_period(1u32.secs());
    //!
    //! let timer_event = SysTimerEtmEvent::new(&mut alarm0);
    //! ```

    use super::*;

    /// An ETM controlled SYSTIMER event
    pub struct SysTimerEtmEvent<'a, M, const N: u8> {
        alarm: &'a mut Alarm<M, N>,
    }

    impl<'a, M, const N: u8> SysTimerEtmEvent<'a, M, N> {
        /// Creates an ETM event from the given [Alarm]
        pub fn new(alarm: &'a mut Alarm<M, N>) -> Self {
            Self { alarm }
        }

        /// Execute closure f with mutable access to the wrapped [Alarm].
        pub fn with<R>(&self, f: impl FnOnce(&&'a mut Alarm<M, N>) -> R) -> R {
            let alarm = &self.alarm;
            f(alarm)
        }
    }

    impl<'a, M, const N: u8> crate::etm::private::Sealed for SysTimerEtmEvent<'a, M, N> {}

    impl<'a, M, const N: u8> crate::etm::EtmEvent for SysTimerEtmEvent<'a, M, N> {
        fn id(&self) -> u8 {
            50 + N
        }
    }

    pub(super) fn enable_etm() {
        let syst = unsafe { crate::peripherals::SYSTIMER::steal() };
        syst.conf().modify(|_, w| w.etm_en().set_bit());
    }
}
