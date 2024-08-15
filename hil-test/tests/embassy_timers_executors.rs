//! Embassy timer and executor Test
//!
//! Description of implemented tests:
//! - run_test_one_shot_timg(): Tests Timg configured as OneShotTimer
//! - run_test_periodic_timg(): Tests Timg configured as PeriodicTimer
//! - run_test_one_shot_systimer(): Tests systimer configured as OneShotTimer
//! - run_test_periodic_systimer(): Tests systimer configured as PeriodicTimer
//! - run_test_periodic_oneshot_timg(): Tests Timg configured as PeriodicTimer
//!   and then reconfigured as OneShotTimer
//! - run_test_periodic_oneshot_systimer(): Tests systimer configured as
//!   PeriodicTimer and then reconfigured as OneShotTimer
//! - run_test_join_timg(): Tests Timg configured as OneShotTimer and wait on
//!   two different timeouts via join
//! - run_test_join_systimer(): Tests systimer configured as OneShotTimer and
//!   wait on two different timeouts via join
//! - run_test_interrupt_executor(): Tests InterruptExecutor and Thread
//!   (default) executor in parallel
//! - run_tick_test_timg(): Tests Timg configured as OneShotTimer if it fires
//!   immediately in the case of the time scheduling was already in the past
//!   (timestamp being too big)

// esp32c2 is disabled currently as it fails
//% CHIPS: esp32 esp32c3 esp32c6 esp32h2 esp32s3

#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_time::{Duration, Ticker, Timer};
use esp_backtrace as _;
use esp_hal::{
    clock::{Clocks, ClockControl},
    peripherals::Peripherals,
    prelude::*,
    system::SystemControl,
    timer::{timg::TimerGroup, ErasedTimer, OneShotTimer, PeriodicTimer},
};
#[cfg(not(feature = "esp32"))]
use esp_hal::{interrupt::Priority, timer::systimer::SystemTimer};
#[cfg(not(feature = "esp32"))]
use esp_hal_embassy::InterruptExecutor;
use embassy_futures::yield_now;

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

// List of the functions that are ACTUALLY TESTS but are called in the invokers
mod test_helpers {
    use super::*;

    pub async fn tick_and_increment() {
        const HZ: u64 = 100_000u64;
        let mut counter = 0;
        let mut ticker = Ticker::every(Duration::from_hz(HZ));

        let t1 = esp_hal::time::current_time();
        let t2;

        loop {
            ticker.next().await;
            counter += 1;

            if counter > 100_000 {
                t2 = esp_hal::time::current_time();
                break;
            }
        }

        assert!(t2 > t1);
        assert!((t2 - t1).to_millis() >= 1000u64);
        assert!((t2 - t1).to_millis() <= 1300u64);
    }

    #[embassy_executor::task]
    pub async fn tick() {
        const HZ: u64 = 1000u64;
        let mut ticker = Ticker::every(Duration::from_hz(HZ));

        loop {
            ticker.next().await;
        }
    }

    #[embassy_executor::task]
    pub async fn e_task300ms() {
        Timer::after_millis(300).await;
    }
}

mod test_cases {
    use super::*;

    use esp_hal::peripheral::Peripheral;

    pub async fn run_test_one_shot_async() {
        let t1 = esp_hal::time::current_time();
        Timer::after_millis(500).await;
        Timer::after_millis(300).await;
        let t2 = esp_hal::time::current_time();

        assert!(t2 > t1);
        assert!((t2 - t1).to_millis() >= 800u64);
    }

    pub fn run_test_periodic_timer<T: esp_hal::timer::Timer>(timer: impl Peripheral<P = T>) {
        let mut periodic = PeriodicTimer::new(timer);

        let t1 = esp_hal::time::current_time();
        periodic.start(1.secs()).unwrap();

        let t2;
        loop {
            nb::block!(periodic.wait()).unwrap();
            t2 = esp_hal::time::current_time();
            break;
        }
        assert!(t2 > t1);
        assert!((t2 - t1).to_millis() >= 1_000u64);
    }

    pub fn run_test_oneshot_timer<T: esp_hal::timer::Timer>(timer: impl Peripheral<P = T>) {
        let timer = OneShotTimer::new(timer);

        let t1 = esp_hal::time::current_time();
        timer.delay_millis(500);
        let t2 = esp_hal::time::current_time();

        assert!(t2 > t1);
        assert!((t2 - t1).to_millis() >= 500u64);
    }

    pub async fn run_join_test() {
        let t1 = esp_hal::time::current_time();
        embassy_futures::join::join(Timer::after_millis(500), Timer::after_millis(300)).await;
        Timer::after_millis(500).await;
        let t2 = esp_hal::time::current_time();

        assert!(t2 > t1);
        assert!((t2 - t1).to_millis() >= 1_000u64);
    }
}

struct Resources {
    clocks: Clocks<'static>,
    timg0: esp_hal::peripherals::TIMG0,
    #[cfg(not(feature = "esp32"))]
    systimer: esp_hal::peripherals::SYSTIMER,
    software_interrupt_control: esp_hal::system::SoftwareInterruptControl,
}

impl Resources {
    fn set_up_embassy_with_timg0(self) {
        let timg0 = TimerGroup::new(self.timg0, &self.clocks);
        let timer0: ErasedTimer = timg0.timer0.into();
        let timers = mk_static!(OneShotTimer<ErasedTimer>, OneShotTimer::new(timer0));
        esp_hal_embassy::init(&self.clocks, core::slice::from_mut(timers));
    }

    #[cfg(not(feature = "esp32"))]
    fn set_up_embassy_with_systimer(self) {
        let systimer = SystemTimer::new(self.systimer);
        let alarm0: ErasedTimer = systimer.alarm0.into();
        let timers = mk_static!(OneShotTimer<ErasedTimer>, OneShotTimer::new(alarm0));
        esp_hal_embassy::init(&self.clocks, core::slice::from_mut(timers));
    }
}

#[cfg(test)]
#[embedded_test::tests(executor = esp_hal_embassy::Executor::new())]
mod test {
    use super::*;
    use crate::test_helpers::*;
    use crate::test_cases::*;

    #[init]
    fn init() -> Resources {
        let peripherals = unsafe { Peripherals::steal() };
        let system = SystemControl::new(peripherals.SYSTEM);

        Resources {
            clocks: ClockControl::boot_defaults(system.clock_control).freeze(),
            timg0: peripherals.TIMG0,
            #[cfg(not(feature = "esp32"))]
            systimer: peripherals.SYSTIMER,
            software_interrupt_control: system.software_interrupt_control,
        }
    }

    #[test]
    #[timeout(3)]
    async fn test_one_shot_timg(resources: Resources) {
        resources.set_up_embassy_with_timg0();

        run_test_one_shot_async().await;
    }

    #[test]
    #[timeout(3)]
    #[cfg(not(feature = "esp32"))]
    async fn test_one_shot_systimer(resources: Resources) {
        resources.set_up_embassy_with_systimer();

        run_test_one_shot_async().await;
    }

    #[test]
    #[timeout(3)]
    fn test_periodic_timg(resources: Resources) {
        let timg0 = TimerGroup::new(resources.timg0, &resources.clocks);

        run_test_periodic_timer(timg0.timer0);
    }

    #[test]
    #[timeout(3)]
    #[cfg(not(feature = "esp32"))]
    fn test_periodic_systimer(resources: Resources) {
        let systimer = SystemTimer::new(resources.systimer);

        run_test_periodic_timer(systimer.alarm0);
    }

    #[test]
    #[timeout(3)]
    fn test_periodic_oneshot_timg(mut resources: Resources) {
        let mut timg0 = TimerGroup::new(&mut resources.timg0, &resources.clocks);
        run_test_periodic_timer(&mut timg0.timer0);

        let mut timg0 = TimerGroup::new(&mut resources.timg0, &resources.clocks);
        run_test_oneshot_timer(&mut timg0.timer0);
    }

    #[test]
    #[timeout(3)]
    #[cfg(not(feature = "esp32"))]
    fn test_periodic_oneshot_systimer(mut resources: Resources) {
        let mut systimer = SystemTimer::new(&mut resources.systimer);
        run_test_periodic_timer(&mut systimer.alarm0);

        let mut systimer = SystemTimer::new(&mut resources.systimer);
        run_test_oneshot_timer(&mut systimer.alarm0);
    }

    #[test]
    #[timeout(3)]
    async fn test_join_timg(resources: Resources) {
        resources.set_up_embassy_with_timg0();

        run_join_test().await;
    }

    #[test]
    #[timeout(3)]
    #[cfg(not(feature = "esp32"))]
    async fn test_join_systimer(resources: Resources) {
        resources.set_up_embassy_with_systimer();

        run_join_test().await;
    }

    #[test]
    #[timeout(3)]
    #[cfg(not(feature = "esp32"))]
    async fn test_interrupt_executor(resources: Resources) {
        let timg0 = TimerGroup::new(resources.timg0, &resources.clocks);
        let timer0: ErasedTimer = timg0.timer0.into();
        let timer0 = OneShotTimer::new(timer0);

        let systimer = SystemTimer::new(resources.systimer);
        let alarm0: ErasedTimer = systimer.alarm0.into();
        let timer1 = OneShotTimer::new(alarm0);

        let timers = mk_static!([OneShotTimer<ErasedTimer>; 2], [timer0, timer1]);
        esp_hal_embassy::init(&resources.clocks, timers);

        let executor = mk_static!(
            InterruptExecutor<2>,
            InterruptExecutor::new(resources.software_interrupt_control.software_interrupt2)
        );

        #[embassy_executor::task]
        #[cfg(not(feature = "esp32"))]
        async fn test_interrupt_executor_invoker() {
            let outcome = async {
                let mut ticker = Ticker::every(Duration::from_millis(300));

                let t1 = esp_hal::time::current_time();
                ticker.next().await;
                ticker.next().await;
                ticker.next().await;
                let t2 = esp_hal::time::current_time();

                assert!(t2 > t1);
                assert!((t2 - t1).to_millis() >= 900u64);
            };

            embedded_test::export::check_outcome(outcome.await);
        }

        let spawner_int = executor.start(Priority::Priority3);
        spawner_int.must_spawn(test_interrupt_executor_invoker());

        let spawner = embassy_executor::Spawner::for_current_executor().await;
        spawner.must_spawn(e_task300ms());

        // we need to delay so the e_task300ms() could be spawned
        yield_now().await;

        // The test ends once the interrupt executor's task has finished
        loop {}
    }

    #[test]
    #[timeout(3)]
    async fn tick_test_timg(resources: Resources) {
        let spawner = embassy_executor::Spawner::for_current_executor().await;

        resources.set_up_embassy_with_timg0();

        spawner.must_spawn(tick());
        tick_and_increment().await;
    }
}
