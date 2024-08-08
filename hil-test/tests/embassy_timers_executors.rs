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
//!   immediatelly in the case of the time scheduling was already in the past
//!   (timestamp being too big)

// esp32c2 is disabled currently as it fails
//% CHIPS: esp32 esp32c3 esp32c6 esp32h2 esp32s3

#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_time::{Duration, Ticker, Timer};
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    peripherals::Peripherals,
    prelude::*,
    system::SystemControl,
    timer::{timg::TimerGroup, ErasedTimer, OneShotTimer, PeriodicTimer},
};
#[cfg(not(feature = "esp32"))]
use esp_hal::{
    interrupt::Priority,
    timer::systimer::{Alarm, FrozenUnit, Periodic, SystemTimer, Target},
};
#[cfg(not(feature = "esp32"))]
use esp_hal_embassy::InterruptExecutor;
#[cfg(not(feature = "esp32"))]
use static_cell::StaticCell;

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

unsafe fn __make_static<T>(t: &mut T) -> &'static mut T {
    ::core::mem::transmute(t)
}

// we need to tell the test framework somehow, if the async test passed or
// failed, this mod is the list of functions that are spawned as an actual tests
mod task_invokers {
    use test_helpers::*;

    use crate::*;
    #[embassy_executor::task]
    pub async fn test_one_shot_timg_invoker() {
        let outcome;
        {
            outcome = test_helpers::test_one_shot_timg().await;
        }
        embedded_test::export::check_outcome(outcome);
    }

    #[embassy_executor::task]
    #[cfg(not(feature = "esp32"))]
    pub async fn test_one_shot_systimer_invoker() {
        let outcome;
        {
            outcome = task_invokers::test_one_shot_systimer().await;
        }
        embedded_test::export::check_outcome(outcome);
    }

    #[embassy_executor::task]
    pub async fn test_join_timg_invoker() {
        let outcome;
        {
            outcome = test_join_timg().await;
        }
        embedded_test::export::check_outcome(outcome);
    }

    #[embassy_executor::task]
    #[cfg(not(feature = "esp32"))]
    pub async fn test_join_systimer_invoker() {
        let outcome;
        {
            outcome = test_join_systimer().await;
        }
        embedded_test::export::check_outcome(outcome);
    }

    #[embassy_executor::task]
    #[cfg(not(feature = "esp32"))]
    pub async fn test_interrupt_executor_invoker() {
        let outcome;
        {
            outcome = test_interrupt_executor().await;
        }
        embedded_test::export::check_outcome(outcome);
    }

    #[embassy_executor::task]
    pub async fn test_tick_and_increment_invoker() {
        let outcome;
        {
            outcome = tick_and_increment().await;
        }
        embedded_test::export::check_outcome(outcome);
    }
}

// List of the functions that are ACTUALLY TESTS but are called in the invokers
mod test_helpers {
    #[cfg(not(feature = "esp32"))]
    use esp_hal::timer::systimer::Target;

    use crate::*;
    pub async fn test_one_shot_timg() {
        let peripherals = unsafe { Peripherals::steal() };
        let system = SystemControl::new(peripherals.SYSTEM);
        let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

        let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks);
        let timer0: ErasedTimer = timg0.timer0.into();
        let timers = [OneShotTimer::new(timer0)];
        let timers = mk_static!([OneShotTimer<ErasedTimer>; 1], timers);
        esp_hal_embassy::init(&clocks, timers);

        let t1 = esp_hal::time::current_time();
        Timer::after_millis(500).await;
        task300ms().await;
        let t2 = esp_hal::time::current_time();

        assert!(t2 > t1);
        assert!((t2 - t1).to_millis() >= 500u64);
    }

    #[cfg(not(feature = "esp32"))]
    pub async fn test_one_shot_systimer() {
        let peripherals = unsafe { Peripherals::steal() };
        let system = SystemControl::new(peripherals.SYSTEM);
        let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

        let systimer = SystemTimer::new(peripherals.SYSTIMER).split::<Target>();
        let alarm0: ErasedTimer = systimer.alarm0.into();
        let timers = [OneShotTimer::new(alarm0)];
        let timers = mk_static!([OneShotTimer<ErasedTimer>; 1], timers);
        esp_hal_embassy::init(&clocks, timers);

        let t1 = esp_hal::time::current_time();
        Timer::after_millis(500).await;
        task300ms().await;
        let t2 = esp_hal::time::current_time();

        assert!(t2 > t1);
        assert!((t2 - t1).to_millis() >= 500u64);
    }

    pub async fn test_join_timg() {
        let peripherals = unsafe { Peripherals::steal() };
        let system = SystemControl::new(peripherals.SYSTEM);
        let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

        let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks);
        let timer0: ErasedTimer = timg0.timer0.into();
        let timers = [OneShotTimer::new(timer0)];
        let timers = mk_static!([OneShotTimer<ErasedTimer>; 1], timers);
        esp_hal_embassy::init(&clocks, timers);

        let t1 = esp_hal::time::current_time();
        embassy_futures::join::join(task500ms(), task300ms()).await;
        task500ms().await;
        let t2 = esp_hal::time::current_time();

        assert!(t2 > t1);
        assert!((t2 - t1).to_millis() >= 1_000u64);
    }

    #[cfg(not(feature = "esp32"))]
    pub async fn test_join_systimer() {
        let peripherals = unsafe { Peripherals::steal() };
        let system = SystemControl::new(peripherals.SYSTEM);
        let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

        let systimer = SystemTimer::new(peripherals.SYSTIMER).split::<Target>();
        let alarm0: ErasedTimer = systimer.alarm0.into();
        let timers = [OneShotTimer::new(alarm0)];
        let timers = mk_static!([OneShotTimer<ErasedTimer>; 1], timers);
        esp_hal_embassy::init(&clocks, timers);

        let t1 = esp_hal::time::current_time();
        embassy_futures::join::join(task500ms(), task300ms()).await;
        task500ms().await;
        let t2 = esp_hal::time::current_time();

        assert!(t2 > t1);
        assert!((t2 - t1).to_millis() >= 1_000u64);
    }

    #[cfg(not(feature = "esp32"))]
    pub async fn test_interrupt_executor() {
        let mut ticker = Ticker::every(Duration::from_millis(300));

        let t1 = esp_hal::time::current_time();
        ticker.next().await;
        ticker.next().await;
        ticker.next().await;
        let t2 = esp_hal::time::current_time();

        assert!(t2 > t1);
        assert!((t2 - t1).to_millis() >= 900u64);
    }

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

    pub async fn task500ms() {
        Timer::after_millis(500).await;
    }

    pub async fn task300ms() {
        Timer::after_millis(300).await;
    }

    #[embassy_executor::task]
    pub async fn e_task300ms() {
        task300ms().await;
    }
}

#[cfg(test)]
#[embedded_test::tests(executor = esp_hal_embassy::Executor::new())]
mod test {
    use super::*;
    use crate::{task_invokers::*, test_helpers::*};

    #[test]
    #[timeout(3)]
    fn run_test_one_shot_timg() {
        let mut executor = esp_hal_embassy::Executor::new();
        let executor = unsafe { __make_static(&mut executor) };
        executor.run(|spawner| {
            spawner.must_spawn(task_invokers::test_one_shot_timg_invoker());
        });
    }

    #[test]
    #[timeout(3)]
    fn run_test_periodic_timg() {
        let peripherals = Peripherals::take();
        let system = SystemControl::new(peripherals.SYSTEM);
        let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

        let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks);

        let mut periodic = PeriodicTimer::new(timg0.timer0);

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

    #[test]
    #[timeout(3)]
    #[cfg(not(feature = "esp32"))]
    fn run_test_one_shot_systimer() {
        let mut executor = esp_hal_embassy::Executor::new();
        let executor = unsafe { __make_static(&mut executor) };
        executor.run(|spawner| {
            spawner.must_spawn(test_one_shot_systimer_invoker());
        });
    }

    #[test]
    #[timeout(3)]
    #[cfg(not(feature = "esp32"))]
    fn run_test_periodic_systimer() {
        let peripherals = Peripherals::take();

        let systimer = SystemTimer::new(peripherals.SYSTIMER).split::<Periodic>();

        let mut periodic = PeriodicTimer::new(systimer.alarm0);

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

    #[test]
    #[timeout(3)]
    fn run_test_periodic_oneshot_timg() {
        let mut peripherals = Peripherals::take();
        let system = SystemControl::new(peripherals.SYSTEM);
        let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

        let mut timg0 = TimerGroup::new(&mut peripherals.TIMG0, &clocks);

        let mut periodic = PeriodicTimer::new(&mut timg0.timer0);

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

        core::mem::drop(periodic);

        let timg0 = TimerGroup::new(&mut peripherals.TIMG0, &clocks);

        let timer0 = OneShotTimer::new(timg0.timer0);

        let t1 = esp_hal::time::current_time();
        timer0.delay_millis(500);
        let t2 = esp_hal::time::current_time();

        assert!(t2 > t1);
        assert!((t2 - t1).to_millis() >= 500u64);
    }

    #[test]
    #[timeout(3)]
    #[cfg(not(feature = "esp32"))]
    fn run_test_periodic_oneshot_systimer() {
        let mut peripherals = Peripherals::take();

        let mut systimer = SystemTimer::new(&mut peripherals.SYSTIMER);

        let unit = FrozenUnit::new(&mut systimer.unit0);
        let mut alarm: Alarm<'_, Periodic, _, 0, 0> = Alarm::new(systimer.comparator0, &unit);
        let mut periodic = PeriodicTimer::new(&mut alarm);

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

        core::mem::drop(periodic);

        let mut systimer = SystemTimer::new(&mut peripherals.SYSTIMER);

        let unit = FrozenUnit::new(&mut systimer.unit0);
        let alarm: Alarm<'_, Target, _, 0, 0> = Alarm::new(systimer.comparator0, &unit);
        let timer0 = OneShotTimer::new(alarm);

        let t1 = esp_hal::time::current_time();
        timer0.delay_millis(500);
        let t2 = esp_hal::time::current_time();

        assert!(t2 > t1);
        assert!((t2 - t1).to_millis() >= 500u64);
    }

    #[test]
    #[timeout(3)]
    fn run_test_join_timg() {
        let mut executor = esp_hal_embassy::Executor::new();
        let executor = unsafe { __make_static(&mut executor) };
        executor.run(|spawner| {
            spawner.must_spawn(test_join_timg_invoker());
        });
    }

    #[test]
    #[timeout(3)]
    #[cfg(not(feature = "esp32"))]
    fn run_test_join_systimer() {
        let mut executor = esp_hal_embassy::Executor::new();
        let executor = unsafe { __make_static(&mut executor) };
        executor.run(|spawner| {
            spawner.must_spawn(test_join_systimer_invoker());
        });
    }

    #[test]
    #[timeout(3)]
    #[cfg(not(feature = "esp32"))]
    async fn run_test_interrupt_executor() {
        let spawner = embassy_executor::Spawner::for_current_executor().await;

        let peripherals = Peripherals::take();
        let system = SystemControl::new(peripherals.SYSTEM);
        let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

        let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks);
        let timer0: ErasedTimer = timg0.timer0.into();
        let timer0 = OneShotTimer::new(timer0);

        let timer1 = {
            let systimer =
                esp_hal::timer::systimer::SystemTimer::new(peripherals.SYSTIMER).split::<Target>();
            let alarm0: ErasedTimer = systimer.alarm0.into();
            OneShotTimer::new(alarm0)
        };

        let timers = [timer0, timer1];
        let timers = mk_static!([OneShotTimer<ErasedTimer>; 2], timers);
        esp_hal_embassy::init(&clocks, timers);

        static EXECUTOR: StaticCell<InterruptExecutor<2>> = StaticCell::new();
        let executor =
            InterruptExecutor::new(system.software_interrupt_control.software_interrupt2);
        let executor = EXECUTOR.init(executor);

        let spawner_int = executor.start(Priority::Priority3);
        spawner_int.must_spawn(test_interrupt_executor_invoker());

        spawner.must_spawn(e_task300ms());

        // we need to delay so the e_task300ms() could be spawned
        task500ms().await;

        loop {}
    }

    #[test]
    #[timeout(3)]
    async fn run_tick_test_timg() {
        let spawner = embassy_executor::Spawner::for_current_executor().await;

        let peripherals = unsafe { Peripherals::steal() };
        let system = SystemControl::new(peripherals.SYSTEM);
        let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

        let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks);
        let timer0: ErasedTimer = timg0.timer0.into();
        let timers = [OneShotTimer::new(timer0)];
        let timers = mk_static!([OneShotTimer<ErasedTimer>; 1], timers);
        esp_hal_embassy::init(&clocks, timers);

        spawner.must_spawn(tick());
        tick_and_increment().await;
    }
}
