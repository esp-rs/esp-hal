//! Embassy timer and executor Test

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: integrated-timers
//% FEATURES: generic-queue

#![no_std]
#![no_main]

use embassy_time::{Duration, Ticker, Timer};
use esp_hal::{
    clock::Clocks,
    interrupt::software::SoftwareInterruptControl,
    peripherals::Peripherals,
    prelude::*,
    timer::{timg::TimerGroup, ErasedTimer, OneShotTimer, PeriodicTimer},
};
#[cfg(not(feature = "esp32"))]
use esp_hal::{
    interrupt::Priority,
    timer::systimer::{Alarm, FrozenUnit, Periodic, SystemTimer, Target},
};
#[cfg(not(feature = "esp32"))]
use esp_hal_embassy::InterruptExecutor;
use hil_test as _;

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

    #[embassy_executor::task]
    pub async fn e_task30ms() {
        Timer::after_millis(30).await;
    }
}

mod test_cases {
    use esp_hal::peripheral::Peripheral;

    use super::*;

    pub async fn run_test_one_shot_async() {
        let t1 = esp_hal::time::current_time();
        Timer::after_millis(50).await;
        Timer::after_millis(30).await;
        let t2 = esp_hal::time::current_time();

        assert!(t2 > t1, "t2: {:?}, t1: {:?}", t2, t1);
        assert!(
            (t2 - t1).to_millis() >= 80u64,
            "diff: {:?}",
            (t2 - t1).to_millis()
        );
    }

    pub fn run_test_periodic_timer<T: esp_hal::timer::Timer>(timer: impl Peripheral<P = T>) {
        let mut periodic = PeriodicTimer::new(timer);

        let t1 = esp_hal::time::current_time();
        periodic.start(100.millis()).unwrap();

        nb::block!(periodic.wait()).unwrap();
        let t2 = esp_hal::time::current_time();

        assert!(t2 > t1, "t2: {:?}, t1: {:?}", t2, t1);
        assert!(
            (t2 - t1).to_millis() >= 100u64,
            "diff: {:?}",
            (t2 - t1).to_millis()
        );
    }

    pub fn run_test_oneshot_timer<T: esp_hal::timer::Timer>(timer: impl Peripheral<P = T>) {
        let timer = OneShotTimer::new(timer);

        let t1 = esp_hal::time::current_time();
        timer.delay_millis(50);
        let t2 = esp_hal::time::current_time();

        assert!(t2 > t1, "t2: {:?}, t1: {:?}", t2, t1);
        assert!(
            (t2 - t1).to_millis() >= 50u64,
            "diff: {:?}",
            (t2 - t1).to_millis()
        );
    }

    pub async fn run_join_test() {
        let t1 = esp_hal::time::current_time();
        embassy_futures::join::join(Timer::after_millis(50), Timer::after_millis(30)).await;
        Timer::after_millis(50).await;
        let t2 = esp_hal::time::current_time();

        assert!(t2 > t1, "t2: {:?}, t1: {:?}", t2, t1);
        assert!(
            (t2 - t1).to_millis() >= 100u64,
            "diff: {:?}",
            (t2 - t1).to_millis()
        );
    }
}

fn set_up_embassy_with_timg0(peripherals: Peripherals, clocks: Clocks<'static>) {
    let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    esp_hal_embassy::init(&clocks, timg0.timer0);
}

#[cfg(not(feature = "esp32"))]
fn set_up_embassy_with_systimer(peripherals: Peripherals, clocks: Clocks<'static>) {
    let systimer = SystemTimer::new(peripherals.SYSTIMER).split::<Target>();
    esp_hal_embassy::init(&clocks, systimer.alarm0);
}

#[cfg(test)]
#[embedded_test::tests(executor = esp_hal_embassy::Executor::new())]
mod test {
    use super::*;
    use crate::{test_cases::*, test_helpers::*};

    #[init]
    fn init() -> (Peripherals, Clocks<'static>) {
        esp_hal::init(esp_hal::Config::default())
    }

    #[test]
    #[timeout(3)]
    async fn test_one_shot_timg((peripherals, clocks): (Peripherals, Clocks<'static>)) {
        set_up_embassy_with_timg0(peripherals, clocks);

        run_test_one_shot_async().await;
    }

    #[test]
    #[timeout(3)]
    #[cfg(not(feature = "esp32"))]
    async fn test_one_shot_systimer((peripherals, clocks): (Peripherals, Clocks<'static>)) {
        set_up_embassy_with_systimer(peripherals, clocks);

        run_test_one_shot_async().await;
    }

    #[test]
    #[timeout(3)]
    fn test_periodic_timg((peripherals, clocks): (Peripherals, Clocks<'static>)) {
        let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks);

        run_test_periodic_timer(timg0.timer0);
    }

    #[test]
    #[timeout(3)]
    #[cfg(not(feature = "esp32"))]
    fn test_periodic_systimer((peripherals, _clocks): (Peripherals, Clocks<'static>)) {
        let systimer = SystemTimer::new(peripherals.SYSTIMER).split::<Periodic>();

        run_test_periodic_timer(systimer.alarm0);
    }

    #[test]
    #[timeout(3)]
    fn test_periodic_oneshot_timg((mut peripherals, clocks): (Peripherals, Clocks<'static>)) {
        let mut timg0 = TimerGroup::new(&mut peripherals.TIMG0, &clocks);
        run_test_periodic_timer(&mut timg0.timer0);

        let mut timg0 = TimerGroup::new(&mut peripherals.TIMG0, &clocks);
        run_test_oneshot_timer(&mut timg0.timer0);
    }

    #[test]
    #[timeout(3)]
    #[cfg(not(feature = "esp32"))]
    fn test_periodic_oneshot_systimer((mut peripherals, _clocks): (Peripherals, Clocks<'static>)) {
        let mut systimer = SystemTimer::new(&mut peripherals.SYSTIMER);
        let unit = FrozenUnit::new(&mut systimer.unit0);
        let mut alarm: Alarm<'_, Periodic, _, _, _> = Alarm::new(systimer.comparator0, &unit);
        run_test_periodic_timer(&mut alarm);

        let mut systimer = SystemTimer::new(&mut peripherals.SYSTIMER);
        let unit = FrozenUnit::new(&mut systimer.unit0);
        let mut alarm: Alarm<'_, Target, _, _, _> = Alarm::new(systimer.comparator0, &unit);
        run_test_oneshot_timer(&mut alarm);
    }

    #[test]
    #[timeout(3)]
    async fn test_join_timg((peripherals, clocks): (Peripherals, Clocks<'static>)) {
        set_up_embassy_with_timg0(peripherals, clocks);

        run_join_test().await;
    }

    #[test]
    #[timeout(3)]
    #[cfg(not(feature = "esp32"))]
    async fn test_join_systimer((peripherals, clocks): (Peripherals, Clocks<'static>)) {
        set_up_embassy_with_systimer(peripherals, clocks);

        run_join_test().await;
    }

    /// Test that the ticker works in tasks ran by the interrupt executors.
    #[test]
    #[timeout(3)]
    #[cfg(not(feature = "esp32"))]
    async fn test_interrupt_executor((peripherals, clocks): (Peripherals, Clocks<'static>)) {
        let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks);
        let timer0: ErasedTimer = timg0.timer0.into();
        let timer0 = OneShotTimer::new(timer0);

        let systimer = SystemTimer::new(peripherals.SYSTIMER).split::<Target>();
        let alarm0: ErasedTimer = systimer.alarm0.into();
        let timer1 = OneShotTimer::new(alarm0);

        let timers = mk_static!([OneShotTimer<ErasedTimer>; 2], [timer0, timer1]);
        esp_hal_embassy::init(&clocks, timers);

        let sw_ints = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);

        let executor = mk_static!(
            InterruptExecutor<2>,
            InterruptExecutor::new(sw_ints.software_interrupt2)
        );

        #[embassy_executor::task]
        #[cfg(not(feature = "esp32"))]
        async fn test_interrupt_executor_invoker() {
            let outcome = async {
                let mut ticker = Ticker::every(Duration::from_millis(30));

                let t1 = esp_hal::time::current_time();
                ticker.next().await;
                ticker.next().await;
                ticker.next().await;
                let t2 = esp_hal::time::current_time();

                assert!(t2 > t1, "t2: {:?}, t1: {:?}", t2, t1);
                assert!(
                    (t2 - t1).to_micros() >= 85000u64,
                    "diff: {:?}",
                    (t2 - t1).to_micros()
                );
            };

            embedded_test::export::check_outcome(outcome.await);
        }

        let spawner_int = executor.start(Priority::Priority3);
        spawner_int.must_spawn(test_interrupt_executor_invoker());

        let spawner = embassy_executor::Spawner::for_current_executor().await;
        spawner.must_spawn(e_task30ms());

        // The test ends once the interrupt executor's task has finished
        loop {}
    }

    /// Test that timg0 and systimer don't have vastly different tick rates.
    #[test]
    #[timeout(3)]
    async fn tick_test_timer_tick_rates((peripherals, clocks): (Peripherals, Clocks<'static>)) {
        set_up_embassy_with_timg0(peripherals, clocks);

        // We are retrying 5 times because probe-rs polling RTT may introduce some
        // jitter.
        for _ in 0..5 {
            let t1 = esp_hal::time::current_time();

            let mut ticker = Ticker::every(Duration::from_hz(100_000));
            for _ in 0..2000 {
                ticker.next().await;
            }
            let t2 = esp_hal::time::current_time();

            assert!(t2 > t1, "t2: {:?}, t1: {:?}", t2, t1);
            let duration = (t2 - t1).to_micros();

            assert!(duration >= 19000, "diff: {:?}", (t2 - t1).to_micros());
            if duration <= 21000 {
                return;
            }
        }

        assert!(false, "Test failed after 5 retries");
    }
}
