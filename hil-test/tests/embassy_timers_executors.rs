//! Embassy timer and executor Test

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: integrated-timers
//% FEATURES: generic-queue

#![no_std]
#![no_main]

use embassy_time::{Duration, Ticker, Timer};
#[cfg(not(feature = "esp32"))]
use esp_hal::{
    interrupt::software::SoftwareInterruptControl,
    interrupt::Priority,
    timer::systimer::SystemTimer,
    timer::AnyTimer,
};
use esp_hal::{
    peripherals::Peripherals,
    time::ExtU64,
    timer::{timg::TimerGroup, OneShotTimer, PeriodicTimer},
};
#[cfg(not(feature = "esp32"))]
use esp_hal_embassy::InterruptExecutor;
use hil_test as _;

#[cfg(not(feature = "esp32"))]
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
        let t1 = esp_hal::time::now();
        Timer::after_millis(50).await;
        Timer::after_millis(30).await;
        let t2 = esp_hal::time::now();

        assert!(t2 > t1, "t2: {:?}, t1: {:?}", t2, t1);
        assert!(
            (t2 - t1).to_millis() >= 80u64,
            "diff: {:?}",
            (t2 - t1).to_millis()
        );
    }

    pub fn run_test_periodic_timer<T: esp_hal::timer::Timer>(timer: impl Peripheral<P = T>) {
        let mut periodic = PeriodicTimer::new(timer);

        let t1 = esp_hal::time::now();
        periodic.start(100.millis()).unwrap();

        nb::block!(periodic.wait()).unwrap();
        let t2 = esp_hal::time::now();

        assert!(t2 > t1, "t2: {:?}, t1: {:?}", t2, t1);
        assert!(
            (t2 - t1).to_millis() >= 100u64,
            "diff: {:?}",
            (t2 - t1).to_millis()
        );
    }

    pub fn run_test_oneshot_timer<T: esp_hal::timer::Timer>(timer: impl Peripheral<P = T>) {
        let mut timer = OneShotTimer::new(timer);

        let t1 = esp_hal::time::now();
        timer.delay_millis(50);
        let t2 = esp_hal::time::now();

        assert!(t2 > t1, "t2: {:?}, t1: {:?}", t2, t1);
        assert!(
            (t2 - t1).to_millis() >= 50u64,
            "diff: {:?}",
            (t2 - t1).to_millis()
        );
    }

    pub async fn run_join_test() {
        let t1 = esp_hal::time::now();
        embassy_futures::join::join(Timer::after_millis(50), Timer::after_millis(30)).await;
        Timer::after_millis(50).await;
        let t2 = esp_hal::time::now();

        assert!(t2 > t1, "t2: {:?}, t1: {:?}", t2, t1);
        assert!(
            (t2 - t1).to_millis() >= 100u64,
            "diff: {:?}",
            (t2 - t1).to_millis()
        );
    }
}

fn set_up_embassy_with_timg0(peripherals: Peripherals) {
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);
}

#[cfg(not(feature = "esp32"))]
fn set_up_embassy_with_systimer(peripherals: Peripherals) {
    let systimer = SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(systimer.alarm0);
}

#[cfg(test)]
#[embedded_test::tests(default_timeout = 3, executor = esp_hal_embassy::Executor::new())]
mod test {
    use super::*;
    use crate::test_cases::*;
    #[cfg(not(feature = "esp32"))]
    use crate::test_helpers::*;

    #[init]
    fn init() -> Peripherals {
        esp_hal::init(esp_hal::Config::default())
    }

    #[test]
    async fn test_one_shot_timg(peripherals: Peripherals) {
        set_up_embassy_with_timg0(peripherals);

        run_test_one_shot_async().await;
    }

    #[test]
    #[cfg(not(feature = "esp32"))]
    async fn test_one_shot_systimer(peripherals: Peripherals) {
        set_up_embassy_with_systimer(peripherals);

        run_test_one_shot_async().await;
    }

    #[test]
    fn test_periodic_timg(peripherals: Peripherals) {
        let timg0 = TimerGroup::new(peripherals.TIMG0);

        run_test_periodic_timer(timg0.timer0);
    }

    #[test]
    #[cfg(not(feature = "esp32"))]
    fn test_periodic_systimer(peripherals: Peripherals) {
        let systimer = SystemTimer::new(peripherals.SYSTIMER);

        run_test_periodic_timer(systimer.alarm0);
    }

    #[test]
    fn test_periodic_oneshot_timg(peripherals: Peripherals) {
        let mut timg0 = TimerGroup::new(peripherals.TIMG0);
        run_test_periodic_timer(&mut timg0.timer0);
        run_test_oneshot_timer(&mut timg0.timer0);
    }

    #[test]
    #[cfg(not(feature = "esp32"))]
    fn test_periodic_oneshot_systimer(peripherals: Peripherals) {
        let mut systimer = SystemTimer::new(peripherals.SYSTIMER);
        run_test_periodic_timer(&mut systimer.alarm0);
        run_test_oneshot_timer(&mut systimer.alarm0);
    }

    #[test]
    async fn test_join_timg(peripherals: Peripherals) {
        set_up_embassy_with_timg0(peripherals);

        run_join_test().await;
    }

    #[test]
    #[cfg(not(feature = "esp32"))]
    async fn test_join_systimer(peripherals: Peripherals) {
        set_up_embassy_with_systimer(peripherals);

        run_join_test().await;
    }

    /// Test that the ticker works in tasks ran by the interrupt executors.
    #[test]
    #[cfg(not(feature = "esp32"))]
    async fn test_interrupt_executor(peripherals: Peripherals) {
        let timg0 = TimerGroup::new(peripherals.TIMG0);
        let timer0: AnyTimer = timg0.timer0.into();

        let systimer = SystemTimer::new(peripherals.SYSTIMER);
        let alarm0: AnyTimer = systimer.alarm0.into();

        esp_hal_embassy::init([timer0, alarm0]);

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

                let t1 = esp_hal::time::now();
                ticker.next().await;
                ticker.next().await;
                ticker.next().await;
                let t2 = esp_hal::time::now();

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
    async fn tick_test_timer_tick_rates(peripherals: Peripherals) {
        set_up_embassy_with_timg0(peripherals);

        // We are retrying 5 times because probe-rs polling RTT may introduce some
        // jitter.
        for _ in 0..5 {
            let t1 = esp_hal::time::now();

            let mut ticker = Ticker::every(Duration::from_hz(100_000));
            for _ in 0..2000 {
                ticker.next().await;
            }
            let t2 = esp_hal::time::now();

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
