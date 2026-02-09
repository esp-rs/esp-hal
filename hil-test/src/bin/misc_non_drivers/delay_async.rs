#[embedded_test::tests(default_timeout = 2, executor = hil_test::Executor::new())]
mod tests {
    use embedded_hal_async::delay::DelayNs;
    #[cfg(systimer_driver_supported)]
    use esp_hal::timer::systimer::SystemTimer;
    use esp_hal::{
        peripherals::Peripherals,
        timer::{OneShotTimer, timg::TimerGroup},
    };
    struct Context {
        peripherals: Peripherals,
    }

    async fn test_async_delay_ns(mut timer: impl DelayNs, duration: u32) {
        for i in 1..5 {
            let t1 = esp_hal::time::Instant::now();
            timer.delay_ns(duration).await;
            let t2 = esp_hal::time::Instant::now();

            assert!(t2 > t1);
            assert!(
                (t2 - t1).as_micros() >= duration.div_ceil(1000) as u64,
                "diff[{}]: {:?} >= {}",
                i,
                (t2 - t1).as_micros(),
                duration
            );
        }
    }

    async fn test_async_delay_us(mut timer: impl DelayNs, duration: u32) {
        for _ in 1..5 {
            let t1 = esp_hal::time::Instant::now();
            timer.delay_us(duration).await;
            let t2 = esp_hal::time::Instant::now();

            assert!(t2 > t1);
            assert!(
                (t2 - t1).as_micros() >= duration as u64,
                "diff: {:?}",
                (t2 - t1).as_micros()
            );
        }
    }

    async fn test_async_delay_ms(mut timer: impl DelayNs, duration: u32) {
        for _ in 1..5 {
            let t1 = esp_hal::time::Instant::now();
            timer.delay_ms(duration).await;
            let t2 = esp_hal::time::Instant::now();

            assert!(t2 > t1);
            assert!(
                (t2 - t1).as_millis() >= duration as u64,
                "diff: {:?}",
                (t2 - t1).as_millis()
            );
        }
    }
    #[init]
    fn init() -> Context {
        Context {
            peripherals: esp_hal::init(esp_hal::Config::default()),
        }
    }

    #[cfg(systimer_driver_supported)]
    #[test]
    async fn test_systimer_async_delay_ns(ctx: Context) {
        let alarms = SystemTimer::new(ctx.peripherals.SYSTIMER);

        test_async_delay_ns(OneShotTimer::new(alarms.alarm0).into_async(), 10_000).await;
    }

    #[cfg(timergroup_timg0)]
    #[test]
    async fn test_timg0_async_delay_ns(ctx: Context) {
        let timg0 = TimerGroup::new(ctx.peripherals.TIMG0);

        test_async_delay_ns(OneShotTimer::new(timg0.timer0).into_async(), 10_000).await;
        #[cfg(timergroup_timg_has_timer1)]
        test_async_delay_ns(OneShotTimer::new(timg0.timer1).into_async(), 10_000).await;
    }

    #[cfg(timergroup_timg1)]
    #[test]
    async fn test_timg1_async_delay_ns(ctx: Context) {
        let timg1 = TimerGroup::new(ctx.peripherals.TIMG1);

        test_async_delay_ns(OneShotTimer::new(timg1.timer0).into_async(), 10_000).await;
        #[cfg(timergroup_timg_has_timer1)]
        test_async_delay_ns(OneShotTimer::new(timg1.timer1).into_async(), 10_000).await;
    }

    #[cfg(systimer_driver_supported)]
    #[test]
    async fn test_systimer_async_delay_us(ctx: Context) {
        let alarms = SystemTimer::new(ctx.peripherals.SYSTIMER);

        test_async_delay_us(OneShotTimer::new(alarms.alarm0).into_async(), 10).await;
    }

    #[cfg(timergroup_timg0)]
    #[test]
    async fn test_timg0_async_delay_us(ctx: Context) {
        let timg0 = TimerGroup::new(ctx.peripherals.TIMG0);

        test_async_delay_us(OneShotTimer::new(timg0.timer0).into_async(), 10).await;
        #[cfg(timergroup_timg_has_timer1)]
        test_async_delay_us(OneShotTimer::new(timg0.timer1).into_async(), 10).await;
    }

    #[cfg(timergroup_timg1)]
    #[test]
    async fn test_timg1_async_delay_us(ctx: Context) {
        let timg1 = TimerGroup::new(ctx.peripherals.TIMG1);

        test_async_delay_us(OneShotTimer::new(timg1.timer0).into_async(), 10).await;
        #[cfg(timergroup_timg_has_timer1)]
        test_async_delay_us(OneShotTimer::new(timg1.timer1).into_async(), 10).await;
    }

    #[cfg(systimer_driver_supported)]
    #[test]
    async fn test_systimer_async_delay_ms(ctx: Context) {
        let alarms = SystemTimer::new(ctx.peripherals.SYSTIMER);

        test_async_delay_ms(OneShotTimer::new(alarms.alarm0).into_async(), 1).await;
    }

    #[cfg(timergroup_timg0)]
    #[test]
    async fn test_timg0_async_delay_ms(ctx: Context) {
        let timg0 = TimerGroup::new(ctx.peripherals.TIMG0);

        test_async_delay_ms(OneShotTimer::new(timg0.timer0).into_async(), 1).await;
        #[cfg(timergroup_timg_has_timer1)]
        test_async_delay_ms(OneShotTimer::new(timg0.timer1).into_async(), 1).await;
    }

    #[cfg(timergroup_timg1)]
    #[test]
    async fn test_timg1_async_delay_ms(ctx: Context) {
        let timg1 = TimerGroup::new(ctx.peripherals.TIMG1);

        test_async_delay_ms(OneShotTimer::new(timg1.timer0).into_async(), 1).await;
        #[cfg(timergroup_timg_has_timer1)]
        test_async_delay_ms(OneShotTimer::new(timg1.timer1).into_async(), 1).await;
    }
}
