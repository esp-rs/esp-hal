//! Async Delay Test
//!
//! Specifically tests the various implementations of the
//! `embedded_hal_async::delay::DelayNs` trait.

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: unstable

#![no_std]
#![no_main]

use embedded_hal_async::delay::DelayNs;
#[cfg(systimer)]
use esp_hal::timer::systimer::SystemTimer;
use esp_hal::{
    peripherals::Peripherals,
    timer::{timg::TimerGroup, OneShotTimer},
};
use hil_test as _;

struct Context {
    peripherals: Peripherals,
}

async fn test_async_delay_ns(mut timer: impl DelayNs, duration: u32) {
    for i in 1..5 {
        let t1 = esp_hal::time::now();
        timer.delay_ns(duration).await;
        let t2 = esp_hal::time::now();

        assert!(t2 > t1);
        assert!(
            (t2 - t1).to_nanos() >= duration as u64,
            "diff[{}]: {:?} >= {}",
            i,
            (t2 - t1).to_nanos(),
            duration
        );
    }
}

async fn test_async_delay_us(mut timer: impl DelayNs, duration: u32) {
    for _ in 1..5 {
        let t1 = esp_hal::time::now();
        timer.delay_us(duration).await;
        let t2 = esp_hal::time::now();

        assert!(t2 > t1);
        assert!(
            (t2 - t1).to_nanos() >= duration as u64,
            "diff: {:?}",
            (t2 - t1).to_nanos()
        );
    }
}

async fn test_async_delay_ms(mut timer: impl DelayNs, duration: u32) {
    for _ in 1..5 {
        let t1 = esp_hal::time::now();
        timer.delay_ms(duration).await;
        let t2 = esp_hal::time::now();

        assert!(t2 > t1);
        assert!(
            (t2 - t1).to_nanos() >= duration as u64,
            "diff: {:?}",
            (t2 - t1).to_nanos()
        );
    }
}

#[cfg(test)]
#[embedded_test::tests(default_timeout = 2, executor = esp_hal_embassy::Executor::new())]
mod tests {
    use super::*;

    #[init]
    fn init() -> Context {
        Context {
            peripherals: esp_hal::init(esp_hal::Config::default()),
        }
    }

    #[cfg(systimer)]
    #[test]
    async fn test_systimer_async_delay_ns(ctx: Context) {
        let alarms = SystemTimer::new(ctx.peripherals.SYSTIMER);

        test_async_delay_ns(OneShotTimer::new(alarms.alarm0).into_async(), 10_000_000).await;
    }

    #[test]
    async fn test_timg0_async_delay_ns(ctx: Context) {
        let timg0 = TimerGroup::new(ctx.peripherals.TIMG0);

        test_async_delay_ns(OneShotTimer::new(timg0.timer0).into_async(), 10_000_000).await;
        #[cfg(timg_timer1)]
        test_async_delay_ns(OneShotTimer::new(timg0.timer1).into_async(), 10_000_000).await;
    }

    #[cfg(timg1)]
    #[test]
    async fn test_timg1_async_delay_ns(ctx: Context) {
        let timg1 = TimerGroup::new(ctx.peripherals.TIMG1);

        test_async_delay_ns(OneShotTimer::new(timg1.timer0).into_async(), 10_000_000).await;
        #[cfg(timg_timer1)]
        test_async_delay_ns(OneShotTimer::new(timg1.timer1).into_async(), 10_000_000).await;
    }

    #[cfg(systimer)]
    #[test]
    async fn test_systimer_async_delay_us(ctx: Context) {
        let alarms = SystemTimer::new(ctx.peripherals.SYSTIMER);

        test_async_delay_us(OneShotTimer::new(alarms.alarm0).into_async(), 10_000).await;
    }

    #[test]
    async fn test_timg0_async_delay_us(ctx: Context) {
        let timg0 = TimerGroup::new(ctx.peripherals.TIMG0);

        test_async_delay_us(OneShotTimer::new(timg0.timer0).into_async(), 10_000).await;
        #[cfg(timg_timer1)]
        test_async_delay_us(OneShotTimer::new(timg0.timer1).into_async(), 10_000).await;
    }

    #[cfg(timg1)]
    #[test]
    async fn test_timg1_async_delay_us(ctx: Context) {
        let timg1 = TimerGroup::new(ctx.peripherals.TIMG1);

        test_async_delay_us(OneShotTimer::new(timg1.timer0).into_async(), 10_000).await;
        #[cfg(timg_timer1)]
        test_async_delay_us(OneShotTimer::new(timg1.timer1).into_async(), 10_000).await;
    }

    #[cfg(systimer)]
    #[test]
    async fn test_systimer_async_delay_ms(ctx: Context) {
        let alarms = SystemTimer::new(ctx.peripherals.SYSTIMER);

        test_async_delay_ms(OneShotTimer::new(alarms.alarm0).into_async(), 10).await;
    }

    #[test]
    async fn test_timg0_async_delay_ms(ctx: Context) {
        let timg0 = TimerGroup::new(ctx.peripherals.TIMG0);

        test_async_delay_ms(OneShotTimer::new(timg0.timer0).into_async(), 10).await;
        #[cfg(timg_timer1)]
        test_async_delay_ms(OneShotTimer::new(timg0.timer1).into_async(), 10).await;
    }

    #[cfg(timg1)]
    #[test]
    async fn test_timg1_async_delay_ms(ctx: Context) {
        let timg1 = TimerGroup::new(ctx.peripherals.TIMG1);

        test_async_delay_ms(OneShotTimer::new(timg1.timer0).into_async(), 10).await;
        #[cfg(timg_timer1)]
        test_async_delay_ms(OneShotTimer::new(timg1.timer1).into_async(), 10).await;
    }
}
