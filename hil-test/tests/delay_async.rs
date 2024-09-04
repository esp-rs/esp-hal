//! Async Delay Test
//!
//! Specifically tests the various implementations of the
//! `embedded_hal_async::delay::DelayNs` trait.

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32s2 esp32s3

#![no_std]
#![no_main]

use embedded_hal_async::delay::DelayNs as _;
use esp_hal::{
    peripherals::Peripherals,
    timer::{
        systimer::{Alarm, FrozenUnit, SystemTimer},
        timg::TimerGroup,
    },
};
use hil_test as _;

struct Context {
    peripherals: Peripherals,
}

#[cfg(test)]
#[embedded_test::tests(executor = esp_hal_embassy::Executor::new())]
mod tests {
    use super::*;

    #[init]
    fn init() -> Context {
        Context {
            peripherals: esp_hal::init(esp_hal::Config::default()),
        }
    }

    #[test]
    #[timeout(2)]
    async fn test_systimer_async_delay_ns(ctx: Context) {
        let mut alarms = SystemTimer::new(ctx.peripherals.SYSTIMER);
        let unit = FrozenUnit::new(&mut alarms.unit0);
        let mut alarm0 = Alarm::new_async(alarms.comparator0, &unit).into_periodic();

        let t1 = esp_hal::time::current_time();
        alarm0.delay_ns(600_000_000).await;
        let t2 = esp_hal::time::current_time();

        assert!(t2 > t1);
        assert!(
            (t2 - t1).to_nanos() >= 600_000_000u64,
            "diff: {:?}",
            (t2 - t1).to_nanos()
        );
    }

    #[test]
    #[timeout(2)]
    async fn test_timg0_async_delay_ns(ctx: Context) {
        let timg0 = TimerGroup::new_async(ctx.peripherals.TIMG0);
        let mut timer0 = timg0.timer0;

        let t1 = esp_hal::time::current_time();
        timer0.delay_ns(600_000_000).await;
        let t2 = esp_hal::time::current_time();

        assert!(t2 > t1);
        assert!(
            (t2 - t1).to_nanos() >= 600_000_000u64,
            "diff: {:?}",
            (t2 - t1).to_nanos()
        );
    }
}
