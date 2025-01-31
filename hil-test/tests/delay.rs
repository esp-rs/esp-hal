//! Delay Test

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: unstable

#![no_std]
#![no_main]

use embedded_hal::delay::DelayNs;
use esp_hal::delay::Delay;
use hil_test as _;

struct Context {
    delay: Delay,
}

#[cfg(test)]
#[embedded_test::tests(default_timeout = 2)]
mod tests {
    use super::*;

    #[init]
    fn init() -> Context {
        let _peripherals = esp_hal::init(esp_hal::Config::default());
        let delay = Delay::new();

        Context { delay }
    }

    #[test]
    fn delay_ns(mut ctx: Context) {
        let t1 = esp_hal::time::now();
        ctx.delay.delay_ns(600_000);
        let t2 = esp_hal::time::now();

        assert!(t2 > t1);
        assert!(
            (t2 - t1).as_micros() >= 600u64,
            "diff: {:?}",
            (t2 - t1).as_micros()
        );
    }

    #[test]
    fn delay_70millis(ctx: Context) {
        let t1 = esp_hal::time::now();
        ctx.delay.delay_millis(70);
        let t2 = esp_hal::time::now();

        assert!(t2 > t1);
        assert!(
            (t2 - t1).as_millis() >= 70u64,
            "diff: {:?}",
            (t2 - t1).as_millis()
        );
    }

    #[test]
    fn delay_1_500us(mut ctx: Context) {
        let t1 = esp_hal::time::now();
        ctx.delay.delay_us(1_500);
        let t2 = esp_hal::time::now();

        assert!(t2 > t1);
        assert!(
            (t2 - t1).as_micros() >= 1_500u64,
            "diff: {:?}",
            (t2 - t1).as_micros()
        );
    }

    #[test]
    fn delay_3_00ms(mut ctx: Context) {
        let t1 = esp_hal::time::now();
        ctx.delay.delay_ms(300);
        let t2 = esp_hal::time::now();

        assert!(t2 > t1);
        assert!(
            (t2 - t1).as_millis() >= 300u64,
            "diff: {:?}",
            (t2 - t1).as_millis()
        );
    }
}
