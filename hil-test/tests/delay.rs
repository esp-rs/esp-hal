//! Delay Test

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32s2 esp32s3

#![no_std]
#![no_main]

use embedded_hal::delay::DelayNs;
use esp_hal::delay::Delay;
use hil_test as _;

struct Context {
    delay: Delay,
}

#[cfg(test)]
#[embedded_test::tests]
mod tests {
    use super::*;

    #[init]
    fn init() -> Context {
        let _peripherals = esp_hal::init(esp_hal::Config::default());
        let delay = Delay::new();

        Context { delay }
    }

    #[test]
    #[timeout(2)]
    fn delay_ns(mut ctx: Context) {
        let t1 = esp_hal::time::now();
        ctx.delay.delay_ns(600_000_000);
        let t2 = esp_hal::time::now();

        assert!(t2 > t1);
        assert!(
            (t2 - t1).to_nanos() >= 600_000_000u64,
            "diff: {:?}",
            (t2 - t1).to_nanos()
        );
    }

    #[test]
    #[timeout(2)]
    fn delay_700millis(ctx: Context) {
        let t1 = esp_hal::time::now();
        ctx.delay.delay_millis(700);
        let t2 = esp_hal::time::now();

        assert!(t2 > t1);
        assert!(
            (t2 - t1).to_millis() >= 700u64,
            "diff: {:?}",
            (t2 - t1).to_millis()
        );
    }

    #[test]
    #[timeout(2)]
    fn delay_1_500_000us(mut ctx: Context) {
        let t1 = esp_hal::time::now();
        ctx.delay.delay_us(1_500_000);
        let t2 = esp_hal::time::now();

        assert!(t2 > t1);
        assert!(
            (t2 - t1).to_micros() >= 1_500_000u64,
            "diff: {:?}",
            (t2 - t1).to_micros()
        );
    }

    #[test]
    #[timeout(5)]
    fn delay_3_000ms(mut ctx: Context) {
        let t1 = esp_hal::time::now();
        ctx.delay.delay_ms(3000);
        let t2 = esp_hal::time::now();

        assert!(t2 > t1);
        assert!(
            (t2 - t1).to_millis() >= 3000u64,
            "diff: {:?}",
            (t2 - t1).to_millis()
        );
    }
}
