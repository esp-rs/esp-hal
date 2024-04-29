//! Delay Test

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32s2 esp32s3
// TODO: esp32h2 is disabled due to https://github.com/esp-rs/esp-hal/issues/1509

#![no_std]
#![no_main]

use defmt_rtt as _;
use embedded_hal::delay::DelayNs;
use esp_backtrace as _;
use esp_hal::{clock::ClockControl, delay::Delay, peripherals::Peripherals, system::SystemControl};

struct Context {
    delay: Delay,
}

impl Context {
    pub fn init() -> Self {
        let peripherals = Peripherals::take();
        let system = SystemControl::new(peripherals.SYSTEM);
        let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

        let delay = Delay::new(&clocks);

        Context { delay }
    }
}

#[cfg(test)]
#[embedded_test::tests]
mod tests {
    use super::*;

    #[init]
    fn init() -> Context {
        Context::init()
    }

    #[test]
    #[timeout(1)]
    fn delay_ns(mut ctx: Context) {
        let t1 = esp_hal::time::current_time();
        ctx.delay.delay_ns(600_000_000);
        let t2 = esp_hal::time::current_time();

        assert!(t2 > t1);
        assert!((t2 - t1).to_nanos() >= 600_000_000u64);
    }

    #[test]
    #[timeout(1)]
    fn delay_700millis(ctx: Context) {
        let t1 = esp_hal::time::current_time();
        ctx.delay.delay_millis(700);
        let t2 = esp_hal::time::current_time();

        assert!(t2 > t1);
        assert!((t2 - t1).to_millis() >= 700u64);
    }

    #[test]
    #[timeout(2)]
    fn delay_1_500_000us(mut ctx: Context) {
        let t1 = esp_hal::time::current_time();
        ctx.delay.delay_us(1_500_000);
        let t2 = esp_hal::time::current_time();

        assert!(t2 > t1);
        assert!((t2 - t1).to_micros() >= 1_500_000u64);
    }

    #[test]
    #[timeout(5)]
    fn delay_3_000ms(mut ctx: Context) {
        let t1 = esp_hal::time::current_time();
        ctx.delay.delay_ms(3000);
        let t2 = esp_hal::time::current_time();

        assert!(t2 > t1);
        assert!((t2 - t1).to_millis() >= 3000u64);
    }
}
