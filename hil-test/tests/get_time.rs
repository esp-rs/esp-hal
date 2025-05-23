//! time::Instant::now Test

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: unstable

#![no_std]
#![no_main]

use esp_hal::delay::Delay;
use hil_test as _;

esp_bootloader_esp_idf::esp_app_desc!();

struct Context {
    delay: Delay,
}

fn time_moves_forward_during<F: FnOnce(Context)>(ctx: Context, f: F) {
    let t1 = esp_hal::time::Instant::now();
    f(ctx);
    let t2 = esp_hal::time::Instant::now();

    assert!(t2 > t1);
}

#[cfg(test)]
#[embedded_test::tests(default_timeout = 3)]
mod tests {
    use super::*;

    #[init]
    fn init() -> Context {
        let _ = esp_hal::init(esp_hal::Config::default());

        let delay = Delay::new();

        Context { delay }
    }

    #[test]
    fn test_current_time(ctx: Context) {
        let t1 = esp_hal::time::Instant::now();
        ctx.delay.delay_millis(500);
        let t2 = esp_hal::time::Instant::now();

        assert!(t2 > t1);
        assert!((t2 - t1).as_millis() >= 500u64);
    }

    #[cfg(systimer)]
    #[test]
    fn test_current_time_construct_systimer(ctx: Context) {
        time_moves_forward_during(ctx, |_| {
            // construct the timer in between calls to current_time
            let _ = esp_hal::timer::systimer::SystemTimer::new(unsafe {
                esp_hal::peripherals::SYSTIMER::steal()
            });
        })
    }

    #[cfg(esp32)]
    #[test]
    fn test_current_time_construct_timg0(ctx: Context) {
        time_moves_forward_during(ctx, |_| {
            // construct the timer in between calls to current_time
            let _ = esp_hal::timer::timg::TimerGroup::new(unsafe {
                esp_hal::peripherals::TIMG0::steal()
            });
        })
    }
}
