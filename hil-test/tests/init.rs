//! Initialization tests

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use esp_hal::{
    config::{Config, WatchdogStatus},
    delay::Delay,
    prelude::*,
    rtc_cntl::Rtc,
    timer::timg::TimerGroup,
};
use hil_test as _;

#[cfg(test)]
#[embedded_test::tests]
mod tests {
    use super::*;

    #[test]
    #[timeout(3)]
    fn test_feeding_timg0_wdt() {
        let peripherals = esp_hal::init({
            let mut config = Config::default();
            config.watchdog.timg0 =
                WatchdogStatus::Enabled(fugit::MicrosDurationU64::millis(500 as u64));
            config
        });

        let timg0 = TimerGroup::new(peripherals.TIMG0);
        let mut wdt0 = timg0.wdt;
        let delay = Delay::new();

        for _ in 0..4 {
            wdt0.feed();
            delay.delay(250.millis());
        }
    }

    #[test]
    #[timeout(4)]
    fn test_feeding_rtc_wdt() {
        let peripherals = esp_hal::init({
            let mut config = Config::default();
            config.watchdog.rwdt =
                WatchdogStatus::Enabled(fugit::MicrosDurationU64::millis(3000 as u64));
            config
        });

        let mut rtc = Rtc::new(peripherals.LPWR);
        let delay = Delay::new();

        rtc.rwdt.feed();
        delay.delay(2500.millis());
    }
}