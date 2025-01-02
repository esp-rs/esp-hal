//! Initialization tests

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use esp_hal::{
    clock::CpuClock,
    config::WatchdogStatus,
    delay::Delay,
    rtc_cntl::Rtc,
    time::ExtU64,
    timer::timg::TimerGroup,
    Config,
};
use hil_test as _;

#[cfg(test)]
#[embedded_test::tests(default_timeout = 3)]
mod tests {
    use super::*;

    #[test]
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
    #[cfg(timg1)]
    fn test_feeding_timg1_wdt() {
        let peripherals = esp_hal::init({
            let mut config = Config::default();
            config.watchdog.timg1 =
                WatchdogStatus::Enabled(fugit::MicrosDurationU64::millis(500 as u64));
            config
        });

        let timg1 = TimerGroup::new(peripherals.TIMG1);
        let mut wdt1 = timg1.wdt;
        let delay = Delay::new();

        for _ in 0..4 {
            wdt1.feed();
            delay.delay(250.millis());
        }
    }

    #[test]
    fn test_feeding_timg0_wdt_max_clock() {
        let peripherals = esp_hal::init({
            let mut config = Config::default();
            config.cpu_clock = CpuClock::max();
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

    #[test]
    fn test_default_config() {
        esp_hal::init(Config::default());

        let delay = Delay::new();
        delay.delay(2000.millis());
    }
}
