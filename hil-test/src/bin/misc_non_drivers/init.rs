#[embedded_test::tests(default_timeout = 3)]
mod tests {
    use esp_hal::{
        Config,
        clock::CpuClock,
        config::{WatchdogConfig, WatchdogStatus},
        delay::Delay,
        rtc_cntl::Rtc,
        time::Duration,
        timer::timg::TimerGroup,
    };

    #[test]
    #[cfg(timergroup_timg0)]
    fn test_feeding_timg0_wdt() {
        let peripherals = esp_hal::init(
            Config::default().with_watchdog(
                WatchdogConfig::default()
                    .with_timg0(WatchdogStatus::Enabled(Duration::from_millis(500))),
            ),
        );

        let timg0 = TimerGroup::new(peripherals.TIMG0);
        let mut wdt0 = timg0.wdt;
        let delay = Delay::new();

        // Loop for more than the timeout of the watchdog.
        for _ in 0..6 {
            wdt0.feed();
            delay.delay(Duration::from_millis(100));
        }
        // Disable the watchdog, to prevent accidentally resetting the MCU while the host is setting
        // up the next test.
        wdt0.disable();
    }

    #[test]
    #[cfg(timergroup_timg0)]
    fn test_wdt0_uses_prescaler() {
        let p = esp_hal::init(
            Config::default().with_watchdog(
                WatchdogConfig::default()
                    .with_timg0(WatchdogStatus::Enabled(Duration::from_micros(53_687_092))), // multiplied by 80 (for the default clock source), then taking the 32 lower bits this is 0x40
            ),
        );

        let delay = Delay::new();
        delay.delay(Duration::from_millis(250));

        // Disable the watchdog, to prevent accidentally resetting the MCU while the host is setting
        // up the next test.
        let timg0 = TimerGroup::new(p.TIMG0);
        let mut wdt0 = timg0.wdt;
        wdt0.disable();
    }

    #[test]
    #[cfg(timergroup_timg1)]
    fn test_feeding_timg1_wdt() {
        let peripherals = esp_hal::init(
            Config::default().with_watchdog(
                WatchdogConfig::default()
                    .with_timg1(WatchdogStatus::Enabled(Duration::from_millis(500))),
            ),
        );

        let timg1 = TimerGroup::new(peripherals.TIMG1);
        let mut wdt1 = timg1.wdt;
        let delay = Delay::new();

        // Loop for more than the timeout of the watchdog.
        for _ in 0..6 {
            wdt1.feed();
            delay.delay(Duration::from_millis(100));
        }
        // Disable the watchdog, to prevent accidentally resetting the MCU while the host is setting
        // up the next test.
        wdt1.disable();
    }

    #[test]
    #[cfg(timergroup_timg0)]
    fn test_feeding_timg0_wdt_max_clock() {
        let peripherals = esp_hal::init(
            Config::default()
                .with_cpu_clock(CpuClock::max())
                .with_watchdog(
                    WatchdogConfig::default()
                        .with_timg0(WatchdogStatus::Enabled(Duration::from_millis(500))),
                ),
        );

        let timg0 = TimerGroup::new(peripherals.TIMG0);
        let mut wdt0 = timg0.wdt;
        let delay = Delay::new();

        // Loop for more than the timeout of the watchdog.
        for _ in 0..6 {
            wdt0.feed();
            delay.delay(Duration::from_millis(100));
        }
        // Disable the watchdog, to prevent accidentally resetting the MCU while the host is setting
        // up the next test.
        wdt0.disable();
    }

    #[test]
    fn test_feeding_rtc_wdt() {
        let peripherals = esp_hal::init(
            Config::default().with_watchdog(
                WatchdogConfig::default()
                    .with_rwdt(WatchdogStatus::Enabled(Duration::from_millis(500))),
            ),
        );

        let mut rtc = Rtc::new(peripherals.LPWR);
        let delay = Delay::new();

        // Loop for more than the timeout of the watchdog.
        for _ in 0..6 {
            rtc.rwdt.feed();
            delay.delay(Duration::from_millis(100));
        }
        // Disable the watchdog, to prevent accidentally resetting the MCU while the host is setting
        // up the next test.
        rtc.rwdt.disable();
    }

    #[test]
    fn test_default_config() {
        esp_hal::init(Config::default());

        let delay = Delay::new();
        delay.delay(Duration::from_millis(1000));
    }
}
