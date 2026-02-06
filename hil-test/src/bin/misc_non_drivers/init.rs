#[embedded_test::tests(default_timeout = 3)]
mod tests {
    use esp_hal::{
        Config,
        delay::Delay,
        rtc_cntl::{Rtc, RwdtStage},
        time::Duration,
    };
    #[cfg(timergroup)]
    use esp_hal::{
        clock::CpuClock,
        timer::timg::{MwdtStage, TimerGroup},
    };

    #[test]
    #[cfg(timergroup_timg0)]
    fn test_feeding_timg0_wdt() {
        let p = esp_hal::init(Config::default());

        let timg0 = TimerGroup::new(p.TIMG0);
        let mut wdt0 = timg0.wdt;

        wdt0.set_timeout(MwdtStage::Stage0, Duration::from_millis(500));
        wdt0.enable();

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
        let p = esp_hal::init(Config::default());

        let timg0 = TimerGroup::new(p.TIMG0);
        let mut wdt0 = timg0.wdt;

        // multiplied by 80 (for the default clock source), then taking the 32 lower bits this is
        // 0x40
        wdt0.set_timeout(MwdtStage::Stage0, Duration::from_millis(53_687_092));
        wdt0.enable();

        let delay = Delay::new();
        delay.delay(Duration::from_millis(250));

        // Disable the watchdog, to prevent accidentally resetting the MCU while the host is setting
        // up the next test.
        wdt0.disable();
    }

    #[test]
    #[cfg(timergroup_timg1)]
    fn test_feeding_timg1_wdt() {
        let p = esp_hal::init(Config::default());

        let timg1 = TimerGroup::new(p.TIMG1);
        let mut wdt1 = timg1.wdt;

        wdt1.set_timeout(MwdtStage::Stage0, Duration::from_millis(500));
        wdt1.enable();

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
        let p = esp_hal::init(Config::default().with_cpu_clock(CpuClock::max()));

        let timg0 = TimerGroup::new(p.TIMG0);
        let mut wdt0 = timg0.wdt;

        wdt0.set_timeout(MwdtStage::Stage0, Duration::from_millis(500));
        wdt0.enable();

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
        let p = esp_hal::init(Config::default());

        let mut rtc = Rtc::new(p.LPWR);

        rtc.rwdt
            .set_timeout(RwdtStage::Stage0, Duration::from_millis(500));
        rtc.rwdt.enable();

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
    fn test_init_disables_watchdogs() {
        esp_hal::init(Config::default());

        let delay = Delay::new();
        delay.delay(Duration::from_millis(1000));
    }
}
