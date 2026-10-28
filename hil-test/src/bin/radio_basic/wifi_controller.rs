#[embedded_test::tests(default_timeout = 3, executor = hil_test::Executor::new())]
mod tests {
    use embassy_time::{Duration, Timer};
    use esp_hal::{clock::CpuClock, peripherals::Peripherals, timer::timg::TimerGroup};
    use esp_radio::wifi::scan::ScanConfig;

    #[init]
    fn init() -> Peripherals {
        crate::init_heap();

        let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
        esp_hal::init(config)
    }

    #[cfg(rng_trng_supported)]
    #[cfg(feature = "esp-radio-unstable")]
    #[test]
    async fn wifi_starts_with_trng_enabled(p: Peripherals) {
        let timg0: TimerGroup<'_, _> = TimerGroup::new(p.TIMG0);
        esp_rtos::start(timg0.timer0);

        let _source = esp_hal::rng::TrngSource::new(p.RNG, p.ADC1);

        let _controller = esp_radio::wifi::WifiController::new(p.WIFI, Default::default()).unwrap();
    }

    #[cfg(feature = "esp-radio-unstable")]
    #[test]
    fn null_mode_transitions(p: Peripherals) {
        use esp_radio::wifi::{
            Config,
            ControllerConfig,
            SecondaryChannel,
            WifiController,
            WifiError,
        };

        fn max_tx_power() -> i8 {
            unsafe extern "C" {
                fn esp_wifi_get_max_tx_power(power: *mut i8) -> i32;
            }
            let mut power = 0;
            assert_eq!(unsafe { esp_wifi_get_max_tx_power(&mut power) }, 0);
            power
        }

        let timg0 = TimerGroup::new(p.TIMG0);
        esp_rtos::start(timg0.timer0);
        let mut controller = WifiController::new(
            p.WIFI,
            ControllerConfig::default().with_initial_config(Config::None),
        )
        .unwrap();
        let station = Config::Station(Default::default());
        assert_eq!(controller.rssi(), Err(WifiError::Unsupported));
        let sniffer = controller.sniffer();
        sniffer.set_promiscuous_mode(true).unwrap();
        controller.set_channel(1, SecondaryChannel::None).unwrap();
        drop(sniffer);
        controller.set_config(&station).unwrap();
        assert_eq!(max_tx_power(), 20);
        controller.set_max_tx_power(8).unwrap();
        let power = max_tx_power();
        assert!(power < 20);
        controller.set_config(&Config::None).unwrap();
        assert_eq!(controller.rssi(), Err(WifiError::Unsupported));
        controller.set_config(&station).unwrap();
        assert_eq!(max_tx_power(), power);
    }

    // If this turns out to be too flaky or time-consuming,
    // we should consider converting this into a qa-test.
    #[test]
    #[timeout(15)]
    async fn test_scan_doesnt_leak(p: Peripherals) {
        let timg0: TimerGroup<'_, _> = TimerGroup::new(p.TIMG0);
        esp_rtos::start(timg0.timer0);

        let mut controller =
            esp_radio::wifi::WifiController::new(p.WIFI, Default::default()).unwrap();

        // scanning all channels takes a (too) long time - even more for dual-band capable targets
        let scan_config = ScanConfig::default().with_max(1).with_channel(13);
        let _ = controller.scan_async(&scan_config).await.unwrap();

        let mut min_free = usize::MAX;
        for _ in 0..30 {
            let _ = controller.scan_async(&scan_config).await.unwrap();
            min_free = usize::min(min_free, esp_alloc::HEAP.free());
        }

        for _ in 0..10 {
            let _ = controller.scan_async(&scan_config).await.unwrap();

            // The Wi-Fi driver runs in its own task. Memory that the driver holds while we read
            // the free heap makes the heap look smaller than it is. The driver free's this memory
            // soon, but it does not free leaked memory, so wait for the heap to recover.
            let mut free = esp_alloc::HEAP.free();
            for _ in 0..20 {
                if free >= min_free {
                    break;
                }
                Timer::after(Duration::from_millis(10)).await;
                free = esp_alloc::HEAP.free();
            }

            assert!(
                free >= min_free,
                "current free: {}, min free: {}",
                free,
                min_free
            );
        }
    }
}
