#[embedded_test::tests(default_timeout = 3, executor = hil_test::Executor::new())]
mod tests {
    use embassy_time::{Duration, Timer};
    use esp_hal::{
        clock::CpuClock,
        interrupt::software::SoftwareInterruptControl,
        peripherals::Peripherals,
        timer::timg::TimerGroup,
    };
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
        let sw_ints = SoftwareInterruptControl::new(p.SW_INTERRUPT);
        esp_rtos::start(timg0.timer0, sw_ints.software_interrupt0);

        let _source = esp_hal::rng::TrngSource::new(p.RNG, p.ADC1);

        let _controller = esp_radio::wifi::WifiController::new(p.WIFI, Default::default()).unwrap();
    }

    // If this turns out to be too flaky or time-consuming,
    // we should consider converting this into a qa-test.
    #[test]
    #[timeout(15)]
    async fn test_scan_doesnt_leak(p: Peripherals) {
        let timg0: TimerGroup<'_, _> = TimerGroup::new(p.TIMG0);
        let sw_ints = SoftwareInterruptControl::new(p.SW_INTERRUPT);
        esp_rtos::start(timg0.timer0, sw_ints.software_interrupt0);

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
