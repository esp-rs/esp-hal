#[embedded_test::tests(default_timeout = 3)]
mod tests {
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

    // If this turns out to be too flaky or time-consuming,
    // we should consider converting this into a qa-test.
    #[test]
    #[timeout(15)]
    fn test_scan_doesnt_leak(p: Peripherals) {
        const ITERATIONS: usize = 30;

        let timg0: TimerGroup<'_, _> = TimerGroup::new(p.TIMG0);
        let sw_ints = SoftwareInterruptControl::new(p.SW_INTERRUPT);
        esp_rtos::start(timg0.timer0, sw_ints.software_interrupt0);

        let (mut controller, _interfaces) =
            esp_radio::wifi::new(p.WIFI, Default::default()).unwrap();

        controller
            .set_mode(esp_radio::wifi::WifiMode::Station)
            .unwrap();
        controller.start().unwrap();

        let scan_config = ScanConfig::default().with_max(1);
        for _ in 0..2 {
            let _ = controller.scan_with_config(scan_config).unwrap();
        }

        let mut more_count = 0;
        let mut min_free = usize::MAX;
        for _ in 0..ITERATIONS {
            let _ = controller.scan_with_config(scan_config).unwrap();
            let free = esp_alloc::HEAP.free();
            defmt::info!("free: {}", free);

            if free <= min_free {
                min_free = free;
            } else {
                more_count += 1;
            }
        }

        assert!(more_count < ITERATIONS / 2, "count: {}", more_count);
    }
}
