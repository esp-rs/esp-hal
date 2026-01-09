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
        for _ in 0..5 {
            let _ = controller.scan_with_config(scan_config).unwrap();
        }

        let mut min_free = usize::MAX;
        for _ in 0..25 {
            let _ = controller.scan_with_config(scan_config).unwrap();
            min_free = usize::min(min_free, esp_alloc::HEAP.free());
        }

        for _ in 0..10 {
            let _ = controller.scan_with_config(scan_config).unwrap();
            let free = esp_alloc::HEAP.free();
            assert!(
                free >= min_free,
                "current free: {}, min free: {}",
                free,
                min_free
            );
        }
    }
}
