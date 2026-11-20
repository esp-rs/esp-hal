#[embedded_test::tests(default_timeout = 10, executor = hil_test::Executor::new())]
mod tests {
    use esp_hal::{clock::CpuClock, peripherals::Peripherals, timer::timg::TimerGroup};
    use esp_radio::wifi::{Config, WifiController, ap::AccessPointConfig, sta::StationConfig};

    // esp-radio has no getter for promiscuous mode.
    unsafe extern "C" {
        fn esp_wifi_get_promiscuous(en: *mut bool) -> i32;
    }

    fn promiscuous_mode() -> bool {
        let mut enabled = false;
        assert_eq!(unsafe { esp_wifi_get_promiscuous(&mut enabled) }, 0);
        enabled
    }

    #[init]
    fn init() -> Peripherals {
        crate::init_heap();

        let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
        esp_hal::init(config)
    }

    #[test]
    fn promiscuous_mode_survives_mode_change(p: Peripherals) {
        let timg0 = TimerGroup::new(p.TIMG0);
        esp_rtos::start(timg0.timer0);

        let mut controller = WifiController::new(p.WIFI, Default::default()).unwrap();
        let sniffer = controller.sniffer();
        sniffer.set_promiscuous_mode(true).unwrap();

        controller
            .set_config(&Config::AccessPointStation(
                StationConfig::default(),
                AccessPointConfig::default(),
            ))
            .unwrap();

        assert!(promiscuous_mode());
    }
}
