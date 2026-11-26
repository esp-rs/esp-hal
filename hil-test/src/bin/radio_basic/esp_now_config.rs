#[embedded_test::tests(default_timeout = 10, executor = hil_test::Executor::new())]
mod tests {
    use esp_hal::{clock::CpuClock, peripherals::Peripherals, timer::timg::TimerGroup};
    use esp_radio::{
        esp_now::{BROADCAST_ADDRESS, Error, EspNowError, EspNowWifiInterface, PeerInfo},
        wifi::{
            Config,
            ControllerConfig,
            WifiController,
            WifiError,
            ap::AccessPointConfig,
            sta::StationConfig,
        },
    };

    const PMK: [u8; 16] = [0x42; 16];
    const LMK: [u8; 16] = [0x11; 16];

    const WIFI_MODE_NULL: u32 = 0;
    const WIFI_MODE_STA: u32 = 1;

    unsafe extern "C" {
        fn esp_wifi_get_mode(mode: *mut u32) -> i32;
    }

    fn wifi_mode() -> u32 {
        let mut mode = 0;
        assert_eq!(unsafe { esp_wifi_get_mode(&mut mode) }, 0);
        mode
    }

    fn peer(index: u8, encrypt: bool) -> PeerInfo {
        PeerInfo {
            interface: EspNowWifiInterface::Station,
            peer_address: [0x02, 0x00, 0x00, 0x00, 0x00, index],
            lmk: encrypt.then_some(LMK),
            channel: None,
            encrypt,
        }
    }

    #[init]
    fn init() -> Peripherals {
        crate::init_heap();

        let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
        esp_hal::init(config)
    }

    #[test]
    fn espnow_max_encrypt_num_is_honored(p: Peripherals) {
        hil_test::init_logger!(p);
        let timg0 = TimerGroup::new(p.TIMG0);
        esp_rtos::start(timg0.timer0);

        let controller = WifiController::new(
            p.WIFI,
            ControllerConfig::default().with_espnow_max_encrypt_num(2),
        )
        .unwrap();

        let esp_now = controller.esp_now();
        esp_now.set_pmk(&PMK).unwrap();

        esp_now.add_peer(peer(1, true)).unwrap();
        esp_now.add_peer(peer(2, true)).unwrap();

        assert_eq!(
            esp_now.add_peer(peer(3, true)),
            Err(EspNowError::Error(Error::PeerListFull))
        );

        esp_now.add_peer(peer(3, false)).unwrap();

        let count = esp_now.peer_count().unwrap();
        assert_eq!(count.encrypted_count, 2);
    }

    #[test]
    fn espnow_max_encrypt_num_raises_the_limit(p: Peripherals) {
        hil_test::init_logger!(p);
        let timg0 = TimerGroup::new(p.TIMG0);
        esp_rtos::start(timg0.timer0);

        let controller = WifiController::new(
            p.WIFI,
            ControllerConfig::default().with_espnow_max_encrypt_num(4),
        )
        .unwrap();

        let esp_now = controller.esp_now();
        esp_now.set_pmk(&PMK).unwrap();

        for index in 1..=4 {
            esp_now.add_peer(peer(index, true)).unwrap();
        }

        assert_eq!(esp_now.peer_count().unwrap().encrypted_count, 4);
    }

    #[test]
    fn esp_now_survives_controller_drop(p: Peripherals) {
        hil_test::init_logger!(p);
        let timg0 = TimerGroup::new(p.TIMG0);
        esp_rtos::start(timg0.timer0);

        let mut wifi = p.WIFI;
        let controller = WifiController::new(wifi.reborrow(), Default::default()).unwrap();
        let mut esp_now = controller.esp_now();
        drop(controller);

        assert_eq!(
            WifiController::new(wifi.reborrow(), Default::default()).err(),
            Some(WifiError::InUse)
        );

        esp_now.set_channel(1).unwrap();
        esp_now
            .send(&BROADCAST_ADDRESS, b"hello")
            .unwrap()
            .wait()
            .unwrap();

        drop(esp_now);

        let _controller = WifiController::new(wifi, Default::default()).unwrap();
    }

    #[test]
    fn failed_set_config_keeps_esp_now_running(p: Peripherals) {
        let timg0 = TimerGroup::new(p.TIMG0);
        esp_rtos::start(timg0.timer0);

        let mut controller = WifiController::new(p.WIFI, Default::default()).unwrap();
        let mut esp_now = controller.esp_now();

        // The default world-safe country doesn't allow channel 13, so applying the access point
        // configuration fails after the mode change has stopped Wi-Fi.
        assert_eq!(
            controller.set_config(&Config::AccessPointStation(
                StationConfig::default(),
                AccessPointConfig::default().with_channel(13),
            )),
            Err(WifiError::InvalidArguments)
        );
        assert_eq!(wifi_mode(), WIFI_MODE_STA);

        esp_now
            .send(&BROADCAST_ADDRESS, b"hello")
            .unwrap()
            .wait()
            .unwrap();
    }

    #[test]
    fn failed_set_config_stops_unshared_wifi(p: Peripherals) {
        let timg0 = TimerGroup::new(p.TIMG0);
        esp_rtos::start(timg0.timer0);

        let mut controller = WifiController::new(p.WIFI, Default::default()).unwrap();

        assert_eq!(
            controller.set_config(&Config::AccessPointStation(
                StationConfig::default(),
                AccessPointConfig::default().with_channel(13),
            )),
            Err(WifiError::InvalidArguments)
        );
        assert_eq!(wifi_mode(), WIFI_MODE_NULL);
    }

    #[test]
    fn sta_disconnected_pm_can_be_configured(p: Peripherals) {
        hil_test::init_logger!(p);
        let timg0 = TimerGroup::new(p.TIMG0);
        esp_rtos::start(timg0.timer0);

        let mut wifi = p.WIFI;
        for enabled in [true, false] {
            let controller = WifiController::new(
                wifi.reborrow(),
                ControllerConfig::default().with_sta_disconnected_pm(enabled),
            )
            .unwrap();

            drop(controller);
        }
    }
}
