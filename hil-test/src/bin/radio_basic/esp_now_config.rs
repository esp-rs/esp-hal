#[embedded_test::tests(default_timeout = 10, executor = hil_test::Executor::new())]
mod tests {
    use esp_hal::{clock::CpuClock, peripherals::Peripherals, timer::timg::TimerGroup};
    use esp_radio::{
        esp_now::{Error, EspNowError, EspNowWifiInterface, PeerInfo},
        wifi::{ControllerConfig, WifiController},
    };

    const PMK: [u8; 16] = [0x42; 16];
    const LMK: [u8; 16] = [0x11; 16];

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
    fn sta_disconnected_pm_can_be_configured(p: Peripherals) {
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
