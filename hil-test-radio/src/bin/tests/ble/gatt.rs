#[embedded_test::tests(default_timeout = 30, executor = esp_rtos::embassy::Executor::new())]
mod tests {
    use embassy_futures::select::select;
    use embassy_time::{Duration, Timer, with_timeout};
    use esp_hal::{
        clock::CpuClock,
        interrupt::software::SoftwareInterruptControl,
        peripherals::Peripherals,
        timer::timg::TimerGroup,
    };
    use esp_radio::ble::controller::BleConnector;
    use trouble_host::prelude::*;

    #[init]
    fn init() -> Peripherals {
        crate::init_heap();

        let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
        esp_hal::init(config)
    }

    #[test]
    async fn ble_central_connects_to_peripheral(p: Peripherals) {
        let timg0 = TimerGroup::new(p.TIMG0);
        let sw_ints = SoftwareInterruptControl::new(p.SW_INTERRUPT);
        esp_rtos::start(timg0.timer0, sw_ints.software_interrupt0);

        let connector = BleConnector::new(p.BT, Default::default()).unwrap();
        let controller: ExternalController<_, 1> = ExternalController::new(connector);

        let address = Address::random(crate::PERIPHERAL_ADDRESS);
        let mut resources: HostResources<_, DefaultPacketPool, 1, 2> = HostResources::new();
        let stack = trouble_host::new(controller, &mut resources)
            .set_random_address(Address::random([0xff, 0x48, 0x49, 0x4c, 0x43, 0xff]))
            .build();
        let mut central = stack.central();
        let runner = stack.runner();

        let _ = select(ble_task(runner), async {
            let accept_list = [address];
            let connect_config = ConnectConfig {
                scan_config: ScanConfig {
                    filter_accept_list: &accept_list,
                    active: true,
                    phys: PhySet::M1,
                    interval: Duration::from_millis(100),
                    window: Duration::from_millis(100),
                    timeout: Duration::from_secs(10),
                    ..Default::default()
                },
                connect_params: RequestedConnParams::default(),
            };

            let conn = with_timeout(Duration::from_secs(15), central.connect(&connect_config))
                .await
                .expect("BLE connect timed out")
                .expect("BLE connect failed");

            assert_eq!(conn.peer_address(), address);
            assert!(conn.is_connected());

            let client = GattClient::<_, DefaultPacketPool, 4>::new(&stack, &conn)
                .await
                .expect("GATT client setup failed");

            let _ = select(client.task(), async {
                let services = client
                    .services_by_uuid(&Uuid::new_short(crate::BATTERY_SERVICE_UUID))
                    .await
                    .expect("Battery service discovery failed");
                let service = services.first().expect("Battery service not found");
                let level = client
                    .characteristic_by_uuid::<u8>(
                        service,
                        &Uuid::new_short(crate::BATTERY_LEVEL_UUID),
                    )
                    .await
                    .expect("Battery Level characteristic not found");

                let mut initial_level = [0];
                let len = client
                    .read_characteristic(&level, &mut initial_level)
                    .await
                    .expect("Battery Level read failed");

                assert_eq!(len, initial_level.len());
                assert!(initial_level[0] <= 100);

                let mut second_level = [0];
                let len = client
                    .read_characteristic(&level, &mut second_level)
                    .await
                    .expect("Second Battery Level read failed");

                assert_eq!(len, second_level.len());
                assert!(second_level[0] <= 100);
                assert_eq!(second_level, initial_level);
            })
            .await;

            Timer::after(Duration::from_millis(250)).await;
        })
        .await;
    }

    async fn ble_task<C: Controller, P: PacketPool>(mut runner: Runner<'_, C, P>) {
        loop {
            if runner.run().await.is_err() {
                Timer::after(Duration::from_millis(100)).await;
            }
        }
    }
}
