#[embedded_test::tests(default_timeout = 30, executor = esp_rtos::embassy::Executor::new())]
mod tests {
    use embassy_futures::select::select;
    use embassy_time::{Duration, Timer, with_timeout};
    use esp_hal::{clock::CpuClock, peripherals::Peripherals, timer::timg::TimerGroup};
    use esp_radio::ble::controller::BleConnector;
    use trouble_host::prelude::*;

    #[init]
    fn init() -> Peripherals {
        crate::init_heap();

        let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
        esp_hal::init(config)
    }

    /// The support firmware sends with an LE data length of `DATA_LENGTH`, so the
    /// controller splits the received ACL packets over a chain of mbufs.
    #[test]
    async fn ble_central_receives_l2cap_data(p: Peripherals) {
        let timg0 = TimerGroup::new(p.TIMG0);
        esp_rtos::start(timg0.timer0);

        let connector = BleConnector::new(p.BT, Default::default()).unwrap();
        let controller: ExternalController<_, 1> = ExternalController::new(connector);

        let address = Address::random(crate::PERIPHERAL_ADDRESS);
        let mut resources: HostResources<DefaultPacketPool, 1, 2> = HostResources::new();
        let stack = trouble_host::new(controller, &mut resources)
            .set_random_address(Address::random([0xff, 0x48, 0x49, 0x4c, 0x43, 0xff]))
            .build();
        let mut central = stack.central();
        let runner = stack.runner();

        let _ = select(crate::ble_task(runner), async {
            let conn = crate::connect(&mut central, address).await;
            // The support firmware rejects the channel until it is listening.
            Timer::after(Duration::from_millis(500)).await;
            let mut channel =
                L2capChannel::create(&stack, &conn, hil_test::ble::L2CAP_PSM, &Default::default())
                    .await
                    .expect("L2CAP channel setup failed");

            let mut received = 0;
            let mut sdu = [0; 255];
            while received < hil_test::ble::L2CAP_BYTES {
                let len = with_timeout(Duration::from_secs(5), channel.receive(&stack, &mut sdu))
                    .await
                    .expect("L2CAP receive timed out")
                    .expect("L2CAP receive failed");
                for (i, byte) in sdu[..len].iter().enumerate() {
                    assert_eq!(*byte, hil_test::ble::l2cap_byte(received + i));
                }
                received += len;
            }

            assert_eq!(received, hil_test::ble::L2CAP_BYTES);
        })
        .await;
    }
}
