#[embedded_test::tests(default_timeout = 3, executor = hil_test::Executor::new())]
mod tests {
    use esp_hal::{
        clock::CpuClock,
        interrupt::software::SoftwareInterruptControl,
        peripherals::Peripherals,
        timer::timg::TimerGroup,
    };
    use esp_radio::ble::controller::BleConnector;
    use trouble_host::prelude::*;

    // GATT Server definition
    #[gatt_server]
    struct Server {}

    fn read_packet(connector: &mut BleConnector, buf: &mut [u8]) -> usize {
        // Read header
        read_all(connector, &mut buf[..3]);

        // Read payload
        let payload_len = buf[2] as usize;
        read_all(connector, &mut buf[3..][..payload_len]);

        3 + payload_len
    }

    fn read_all(connector: &mut BleConnector, mut buf: &mut [u8]) {
        while !buf.is_empty() {
            let len = connector.read(buf).unwrap();
            buf = &mut buf[len..];
        }
    }

    // Compile-time test to check that esp-radio can be reinitialized.
    fn _esp_radio_can_be_reinited() {
        let mut p = esp_hal::init(esp_hal::Config::default());

        let timg0: TimerGroup<'_, _> = TimerGroup::new(p.TIMG0);
        let sw_ints = SoftwareInterruptControl::new(p.SW_INTERRUPT);
        esp_rtos::start(timg0.timer0, sw_ints.software_interrupt0);

        {
            let _connector = BleConnector::new(p.BT.reborrow(), Default::default()).unwrap();
        }

        {
            let _connector = BleConnector::new(p.BT.reborrow(), Default::default()).unwrap();
        }
    }
    const H4_TYPE_COMMAND: u8 = 1;

    pub const CONTROLLER_OGF: u8 = 0x03;
    pub const RESET_OCF: u16 = 0x03;

    #[init]
    fn init() -> Peripherals {
        crate::init_heap();

        let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
        esp_hal::init(config)
    }

    #[test]
    fn test_controller_comms(p: Peripherals) {
        let timg0: TimerGroup<'_, _> = TimerGroup::new(p.TIMG0);
        let sw_ints = SoftwareInterruptControl::new(p.SW_INTERRUPT);
        esp_rtos::start(timg0.timer0, sw_ints.software_interrupt0);

        let mut connector = BleConnector::new(p.BT, Default::default()).unwrap();

        // send reset cmd
        pub const fn opcode(ogf: u8, ocf: u16) -> [u8; 2] {
            let opcode = ((ogf as u16) << 10) + ocf as u16;
            opcode.to_le_bytes()
        }

        let opcode = opcode(CONTROLLER_OGF, RESET_OCF);

        let mut buf = [0u8; 4];
        buf[0] = H4_TYPE_COMMAND;
        buf[1] = opcode[0];
        buf[2] = opcode[1];
        buf[3] = 0; // payload length - RESET COMMAND doesn't have any parameters

        connector.write(&buf).unwrap();

        // wait for an expected response - we might get some other events, too, since
        // different chips have different defaults for which events are masked away
        // so if we don't get the expected response we'll timeout and fail
        let mut buf = [0u8; 255];
        loop {
            let len = read_packet(&mut connector, &mut buf);
            if len == 7 {
                assert_eq!(buf[0], 4, "Expected packet type = 4 (EVENT)");
                assert_eq!(buf[1], 14, "Expected event code = 14 (COMMAND_COMPLETE)");
                assert_eq!(buf[2], 4, "Expected payload length = 4");

                // don't care about NUM_HANDLES (different for different chips)

                assert_eq!(buf[4], opcode[0], "Unexpected op-code first byte");
                assert_eq!(buf[5], opcode[1], "Unexpected op-code second byte");
                assert_eq!(
                    buf[6], 0,
                    "Expected RESET COMMAND to succeed (return code = 0)"
                );
                break;
            }
        }
    }

    // This test case is a regression test for #3760 and asserts that dropping the controller during
    // reset does not generate an exception.
    #[test]
    fn test_dropping_controller_during_reset(p: Peripherals) {
        let timg0: TimerGroup<'_, _> = TimerGroup::new(p.TIMG0);
        let sw_ints = SoftwareInterruptControl::new(p.SW_INTERRUPT);
        esp_rtos::start(timg0.timer0, sw_ints.software_interrupt0);

        let mut connector = BleConnector::new(p.BT, Default::default()).unwrap();

        // send reset cmd
        pub const fn opcode(ogf: u8, ocf: u16) -> [u8; 2] {
            let opcode = ((ogf as u16) << 10) + ocf as u16;
            opcode.to_le_bytes()
        }

        let opcode = opcode(CONTROLLER_OGF, RESET_OCF);

        let mut buf = [0u8; 4];
        buf[0] = H4_TYPE_COMMAND;
        buf[1] = opcode[0];
        buf[2] = opcode[1];
        buf[3] = 0; // payload length - RESET COMMAND doesn't have any parameters

        connector.write(&buf).unwrap();

        core::mem::drop(connector);

        esp_hal::delay::Delay::new().delay_millis(10);
    }

    #[test]
    async fn test_trouble_starts_advertising(p: Peripherals) {
        let timg0: TimerGroup<'_, _> = TimerGroup::new(p.TIMG0);
        let sw_ints = SoftwareInterruptControl::new(p.SW_INTERRUPT);
        esp_rtos::start(timg0.timer0, sw_ints.software_interrupt0);

        let connector = BleConnector::new(p.BT, Default::default()).unwrap();
        let controller: ExternalController<_, 1> = ExternalController::new(connector);

        let address: Address = Address::random([0xff, 0x8f, 0x1a, 0x05, 0xe4, 0xff]);

        let mut resources: HostResources<DefaultPacketPool, 1, 1> = HostResources::new();
        let stack = trouble_host::new(controller, &mut resources).set_random_address(address);
        let Host { mut peripheral, .. } = stack.build();

        let mut advertiser_data = [0; 31];
        let len = AdStructure::encode_slice(
            &[
                AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
                AdStructure::ServiceUuids16(&[[0x0f, 0x18]]),
                AdStructure::CompleteLocalName("Peripheral".as_bytes()),
            ],
            &mut advertiser_data[..],
        )
        .unwrap();

        let params = Default::default();
        let res = embassy_time::with_timeout(
            embassy_time::Duration::from_millis(1000),
            peripheral.advertise(
                &params,
                Advertisement::ConnectableScannableUndirected {
                    adv_data: &advertiser_data[..len],
                    scan_data: &[],
                },
            ),
        )
        .await;

        match res {
            Ok(res) => match res {
                Ok(_) => {
                    panic!("Advertise completed")
                }
                Err(err) => {
                    panic!("Advertise completed with error {:?}", err);
                }
            },
            Err(_err) => {
                // ok - we expect the timeout
            }
        }
    }
}
