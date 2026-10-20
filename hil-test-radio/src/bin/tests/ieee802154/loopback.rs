#[embedded_test::tests(default_timeout = 30, executor = esp_rtos::embassy::Executor::new())]
mod tests {
    use embassy_time::{Duration, Timer};
    use esp_hal::{
        clock::CpuClock,
        peripherals::{IEEE802154, Peripherals, TIMG0},
        timer::timg::TimerGroup,
    };
    use esp_radio::ieee802154::{Config, Frame, Ieee802154};
    use hil_test::ieee802154::{CHANNEL, DUT_ADDRESS, PAN_ID, PAYLOAD, SUPPORT_ADDRESS};
    use ieee802154::mac::{
        Address,
        FrameContent,
        FrameType,
        FrameVersion,
        Header,
        PanId,
        ShortAddress,
    };

    #[init]
    fn init() -> Peripherals {
        crate::init_heap();

        let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
        esp_hal::init(config)
    }

    fn dut_config() -> Config {
        Config {
            channel: CHANNEL,
            pan_id: Some(PAN_ID),
            short_addr: Some(DUT_ADDRESS),
            rx_when_idle: true,
            auto_ack_rx: true,
            auto_ack_tx: true,
            ..Default::default()
        }
    }

    fn data_frame(seq: u8, ack_request: bool) -> Frame {
        Frame {
            header: Header {
                frame_type: FrameType::Data,
                frame_pending: false,
                ack_request,
                pan_id_compress: false,
                seq_no_suppress: false,
                ie_present: false,
                version: FrameVersion::Ieee802154_2003,
                seq,
                destination: Some(Address::Short(PanId(PAN_ID), ShortAddress(SUPPORT_ADDRESS))),
                source: None,
                auxiliary_security_header: None,
            },
            content: FrameContent::Data,
            payload: PAYLOAD.to_vec(),
            footer: [0u8; 2],
        }
    }

    fn start_radio(timg0: TIMG0<'static>, radio: IEEE802154<'static>) -> Ieee802154<'static> {
        let timg0 = TimerGroup::new(timg0);
        esp_rtos::start(timg0.timer0);

        let mut ieee802154 = Ieee802154::new(radio);
        ieee802154.set_config(dut_config());
        ieee802154.start_receive();
        ieee802154
    }

    /// The peer board auto-acknowledges an ack-requesting frame, so the DUT
    /// should be able to observe the received ACK frame.
    #[test]
    async fn transmit_is_acknowledged(p: Peripherals) {
        let mut ieee802154 = start_radio(p.TIMG0, p.IEEE802154);

        let mut acked = false;
        for seq in 0..30u8 {
            ieee802154.transmit(&data_frame(seq, true), false).ok();

            // Give the peer time to auto-ACK (the driver waits up to 200ms).
            Timer::after(Duration::from_millis(50)).await;
            if ieee802154.get_ack_frame().is_some() {
                acked = true;
                break;
            }

            Timer::after(Duration::from_millis(50)).await;
        }

        assert!(acked, "did not receive an ACK from the peer board");
    }

    /// The peer board echoes back the payload of every frame it receives, so
    /// the DUT should receive a frame carrying the payload it sent.
    #[test]
    async fn receives_echoed_frame(p: Peripherals) {
        let mut ieee802154 = start_radio(p.TIMG0, p.IEEE802154);

        assert!(
            receives_echo(&mut ieee802154).await,
            "did not receive an echoed frame from the peer board"
        );
    }

    /// A BLE controller created after the 802.15.4 driver, and dropped before it, must leave the
    /// running driver alone: creating it must not reset the 802.15.4 MAC (and the RF frontend
    /// under the PHY), and dropping it must not power down or gate what the driver still uses.
    #[test]
    async fn survives_ble_init_and_drop(p: Peripherals) {
        use esp_radio::ble::controller::BleConnector;

        let mut ieee802154 = start_radio(p.TIMG0, p.IEEE802154);

        let ble = BleConnector::new(p.BT, Default::default()).unwrap();
        assert!(
            receives_echo(&mut ieee802154).await,
            "802.15.4 stopped working once BLE was initialized"
        );

        drop(ble);
        assert!(
            receives_echo(&mut ieee802154).await,
            "802.15.4 stopped working once BLE was dropped"
        );
    }

    /// Sends frames to the peer board until it echoes one back.
    async fn receives_echo(ieee802154: &mut Ieee802154<'_>) -> bool {
        for seq in 0..30u8 {
            ieee802154.transmit(&data_frame(seq, true), false).ok();

            // Wait for the peer to auto-ACK and echo the frame back to us.
            for _ in 0..20 {
                Timer::after(Duration::from_millis(20)).await;
                if let Some(Ok(received)) = ieee802154.received()
                    && received.frame.payload.as_slice() == PAYLOAD
                {
                    return true;
                }
            }
        }

        false
    }
}
