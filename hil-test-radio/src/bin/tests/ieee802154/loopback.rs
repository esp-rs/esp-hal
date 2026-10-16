#[embedded_test::tests(default_timeout = 30, executor = esp_rtos::embassy::Executor::new())]
mod tests {
    use core::sync::atomic::{AtomicBool, Ordering};

    use embassy_time::{Duration, Instant, Timer};
    use esp_hal::{
        clock::CpuClock,
        peripherals::{IEEE802154, Peripherals, TIMG0},
        timer::timg::TimerGroup,
    };
    use esp_radio::ieee802154::{Config, Frame, Ieee802154};
    use hil_test::ieee802154::{
        CHANNEL,
        DUT_ADDRESS,
        PAN_ID,
        PAYLOAD,
        PAYLOAD_ACKED,
        SUPPORT_ADDRESS,
    };
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
        data_frame_with_payload(seq, ack_request, PAYLOAD)
    }

    fn data_frame_with_payload(seq: u8, ack_request: bool, payload: &[u8]) -> Frame {
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
            payload: payload.to_vec(),
            footer: [0u8; 2],
        }
    }

    static TX_DONE: AtomicBool = AtomicBool::new(false);
    static TX_FAILED: AtomicBool = AtomicBool::new(false);

    fn on_tx_done() {
        TX_DONE.store(true, Ordering::Relaxed);
    }

    fn on_tx_failed() {
        TX_FAILED.store(true, Ordering::Relaxed);
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

    /// The peer board echoes an ACK-requesting frame back when it receives `PAYLOAD_ACKED`. The
    /// DUT acknowledges it, and only then delivers it - so it must arrive.
    #[test]
    async fn receives_acknowledged_echo(p: Peripherals) {
        let mut ieee802154 = start_radio(p.TIMG0, p.IEEE802154);

        let mut echoed = false;
        'outer: for seq in 0..30u8 {
            ieee802154
                .transmit(&data_frame_with_payload(seq, true, PAYLOAD_ACKED), false)
                .ok();

            for _ in 0..20 {
                Timer::after(Duration::from_millis(20)).await;
                if let Some(Ok(received)) = ieee802154.received()
                    && received.frame.payload.as_slice() == PAYLOAD_ACKED
                {
                    assert!(
                        received.frame.header.ack_request,
                        "the echoed frame does not request an ACK"
                    );
                    echoed = true;
                    break 'outer;
                }
            }
        }

        assert!(
            echoed,
            "did not receive the acknowledged echo from the peer board"
        );
    }

    /// Once asleep, the DUT receives nothing - not the echo of the frame it has just sent - until
    /// `start_receive`.
    #[test]
    async fn sleep_stops_reception_until_start_receive(p: Peripherals) {
        let mut ieee802154 = start_radio(p.TIMG0, p.IEEE802154);
        ieee802154.set_tx_done_callback_fn(on_tx_done);

        for seq in 0..5u8 {
            TX_DONE.store(false, Ordering::Relaxed);
            ieee802154.transmit(&data_frame(seq, false), false).ok();

            // `rx_when_idle` turns the receiver back on after the transmission. Put the radio to
            // sleep right then: the peer needs far longer than this to echo the frame back.
            let deadline = Instant::now() + Duration::from_millis(50);
            while !TX_DONE.load(Ordering::Relaxed) && Instant::now() < deadline {}
            assert!(
                TX_DONE.load(Ordering::Relaxed),
                "the transmission did not complete"
            );

            ieee802154.sleep();

            Timer::after(Duration::from_millis(200)).await;
            assert!(
                ieee802154.received().is_none(),
                "received a frame while asleep"
            );
        }

        // Awake again, the echoes arrive - which also shows that the peer did echo above.
        ieee802154.start_receive();

        let mut echoed = false;
        'outer: for seq in 0..30u8 {
            ieee802154.transmit(&data_frame(seq, true), false).ok();

            for _ in 0..20 {
                Timer::after(Duration::from_millis(20)).await;
                if let Some(Ok(received)) = ieee802154.received()
                    && received.frame.payload.as_slice() == PAYLOAD
                {
                    echoed = true;
                    break 'outer;
                }
            }
        }

        assert!(
            echoed,
            "did not receive an echoed frame after start_receive"
        );
    }

    /// `sleep` stops a transmission that is in progress, as the ESP-IDF driver does: it fails.
    #[test]
    async fn sleep_aborts_transmission(p: Peripherals) {
        let mut ieee802154 = start_radio(p.TIMG0, p.IEEE802154);
        ieee802154.set_tx_done_callback_fn(on_tx_done);
        ieee802154.set_tx_failed_callback_fn(on_tx_failed);

        TX_DONE.store(false, Ordering::Relaxed);
        TX_FAILED.store(false, Ordering::Relaxed);

        // The frame, and the ACK it waits for, take far longer to go over the air than it takes to
        // get to the `sleep` call.
        ieee802154.transmit(&data_frame(0, true), false).ok();
        ieee802154.sleep();

        assert!(
            TX_FAILED.load(Ordering::Relaxed),
            "the stopped transmission did not fail"
        );
        assert!(
            !TX_DONE.load(Ordering::Relaxed),
            "the stopped transmission completed"
        );
    }
}
