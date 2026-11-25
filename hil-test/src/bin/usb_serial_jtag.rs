//! USB-Serial-JTAG Test
//!
//! The tests drive the USB-Serial-JTAG port, which the serial logger uses, so they log over RTT.

//% CHIP_FILTER: usb_serial_jtag_driver_supported
//% FEATURES: unstable log-rtt
//% DEFAULT-FEATURES: false

#![no_std]
#![no_main]

#[embedded_test::tests(default_timeout = 2)]
mod tests {
    use esp_hal::{peripherals::Peripherals, usb::usb_serial_jtag::Config};
    use hil_test as _;

    struct Context {
        p: Peripherals,
    }

    #[init]
    fn init() -> Context {
        Context {
            p: esp_hal::init(esp_hal::Config::default()),
        }
    }

    #[test]
    fn creating_peripheral_does_not_break_debug_connection(ctx: Context) {
        _ = hil_test::usb_serial_jtag!(ctx.p, Config::default())
            .unwrap()
            .into_async()
            .split();
    }

    #[test]
    #[cfg(all(usb_serial_jtag_phy_pads_usj, usb_serial_jtag_phy_pads_usb_fs))]
    fn usb_serial_jtag_rejects_the_pins_of_the_other_phy(ctx: Context) {
        use esp_hal::usb::usb_serial_jtag::ConfigError;

        let result = hil_test::usb_serial_jtag!(ctx.p, Config::default(), other_phy);
        assert!(matches!(result, Err(ConfigError::WrongPins)));
    }

    // These tests need a runner that reads the serial endpoint, as `probe-rs run` does. Without
    // a reader, the boot messages fill the output FIFO and a write never finishes. The qa example
    // `usb-serial-jtag-io` tests reads.
    #[test]
    fn usb_serial_jtag_reports_room_and_no_data(ctx: Context) {
        let mut usb_serial = hil_test::usb_serial_jtag!(ctx.p, Config::default()).unwrap();

        // Other programs on the runner can leave bytes in the input FIFO. A chip reset does not
        // clear the input FIFO.
        while usb_serial.read_buffered(&mut [0; 64]) != Ok(0) {}

        assert!(usb_serial.write_ready());
        assert!(!usb_serial.read_ready());
        assert_eq!(
            embedded_io::WriteReady::write_ready(&mut usb_serial),
            Ok(true)
        );
        assert_eq!(
            embedded_io::ReadReady::read_ready(&mut usb_serial),
            Ok(false)
        );
        assert_eq!(
            embedded_io_06::WriteReady::write_ready(&mut usb_serial),
            Ok(true)
        );
        assert_eq!(
            embedded_io_06::ReadReady::read_ready(&mut usb_serial),
            Ok(false)
        );

        let (mut rx, mut tx) = usb_serial.split();
        assert!(tx.write_ready());
        assert!(!rx.read_ready());
        assert_eq!(embedded_io::WriteReady::write_ready(&mut tx), Ok(true));
        assert_eq!(embedded_io::ReadReady::read_ready(&mut rx), Ok(false));
        assert_eq!(embedded_io_06::WriteReady::write_ready(&mut tx), Ok(true));
        assert_eq!(embedded_io_06::ReadReady::read_ready(&mut rx), Ok(false));
    }

    #[test]
    fn usb_serial_jtag_io_write_returns_after_one_packet(ctx: Context) {
        let mut usb_serial = hil_test::usb_serial_jtag!(ctx.p, Config::default()).unwrap();
        assert_eq!(
            embedded_io::Write::write(&mut usb_serial, &[0x55; 100]),
            Ok(64)
        );
        assert_eq!(
            embedded_io_06::Write::write(&mut usb_serial, &[0x55; 100]),
            Ok(64)
        );

        let (_, mut tx) = usb_serial.split();
        assert_eq!(embedded_io::Write::write(&mut tx, &[0x55; 100]), Ok(64));
        assert_eq!(embedded_io_06::Write::write(&mut tx, &[0x55; 100]), Ok(64));
    }

    #[test]
    fn usb_serial_jtag_io_write_returns_a_short_count(ctx: Context) {
        let (_, mut tx) = hil_test::usb_serial_jtag!(ctx.p, Config::default())
            .unwrap()
            .split();

        assert_eq!(embedded_io::Write::write(&mut tx, b"hello"), Ok(5));
        assert_eq!(embedded_io::Write::write(&mut tx, &[]), Ok(0));
    }

    #[test]
    fn usb_serial_jtag_io_flush_and_write_all_finish(ctx: Context) {
        let (_, mut tx) = hil_test::usb_serial_jtag!(ctx.p, Config::default())
            .unwrap()
            .split();

        assert_eq!(embedded_io::Write::write(&mut tx, b"hello"), Ok(5));
        assert_eq!(embedded_io::Write::flush(&mut tx), Ok(()));
        assert!(tx.write_ready());
        assert_eq!(
            embedded_io::Write::write_all(&mut tx, &[b'-'; 1000]),
            Ok(())
        );
        assert_eq!(embedded_io::Write::flush(&mut tx), Ok(()));
    }

    #[test]
    fn usb_serial_jtag_empty_buffers_return_zero(ctx: Context) {
        use embassy_futures::block_on;

        let mut usb_serial = hil_test::usb_serial_jtag!(ctx.p, Config::default())
            .unwrap()
            .into_async();

        assert_eq!(usb_serial.write(&[]), Ok(0));
        assert_eq!(usb_serial.read(&mut []), Ok(0));
        assert_eq!(usb_serial.read_buffered(&mut [0; 8]), Ok(0));
        assert_eq!(block_on(usb_serial.write_async(&[])), Ok(0));
        assert_eq!(block_on(usb_serial.read_async(&mut [])), Ok(0));
        assert_eq!(block_on(usb_serial.read_exact_async(&mut [])), Ok(()));
    }

    #[test]
    fn usb_serial_jtag_async_write_returns_after_one_packet(ctx: Context) {
        use embassy_futures::block_on;

        let (_, mut tx) = hil_test::usb_serial_jtag!(ctx.p, Config::default())
            .unwrap()
            .into_async()
            .split();

        assert_eq!(
            block_on(embedded_io_async::Write::write(&mut tx, &[0x55; 100])),
            Ok(64)
        );
        assert_eq!(block_on(tx.write_async(&[0x55; 100])), Ok(64));
        assert_eq!(block_on(tx.flush_async()), Ok(()));
    }

    #[test]
    fn usb_serial_jtag_write_after_flush_and_wait_finishes(ctx: Context) {
        use embassy_futures::block_on;

        let (_, mut tx) = hil_test::usb_serial_jtag!(ctx.p, Config::default())
            .unwrap()
            .into_async()
            .split();

        block_on(tx.wait_for_write_ready());
        assert_eq!(block_on(tx.write_async(b"hello")), Ok(5));
        assert_eq!(block_on(tx.flush_async()), Ok(()));
        block_on(tx.wait_for_write_ready());
        assert_eq!(block_on(tx.write_async(b"!")), Ok(1));
        assert_eq!(block_on(tx.flush_async()), Ok(()));
    }

    #[test]
    fn usb_serial_jtag_two_full_packets_without_flush_finish(ctx: Context) {
        let (_, mut tx) = hil_test::usb_serial_jtag!(ctx.p, Config::default())
            .unwrap()
            .split();

        assert_eq!(tx.write(&[b'-'; 64]), Ok(64));
        assert_eq!(tx.write(&[b'-'; 64]), Ok(64));
        assert_eq!(tx.flush(), Ok(()));
    }

    #[test]
    fn usb_serial_jtag_try_flush_sends_a_packet_once(ctx: Context) {
        let (_, mut tx) = hil_test::usb_serial_jtag!(ctx.p, Config::default())
            .unwrap()
            .split();

        // With nothing pending, a flush sends no packet, so the FIFO stays free.
        assert!(tx.try_flush());
        assert!(tx.write_ready());

        assert_eq!(tx.write(b"hello"), Ok(5));
        assert!(!tx.try_flush());
        while !tx.try_flush() {}

        assert!(tx.try_flush());
        assert!(tx.write_ready());
        assert_eq!(tx.flush(), Ok(()));
        assert!(tx.write_ready());
    }

    #[test]
    fn usb_serial_jtag_tx_timeout(ctx: Context) {
        use esp_hal::{
            time::Duration,
            usb::usb_serial_jtag::{TxConfig, TxError},
        };

        let config = Config::default().with_tx(TxConfig::default().with_timeout(Duration::ZERO));
        let (_, mut tx) = hil_test::usb_serial_jtag!(ctx.p, config).unwrap().split();

        // A full-speed host needs about 40 µs to take a packet of 63 or 64 bytes, so the
        // timeouts below can't race the host. A shorter packet would leave only a few µs.

        // The FIFO is busy from the 64th byte until the host takes the packet.
        assert_eq!(tx.write(&[b'-'; 64]), Ok(64));
        assert_eq!(tx.write(b"late"), Err(TxError::Timeout));
        while !tx.write_ready() {}
        assert_eq!(tx.write(&[b'-'; 63]), Ok(63));

        // The FIFO is busy from `WR_DONE` until the host takes the packet.
        assert_eq!(tx.flush(), Err(TxError::Timeout));

        tx.apply_config(&TxConfig::default()).unwrap();
        assert_eq!(tx.flush(), Ok(()));
        assert!(tx.write_ready());
    }

    #[test]
    fn usb_serial_jtag_rx_timeout(ctx: Context) {
        use esp_hal::{
            time::{Duration, Instant},
            usb::usb_serial_jtag::{IoError, RxConfig, RxError},
        };

        let timeout = Duration::from_millis(5);
        let config = Config::default().with_rx(RxConfig::default().with_timeout(timeout));
        let mut usb_serial = hil_test::usb_serial_jtag!(ctx.p, config).unwrap();
        let mut buf = [0u8; 8];

        // Other programs on the runner can leave bytes in the input FIFO. A chip reset does not
        // clear the input FIFO.
        while usb_serial.read_buffered(&mut [0; 64]) != Ok(0) {}

        let start = Instant::now();
        assert_eq!(usb_serial.read(&mut buf), Err(RxError::Timeout));
        assert!(start.elapsed() >= timeout);

        assert_eq!(
            embedded_io::Read::read(&mut usb_serial, &mut buf),
            Err(IoError::Rx(RxError::Timeout))
        );

        let (mut rx, _) = usb_serial.split();
        assert_eq!(
            embedded_io::Read::read(&mut rx, &mut buf),
            Err(RxError::Timeout)
        );
    }
}
