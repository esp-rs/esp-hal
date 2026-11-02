#[embedded_test::tests(default_timeout = 2)]
mod tests {
    use embedded_hal::delay::DelayNs;
    use esp_hal::{
        delay::Delay,
        peripherals::Peripherals,
        rom::{crc, md5},
        time::{Duration, Instant},
    };
    use hil_test as _;

    fn time_moves_forward_during<F: FnOnce(Context)>(ctx: Context, f: F) {
        let t1 = Instant::now();
        f(ctx);
        let t2 = Instant::now();

        assert!(t2 > t1);
    }

    struct Context {
        p: Peripherals,
    }

    #[init]
    fn init() -> Context {
        Context {
            p: esp_hal::init(esp_hal::Config::default()),
        }
    }

    // Test time

    #[test]
    fn duration_since_epoch_is_not_relative_to_now() {
        let now = Instant::EPOCH;

        Delay::new().delay_ns(10_000);

        assert_eq!(now.duration_since_epoch(), Duration::ZERO);
    }

    #[test]
    fn large_instant_difference_does_not_panic() {
        assert_eq!(
            (Instant::EPOCH + Duration::MAX).duration_since_epoch(),
            Duration::MAX
        );
    }

    #[cfg(systimer_driver_supported)]
    #[test]
    fn test_current_time_construct_systimer(ctx: Context) {
        time_moves_forward_during(ctx, |ctx| {
            // construct the timer in between calls to current_time
            let _ = esp_hal::timer::systimer::SystemTimer::new(ctx.p.SYSTIMER);
        })
    }

    #[cfg(esp32)]
    #[test]
    fn test_current_time_construct_timg0(ctx: Context) {
        time_moves_forward_during(ctx, |ctx| {
            // construct the timer in between calls to current_time
            let _ = esp_hal::timer::timg::TimerGroup::new(ctx.p.TIMG0);
        })
    }

    #[test]
    fn delay_ns() {
        let t1 = Instant::now();
        Delay::new().delay_ns(600_000);
        let t2 = Instant::now();

        assert!(t2 > t1);
        assert!(
            (t2 - t1).as_micros() >= 600u64,
            "diff: {:?}",
            (t2 - t1).as_micros()
        );
    }

    #[test]
    fn delay_70millis() {
        let t1 = Instant::now();
        Delay::new().delay_millis(70);
        let t2 = Instant::now();

        assert!(t2 > t1);
        assert!(
            (t2 - t1).as_millis() >= 70u64,
            "diff: {:?}",
            (t2 - t1).as_millis()
        );
    }

    #[test]
    fn delay_1_500us() {
        let t1 = Instant::now();
        Delay::new().delay_us(1_500);
        let t2 = Instant::now();

        assert!(t2 > t1);
        assert!(
            (t2 - t1).as_micros() >= 1_500u64,
            "diff: {:?}",
            (t2 - t1).as_micros()
        );
    }

    #[test]
    fn delay_300ms() {
        let t1 = Instant::now();
        Delay::new().delay_ms(300);
        let t2 = Instant::now();

        assert!(t2 > t1);
        assert!(
            (t2 - t1).as_millis() >= 300u64,
            "diff: {:?}",
            (t2 - t1).as_millis()
        );
    }

    // Test ROM functions

    #[test]
    fn test_crc() {
        let data = "123456789";

        let crc_hdlc = crc::crc32_le(!0xffffffff, data.as_ref());
        let crc_bzip2 = crc::crc32_be(!0xffffffff, data.as_ref());
        let crc_mpeg2 = !crc::crc32_be(!0xffffffff, data.as_ref());
        let crc_cksum = crc::crc32_be(!0, data.as_ref());
        let crc_kermit = !crc::crc16_le(!0, data.as_ref());
        let crc_genibus = crc::crc16_be(!0xffff, data.as_ref());
        let crc_rohc = !crc::crc8_le(!0xff, data.as_ref());
        let crc_smbus = !crc::crc8_be(!0, data.as_ref());

        assert_eq!(crc_hdlc, 0xcbf43926);
        assert_eq!(crc_bzip2, 0xfc891918);
        assert_eq!(crc_mpeg2, 0x0376e6e7);
        assert_eq!(crc_cksum, 0x765e7680);
        assert_eq!(crc_kermit, 0x2189);
        assert_eq!(crc_genibus, 0xd64e);
        assert_eq!(crc_rohc, 0xd0);
        assert_eq!(crc_smbus, 0xf4);
    }

    #[test]
    fn test_crc_rom_function() {
        let crc = esp_bootloader_esp_idf::Crc32ForTesting::new();
        let res = crc.crc(&[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16]);
        assert_eq!(res, 436745307);
    }

    #[test]
    fn test_md5() {
        let sentence = "The quick brown fox jumps over a lazy dog";

        let mut md5_ctx = md5::Context::new();
        let mut it = sentence.split_whitespace().peekable();
        while let Some(word) = it.next() {
            md5_ctx.consume(word);
            if it.peek().is_some() {
                md5_ctx.consume(" ");
            }
        }
        let md5_digest = md5_ctx.compute();

        let expected_md5_digest = [
            0x30, 0xde, 0xd8, 0x07, 0xd6, 0x5e, 0xe0, 0x37, 0x0f, 0xc6, 0xd7, 0x3d, 0x6a, 0xb5,
            0x5a, 0x95,
        ];

        assert_eq!(expected_md5_digest, *md5_digest);
    }

    /// This is just to check that we can create a bootable image
    /// if there is something with a "specific" alignment in `.rodata`
    /// on chips with "D/I vaddr are shared" (SOC_MMU_DI_VADDR_SHARED)
    /// (Which is C6 and later)
    ///
    /// The gap produced previously made tooling create three image segments but
    /// the bootloader will refuse to boot anything with >2 segments in flash.
    ///
    /// Unfortunately the way things fail if this fails is "unfortunate"
    /// (i.e. every test here fails with an unhelpful `ERROR probe_rs_debug::debug_info:Other("Stack
    /// pointer is too far away to unwind")``)
    #[test]
    fn can_boot() {
        #[repr(align(64))]
        struct Aligned {
            _data: [u8; 128],
        }

        #[used]
        static ALIGNED: Aligned = Aligned { _data: [0; 128] };
    }

    #[test]
    #[cfg(usb_serial_jtag_driver_supported)]
    fn creating_peripheral_does_not_break_debug_connection(ctx: Context) {
        use esp_hal::usb::usb_serial_jtag::UsbSerialJtag;

        _ = UsbSerialJtag::new(ctx.p.USB_DEVICE).into_async().split();
    }

    // These tests need a runner that reads the serial endpoint, as `probe-rs run` does. Without
    // a reader, the boot messages fill the output FIFO and a write never finishes. Nothing sends
    // to the board, so the qa example `usb-serial-jtag-io` tests reads.
    #[test]
    #[cfg(usb_serial_jtag_driver_supported)]
    fn usb_serial_jtag_reports_room_and_no_data(ctx: Context) {
        use esp_hal::usb::usb_serial_jtag::UsbSerialJtag;

        let mut usb_serial = UsbSerialJtag::new(ctx.p.USB_DEVICE);

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
    #[cfg(usb_serial_jtag_driver_supported)]
    fn usb_serial_jtag_io_write_returns_after_one_packet(ctx: Context) {
        use esp_hal::usb::usb_serial_jtag::UsbSerialJtag;

        let mut usb_serial = UsbSerialJtag::new(ctx.p.USB_DEVICE);
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
    #[cfg(usb_serial_jtag_driver_supported)]
    fn usb_serial_jtag_io_write_returns_a_short_count(ctx: Context) {
        use esp_hal::usb::usb_serial_jtag::UsbSerialJtag;

        let (_, mut tx) = UsbSerialJtag::new(ctx.p.USB_DEVICE).split();

        assert_eq!(embedded_io::Write::write(&mut tx, b"hello"), Ok(5));
        assert_eq!(embedded_io::Write::write(&mut tx, &[]), Ok(0));
    }

    #[test]
    #[cfg(usb_serial_jtag_driver_supported)]
    fn usb_serial_jtag_io_flush_and_write_all_finish(ctx: Context) {
        use esp_hal::usb::usb_serial_jtag::UsbSerialJtag;

        let (_, mut tx) = UsbSerialJtag::new(ctx.p.USB_DEVICE).split();

        assert_eq!(embedded_io::Write::write(&mut tx, b"hello"), Ok(5));
        assert_eq!(embedded_io::Write::flush(&mut tx), Ok(()));
        assert!(tx.write_ready());
        assert_eq!(
            embedded_io::Write::write_all(&mut tx, &[b'-'; 1000]),
            Ok(())
        );
    }
}
