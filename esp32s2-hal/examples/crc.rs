//! This shows example usage of the CRC functions in ROM

#![no_std]
#![no_main]

use core::fmt::Write;

use esp32s2_hal::{
    clock::ClockControl,
    peripherals::Peripherals,
    prelude::*,
    rom::{crc, md5},
    timer::TimerGroup,
    Rtc,
    Uart,
};
use esp_backtrace as _;
use nb::block;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let mut system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let timer_group0 = TimerGroup::new(
        peripherals.TIMG0,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut timer0 = timer_group0.timer0;
    let mut wdt = timer_group0.wdt;
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let mut serial0 = Uart::new(peripherals.UART0, &mut system.peripheral_clock_control);

    // Disable MWDT and RWDT (Watchdog) flash boot protection
    wdt.disable();
    rtc.rwdt.disable();

    timer0.start(1u64.secs());

    let data = "123456789";
    let sentence = "The quick brown fox jumps over a lazy dog";

    writeln!(
        serial0,
        "Performing CRC calculations on test string \"{data}\""
    )
    .unwrap();

    loop {
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

        // Hash the sentence one word at a time to *really* test the context
        // Use Peekable while iter_intersperse is unstable
        let mut md5_ctx = md5::Context::new();
        let mut it = sentence.split_whitespace().peekable();
        while let Some(word) = it.next() {
            md5_ctx.consume(word);
            if it.peek().is_some() {
                md5_ctx.consume(" ");
            }
        }
        let md5_digest = md5_ctx.compute();

        assert_eq!(
            md5_digest,
            md5::Digest([
                0x30, 0xde, 0xd8, 0x07, 0xd6, 0x5e, 0xe0, 0x37, 0x0f, 0xc6, 0xd7, 0x3d, 0x6a, 0xb5,
                0x5a, 0x95
            ])
        );

        writeln!(
            serial0,
            "{:08x} {:08x} {:08x} {:08x} {:04x} {:04x} {:02x} {:02x} {}",
            crc_hdlc,
            crc_bzip2,
            crc_mpeg2,
            crc_cksum,
            crc_kermit,
            crc_genibus,
            crc_rohc,
            crc_smbus,
            md5_digest
        )
        .unwrap();

        block!(timer0.wait()).unwrap();
    }
}
