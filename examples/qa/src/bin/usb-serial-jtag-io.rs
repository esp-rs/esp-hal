//! Checks USB Serial/JTAG reads, and the data in both directions, against a host.
//!
//! The HIL tests cannot check reads, because nothing sends to the board there, and nothing
//! checks the bytes the board writes. Here a host program reads and writes the port.
//!
//! Flash the example, close any monitor, then run the host program:
//!
//! ```text
//! cd extras/usb-serial-jtag-host
//! cargo run --release
//! ```
//!
//! It finds the board's port. With more than one board connected, it lists the ports and takes
//! one as its argument, for example `cargo run --release -- /dev/ttyACM0`.
//!
//! The example repeats a ready line until the host program answers, so either can start first.
//! It checks the reads, then sends 1 MiB to the host and receives 1 MiB from it, and both sides
//! compare every byte with a pattern. It prints one line for each check and a summary, and the host
//! program exits with an error if any check fails.

//% CHIP_FILTER: usb_serial_jtag_driver_supported

#![no_std]
#![no_main]

use core::fmt::Write as _;

use esp_backtrace as _;
use esp_hal::{delay::Delay, main, usb::usb_serial_jtag::UsbSerialJtag};

esp_bootloader_esp_idf::esp_app_desc!();

const MEBIBYTE: usize = 1024 * 1024;

// `extras/usb-serial-jtag-host` must use the same seeds and the same generator.
const DEVICE_TO_HOST_SEED: u32 = 0x1234_5678;
const HOST_TO_DEVICE_SEED: u32 = 0x9abc_def0;

/// A xorshift32 byte stream, so a lost, repeated or reordered byte breaks every byte after it.
struct Pattern(u32);

impl Pattern {
    fn next_byte(&mut self) -> u8 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 17;
        self.0 ^= self.0 << 5;
        self.0 as u8
    }
}

#[derive(Default)]
struct Checks {
    passed: u32,
    failed: u32,
}

impl Checks {
    fn check(&mut self, out: &mut impl core::fmt::Write, name: &str, ok: bool) {
        if ok {
            self.passed += 1;
        } else {
            self.failed += 1;
        }
        _ = write!(out, "{} {}\r\n", if ok { "PASS" } else { "FAIL" }, name);
    }
}

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let mut usb = UsbSerialJtag::new(peripherals.USB_DEVICE);
    let mut checks = Checks::default();
    let delay = Delay::new();

    // The host sends only after it sees the ready line, so earlier bytes are not the host's.
    // The host system may read and drop the output while no program has the port open, so the
    // line repeats until the host answers.
    while usb.read_byte().is_ok() {}
    while !usb.read_ready() {
        _ = write!(usb, "usb-serial-jtag-io ready\r\n");
        delay.delay_millis(100);
    }

    let ready_07 = embedded_io_07::ReadReady::read_ready(&mut usb);
    let ready_06 = embedded_io_06::ReadReady::read_ready(&mut usb);
    let mut buf = [0u8; 8];
    let read = embedded_io_07::Read::read(&mut usb, &mut buf);
    let empty_after = !usb.read_ready();

    checks.check(
        &mut usb,
        "ReadReady 0.7 is true after the host sends",
        ready_07 == Ok(true),
    );
    checks.check(
        &mut usb,
        "ReadReady 0.6 is true after the host sends",
        ready_06 == Ok(true),
    );
    checks.check(
        &mut usb,
        "Read::read returns the three bytes",
        read == Ok(3) && &buf[..3] == b"abc",
    );
    let Ok(count) = read;
    _ = write!(usb, "read {} bytes: {:02x?}\r\n", count, &buf[..count]);
    checks.check(&mut usb, "read_ready is false after the read", empty_after);

    let (mut rx, mut tx) = usb.split();

    // The host compares these bytes with the same pattern.
    _ = write!(tx, "TX {}\r\n", MEBIBYTE);
    _ = tx.flush_tx();
    let mut pattern = Pattern(DEVICE_TO_HOST_SEED);
    let mut chunk = [0u8; 256];
    let mut sent_ok = true;
    for _ in 0..MEBIBYTE / chunk.len() {
        chunk.fill_with(|| pattern.next_byte());
        sent_ok &= embedded_io_07::Write::write_all(&mut tx, &chunk).is_ok();
    }
    _ = tx.flush_tx();
    checks.check(
        &mut tx,
        "Write::write_all of 1 MiB to the host returns",
        sent_ok,
    );

    // The host sends exactly this many bytes, so every read stops within them.
    _ = write!(tx, "RX {}\r\n", MEBIBYTE);
    _ = tx.flush_tx();
    let mut pattern = Pattern(HOST_TO_DEVICE_SEED);
    let (mut received, mut wrong) = (0, 0);
    while received < MEBIBYTE {
        let wanted = chunk.len().min(MEBIBYTE - received);
        let Ok(count) = embedded_io_07::Read::read(&mut rx, &mut chunk[..wanted]);
        wrong += chunk[..count]
            .iter()
            .filter(|byte| **byte != pattern.next_byte())
            .count();
        received += count;
    }
    checks.check(&mut tx, "1 MiB from the host arrives intact", wrong == 0);

    _ = write!(
        tx,
        "usb-serial-jtag-io: {} of {} checks passed\r\n",
        checks.passed,
        checks.passed + checks.failed
    );
    _ = tx.flush_tx();

    loop {}
}
