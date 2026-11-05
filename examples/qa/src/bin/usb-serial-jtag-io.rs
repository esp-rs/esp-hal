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
//! It checks the reads, then sends 1 MiB to the host and receives 1 MiB from it, first with the
//! blocking driver and then with the async driver, and both sides compare every byte with a
//! pattern. The async transfers cancel many writes and reads. It also checks that the host gets
//! a full 64-byte packet at once after a flush, also after a cancelled flush. It prints one line
//! for each check and a summary, and the host program exits with an error if any check fails.

//% CHIP_FILTER: usb_serial_jtag_driver_supported

#![no_std]
#![no_main]

use core::{
    fmt::Write as _,
    future::Future,
    pin::pin,
    task::{Context, Waker},
};

use embassy_futures::{
    block_on,
    select::{Either, select},
    yield_now,
};
use esp_backtrace as _;
use esp_hal::{
    Async,
    DriverMode,
    delay::Delay,
    main,
    usb::usb_serial_jtag::{Config, UsbSerialJtag, UsbSerialJtagRx, UsbSerialJtagTx},
};

esp_bootloader_esp_idf::esp_app_desc!();

const MEBIBYTE: usize = 1024 * 1024;

// `extras/usb-serial-jtag-host` must use the same seeds and the same generator.
const DEVICE_TO_HOST_SEED: u32 = 0x1234_5678;
const HOST_TO_DEVICE_SEED: u32 = 0x9abc_def0;

/// How long the board sends nothing after a timed transfer. The host program fails a timed
/// transfer that takes more than half of this time.
const HOLD_MS: u32 = 500;

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

/// Writes a line and sends it to the host.
fn say<Dm: DriverMode>(tx: &mut UsbSerialJtagTx<'_, Dm>, args: core::fmt::Arguments<'_>) {
    _ = tx.write_fmt(args);
    _ = tx.flush();
}

#[derive(Default)]
struct Checks {
    passed: u32,
    failed: u32,
}

impl Checks {
    fn check<Dm: DriverMode>(&mut self, tx: &mut UsbSerialJtagTx<'_, Dm>, name: &str, ok: bool) {
        if ok {
            self.passed += 1;
        } else {
            self.failed += 1;
        }
        say(
            tx,
            format_args!("{} {}\r\n", if ok { "PASS" } else { "FAIL" }, name),
        );
    }
}

/// Polls `future` once, then drops it.
fn poll_once(future: impl Future) {
    let mut future = pin!(future);
    _ = future
        .as_mut()
        .poll(&mut Context::from_waker(Waker::noop()));
}

/// Receives 1 MiB and returns how many bytes differ from the pattern.
fn receive<Dm: DriverMode>(rx: &mut UsbSerialJtagRx<'_, Dm>) -> usize {
    let mut pattern = Pattern(HOST_TO_DEVICE_SEED);
    let mut chunk = [0u8; 256];
    let (mut received, mut wrong) = (0, 0);
    while received < MEBIBYTE {
        let wanted = chunk.len().min(MEBIBYTE - received);
        let count = embedded_io_07::Read::read(rx, &mut chunk[..wanted]).unwrap_or(0);
        wrong += chunk[..count]
            .iter()
            .filter(|byte| **byte != pattern.next_byte())
            .count();
        received += count;
    }
    wrong
}

#[main]
fn main() -> ! {
    let mut peripherals = esp_hal::init(esp_hal::Config::default());
    let mut checks = Checks::default();
    let delay = Delay::new();

    let (mut dp, mut dm) = cfg_select! {
        feature = "esp32c3" => (peripherals.GPIO19, peripherals.GPIO18),
        any(feature = "esp32c5", feature = "esp32h4") => (peripherals.GPIO14, peripherals.GPIO13),
        any(feature = "esp32c6", feature = "esp32c61") => (peripherals.GPIO13, peripherals.GPIO12),
        feature = "esp32h2" => (peripherals.GPIO27, peripherals.GPIO26),
        feature = "esp32p4" => (peripherals.GPIO25, peripherals.GPIO24),
        feature = "esp32s3" => (peripherals.GPIO20, peripherals.GPIO19),
        feature = "esp32s31" => (peripherals.GPIO34, peripherals.GPIO33),
    };

    let (mut rx, mut tx) = UsbSerialJtag::new(
        peripherals.USB_DEVICE.reborrow(),
        dp.reborrow(),
        dm.reborrow(),
        Config::default(),
    )
    .unwrap()
    .split();

    // The host sends only after it sees the ready line, so earlier bytes are not the host's.
    // The host system may read and drop the output while no program has the port open, so the
    // line repeats until the host answers.
    let mut buf = [0u8; 8];
    while rx.read_buffered(&mut buf) != Ok(0) {}
    while !rx.read_ready() {
        say(&mut tx, format_args!("usb-serial-jtag-io ready\r\n"));
        delay.delay_millis(100);
    }

    let ready_07 = embedded_io_07::ReadReady::read_ready(&mut rx);
    let ready_06 = embedded_io_06::ReadReady::read_ready(&mut rx);
    let read = embedded_io_07::Read::read(&mut rx, &mut buf);
    let empty_after = !rx.read_ready();

    checks.check(
        &mut tx,
        "ReadReady 0.7 is true after the host sends",
        ready_07 == Ok(true),
    );
    checks.check(
        &mut tx,
        "ReadReady 0.6 is true after the host sends",
        ready_06 == Ok(true),
    );
    checks.check(
        &mut tx,
        "Read::read returns the three bytes",
        read == Ok(3) && &buf[..3] == b"abc",
    );
    let count = read.unwrap_or(0);
    say(
        &mut tx,
        format_args!("read {} bytes: {:02x?}\r\n", count, &buf[..count]),
    );
    checks.check(&mut tx, "read_ready is false after the read", empty_after);

    // The host compares these bytes with the same pattern.
    say(&mut tx, format_args!("TX {}\r\n", MEBIBYTE));
    let mut pattern = Pattern(DEVICE_TO_HOST_SEED);
    let mut chunk = [0u8; 256];
    let mut sent_ok = true;
    for _ in 0..MEBIBYTE / chunk.len() {
        chunk.fill_with(|| pattern.next_byte());
        sent_ok &= embedded_io_07::Write::write_all(&mut tx, &chunk).is_ok();
    }
    sent_ok &= embedded_io_07::Write::flush(&mut tx).is_ok();
    checks.check(
        &mut tx,
        "Write::write_all of 1 MiB to the host returns",
        sent_ok,
    );

    // The host sends exactly this many bytes, so every read stops within them.
    say(&mut tx, format_args!("RX {}\r\n", MEBIBYTE));
    let wrong = receive(&mut rx);
    checks.check(&mut tx, "1 MiB from the host arrives intact", wrong == 0);

    // A 64-byte packet does not end the transfer. Only the flush lets the host see it at once.
    say(&mut tx, format_args!("TIMED 64\r\n"));
    let mut pattern = Pattern(DEVICE_TO_HOST_SEED);
    let packet: [u8; 64] = core::array::from_fn(|_| pattern.next_byte());
    let written = tx.write(&packet);
    _ = tx.flush();
    delay.delay_millis(HOLD_MS);
    checks.check(
        &mut tx,
        "write of 64 bytes stores one packet",
        written == Ok(64),
    );

    drop((rx, tx));
    let (mut rx, mut tx) = UsbSerialJtag::new(peripherals.USB_DEVICE, dp, dm, Config::default())
        .unwrap()
        .into_async()
        .split();

    say(&mut tx, format_args!("TIMED 74\r\n"));
    let cancelled_flush_ok = block_on(cancelled_flushes(&mut tx, &delay));
    checks.check(
        &mut tx,
        "flush_async finishes after a cancelled flush_async",
        cancelled_flush_ok,
    );

    say(&mut tx, format_args!("TX {}\r\n", MEBIBYTE));
    let cancelled_writes = block_on(send_async(&mut tx));
    say(
        &mut tx,
        format_args!("write_async cancelled {} times\r\n", cancelled_writes),
    );
    checks.check(
        &mut tx,
        "write_async is cancelled during the transfer",
        cancelled_writes > 0,
    );

    say(&mut tx, format_args!("RX {}\r\n", MEBIBYTE));
    let (wrong, cancelled_reads) = block_on(receive_async(&mut rx));
    say(
        &mut tx,
        format_args!("read_async cancelled {} times\r\n", cancelled_reads),
    );
    checks.check(
        &mut tx,
        "1 MiB from the host arrives intact through cancelled read_async calls",
        wrong == 0 && cancelled_reads > 0,
    );

    say(
        &mut tx,
        format_args!(
            "usb-serial-jtag-io: {} of {} checks passed\r\n",
            checks.passed,
            checks.passed + checks.failed
        ),
    );

    loop {}
}

/// Sends 10 bytes, then 64 bytes, and drops a flush after it started each packet.
async fn cancelled_flushes(tx: &mut UsbSerialJtagTx<'_, Async>, delay: &Delay) -> bool {
    let mut pattern = Pattern(DEVICE_TO_HOST_SEED);
    let data: [u8; 74] = core::array::from_fn(|_| pattern.next_byte());
    let (short, full) = data.split_at(10);

    let mut ok = write_all_async(tx, short).await;
    // The first poll sets `WR_DONE` and then waits for the host.
    poll_once(tx.flush_async());
    ok &= tx.flush_async().await.is_ok();

    ok &= write_all_async(tx, full).await;
    tx.wait_for_write_ready().await;
    // The first poll sends the zero-length packet and then waits for the host.
    poll_once(tx.flush_async());
    ok &= tx.flush_async().await.is_ok();

    delay.delay_millis(HOLD_MS);
    ok
}

async fn write_all_async(tx: &mut UsbSerialJtagTx<'_, Async>, mut data: &[u8]) -> bool {
    while !data.is_empty() {
        match tx.write_async(data).await {
            Ok(written) => data = &data[written..],
            Err(_) => return false,
        }
    }
    true
}

/// Sends 1 MiB of the pattern and returns how many writes were cancelled.
async fn send_async(tx: &mut UsbSerialJtagTx<'_, Async>) -> u32 {
    let mut pattern = Pattern(DEVICE_TO_HOST_SEED);
    let mut chunk = [0u8; 256];
    let mut cancelled = 0;
    for _ in 0..MEBIBYTE / chunk.len() {
        chunk.fill_with(|| pattern.next_byte());
        let mut data = &chunk[..];
        while !data.is_empty() {
            match select(tx.write_async(data), yield_now()).await {
                Either::First(Ok(written)) => data = &data[written..],
                Either::First(Err(error)) => panic!("write_async failed: {error}"),
                Either::Second(()) => cancelled += 1,
            }
        }
    }
    _ = tx.flush_async().await;
    cancelled
}

/// Receives 1 MiB, and returns how many bytes differ from the pattern and how many reads were
/// cancelled.
async fn receive_async(rx: &mut UsbSerialJtagRx<'_, Async>) -> (usize, u32) {
    let mut pattern = Pattern(HOST_TO_DEVICE_SEED);
    let mut chunk = [0u8; 256];
    let (mut received, mut wrong, mut cancelled) = (0, 0, 0);
    while received < MEBIBYTE {
        let wanted = chunk.len().min(MEBIBYTE - received);
        let count = match select(rx.read_async(&mut chunk[..wanted]), yield_now()).await {
            Either::First(Ok(count)) => count,
            Either::First(Err(error)) => panic!("read_async failed: {error}"),
            Either::Second(()) => {
                cancelled += 1;
                continue;
            }
        };
        wrong += chunk[..count]
            .iter()
            .filter(|byte| **byte != pattern.next_byte())
            .count();
        received += count;
    }
    (wrong, cancelled)
}
