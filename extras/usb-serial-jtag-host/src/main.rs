//! The host side of the qa example `usb-serial-jtag-io`.
//!
//! Flash the example, then run `cargo run --release`. It finds the board's USB Serial/JTAG port,
//! or takes the port as its argument when more than one board is connected. It sends the bytes
//! the example reads, prints the example's output, takes part in the 1 MiB transfer each way, and
//! exits with an error if any check fails.

use std::{
    io::{self, Read, Write},
    process::ExitCode,
    time::{Duration, Instant},
};

use serialport::{SerialPort, SerialPortType};

const ESPRESSIF_VID: u16 = 0x303a;
const USB_SERIAL_JTAG_PID: u16 = 0x1001;

// The example must use the same seeds and the same generator.
const DEVICE_TO_HOST_SEED: u32 = 0x1234_5678;
const HOST_TO_DEVICE_SEED: u32 = 0x9abc_def0;

/// The line the example repeats until the host answers.
const READY: &str = "usb-serial-jtag-io ready";

/// How long the port may stay silent before the run fails.
const SILENCE: Duration = Duration::from_secs(10);

/// The port timeout while the host program reads.
const READ_TIMEOUT: Duration = Duration::from_millis(200);

/// The port timeout while the host program sends a block to the board.
const WRITE_TIMEOUT: Duration = Duration::from_secs(60);

/// The longest time a `TIMED` transfer may take. The board sends nothing for 500 ms after it, so
/// a packet that waits for later data takes longer.
const TIMED_LIMIT: Duration = Duration::from_millis(250);

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

/// The port, with the bytes read past the last line kept for the next read.
struct Device {
    port: Box<dyn SerialPort>,
    pending: Vec<u8>,
}

impl Device {
    /// Reads at least one more byte into `pending`, or fails after `SILENCE`.
    fn fill(&mut self) -> io::Result<()> {
        let deadline = Instant::now() + SILENCE;
        let mut buf = [0u8; 4096];
        loop {
            match self.port.read(&mut buf) {
                Ok(0) => {}
                Ok(count) => {
                    self.pending.extend_from_slice(&buf[..count]);
                    return Ok(());
                }
                Err(error) if error.kind() == io::ErrorKind::TimedOut => {}
                Err(error) => return Err(error),
            }
            if Instant::now() > deadline {
                return Err(io::Error::new(
                    io::ErrorKind::TimedOut,
                    "the board sent nothing: reset it and run this again",
                ));
            }
        }
    }

    fn read_line(&mut self) -> io::Result<String> {
        loop {
            if let Some(end) = self.pending.iter().position(|byte| *byte == b'\n') {
                let line: Vec<u8> = self.pending.drain(..=end).collect();
                return Ok(String::from_utf8_lossy(&line).trim_end().to_string());
            }
            self.fill()?;
        }
    }

    /// Reads `count` bytes and returns how many differ from the pattern.
    fn compare(&mut self, count: usize, pattern: &mut Pattern) -> io::Result<usize> {
        let (mut seen, mut wrong) = (0, 0);
        while seen < count {
            if self.pending.is_empty() {
                self.fill()?;
            }
            let take = self.pending.len().min(count - seen);
            wrong += self
                .pending
                .drain(..take)
                .filter(|byte| *byte != pattern.next_byte())
                .count();
            seen += take;
        }
        Ok(wrong)
    }
}

fn count_in(line: &str, prefix: &str) -> Option<usize> {
    line.strip_prefix(prefix)?.trim().parse().ok()
}

fn run(port_name: &str) -> io::Result<bool> {
    let port = serialport::new(port_name, 115_200)
        .timeout(READ_TIMEOUT)
        .open()?;
    let mut device = Device {
        port,
        pending: Vec::new(),
    };

    // The board discards what arrives before it repeats this line, so nothing is sent earlier.
    while device.read_line()? != READY {}
    device.port.write_all(b"abc")?;

    let mut ok = true;
    loop {
        let line = device.read_line()?;
        if line == READY {
            continue;
        }
        println!("{line}");
        if line.starts_with("FAIL") {
            ok = false;
        }
        if let Some(count) = count_in(&line, "TX ") {
            let wrong = device.compare(count, &mut Pattern(DEVICE_TO_HOST_SEED))?;
            println!("host: {count} bytes from the board, {wrong} wrong");
            ok &= wrong == 0;
        } else if let Some(count) = count_in(&line, "TIMED ") {
            let start = Instant::now();
            let wrong = device.compare(count, &mut Pattern(DEVICE_TO_HOST_SEED))?;
            let elapsed = start.elapsed();
            let on_time = elapsed <= TIMED_LIMIT;
            println!(
                "{} host: {count} bytes from the board after {} ms, {wrong} wrong",
                if on_time && wrong == 0 {
                    "PASS"
                } else {
                    "FAIL"
                },
                elapsed.as_millis()
            );
            ok &= on_time && wrong == 0;
        } else if let Some(count) = count_in(&line, "RX ") {
            let mut pattern = Pattern(HOST_TO_DEVICE_SEED);
            let bytes: Vec<u8> = (0..count).map(|_| pattern.next_byte()).collect();
            // On Windows, the timeout limits the whole write, and 1 MiB takes seconds.
            device.port.set_timeout(WRITE_TIMEOUT)?;
            device.port.write_all(&bytes)?;
            device.port.flush()?;
            device.port.set_timeout(READ_TIMEOUT)?;
            println!("host: {count} bytes sent to the board");
        } else if line.starts_with("usb-serial-jtag-io:") {
            return Ok(ok);
        }
    }
}

/// Finds the only USB Serial/JTAG port. Every chip reports this port with the same USB IDs.
fn find_port() -> Result<String, String> {
    let ports = serialport::available_ports().map_err(|error| error.to_string())?;
    let boards: Vec<(String, String)> = ports
        .into_iter()
        .filter_map(|port| match port.port_type {
            // macOS lists each device twice, and the call-out device is the one to open.
            SerialPortType::UsbPort(usb)
                if usb.vid == ESPRESSIF_VID
                    && usb.pid == USB_SERIAL_JTAG_PID
                    && !port.port_name.starts_with("/dev/tty.") =>
            {
                Some((port.port_name, usb.serial_number.unwrap_or_default()))
            }
            _ => None,
        })
        .collect();
    match boards.as_slice() {
        [(name, _)] => Ok(name.clone()),
        [] => Err("no USB Serial/JTAG port found: connect the board, or name its port".into()),
        _ => {
            let list: Vec<String> = boards
                .iter()
                .map(|(name, serial)| format!("  {name}  {serial}"))
                .collect();
            Err(format!(
                "more than one USB Serial/JTAG port, name one:\n{}",
                list.join("\n")
            ))
        }
    }
}

fn main() -> ExitCode {
    let port_name = match std::env::args().nth(1).map_or_else(find_port, Ok) {
        Ok(port_name) => port_name,
        Err(error) => {
            eprintln!("host: {error}");
            return ExitCode::FAILURE;
        }
    };
    match run(&port_name) {
        Ok(true) => ExitCode::SUCCESS,
        Ok(false) => {
            eprintln!("host: a check failed");
            ExitCode::FAILURE
        }
        Err(error) => {
            eprintln!("host: {error}");
            ExitCode::FAILURE
        }
    }
}
