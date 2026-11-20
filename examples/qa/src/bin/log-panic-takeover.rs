//! Checks that a panic in the middle of a log write still prints the panic report.
//!
//! The log writer wraps UART0 and panics in the middle of the third record, while `esp-log`
//! holds the writer. esp-backtrace then logs the panic report through the same logger.
//!
//! Expected output on UART0:
//!
//! ```text
//! INFO - log-panic-takeover: first record
//! INFO - log-panic-takeover: second record
//! INFO - log-panic-takeover: the writer panics after
//! ERROR - ====================== PANIC ======================
//! ERROR - panicked at ...: the log writer panicked in the middle of a record
//! ERROR - Backtrace: (or a STACKDUMP line on RISC-V without frame pointers)
//! ...
//! ```
//!
//! The third record stops at "after". The report starts on a new line and prints every line, not
//! only the first one. "never printed" must not appear.

#![no_std]
#![no_main]

use embedded_io_07::{ErrorType, Write};
use esp_backtrace as _;
use esp_hal::{Blocking, main, uart::UartTx};
use static_cell::StaticCell;

esp_bootloader_esp_idf::esp_app_desc!();

/// Bytes after this marker make the writer panic, once.
const MARKER: u8 = b'#';

struct PanicMidRecord {
    uart: UartTx<'static, Blocking>,
    armed: bool,
}

impl ErrorType for PanicMidRecord {
    type Error = <UartTx<'static, Blocking> as ErrorType>::Error;
}

impl Write for PanicMidRecord {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        if self.armed
            && let Some(marker) = buf.iter().position(|&b| b == MARKER)
        {
            self.armed = false;
            self.uart.write_all(&buf[..marker])?;
            self.uart.flush()?;
            panic!("the log writer panicked in the middle of a record");
        }
        Write::write(&mut self.uart, buf)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        Write::flush(&mut self.uart)
    }
}

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    static WRITER: StaticCell<PanicMidRecord> = StaticCell::new();
    esp_log::init(WRITER.init(PanicMidRecord {
        uart: qa_test::console_uart!(peripherals),
        armed: true,
    }));

    log::info!("log-panic-takeover: first record");
    log::info!("log-panic-takeover: second record");
    log::info!("log-panic-takeover: the writer panics after # never printed");

    unreachable!("the third record panics");
}
