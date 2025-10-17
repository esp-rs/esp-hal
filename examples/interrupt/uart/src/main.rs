//! Example of responding to UART interrupts.
//!
//! The following wiring is assumed:
//! - RX => GPIO3 (ESP32), GPIO19 (ESP32-C2), GPIO20 (ESP32-C3), GPIO17 (ESP32-C6), GPIO23
//!   (ESP32-H2), GPIO44 (ESP32-S2/S3)

#![no_std]
#![no_main]

use core::cell::RefCell;

use critical_section::Mutex;
use esp_backtrace as _;
use esp_hal::{
    Blocking,
    handler,
    main,
    ram,
    uart::{Config as UartConfig, DataBits, Parity, RxConfig, StopBits, Uart, UartInterrupt},
};

static SERIAL: Mutex<RefCell<Option<Uart<Blocking>>>> = Mutex::new(RefCell::new(None));

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    cfg_if::cfg_if! {
        if #[cfg(feature = "esp32")] {
            let rx_pin = peripherals.GPIO3;
        } else if #[cfg(feature = "esp32c2")] {
            let rx_pin = peripherals.GPIO19;
        } else if #[cfg(feature = "esp32c3")] {
            let rx_pin = peripherals.GPIO20;
        } else if #[cfg(feature = "esp32c6")] {
            let rx_pin = peripherals.GPIO17;
        } else if #[cfg(feature = "esp32h2")] {
            let rx_pin = peripherals.GPIO23;
        } else if #[cfg(any(feature = "esp32s2", feature = "esp32s3"))] {
            let rx_pin = peripherals.GPIO44;
        }
    }

    let uart_config = UartConfig::default()
        .with_baudrate(19200)
        .with_data_bits(DataBits::_8)
        .with_parity(Parity::None)
        .with_stop_bits(StopBits::_1)
        .with_rx(RxConfig::default().with_fifo_full_threshold(1));
    let mut uart = Uart::new(peripherals.UART1, uart_config)
        .expect("Failed to initialize UART")
        .with_rx(rx_pin);

    uart.set_interrupt_handler(handler);

    critical_section::with(|cs| {
        uart.clear_interrupts(UartInterrupt::RxBreakDetected | UartInterrupt::RxFifoFull);
        uart.listen(UartInterrupt::RxBreakDetected | UartInterrupt::RxFifoFull);
        SERIAL.borrow_ref_mut(cs).replace(uart);
    });

    loop {}
}

#[handler]
#[ram]
fn handler() {
    critical_section::with(|cs| {
        let mut serial = SERIAL.borrow_ref_mut(cs);
        let serial = serial.as_mut().unwrap();

        if serial.interrupts().contains(UartInterrupt::RxBreakDetected) {
            esp_println::print!("\nBREAK");
        }
        if serial.interrupts().contains(UartInterrupt::RxFifoFull) {
            let mut byte = [0u8; 1];
            serial.read(&mut byte).unwrap();
            esp_println::print!(" {:02X}", byte[0]);
        }

        serial.clear_interrupts(UartInterrupt::RxBreakDetected | UartInterrupt::RxFifoFull);
    });
}
