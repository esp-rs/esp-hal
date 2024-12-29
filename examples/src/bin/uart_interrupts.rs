//! Example of responding to UART interrupts.
//!
//! The following wiring is assumed:
//! - TX => GPIO17
//! - RX => GPIO16

//% CHIPS: esp32

#![no_std]
#![no_main]

use core::cell::RefCell;

use critical_section::Mutex;
use esp_backtrace as _;
use esp_hal::{
    entry,
    interrupt::InterruptConfigurable,
    macros::{handler, ram},
    uart::{Config as UartConfig, DataBits, StopBits, Uart, UartInterrupt},
    Blocking,
};

static SERIAL: Mutex<RefCell<Option<Uart<Blocking>>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let uart_config = UartConfig::default()
        .baudrate(19200)
        .data_bits(DataBits::DataBits8)
        .parity_none()
        .stop_bits(StopBits::Stop1)
        .rx_fifo_full_threshold(1); // when more data than this, an interrupt will be triggered
    let mut uart = Uart::new(
        peripherals.UART1,
        uart_config,
        peripherals.GPIO16, // RX
        peripherals.GPIO17, // TX
    )
    .expect("Failed to initialize UART");

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
            esp_println::println!("Break detected");

            // Clear the RX FIFO
            while serial.read_byte().is_ok() {}
        }
        if serial.interrupts().contains(UartInterrupt::RxFifoFull) {
            // esp_println::println!("Byte received: {:?}", serial.read_byte());
        }

        serial.clear_interrupts(UartInterrupt::RxBreakDetected | UartInterrupt::RxFifoFull);
    });
}
