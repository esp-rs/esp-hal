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
    handler,
    interrupt::InterruptConfigurable,
    main,
    ram,
    uart::{Config as UartConfig, DataBits, Parity, RxConfig, StopBits, Uart, UartInterrupt},
    Blocking,
};

static SERIAL: Mutex<RefCell<Option<Uart<Blocking>>>> = Mutex::new(RefCell::new(None));

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let uart_config = UartConfig::default()
        .with_baudrate(19200)
        .with_data_bits(DataBits::_8)
        .with_parity(Parity::None)
        .with_stop_bits(StopBits::_1)
        .with_rx(RxConfig::default().with_fifo_full_threshold(1));
    let mut uart = Uart::new(peripherals.UART1, uart_config)
        .expect("Failed to initialize UART")
        .with_rx(peripherals.GPIO16)
        .with_tx(peripherals.GPIO17);

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
            serial.read_bytes(&mut byte).unwrap();
            esp_println::print!(" {:02X}", byte[0]);
        }

        serial.clear_interrupts(UartInterrupt::RxBreakDetected | UartInterrupt::RxFifoFull);
    });
}
