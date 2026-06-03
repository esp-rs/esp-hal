//! Example of responding to UART interrupts and sending break conditions.
//!
//! This example can be flashed on two devices to demonstrate UART communication
//! with break detection. Connect the devices as follows:
//! - Device 1 TX => Device 2 RX
//! - Device 2 TX => Device 1 RX
//! - Connect GND between devices
//!
//! Default TX/RX pins:
//! - ESP32:     GPIO1  / GPIO3
//! - ESP32-C2:  GPIO20 / GPIO19
//! - ESP32-C3:  GPIO21 / GPIO20
//! - ESP32-C5:  GPIO10 / GPIO9
//! - ESP32-C6:  GPIO16 / GPIO17
//! - ESP32-C61: GPIO3  / GPIO2
//! - ESP32-H2:  GPIO24 / GPIO23
//! - ESP32-P4:  GPIO6  / GPIO5
//! - ESP32-S2:  GPIO43 / GPIO44
//! - ESP32-S3:  GPIO43 / GPIO44
//!
//! Each device will:
//! - Send a counter value every second
//! - Send a break condition every 3 seconds
//! - Print received data and break conditions via esp-println

//% CHIP_FILTER: uart_driver_supported

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
    time::Instant,
    uart::{Config as UartConfig, DataBits, Parity, RxConfig, StopBits, Uart, UartInterrupt},
};

esp_bootloader_esp_idf::esp_app_desc!();

static SERIAL: Mutex<RefCell<Option<Uart<Blocking>>>> = Mutex::new(RefCell::new(None));

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let (tx_pin, rx_pin) = cfg_select! {
        feature = "esp32" => (peripherals.GPIO1, peripherals.GPIO3),
        feature = "esp32c2" => (peripherals.GPIO20, peripherals.GPIO19),
        feature = "esp32c3" => (peripherals.GPIO21, peripherals.GPIO20),
        feature = "esp32c5" => (peripherals.GPIO10, peripherals.GPIO9),
        feature = "esp32c6" => (peripherals.GPIO16, peripherals.GPIO17),
        feature = "esp32c61" => (peripherals.GPIO3, peripherals.GPIO2),
        feature = "esp32h2" => (peripherals.GPIO24, peripherals.GPIO23),
        feature = "esp32p4" => (peripherals.GPIO6, peripherals.GPIO5),
        any(feature = "esp32s2", feature = "esp32s3") => (peripherals.GPIO43, peripherals.GPIO44),
    };

    let uart_config = UartConfig::default()
        .with_baudrate(19200)
        .with_data_bits(DataBits::_8)
        .with_parity(Parity::None)
        .with_stop_bits(StopBits::_1)
        .with_rx(RxConfig::default().with_fifo_full_threshold(1));
    let mut uart = Uart::new(peripherals.UART1, uart_config)
        .expect("Failed to initialize UART")
        .with_tx(tx_pin)
        .with_rx(rx_pin);

    uart.set_interrupt_handler(handler);

    critical_section::with(|cs| {
        uart.clear_interrupts(UartInterrupt::RxBreakDetected | UartInterrupt::RxFifoFull);
        uart.listen(UartInterrupt::RxBreakDetected | UartInterrupt::RxFifoFull);
        SERIAL.borrow_ref_mut(cs).replace(uart);
    });

    esp_println::println!("UART interrupt example with break conditions");
    esp_println::println!("Sending data every second, break every 3 seconds");

    let mut last_send = Instant::now();
    let mut last_break = Instant::now();

    loop {
        let now = Instant::now();

        if (now - last_send).as_millis() >= 1000 {
            last_send = now;

            critical_section::with(|cs| {
                if let Some(uart) = SERIAL.borrow_ref_mut(cs).as_mut() {
                    let msg = [
                        0x55, 0x22, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x46,
                    ];
                    let _ = uart.write(&msg);
                    let _ = uart.flush();
                }
            });
        }

        if (now - last_break).as_millis() >= 3000 {
            last_break = now;

            critical_section::with(|cs| {
                if let Some(uart) = SERIAL.borrow_ref_mut(cs).as_mut() {
                    esp_println::println!("Sending break condition...");
                    uart.send_break(14);
                }
            });
        }
    }
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
