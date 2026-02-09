//% CHIPS: esp32c5 esp32c6

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    main,
};
use esp_println::println;
use esp_hal::uart::{Config, Uart};

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    
    let peripherals = esp_hal::init(esp_hal::Config::default());
    
    println!("init");
    let mut uart = Uart::new(peripherals.UART0, Config::default()).unwrap()
        .with_rx(peripherals.GPIO6)
        .with_tx(peripherals.GPIO7);

    println!("new");
    const MESSAGE: &[u8] = b"Hello, world!";

    uart.write(&MESSAGE).unwrap();
    println!("write");
    uart.flush().unwrap();
    println!("flush");
    
    let mut buf = [0u8; MESSAGE.len()];
    uart.read(&mut buf[..]).unwrap();
    println!("read");

    println!("{:?}", buf);

    loop {
    }
}
