//! I2C Slave Mode Example
//!
//! This example shows how to configure I2C in slave (target) mode.
//! The slave listens for commands from the master:
//! - When the master writes to the slave, the slave prints the received data.
//! - When the master reads from the slave, the slave responds by sending back a message.

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    Config,
    i2c::slave::{Command, Config as SlaveConfig, I2cSlave},
    main,
};

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(Config::default());

    esp_println::println!("Initializing I2C Slave on I2C0...");

    // Configure SDA and SCL pins
    let sda = peripherals.GPIO2;
    let scl = peripherals.GPIO3;

    // Create a new I2C slave instance on I2C0 with address 0x55
    let mut i2c_slave = I2cSlave::new(peripherals.I2C0, SlaveConfig::new(0x55))
        .unwrap()
        .with_sda(sda)
        .with_scl(scl);

    esp_println::println!("I2C Slave listening on address 0x55...");

    let mut rx_buf = [0u8; 64];

    loop {
        match i2c_slave.listen(&mut rx_buf) {
            Ok(Command::Write(len)) => {
                esp_println::println!("Received {} bytes from master: {:x?}", len, &rx_buf[..len]);
            }
            Ok(Command::Read) => {
                esp_println::println!("Master requested a read. Responding...");
                let response = b"Hello from I2C Slave!";
                match i2c_slave.respond_to_read(response) {
                    Ok(_) => esp_println::println!("Responded successfully"),
                    Err(e) => esp_println::println!("Failed to respond to read: {:?}", e),
                }
            }
            Err(e) => {
                esp_println::println!("I2C Slave error: {:?}", e);
            }
        }
    }
}
