//! I2C Slave Mode Example
//!
//! This example shows how to configure I2C in slave (target) mode.
//! The slave listens for commands from the master:
//! - When the master writes to the slave, the slave prints the received data.
//! - When the master reads from the slave, the slave responds by sending back a message.

//% CHIP_FILTER: i2c_slave_driver_supported

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
    let peripherals = esp_hal::init(Config::default());

    // Log on UART0's default TX pin.
    let log_tx = cfg_select! {
        feature = "esp32c3" => peripherals.GPIO21,
        feature = "esp32c6" => peripherals.GPIO16,
        feature = "esp32c61" => peripherals.GPIO11,
        any(feature = "esp32h2", feature = "esp32h4") => peripherals.GPIO24,
        feature = "esp32p4" => peripherals.GPIO37,
        feature = "esp32s3" => peripherals.GPIO43,
        feature = "esp32s31" => peripherals.GPIO58,
    };
    let log_writer = esp_hal::uart::UartTx::new(peripherals.UART0, Default::default())
        .unwrap()
        .with_tx(log_tx);
    static LOG_WRITER: static_cell::StaticCell<esp_hal::uart::UartTx<'static, esp_hal::Blocking>> =
        static_cell::StaticCell::new();
    embedded_log::init(LOG_WRITER.init(log_writer));

    log::info!("Initializing I2C Slave on I2C0...");

    // Configure SDA and SCL pins
    let sda = peripherals.GPIO2;
    let scl = peripherals.GPIO3;

    // Create a new I2C slave instance on I2C0 with address 0x55
    let mut i2c_slave = I2cSlave::new(peripherals.I2C0, SlaveConfig::new(0x55))
        .unwrap()
        .with_sda(sda)
        .with_scl(scl);

    log::info!("I2C Slave listening on address 0x55...");

    let mut rx_buf = [0u8; 64];

    loop {
        match i2c_slave.listen(&mut rx_buf) {
            Ok(Command::Write(len)) => {
                log::info!("Received {} bytes from master: {:x?}", len, &rx_buf[..len]);
            }
            Ok(Command::Read) => {
                log::info!("Master requested a read. Responding...");
                let response = b"Hello from I2C Slave!";
                match i2c_slave.respond_to_read(response) {
                    Ok(_) => log::info!("Responded successfully"),
                    Err(e) => log::info!("Failed to respond to read: {:?}", e),
                }
            }
            Err(e) => {
                log::info!("I2C Slave error: {:?}", e);
            }
        }
    }
}
