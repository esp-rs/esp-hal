//! SDIO SPI mode master driver example.

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    Blocking,
    sdio::{
        command::{Cmd0, Cmd5, Cmd52, Cmd53, FunctionNumber, RwFlag},
        response::{
            R1,
            spi::{R4, R5},
        },
    },
    spi::{Mode, master as spi},
    time::Rate,
};
use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

// Represents the byte length of a SDIO block.
const BLOCK_LEN: usize = 512;

struct Context {
    spi: spi::Spi<'static, Blocking>,
}

impl Context {
    /// Creates a new context for the SDIO SPI master controller.
    pub fn new() -> Self {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let config = spi::Config::default()
            .with_mode(Mode::_2)
            .with_frequency(Rate::from_mhz(40));

        cfg_if::cfg_if! {
            if #[cfg(feature = "esp32")] {
                // Create SPI master for mock SDIO host
                // HSPI config
                let spi = spi::Spi::new(
                    peripherals.SPI2,
                    config,
                )
                .unwrap()
                .with_sck(peripherals.GPIO14)
                .with_mosi(peripherals.GPIO13)
                .with_miso(peripherals.GPIO12)
                .with_cs(peripherals.GPIO15);

                // Create SPI master for mock SDIO host
                // VSPI config
                //let spi = spi::Spi::new(
                //    peripherals.SPI3,
                //    spi::Config::default().with_frequency(Rate::from_mhz(40)),
                //)
                //.unwrap()
                //.with_sck(peripherals.GPIO18)
                //.with_mosi(peripherals.GPIO23)
                //.with_miso(peripherals.GPIO19)
                //.with_cs(peripherals.GPIO5);
            } else if #[cfg(feature = "esp32c6")] {
                // Create SPI master for mock SDIO host
                let spi = spi::Spi::new(
                    peripherals.SPI2,
                    config,
                )
                .unwrap()
                .with_sck(peripherals.GPIO6)
                .with_mosi(peripherals.GPIO7)
                .with_miso(peripherals.GPIO2)
                .with_cs(peripherals.GPIO16);
            } else {
                panic!("unsupported platform");
            }
        }

        Self { spi }
    }
}

#[esp_hal::main]
fn main() -> ! {
    let mut ctx = Context::new();

    ctx.spi
        .write(&[0u8; 10])
        .expect("error sending SPI init cycles");

    let mut poll = true;

    // continually send CMD0 until an IDLE R1 response
    while poll {
        let mut packet = Cmd0::new().into_bytes();
        ctx.spi
            .transfer(&mut packet)
            .expect("error writing SDIO from host");

        // expect R1 response from SDIO device
        match R1::try_from(packet.as_ref()) {
            Ok(r1) if r1.idle() && !r1.is_err() => {
                println!("response: {r1}");
                poll = false;
            }
            _ => (),
        }
    }

    println!("received R1 IDLE response ...");
    poll = true;

    while poll {
        let mut packet = Cmd5::new().into_bytes();

        // expect R4 response from SDIO device
        ctx.spi
            .transfer(&mut packet)
            .expect("error transfer CMD5 from SPI host");

        match R4::try_from(packet.as_ref()) {
            Ok(r4) if r4.ready() && !r4.is_err() => {
                poll = false;
                println!("response: {r4:x?}");
            }
            Ok(r4) if r4.is_err() => {
                println!("error response: {}", r4.modified_r1());
            }
            _ => (),
        }
    }

    println!("received R4 READY response ...");
    poll = true;

    let mut packet = [0u8; Cmd53::LEN + BLOCK_LEN];

    while poll {
        let mut cmd = Cmd53::new();

        cmd.set_function_number(FunctionNumber::Io2);
        cmd.set_read_write_flag(RwFlag::ReadOnly);
        // Fixed-address transfer
        cmd.set_register_address(0x0);
        cmd.set_crc();

        packet[..Cmd53::LEN].copy_from_slice(cmd.into_bytes().as_slice());

        ctx.spi
            .transfer(&mut packet)
            .expect("error transfer CMD53 from SPI host");

        match R5::try_from(packet.as_ref()) {
            Ok(r5) if r5.idle() && !r5.is_err() => {
                poll = false;

                println!("response: {r5}");

                let data = &packet[R5::LEN..];
                println!("data: {data:x?}");
            }
            Ok(r5) if r5.is_err() => {
                println!("error response: {r5}");
            }
            _ => (),
        }
    }

    loop {
        core::hint::spin_loop()
    }
}
