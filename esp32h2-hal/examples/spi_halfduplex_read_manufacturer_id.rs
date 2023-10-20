//! SPI read manufacturer id from flash chip
//!
//! Folowing pins are used:
//! SCLK            GPIO1
//! MISOI/IO0       GPIO2
//! MOSI/IO1        GPIO3
//! IO2             GPIO4
//! IO3             GPIO5
//! CS              GPIO11
//!
//! Depending on your target and the board you are using you have to change the
//! pins.
//!
//! Connect a flash chip (GD25Q64C was used) and make sure QE in the status
//! register is set.

#![no_std]
#![no_main]

use esp32h2_hal::{
    clock::ClockControl,
    gpio::IO,
    peripherals::Peripherals,
    prelude::*,
    spi::{
        master::{prelude::*, Address, Command, HalfDuplexReadWrite, Spi},
        SpiDataMode,
        SpiMode,
    },
    Delay,
};
use esp_backtrace as _;
use esp_println::println;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let sclk = io.pins.gpio1;
    let miso = io.pins.gpio2;
    let mosi = io.pins.gpio3;
    let sio2 = io.pins.gpio4;
    let sio3 = io.pins.gpio5;
    let cs = io.pins.gpio11;

    let mut spi = Spi::new_half_duplex(
        peripherals.SPI2,
        Some(sclk),
        Some(mosi),
        Some(miso),
        Some(sio2),
        Some(sio3),
        Some(cs),
        100u32.kHz(),
        SpiMode::Mode0,
        &clocks,
    );

    let mut delay = Delay::new(&clocks);

    loop {
        // READ MANUFACTURER ID FROM FLASH CHIP
        let mut data = [0u8; 2];
        spi.read(
            SpiDataMode::Single,
            Command::Command8(0x90, SpiDataMode::Single),
            Address::Address24(0x000000, SpiDataMode::Single),
            0,
            &mut data,
        )
        .unwrap();
        println!("Single {:x?}", data);
        delay.delay_ms(250u32);

        // READ MANUFACTURER ID FROM FLASH CHIP
        let mut data = [0u8; 2];
        spi.read(
            SpiDataMode::Dual,
            Command::Command8(0x92, SpiDataMode::Single),
            Address::Address32(0x000000_00, SpiDataMode::Dual),
            0,
            &mut data,
        )
        .unwrap();
        println!("Dual {:x?}", data);
        delay.delay_ms(250u32);

        // READ MANUFACTURER ID FROM FLASH CHIP
        let mut data = [0u8; 2];
        spi.read(
            SpiDataMode::Quad,
            Command::Command8(0x94, SpiDataMode::Single),
            Address::Address32(0x000000_00, SpiDataMode::Quad),
            4,
            &mut data,
        )
        .unwrap();
        println!("Quad {:x?}", data);
        delay.delay_ms(1500u32);
    }
}
