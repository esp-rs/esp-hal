//! SPI read manufacturer id from flash chip
//!
//! Folowing pins are used:
//! SCLK            GPIO19
//! MISO/IO0        GPIO18
//! MOSI/IO1        GPIO5
//! IO2             GPIO17
//! IO3             GPIO16
//! CS              GPIO4
//!
//! Depending on your target and the board you are using you have to change the
//! pins.
//!
//! Connect a flash chip (GD25Q64C was used) and make sure QE in the status
//! register is set.

#![no_std]
#![no_main]

use esp32_hal::{
    clock::ClockControl,
    gpio::IO,
    peripherals::Peripherals,
    prelude::*,
    spi::{Address, Command, HalfDuplexReadWrite, Spi, SpiDataMode, SpiMode},
    timer::TimerGroup,
    Delay,
    Rtc,
};
use esp_backtrace as _;
use esp_println::println;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let mut system = peripherals.DPORT.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    // Disable the watchdog timers. For the ESP32, this includes
    // the RTC WDT, and the TIMG WDTs.
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(
        peripherals.TIMG0,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(
        peripherals.TIMG1,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt1 = timer_group1.wdt;

    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let sclk = io.pins.gpio19;
    let miso = io.pins.gpio18;
    let mosi = io.pins.gpio5;
    let sio2 = io.pins.gpio17;
    let sio3 = io.pins.gpio16;
    let cs = io.pins.gpio4;

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
        &mut system.peripheral_clock_control,
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
