//! SPI write and read a flash chip
//!
//! Folowing pins are used:
//! SCLK            GPIO4
//! MISOI/IO0       GPIO5
//! MOSI/IO1        GPIO6
//! IO2             GPIO7
//! IO3             GPIO8
//! CS              GPIO9
//!
//! Depending on your target and the board you are using you have to change the
//! pins.
//!
//! Connect a flash chip (GD25Q64C was used) and make sure QE in the status
//! register is set.

#![no_std]
#![no_main]

use esp32c2_hal::{
    clock::ClockControl,
    dma::DmaPriority,
    gdma::Gdma,
    gpio::IO,
    peripherals::Peripherals,
    prelude::*,
    spi::{Address, Command, Spi, SpiDataMode, SpiMode},
    timer::TimerGroup,
    Delay,
    Rtc,
};
use esp_backtrace as _;
use esp_println::{print, println};

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let mut system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    // Disable the watchdog timers. For the ESP32-C2, this includes the Super WDT,
    // the RTC WDT, and the TIMG WDTs.
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(
        peripherals.TIMG0,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt0 = timer_group0.wdt;

    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.disable();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let sclk = io.pins.gpio4;
    let miso = io.pins.gpio5;
    let mosi = io.pins.gpio6;
    let sio2 = io.pins.gpio7;
    let sio3 = io.pins.gpio8;
    let cs = io.pins.gpio9;

    let dma = Gdma::new(peripherals.DMA, &mut system.peripheral_clock_control);
    let dma_channel = dma.channel0;

    let mut descriptors = [0u32; 8 * 3];
    let mut rx_descriptors = [0u32; 8 * 3];

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
    )
    .with_dma(dma_channel.configure(
        false,
        &mut descriptors,
        &mut rx_descriptors,
        DmaPriority::Priority0,
    ));

    let mut delay = Delay::new(&clocks);

    // DMA buffer require a static life-time
    let send = send_buffer();
    let mut receive = receive_buffer();
    let mut zero_buf = zero_buffer();

    // write enable
    let transfer = spi
        .write(
            SpiDataMode::Single,
            Command::Command8(0x06, SpiDataMode::Single),
            Address::None,
            0,
            zero_buf,
        )
        .unwrap();
    (zero_buf, spi) = transfer.wait();
    delay.delay_ms(250u32);

    // erase sector
    let transfer = spi
        .write(
            SpiDataMode::Single,
            Command::Command8(0x20, SpiDataMode::Single),
            Address::Address24(0x000000, SpiDataMode::Single),
            0,
            zero_buf,
        )
        .unwrap();
    (zero_buf, spi) = transfer.wait();
    delay.delay_ms(250u32);

    // write enable
    let transfer = spi
        .write(
            SpiDataMode::Single,
            Command::Command8(0x06, SpiDataMode::Single),
            Address::None,
            0,
            zero_buf,
        )
        .unwrap();
    (_, spi) = transfer.wait();
    delay.delay_ms(250u32);

    // write data / program page
    send.fill(b'!');
    send[0..][..5].copy_from_slice(&b"Hello"[..]);
    let transfer = spi
        .write(
            SpiDataMode::Quad,
            Command::Command8(0x32, SpiDataMode::Single),
            Address::Address24(0x000000, SpiDataMode::Single),
            0,
            send,
        )
        .unwrap();
    (_, spi) = transfer.wait();
    delay.delay_ms(250u32);

    loop {
        // quad fast read
        let transfer = spi
            .read(
                SpiDataMode::Quad,
                Command::Command8(0xeb, SpiDataMode::Single),
                Address::Address32(0x000000 << 8, SpiDataMode::Quad),
                4,
                receive,
            )
            .unwrap();

        // here we could do something else while DMA transfer is in progress
        // the buffers and spi is moved into the transfer and we can get it back via
        // `wait`
        (receive, spi) = transfer.wait();

        println!("{:x?}", &receive);
        for b in &mut receive.iter() {
            if *b >= 32 && *b <= 127 {
                print!("{}", *b as char);
            } else {
                print!(".");
            }
        }
        println!();

        delay.delay_ms(250u32);
    }
}

fn zero_buffer() -> &'static mut [u8; 0] {
    static mut BUFFER: [u8; 0] = [0u8; 0];
    unsafe { &mut BUFFER }
}

fn send_buffer() -> &'static mut [u8; 256] {
    static mut BUFFER: [u8; 256] = [0u8; 256];
    unsafe { &mut BUFFER }
}

fn receive_buffer() -> &'static mut [u8; 320] {
    static mut BUFFER: [u8; 320] = [0u8; 320];
    unsafe { &mut BUFFER }
}
