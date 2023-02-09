//! This shows how to continously receive data via I2S
//!
//! Pins used
//! BCLK    GPIO12
//! WS      GPIO13
//! DIN     GPIO17
//!
//! Without an additional I2S source device you can connect 3V3 or GND to DIN to
//! read 0 or 0xFF or connect DIN to WS to read two different values
//!
//! You can also inspect the MCLK, BCLK and WS with a logic analyzer

#![no_std]
#![no_main]

use esp32_hal::{
    clock::ClockControl,
    dma::DmaPriority,
    i2s::{DataFormat, I2s, I2s0New, I2sReadDma, NoMclk, PinsBclkWsDin, Standard},
    pdma::Dma,
    peripherals::Peripherals,
    prelude::*,
    timer::TimerGroup,
    Rtc,
    IO,
};
use esp_backtrace as _;
use esp_println::println;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let mut system = peripherals.DPORT.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut wdt = timer_group0.wdt;
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);

    // Disable MWDT and RWDT (Watchdog) flash boot protection
    wdt.disable();
    rtc.rwdt.disable();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let dma = Dma::new(system.dma, &mut system.peripheral_clock_control);
    let dma_channel = dma.i2s0channel;

    let mut tx_descriptors = [0u32; 8 * 3];
    let mut rx_descriptors = [0u32; 8 * 3];

    let i2s = I2s::new(
        peripherals.I2S0,
        NoMclk {},
        Standard::Philips,
        DataFormat::Data16Channel16,
        44100u32.Hz(),
        dma_channel.configure(
            false,
            &mut tx_descriptors,
            &mut rx_descriptors,
            DmaPriority::Priority0,
        ),
        &mut system.peripheral_clock_control,
        &clocks,
    );

    let i2s_rx = i2s.i2s_rx.with_pins(PinsBclkWsDin::new(
        io.pins.gpio12,
        io.pins.gpio13,
        io.pins.gpio17,
    ));

    let buffer = dma_buffer();

    let mut transfer = i2s_rx.read_dma_circular(buffer).unwrap();
    println!("Started transfer");

    loop {
        let avail = transfer.available();

        if avail > 0 {
            let mut rcv = [0u8; 5000];
            transfer.pop(&mut rcv[..avail]).unwrap();
            println!("Received {:x?}...", &rcv[..30]);
        }
    }
}

fn dma_buffer() -> &'static mut [u8; 4092 * 4] {
    static mut BUFFER: [u8; 4092 * 4] = [0u8; 4092 * 4];
    unsafe { &mut BUFFER }
}
