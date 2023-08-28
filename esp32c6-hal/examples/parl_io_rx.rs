//! This shows using Parallel IO to input 4 bit parallel data at 1MHz clock
//! rate.
//!
//! Uses GPIO 1, 2, 3 and 4 as the data pins.

#![no_std]
#![no_main]

use esp32c6_hal::{
    clock::ClockControl,
    dma::DmaPriority,
    gdma::Gdma,
    gpio::IO,
    parl_io::{BitPackOrder, NoClkPin, ParlIoRxOnly, RxFourBits},
    peripherals::Peripherals,
    prelude::*,
    timer::TimerGroup,
    Delay,
    Rtc,
};
use esp_backtrace as _;
use esp_println::println;

#[entry]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();

    let peripherals = Peripherals::take();
    let mut system = peripherals.PCR.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    // Disable the watchdog timers.
    let mut rtc = Rtc::new(peripherals.LP_CLKRST);
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

    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let mut tx_descriptors = [0u32; 8 * 3];
    let mut rx_descriptors = [0u32; 8 * 3];

    let dma = Gdma::new(peripherals.DMA, &mut system.peripheral_clock_control);
    let dma_channel = dma.channel0;

    let rx_pins = RxFourBits::new(io.pins.gpio1, io.pins.gpio2, io.pins.gpio3, io.pins.gpio4);

    let parl_io = ParlIoRxOnly::new(
        peripherals.PARL_IO,
        dma_channel.configure(
            false,
            &mut tx_descriptors,
            &mut rx_descriptors,
            DmaPriority::Priority0,
        ),
        1u32.MHz(),
        &mut system.peripheral_clock_control,
        &clocks,
    )
    .unwrap();

    let mut parl_io_rx = parl_io
        .rx
        .with_config(rx_pins, NoClkPin, BitPackOrder::Msb, Some(0xfff))
        .unwrap();

    let mut buffer = dma_buffer();
    buffer.fill(0u8);

    let mut delay = Delay::new(&clocks);

    loop {
        let transfer = parl_io_rx.read_dma(buffer).unwrap();

        // the buffer and driver is moved into the transfer and we can get it back via
        // `wait`
        (buffer, parl_io_rx) = transfer.wait().unwrap();
        println!("Received: {:02x?} ...", &buffer[..30]);

        delay.delay_ms(500u32);
    }
}

fn dma_buffer() -> &'static mut [u8; 4092 * 4] {
    static mut BUFFER: [u8; 4092 * 4] = [0u8; 4092 * 4];
    unsafe { &mut BUFFER }
}
