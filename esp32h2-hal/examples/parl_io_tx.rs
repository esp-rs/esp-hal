//! This shows using Parallel IO to output 4 bit parallel data at 1MHz clock
//! rate.
//!
//! Uses GPIO 1, 2, 3 and 4 as the data pins.
//! GPIO 5 as the "valid pin" (driven high during an active transfer) and GPIO
//! 10 as the clock signal output.
//!
//! You can use a logic analyzer to see how the pins are used.

#![no_std]
#![no_main]

use esp32h2_hal::{
    clock::ClockControl,
    dma::DmaPriority,
    gdma::Gdma,
    gpio::IO,
    parl_io::{
        BitPackOrder,
        ClkOutPin,
        ParlIoTxOnly,
        SampleEdge,
        TxFourBits,
        TxPinConfigWithValidPin,
    },
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

    let tx_pins = TxFourBits::new(io.pins.gpio1, io.pins.gpio2, io.pins.gpio3, io.pins.gpio4);

    let pin_conf = TxPinConfigWithValidPin::new(tx_pins, io.pins.gpio5);

    let parl_io = ParlIoTxOnly::new(
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

    let clock_pin = ClkOutPin::new(io.pins.gpio10);

    let mut parl_io_tx = parl_io.tx.with_config(
        pin_conf,
        clock_pin,
        0,
        SampleEdge::Normal,
        BitPackOrder::Msb,
    );

    let mut buffer = dma_buffer();
    for i in 0..buffer.len() {
        buffer[i] = (i % 255) as u8;
    }

    let mut delay = Delay::new(&clocks);

    loop {
        let transfer = parl_io_tx.write_dma(buffer).unwrap();

        // the buffer and driver is moved into the transfer and we can get it back via
        // `wait`
        (buffer, parl_io_tx) = transfer.wait().unwrap();
        println!("Transferred {} bytes", buffer.len());

        delay.delay_ms(500u32);
    }
}

fn dma_buffer() -> &'static mut [u8; 4092 * 4] {
    static mut BUFFER: [u8; 4092 * 4] = [0u8; 4092 * 4];
    unsafe { &mut BUFFER }
}
