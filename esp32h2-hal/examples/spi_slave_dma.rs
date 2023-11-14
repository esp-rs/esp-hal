//! SPI slave loopback test using DMA
//!
//! Following pins are used for the slave:
//! SCLK    GPIO6
//! MISO    GPIO2
//! MOSI    GPIO7
//! CS      GPIO10
//!
//! Following pins are used for the (bitbang) master:
//! SCLK    GPIO5
//! MISO    GPIO1
//! MOSI    GPIO8
//! CS      GPIO9
//!
//! Depending on your target and the board you are using you have to change the
//! pins.
//!
//! This example transfers data via SPI.
//! Connect corresponding master and slave pins to see the outgoing data is read
//! as incoming data. The master-side pins are chosen to make these connections
//! easy for the barebones ESP32C3 chip; all are immediate neighbors of the
//! slave-side pins except SCLK. SCLK is between MOSI and VDD3P3_RTC on the
//! barebones ESP32C3, so no immediate neighbor is available.

#![no_std]
#![no_main]

use esp32h2_hal::{
    clock::ClockControl,
    dma::DmaPriority,
    dma_buffers,
    gdma::Gdma,
    gpio::IO,
    peripherals::Peripherals,
    prelude::*,
    spi::{
        slave::{prelude::*, Spi},
        SpiMode,
    },
    timer::TimerGroup,
    Delay,
    Rtc,
};
use esp_backtrace as _;
use esp_println::println;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    // Disable the watchdog timers. For the ESP32-C6, this includes the Super WDT,
    // the RTC WDT, and the TIMG WDTs.
    let mut rtc = Rtc::new(peripherals.LP_CLKRST);
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(peripherals.TIMG1, &clocks);
    let mut wdt1 = timer_group1.wdt;

    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let slave_sclk = io.pins.gpio4;
    let mut master_sclk = io.pins.gpio5.into_push_pull_output();
    let slave_miso = io.pins.gpio2;
    let master_miso = io.pins.gpio1.into_floating_input();
    let slave_mosi = io.pins.gpio12;
    let mut master_mosi = io.pins.gpio11.into_push_pull_output();
    let slave_cs = io.pins.gpio10;
    let mut master_cs = io.pins.gpio9.into_push_pull_output();
    master_cs.set_high().unwrap();
    master_sclk.set_low().unwrap();
    master_mosi.set_low().unwrap();

    let dma = Gdma::new(peripherals.DMA);
    let dma_channel = dma.channel0;

    let (tx_buffer, mut tx_descriptors, rx_buffer, mut rx_descriptors) = dma_buffers!(32000, 32000);

    let mut spi = Spi::new(
        peripherals.SPI2,
        slave_sclk,
        slave_mosi,
        slave_miso,
        slave_cs,
        SpiMode::Mode0,
    )
    .with_dma(dma_channel.configure(
        false,
        &mut tx_descriptors,
        &mut rx_descriptors,
        DmaPriority::Priority0,
    ));

    let mut delay = Delay::new(&clocks);

    // DMA buffer require a static life-time
    let master_send = &mut [0u8; 32000];
    let master_receive = &mut [0u8; 32000];
    let mut slave_send = tx_buffer;
    let mut slave_receive = rx_buffer;
    let mut i = 0;

    for (i, v) in master_send.iter_mut().enumerate() {
        *v = (i % 255) as u8;
    }
    for (i, v) in slave_send.iter_mut().enumerate() {
        *v = (254 - (i % 255)) as u8;
    }

    loop {
        master_send[0] = i;
        master_send[master_send.len() - 1] = i;
        slave_send[0] = i;
        slave_send[slave_send.len() - 1] = i;
        slave_receive.fill(0xff);
        i = i.wrapping_add(1);

        let transfer = spi.dma_transfer(slave_send, slave_receive).unwrap();
        // Bit-bang out the contents of master_send and read into master_receive
        // as quickly as manageable. MSB first. Mode 0, so sampled on the rising
        // edge and set on the falling edge.
        master_cs.set_low().unwrap();
        for (j, v) in master_send.iter().enumerate() {
            let mut b = *v;
            let mut rb = 0u8;
            for _ in 0..8 {
                if b & 128 != 0 {
                    master_mosi.set_high().unwrap();
                } else {
                    master_mosi.set_low().unwrap();
                }
                master_sclk.set_low().unwrap();
                b <<= 1;
                rb <<= 1;
                // NB: adding about 24 NOPs here makes the clock's duty cycle
                // run at about 50% ... but we don't strictly need the delay,
                // either.
                master_sclk.set_high().unwrap();
                if master_miso.is_high().unwrap() {
                    rb |= 1;
                }
            }
            master_receive[j] = rb;
        }
        master_cs.set_high().unwrap();
        master_sclk.set_low().unwrap();
        // the buffers and spi is moved into the transfer and we can get it back via
        // `wait`
        (slave_receive, slave_send, spi) = transfer.wait().unwrap();
        println!(
            "slave got {:x?} .. {:x?}, master got {:x?} .. {:x?}",
            &slave_receive[..10],
            &slave_receive[slave_receive.len() - 10..],
            &master_receive[..10],
            &master_receive[master_receive.len() - 10..]
        );

        delay.delay_ms(250u32);
    }
}
