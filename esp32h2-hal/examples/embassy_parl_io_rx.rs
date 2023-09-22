//! This shows using Parallel IO to input 4 bit parallel data at 1MHz clock
//! rate.
//!
//! Uses GPIO 1, 2, 3 and 4 as the data pins.

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_executor::Executor;
use embassy_time::{Duration, Timer};
use esp32h2_hal::{
    clock::ClockControl,
    dma::DmaPriority,
    embassy,
    gdma::{self, Gdma},
    gpio::{GpioPin, Unknown, IO},
    interrupt,
    parl_io::{BitPackOrder, NoClkPin, ParlIoRx, ParlIoRxOnly, RxFourBits},
    peripherals,
    peripherals::Peripherals,
    prelude::*,
};
use esp_backtrace as _;
use esp_println::println;
use static_cell::make_static;

#[embassy_executor::task]
async fn parl_io_task(
    mut parl_io_rx: ParlIoRx<
        'static,
        gdma::Channel0,
        RxFourBits<
            'static,
            GpioPin<Unknown, 1>,
            GpioPin<Unknown, 2>,
            GpioPin<Unknown, 3>,
            GpioPin<Unknown, 4>,
        >,
        NoClkPin,
    >,
) {
    let buffer = dma_buffer();
    loop {
        parl_io_rx.read_dma_async(buffer).await.unwrap();
        println!("Received: {:02x?} ...", &buffer[..30]);

        Timer::after(Duration::from_millis(500)).await;
    }
}

#[entry]
fn main() -> ! {
    esp_println::println!("Init!");
    let peripherals = Peripherals::take();
    let mut system = peripherals.PCR.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    #[cfg(feature = "embassy-time-systick")]
    embassy::init(
        &clocks,
        esp32h2_hal::systimer::SystemTimer::new(peripherals.SYSTIMER),
    );

    #[cfg(feature = "embassy-time-timg0")]
    {
        let timer_group0 = esp32h2_hal::timer::TimerGroup::new(
            peripherals.TIMG0,
            &clocks,
            &mut system.peripheral_clock_control,
        );
        embassy::init(&clocks, timer_group0.timer0);
    }

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let tx_descriptors = make_static!([0u32; 8 * 3]);
    let rx_descriptors = make_static!([0u32; 8 * 3]);

    let dma = Gdma::new(peripherals.DMA, &mut system.peripheral_clock_control);
    let dma_channel = dma.channel0;

    let rx_pins = RxFourBits::new(io.pins.gpio1, io.pins.gpio2, io.pins.gpio3, io.pins.gpio4);

    let parl_io = ParlIoRxOnly::new(
        peripherals.PARL_IO,
        dma_channel.configure(
            false,
            tx_descriptors,
            rx_descriptors,
            DmaPriority::Priority0,
        ),
        1u32.MHz(),
        &mut system.peripheral_clock_control,
        &clocks,
    )
    .unwrap();

    let parl_io_rx = parl_io
        .rx
        .with_config(rx_pins, NoClkPin, BitPackOrder::Msb, Some(0xfff))
        .unwrap();

    // you need to manually enable the DMA channel's interrupt!
    interrupt::enable(
        peripherals::Interrupt::DMA_IN_CH0,
        interrupt::Priority::Priority1,
    )
    .unwrap();

    let executor = make_static!(Executor::new());
    executor.run(|spawner| {
        spawner.spawn(parl_io_task(parl_io_rx)).ok();
    });
}

fn dma_buffer() -> &'static mut [u8; 4092 * 4] {
    static mut BUFFER: [u8; 4092 * 4] = [0u8; 4092 * 4];
    unsafe { &mut BUFFER }
}
