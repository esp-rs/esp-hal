//! Demonstrates decoding pulse sequences with RMT
//! This uses the boot button as input - press the button a couple of
//! times to generate a pulse sequence and then wait for the idle timeout to see
//! the recorded pulse sequence

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_executor::Executor;
use esp32c6_hal::{
    clock::ClockControl,
    embassy::{self},
    peripherals::Peripherals,
    prelude::*,
    rmt::{asynch::RxChannelAsync, Channel2, PulseCode, RxChannelConfig, RxChannelCreator},
    Rmt,
    IO,
};
use esp_backtrace as _;
use esp_println::{print, println};
use static_cell::make_static;

const WIDTH: usize = 80;

#[embassy_executor::task]
async fn rmt_task(mut channel: Channel2<2>) {
    let mut data = [PulseCode {
        level1: true,
        length1: 1,
        level2: false,
        length2: 1,
    }; 48];

    loop {
        println!("receive");
        channel.receive(&mut data).await.unwrap();
        println!("received");
        let mut total = 0usize;
        for entry in &data[..data.len()] {
            if entry.length1 == 0 {
                break;
            }
            total += entry.length1 as usize;

            if entry.length2 == 0 {
                break;
            }
            total += entry.length2 as usize;
        }

        for entry in &data[..data.len()] {
            if entry.length1 == 0 {
                break;
            }

            let count = WIDTH / (total / entry.length1 as usize);
            let c = if entry.level1 { '-' } else { '_' };
            for _ in 0..count + 1 {
                print!("{}", c);
            }

            if entry.length2 == 0 {
                break;
            }

            let count = WIDTH / (total / entry.length2 as usize);
            let c = if entry.level2 { '-' } else { '_' };
            for _ in 0..count + 1 {
                print!("{}", c);
            }
        }

        println!();
    }
}

#[entry]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    println!("Init!");
    let peripherals = Peripherals::take();
    let system = peripherals.PCR.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
    let mut clock_control = system.peripheral_clock_control;

    #[cfg(feature = "embassy-time-systick")]
    embassy::init(
        &clocks,
        esp32c6_hal::systimer::SystemTimer::new(peripherals.SYSTIMER),
    );

    #[cfg(feature = "embassy-time-timg0")]
    embassy::init(
        &clocks,
        esp32c6_hal::timer::TimerGroup::new(peripherals.TIMG0, &clocks, &mut clock_control).timer0,
    );

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let rmt = Rmt::new(peripherals.RMT, 8u32.MHz(), &mut clock_control, &clocks).unwrap();

    let channel = rmt
        .channel2
        .configure(
            io.pins.gpio9,
            RxChannelConfig {
                clk_divider: 255,
                idle_threshold: 10000,
                ..RxChannelConfig::default()
            },
        )
        .unwrap();

    // you have to enable the interrupt for async to work
    esp32c6_hal::interrupt::enable(
        esp32c6_hal::peripherals::Interrupt::RMT,
        esp32c6_hal::interrupt::Priority::Priority1,
    )
    .unwrap();

    let executor = make_static!(Executor::new());
    executor.run(|spawner| {
        spawner.spawn(rmt_task(channel)).ok();
    });
}
