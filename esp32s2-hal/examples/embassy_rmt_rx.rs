//! Demonstrates decoding pulse sequences with RMT
//! Connect GPIO15 to GPIO4

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp32s2_hal::{
    clock::ClockControl,
    embassy::{self},
    peripherals::Peripherals,
    prelude::*,
    rmt::{asynch::RxChannelAsync, PulseCode, RxChannelConfig, RxChannelCreator},
    Rmt,
    IO,
};
use esp_backtrace as _;
use esp_hal_common::gpio::{Gpio15, Output, PushPull};
use esp_println::{print, println};

const WIDTH: usize = 80;

#[cfg(debug_assertions)]
compile_error!("Run this example in release mode");

#[embassy_executor::task]
async fn signal_task(mut pin: Gpio15<Output<PushPull>>) {
    loop {
        for _ in 0..10 {
            pin.toggle().unwrap();
            Timer::after(Duration::from_micros(10)).await;
        }
        Timer::after(Duration::from_millis(1000)).await;
    }
}

#[main]
async fn main(spawner: Spawner) -> ! {
    #[cfg(feature = "log")]
    esp_println::logger::init_logger_from_env();
    println!("Init!");
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    #[cfg(feature = "embassy-time-systick")]
    embassy::init(
        &clocks,
        esp32s2_hal::systimer::SystemTimer::new(peripherals.SYSTIMER),
    );

    #[cfg(feature = "embassy-time-timg0")]
    {
        let timer_group0 = esp32s2_hal::timer::TimerGroup::new(peripherals.TIMG0, &clocks);
        embassy::init(&clocks, timer_group0.timer0);
    }

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let rmt = Rmt::new(peripherals.RMT, 80u32.MHz(), &clocks).unwrap();

    let mut channel = rmt
        .channel2
        .configure(
            io.pins.gpio4,
            RxChannelConfig {
                clk_divider: 1,
                idle_threshold: 0b111_1111_1111_1111,
                ..RxChannelConfig::default()
            },
        )
        .unwrap();

    // you have to enable the interrupt for async to work
    esp32s2_hal::interrupt::enable(
        esp32s2_hal::peripherals::Interrupt::RMT,
        esp32s2_hal::interrupt::Priority::Priority1,
    )
    .unwrap();

    spawner
        .spawn(signal_task(io.pins.gpio15.into_push_pull_output()))
        .ok();
    let mut data = [PulseCode {
        level1: true,
        length1: 1,
        level2: false,
        length2: 1,
    }; 48];

    loop {
        println!("receive");
        channel.receive(&mut data).await.unwrap();
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
