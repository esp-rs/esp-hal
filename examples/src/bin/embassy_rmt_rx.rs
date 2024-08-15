//! Demonstrates decoding pulse sequences with RMT
//!
//! The following wiring is assumed:
//! - Connect GPIO4 and GPIO5

//% CHIPS: esp32 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: async embassy embassy-generic-timers

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    gpio::{Gpio5, Io, Level, Output},
    peripherals::Peripherals,
    prelude::*,
    rmt::{asynch::RxChannelAsync, PulseCode, Rmt, RxChannelConfig, RxChannelCreatorAsync},
    system::SystemControl,
    timer::timg::TimerGroup,
};
use esp_println::{print, println};

const WIDTH: usize = 80;

#[cfg(debug_assertions)]
compile_error!("Run this example in release mode");

#[embassy_executor::task]
async fn signal_task(mut pin: Output<'static, Gpio5>) {
    loop {
        for _ in 0..10 {
            pin.toggle();
            Timer::after(Duration::from_micros(10)).await;
        }
        Timer::after(Duration::from_millis(1000)).await;
    }
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    println!("Init!");
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    esp_hal_embassy::init(&clocks, timg0.timer0);

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    cfg_if::cfg_if! {
        if #[cfg(feature = "esp32h2")] {
            let freq = 32.MHz();
        } else {
            let freq = 80.MHz();
        }
    };

    let rmt = Rmt::new_async(peripherals.RMT, freq, &clocks).unwrap();
    let rx_config = RxChannelConfig {
        clk_divider: 255,
        idle_threshold: 10000,
        ..RxChannelConfig::default()
    };

    cfg_if::cfg_if! {
        if #[cfg(any(feature = "esp32", feature = "esp32s2"))] {
            let mut channel = rmt.channel0.configure(io.pins.gpio4, rx_config).unwrap();
        } else if #[cfg(feature = "esp32s3")] {
            let mut channel = rmt.channel7.configure(io.pins.gpio4, rx_config).unwrap();
        } else {
            let mut channel = rmt.channel2.configure(io.pins.gpio4, rx_config).unwrap();
        }
    }

    spawner
        .spawn(signal_task(Output::new(io.pins.gpio5, Level::Low)))
        .unwrap();

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
