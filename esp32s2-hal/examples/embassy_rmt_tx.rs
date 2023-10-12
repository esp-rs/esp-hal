//! Demonstrates generating pulse sequences with RMT
//! Connect a logic analyzer to GPIO4 to see the generated pulses.

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
    rmt::{asynch::TxChannelAsync, PulseCode, TxChannelConfig, TxChannelCreator},
    Rmt,
    IO,
};
use esp_backtrace as _;
use esp_println::println;
use xtensa_atomic_emulation_trap as _;

#[main]
async fn main(_spawner: Spawner) -> ! {
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
        .channel0
        .configure(
            io.pins.gpio4.into_push_pull_output(),
            TxChannelConfig {
                clk_divider: 255,
                ..TxChannelConfig::default()
            },
        )
        .unwrap();

    // you have to enable the interrupt for async to work
    esp32s2_hal::interrupt::enable(
        esp32s2_hal::peripherals::Interrupt::RMT,
        esp32s2_hal::interrupt::Priority::Priority1,
    )
    .unwrap();

    let mut data = [PulseCode {
        level1: true,
        length1: 200,
        level2: false,
        length2: 50,
    }; 20];

    data[data.len() - 2] = PulseCode {
        level1: true,
        length1: 3000,
        level2: false,
        length2: 500,
    };
    data[data.len() - 1] = PulseCode::default();

    loop {
        println!("transmit");
        channel.transmit(&data).await.unwrap();
        println!("transmitted\n");
        Timer::after(Duration::from_millis(500)).await;
    }
}
