//! Demonstrates generating pulse sequences with RMT
//! Connect a logic analyzer to GPIO4 to see the generated pulses.

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_time::{Duration, Timer};
use esp32_hal::{
    clock::ClockControl,
    embassy::{self, executor::Executor},
    peripherals::Peripherals,
    prelude::*,
    rmt::{asynch::TxChannelAsync, Channel0, PulseCode, TxChannelConfig, TxChannelCreator},
    Rmt,
    IO,
};
use esp_backtrace as _;
use static_cell::make_static;

#[embassy_executor::task]
async fn rmt_task(mut channel: Channel0<0>) {
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
        esp_println::println!("transmit");
        channel.transmit(&data).await.unwrap();
        esp_println::println!("transmitted\n");
        Timer::after(Duration::from_millis(500)).await;
    }
}

#[entry]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    esp_println::println!("Init!");
    let peripherals = Peripherals::take();
    let system = peripherals.DPORT.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    #[cfg(feature = "embassy-time-timg0")]
    {
        let timer_group0 = esp32_hal::timer::TimerGroup::new(peripherals.TIMG0, &clocks);
        embassy::init(&clocks, timer_group0.timer0);
    }

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let rmt = Rmt::new(peripherals.RMT, 80u32.MHz(), &clocks).unwrap();

    let channel = rmt
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
    esp32_hal::interrupt::enable(
        esp32_hal::peripherals::Interrupt::RMT,
        esp32_hal::interrupt::Priority::Priority1,
    )
    .unwrap();

    let executor = make_static!(Executor::new());
    executor.run(|spawner| {
        spawner.spawn(rmt_task(channel)).ok();
    });
}
