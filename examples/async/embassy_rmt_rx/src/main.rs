//! Demonstrates decoding pulse sequences with RMT
//!
//! The following wiring is assumed:
//! - Connect GPIO4 and GPIO5

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
#[cfg(target_arch = "riscv32")]
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::{
    gpio::{Level, Output, OutputConfig},
    rmt::{PulseCode, Rmt, RxChannelConfig, RxChannelCreator},
    time::Rate,
    timer::timg::TimerGroup,
};
use esp_println::{print, println};

esp_bootloader_esp_idf::esp_app_desc!();

const WIDTH: usize = 80;

#[cfg(is_not_release)]
compile_error!("Run this example in release mode");

#[embassy_executor::task]
async fn signal_task(mut pin: Output<'static>) {
    loop {
        for _ in 0..10 {
            pin.toggle();
            Timer::after(Duration::from_micros(10)).await;
        }
        Timer::after(Duration::from_millis(1000)).await;
    }
}

#[esp_rtos::main]
async fn main(spawner: Spawner) {
    println!("Init!");

    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default());

    #[cfg(target_arch = "riscv32")]
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(
        timg0.timer0,
        #[cfg(target_arch = "riscv32")]
        sw_int.software_interrupt0,
    );

    cfg_if::cfg_if! {
        if #[cfg(feature = "esp32h2")] {
            let freq = Rate::from_mhz(32);
        } else {
            let freq = Rate::from_mhz(80);
        }
    };

    let rmt = Rmt::new(peripherals.RMT, freq).unwrap().into_async();
    let rx_config = RxChannelConfig::default()
        .with_clk_divider(255)
        .with_idle_threshold(10000);

    cfg_if::cfg_if! {
        if #[cfg(any(feature = "esp32", feature = "esp32s2"))] {
            let mut channel = rmt.channel0.configure_rx(peripherals.GPIO4, rx_config).unwrap();
        } else if #[cfg(feature = "esp32s3")] {
            let mut channel = rmt.channel7.configure_rx(peripherals.GPIO4, rx_config).unwrap();
        } else {
            let mut channel = rmt.channel2.configure_rx(peripherals.GPIO4, rx_config).unwrap();
        }
    }

    spawner
        .spawn(signal_task(Output::new(
            peripherals.GPIO5,
            Level::Low,
            OutputConfig::default(),
        )))
        .unwrap();

    let mut data = [PulseCode::default(); 48];

    loop {
        println!("receive");
        channel.receive(&mut data).await.unwrap();
        let mut total = 0usize;
        for entry in &data[..data.len()] {
            if entry.length1() == 0 {
                break;
            }
            total += entry.length1() as usize;

            if entry.length2() == 0 {
                break;
            }
            total += entry.length2() as usize;
        }

        for entry in &data[..data.len()] {
            if entry.length1() == 0 {
                break;
            }

            let count = WIDTH / (total / entry.length1() as usize);
            let c = match entry.level1() {
                Level::High => '-',
                Level::Low => '_',
            };
            for _ in 0..count + 1 {
                print!("{}", c);
            }

            if entry.length2() == 0 {
                break;
            }

            let count = WIDTH / (total / entry.length2() as usize);
            let c = match entry.level2() {
                Level::High => '-',
                Level::Low => '_',
            };
            for _ in 0..count + 1 {
                print!("{}", c);
            }
        }

        println!();
    }
}
