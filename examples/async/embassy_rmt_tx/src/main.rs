//! Demonstrates generating pulse sequences with RMT
//!
//! Connect a logic analyzer to GPIO4 to see the generated pulses.
//!
//! The following wiring is assumed:
//! - generated pulses => GPIO4

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
#[cfg(target_arch = "riscv32")]
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::{
    gpio::Level,
    rmt::{PulseCode, Rmt, TxChannelConfig, TxChannelCreator},
    time::Rate,
    timer::timg::TimerGroup,
};
use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(_spawner: Spawner) {
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

    let mut channel = rmt
        .channel0
        .configure_tx(
            peripherals.GPIO4,
            TxChannelConfig::default().with_clk_divider(255),
        )
        .unwrap();

    let mut data = [PulseCode::new(Level::High, 200, Level::Low, 50); 20];

    data[data.len() - 2] = PulseCode::new(Level::High, 3000, Level::Low, 500);
    data[data.len() - 1] = PulseCode::end_marker();

    loop {
        println!("transmit");
        channel.transmit(&data).await.unwrap();
        println!("transmitted\n");
        Timer::after(Duration::from_millis(500)).await;
    }
}
