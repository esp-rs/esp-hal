//! SDMMC SD-card bring-up test (interrupt-driven async, ESP32 / ESP32-S3).
//!
//! Initializes an SD card through the `sdio` crate's `MmcBus` integration, reads
//! block 0, then round-trips a known pattern through a scratch block. A
//! concurrent heartbeat task confirms the data path does not busy-wait.
//!
//! Slot 1 pins (fixed IO_MUX pads on ESP32, GPIO matrix on ESP32-S3):
//! - CLK  => GPIO14
//! - CMD  => GPIO15
//! - DAT0 => GPIO2
//! - DAT1 => GPIO4
//! - DAT2 => GPIO12
//! - DAT3 => GPIO13

//% CHIP_FILTER: esp32s3 || esp32

#![no_std]
#![no_main]

use aligned::{A4, Aligned};
use block_device_driver::BlockDevice as _;
use embassy_executor::Spawner;
use embassy_time::Delay;
use esp_backtrace as _;
use esp_hal::{
    interrupt::software::SoftwareInterruptControl,
    sdmmc::{Config, SdHostController},
    timer::timg::TimerGroup,
};
use sdio::{BlockDevice, sd::Card};

esp_bootloader_esp_idf::esp_app_desc!();

/// Block index used for the destructive write/read-back round-trip.
const SCRATCH_BLOCK: u32 = 0x2000;

/// Heartbeat task: proves the executor keeps running during transfers.
#[embassy_executor::task]
async fn heartbeat() {
    let mut tick = 0u32;
    loop {
        esp_println::println!("tick {tick}");
        tick += 1;
        embassy_time::Timer::after(embassy_time::Duration::from_millis(100)).await;
    }
}

#[esp_hal::main]
async fn main(spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    spawner.spawn(heartbeat().unwrap());

    let mut controller = SdHostController::new(peripherals.SDHOST);
    let slot = controller
        .slot::<1>(Config::default())
        .unwrap()
        .with_clk(peripherals.GPIO14)
        .with_cmd(peripherals.GPIO15)
        .with_data0(peripherals.GPIO2)
        .with_data1(peripherals.GPIO4)
        .with_data2(peripherals.GPIO12)
        .with_data3(peripherals.GPIO13)
        .into_async();

    let mut card: BlockDevice<Card, _, _, 512> = BlockDevice::new_sd_card(slot, 40_000_000, Delay)
        .await
        .unwrap();

    esp_println::println!("card initialized: {:?}", card.card());

    // Read block 0 and report the first bytes and the MBR signature.
    let mut block = [Aligned::<A4, _>([0u8; 512])];
    card.read(0, &mut block).await.unwrap();
    esp_println::println!("block 0 head: {:02x?}", &block[0][..16]);
    esp_println::println!("mbr signature: {:02x}{:02x}", block[0][510], block[0][511]);

    // Round-trip a known pattern through a scratch block.
    let mut pattern = [Aligned::<A4, _>([0u8; 512])];
    for (i, b) in pattern[0].iter_mut().enumerate() {
        *b = (i as u8) ^ 0xA5;
    }
    card.write(SCRATCH_BLOCK, &pattern).await.unwrap();

    let mut readback = [Aligned::<A4, _>([0u8; 512])];
    card.read(SCRATCH_BLOCK, &mut readback).await.unwrap();

    if readback[0][..] == pattern[0][..] {
        esp_println::println!("scratch block round-trip: PASS");
    } else {
        esp_println::println!("scratch block round-trip: FAIL");
    }

    loop {
        embassy_time::Timer::after(embassy_time::Duration::from_secs(1)).await;
    }
}
