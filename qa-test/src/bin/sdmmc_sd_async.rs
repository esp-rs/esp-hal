//! SDMMC async SD-card test (interrupt-driven) on a FAT volume via embedded-fatfs.
//!
//! Initializes an SD card through the `sdio` crate's `MmcBus` integration, mounts
//! the first FAT partition (or a superfloppy volume), then performs a
//! non-destructive create/update/read/delete cycle on a test-owned file that is
//! assumed not to exist. No raw blocks are overwritten.
//!
//! Slot 1 pins:
//!
//! | Signal | ESP32-S3 (1-bit, GPIO matrix) | ESP32 (4-bit, fixed IO_MUX) |
//! | ------ | ----------------------------- | --------------------------- |
//! | CLK    | GPIO39                        | GPIO14                      |
//! | CMD    | GPIO38                        | GPIO15                      |
//! | DAT0   | GPIO40                        | GPIO2                       |
//! | DAT1   | -                             | GPIO4                       |
//! | DAT2   | -                             | GPIO12                      |
//! | DAT3   | -                             | GPIO13                      |

//% CHIP_FILTER: sdmmc_driver_supported

#![no_std]
#![no_main]

use block_device_adapters::BufStream;
use embassy_executor::Spawner;
use embassy_time::Delay;
use embedded_fatfs::{FileSystem, FsOptions, ReadWriteSeek};
use embedded_io_async::{Read, Write};
use embedded_partitions::mbr::Scheme;
use esp_backtrace as _;
use esp_hal::{
    gpio::{Input, InputConfig, Pull},
    interrupt::software::SoftwareInterruptControl,
    sdmmc::{Config, DelayPhase, SdHostController},
    timer::timg::TimerGroup,
};
use esp_println::println;
use sdio::{BlockDevice, sd::Card};

esp_bootloader_esp_idf::esp_app_desc!();

/// Target card clock. 40 MHz exercises the SD high-speed (SDR25) path.
const CARD_HZ: u32 = 40_000_000;

/// Input sampling phase for the high-speed path (esp32s3 only). If 40 MHz
/// init fails with a timeout, try `_1`, `_2`, then `_3`.
const INPUT_DELAY_PHASE: DelayPhase = DelayPhase::_0;

/// Test-owned file (8.3 name); assumed not to exist on the card.
const TEST_FILE: &str = "ESPQA.TXT";
const MSG_CREATE: &[u8] = b"esp-hal sdmmc async create\n";
const MSG_UPDATE: &[u8] = b"esp-hal sdmmc async update (longer payload)\n";

#[esp_hal::main]
async fn main(_spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    let controller = SdHostController::new(peripherals.SDHOST);
    let config = Config::default().with_input_delay_phase(INPUT_DELAY_PHASE);
    let slot = controller.slot::<1>(config).unwrap();
    #[cfg(feature = "esp32s3")]
    let slot = slot
        .with_clk(peripherals.GPIO39)
        .with_cmd(peripherals.GPIO38)
        .with_data0(peripherals.GPIO40);
    #[cfg(feature = "esp32")]
    let slot = slot
        .with_clk(peripherals.GPIO14)
        .with_cmd(peripherals.GPIO15)
        .with_data0(peripherals.GPIO2)
        .with_data1(peripherals.GPIO4)
        .with_data2(peripherals.GPIO12)
        .with_data3(peripherals.GPIO13);
    let slot = slot.into_async();

    let mut card: BlockDevice<Card, _, _, 512> = BlockDevice::new_sd_card(slot, CARD_HZ, Delay)
        .await
        .unwrap();
    println!("card initialized: {:?}", card.card());

    let mut button = Input::new(
        peripherals.GPIO0,
        InputConfig::default().with_pull(Pull::Up),
    );
    println!("Press the BOOT button to run the SD card test.");

    loop {
        button.wait_for_falling_edge().await;

        let stream = BufStream::<_, 512>::new(&mut card);
        match Scheme::open(stream).await {
            Ok(Scheme::Mbr(mut mbr)) => {
                let fat_idx = mbr.iter_used().find(|(_, p)| p.is_fat()).map(|(i, _)| i);
                match fat_idx {
                    Some(idx) => match mbr.open_partition(idx).await {
                        Ok(slice) => fat_crud(slice).await,
                        Err(e) => println!("open_partition failed: {e:?}"),
                    },
                    None => println!("no FAT partition found"),
                }
            }
            Ok(Scheme::Superfloppy(io)) => fat_crud(io).await,
            Ok(Scheme::Unknown(_)) => println!("unrecognised volume layout"),
            Err(e) => println!("partition scan failed: {e:?}"),
        }

        println!("Press the BOOT button to re-run.");
        button.wait_for_high().await;
    }
}

/// Runs the CRUD cycle and reports the outcome.
async fn fat_crud<IO: ReadWriteSeek>(io: IO) {
    match fat_crud_inner(io).await {
        Ok(true) => println!("async FAT CRUD: PASS"),
        Ok(false) => println!("async FAT CRUD: FAIL (content mismatch)"),
        Err(e) => println!("async FAT CRUD: FAIL: {e:?}"),
    }
}

/// Creates, reads, updates and deletes [`TEST_FILE`]; returns whether the
/// round-tripped contents matched.
async fn fat_crud_inner<IO: ReadWriteSeek>(
    io: IO,
) -> Result<bool, embedded_fatfs::Error<IO::Error>> {
    let fs = FileSystem::new(io, FsOptions::new()).await?;
    let ok = {
        let root = fs.root_dir();
        let mut buf = [0u8; 64];

        // Create + write.
        let mut f = root.create_file(TEST_FILE).await?;
        f.truncate().await?;
        f.write_all(MSG_CREATE).await?;
        f.flush().await?;
        drop(f);

        // Read back the created content.
        let mut f = root.open_file(TEST_FILE).await?;
        f.read_exact(&mut buf[..MSG_CREATE.len()]).await?;
        let created_ok = &buf[..MSG_CREATE.len()] == MSG_CREATE;
        drop(f);

        // Update: truncate and rewrite with a different payload.
        let mut f = root.open_file(TEST_FILE).await?;
        f.truncate().await?;
        f.write_all(MSG_UPDATE).await?;
        f.flush().await?;
        drop(f);

        // Read back the updated content.
        let mut f = root.open_file(TEST_FILE).await?;
        f.read_exact(&mut buf[..MSG_UPDATE.len()]).await?;
        let updated_ok = &buf[..MSG_UPDATE.len()] == MSG_UPDATE;
        drop(f);

        // Delete, leaving the card as we found it.
        root.remove(TEST_FILE).await?;
        created_ok && updated_ok
    };
    fs.unmount().await?;
    Ok(ok)
}
