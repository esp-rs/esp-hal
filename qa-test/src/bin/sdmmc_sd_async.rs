//! SDMMC async SD-card test (interrupt-driven) on a FAT volume via embedded-fatfs.
//!
//! Initializes an SD card through the `sdio` crate's `MmcBus` integration, mounts
//! the first FAT partition (or a superfloppy volume), then performs a
//! non-destructive create/update/read/delete cycle on a test-owned file that is
//! assumed not to exist. The cycle includes a multi-block payload so both the
//! single-block (CMD24/CMD17) and multi-block (CMD25/CMD18, with auto-stop)
//! transfer paths are written and read back. No raw blocks are overwritten.
//!
//! Pins:
//!
//! | Signal | ESP32-S3 (1-bit) | ESP32 (4-bit) | ESP32-P4 (4-bit, slot 0)
//! | ------ | ---------------  | ------------- | ------------------------
//! | CLK    | GPIO39           | GPIO14        | GPIO43
//! | CMD    | GPIO38           | GPIO15        | GPIO44
//! | DAT0   | GPIO40           | GPIO2         | GPIO39
//! | DAT1   | -                | GPIO4         | GPIO40
//! | DAT2   | -                | GPIO12        | GPIO41
//! | DAT3   | -                | GPIO13        | GPIO42

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
#[cfg(any(feature = "esp32s3", feature = "esp32p4"))]
use esp_hal::sdmmc::DelayPhase;
use esp_hal::{
    gpio::{Input, InputConfig, Pull},
    interrupt::software::SoftwareInterruptControl,
    sdmmc::{Config, SdHostController, SlotConfig},
    timer::timg::TimerGroup,
};
use esp_println::println;
use sdio::{BlockDevice, sd::Card};

esp_bootloader_esp_idf::esp_app_desc!();

/// Target card clock. 40 MHz exercises the SD high-speed (SDR25) path.
const CARD_HZ: u32 = 40_000_000;

/// Input sampling phase for the high-speed path (esp32s3/p4 only). If 40 MHz
/// init fails with a timeout, try `_1`, `_2`, then `_3`.
#[cfg(any(feature = "esp32s3", feature = "esp32p4"))]
const INPUT_DELAY_PHASE: DelayPhase = DelayPhase::_0;

/// Test-owned file (8.3 name); assumed not to exist on the card.
const TEST_FILE: &str = "ESPQA.TXT";
const MSG_CREATE: &[u8] = b"esp-hal sdmmc async create\n";
const MSG_UPDATE: &[u8] = b"esp-hal sdmmc async update (longer payload)\n";

/// Length of the multi-block payload. Spans several 512-byte blocks so the
/// write and read take the multi-block path (CMD25 / CMD18), which appends an
/// auto-stop (CMD12) the single-block path does not.
const MULTIBLOCK_LEN: usize = 2048;

/// Deterministic, position-dependent byte for the multi-block payload. The
/// prime modulus keeps the pattern from lining up with 512-byte block
/// boundaries, so a dropped or mis-ordered block is caught on read-back.
const fn pattern_byte(i: usize) -> u8 {
    (i % 251) as u8
}

#[esp_hal::main]
async fn main(_spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    esp_println::logger::init_logger(log::LevelFilter::Info);
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    let controller = SdHostController::new(peripherals.SDHOST, Config::default()).unwrap();
    let slot_config = SlotConfig::default();
    cfg_select! {
        feature = "esp32s3" => {
            let slot = controller.slot::<1>(slot_config.with_input_delay_phase(INPUT_DELAY_PHASE)).unwrap();
            let slot = slot
                .with_clk(peripherals.GPIO39)
                .with_cmd(peripherals.GPIO38)
                .with_data0(peripherals.GPIO40);
        }
        feature = "esp32" => {
            let slot = controller.slot::<1>(slot_config).unwrap();
            let slot = slot
                .with_clk(peripherals.GPIO14)
                .with_cmd(peripherals.GPIO15)
                .with_data0(peripherals.GPIO2)
                .with_data1(peripherals.GPIO4)
                .with_data2(peripherals.GPIO12)
                .with_data3(peripherals.GPIO13);
        }
        feature = "esp32p4" => {
            let slot = controller.slot::<0>(slot_config.with_input_delay_phase(INPUT_DELAY_PHASE)).unwrap();
            let slot = slot
                .with_clk(peripherals.GPIO43)
                .with_cmd(peripherals.GPIO44)
                .with_data0(peripherals.GPIO39)
                .with_data1(peripherals.GPIO40)
                .with_data2(peripherals.GPIO41)
                .with_data3(peripherals.GPIO42);
        }
    }

    let slot = slot.into_async();

    let mut card: BlockDevice<Card, _, _, 512> = BlockDevice::new_sd_card(slot, CARD_HZ, Delay)
        .await
        .unwrap();
    println!("card initialized: {:?}", card.card());

    let mut button = Input::new(
        cfg_select! {
            feature = "esp32p4" => peripherals.GPIO35,
            _ => peripherals.GPIO0,
        },
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

/// Creates, reads, updates and deletes [`TEST_FILE`]; returns whether every
/// round-tripped payload (single- and multi-block) matched.
async fn fat_crud_inner<IO: ReadWriteSeek>(
    io: IO,
) -> Result<bool, embedded_fatfs::Error<IO::Error>> {
    let fs = FileSystem::new(io, FsOptions::new()).await?;
    let ok = {
        let root = fs.root_dir();

        // Single-block create: write, then read back and verify.
        let mut f = root.create_file(TEST_FILE).await?;
        f.truncate().await?;
        f.write_all(MSG_CREATE).await?;
        f.flush().await?;
        drop(f);
        let created_ok = verify(root.open_file(TEST_FILE).await?, MSG_CREATE.len(), |i| {
            MSG_CREATE[i]
        })
        .await?;

        // Single-block update: overwrite with a different payload, verify.
        let mut f = root.open_file(TEST_FILE).await?;
        f.truncate().await?;
        f.write_all(MSG_UPDATE).await?;
        f.flush().await?;
        drop(f);
        let updated_ok = verify(root.open_file(TEST_FILE).await?, MSG_UPDATE.len(), |i| {
            MSG_UPDATE[i]
        })
        .await?;

        // Multi-block update: a single large write forces the block layer down
        // the CMD25 (write) / CMD18 (read) path. The buffer lives on the stack
        // (DMA-reachable RAM), not in flash.
        let mut payload = [0u8; MULTIBLOCK_LEN];
        for (i, b) in payload.iter_mut().enumerate() {
            *b = pattern_byte(i);
        }
        let mut f = root.open_file(TEST_FILE).await?;
        f.truncate().await?;
        f.write_all(&payload).await?;
        f.flush().await?;
        drop(f);
        let multiblock_ok = verify(
            root.open_file(TEST_FILE).await?,
            MULTIBLOCK_LEN,
            pattern_byte,
        )
        .await?;

        // Delete, leaving the card as we found it.
        root.remove(TEST_FILE).await?;
        created_ok && updated_ok && multiblock_ok
    };
    fs.unmount().await?;
    Ok(ok)
}

/// Reads exactly `len` bytes from `r` in chunks and checks each against
/// `expected(offset)`. Returns `Ok(false)` on any mismatch or a short read
/// (premature EOF), so it doubles as the multi-block read-back check.
async fn verify<R: Read>(
    mut r: R,
    len: usize,
    expected: impl Fn(usize) -> u8,
) -> Result<bool, R::Error> {
    let mut buf = [0u8; 64];
    let mut off = 0;
    while off < len {
        let want = (len - off).min(buf.len());
        let mut got = 0;
        while got < want {
            match r.read(&mut buf[got..want]).await? {
                0 => return Ok(false),
                n => got += n,
            }
        }
        if buf[..want]
            .iter()
            .enumerate()
            .any(|(i, &b)| b != expected(off + i))
        {
            return Ok(false);
        }
        off += want;
    }
    Ok(true)
}
