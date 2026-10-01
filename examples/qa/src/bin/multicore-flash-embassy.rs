//! Flash writes with `multicore_auto_park()` while the other core is busy with cross-core work.
//!
//! Both cores run esp-rtos + embassy. Core 0 erases and writes flash in a loop, and a task on
//! each core ping-pongs a `Signal` with no delay, with a few short timers on core 1, so core 1
//! spends most of its time in the scheduler and cross-core wake paths.
//!
//! Stalling core 1 from outside used to deadlock here within seconds: interrupts on core 0 ran
//! between the ROM calls of a write, while core 1 was still stalled, and waited on scheduler
//! state core 1 held. Success: `writes=` and `round_trips=` keep increasing. Failure: the output
//! stops, or "FAIL: ping-pong stopped".
//!
//! NOTE: erases and rewrites 0x3C0000..0x400000, the last 256 KB of a 4 MB flash, which the
//! default partition table leaves unused. Sectors are erased in rotation to spread the wear.

//% CHIP_FILTER: esp32s3
//% FEATURES: unstable esp-storage

#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU32, Ordering};

use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Instant, Timer};
use esp_backtrace as _;
use esp_hal::{clock::CpuClock, system::Stack, timer::timg::TimerGroup};
use esp_println::println;
use esp_rtos::embassy::Executor;
use esp_storage::FlashStorage;
use static_cell::StaticCell;

esp_bootloader_esp_idf::esp_app_desc!();

const SECTOR_SIZE: u32 = FlashStorage::SECTOR_SIZE;
const REGION_SECTORS: u32 = 64;
/// Below 4 MB on purpose: the ROM rejects anything past the flash size in the image header.
const REGION_START: u32 = 0x40_0000 - REGION_SECTORS * SECTOR_SIZE;
/// Each write parks core 1 once, so small chunks mean many parks.
const CHUNK_SIZE: usize = 16;

const SHORT_TIMERS: usize = 4;
const SHORT_TIMER_PERIOD: Duration = Duration::from_micros(100);

static PING: Signal<CriticalSectionRawMutex, ()> = Signal::new();
static PONG: Signal<CriticalSectionRawMutex, ()> = Signal::new();
static ROUND_TRIPS: AtomicU32 = AtomicU32::new(0);

/// Core 0 half of the ping-pong.
#[embassy_executor::task]
async fn pinger() {
    loop {
        PING.signal(());
        PONG.wait().await;
        ROUND_TRIPS.fetch_add(1, Ordering::Relaxed);
    }
}

/// Core 1 half of the ping-pong.
#[embassy_executor::task]
async fn ponger() {
    loop {
        PING.wait().await;
        PONG.signal(());
    }
}

#[embassy_executor::task(pool_size = SHORT_TIMERS)]
async fn short_timer() {
    loop {
        Timer::after(SHORT_TIMER_PERIOD).await;
    }
}

#[embassy_executor::task]
async fn flash_task(mut flash: FlashStorage<'static>) {
    let data = [0x5Au8; CHUNK_SIZE];
    let mut sector_index = 0;
    let mut writes: u32 = 0;
    let mut last_report = Instant::now();
    let mut last_round_trips = 0;

    loop {
        let sector = REGION_START + sector_index * SECTOR_SIZE;
        flash.erase(sector, sector + SECTOR_SIZE).unwrap();
        for offset in (sector..sector + SECTOR_SIZE).step_by(CHUNK_SIZE) {
            flash.write_nor(offset, &data).unwrap();
            writes += 1;
        }
        sector_index = (sector_index + 1) % REGION_SECTORS;

        if last_report.elapsed() > Duration::from_secs(2) {
            let round_trips = ROUND_TRIPS.load(Ordering::Relaxed);
            println!("writes={writes} round_trips={round_trips}");
            if round_trips == last_round_trips {
                println!("FAIL: ping-pong stopped");
            }
            last_round_trips = round_trips;
            last_report = Instant::now();
        }

        // Let the ping-pong run on core 0 between sectors.
        embassy_futures::yield_now().await;
    }
}

#[esp_hal::main]
async fn main(spawner: Spawner) {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    static APP_CORE_STACK: StaticCell<Stack<8192>> = StaticCell::new();
    let app_core_stack = APP_CORE_STACK.init(Stack::new());
    esp_rtos::start_second_core(peripherals.CPU_CTRL, app_core_stack, || {
        static EXECUTOR: StaticCell<Executor> = StaticCell::new();
        let executor = EXECUTOR.init(Executor::new());
        executor.run(|spawner| {
            spawner.spawn(ponger().unwrap());
            for _ in 0..SHORT_TIMERS {
                spawner.spawn(short_timer().unwrap());
            }
        });
    });

    let flash = FlashStorage::new(peripherals.FLASH).multicore_auto_park();
    spawner.spawn(flash_task(flash).unwrap());
    spawner.spawn(pinger().unwrap());

    loop {
        Timer::after(Duration::from_secs(10)).await;
    }
}
