//! Multicore flash cache coherency test.
//!
//! This test verifies that flash operations remain correct when running
//! on one core while the other core creates significant cache pressure.
//!
//! It tests the `multicore_auto_park()` and `multicore_ignore()` functionality based on the boolean
//! values of `FLASH_ON_CORE_0` and `USE_AUTO_PARK`.

//% CHIPS: esp32s3
//% FEATURES: unstable esp-storage

#![no_std]
#![no_main]

use core::ptr::addr_of_mut;

use embedded_storage::nor_flash::NorFlash;
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    main,
    peripherals::FLASH,
    system::{CpuControl, Stack},
};
use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

const FLASH_ON_CORE_0: bool = false;
const USE_AUTO_PARK: bool = true;

const CACHE_PRESSURE_BASE: u32 = 0x4200_0000;

const CACHE_PRESSURE_SIZE: usize = 65536; // 64KB

unsafe extern "C" {
    fn Cache_Invalidate_ICache_All();
}

#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    println!("firmware running");

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let mut cpu_control = CpuControl::new(peripherals.CPU_CTRL);
    static mut APP_CORE_STACK: Stack<{ 4096 * 8 }> = Stack::new();
    let _guard = cpu_control
        .start_app_core(unsafe { &mut *addr_of_mut!(APP_CORE_STACK) }, move || {
            println!("second core started");

            if !FLASH_ON_CORE_0 {
                println!("flash access on core0");
                flash_access(peripherals.FLASH);
                println!("end flash access on core0");
            } else {
                println!("NO flash access on core0");
                other_core();
            }

            println!("second core finished");

            loop {}
        })
        .unwrap();

    let d = esp_hal::delay::Delay::new();
    d.delay_millis(1_500);

    if FLASH_ON_CORE_0 {
        flash_access(unsafe { FLASH::steal() });
    } else {
        other_core();
    }

    loop {
        println!("guard = {:p}", &_guard);
    }
}

fn read() {
    unsafe {
        let base_ptr = CACHE_PRESSURE_BASE as *const u32;

        // Read through the entire region to stress cache
        for offset in (0..CACHE_PRESSURE_SIZE).step_by(32) {
            let ptr = base_ptr.add(offset / 4);
            Cache_Invalidate_ICache_All();
            core::ptr::read_volatile(ptr);
        }
    }
}

fn flash_access(flash: esp_hal::peripherals::FLASH) {
    println!("flash access running");

    let flash = esp_storage::FlashStorage::new(flash);
    let mut flash = if USE_AUTO_PARK {
        flash.multicore_auto_park()
    } else {
        unsafe { flash.multicore_ignore() }
    };
    println!("flash created");
    let d = esp_hal::delay::Delay::new();

    loop {
        println!("Hello world!1");
        d.delay_millis(500);

        println!("write flash");
        other2();

        let mut foo = [0u8; 0x4000];

        let res = flash.write(0x9000, &mut foo);
        println!("Writing to flash result: {:?}", res);
    }
}

#[inline(never)]
fn other2() {
    read();
}

fn other_core() -> ! {
    println!("random stuff running");

    loop {
        read();
    }
}
