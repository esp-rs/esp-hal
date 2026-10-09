//! Multicore flash cache coherency test.
//!
//! This test verifies that flash operations remain correct when running
//! on one core while the other core creates significant cache pressure.
//!
//! It tests the `MultiCoreStrategy::AutoPark` and `MultiCoreStrategy::ignore()` strategies of
//! `esp_hal::flash::Flash` based on the boolean values of `FLASH_ON_CORE_0` and `USE_AUTO_PARK`.

//% CHIP_FILTER: multi_core && flash_driver_supported && !esp32 && !esp32p4
//% FEATURES: unstable

// TODO: Make esp32 work

#![no_std]
#![no_main]

use core::ptr::addr_of_mut;

use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    flash::{Config, Flash, MultiCoreStrategy},
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
    #[cfg(any(feature = "esp32h4", feature = "esp32s31"))]
    fn Cache_Invalidate_All(cache_map: u32);
    #[cfg(not(any(feature = "esp32h4", feature = "esp32s31")))]
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
            #[cfg(any(feature = "esp32h4", feature = "esp32s31"))]
            // CACHE_MAP_L1_ICACHE_0 | CACHE_MAP_L1_ICACHE_1 | CACHE_MAP_L1_DCACHE
            Cache_Invalidate_All(0x13);
            #[cfg(not(any(feature = "esp32h4", feature = "esp32s31")))]
            Cache_Invalidate_ICache_All();
            core::ptr::read_volatile(ptr);
        }
    }
}

fn flash_access(flash: esp_hal::peripherals::FLASH) {
    println!("flash access running");

    let strategy = if USE_AUTO_PARK {
        MultiCoreStrategy::AutoPark
    } else {
        // SAFETY: not sound, the other core reads from flash. This is what the
        // test exercises.
        unsafe { MultiCoreStrategy::ignore() }
    };
    let mut flash =
        Flash::new(flash, Config::default().with_multi_core_strategy(strategy)).unwrap();
    println!("flash created");
    let d = esp_hal::delay::Delay::new();

    loop {
        println!("Hello world!1");
        d.delay_millis(500);

        println!("write flash");
        other2();

        let foo = [0u32; 0x1000];

        // SAFETY: 0x9000 is the NVS partition of the default partition table,
        // which is not mapped.
        let res = unsafe { flash.write(0x9000, &foo) };
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
