//! Hello world, booted directly by the ROM
//!
//! Besides printing, this checks what direct boot is responsible for: nothing but the entry stub
//! of `esp-bootloader-direct-boot` loads RAM.

//% CHIP_FILTER: !esp32 && !esp32s2

#![no_std]
#![no_main]

use core::{
    ptr::{addr_of, addr_of_mut},
    sync::atomic::{AtomicBool, Ordering},
};

use esp_backtrace as _;
use esp_hal::{
    handler,
    interrupt::software::SoftwareInterrupt,
    main,
    peripherals::FROM_CPU_INTR2,
    ram,
    time::{Duration, Instant},
};
use esp_println::println;

esp_bootloader_direct_boot::direct_boot!();

const DATA_PATTERN: u32 = 0xC0FF_EE42;

// Both are also written below, so the compiler can't turn them into constants.
static mut DATA_CHECK: u32 = DATA_PATTERN;
static mut BSS_CHECK: u32 = 0;

static SWINT_FIRED: AtomicBool = AtomicBool::new(false);

#[ram]
fn ram_function(x: u32) -> u32 {
    x.wrapping_mul(3).wrapping_add(1)
}

#[handler]
fn swint_handler() {
    // SAFETY: only used to clear the interrupt request that triggered this handler.
    unsafe { SoftwareInterrupt::<2>::new(FROM_CPU_INTR2::steal()) }.reset();
    SWINT_FIRED.store(true, Ordering::Release);
}

fn report(name: &str, ok: bool) {
    println!("[{}] {}", if ok { " OK " } else { "FAIL" }, name);
}

#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default());

    println!("Hello from direct boot!");

    // SAFETY: single-threaded, no references are created.
    let (data, bss) = unsafe {
        let data = addr_of!(DATA_CHECK).read_volatile();
        let bss = addr_of!(BSS_CHECK).read_volatile();
        addr_of_mut!(DATA_CHECK).write_volatile(data.wrapping_add(1));
        addr_of_mut!(BSS_CHECK).write_volatile(bss.wrapping_add(1));
        (data, bss)
    };
    report(".data copied from flash", data == DATA_PATTERN);
    report(".bss zeroed", bss == 0);

    let ram_fn: fn(u32) -> u32 = ram_function;
    report(".rwtext copied from flash", ram_fn(41) == 124);

    let mut swint = SoftwareInterrupt::new(peripherals.FROM_CPU_INTR2);
    swint.set_interrupt_handler(swint_handler);
    swint.raise();
    let start = Instant::now();
    while !SWINT_FIRED.load(Ordering::Acquire) && start.elapsed() < Duration::from_millis(100) {}
    report(
        "software interrupt handled",
        SWINT_FIRED.load(Ordering::Acquire),
    );

    loop {
        println!("Bing!");

        let now = Instant::now();
        while now.elapsed() < Duration::from_millis(5000) {}
    }
}
