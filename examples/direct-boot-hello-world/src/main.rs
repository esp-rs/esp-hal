//! Hello world, booted directly by the ROM
//!
//! Besides printing, this checks what direct boot is responsible for: nothing but the entry stub
//! of `esp-bootloader-direct-boot` loads RAM.
//!
//! After a few seconds the chip enters deep sleep and wakes up again, so every other boot is a
//! deep-sleep wakeup. Initialized RTC fast memory statics are loaded from flash after any other
//! reset, and keep their values over deep sleep.

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
    rtc_cntl::{
        SocResetReason,
        reset_reason,
        sleep::{LowPower, RtcSleepConfig},
    },
    system::Cpu,
    time::{Duration, Instant},
};
use esp_println::println;

esp_bootloader_direct_boot::direct_boot!();

const DATA_PATTERN: u32 = 0xC0FF_EE42;

// Both are also written below, so the compiler can't turn them into constants.
static mut DATA_CHECK: u32 = DATA_PATTERN;
static mut BSS_CHECK: u32 = 0;

// The pattern, and a counter of the boots since the last reset that was not a deep-sleep wakeup.
// The linker scripts of these chips have no RTC fast memory.
#[cfg(not(any(feature = "esp32c2", feature = "esp32c61", feature = "esp32h4")))]
#[ram(unstable(rtc_fast))]
static mut RTC_DATA_CHECK: [u32; 2] = [DATA_PATTERN, 0];

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

    let reason = reset_reason(Cpu::ProCpu);
    println!("reset reason: {:?}", reason);
    let deep_sleep_wakeup = reason == Some(SocResetReason::CoreDeepSleep);

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

    #[cfg(not(any(feature = "esp32c2", feature = "esp32c61", feature = "esp32h4")))]
    {
        // SAFETY: single-threaded, no references are created.
        let [pattern, boots] = unsafe {
            let rtc_data = addr_of!(RTC_DATA_CHECK).read_volatile();
            addr_of_mut!(RTC_DATA_CHECK).write_volatile([rtc_data[0], rtc_data[1].wrapping_add(1)]);
            rtc_data
        };
        if deep_sleep_wakeup {
            report(
                "RTC .data kept over deep sleep",
                pattern == DATA_PATTERN && boots > 0,
            );
            println!("deep-sleep wakeups since the last reset: {}", boots);
        } else {
            report(
                "RTC .data copied from flash",
                pattern == DATA_PATTERN && boots == 0,
            );
        }
    }

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

    // Long enough for a watchdog the ROM armed to reset the chip.
    for _ in 0..3 {
        println!("Bing!");

        let now = Instant::now();
        while now.elapsed() < Duration::from_millis(5000) {}
    }

    println!("Entering deep sleep for 5 seconds");
    let mut lpwr = LowPower::new(peripherals.LPWR);
    lpwr.set_wakeup_deadline(Instant::now() + Duration::from_secs(5));
    // Let the UART finish the message.
    let now = Instant::now();
    while now.elapsed() < Duration::from_millis(100) {}
    lpwr.sleep_deep(RtcSleepConfig::deep());
}
