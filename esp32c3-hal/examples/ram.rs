//! This shows how to use RTC memory.
//! RTC memory is retained during resets and during most sleep modes.
//! Initialized memory is always re-initialized on startup.
//! Uninitialzed memory isn't initialized on startup and can be used to keep
//! data during resets. Zeroed memory is initialized to zero on startup.
//! We can also run code from RTC memory.

#![no_std]
#![no_main]

use esp32c3_hal::{
    clock::ClockControl,
    macros::ram,
    peripherals::Peripherals,
    prelude::*,
    timer::TimerGroup,
};
use esp_backtrace as _;
use esp_println::println;
use nb::block;
use riscv_rt::entry;

#[ram(rtc_fast)]
static mut SOME_INITED_DATA: [u8; 2] = [0xaa, 0xbb];

#[ram(rtc_fast, uninitialized)]
static mut SOME_UNINITED_DATA: [u8; 2] = [0; 2];

#[ram(rtc_fast, zeroed)]
static mut SOME_ZEROED_DATA: [u8; 8] = [0; 8];

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut timer0 = timer_group0.timer0;
    let mut wdt0 = timer_group0.wdt;

    // Disable MWDT flash boot protection
    wdt0.disable();
    // The RWDT flash boot protection remains enabled and it being triggered is part
    // of the example

    timer0.start(1u64.secs());

    println!(
        "IRAM function located at {:p}",
        function_in_ram as *const ()
    );
    unsafe {
        println!("SOME_INITED_DATA {:x?}", SOME_INITED_DATA);
        println!("SOME_UNINITED_DATA {:x?}", SOME_UNINITED_DATA);
        println!("SOME_ZEROED_DATA {:x?}", SOME_ZEROED_DATA);

        SOME_INITED_DATA[0] = 0xff;
        SOME_ZEROED_DATA[0] = 0xff;

        println!("SOME_INITED_DATA {:x?}", SOME_INITED_DATA);
        println!("SOME_UNINITED_DATA {:x?}", SOME_UNINITED_DATA);
        println!("SOME_ZEROED_DATA {:x?}", SOME_ZEROED_DATA);

        if SOME_UNINITED_DATA[0] != 0 {
            SOME_UNINITED_DATA[0] = 0;
            SOME_UNINITED_DATA[1] = 0;
        }

        if SOME_UNINITED_DATA[1] == 0xff {
            SOME_UNINITED_DATA[1] = 0;
        }

        println!("Counter {}", SOME_UNINITED_DATA[1]);
        SOME_UNINITED_DATA[1] += 1;
    }

    println!(
        "RTC_FAST function located at {:p}",
        function_in_rtc_ram as *const ()
    );
    println!("Result {}", function_in_rtc_ram());

    loop {
        function_in_ram();
        block!(timer0.wait()).unwrap();
    }
}

#[ram]
fn function_in_ram() {
    println!("Hello world!");
}

#[ram(rtc_fast)]
fn function_in_rtc_ram() -> u32 {
    42
}
