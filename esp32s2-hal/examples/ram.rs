//! This shows how to use RTC memory.
//! RTC memory is retained during resets and during most sleep modes.
//! Initialized memory is always re-initialized on startup.
//! Uninitialzed memory isn't initialized on startup and can be used to keep
//! data during resets. Zeroed memory is initialized to zero on startup.
//! We can also run code from RTC memory.

#![no_std]
#![no_main]

use core::fmt::Write;

use esp32s2_hal::{
    clock::ClockControl,
    macros::ram,
    pac::{Peripherals, UART0},
    prelude::*,
    timer::TimerGroup,
    Serial,
};
use nb::block;
use panic_halt as _;
use xtensa_lx_rt::entry;

#[ram(rtc_fast)]
static mut SOME_INITED_DATA: [u8; 2] = [0xaa, 0xbb];

#[ram(rtc_fast, uninitialized)]
static mut SOME_UNINITED_DATA: [u8; 2] = [0; 2];

#[ram(rtc_fast, zeroed)]
static mut SOME_ZEROED_DATA: [u8; 8] = [0; 8];

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut timer0 = timer_group0.timer0;
    let mut wdt = timer_group0.wdt;
    let mut serial0 = Serial::new(peripherals.UART0);

    // Disable MWDT flash boot protection
    wdt.disable();
    // The RWDT flash boot protection remains enabled and it being triggered is part
    // of the example

    timer0.start(1u64.secs());

    writeln!(
        serial0,
        "IRAM function located at {:p}",
        function_in_ram as *const ()
    )
    .unwrap();
    unsafe {
        writeln!(serial0, "SOME_INITED_DATA {:x?}", SOME_INITED_DATA).unwrap();
        writeln!(serial0, "SOME_UNINITED_DATA {:x?}", SOME_UNINITED_DATA).unwrap();
        writeln!(serial0, "SOME_ZEROED_DATA {:x?}", SOME_ZEROED_DATA).unwrap();

        SOME_INITED_DATA[0] = 0xff;
        SOME_ZEROED_DATA[0] = 0xff;

        writeln!(serial0, "SOME_INITED_DATA {:x?}", SOME_INITED_DATA).unwrap();
        writeln!(serial0, "SOME_UNINITED_DATA {:x?}", SOME_UNINITED_DATA).unwrap();
        writeln!(serial0, "SOME_ZEROED_DATA {:x?}", SOME_ZEROED_DATA).unwrap();

        if SOME_UNINITED_DATA[0] != 0 {
            SOME_UNINITED_DATA[0] = 0;
            SOME_UNINITED_DATA[1] = 0;
        }

        if SOME_UNINITED_DATA[1] == 0xff {
            SOME_UNINITED_DATA[1] = 0;
        }

        writeln!(serial0, "Counter {}", SOME_UNINITED_DATA[1]).unwrap();
        SOME_UNINITED_DATA[1] += 1;
    }

    writeln!(
        serial0,
        "RTC_FAST function located at {:p}",
        function_in_rtc_ram as *const ()
    )
    .unwrap();
    writeln!(serial0, "Result {}", function_in_rtc_ram()).unwrap();

    loop {
        function_in_ram(&mut serial0);
        block!(timer0.wait()).unwrap();
    }
}

#[ram]
fn function_in_ram(serial0: &mut Serial<UART0>) {
    writeln!(serial0, "Hello world!").unwrap();
}

#[ram(rtc_fast)]
fn function_in_rtc_ram() -> u32 {
    42
}
