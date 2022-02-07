#![no_std]
#![no_main]

use core::fmt::Write;

use esp32s2_hal::{
    pac::{Peripherals, UART0},
    prelude::*,
    ram,
    Serial,
    Timer,
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

    let mut timer0 = Timer::new(peripherals.TIMG0);
    let mut serial0 = Serial::new(peripherals.UART0).unwrap();

    // Disable watchdog timer
    timer0.disable();

    timer0.start(10_000_000u64);

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
