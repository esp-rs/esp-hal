//! This shows a very basic example of running code on the LP core.
//!
//! Code on LP core just increments a counter. The current value is printed by
//! the HP core.

#![no_std]
#![no_main]

use esp32c6_hal::{
    clock::ClockControl,
    peripherals::Peripherals,
    prelude::*,
    timer::TimerGroup,
    Rtc,
};
use esp_backtrace as _;
use esp_hal_common::lp_core;
use esp_println::println;

// 50000000 <_start>:
// 50000000:       00000517                auipc   a0,0x0
// 50000004:       01050513                addi    a0,a0,16 # 50000010 <data>
// 50000008:       4581                    li      a1,0
//
// 5000000a <_loop>:
// 5000000a:       0585                    addi    a1,a1,1
// 5000000c:       c10c                    sw      a1,0(a0)
// 5000000e:       bff5                    j       5000000a <_loop>
//
// 50000010 <data>:
// 50000010:       0000 0000

const CODE: &[u8] = &[
    0x17, 0x05, 0x00, 0x00, 0x13, 0x05, 0x05, 0x01, 0x81, 0x45, 0x85, 0x05, 0x0c, 0xc1, 0xf5, 0xbf,
    0x00, 0x00, 0x00, 0x00,
];

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let mut system = peripherals.PCR.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let mut rtc = Rtc::new(peripherals.LP_CLKRST);
    let timer_group0 = TimerGroup::new(
        peripherals.TIMG0,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(
        peripherals.TIMG1,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt1 = timer_group1.wdt;

    // Disable watchdog timers
    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    let mut lp_core = esp32c6_hal::lp_core::LpCore::new(peripherals.LP_CORE);
    lp_core.stop();
    println!("lp core stopped");

    // copy code to LP ram
    let lp_ram = 0x5000_0000 as *mut u8;
    unsafe {
        core::ptr::copy_nonoverlapping(CODE as *const _ as *const u8, lp_ram, CODE.len());
    }
    println!("copied code (len {})", CODE.len());

    // start LP core
    lp_core.run(lp_core::LpCoreWakeupSource::HpCpu);
    println!("lpcore run");

    let data = (0x5000_0010 - 0) as *mut u32;
    loop {
        println!("Current {}", unsafe { data.read_volatile() });
    }
}
