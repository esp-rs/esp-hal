//! This shows a very basic example of running code on the ULP RISCV core.
//!
//! Code on ULP core just increments a counter. The current value is printed by
//! the HP core.

#![no_std]
#![no_main]

use esp32s2_hal::{
    clock::ClockControl,
    peripherals::Peripherals,
    prelude::*,
    timer::TimerGroup,
    Rtc,
};
use esp_backtrace as _;
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
    let mut system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let timer_group0 = TimerGroup::new(
        peripherals.TIMG0,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt = timer_group0.wdt;
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);

    // Disable MWDT and RWDT (Watchdog) flash boot protection
    wdt.disable();
    rtc.rwdt.disable();

    let mut ulp_core = esp32s2_hal::ulp_core::UlpCore::new(peripherals.ULP_RISCV_CORE);

    // copy code to RTC ram
    let lp_ram = 0x5000_0000 as *mut u8;
    unsafe {
        core::ptr::copy_nonoverlapping(CODE as *const _ as *const u8, lp_ram, CODE.len());
    }
    println!("copied code (len {})", CODE.len());

    // start ULP core
    ulp_core.run(esp32s2_hal::ulp_core::UlpCoreWakeupSource::HpCpu);
    println!("ulpcore run");

    let data = (0x5000_0010 - 0) as *mut u32;
    loop {
        println!("Current {}", unsafe { data.read_volatile() });
    }
}
