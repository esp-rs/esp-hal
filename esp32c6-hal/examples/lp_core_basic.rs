//! This shows a very basic example of running code on the LP core.
//!
//! Code on LP core increments a counter and continuously toggles GPIO1. The
//! current value is printed by the HP core.

#![no_std]
#![no_main]

use esp32c6_hal::{
    clock::ClockControl,
    gpio::lp_gpio::IntoLowPowerPin,
    lp_core,
    peripherals::Peripherals,
    prelude::*,
    timer::TimerGroup,
    Rtc,
    IO,
};
use esp_backtrace as _;
use esp_println::{print, println};

// 50000000 <_vector_table>:
// 50000000:       00000013                nop
// 50000004:       00000013                nop
// 50000008:       00000013                nop
// 5000000c:       00000013                nop
// 50000010:       00000013                nop
// 50000014:       00000013                nop
// 50000018:       00000013                nop
// 5000001c:       00000013                nop
// 50000020:       00000013                nop
// 50000024:       00000013                nop
// 50000028:       00000013                nop
// 5000002c:       00000013                nop
// 50000030:       00000013                nop
// 50000034:       00000013                nop
// 50000038:       00000013                nop
// 5000003c:       00000013                nop
// 50000040:       00000013                nop
// 50000044:       00000013                nop
// 50000048:       00000013                nop
// 5000004c:       00000013                nop
// 50000050:       00000013                nop
// 50000054:       00000013                nop
// 50000058:       00000013                nop
// 5000005c:       00000013                nop
// 50000060:       00000013                nop
// 50000064:       00000013                nop
// 50000068:       00000013                nop
// 5000006c:       00000013                nop
// 50000070:       00000013                nop
// 50000074:       00000013                nop
// 50000078:       00000013                nop
// 5000007c:       00000013                nop

// 50000080 <_start>:
// 50000080:       00000517                auipc   a0,0x0
// 50000084:       04050513                addi    a0,a0,64 # 500000c0 <data>
// 50000088:       4585                    li      a1,1
// 5000008a:       00b50023                sb      a1,0(a0)
// 5000008e:       600b26b7                lui     a3,0x600b2
// 50000092:       06a1                    addi    a3,a3,8
// 50000094:       600b2637                lui     a2,0x600b2
// 50000098:       0611                    addi    a2,a2,4
// 5000009a:       4709                    li      a4,2

// 5000009c <_loop>:
// 5000009c:       c218                    sw      a4,0(a2)
// 5000009e:       000f47b7                lui     a5,0xf4
// 500000a2:       24078793                addi    a5,a5,576 # f4240
// <_vector_table-0x4ff0bdc0>

// 500000a6 <_wait1>:
// 500000a6:       17fd                    addi    a5,a5,-1
// 500000a8:       fffd                    bnez    a5,500000a6 <_wait1>
// 500000aa:       0585                    addi    a1,a1,1
// 500000ac:       00b50023                sb      a1,0(a0)
// 500000b0:       c298                    sw      a4,0(a3)
// 500000b2:       000f47b7                lui     a5,0xf4
// 500000b6:       24078793                addi    a5,a5,576 # f4240
// <_vector_table-0x4ff0bdc0>

// 500000ba <_wait2>:
// 500000ba:       17fd                    addi    a5,a5,-1
// 500000bc:       fffd                    bnez    a5,500000ba <_wait2>
// 500000be:       bff9                    j       5000009c <_loop>

// 500000c0 <data>:
//         ...

const CODE: &[u8] = &[
    0x13, 0x00, 0x00, 0x00, 0x13, 0x00, 0x00, 0x00, 0x13, 0x00, 0x00, 0x00, 0x13, 0x00, 0x00, 0x00,
    0x13, 0x00, 0x00, 0x00, 0x13, 0x00, 0x00, 0x00, 0x13, 0x00, 0x00, 0x00, 0x13, 0x00, 0x00, 0x00,
    0x13, 0x00, 0x00, 0x00, 0x13, 0x00, 0x00, 0x00, 0x13, 0x00, 0x00, 0x00, 0x13, 0x00, 0x00, 0x00,
    0x13, 0x00, 0x00, 0x00, 0x13, 0x00, 0x00, 0x00, 0x13, 0x00, 0x00, 0x00, 0x13, 0x00, 0x00, 0x00,
    0x13, 0x00, 0x00, 0x00, 0x13, 0x00, 0x00, 0x00, 0x13, 0x00, 0x00, 0x00, 0x13, 0x00, 0x00, 0x00,
    0x13, 0x00, 0x00, 0x00, 0x13, 0x00, 0x00, 0x00, 0x13, 0x00, 0x00, 0x00, 0x13, 0x00, 0x00, 0x00,
    0x13, 0x00, 0x00, 0x00, 0x13, 0x00, 0x00, 0x00, 0x13, 0x00, 0x00, 0x00, 0x13, 0x00, 0x00, 0x00,
    0x13, 0x00, 0x00, 0x00, 0x13, 0x00, 0x00, 0x00, 0x13, 0x00, 0x00, 0x00, 0x13, 0x00, 0x00, 0x00,
    0x17, 0x05, 0x00, 0x00, 0x13, 0x05, 0x05, 0x04, 0x85, 0x45, 0x23, 0x00, 0xb5, 0x00, 0xb7, 0x26,
    0x0b, 0x60, 0xa1, 0x06, 0x37, 0x26, 0x0b, 0x60, 0x11, 0x06, 0x09, 0x47, 0x18, 0xc2, 0xb7, 0x47,
    0x0f, 0x00, 0x93, 0x87, 0x07, 0x24, 0xfd, 0x17, 0xfd, 0xff, 0x85, 0x05, 0x23, 0x00, 0xb5, 0x00,
    0x98, 0xc2, 0xb7, 0x47, 0x0f, 0x00, 0x93, 0x87, 0x07, 0x24, 0xfd, 0x17, 0xfd, 0xff, 0xf9, 0xbf,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00,
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

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    // configure GPIO 1 as LP output pin
    let mut lp_pin = io.pins.gpio1.into_low_power();
    lp_pin.output_enable(true);

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

    let data = (0x500000c0) as *mut u32;
    loop {
        print!("Current {:x}           \u{000d}", unsafe {
            data.read_volatile()
        });
    }
}
