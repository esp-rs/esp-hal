#![no_std]
#![no_main]

use core::fmt::Write;

use esp32c3_hal::{efuse::Efuse, pac::Peripherals, prelude::*, RtcCntl, Serial, Timer};
use panic_halt as _;
use riscv_rt::entry;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();

    let mut rtc_cntl = RtcCntl::new(peripherals.RTC_CNTL);
    let mut serial0 = Serial::new(peripherals.UART0).unwrap();
    let mut timer0 = Timer::new(peripherals.TIMG0);
    let mut timer1 = Timer::new(peripherals.TIMG1);

    // Disable watchdog timers
    rtc_cntl.set_super_wdt_enable(false);
    rtc_cntl.set_wdt_enable(false);
    timer0.disable();
    timer1.disable();

    writeln!(serial0, "MAC address {:02x?}", Efuse::get_mac_address()).unwrap();
    writeln!(
        serial0,
        "Flash Encryption {:?}",
        Efuse::get_flash_encryption()
    )
    .unwrap();

    loop {}
}
