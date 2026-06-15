//! Regression test for `software_reset()`.
//!
//! On ESP32-C5/C61 a ROM bug leaves the MSPI AXI bus frozen after a core
//! reset, causing the ROM bootloader to hang when reading flash. This test
//! verifies that the chip resets cleanly and boots again after a software
//! reset.
//!
//! Issue: <https://github.com/esp-rs/esp-hal/issues/5703>

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    main,
    rtc_cntl::{SocResetReason, reset_reason},
    system::{Cpu, software_reset},
    time::Duration,
};
use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    let _peripherals = esp_hal::init(esp_hal::Config::default());
    let delay = Delay::new();

    let reason = reset_reason(Cpu::ProCpu).unwrap_or(SocResetReason::ChipPowerOn);
    println!("reset reason: {:?}", reason);

    match reason {
        SocResetReason::ChipPowerOn => {
            println!("calling software_reset()...");
        }
        // CoreUsbUart is produced when flashing via USB; treat as a fresh start.
        #[cfg(not(any(feature = "esp32", feature = "esp32s2", feature = "esp32c2")))]
        SocResetReason::CoreUsbUart => {
            println!("calling software_reset()...");
        }
        SocResetReason::CoreSw => {
            println!("PASS");
        }
        other => {
            panic!("software_reset() failed; expected CoreSw, got {:?}", other);
        }
    }

    loop {
        delay.delay(Duration::from_secs(1));
        software_reset();
    }
}
