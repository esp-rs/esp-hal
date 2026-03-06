//! This shows a very basic example of running code on the LP core.
//!
//! Code on LP core increments a counter and continuously toggles LED. The
//! current value is printed by the HP core.
//!
//! ⚠️ Make sure to first compile the `esp-lp-hal/examples/blinky.rs` example ⚠️
//!
//! The following wiring is assumed:
//! - LED => GPIO1

#![no_std]
#![no_main]

use esp_backtrace as _;
#[cfg(feature = "esp32c6")]
use esp_hal::{
    gpio::lp_io::LowPowerOutput,
    lp_core::{LpCore, LpCoreWakeupSource},
};
#[cfg(any(feature = "esp32s2", feature = "esp32s3"))]
use esp_hal::{
    gpio::rtc_io::LowPowerOutput,
    ulp_core::{UlpCore, UlpCoreWakeupSource},
};
use esp_hal::{load_lp_code, main};
use esp_println::{print, println};

esp_bootloader_esp_idf::esp_app_desc!();

#[cfg(feature = "esp32c6")]
#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default());

    // configure GPIO 1 as LP output pin

    let lp_pin = LowPowerOutput::new(peripherals.GPIO1);

    let mut lp_core = LpCore::new(peripherals.LP_CORE);
    lp_core.stop();
    println!("lp core stopped");

    // load code to LP core
    let lp_core_code = load_lp_code!(
        "../../../../esp-lp-hal/target/riscv32imac-unknown-none-elf/release/examples/blinky"
    );

    // start LP core
    lp_core_code.run(&mut lp_core, LpCoreWakeupSource::HpCpu, lp_pin);
    println!("lp core run");

    let data = (0x5000_2000) as *const u32;
    loop {
        print!("Current {:x}           \u{000d}", unsafe {
            data.read_volatile()
        });
    }
}

#[cfg(any(feature = "esp32s2", feature = "esp32s3"))]
#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default());

    // configure GPIO 1 as LP output pin
    let rtc_pin = LowPowerOutput::new(peripherals.GPIO1);

    let mut ulp_core = UlpCore::new(peripherals.ULP_RISCV_CORE);
    #[cfg(not(feature = "esp32s2"))]
    {
        ulp_core.stop(); // currently not implemented for ESP32-S2.
        println!("ulp core stopped");
    }

    // load code to ULP coprocessor
    let ulp_core_code = load_lp_code!(
        "../../../../esp-lp-hal/target/riscv32imc-unknown-none-elf/release/examples/blinky"
    );

    // start ULP coprocessor
    ulp_core_code.run(&mut ulp_core, UlpCoreWakeupSource::HpCpu, rtc_pin);
    println!("ulp core run");

    let data = (0x5000_0400) as *const u32;
    loop {
        print!("Current {:x}           \u{000d}", unsafe {
            data.read_volatile()
        });
    }
}
