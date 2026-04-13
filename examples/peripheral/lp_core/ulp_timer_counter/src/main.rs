//! Demonstrates running the ULP core on a roughly 1Hz timer.
//!
//! Code on ULP core increments a counter at a fixed memory address, every time it runs.
//! The current value is printed by the HP core.
//! When the user presses the GPIO0 button, it will fire an interrupt on the ULP core, and reset the
//! counter.
//!
//! ⚠️ Make sure to first compile the `esp-lp-hal/examples/interrupt_counter.rs` example ⚠️
//!
//! The following wiring is assumed:
//! - BUTTON => GPIO0

//% CHIPS: esp32s3,esp32s2

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    gpio::rtc_io::LowPowerInput,
    load_lp_code,
    main,
    ulp_core::{UlpCore, UlpCoreTimerCycles, UlpCoreWakeupSource},
};
use esp_println::{print, println};

esp_bootloader_esp_idf::esp_app_desc!();

// Roughly 1Hz on ESP32S3
#[cfg(feature = "esp32s3")]
const ULP_SLEEP_CYCLES: u32 = 530;
// Roughly 1Hz on ESP32S2
#[cfg(feature = "esp32s2")]
const ULP_SLEEP_CYCLES: u32 = 1060;

#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default());

    // configure GPIO0 as LP input pin, which will reset the counter
    let ulp_counter_reset_btn = LowPowerInput::new(peripherals.GPIO0);

    let mut lp_core = UlpCore::new(peripherals.ULP_RISCV_CORE);

    // load code to LP core
    let lp_core_code = load_lp_code!(
        "../../../../esp-lp-hal/target/riscv32imc-unknown-none-elf/release/examples/interrupt_counter"
    );

    // start LP core
    lp_core_code.run(
        &mut lp_core,
        UlpCoreWakeupSource::Timer(UlpCoreTimerCycles::new(ULP_SLEEP_CYCLES)),
        ulp_counter_reset_btn,
    );

    println!("ulp core run");

    const ADDR: usize = 0x5000_1000;

    let data = (ADDR) as *const u32;
    loop {
        print!("Current {:x}           \u{000d}", unsafe {
            data.read_volatile()
        });
    }
}
