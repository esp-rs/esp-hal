//! This shows a very basic example of running code on the LP core.
//!
//! Code on LP core increments a counter and continuously toggles LED. The
//! current value is printed by the HP core.
//!
//! ⚠️ Make sure to first compile the `esp-lp-hal/examples/blinky.rs` example ⚠️
//!
//! The following wiring is assumed:
//! - LED => GPIO1

//% CHIP_FILTER: ulp_riscv_driver_supported

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{gpio::lp_io::LowPowerOutput, load_lp_code, main};
cfg_select! {
    any(feature = "esp32s2", feature = "esp32s3") => {
        use esp_hal::lp_core::{UlpCore, UlpCoreWakeupSource};
    }
    _ => {
        use esp_hal::lp_core::{LpCore, LpCoreWakeupSource};
    }
}
use esp_println::{print, println};

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default());

    // configure GPIO 1 as LP output pin
    let lp_pin = LowPowerOutput::new(peripherals.GPIO1);

    let mut lp_core = cfg_select! {
        any(feature = "esp32s2", feature = "esp32s3") => UlpCore::new(peripherals.ULP_RISCV_CORE),
        _ => LpCore::new(peripherals.LP_CORE),
    };

    #[cfg(not(feature = "esp32s2"))]
    {
        lp_core.stop(); // currently not implemented for ESP32-S2.
        println!("ulp core stopped");
    }

    // load code to LP core
    let lp_core_code = cfg_select! {
        any(feature = "esp32s2", feature = "esp32s3") => load_lp_code!(
            "../../../../esp-lp-hal/target/riscv32imc-unknown-none-elf/release/examples/blinky"
        ),
        _ => load_lp_code!(
            "../../../../esp-lp-hal/target/riscv32imac-unknown-none-elf/release/examples/blinky"
        ),
    };

    // start LP core
    let wakeup_source = cfg_select! {
        any(feature = "esp32s2", feature = "esp32s3") => UlpCoreWakeupSource::HpCpu,
        _ => LpCoreWakeupSource::HpCpu,
    };
    lp_core_code.run(&mut lp_core, wakeup_source, lp_pin);

    println!("lp core run");

    const ADDR: usize = cfg_select! {
        any(feature = "esp32s2", feature = "esp32s3") => 0x5000_0400,
        _ => 0x5000_2000,
    };

    let data = (ADDR) as *const u32;
    loop {
        print!("Current {:x}           \u{000d}", unsafe {
            data.read_volatile()
        });
    }
}
