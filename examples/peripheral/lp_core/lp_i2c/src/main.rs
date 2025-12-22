//! SHT30 humidity and temperature sensor example running on the LP core via LP I2C
//!
//! Code on LP core reads humidity and temperature from an SHT30 sensor via I2C.
//! The current value is printed by the HP core.
//!
//! ⚠️ Make sure to first compile the `esp-lp-hal/examples/i2c_sht30.rs` example ⚠️
//!
//! The following wiring is assumed:
//! - SDA => GPIO6
//! - SCL => GPIO7

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    gpio::lp_io::LowPowerOutputOpenDrain,
    i2c::lp_i2c::LpI2c,
    load_lp_code,
    lp_core::{LpCore, LpCoreWakeupSource},
    main,
    time::Rate,
};
use esp_println::{print, println};

esp_bootloader_esp_idf::esp_app_desc!();

// Shared-memory locations used for communication with the LP core firmware.
// The `esp-lp-hal` example `i2c_sht30.rs` running on the LP core writes the
// latest temperature and humidity readings as `f32` values to these addresses:
// - TEMP_ADDRESS: 4-byte `f32` at 0x5000_2000
// - HUMID_ADDRESS: 4-byte `f32` at 0x5000_2004 (immediately after temperature)
//
// These addresses must match the memory layout used by the LP core code; if you
// change them here, you must also update the corresponding addresses in the LP
// core example so both cores agree on the shared memory region.
const TEMP_ADDRESS: u32 = 0x5000_2000;
const HUMID_ADDRESS: u32 = 0x5000_2004;

#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default());

    // configure LP I2C
    let i2c = LpI2c::new(
        peripherals.LP_I2C0,
        LowPowerOutputOpenDrain::new(peripherals.GPIO6),
        LowPowerOutputOpenDrain::new(peripherals.GPIO7),
        Rate::from_khz(100),
    );

    let mut lp_core = LpCore::new(peripherals.LP_CORE);
    lp_core.stop();
    println!("lp core stopped");

    // load code to LP core
    let lp_core_code = load_lp_code!(
        "../../../../esp-lp-hal/target/riscv32imac-unknown-none-elf/release/examples/i2c_sht30"
    );

    // start LP core
    lp_core_code.run(&mut lp_core, LpCoreWakeupSource::HpCpu, i2c);
    println!("lp core run");

    let temp = TEMP_ADDRESS as *mut f32;
    let humid = HUMID_ADDRESS as *mut f32;
    loop {
        print!(
            "Current {:.2} C {:.2} %           \u{000d}",
            unsafe { temp.read_volatile() },
            unsafe { humid.read_volatile() }
        );
    }
}
