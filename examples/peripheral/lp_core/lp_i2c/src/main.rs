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

//% CHIP_FILTER: ulp_riscv_driver_supported && lp_i2c_master_version == "lp_i2c"

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    i2c::lp_i2c::{Config, LpI2c},
    load_lp_code,
    lp_core::{LpCore, LpCoreWakeupSource},
    main,
    time::Rate,
};
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
    let peripherals = esp_hal::init(esp_hal::Config::default());

    // Log on UART0's default TX pin.
    let log_tx = peripherals.GPIO16;
    let log_writer = esp_hal::uart::UartTx::new(peripherals.UART0, Default::default())
        .unwrap()
        .with_tx(log_tx);
    static LOG_WRITER: static_cell::StaticCell<esp_hal::uart::UartTx<'static, esp_hal::Blocking>> =
        static_cell::StaticCell::new();
    embedded_log::init(LOG_WRITER.init(log_writer));

    // configure LP I2C
    let i2c = LpI2c::new(
        peripherals.LP_I2C0,
        Config::default().with_frequency(Rate::from_khz(100)),
        peripherals.GPIO6,
        peripherals.GPIO7,
    )
    .unwrap();

    let mut lp_core = LpCore::new(peripherals.LP_CORE);
    lp_core.stop();
    log::info!("lp core stopped");

    // load code to LP core
    let lp_core_code = load_lp_code!(
        "../../../../esp-lp-hal/target/riscv32imac-unknown-none-elf/release/examples/i2c_sht30"
    );

    // start LP core
    lp_core_code.run(&mut lp_core, LpCoreWakeupSource::HpCpu, i2c);
    log::info!("lp core run");

    let temp = TEMP_ADDRESS as *mut f32;
    let humid = HUMID_ADDRESS as *mut f32;
    let mut last = None;
    loop {
        let current = unsafe { (temp.read_volatile(), humid.read_volatile()) };
        if last != Some(current) {
            log::info!("Current {:.2} C {:.2} %", current.0, current.1);
            last = Some(current);
        }
    }
}
