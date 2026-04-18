//! Demonstrates running the ULP core being woken up from GPIO0 button press.
//!
//! Code on ULP core increments a counter at a fixed memory address, every time it runs.
//! The current value is printed by the HP core.
//! When the user presses the GPIO0 button, the ULP core will be woken up, incrementing the counter.
//!
//! ⚠️ Make sure to first compile the `esp-lp-hal/examples/gpio_wakeup.rs` example ⚠️
//!
//! The following wiring is assumed:
//! - BUTTON => GPIO0

//% CHIPS: esp32s3 esp32s2

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    gpio::{WakeEvent, rtc_io::LowPowerInput},
    load_lp_code,
    main,
    ulp_core::{UlpCore, UlpCoreWakeupSource},
};
use esp_println::{print, println};

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default());

    // configure GPIO0 as LP input pin, and enable wakeup on it.
    let ulp_wakeup_btn = LowPowerInput::new(peripherals.GPIO0);
    ulp_wakeup_btn.wakeup_enable(Some(WakeEvent::LowLevel));

    let mut lp_core = UlpCore::new(peripherals.ULP_RISCV_CORE);

    // load code to LP core
    let lp_core_code = load_lp_code!(
        "../../../../esp-lp-hal/target/riscv32imc-unknown-none-elf/release/examples/gpio_wakeup"
    );

    // start LP core
    lp_core_code.run(&mut lp_core, UlpCoreWakeupSource::Gpio, ulp_wakeup_btn);

    println!("ulp core run");

    const ADDR: usize = 0x5000_1000;

    let data = (ADDR) as *const u32;
    loop {
        print!("Current {:x}           \u{000d}", unsafe {
            data.read_volatile()
        });
    }
}
