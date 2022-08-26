//! This shows how to write text to serial0.
//! You can see the output with `espflash` if you provide the `--monitor` option

#![no_std]
#![no_main]

use esp32c2_hal as _;
use esp_backtrace as _;
use riscv_rt::entry;

#[entry]
fn main() -> ! {
    loop {}
}
