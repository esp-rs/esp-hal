#![no_std]
#![cfg_attr(
    any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"),
    feature(asm_experimental_arch)
)]
#![doc(html_logo_url = "https://avatars.githubusercontent.com/u/46717278")]

use core::cell::Cell;

#[cfg(any(
    feature = "executor-thread",
    feature = "time-systimer",
    feature = "time-timg0"
))]
use esp_hal_common::{clock::Clocks, peripherals, trapframe};
pub use procmacros::main;

// MUST be the first module
mod fmt;

pub mod executor;
mod time_driver;

/// TODO: document me
#[cfg(any(feature = "time-systimer", feature = "time-timg0"))]
pub fn init(clocks: &Clocks, td: time_driver::TimerType) {
    self::time_driver::EmbassyTimer::init(clocks, td)
}

/// TODO: document me
pub struct AlarmState {
    pub callback: Cell<Option<(fn(*mut ()), *mut ())>>,
    pub allocated: Cell<bool>,
}

unsafe impl Send for AlarmState {}

impl AlarmState {
    /// TODO: document me
    pub const fn new() -> Self {
        Self {
            callback: Cell::new(None),
            allocated: Cell::new(false),
        }
    }
}
