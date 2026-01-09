//! Test we get an error when attempting to initialize esp-radio with interrupts
//! disabled in common ways

//% CHIPS(no_wifi): esp32h2
//% CHIPS(no_ble): esp32s2
//% CHIPS(has_wifi_ble): esp32 esp32c2 esp32c3 esp32c6 esp32s3

//% FEATURES: unstable esp-radio esp-alloc esp-radio/unstable embassy defmt
//% FEATURES(no_ble): esp-radio/wifi
//% FEATURES(no_wifi): esp-radio/ble
//% FEATURES(has_wifi_ble): esp-radio/wifi esp-radio/ble

#![no_std]
#![no_main]

#[cfg(multi_core)]
use esp_hal::system::Stack;
use hil_test as _;

extern crate alloc;

fn init_heap() {
    cfg_if::cfg_if! {
        if #[cfg(any(esp32, esp32s2, esp32s3, esp32c3, esp32c2, esp32c6))] {
            use esp_hal::ram;
            esp_alloc::heap_allocator!(#[ram(reclaimed)] size: 64 * 1024);
            esp_alloc::heap_allocator!(size: 36 * 1024);
        } else if #[cfg(esp32h2)] {
            esp_alloc::heap_allocator!(size: 72 * 1024);
        }
    }
}

#[cfg(multi_core)]
static mut APP_CORE_STACK: Stack<8192> = Stack::new();

#[path = "esp_radio/esp_rtos.rs"]
mod esp_rtos;

#[path = "esp_radio/init_tests.rs"]
mod init_tests;

#[cfg(soc_has_bt)]
#[path = "esp_radio/ble_controller.rs"]
mod ble_controller;

#[cfg(soc_has_wifi)]
#[path = "esp_radio/wifi_controller.rs"]
mod wifi_controller;

#[cfg(xtensa)]
#[path = "esp_radio/fpu.rs"]
mod fpu;
