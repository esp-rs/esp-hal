//% CHIPS(no_wifi): esp32h2
//% CHIPS(no_ble): esp32s2
//% CHIPS(no_radio): esp32p4
//% CHIPS(has_wifi_ble): esp32 esp32c2 esp32c3 esp32c6 esp32s3 esp32c5
//% CHIPS(stable_wifi): esp32 esp32c2 esp32c3 esp32c5 esp32c6 esp32s2 esp32s3

//% FEATURES: unstable esp-alloc embassy
//% FEATURES(no_radio): rtos-radio-driver
//% FEATURES(no_ble): esp-radio/wifi esp-radio esp-radio-unstable
//% FEATURES(no_wifi): esp-radio/ble esp-radio esp-radio-unstable trouble-host
//% FEATURES(has_wifi_ble): esp-radio/wifi esp-radio/ble esp-radio/coex esp-radio-unstable
//% FEATURES(has_wifi_ble): trouble-host
//% FEATURES(stable_wifi): esp-radio/wifi esp-radio esp-radio/defmt

// Even if the defaults change, keep this at a low-ish value for
// the esp_rtos/moving_data_to_second_core test
//% ENV: ESP_HAL_CONFIG_STACK_GUARD_OFFSET=4

#![no_std]
#![no_main]

#[cfg(multi_core)]
use esp_hal::system::Stack;
use hil_test as _;

extern crate alloc;

fn init_heap() {
    cfg_if::cfg_if! {
        if #[cfg(any(esp32, esp32s2, esp32s3, esp32c3, esp32c2, esp32c5, esp32c6, esp32c61, esp32p4))] {
            use esp_hal::ram;
            esp_alloc::heap_allocator!(#[ram(reclaimed)] size: 64 * 1024);
            esp_alloc::heap_allocator!(size: 48 * 1024);
        } else if #[cfg(esp32h2)] {
            esp_alloc::heap_allocator!(size: 72 * 1024);
        }
    }
}

#[cfg(multi_core)]
static mut APP_CORE_STACK: Stack<8192> = Stack::new();

#[path = "radio_basic/esp_rtos.rs"]
mod esp_rtos;

#[path = "radio_basic/init_tests.rs"]
#[cfg(feature = "esp-radio")]
mod init_tests;

#[cfg(bt_driver_supported)]
#[path = "radio_basic/ble_controller.rs"]
#[cfg(feature = "esp-radio-unstable")]
mod ble_controller;

#[cfg(soc_has_wifi)]
#[path = "radio_basic/wifi_controller.rs"]
#[cfg(feature = "esp-radio")]
mod wifi_controller;

#[cfg(xtensa)]
#[path = "radio_basic/fpu.rs"]
mod fpu;
