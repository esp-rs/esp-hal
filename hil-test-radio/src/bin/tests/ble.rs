//! BLE HIL tests.
//!
//! The support firmware is a BLE peripheral; this test binary acts as the
//! central device and validates that scanning and connection establishment work
//! against real peer hardware.

//% CHIP_FILTER(has_wifi_ble): esp32c6 || esp32s3
//% HARNESS-FIRMWARE(has_wifi_ble): ble_peripheral_support

//% FEATURES: unstable esp-alloc embassy
//% FEATURES(has_wifi_ble): esp-radio/ble esp-radio esp-radio-unstable

//% ENV: ESP_HAL_CONFIG_STACK_GUARD_OFFSET=4

#![no_std]
#![no_main]

use hil_test as _;

extern crate alloc;

const PERIPHERAL_ADDRESS: [u8; 6] = [0xff, 0x48, 0x49, 0x4c, 0x42, 0xff];
const BATTERY_SERVICE_UUID: u16 = 0x180f;
const BATTERY_LEVEL_UUID: u16 = 0x2a19;

fn init_heap() {
    cfg_select! {
        any(esp32, esp32s2, esp32s3, esp32c3, esp32c2, esp32c6) => {
            use esp_hal::ram;
            esp_alloc::heap_allocator!(#[ram(reclaimed)] size: 64 * 1024);
            esp_alloc::heap_allocator!(size: 36 * 1024);
        },
        any(esp32c5, esp32h2) => {
            esp_alloc::heap_allocator!(size: 72 * 1024);
        },
        _ => {},
    }
}

#[cfg(soc_has_bt)]
#[path = "ble/gatt.rs"]
mod ble_gatt;
