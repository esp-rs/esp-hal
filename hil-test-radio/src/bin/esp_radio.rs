//% CHIPS(no_wifi): esp32h2
//% CHIPS(no_ble): esp32s2
//% CHIPS(has_wifi_ble): esp32 esp32c2 esp32c3 esp32c6 esp32s3 esp32c5

//% FEATURES: unstable esp-alloc embassy
//% FEATURES(has_wifi_ble): esp-radio/wifi esp-radio/ble  esp-radio esp-radio/unstable
//% FEATURES(has_wifi_ble): esp-radio/defmt defmt esp-radio/csi

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
        if #[cfg(any(esp32, esp32s2, esp32s3, esp32c3, esp32c2, esp32c6))] {
            use esp_hal::ram;
            esp_alloc::heap_allocator!(#[ram(reclaimed)] size: 64 * 1024);
            esp_alloc::heap_allocator!(size: 36 * 1024);
        } else if #[cfg(any(esp32c5, esp32h2))] {
            esp_alloc::heap_allocator!(size: 72 * 1024);
        }
    }
}

#[cfg(multi_core)]
static mut APP_CORE_STACK: Stack<8192> = Stack::new();

#[cfg(soc_has_wifi)]
#[path = "esp_radio/wifi_dhcp.rs"]
mod wifi_dhcp;

#[cfg(soc_has_wifi)]
#[path = "esp_radio/wifi_ap.rs"]
mod wifi_ap;
