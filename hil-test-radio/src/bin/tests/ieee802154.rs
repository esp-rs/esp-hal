//! IEEE 802.15.4 HIL tests.
//!
//! Each radio domain gets its own entry binary under `src/bin/tests/`
//! (`wifi`, `ble`, and now `ieee802154`), so domains stay independent instead
//! of everything being folded into a single binary.
//!
//! The support firmware (`ieee802154_echo_support`) runs on a second board and
//! echoes back every frame it receives, so this binary can exercise both the
//! transmit/ACK and the receive paths against real peer hardware.

//% CHIP_FILTER(has_ieee802154): esp32c6
//% HARNESS-FIRMWARE(has_ieee802154): ieee802154_echo_support

//% FEATURES: unstable esp-alloc embassy
//% FEATURES(has_ieee802154): esp-radio/ieee802154 esp-radio esp-radio-unstable

//% ENV: ESP_HAL_CONFIG_STACK_GUARD_OFFSET=4

#![no_std]
#![no_main]

use hil_test as _;

extern crate alloc;

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

#[cfg(soc_has_ieee802154)]
#[path = "ieee802154/loopback.rs"]
mod ieee802154_loopback;
