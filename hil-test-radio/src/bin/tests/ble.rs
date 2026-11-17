//! BLE HIL tests.
//!
//! The support firmware is a BLE peripheral; this test binary acts as the
//! central device and validates that scanning and connection establishment work
//! against real peer hardware.
//!
//! On chips with an IEEE 802.15.4 radio, it also checks that BLE still works when
//! the 802.15.4 driver was brought up first.
//!
//! It also checks that L2CAP data arrives intact when the controller splits ACL
//! packets over a chain of mbufs.

//% CHIP_FILTER(has_wifi_ble): esp32c6 || esp32s3
//% HARNESS-FIRMWARE(has_wifi_ble): ble_peripheral_support

//% FEATURES: unstable esp-alloc embassy
//% FEATURES(has_wifi_ble): esp-radio/ble esp-radio esp-radio-unstable
//% FEATURES-IF(soc_has_ieee802154): esp-radio/ieee802154

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
        }
        any(esp32c5, esp32h2) => {
            esp_alloc::heap_allocator!(size: 72 * 1024);
        }
        _ => {}
    }
}

#[cfg(soc_has_bt)]
async fn connect<'d, C: trouble_host::Controller>(
    central: &mut trouble_host::central::Central<'d, C, trouble_host::prelude::DefaultPacketPool>,
    address: trouble_host::Address,
) -> trouble_host::connection::Connection<'d, trouble_host::prelude::DefaultPacketPool> {
    use embassy_time::{Duration, with_timeout};
    use trouble_host::prelude::*;

    let accept_list = [address];
    let connect_config = ConnectConfig {
        scan_config: ScanConfig {
            filter_accept_list: &accept_list,
            active: true,
            phys: PhySet::M1,
            interval: Duration::from_millis(100),
            window: Duration::from_millis(100),
            timeout: Duration::from_secs(10),
            ..Default::default()
        },
        connect_params: RequestedConnParams::default(),
    };

    let conn = with_timeout(Duration::from_secs(15), central.connect(&connect_config))
        .await
        .expect("BLE connect timed out")
        .expect("BLE connect failed");

    assert_eq!(conn.peer_address(), address);
    assert!(conn.is_connected());
    conn
}

#[cfg(soc_has_bt)]
async fn ble_task<C: trouble_host::Controller, P: trouble_host::PacketPool>(
    mut runner: trouble_host::prelude::Runner<'_, C, P>,
) {
    loop {
        if runner.run().await.is_err() {
            embassy_time::Timer::after(embassy_time::Duration::from_millis(100)).await;
        }
    }
}

#[cfg(soc_has_bt)]
#[path = "ble/gatt.rs"]
mod ble_gatt;

#[cfg(soc_has_bt)]
#[path = "ble/l2cap.rs"]
mod ble_l2cap;
