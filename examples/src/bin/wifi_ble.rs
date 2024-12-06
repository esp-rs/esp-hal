//! BLE Example
//!
//! - starts Bluetooth advertising
//! - offers one service with three characteristics (one is read/write, one is write only, one is read/write/notify)
//! - pressing the boot-button on a dev-board will send a notification if it is subscribed

//% FEATURES: esp-wifi esp-wifi/ble esp-hal/unstable
//% CHIPS: esp32 esp32s3 esp32c2 esp32c3 esp32c6 esp32h2

#![no_std]
#![no_main]

use bleps::{
    ad_structure::{
        create_advertising_data,
        AdStructure,
        BR_EDR_NOT_SUPPORTED,
        LE_GENERAL_DISCOVERABLE,
    },
    attribute_server::{AttributeServer, NotificationData, WorkResult},
    gatt,
    Ble,
    HciConnector,
};
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    gpio::{Input, Pull},
    prelude::*,
    rng::Rng,
    time,
    timer::timg::TimerGroup,
};
use esp_println::println;
use esp_wifi::{ble::controller::BleConnector, init};

#[entry]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(72 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);

    let init = init(
        timg0.timer0,
        Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
    )
    .unwrap();

    cfg_if::cfg_if! {
        if #[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))] {
            let button = Input::new(peripherals.GPIO0, Pull::Down);
        } else {
            let button = Input::new(peripherals.GPIO9, Pull::Down);
        }
    }

    let mut debounce_cnt = 500;

    let mut bluetooth = peripherals.BT;

    let now = || time::now().duration_since_epoch().to_millis();
    loop {
        let connector = BleConnector::new(&init, &mut bluetooth);
        let hci = HciConnector::new(connector, now);
        let mut ble = Ble::new(&hci);

        println!("{:?}", ble.init());
        println!("{:?}", ble.cmd_set_le_advertising_parameters());
        println!(
            "{:?}",
            ble.cmd_set_le_advertising_data(
                create_advertising_data(&[
                    AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
                    AdStructure::ServiceUuids16(&[Uuid::Uuid16(0x1809)]),
                    AdStructure::CompleteLocalName(esp_hal::chip!()),
                ])
                .unwrap()
            )
        );
        println!("{:?}", ble.cmd_set_le_advertise_enable(true));

        println!("started advertising");

        let mut rf = |_offset: usize, data: &mut [u8]| {
            data[..20].copy_from_slice(&b"Hello Bare-Metal BLE"[..]);
            17
        };
        let mut wf = |offset: usize, data: &[u8]| {
            println!("RECEIVED: {} {:?}", offset, data);
        };

        let mut wf2 = |offset: usize, data: &[u8]| {
            println!("RECEIVED: {} {:?}", offset, data);
        };

        let mut rf3 = |_offset: usize, data: &mut [u8]| {
            data[..5].copy_from_slice(&b"Hola!"[..]);
            5
        };
        let mut wf3 = |offset: usize, data: &[u8]| {
            println!("RECEIVED: Offset {}, data {:?}", offset, data);
        };

        gatt!([service {
            uuid: "937312e0-2354-11eb-9f10-fbc30a62cf38",
            characteristics: [
                characteristic {
                    uuid: "937312e0-2354-11eb-9f10-fbc30a62cf38",
                    read: rf,
                    write: wf,
                },
                characteristic {
                    uuid: "957312e0-2354-11eb-9f10-fbc30a62cf38",
                    write: wf2,
                },
                characteristic {
                    name: "my_characteristic",
                    uuid: "987312e0-2354-11eb-9f10-fbc30a62cf38",
                    notify: true,
                    read: rf3,
                    write: wf3,
                },
            ],
        },]);

        let mut rng = bleps::no_rng::NoRng;
        let mut srv = AttributeServer::new(&mut ble, &mut gatt_attributes, &mut rng);

        loop {
            let mut notification = None;

            if button.is_low() && debounce_cnt > 0 {
                debounce_cnt -= 1;
                if debounce_cnt == 0 {
                    let mut cccd = [0u8; 1];
                    if let Some(1) = srv.get_characteristic_value(
                        my_characteristic_notify_enable_handle,
                        0,
                        &mut cccd,
                    ) {
                        // if notifications enabled
                        if cccd[0] == 1 {
                            notification = Some(NotificationData::new(
                                my_characteristic_handle,
                                &b"Notification"[..],
                            ));
                        }
                    }
                }
            };

            if button.is_high() {
                debounce_cnt = 500;
            }

            match srv.do_work_with_notification(notification) {
                Ok(res) => {
                    if let WorkResult::GotDisconnected = res {
                        break;
                    }
                }
                Err(err) => {
                    println!("{:?}", err);
                }
            }
        }
    }
}
