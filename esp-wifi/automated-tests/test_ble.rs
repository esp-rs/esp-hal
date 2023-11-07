#![no_std]
#![no_main]

use core::cell::RefCell;

use bleps::{
    ad_structure::{
        create_advertising_data, AdStructure, BR_EDR_NOT_SUPPORTED, LE_GENERAL_DISCOVERABLE,
    },
    attribute_server::{AttributeServer, WorkResult},
    gatt, Ble, HciConnector,
};
use esp_backtrace as _;
use esp_println::println;
use esp_wifi::{ble::controller::BleConnector, initialize, EspWifiInitFor};
use examples_util::hal;
use hal::{clock::ClockControl, peripherals::*, prelude::*, Rng, IO};
#[path = "../../examples-util/util.rs"]
mod examples_util;

#[entry]
fn main() -> ! {
    #[cfg(feature = "log")]
    esp_println::logger::init_logger(log::LevelFilter::Info);

    let peripherals = Peripherals::take();

    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::max(system.clock_control).freeze();

    #[cfg(target_arch = "xtensa")]
    let timer = hal::timer::TimerGroup::new(peripherals.TIMG1, &clocks).timer0;
    #[cfg(target_arch = "riscv32")]
    let timer = hal::systimer::SystemTimer::new(peripherals.SYSTIMER).alarm0;
    let init = initialize(
        EspWifiInitFor::Ble,
        timer,
        Rng::new(peripherals.RNG),
        system.radio_clock_control,
        &clocks,
    )
    .unwrap();

    let mut bluetooth = peripherals.BT;

    loop {
        let connector = BleConnector::new(&init, &mut bluetooth);
        let hci = HciConnector::new(connector, esp_wifi::current_millis);
        let mut ble = Ble::new(&hci);

        println!("{:?}", ble.init());
        println!("{:?}", ble.cmd_set_le_advertising_parameters());
        println!(
            "{:?}",
            ble.cmd_set_le_advertising_data(
                create_advertising_data(&[
                    AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
                    AdStructure::ServiceUuids16(&[Uuid::Uuid16(0x1809)]),
                    AdStructure::CompleteLocalName("ESP-WIFI"),
                ])
                .unwrap()
            )
        );
        println!("{:?}", ble.cmd_set_le_advertise_enable(true));
        println!("started advertising");

        println!("[HOST bletool write ESP-WIFI 937312E0-2354-11EB-9F10-FBC30A62CF38 937312E0-2354-11EB-9F10-FBC30A62CF38 Hello]");

        let rcv_buffer = RefCell::new([0u8; 32]);
        let mut rf = |_offset: usize, data: &mut [u8]| {
            data[..20].copy_from_slice(&b"Hello Bare-Metal BLE"[..]);
            17
        };
        let mut wf = |offset: usize, data: &[u8]| {
            println!("RECEIVED: {} {:?}", offset, data);
            rcv_buffer.borrow_mut()[..data.len()].copy_from_slice(data);
        };

        gatt!([service {
            uuid: "937312e0-2354-11eb-9f10-fbc30a62cf38",
            characteristics: [characteristic {
                uuid: "937312e0-2354-11eb-9f10-fbc30a62cf38",
                read: rf,
                write: wf,
            },],
        },]);

        let mut srv = AttributeServer::new(&mut ble, &mut gatt_attributes);

        loop {
            match srv.do_work_with_notification(None) {
                Ok(res) => {
                    if let WorkResult::GotDisconnected = res {
                        break;
                    }
                }
                Err(err) => {
                    println!("{:?}", err);
                }
            }

            if &rcv_buffer.borrow()[..5] == b"Hello" {
                println!("[PASSED]");
                loop {}
            }
        }
    }
}
