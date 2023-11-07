#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_println::println;
use esp_wifi::esp_now::{PeerInfo, BROADCAST_ADDRESS};
use esp_wifi::{initialize, EspWifiInitFor};
#[path = "../../examples-util/util.rs"]
mod examples_util;
use examples_util::hal;
use hal::clock::ClockControl;
use hal::Rng;
use hal::{peripherals::Peripherals, prelude::*};

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
        EspWifiInitFor::Wifi,
        timer,
        Rng::new(peripherals.RNG),
        system.radio_clock_control,
        &clocks,
    )
    .unwrap();

    let wifi = peripherals.WIFI;
    let esp_now = esp_wifi::esp_now::EspNow::new(&init, wifi).unwrap();

    println!("esp-now version {}", esp_now.get_version().unwrap());

    #[cfg(not(feature = "esp32"))]
    println!("[RUN esp32 esp_now_broadcaster]");

    #[cfg(feature = "esp32")]
    println!("[RUN esp32c3 esp_now_broadcaster]");

    loop {
        let r = esp_now.receive();
        if let Some(r) = r {
            println!("Received {:?}", r);

            if r.info.dst_address == BROADCAST_ADDRESS {
                if !esp_now.peer_exists(&r.info.src_address) {
                    esp_now
                        .add_peer(PeerInfo {
                            peer_address: r.info.src_address,
                            lmk: None,
                            channel: None,
                            encrypt: false,
                        })
                        .unwrap();
                }
                println!("[PASSED]");
            }
        }
    }
}
