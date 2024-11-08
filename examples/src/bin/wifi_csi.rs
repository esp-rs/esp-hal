//! CSI Example
//!
//!
//! Set SSID and PASSWORD env variable before running this example.
//!

//% FEATURES: esp-wifi esp-wifi/wifi-default esp-wifi/wifi esp-wifi/utils esp-wifi/log
//% CHIPS: esp32 esp32s2 esp32s3 esp32c2 esp32c3 esp32c6

#![no_std]
#![no_main]

extern crate alloc;

use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    prelude::*,
    rng::Rng,
    time::{self},
    timer::timg::TimerGroup,
};
use esp_println::println;
use esp_wifi::{
    init,
    wifi::{
        utils::create_network_interface,
        AccessPointInfo,
        ClientConfiguration,
        Configuration,
        CsiConfig,
        WifiError,
        WifiStaDevice,
    },
    wifi_interface::WifiStack,
    // EspWifiInitFor,
};
use smoltcp::iface::SocketStorage;

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");

#[entry]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init({
        let mut config = esp_hal::Config::default();
        config.cpu_clock = CpuClock::max();
        config
    });

    esp_alloc::heap_allocator!(72 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let init = init(
        timg0.timer0,
        Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
    )
    .unwrap();

    let mut wifi = peripherals.WIFI;
    let mut socket_set_entries: [SocketStorage; 3] = Default::default();
    let (iface, device, mut controller, sockets) =
        create_network_interface(&init, &mut wifi, WifiStaDevice, &mut socket_set_entries).unwrap();
    let now = || time::now().duration_since_epoch().to_millis();
    let wifi_stack = WifiStack::new(iface, device, sockets, now);

    let client_config = Configuration::Client(ClientConfiguration {
        ssid: SSID.try_into().unwrap(),
        password: PASSWORD.try_into().unwrap(),
        ..Default::default()
    });
    let res = controller.set_configuration(&client_config);
    println!("wifi_set_configuration returned {:?}", res);

    controller.start().unwrap();
    println!("is wifi started: {:?}", controller.is_started());

    let csi = CsiConfig::default();
    controller
        .set_csi(csi, |data: esp_wifi::wifi::wifi_csi_info_t| {
            let rx_ctrl = data.rx_ctrl;
            // Signed bitfields are broken in rust-bingen, see https://github.com/esp-rs/esp-wifi-sys/issues/482
            let rssi = if rx_ctrl.rssi() > 127 {
                rx_ctrl.rssi() - 256
            } else {
                rx_ctrl.rssi()
            };
            println!("rssi: {:?} rate: {}", rssi, rx_ctrl.rate());
        })
        .unwrap();

    println!("Waiting for CSI data...");
    println!("Start Wifi Scan");
    let res: Result<(heapless::Vec<AccessPointInfo, 10>, usize), WifiError> = controller.scan_n();
    if let Ok((res, _count)) = res {
        for ap in res {
            println!("{:?}", ap);
        }
    }

    println!("{:?}", controller.get_capabilities());
    println!("wifi_connect {:?}", controller.connect());

    // wait to get connected
    println!("Wait to get connected");
    loop {
        match controller.is_connected() {
            Ok(true) => break,
            Ok(false) => {}
            Err(err) => {
                println!("{:?}", err);
                loop {}
            }
        }
    }
    println!("{:?}", controller.is_connected());

    // wait for getting an ip address
    println!("Wait to get an ip address");
    loop {
        wifi_stack.work();

        if wifi_stack.is_iface_up() {
            println!("got ip {:?}", wifi_stack.get_ip_info());
            break;
        }
    }

    println!("Start busy loop on main");
    loop {}
}
