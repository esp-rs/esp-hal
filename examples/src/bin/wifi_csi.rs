//! CSI Example
//!
//!
//! Set SSID and PASSWORD env variable before running this example.
//!

//% FEATURES: esp-wifi esp-wifi/wifi esp-wifi/utils esp-wifi/log
//% CHIPS: esp32 esp32s2 esp32s3 esp32c2 esp32c3 esp32c6

#![no_std]
#![no_main]

extern crate alloc;

use blocking_network_stack::Stack;
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
};
use smoltcp::{
    iface::{SocketSet, SocketStorage},
    wire::DhcpOption,
};

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

    let mut rng = Rng::new(peripherals.RNG);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let init = init(timg0.timer0, rng.clone(), peripherals.RADIO_CLK).unwrap();

    let mut wifi = peripherals.WIFI;
    let (iface, device, mut controller) =
        create_network_interface(&init, &mut wifi, WifiStaDevice).unwrap();

    let mut socket_set_entries: [SocketStorage; 3] = Default::default();
    let mut socket_set = SocketSet::new(&mut socket_set_entries[..]);
    let mut dhcp_socket = smoltcp::socket::dhcpv4::Socket::new();
    // we can set a hostname here (or add other DHCP options)
    dhcp_socket.set_outgoing_options(&[DhcpOption {
        kind: 12,
        data: b"esp-wifi",
    }]);
    socket_set.add(dhcp_socket);

    let now = || time::now().duration_since_epoch().to_millis();
    let stack = Stack::new(iface, device, socket_set, now, rng.random());

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
        stack.work();

        if stack.is_iface_up() {
            println!("got ip {:?}", stack.get_ip_info());
            break;
        }
    }

    println!("Start busy loop on main");
    loop {}
}
