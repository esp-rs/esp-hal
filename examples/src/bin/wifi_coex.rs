//! WiFi and BLE COEXistence example
//!
//! - set SSID and PASSWORD env variable
//! - gets an ip address via DHCP
//! - performs an HTTP get request to some "random" server
//! - does BLE advertising (you cannot connect to it - it's just not implemented in the example)
//!
//! Note: On ESP32-C2 and ESP32-C3 you need a wifi-heap size of 70000, on ESP32-C6 you need 80000 and a tx_queue_size of 10
//!

//% FEATURES: esp-wifi esp-wifi/wifi esp-wifi/smoltcp esp-wifi/ble esp-wifi/coex esp-hal/unstable
//% CHIPS: esp32 esp32s3 esp32c2 esp32c3 esp32c6

#![no_std]
#![no_main]

use core::net::Ipv4Addr;

use bleps::{
    Ble,
    HciConnector,
    ad_structure::{
        AdStructure,
        BR_EDR_NOT_SUPPORTED,
        LE_GENERAL_DISCOVERABLE,
        create_advertising_data,
    },
    att::Uuid,
};
use blocking_network_stack::Stack;
use embedded_io::*;
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    main,
    rng::Rng,
    time::{self, Duration},
    timer::timg::TimerGroup,
};
use esp_println::{print, println};
use esp_wifi::{
    ble::controller::BleConnector,
    init,
    wifi::{ClientConfiguration, Configuration},
};
use smoltcp::{
    iface::{SocketSet, SocketStorage},
    wire::{DhcpOption, IpAddress},
};

esp_bootloader_esp_idf::esp_app_desc!();

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");

#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // COEX needs more RAM - add some more
    #[cfg(feature = "esp32")]
    {
        esp_alloc::heap_allocator!(#[unsafe(link_section = ".dram2_uninit")] size: 96 * 1024);
        esp_alloc::heap_allocator!(size: 24 * 1024);
    }
    #[cfg(not(feature = "esp32"))]
    {
        esp_alloc::heap_allocator!(#[unsafe(link_section = ".dram2_uninit")] size: 64 * 1024);
        esp_alloc::heap_allocator!(size: 64 * 1024);
    }

    let timg0 = TimerGroup::new(peripherals.TIMG0);

    let mut rng = Rng::new(peripherals.RNG);

    let esp_wifi_ctrl = init(timg0.timer0, rng.clone(), peripherals.RADIO_CLK).unwrap();

    let now = || time::Instant::now().duration_since_epoch().as_millis();

    // initializing Bluetooth first results in a more stable WiFi connection on ESP32
    let connector = BleConnector::new(&esp_wifi_ctrl, peripherals.BT);
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

    let (mut controller, interfaces) =
        esp_wifi::wifi::new(&esp_wifi_ctrl, peripherals.WIFI).unwrap();

    let mut device = interfaces.sta;
    let iface = create_interface(&mut device);

    controller
        .set_power_saving(esp_wifi::config::PowerSaveMode::None)
        .unwrap();

    let mut socket_set_entries: [SocketStorage; 3] = Default::default();
    let mut socket_set = SocketSet::new(&mut socket_set_entries[..]);
    let mut dhcp_socket = smoltcp::socket::dhcpv4::Socket::new();
    // we can set a hostname here (or add other DHCP options)
    dhcp_socket.set_outgoing_options(&[DhcpOption {
        kind: 12,
        data: b"esp-wifi",
    }]);
    socket_set.add(dhcp_socket);

    let stack = Stack::new(iface, device, socket_set, now, rng.random());

    let client_config = Configuration::Client(ClientConfiguration {
        ssid: SSID.into(),
        password: PASSWORD.into(),
        ..Default::default()
    });
    let res = controller.set_configuration(&client_config);
    println!("wifi_set_configuration returned {:?}", res);

    controller.start().unwrap();
    println!("is wifi started: {:?}", controller.is_started());
    println!("{:?}", controller.capabilities());
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

    let mut rx_buffer = [0u8; 128];
    let mut tx_buffer = [0u8; 128];
    let mut socket = stack.get_socket(&mut rx_buffer, &mut tx_buffer);

    loop {
        println!("Making HTTP request");
        socket.work();

        socket
            .open(IpAddress::Ipv4(Ipv4Addr::new(142, 250, 185, 115)), 80)
            .unwrap();

        socket
            .write(b"GET / HTTP/1.0\r\nHost: www.mobile-j.de\r\n\r\n")
            .unwrap();
        socket.flush().unwrap();

        let deadline = time::Instant::now() + Duration::from_secs(20);
        let mut buffer = [0u8; 128];
        while let Ok(len) = socket.read(&mut buffer) {
            let to_print = unsafe { core::str::from_utf8_unchecked(&buffer[..len]) };
            print!("{}", to_print);

            if time::Instant::now() > deadline {
                println!("Timeout");
                break;
            }
        }
        println!();

        socket.disconnect();

        let deadline = time::Instant::now() + Duration::from_secs(5);
        while time::Instant::now() < deadline {
            socket.work();
        }
    }
}

// some smoltcp boilerplate
fn timestamp() -> smoltcp::time::Instant {
    smoltcp::time::Instant::from_micros(
        esp_hal::time::Instant::now()
            .duration_since_epoch()
            .as_micros() as i64,
    )
}

pub fn create_interface(device: &mut esp_wifi::wifi::WifiDevice) -> smoltcp::iface::Interface {
    // users could create multiple instances but since they only have one WifiDevice
    // they probably can't do anything bad with that
    smoltcp::iface::Interface::new(
        smoltcp::iface::Config::new(smoltcp::wire::HardwareAddress::Ethernet(
            smoltcp::wire::EthernetAddress::from_bytes(&device.mac_address()),
        )),
        device,
        timestamp(),
    )
}
