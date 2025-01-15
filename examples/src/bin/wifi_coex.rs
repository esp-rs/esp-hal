//! WiFi and BLE COEXistence example
//!
//! - set SSID and PASSWORD env variable
//! - gets an ip address via DHCP
//! - performs an HTTP get request to some "random" server
//! - does BLE advertising (you cannot connect to it - it's just not implemented in the example)
//!
//! Note: On ESP32-C2 and ESP32-C3 you need a wifi-heap size of 70000, on ESP32-C6 you need 80000 and a tx_queue_size of 10
//!

//% FEATURES: esp-wifi esp-wifi/wifi esp-wifi/utils esp-wifi/ble esp-wifi/coex esp-hal/unstable
//% CHIPS: esp32 esp32s3 esp32c2 esp32c3 esp32c6

#![allow(static_mut_refs)]
#![no_std]
#![no_main]

use core::net::Ipv4Addr;

use bleps::{
    ad_structure::{
        create_advertising_data,
        AdStructure,
        BR_EDR_NOT_SUPPORTED,
        LE_GENERAL_DISCOVERABLE,
    },
    att::Uuid,
    Ble,
    HciConnector,
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
    wifi::{utils::create_network_interface, ClientConfiguration, Configuration, WifiStaDevice},
};
use smoltcp::{
    iface::{SocketSet, SocketStorage},
    wire::{DhcpOption, IpAddress},
};

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");

#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    static mut HEAP: core::mem::MaybeUninit<[u8; 72 * 1024]> = core::mem::MaybeUninit::uninit();

    #[link_section = ".dram2_uninit"]
    static mut HEAP2: core::mem::MaybeUninit<[u8; 64 * 1024]> = core::mem::MaybeUninit::uninit();

    unsafe {
        esp_alloc::HEAP.add_region(esp_alloc::HeapRegion::new(
            HEAP.as_mut_ptr() as *mut u8,
            core::mem::size_of_val(&*core::ptr::addr_of!(HEAP)),
            esp_alloc::MemoryCapability::Internal.into(),
        ));

        // COEX needs more RAM - add some more
        esp_alloc::HEAP.add_region(esp_alloc::HeapRegion::new(
            HEAP2.as_mut_ptr() as *mut u8,
            core::mem::size_of_val(&*core::ptr::addr_of!(HEAP2)),
            esp_alloc::MemoryCapability::Internal.into(),
        ));
    }

    let timg0 = TimerGroup::new(peripherals.TIMG0);

    let mut rng = Rng::new(peripherals.RNG);

    let init = init(timg0.timer0, rng.clone(), peripherals.RADIO_CLK).unwrap();

    let mut wifi = peripherals.WIFI;
    let bluetooth = peripherals.BT;

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

    let connector = BleConnector::new(&init, bluetooth);
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

        let deadline = time::now() + Duration::secs(20);
        let mut buffer = [0u8; 128];
        while let Ok(len) = socket.read(&mut buffer) {
            let to_print = unsafe { core::str::from_utf8_unchecked(&buffer[..len]) };
            print!("{}", to_print);

            if time::now() > deadline {
                println!("Timeout");
                break;
            }
        }
        println!();

        socket.disconnect();

        let deadline = time::now() + Duration::secs(5);
        while time::now() < deadline {
            socket.work();
        }
    }
}
