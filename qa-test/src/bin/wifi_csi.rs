//! CSI Example
//!
//! Set SSID and PASSWORD env variable before running this example.

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32s2 esp32s3
//% FEATURES: esp-radio esp-radio/wifi esp-radio/smoltcp esp-radio/log-04 esp-radio/csi
//% FEATURES: esp-radio/unstable esp-hal/unstable

#![no_std]
#![no_main]

extern crate alloc;

use blocking_network_stack::Stack;
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    interrupt::software::SoftwareInterruptControl,
    main,
    rng::Rng,
    time,
    timer::timg::TimerGroup,
};
use esp_println::println;
use esp_radio::wifi::{ModeConfig, csi::CsiConfig, scan::ScanConfig, sta::StationConfig};
use smoltcp::{
    iface::{SocketSet, SocketStorage},
    wire::DhcpOption,
};

esp_bootloader_esp_idf::esp_app_desc!();

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");

#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 72 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    let (mut controller, interfaces) =
        esp_radio::wifi::new(peripherals.WIFI, Default::default()).unwrap();

    let mut device = interfaces.station;
    let iface = create_interface(&mut device);

    let mut socket_set_entries: [SocketStorage; 3] = Default::default();
    let mut socket_set = SocketSet::new(&mut socket_set_entries[..]);
    let mut dhcp_socket = smoltcp::socket::dhcpv4::Socket::new();
    // we can set a hostname here (or add other DHCP options)
    dhcp_socket.set_outgoing_options(&[DhcpOption {
        kind: 12,
        data: b"esp-radio",
    }]);
    socket_set.add(dhcp_socket);

    let rng = Rng::new();
    let now = || time::Instant::now().duration_since_epoch().as_millis();
    let stack = Stack::new(iface, device, socket_set, now, rng.random());

    let station_config = ModeConfig::Station(
        StationConfig::default()
            .with_ssid(SSID.into())
            .with_password(PASSWORD.into()),
    );
    let res = controller.set_config(&station_config);
    println!("wifi_set_configuration returned {:?}", res);

    controller.start().unwrap();
    println!("is wifi started: {:?}", controller.is_started());

    let csi = CsiConfig::default();
    controller
        .set_csi(csi, |data: esp_radio::wifi::csi::WifiCsiInfo| {
            println!("rssi: {:?} rate: {}", data.rssi(), data.rate());
        })
        .unwrap();

    println!("Waiting for CSI data...");
    println!("Start Wifi Scan");
    let scan_config = ScanConfig::default().with_max(10);
    let res = controller.scan_with_config(scan_config).unwrap();
    for ap in res {
        println!("{:?}", ap);
    }

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
    loop {}
}

// some smoltcp boilerplate
fn timestamp() -> smoltcp::time::Instant {
    smoltcp::time::Instant::from_micros(
        esp_hal::time::Instant::now()
            .duration_since_epoch()
            .as_micros() as i64,
    )
}

pub fn create_interface(device: &mut esp_radio::wifi::WifiDevice) -> smoltcp::iface::Interface {
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
