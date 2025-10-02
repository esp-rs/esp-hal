//! Run a test of download, upload and download+upload in a blocking fashion.
//!
//! A prerequisite to running the benchmark examples is to run the benchmark
//! server on your local machine. Simply run the following commands to do so.
//! ```
//! cd extras/bench-server
//! cargo run --release
//! ```
//! Ensure you have set the IP of your local machine in the `HOST_IP` env
//! variable. E.g `HOST_IP="192.168.0.24"` and also set SSID and PASSWORD env
//! variable before running this example.

//% FEATURES: esp-radio esp-radio/wifi esp-radio/smoltcp esp-radio/unstable esp-hal/unstable
//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32s2 esp32s3

#![no_std]
#![no_main]

use core::net::Ipv4Addr;

use blocking_network_stack::Stack;
use embedded_io::*;
use esp_alloc as _;
use esp_backtrace as _;
#[cfg(target_arch = "riscv32")]
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::{
    clock::CpuClock,
    delay::Delay,
    main,
    ram,
    rng::Rng,
    time::{self, Duration},
    timer::timg::TimerGroup,
};
use esp_println::println;
use esp_radio::wifi::{ClientConfig, Config, ScanConfig};
use smoltcp::{
    iface::{SocketSet, SocketStorage},
    wire::{DhcpOption, IpAddress},
};

esp_bootloader_esp_idf::esp_app_desc!();

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");
const HOST_IP: &str = env!("HOST_IP");

const TEST_DURATION: Duration = Duration::from_secs(15);
const RX_BUFFER_SIZE: usize = 16384;
const TX_BUFFER_SIZE: usize = 16384;
const IO_BUFFER_SIZE: usize = 1024;
const DOWNLOAD_PORT: u16 = 4321;
const UPLOAD_PORT: u16 = 4322;
const UPLOAD_DOWNLOAD_PORT: u16 = 4323;

#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 32 * 1024);
    // add some more RAM
    esp_alloc::heap_allocator!(#[ram(reclaimed)] size: 64 * 1024);

    let server_address: Ipv4Addr = HOST_IP.parse().expect("Invalid HOST_IP address");

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    #[cfg(target_arch = "riscv32")]
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(
        timg0.timer0,
        #[cfg(target_arch = "riscv32")]
        sw_int.software_interrupt0,
    );

    let esp_radio_ctrl = esp_radio::init().unwrap();

    let (mut controller, interfaces) =
        esp_radio::wifi::new(&esp_radio_ctrl, peripherals.WIFI, Default::default()).unwrap();

    let mut device = interfaces.sta;
    let iface = create_interface(&mut device);

    controller
        .set_power_saving(esp_radio::wifi::PowerSaveMode::None)
        .unwrap();

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

    let client_config = Config::Client(
        ClientConfig::default()
            .with_ssid(SSID.into())
            .with_password(PASSWORD.into()),
    );
    let res = controller.set_config(&client_config);
    println!("wifi_set_configuration returned {:?}", res);

    controller.start().unwrap();
    println!("is wifi started: {:?}", controller.is_started());

    println!("Start Wifi Scan");
    let scan_config = ScanConfig::default().with_max(10);
    let res = controller.scan_with_config_sync(scan_config).unwrap();
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

    let mut rx_buffer = [0u8; RX_BUFFER_SIZE];
    let mut tx_buffer = [0u8; TX_BUFFER_SIZE];
    let mut socket = stack.get_socket(&mut rx_buffer, &mut tx_buffer);

    let delay = Delay::new();

    loop {
        test_download(server_address, &mut socket);
        delay.delay_millis(3_000u32);
        socket.work();
        test_upload(server_address, &mut socket);
        socket.work();
        delay.delay_millis(3_000u32);
        test_upload_download(server_address, &mut socket);
        socket.work();
        delay.delay_millis(3_000u32);
    }
}

fn test_download<'a, D: smoltcp::phy::Device>(
    server_address: Ipv4Addr,
    socket: &mut blocking_network_stack::Socket<'a, 'a, D>,
) {
    println!("Testing download...");
    socket.work();

    socket
        .open(IpAddress::Ipv4(server_address), DOWNLOAD_PORT)
        .unwrap();

    let mut buf = [0; IO_BUFFER_SIZE];

    let mut total = 0;
    let deadline = time::Instant::now() + TEST_DURATION;
    loop {
        socket.work();
        if let Ok(len) = socket.read(&mut buf) {
            total += len as u64;
        } else {
            break;
        }

        if time::Instant::now() > deadline {
            break;
        }
    }

    let kbps = (total + 512) / 1024 / TEST_DURATION.as_secs();
    println!("download: {} kB/s", kbps);

    socket.disconnect();
}

fn test_upload<'a, D: smoltcp::phy::Device>(
    server_address: Ipv4Addr,
    socket: &mut blocking_network_stack::Socket<'a, 'a, D>,
) {
    println!("Testing upload...");
    socket.work();

    socket
        .open(IpAddress::Ipv4(server_address), UPLOAD_PORT)
        .unwrap();

    let buf = [0; IO_BUFFER_SIZE];

    let mut total = 0;
    let deadline = time::Instant::now() + TEST_DURATION;
    loop {
        socket.work();
        if let Ok(len) = socket.write(&buf) {
            total += len as u64;
        } else {
            break;
        }

        if time::Instant::now() > deadline {
            break;
        }
    }

    let kbps = (total + 512) / 1024 / TEST_DURATION.as_secs();
    println!("upload: {} kB/s", kbps);

    socket.disconnect();
}

fn test_upload_download<'a, D: smoltcp::phy::Device>(
    server_address: Ipv4Addr,
    socket: &mut blocking_network_stack::Socket<'a, 'a, D>,
) {
    println!("Testing upload+download...");
    socket.work();

    socket
        .open(IpAddress::Ipv4(server_address), UPLOAD_DOWNLOAD_PORT)
        .unwrap();

    let tx_buf = [0; IO_BUFFER_SIZE];
    let mut rx_buf = [0; IO_BUFFER_SIZE];

    let mut total = 0;
    let deadline = time::Instant::now() + TEST_DURATION;
    loop {
        socket.work();
        if let Err(_) = socket.write(&tx_buf) {
            break;
        }

        socket.work();

        if let Ok(len) = socket.read(&mut rx_buf) {
            total += len as u64;
        } else {
            break;
        }

        if time::Instant::now() > deadline {
            break;
        }
    }

    let kbps = (total + 512) / 1024 / TEST_DURATION.as_secs();
    println!("upload+download: {} kB/s", kbps);

    socket.disconnect();
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
