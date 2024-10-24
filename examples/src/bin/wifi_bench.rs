//! Run a test of download, upload and download+upload in a blocking fashion.
//!
//! A prerequisite to running the benchmark examples is to run the benchmark server on your local machine. Simply run the following commands to do so.
//! ```
//! cd extras/bench-server
//! cargo run --release
//! ```
//! Ensure you have set the IP of your local machine in the `HOST_IP` env variable. E.g `HOST_IP="192.168.0.24"` and also set SSID and PASSWORD env variable before running this example.
//! When using USB-SERIAL-JTAG you may have to activate the feature `phy-enable-usb` in the esp-wifi crate.

//% FEATURES: esp-wifi esp-wifi/wifi-default esp-wifi/wifi esp-wifi/utils
//% CHIPS: esp32 esp32s2 esp32s3 esp32c2 esp32c3 esp32c6

#![no_std]
#![no_main]

use embedded_io::*;
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    prelude::*,
    rng::Rng,
    time::{self, Duration},
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
        WifiError,
        WifiStaDevice,
    },
    wifi_interface::WifiStack,
};
use smoltcp::{
    iface::SocketStorage,
    wire::{IpAddress, Ipv4Address},
};

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");
const HOST_IP: &str = env!("HOST_IP");

const TEST_DURATION: Duration = Duration::secs(15);
const RX_BUFFER_SIZE: usize = 16384;
const TX_BUFFER_SIZE: usize = 16384;
const IO_BUFFER_SIZE: usize = 1024;
const DOWNLOAD_PORT: u16 = 4321;
const UPLOAD_PORT: u16 = 4322;
const UPLOAD_DOWNLOAD_PORT: u16 = 4323;

#[entry]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init({
        let mut config = esp_hal::Config::default();
        config.cpu_clock = CpuClock::max();
        config
    });

    esp_alloc::heap_allocator!(72 * 1024);

    let server_address: Ipv4Address = HOST_IP.parse().expect("Invalid HOST_IP address");

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

    let mut rx_buffer = [0u8; RX_BUFFER_SIZE];
    let mut tx_buffer = [0u8; TX_BUFFER_SIZE];
    let mut socket = wifi_stack.get_socket(&mut rx_buffer, &mut tx_buffer);

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

fn test_download<'a>(
    server_address: Ipv4Address,
    socket: &mut esp_wifi::wifi_interface::Socket<'a, 'a, WifiStaDevice>,
) {
    println!("Testing download...");
    socket.work();

    socket
        .open(IpAddress::Ipv4(server_address), DOWNLOAD_PORT)
        .unwrap();

    let mut buf = [0; IO_BUFFER_SIZE];

    let mut total = 0;
    let deadline = time::now() + TEST_DURATION;
    loop {
        socket.work();
        if let Ok(len) = socket.read(&mut buf) {
            total += len as u64;
        } else {
            break;
        }

        if time::now() > deadline {
            break;
        }
    }

    let kbps = (total + 512) / 1024 / TEST_DURATION.to_secs();
    println!("download: {} kB/s", kbps);

    socket.disconnect();
}

fn test_upload<'a>(
    server_address: Ipv4Address,
    socket: &mut esp_wifi::wifi_interface::Socket<'a, 'a, WifiStaDevice>,
) {
    println!("Testing upload...");
    socket.work();

    socket
        .open(IpAddress::Ipv4(server_address), UPLOAD_PORT)
        .unwrap();

    let buf = [0; IO_BUFFER_SIZE];

    let mut total = 0;
    let deadline = time::now() + TEST_DURATION;
    loop {
        socket.work();
        if let Ok(len) = socket.write(&buf) {
            total += len as u64;
        } else {
            break;
        }

        if time::now() > deadline {
            break;
        }
    }

    let kbps = (total + 512) / 1024 / TEST_DURATION.to_secs();
    println!("upload: {} kB/s", kbps);

    socket.disconnect();
}

fn test_upload_download<'a>(
    server_address: Ipv4Address,
    socket: &mut esp_wifi::wifi_interface::Socket<'a, 'a, WifiStaDevice>,
) {
    println!("Testing upload+download...");
    socket.work();

    socket
        .open(IpAddress::Ipv4(server_address), UPLOAD_DOWNLOAD_PORT)
        .unwrap();

    let tx_buf = [0; IO_BUFFER_SIZE];
    let mut rx_buf = [0; IO_BUFFER_SIZE];

    let mut total = 0;
    let deadline = time::now() + TEST_DURATION;
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

        if time::now() > deadline {
            break;
        }
    }

    let kbps = (total + 512) / 1024 / TEST_DURATION.to_secs();
    println!("upload+download: {} kB/s", kbps);

    socket.disconnect();
}
