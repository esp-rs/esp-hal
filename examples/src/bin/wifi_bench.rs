//! Run a test of download, upload and download+upload in a blocking fashion.
//!
//! A prerequisite to running the benchmark examples is to run the benchmark server on your local machine. Simply run the following commands to do so.
//! ```
//! cd extras/bench-server
//! cargo run --release
//! ```
//! Ensure you have set the IP of your local machine in the `HOST_IP` env variable. E.g `HOST_IP="192.168.0.24"` and also set SSID and PASSWORD env variable before running this example.

//% FEATURES: esp-wifi esp-wifi/wifi-default esp-wifi/wifi esp-wifi/utils
//% CHIPS: esp32 esp32s2 esp32s3 esp32c2 esp32c3 esp32c6

#![no_std]
#![no_main]

use embedded_io::*;
use esp_backtrace as _;
use esp_hal::{delay::Delay, prelude::*, rng::Rng, timer::timg::TimerGroup};
use esp_println::println;
use esp_wifi::{
    current_millis,
    initialize,
    wifi::{
        utils::create_network_interface,
        AccessPointInfo,
        ClientConfiguration,
        Configuration,
        WifiError,
        WifiStaDevice,
    },
    wifi_interface::WifiStack,
    EspWifiInitFor,
};
use smoltcp::{
    iface::SocketStorage,
    wire::{IpAddress, Ipv4Address},
};

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");
const HOST_IP: &str = env!("HOST_IP");

const TEST_DURATION: usize = 15;
const RX_BUFFER_SIZE: usize = 16384;
const TX_BUFFER_SIZE: usize = 16384;
const IO_BUFFER_SIZE: usize = 1024;
const DOWNLOAD_PORT: u16 = 4321;
const UPLOAD_PORT: u16 = 4322;
const UPLOAD_DOWNLOAD_PORT: u16 = 4323;

#[entry]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let (peripherals, clocks) = esp_hal::init({
        let mut config = Config::default();
        config.cpu_clock = CpuClock::max();
        config
    });

    let server_address: Ipv4Address = HOST_IP.parse().expect("Invalid HOST_IP address");

    let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks);

    let init = initialize(
        EspWifiInitFor::Wifi,
        timg0.timer0,
        Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
        &clocks,
    )
    .unwrap();

    let wifi = peripherals.WIFI;
    let mut socket_set_entries: [SocketStorage; 3] = Default::default();
    let (iface, device, mut controller, sockets) =
        create_network_interface(&init, wifi, WifiStaDevice, &mut socket_set_entries).unwrap();
    let wifi_stack = WifiStack::new(iface, device, sockets, current_millis);

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
        let res = controller.is_connected();
        match res {
            Ok(connected) => {
                if connected {
                    break;
                }
            }
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

    let delay = Delay::new(&clocks);

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
    let wait_end = current_millis() + (TEST_DURATION as u64 * 1000);
    loop {
        socket.work();
        if let Ok(len) = socket.read(&mut buf) {
            total += len;
        } else {
            break;
        }

        if current_millis() > wait_end {
            break;
        }
    }

    let kbps = (total + 512) / 1024 / TEST_DURATION;
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
    let wait_end = current_millis() + (TEST_DURATION as u64 * 1000);
    loop {
        socket.work();
        if let Ok(len) = socket.write(&buf) {
            total += len;
        } else {
            break;
        }

        if current_millis() > wait_end {
            break;
        }
    }

    let kbps = (total + 512) / 1024 / TEST_DURATION;
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
    let wait_end = current_millis() + (TEST_DURATION as u64 * 1000);
    loop {
        socket.work();
        if let Err(_) = socket.write(&tx_buf) {
            break;
        }

        socket.work();

        if let Ok(len) = socket.read(&mut rx_buf) {
            total += len;
        } else {
            break;
        }

        if current_millis() > wait_end {
            break;
        }
    }

    let kbps = (total + 512) / 1024 / TEST_DURATION;
    println!("upload+download: {} kB/s", kbps);

    socket.disconnect();
}
