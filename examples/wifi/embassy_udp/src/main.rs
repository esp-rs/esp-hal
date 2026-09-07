//! Minimal Embassy UDP example
//!
//! Connects to Wi-Fi, waits for DHCP, sends `banana` to a UDP socket on another
//! machine, and prints any replies.
//!
//! On the receiving Linux PC, run:
//!
//! ```text
//! nc -u -l 5000
//! ```
//!
//! Then set `UDP_DESTINATION` to that PC's IP address and port. For example:
//!
//! ```text
//! SSID='your-network' PASSWORD='your-password' \
//!   UDP_DESTINATION='192.168.1.100:5000' \
//!   cargo xtask run example embassy_udp --chip=esp32c3
//! ```

//% CHIP_FILTER: wifi_driver_supported

#![no_std]
#![no_main]

use core::net::SocketAddrV4;

use embassy_executor::Spawner;
use embassy_net::{
    Runner, StackResources,
    udp::{PacketMetadata, UdpSocket},
};
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{clock::CpuClock, ram, rng::Rng, timer::timg::TimerGroup};
use esp_println::println;
use esp_radio::wifi::{
    AuthenticationMethodConfig, Config, ControllerConfig, Interface, sta::StationConfig,
};
use static_cell::StaticCell;

esp_bootloader_esp_idf::esp_app_desc!();

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");
const UDP_DESTINATION: &str = env!("UDP_DESTINATION");
static RESOURCES: StaticCell<StackResources<3>> = StaticCell::new();

#[esp_hal::main]
async fn main(spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[ram(reclaimed)] size: 64 * 1024);
    esp_alloc::heap_allocator!(size: 36 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, peripherals.FROM_CPU_INTR0);

    let station_config = Config::Station(
        StationConfig::default()
            .with_ssid(SSID.try_into().unwrap())
            .with_authentication(AuthenticationMethodConfig::Wpa2Personal(
                PASSWORD.try_into().unwrap(),
            )),
    );

    let wifi_interface = Interface::station();
    let mut controller = esp_radio::wifi::WifiController::new(
        peripherals.WIFI,
        ControllerConfig::default().with_initial_config(station_config),
    )
    .unwrap();

    let net_config = embassy_net::Config::dhcpv4(Default::default());
    let rng = Rng::new();
    let seed = (rng.random() as u64) << 32 | rng.random() as u64;
    let (stack, runner) = embassy_net::new(
        wifi_interface,
        net_config,
        RESOURCES.init(StackResources::new()),
        seed,
    );

    spawner.spawn(net_task(runner).unwrap());

    println!("Connecting to {SSID}...");
    controller.connect_async().await.unwrap();
    println!("Wi-Fi connected");

    stack.wait_config_up().await;
    println!("DHCP is up: {}", stack.config_v4().unwrap().address);

    let mut rx_meta = [PacketMetadata::EMPTY; 1];
    let mut rx_buffer = [0; 64];
    let mut tx_meta = [PacketMetadata::EMPTY; 1];
    let mut tx_buffer = [0; 64];
    let mut socket = UdpSocket::new(
        stack,
        &mut rx_meta,
        &mut rx_buffer,
        &mut tx_meta,
        &mut tx_buffer,
    );
    socket.bind(0).unwrap();

    let destination: SocketAddrV4 = UDP_DESTINATION.parse().unwrap();
    socket.send_to(b"banana", destination).await.unwrap();
    println!("Sent 'banana' to {destination}");
    println!("Type a reply in netcat and press Enter");

    let mut reply = [0; 64];
    loop {
        let (length, remote) = socket.recv_from(&mut reply).await.unwrap();
        println!("Reply from {remote}: {:?}", &reply[..length]);
    }
}

#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, Interface>) {
    runner.run().await
}
