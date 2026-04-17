//% CHIPS(has_wifi_ble): esp32c6
//% SUPPORT_FIRMWARE: true
//% FEATURES: unstable esp-alloc embassy
//% FEATURES(has_wifi_ble): esp-radio/wifi esp-radio/ble esp-radio esp-radio/unstable
//% FEATURES(has_wifi_ble): esp-radio/defmt defmt esp-radio/csi
//% ENV: ESP_HAL_CONFIG_STACK_GUARD_OFFSET=4

#![no_std]
#![no_main]

use core::{net::Ipv4Addr, str::FromStr};

use embassy_executor::Spawner;
use embassy_net::{
    IpListenEndpoint,
    Ipv4Cidr,
    Runner,
    Stack,
    StackResources,
    StaticConfigV4,
    tcp::TcpSocket,
};
use embassy_time::{Duration, Timer};
use esp_hal::{
    clock::CpuClock,
    interrupt::software::SoftwareInterruptControl,
    rng::Rng,
    timer::timg::TimerGroup,
};
use esp_radio::wifi::{Config, ControllerConfig, Interface, WifiController, ap::AccessPointConfig};
use hil_test as _;
use hil_test::mk_static;
use semihosting as _;

extern crate alloc;

fn init_heap() {
    cfg_if::cfg_if! {
        if #[cfg(any(esp32, esp32s2, esp32s3, esp32c3, esp32c2, esp32c6))] {
            use esp_hal::ram;
            esp_alloc::heap_allocator!(#[ram(reclaimed)] size: 64 * 1024);
            esp_alloc::heap_allocator!(size: 36 * 1024);
        } else if #[cfg(any(esp32c5, esp32h2))] {
            esp_alloc::heap_allocator!(size: 72 * 1024);
        }
    }
}

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    init_heap();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let p = esp_hal::init(config);

    let timg0 = TimerGroup::new(p.TIMG0);
    let sw_ints = SoftwareInterruptControl::new(p.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_ints.software_interrupt0);

    let access_point_config = Config::AccessPoint(
        AccessPointConfig::default()
            .with_ssid("AP")
            .with_auth_method(esp_radio::wifi::AuthenticationMethod::None),
    );

    let (controller, interfaces) = esp_radio::wifi::new(
        p.WIFI,
        ControllerConfig::default().with_initial_config(access_point_config),
    )
    .unwrap();

    let gw_ip_addr_str = "192.168.2.1";
    let gw_ip_addr = Ipv4Addr::from_str(gw_ip_addr_str).expect("failed to parse gateway ip");

    let net_config = embassy_net::Config::ipv4_static(StaticConfigV4 {
        address: Ipv4Cidr::new(gw_ip_addr, 24),
        gateway: Some(gw_ip_addr),
        dns_servers: Default::default(),
    });

    let rng = Rng::new();
    let seed = (rng.random() as u64) << 32 | rng.random() as u64;

    let (stack, runner) = embassy_net::new(
        interfaces.access_point,
        net_config,
        mk_static!(StackResources<3>, StackResources::<3>::new()),
        seed,
    );

    spawner.spawn(connection(controller).unwrap());
    spawner.spawn(net_task(runner).unwrap());
    spawner.spawn(run_dhcp(stack, gw_ip_addr_str).unwrap());

    stack.wait_config_up().await;
    defmt::info!("[AP-SUPPORT] ready at {}:8080", gw_ip_addr_str);

    let mut rx_buffer = [0u8; 1536];
    let mut tx_buffer = [0u8; 1536];
    let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
    socket.set_timeout(Some(Duration::from_secs(10)));

    loop {
        let _ = socket
            .accept(IpListenEndpoint {
                addr: None,
                port: 8080,
            })
            .await;

        use embedded_io_async::Write;
        let mut buffer = [0u8; 1024];
        let mut pos = 0;
        loop {
            match socket.read(&mut buffer[pos..]).await {
                Ok(0) => break,
                Ok(len) => {
                    pos += len;
                    if buffer[..pos].windows(4).any(|w| w == b"\r\n\r\n") {
                        break;
                    }
                    if pos == buffer.len() {
                        break;
                    }
                }
                Err(_) => break,
            }
        }

        let _ = socket
            .write_all(
                b"HTTP/1.0 200 OK\r\n\r\n\
                <html>\
                    <body>\
                        <h1>Hello Rust! Hello esp-radio!</h1>\
                    </body>\
                </html>\r\n",
            )
            .await;
        let _ = socket.flush().await;

        Timer::after(Duration::from_millis(50)).await;
        socket.close();
        Timer::after(Duration::from_millis(50)).await;
        socket.abort();
    }
}

#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, Interface<'static>>) {
    runner.run().await
}

#[embassy_executor::task]
async fn run_dhcp(stack: Stack<'static>, gw_ip_addr: &'static str) {
    use core::net::{Ipv4Addr, SocketAddrV4};

    use edge_dhcp::{
        io::{self, DEFAULT_SERVER_PORT},
        server::{Server, ServerOptions},
    };
    use edge_nal::UdpBind;
    use edge_nal_embassy::{Udp, UdpBuffers};

    let ip = Ipv4Addr::from_str(gw_ip_addr).expect("dhcp task failed to parse gw ip");
    let mut buf = [0u8; 1500];
    let mut gw_buf = [Ipv4Addr::UNSPECIFIED];

    let buffers = UdpBuffers::<3, 1024, 1024, 10>::new();
    let unbound_socket = Udp::new(stack, &buffers);
    let mut bound_socket = unbound_socket
        .bind(core::net::SocketAddr::V4(SocketAddrV4::new(
            Ipv4Addr::UNSPECIFIED,
            DEFAULT_SERVER_PORT,
        )))
        .await
        .unwrap();

    loop {
        _ = io::server::run(
            &mut Server::<_, 64>::new_with_et(ip),
            &ServerOptions::new(ip, Some(&mut gw_buf)),
            &mut bound_socket,
            &mut buf,
        )
        .await;
        Timer::after(Duration::from_millis(500)).await;
    }
}

#[embassy_executor::task]
async fn connection(controller: WifiController<'static>) {
    loop {
        let _ = controller
            .wait_for_access_point_connected_event_async()
            .await;
    }
}
