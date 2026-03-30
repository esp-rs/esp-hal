//! CDC-NCM example using embassy and picoserve.
//!
//! This example creates a USB network device with a DHCP server and a simple webserver running on
//! it. Connect your PC to the USB port, and open `http://10.42.0.1` in your browser, to see the
//! "Hello World" output.
//!
//! This example should be built in release mode.
//!
//! The following wiring is assumed:
//! - DP => GPIO20
//! - DM => GPIO19

#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

use core::{net::Ipv4Addr, str::FromStr};

use embassy_executor::{Spawner, task};
use embassy_net::{Ipv4Cidr, Runner, Stack, StackResources, StaticConfigV4};
use embassy_time::{Duration, Timer};
use embassy_usb::{
    Builder,
    UsbDevice,
    class::cdc_ncm::{
        CdcNcmClass,
        State as CdcNcmState,
        embassy_net::{Device, Runner as NcmRunner, State as NetState},
    },
};
use esp_backtrace as _;
use esp_hal::{
    interrupt::software::SoftwareInterruptControl,
    otg_fs::{
        Usb,
        embassy_usb_device::{Config, Driver as UsbDriver},
    },
    timer::timg::TimerGroup,
};
use picoserve::AppBuilder as AppBuilderTrait;
use static_cell::StaticCell;

static WEBSERVER: StaticCell<picoserve::Router<AppRouter>> = StaticCell::new();
static SERVER_CONFIG: picoserve::Config =
    picoserve::Config::const_default().keep_connection_alive();

static CDC_NCM_STATE: StaticCell<CdcNcmState<'_>> = StaticCell::new();
static NET_STATE: StaticCell<NetState<{ ETHERNET_MTU }, 4, 4>> = StaticCell::new();
static RESOURCES: StaticCell<StackResources<4>> = StaticCell::new();

const GW_IP_ADDR_ENV: Option<&'static str> = option_env!("GATEWAY_IP");
const WEB_TASK_POOL_SIZE: usize = 1;
const ETHERNET_MTU: usize = 1514;
const HTTP_PORT: u16 = 80;

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_hal::main]
async fn main(spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    let usb = Usb::new(peripherals.USB0, peripherals.GPIO20, peripherals.GPIO19);

    // Create the USB device driver.
    static EP_OUT_BUFFER: StaticCell<[u8; 1024]> = StaticCell::new();
    let driver = UsbDriver::new(usb, EP_OUT_BUFFER.init([0u8; 1024]), Config::default());

    // Create embassy-usb Config
    let mut config = embassy_usb::Config::new(0x303A, 0x3001);
    config.manufacturer = Some("Espressif");
    config.product = Some("USB-Ethernet example");
    config.serial_number = Some("12345678");

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    static CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();

    let mut usb_builder = Builder::new(
        driver,
        config,
        CONFIG_DESCRIPTOR.init([0; 256]),
        BOS_DESCRIPTOR.init([0; 256]),
        &mut [], // no msos descriptors
        CONTROL_BUF.init([0; 64]),
    );

    // Host's MAC addr. This is the MAC the host "thinks" its USB-to-ethernet adapter has.
    let host_mac_addr = [0x8A, 0x88, 0x88, 0x88, 0x88, 0x88];
    let our_mac_addr = [0xCA, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC];

    // Create classes on the builder.
    let usb_cdc_ncm = CdcNcmClass::new(
        &mut usb_builder,
        CDC_NCM_STATE.init_with(CdcNcmState::new),
        host_mac_addr,
        64,
    );

    let (runner, device) = usb_cdc_ncm.into_embassy_net_device::<{ ETHERNET_MTU }, 4, 4>(
        NET_STATE.init_with(NetState::new),
        our_mac_addr,
    );

    // Run the USB device.
    spawner.spawn(usb_task(usb_builder.build()).unwrap());
    spawner.spawn(usb_ncm_task(runner).unwrap());

    // Init network stack
    let gw_ip_addr_str = GW_IP_ADDR_ENV.unwrap_or("10.42.0.1");
    let gw_ip_addr = Ipv4Addr::from_str(gw_ip_addr_str).expect("failed to parse gateway ip");

    let config = embassy_net::Config::ipv4_static(StaticConfigV4 {
        address: Ipv4Cidr::new(gw_ip_addr, 24),
        gateway: Some(gw_ip_addr),
        dns_servers: Default::default(),
    });
    let (stack, runner) = embassy_net::new(
        device,
        config,
        RESOURCES.init_with(StackResources::new),
        1234,
    );
    spawner.spawn(net_task(runner).unwrap());

    // Start the DHCP server task.
    spawner.spawn(run_dhcp(stack, gw_ip_addr_str).unwrap());

    // Start the web server tasks.
    let webserver = WEBSERVER.init_with(|| AppBuilder.build_app());
    for task_id in 0..WEB_TASK_POOL_SIZE {
        spawner.spawn(web_task(task_id, webserver, stack).unwrap());
    }

    core::future::pending().await
}

#[task]
async fn net_task(mut runner: Runner<'static, Device<'static, ETHERNET_MTU>>) -> ! {
    runner.run().await
}

#[task]
async fn usb_task(mut device: UsbDevice<'static, UsbDriver<'static>>) -> ! {
    device.run().await
}

#[task]
async fn usb_ncm_task(class: NcmRunner<'static, UsbDriver<'static>, ETHERNET_MTU>) -> ! {
    class.run().await;
}

#[task]
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
        .await
        .inspect_err(|e| log::warn!("DHCP server error: {e:?}"));
        Timer::after(Duration::from_millis(500)).await;
    }
}

#[task(pool_size = WEB_TASK_POOL_SIZE)]
async fn web_task(
    task_id: usize,
    app: &'static picoserve::Router<AppRouter>,
    stack: Stack<'static>,
) -> ! {
    let mut tcp_rx_buffer = [0; 1024];
    let mut tcp_tx_buffer = [0; 1024];
    let mut http_buffer = [0; 2048];

    picoserve::Server::new(app, &SERVER_CONFIG, &mut http_buffer)
        .listen_and_serve(
            task_id,
            stack,
            HTTP_PORT,
            &mut tcp_rx_buffer,
            &mut tcp_tx_buffer,
        )
        .await
        .into_never()
}

pub struct AppBuilder;

pub type AppRouter = <AppBuilder as AppBuilderTrait>::PathRouter;

impl AppBuilderTrait for AppBuilder {
    type PathRouter = impl picoserve::routing::PathRouter;

    fn build_app(self) -> picoserve::Router<Self::PathRouter> {
        picoserve::Router::new().route(
            "/",
            picoserve::routing::get(|| async move { "Hello World" }),
        )
    }
}
