//! Ethernet DHCP Example
//!
//! Demonstrates RMII Ethernet: initialises the Ethernet peripheral, waits for a DHCP-assigned IP
//! address, then periodically issues an HTTP GET request to httpbin.org and prints the response
//! body.
//!
//! The example is configured for ESP32-Ethernet-Kit v1.2 and ESP32-P4-Function EV Board.
//!
//! The dev kits use an IP101GRI PHY, which is compatible with `GenericPhy`, but this example
//! showcases a more efficient wrapper using embassy-time.
//!
//! # Board pin mapping
//!
//! | Signal       | ESP32-Ethernet-Kit v1.2 | ESP32-P4-Function EV Board |
//! |--------------|-------------------------|----------------------------|
//! | REF_CLK (in) |                    0    |                       50   |
//! | MDC          |                    23   |                       31   |
//! | MDIO         |                    18   |                       52   |
//! | RXD0         |                    25   |                       29   |
//! | RXD1         |                    26   |                       30   |
//! | RX_DV / CRS  |                    27   |                       28   |
//! | TXD0         |                    19   |                       34   |
//! | TXD1         |                    22   |                       35   |
//! | TX_EN        |                    21   |                       49   |
//! | PHY Reset    |                    5    |                       51   |
//! | PHY address  |                    1    |                        1   |

#![no_std]
#![no_main]

use core::task::{Context, Poll};

use embassy_executor::Spawner;
use embassy_net::{
    Runner,
    StackResources,
    dns::DnsSocket,
    tcp::client::{TcpClient, TcpClientState},
};
use embassy_time::{Duration, Timer};
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    ethernet::{
        Ethernet,
        EthernetDmaStorage,
        RmiiPinBundle,
        clock::ExternalRefClock,
        mac::{Duplex, LinkState, Speed},
        phy::{MdioBus, Phy, PhyError, generic::GenericPhy},
    },
    gpio::{Level, Output, OutputConfig},
    interrupt::software::SoftwareInterruptControl,
    rng::Rng,
    timer::timg::TimerGroup,
};
use esp_println::println;
use futures_util::FutureExt;
use reqwless::{
    client::HttpClient,
    request::{Method, RequestBuilder},
};
use static_cell::{ConstStaticCell, StaticCell};

esp_bootloader_esp_idf::esp_app_desc!();

/// MAC address — change to a locally-administered address unique to your board.
const MAC_ADDR: [u8; 6] = [0x02, 0x00, 0x00, 0xAB, 0xCD, 0xEF];

/// DMA storage — must outlive `eth`, so we keep it as a `'static`.
static STORAGE: ConstStaticCell<EthernetDmaStorage<10, 10>> =
    ConstStaticCell::new(EthernetDmaStorage::new());

static STACK_RESOURCES: StaticCell<StackResources<3>> = StaticCell::new();
static TCP_CLIENT_STATE: ConstStaticCell<TcpClientState<1, 1500, 1500>> =
    ConstStaticCell::new(TcpClientState::new());

type EthDriver = Ethernet<'static, esp_hal::Async, ExamplePhy>;

/// A custom PHY implementation that wraps [`GenericPhy`], and uses embassy-time
/// to implement polling instead of the busy looping done by GenericPhy.
struct ExamplePhy {
    phy: GenericPhy,
    timer: Timer,
    cached_link_state: LinkState,
}

impl ExamplePhy {
    fn new_auto() -> Self {
        Self {
            phy: GenericPhy::new_auto(),
            timer: Timer::after_ticks(0),
            cached_link_state: LinkState {
                up: false,
                speed: Speed::_100M,
                duplex: Duplex::Full,
            },
        }
    }
}

impl Phy for ExamplePhy {
    fn address(&self) -> u8 {
        self.phy.address()
    }

    fn init<M: MdioBus>(&mut self, mdio: &mut M) -> Result<(), PhyError> {
        self.phy.init(mdio)
    }

    fn poll_link<M: MdioBus>(&mut self, mdio: &mut M, cx: Option<&mut Context<'_>>) -> LinkState {
        if let Some(cx) = cx {
            if matches!(self.timer.poll_unpin(cx), Poll::Pending) {
                return self.cached_link_state;
            }

            // Timer done, re-arm it
            self.timer = Timer::after(Duration::from_millis(500));

            // Ensure the next wakeup is scheduled
            let _ = self.timer.poll_unpin(cx);
        }

        // Poll the PHY for link state. Do not pass `cx`, we're handling the scheduling.
        self.cached_link_state = self.phy.poll_link(mdio, None);

        // Uncomment to observe how often the link state is polled
        // println!("Polled link state: {:?}", self.cached_link_state);

        self.cached_link_state
    }
}

#[esp_rtos::main]
async fn main(spawner: Spawner) {
    esp_println::logger::init_logger_from_env();

    esp_alloc::heap_allocator!(size: 72 * 1024);

    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    // ── PHY reset ─────────────────────────────────────────────────────────────
    // Assert reset for at least 100 ms, then release and wait ≥ 300 ms for the PHY to stabilise.
    #[cfg(feature = "esp32")]
    let mut phy_reset = Output::new(peripherals.GPIO5, Level::Low, OutputConfig::default());

    #[cfg(feature = "esp32p4")]
    let mut phy_reset = Output::new(peripherals.GPIO51, Level::Low, OutputConfig::default());

    Timer::after(Duration::from_millis(100)).await;
    phy_reset.set_high();
    Timer::after(Duration::from_millis(300)).await;

    // ── Ethernet ─────────────────────────────────────────────────────────────

    #[cfg(feature = "esp32")]
    let eth: EthDriver = Ethernet::new(
        peripherals.ETH,
        STORAGE.take(),
        MAC_ADDR,
        ExamplePhy::new_auto(),
        RmiiPinBundle {
            clock: ExternalRefClock::new(peripherals.GPIO0), // REF_CLK from IP101GRI REFCLKO
            rxd0: peripherals.GPIO25,
            rxd1: peripherals.GPIO26,
            rx_dv: peripherals.GPIO27,
            txd0: peripherals.GPIO19,
            txd1: peripherals.GPIO22,
            tx_en: peripherals.GPIO21,
            mdc: peripherals.GPIO23,
            mdio: peripherals.GPIO18,
        },
    )
    .expect("Ethernet init failed")
    .into_async();

    #[cfg(feature = "esp32p4")]
    let eth: EthDriver = Ethernet::new(
        peripherals.ETH,
        STORAGE.take(),
        MAC_ADDR,
        ExamplePhy::new_auto(),
        RmiiPinBundle {
            clock: ExternalRefClock::new(peripherals.GPIO50), // REF_CLK from IP101GRI REFCLKO
            rxd0: peripherals.GPIO29,
            rxd1: peripherals.GPIO30,
            rx_dv: peripherals.GPIO28,
            txd0: peripherals.GPIO34,
            txd1: peripherals.GPIO35,
            tx_en: peripherals.GPIO49,
            mdc: peripherals.GPIO31,
            mdio: peripherals.GPIO52,
        },
    )
    .expect("Ethernet init failed")
    .into_async();

    // ── Network stack ─────────────────────────────────────────────────────────

    let config = embassy_net::Config::dhcpv4(Default::default());

    let rng = Rng::new();
    let seed = (rng.random() as u64) << 32 | rng.random() as u64;

    let (stack, runner) = embassy_net::new(
        eth,
        config,
        STACK_RESOURCES.init(StackResources::<3>::new()),
        seed,
    );

    spawner.spawn(net_task(runner).unwrap());

    // ── Wait for DHCP ─────────────────────────────────────────────────────────

    println!("Waiting for DHCP lease…");
    stack.wait_config_up().await;

    if let Some(cfg) = stack.config_v4() {
        println!("IP:      {}", cfg.address);
        println!("Gateway: {:?}", cfg.gateway);
    }

    // ── Have fun ───────────────────────────────────────────────────────

    let tcp_client = TcpClient::new(stack, TCP_CLIENT_STATE.take());
    let dns_client = DnsSocket::new(stack);

    loop {
        Timer::after(Duration::from_millis(1000)).await;

        let mut client = HttpClient::new(&tcp_client, &dns_client);
        let mut rx_buf = [0u8; 4096];

        let builder = client
            .request(Method::GET, "http://httpbin.org/get?hello=Hello+esp-hal")
            .await
            .unwrap();

        let mut builder = builder.headers(&[("Host", "httpbin.org"), ("Connection", "close")]);

        let response = builder.send(&mut rx_buf).await.unwrap();

        match response.body().read_to_end().await {
            Ok(data) => {
                if let Ok(st) = core::str::from_utf8(data) {
                    println!("Body: {}", st);
                }
            }
            Err(e) => println!("Body error: {:?}", e),
        }
        Timer::after(Duration::from_millis(3000)).await;
    }
}

#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, EthDriver>) {
    runner.run().await
}
