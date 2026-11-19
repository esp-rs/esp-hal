//! Embassy ESP-NOW Example
//!
//! Broadcasts, receives and sends messages via esp-now in an async way

//% CHIP_FILTER: wifi_driver_supported

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Ticker};
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{clock::CpuClock, ram, timer::timg::TimerGroup};
use esp_radio::esp_now::{BROADCAST_ADDRESS, PeerInfo};

esp_bootloader_esp_idf::esp_app_desc!();

// Full `.dram2_uninit` region for this chip.
const DRAM2_UNINIT_SIZE: usize = {
    let range: core::ops::Range<usize> = esp_metadata_generated::memory_range!("DRAM2_UNINIT");
    range.end - range.start
};

#[esp_hal::main]
async fn main(_spawner: Spawner) -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // Log on UART0's default TX pin.
    let log_tx = cfg_select! {
        feature = "esp32" => peripherals.GPIO1,
        feature = "esp32c2" => peripherals.GPIO20,
        feature = "esp32c3" => peripherals.GPIO21,
        any(feature = "esp32c5", feature = "esp32c61") => peripherals.GPIO11,
        feature = "esp32c6" => peripherals.GPIO16,
        any(feature = "esp32s2", feature = "esp32s3") => peripherals.GPIO43,
        feature = "esp32s31" => peripherals.GPIO58,
    };
    let log_writer = esp_hal::uart::UartTx::new(peripherals.UART0, Default::default())
        .unwrap()
        .with_tx(log_tx);
    static LOG_WRITER: static_cell::StaticCell<esp_hal::uart::UartTx<'static, esp_hal::Blocking>> =
        static_cell::StaticCell::new();
    embedded_log::init(LOG_WRITER.init(log_writer));

    esp_alloc::heap_allocator!(#[ram(reclaimed)] size: DRAM2_UNINIT_SIZE);
    esp_alloc::heap_allocator!(size: 36 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    let wifi = peripherals.WIFI;
    let controller = esp_radio::wifi::WifiController::new(wifi, Default::default()).unwrap();

    let mut esp_now = controller.esp_now();
    esp_now.set_channel(11).unwrap();

    log::info!("esp-now version {}", esp_now.version().unwrap());

    let mut ticker = Ticker::every(Duration::from_secs(5));
    loop {
        let res = select(ticker.next(), async {
            let r = esp_now.receive_async().await;
            log::info!("Received {:?}", r);
            if r.info.dst_address == BROADCAST_ADDRESS {
                if !esp_now.peer_exists(&r.info.src_address) {
                    esp_now
                        .add_peer(PeerInfo {
                            interface: esp_radio::esp_now::EspNowWifiInterface::Station,
                            peer_address: r.info.src_address,
                            lmk: None,
                            channel: None,
                            encrypt: false,
                        })
                        .unwrap();
                }
                let status = esp_now.send_async(&r.info.src_address, b"Hello Peer").await;
                log::info!("Send hello to peer status: {:?}", status);
            }
        })
        .await;

        match res {
            Either::First(_) => {
                log::info!("Send");
                let status = esp_now.send_async(&BROADCAST_ADDRESS, b"0123456789").await;
                log::info!("Send broadcast status: {:?}", status)
            }
            Either::Second(_) => (),
        }
    }
}
