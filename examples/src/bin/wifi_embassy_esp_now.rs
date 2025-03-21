//! Embassy ESP-NOW Example
//!
//! Broadcasts, receives and sends messages via esp-now in an async way
//!
//! Because of the huge task-arena size configured this won't work on ESP32-S2

//% FEATURES: embassy esp-wifi esp-wifi/esp-now esp-hal/unstable
//% CHIPS: esp32 esp32s2 esp32s3 esp32c2 esp32c3 esp32c6

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_time::{Duration, Ticker};
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{clock::CpuClock, rng::Rng, timer::timg::TimerGroup};
use esp_println::println;
use esp_wifi::{
    esp_now::{PeerInfo, BROADCAST_ADDRESS},
    init,
    EspWifiController,
};

// When you are okay with using a nightly compiler it's better to use https://docs.rs/static_cell/2.1.0/static_cell/macro.make_static.html
macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 72 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);

    let esp_wifi_ctrl = &*mk_static!(
        EspWifiController<'static>,
        init(
            timg0.timer0,
            Rng::new(peripherals.RNG),
            peripherals.RADIO_CLK,
        )
        .unwrap()
    );

    let wifi = peripherals.WIFI;
    let (mut controller, interfaces) = esp_wifi::wifi::new(&esp_wifi_ctrl, wifi).unwrap();
    controller.set_mode(esp_wifi::wifi::WifiMode::Sta).unwrap();
    controller.start().unwrap();

    let mut esp_now = interfaces.esp_now;
    esp_now.set_channel(11).unwrap();

    println!("esp-now version {}", esp_now.version().unwrap());

    cfg_if::cfg_if! {
        if #[cfg(feature = "esp32")] {
            let timg1 = TimerGroup::new(peripherals.TIMG1);
            esp_hal_embassy::init(timg1.timer0);
        } else {
            use esp_hal::timer::systimer::SystemTimer;
            let systimer = SystemTimer::new(peripherals.SYSTIMER);
            esp_hal_embassy::init(systimer.alarm0);
        }
    }

    let mut ticker = Ticker::every(Duration::from_secs(5));
    loop {
        let res = select(ticker.next(), async {
            let r = esp_now.receive_async().await;
            println!("Received {:?}", r);
            if r.info.dst_address == BROADCAST_ADDRESS {
                if !esp_now.peer_exists(&r.info.src_address) {
                    esp_now
                        .add_peer(PeerInfo {
                            interface: esp_wifi::esp_now::EspNowWifiInterface::Sta,
                            peer_address: r.info.src_address,
                            lmk: None,
                            channel: None,
                            encrypt: false,
                        })
                        .unwrap();
                }
                let status = esp_now.send_async(&r.info.src_address, b"Hello Peer").await;
                println!("Send hello to peer status: {:?}", status);
            }
        })
        .await;

        match res {
            Either::First(_) => {
                println!("Send");
                let status = esp_now.send_async(&BROADCAST_ADDRESS, b"0123456789").await;
                println!("Send broadcast status: {:?}", status)
            }
            Either::Second(_) => (),
        }
    }
}
