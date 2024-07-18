//! Embassy ESP-NOW Example (Duplex)
//!
//! Asynchronously broadcasts, receives and sends messages via esp-now in multiple embassy tasks
//!
//! Because of the huge task-arena size configured this won't work on ESP32-S2

//% FEATURES: async embassy embassy-generic-timers esp-wifi esp-wifi/async esp-wifi/embassy-net esp-wifi/wifi-default esp-wifi/wifi esp-wifi/utils esp-wifi/esp-now
//% CHIPS: esp32 esp32s3 esp32c2 esp32c3 esp32c6

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};
use embassy_time::{Duration, Ticker};
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    peripherals::Peripherals,
    prelude::*,
    rng::Rng,
    system::SystemControl,
    timer::{ErasedTimer, OneShotTimer, PeriodicTimer},
};
use esp_println::println;
use esp_wifi::{
    esp_now::{EspNowManager, EspNowReceiver, EspNowSender, PeerInfo, BROADCAST_ADDRESS},
    initialize,
    EspWifiInitFor,
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
async fn main(spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();

    let peripherals = Peripherals::take();

    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::max(system.clock_control).freeze();

    let timer = PeriodicTimer::new(
        esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG0, &clocks, None)
            .timer0
            .into(),
    );

    let init = initialize(
        EspWifiInitFor::Wifi,
        timer,
        Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
        &clocks,
    )
    .unwrap();

    let wifi = peripherals.WIFI;
    let esp_now = esp_wifi::esp_now::EspNow::new(&init, wifi).unwrap();
    println!("esp-now version {}", esp_now.get_version().unwrap());

    #[cfg(feature = "esp32")]
    {
        let timg1 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG1, &clocks, None);
        esp_hal_embassy::init(
            &clocks,
            mk_static!(
                [OneShotTimer<ErasedTimer>; 1],
                [OneShotTimer::new(timg1.timer0.into())]
            ),
        );
    }

    #[cfg(not(feature = "esp32"))]
    {
        let systimer = esp_hal::timer::systimer::SystemTimer::new(peripherals.SYSTIMER);
        esp_hal_embassy::init(
            &clocks,
            mk_static!(
                [OneShotTimer<ErasedTimer>; 1],
                [OneShotTimer::new(systimer.alarm0.into())]
            ),
        );
    }

    let (manager, sender, receiver) = esp_now.split();
    let manager = mk_static!(EspNowManager<'static>, manager);
    let sender = mk_static!(
        Mutex::<NoopRawMutex, EspNowSender<'static>>,
        Mutex::<NoopRawMutex, _>::new(sender)
    );

    spawner.spawn(listener(manager, receiver)).ok();
    spawner.spawn(broadcaster(sender)).ok();

    let mut ticker = Ticker::every(Duration::from_millis(500));
    loop {
        ticker.next().await;
        let peer = match manager.fetch_peer(false) {
            Ok(peer) => peer,
            Err(_) => {
                if let Ok(peer) = manager.fetch_peer(true) {
                    peer
                } else {
                    continue;
                }
            }
        };

        println!("Send hello to peer {:?}", peer.peer_address);
        let mut sender = sender.lock().await;
        let status = sender.send_async(&peer.peer_address, b"Hello Peer.").await;
        println!("Send hello status: {:?}", status);
    }
}

#[embassy_executor::task]
async fn broadcaster(sender: &'static Mutex<NoopRawMutex, EspNowSender<'static>>) {
    let mut ticker = Ticker::every(Duration::from_secs(1));
    loop {
        ticker.next().await;

        println!("Send Broadcast...");
        let mut sender = sender.lock().await;
        let status = sender.send_async(&BROADCAST_ADDRESS, b"Hello.").await;
        println!("Send broadcast status: {:?}", status);
    }
}

#[embassy_executor::task]
async fn listener(manager: &'static EspNowManager<'static>, mut receiver: EspNowReceiver<'static>) {
    loop {
        let r = receiver.receive_async().await;
        println!("Received {:?}", r.get_data());
        if r.info.dst_address == BROADCAST_ADDRESS {
            if !manager.peer_exists(&r.info.src_address) {
                manager
                    .add_peer(PeerInfo {
                        peer_address: r.info.src_address,
                        lmk: None,
                        channel: None,
                        encrypt: false,
                    })
                    .unwrap();
                println!("Added peer {:?}", r.info.src_address);
            }
        }
    }
}
