#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_executor::_export::StaticCell;
use embassy_futures::select::{select, Either};
#[cfg(feature = "esp32")]
use esp32_hal as hal;
#[cfg(feature = "esp32c2")]
use esp32c2_hal as hal;
#[cfg(feature = "esp32c3")]
use esp32c3_hal as hal;
#[cfg(feature = "esp32s2")]
use esp32s2_hal as hal;
#[cfg(feature = "esp32s3")]
use esp32s3_hal as hal;

use embassy_executor::Executor;
use embassy_time::{Duration, Ticker};
use esp_backtrace as _;
use esp_println::logger::init_logger;
use esp_println::println;
use esp_wifi::esp_now::{EspNow, PeerInfo, BROADCAST_ADDRESS};
use esp_wifi::initialize;
use futures_util::StreamExt;
use hal::clock::{ClockControl, CpuClock};
use hal::Rng;
use hal::{embassy, peripherals::Peripherals, prelude::*, timer::TimerGroup, Rtc};

#[cfg(any(feature = "esp32c3", feature = "esp32c2"))]
use hal::system::SystemExt;

#[cfg(any(feature = "esp32c3", feature = "esp32c2"))]
use riscv_rt::entry;
#[cfg(any(feature = "esp32", feature = "esp32s3", feature = "esp32s2"))]
use xtensa_lx_rt::entry;

#[embassy_executor::task]
async fn run(mut esp_now: EspNow) {
    let mut ticker = Ticker::every(Duration::from_secs(5));
    loop {
        let res = select(ticker.next(), async {
            let r = esp_now.receive_async().await;
            println!("Received {:x?}", r);
            if r.info.dst_address == BROADCAST_ADDRESS {
                if !esp_now.peer_exists(&r.info.src_address).unwrap() {
                    esp_now
                        .add_peer(PeerInfo {
                            peer_address: r.info.src_address,
                            lmk: None,
                            channel: None,
                            encrypt: false,
                        })
                        .unwrap();
                }
                esp_now.send(&r.info.src_address, b"Hello Peer").unwrap();
            }
        })
        .await;

        match res {
            Either::First(_) => {
                println!("Send");
                esp_now.send(&BROADCAST_ADDRESS, b"0123456789").unwrap();
            }
            Either::Second(_) => (),
        }
    }
}

static EXECUTOR: StaticCell<Executor> = StaticCell::new();

#[entry]
fn main() -> ! {
    init_logger(log::LevelFilter::Info);
    esp_wifi::init_heap();

    let peripherals = Peripherals::take();

    #[cfg(not(feature = "esp32"))]
    let system = peripherals.SYSTEM.split();
    #[cfg(feature = "esp32")]
    let system = peripherals.DPORT.split();

    #[cfg(feature = "esp32c3")]
    let clocks = ClockControl::configure(system.clock_control, CpuClock::Clock160MHz).freeze();
    #[cfg(feature = "esp32c2")]
    let clocks = ClockControl::configure(system.clock_control, CpuClock::Clock120MHz).freeze();
    #[cfg(any(feature = "esp32", feature = "esp32s3", feature = "esp32s2"))]
    let clocks = ClockControl::configure(system.clock_control, CpuClock::Clock240MHz).freeze();

    let mut rtc = Rtc::new(peripherals.RTC_CNTL);

    // Disable watchdog timers
    #[cfg(not(any(feature = "esp32", feature = "esp32s2")))]
    rtc.swd.disable();

    rtc.rwdt.disable();

    #[cfg(any(feature = "esp32c3", feature = "esp32c2"))]
    {
        use hal::systimer::SystemTimer;
        let syst = SystemTimer::new(peripherals.SYSTIMER);
        initialize(syst.alarm0, Rng::new(peripherals.RNG), &clocks).unwrap();
    }
    #[cfg(any(feature = "esp32", feature = "esp32s3", feature = "esp32s2"))]
    {
        use hal::timer::TimerGroup;
        let timg1 = TimerGroup::new(peripherals.TIMG1, &clocks);
        initialize(timg1.timer0, Rng::new(peripherals.RNG), &clocks).unwrap();
    }

    let esp_now = esp_wifi::esp_now::esp_now().initialize().unwrap();
    println!("esp-now version {}", esp_now.get_version().unwrap());

    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    embassy::init(&clocks, timer_group0.timer0);
    let executor = EXECUTOR.init(Executor::new());
    executor.run(|spawner| {
        spawner.spawn(run(esp_now)).ok();
    });
}
