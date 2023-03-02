#![no_std]
#![no_main]

#[cfg(feature = "esp32")]
use esp32_hal as hal;
#[cfg(feature = "esp32c2")]
use esp32c2_hal as hal;
#[cfg(feature = "esp32c3")]
use esp32c3_hal as hal;
#[cfg(feature = "esp32c6")]
use esp32c6_hal as hal;
#[cfg(feature = "esp32s2")]
use esp32s2_hal as hal;
#[cfg(feature = "esp32s3")]
use esp32s3_hal as hal;

use esp_backtrace as _;
use esp_println::logger::init_logger;
use esp_println::println;
use esp_wifi::esp_now::{PeerInfo, BROADCAST_ADDRESS};
use esp_wifi::{current_millis, initialize};
use hal::clock::{ClockControl, CpuClock};
use hal::Rng;
use hal::{peripherals::Peripherals, prelude::*, Rtc};

#[cfg(any(feature = "esp32c3", feature = "esp32c2", feature = "esp32c6"))]
use hal::system::SystemExt;

#[entry]
fn main() -> ! {
    init_logger(log::LevelFilter::Info);
    esp_wifi::init_heap();

    let peripherals = Peripherals::take();

    #[cfg(not(any(feature = "esp32", feature = "esp32c6")))]
    let system = peripherals.SYSTEM.split();
    #[cfg(feature = "esp32")]
    let system = peripherals.DPORT.split();
    #[cfg(any(feature = "esp32c6"))]
    let system = peripherals.PCR.split();

    #[cfg(feature = "esp32c3")]
    let clocks = ClockControl::configure(system.clock_control, CpuClock::Clock160MHz).freeze();
    #[cfg(feature = "esp32c2")]
    let clocks = ClockControl::configure(system.clock_control, CpuClock::Clock120MHz).freeze();
    #[cfg(feature = "esp32c6")]
    let clocks = ClockControl::configure(system.clock_control, CpuClock::Clock160MHz).freeze();
    #[cfg(any(feature = "esp32", feature = "esp32s3", feature = "esp32s2"))]
    let clocks = ClockControl::configure(system.clock_control, CpuClock::Clock240MHz).freeze();

    #[cfg(not(any(feature = "esp32c6")))]
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);

    #[cfg(any(feature = "esp32c6"))]
    let mut rtc = Rtc::new(peripherals.LP_CLKRST);

    // Disable watchdog timers
    #[cfg(not(any(feature = "esp32", feature = "esp32s2")))]
    rtc.swd.disable();

    rtc.rwdt.disable();

    #[cfg(any(feature = "esp32c3", feature = "esp32c2", feature = "esp32c6"))]
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

    let mut esp_now = esp_wifi::esp_now::esp_now().initialize().unwrap();

    println!("esp-now version {}", esp_now.get_version().unwrap());

    let mut next_send_time = current_millis() + 5 * 1000;
    loop {
        let r = esp_now.receive();
        if let Some(r) = r {
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
        }

        if current_millis() >= next_send_time {
            next_send_time = current_millis() + 5 * 1000;
            println!("Send");
            esp_now.send(&BROADCAST_ADDRESS, b"0123456789").unwrap();
        }
    }
}
