//! ESP-NOW Example
//!
//! Broadcasts, receives and sends messages via esp-now

#![no_std]
#![no_main]

use esp_alloc as _;
use esp_backtrace as _;
#[cfg(target_arch = "riscv32")]
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::{
    clock::CpuClock,
    main,
    time::{self, Duration},
    timer::timg::TimerGroup,
};
use esp_println::println;
use esp_radio::esp_now::{BROADCAST_ADDRESS, PeerInfo};

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 72 * 1024);

    #[cfg(target_arch = "riscv32")]
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(
        timg0.timer0,
        #[cfg(target_arch = "riscv32")]
        sw_int.software_interrupt0,
    );

    let esp_radio_ctrl = esp_radio::init().unwrap();

    let wifi = peripherals.WIFI;
    let (mut controller, interfaces) =
        esp_radio::wifi::new(&esp_radio_ctrl, wifi, Default::default()).unwrap();
    controller.set_mode(esp_radio::wifi::WifiMode::Sta).unwrap();
    controller.start().unwrap();

    let mut esp_now = interfaces.esp_now;

    println!("esp-now version {}", esp_now.version().unwrap());

    esp_now.set_channel(11).unwrap();

    let mut next_send_time = time::Instant::now() + Duration::from_secs(5);
    loop {
        let r = esp_now.receive();
        if let Some(r) = r {
            println!("Received {:?}", r);

            if r.info.dst_address == BROADCAST_ADDRESS {
                if !esp_now.peer_exists(&r.info.src_address) {
                    esp_now
                        .add_peer(PeerInfo {
                            interface: esp_radio::esp_now::EspNowWifiInterface::Sta,
                            peer_address: r.info.src_address,
                            lmk: None,
                            channel: None,
                            encrypt: false,
                        })
                        .unwrap();
                }
                let status = esp_now
                    .send(&r.info.src_address, b"Hello Peer")
                    .unwrap()
                    .wait();
                println!("Send hello to peer status: {:?}", status);
            }
        }

        if time::Instant::now() >= next_send_time {
            next_send_time = time::Instant::now() + Duration::from_secs(5);
            println!("Send");
            let status = esp_now
                .send(&BROADCAST_ADDRESS, b"0123456789")
                .unwrap()
                .wait();
            println!("Send broadcast status: {:?}", status)
        }
    }
}
