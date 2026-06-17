//! Auto light-sleep with esp-rtos.
//!
//! Whenever no task is ready and no [`WakeLock`] is held, the chip enters light
//! sleep until the next scheduled wakeup. `Instant::now()` and `embassy_time`
//! delays stay continuous across sleep cycles.
//!
//! `periodic` wakes once a second, letting the system sleep in between.
//! `wake_lock_demo` shows a region guarded by a manual [`WakeLock`]: while the
//! guard is alive, the idle hook falls back to `WFI` and the chip never sleeps.

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::{
    interrupt::software::SoftwareInterruptControl,
    rtc_cntl::WakeLock,
    timer::timg::TimerGroup,
};

esp_bootloader_esp_idf::esp_app_desc!();

#[embassy_executor::task]
async fn periodic() {
    loop {
        esp_println::println!("tick");
        Timer::after(Duration::from_millis(1_000)).await;
    }
}

#[embassy_executor::task]
async fn wake_lock_demo() {
    loop {
        Timer::after(Duration::from_secs(5)).await;

        let guard = WakeLock::new();
        esp_println::println!("wake lock held: auto light-sleep disabled for 3s");
        Timer::after(Duration::from_secs(3)).await;
        core::mem::drop(guard);

        esp_println::println!("wake lock released: auto light-sleep re-enabled");
    }
}

#[esp_hal::main]
async fn main(spawner: Spawner) {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let timg0 = TimerGroup::new(peripherals.TIMG0);

    esp_rtos::start_with_idle_hook(
        timg0.timer0,
        sw_int.software_interrupt0,
        esp_rtos::auto_light_sleep(),
    );

    spawner.spawn(periodic().unwrap());
    spawner.spawn(wake_lock_demo().unwrap());
}
