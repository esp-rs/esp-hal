//! Auto light-sleep with esp-rtos.
//!
//! Whenever no task is ready and no [`WakeLock`] is held, the chip enters light
//! sleep until the next scheduled wakeup. `Instant::now()` and `embassy_time`
//! delays stay continuous across sleep cycles.
//!
//! `periodic` wakes once a second, letting the system sleep in between.
//! `wake_lock_demo` shows a region guarded by a manual [`WakeLock`]: while the
//! guard is alive, the idle hook falls back to `WFI` and the chip never sleeps.

//% CHIP_FILTER: sleep_auto_light_sleep

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::{
    gpio::{Event, Input, WaitForOptions},
    interrupt::software::SoftwareInterruptControl,
    peripherals,
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

cfg_select! {
    // any(feature = "esp32", feature = "esp32s2", feature = "esp32s3") => {
    //  peripherals::GPIO0 as BOOT_GPIO;
    // }
    // feature = "esp32c2" => {
    //  peripherals::GPIO8 as BOOT_GPIO;
    // }
    // feature = "esp32c5" => {
    //  peripherals::GPIO28 as BOOT_GPIO;
    // }
    // feature = "esp32p4" => {
    //  peripherals::GPIO35 as BOOT_GPIO;
    // }
    // esp32c3, esp32c6, esp32c61, esp32h2
    _ => {
        use peripherals::GPIO9 as BOOT_GPIO;
    }
}

#[embassy_executor::task]
async fn gpio(boot_btn: BOOT_GPIO<'static>) {
    let mut input = Input::new(boot_btn, Default::default());

    loop {
        input
            .wait_for_with_options(
                Event::LowLevel,
                WaitForOptions::default().with_wake_enable(true),
            )
            .await
            .unwrap();
        esp_println::println!("button low");

        input
            .wait_for_with_options(
                Event::HighLevel,
                WaitForOptions::default().with_wake_enable(true),
            )
            .await
            .unwrap();
        esp_println::println!("button high");
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
    let p = esp_hal::init(esp_hal::Config::default());

    let sw_int = SoftwareInterruptControl::new(p.SW_INTERRUPT);
    let timg0 = TimerGroup::new(p.TIMG0);

    esp_rtos::start_with_idle_hook(
        timg0.timer0,
        sw_int.software_interrupt0,
        esp_rtos::auto_light_sleep(),
    );

    let boot_btn = cfg_select! {
        // any(feature = "esp32", feature = "esp32s2", feature = "esp32s3") => p.GPIO0,
        // feature = "esp32c2" => p.GPIO8,
        // feature = "esp32c5" => p.GPIO28,
        // feature = "esp32p4" => p.GPIO35,
        // esp32c3, esp32c6, esp32c61, esp32h2
        _ => p.GPIO9,
    };

    spawner.spawn(gpio(boot_btn).unwrap());
    spawner.spawn(periodic().unwrap());
    spawner.spawn(wake_lock_demo().unwrap());
}
