//! Auto light-sleep with esp-rtos.
//!
//! Whenever no task is ready and no [`WakeLock`] is held, the chip enters light
//! sleep until the next scheduled wakeup. `Instant::now()` and `embassy_time`
//! delays stay continuous across sleep cycles.
//!
//! `periodic` wakes once a second, letting the system sleep in between.
//! `wake_lock_demo` shows a region guarded by a manual [`WakeLock`]: while the
//! guard is alive, the idle hook falls back to `WFI` and the chip never sleeps.
//!
//! Both the `periodic` and `gpio` tasks print `wakeup_cause()`, so you can
//! observe the chip waking from light sleep via the timer (`WakeupSource::Timer`)
//! or via the BOOT button (`WakeupSource::Gpio`).
//!
//! Use the UART port for this example - USB Serial/JTAG will break when the device enters sleep.

//% CHIP_FILTER: sleep_light_sleep

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::{
    gpio::{Event, Input, InputConfig, Pull, WaitForOptions},
    interrupt::software::SoftwareInterruptControl,
    peripherals,
    rtc_cntl::WakeLock,
    system::wakeup_cause,
    time::Instant,
    timer::timg::TimerGroup,
};

esp_bootloader_esp_idf::esp_app_desc!();

#[embassy_executor::task]
async fn periodic() {
    loop {
        esp_println::println!(
            "tick @ {} (wakeup cause: {:?})",
            Instant::now().duration_since_epoch().as_millis(),
            wakeup_cause(),
        );
        Timer::after(Duration::from_millis(1_000)).await;
    }
}

cfg_select! {
    any(feature = "esp32", feature = "esp32s2", feature = "esp32s3") => {
        use peripherals::GPIO0 as BOOT_GPIO;
    }
    feature = "esp32c5" => {
       use peripherals::GPIO28 as BOOT_GPIO;
    }
    feature = "esp32p4" => {
       use peripherals::GPIO35 as BOOT_GPIO;
    }
    // esp32c3, esp32c6, esp32c61, esp32h2
    _ => {
        use peripherals::GPIO9 as BOOT_GPIO;
    }
}

#[embassy_executor::task]
async fn gpio(boot_btn: BOOT_GPIO<'static>) {
    let mut input = Input::new(boot_btn, InputConfig::default().with_pull(Pull::Up));

    loop {
        input
            .wait_for_with_options(
                Event::LowLevel,
                WaitForOptions::default().with_wake_enable(true),
            )
            .await
            .unwrap();
        esp_println::println!("button low (wakeup cause: {:?})", wakeup_cause());

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

#[cfg(feature = "multi_core")]
#[embassy_executor::task]
async fn periodic_core2() {
    loop {
        esp_println::println!(
            "tick on core 2 @ {}",
            Instant::now().duration_since_epoch().as_millis()
        );
        Timer::after(Duration::from_millis(1_500)).await;
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

    let sleep = esp_rtos::sleep::configure(p.LPWR);
    esp_rtos::start_with_idle_hook(
        timg0.timer0,
        sw_int.software_interrupt0,
        sleep.light_sleep_hook,
    );

    let boot_btn = cfg_select! {
        any(feature = "esp32", feature = "esp32s2", feature = "esp32s3") => p.GPIO0,
        feature = "esp32c5" => p.GPIO28,
        feature = "esp32p4" => p.GPIO35,
        // esp32c2, esp32c3, esp32c6, esp32c61, esp32h2
        _ => p.GPIO9,
    };

    spawner.spawn(gpio(boot_btn).unwrap());
    spawner.spawn(periodic().unwrap());
    spawner.spawn(wake_lock_demo().unwrap());

    #[cfg(feature = "multi_core")]
    {
        use esp_hal::system::Stack;
        use esp_rtos::embassy::Executor;
        use static_cell::StaticCell;

        static APP_CORE_STACK: StaticCell<Stack<8192>> = StaticCell::new();
        let app_core_stack = APP_CORE_STACK.init(Stack::new());

        esp_rtos::start_second_core(
            p.CPU_CTRL,
            sw_int.software_interrupt1,
            app_core_stack,
            move || {
                static EXECUTOR: StaticCell<Executor> = StaticCell::new();
                let executor = EXECUTOR.init(Executor::new());
                executor.run(|spawner| {
                    spawner.spawn(periodic_core2().unwrap());
                });
            },
        );
    }
}
