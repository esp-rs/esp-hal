//! Demonstrates entering/waking from deep sleep with timer, when the WiFi radio is enabled.

//% FEATURES: esp-radio esp-radio/wifi esp-hal/unstable esp-radio/unstable
//% CHIP_FILTER: sleep_driver_supported && wifi_driver_supported

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    interrupt::software::SoftwareInterruptControl,
    main,
    ram,
    rtc_cntl::{
        SocResetReason,
        reset_reason,
        sleep::{LowPower, TimerWakeupSource},
        wakeup_cause,
    },
    system::Cpu,
    time::Duration,
    timer::timg::TimerGroup,
};
use esp_println::println;
use esp_radio::wifi::{
    self,
    ControllerConfig,
    Interface,
    WifiController,
    scan::ScanConfig,
    sta::StationConfig,
};

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
async fn main(_spawner: Spawner) -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    esp_alloc::heap_allocator!(#[ram(reclaimed)] size: 64 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    let delay = Delay::new();
    let mut lpwr = LowPower::new(peripherals.LPWR);

    println!("up and runnning!");
    let reason = reset_reason(Cpu::ProCpu).unwrap_or(SocResetReason::ChipPowerOn);
    println!("reset reason: {:?}", reason);
    let wake_reason = wakeup_cause();
    println!("wake reason: {:?}", wake_reason);

    println!("Starting wifi");

    let station_config = wifi::Config::Station(
        StationConfig::default()
            .with_ssid("FakeNetwork")
            .with_password("password".into()),
    );
    let _wifi_interface = Interface::station();
    let mut controller = WifiController::new(
        peripherals.WIFI,
        ControllerConfig::default().with_initial_config(station_config),
    )
    .unwrap();

    println!("Scan");
    let scan_config = ScanConfig::default().with_max(10);
    let result = controller.scan_async(&scan_config).await.unwrap();
    for ap in result {
        println!("{:?}", ap);
    }

    println!("Wifi configured and started!");

    let timer = TimerWakeupSource::new(Duration::from_secs(5));
    println!("sleeping!");
    delay.delay_millis(100);
    lpwr.sleep_deep(&[&timer]);
}
