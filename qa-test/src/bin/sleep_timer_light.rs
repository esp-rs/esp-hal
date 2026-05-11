//! Light sleep with timer + UART0 wake-up.
//!
//! Sleeps for 2 s on each loop iteration; either the timer or a character
//! arriving on UART0 RX wakes the CPU. Prints wake reason after each wake.

//% CHIPS: esp32p4

#![no_std]
#![no_main]

use core::time::Duration;

use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    main,
    rtc_cntl::{
        Rtc,
        SocResetReason,
        reset_reason,
        sleep::{TimerWakeupSource, Uart0WakeupSource},
        wakeup_cause,
    },
    system::Cpu,
};
use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let delay = Delay::new();
    let mut rtc = Rtc::new(peripherals.LPWR);

    let reason = reset_reason(Cpu::ProCpu).unwrap_or(SocResetReason::ChipPowerOn);
    println!("boot, reset reason: {:?}", reason);

    let timer = TimerWakeupSource::new(Duration::from_secs(2));
    let uart_wake = Uart0WakeupSource::new(3);

    loop {
        println!("sleeping (timer 2s | UART0 RX)");
        delay.delay_millis(20);
        rtc.sleep_light(&[&timer, &uart_wake]);
        println!("woke, cause: {:?}", wakeup_cause());
    }
}
