#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_executor::Executor;
use embassy_time::{Duration, Timer};

use esp32c3_hal::{
    clock::ClockControl,
    prelude::*,
    timer::TimerGroup,
    Rtc, embassy, pac::Peripherals, IO,
};
use esp_backtrace as _;
use esp_hal_common::{PullDown, Bank0GpioRegisterAccess, InputOutputAnalogPinType, Input, GpioPin};
use static_cell::StaticCell;

use embedded_hal_async::digital::Wait;

#[embassy_executor::task]
async fn ping(mut pin: GpioPin<Input<PullDown>, Bank0GpioRegisterAccess, InputOutputAnalogPinType, 5>) {
    loop {
        esp_println::println!("Waiting...");
        pin.wait_for_rising_edge().await.unwrap();
        esp_println::println!("Ping!");
        Timer::after(Duration::from_millis(100)).await;
    }
}

static EXECUTOR: StaticCell<Executor> = StaticCell::new();

#[riscv_rt::entry]
fn main() -> ! {
    esp_println::println!("Init!");
    let peripherals = Peripherals::take().unwrap();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(peripherals.TIMG1, &clocks);
    let mut wdt1 = timer_group1.wdt;

    // Disable watchdog timers
    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    #[cfg(feature = "embassy-time-systick")]
    embassy::init(&clocks, esp32c3_hal::systimer::SystemTimer::new(peripherals.SYSTIMER));

    #[cfg(feature = "embassy-time-timg0")]
    embassy::init(&clocks, timer_group0.timer0);

     // Set GPIO5 as an output, and set its state high initially.
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let input = io.pins.gpio5.into_pull_down_input();

    esp32c3_hal::interrupt::enable(
        esp32c3_hal::pac::Interrupt::GPIO,
        esp32c3_hal::interrupt::Priority::Priority1,
    )
    .unwrap();

    let executor = EXECUTOR.init(Executor::new());
    executor.run(|spawner| {
        spawner.spawn(ping(input)).ok();
    });
}