//! embassy serial
//!
//! This is an example of running the embassy executor and asynchronously
//! writing to a uart.

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_executor::Executor;
use embassy_time::{Duration, Timer};
use esp32h2_hal::{
    clock::ClockControl,
    embassy,
    interrupt,
    peripherals::{Interrupt, Peripherals, UART0},
    prelude::*,
    timer::TimerGroup,
    Rtc,
    Uart,
};
use esp_backtrace as _;
use static_cell::StaticCell;

#[embassy_executor::task]
async fn run(mut uart: Uart<'static, UART0>) {
    loop {
        embedded_hal_async::serial::Write::write(&mut uart, b"Hello async write!!!\r\n")
            .await
            .unwrap();
        embedded_hal_async::serial::Write::flush(&mut uart)
            .await
            .unwrap();
        Timer::after(Duration::from_millis(1_000)).await;
    }
}

static EXECUTOR: StaticCell<Executor> = StaticCell::new();

#[entry]
fn main() -> ! {
    esp_println::println!("Init!");
    let peripherals = Peripherals::take();
    let mut system = peripherals.PCR.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let mut rtc = Rtc::new(peripherals.LP_CLKRST);
    let timer_group0 = TimerGroup::new(
        peripherals.TIMG0,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(
        peripherals.TIMG1,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt1 = timer_group1.wdt;

    // Disable watchdog timers
    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    #[cfg(feature = "embassy-time-systick")]
    embassy::init(
        &clocks,
        esp32h2_hal::systimer::SystemTimer::new(peripherals.SYSTIMER),
    );

    #[cfg(feature = "embassy-time-timg0")]
    embassy::init(&clocks, timer_group0.timer0);

    let uart0 = Uart::new(peripherals.UART0, &mut system.peripheral_clock_control);

    interrupt::enable(Interrupt::UART0, interrupt::Priority::Priority1).unwrap();

    let executor = EXECUTOR.init(Executor::new());
    executor.run(|spawner| {
        spawner.spawn(run(uart0)).ok();
    });
}
