//! embassy serial
//!
//! This is an example of running the embassy executor and asynchronously
//! writing to and reading from uart

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::fmt::Write;

use embassy_executor::Executor;
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, channel::Channel};
use embassy_time::{with_timeout, Duration};
use esp32s3_hal::{
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
use esp_hal_common::uart::config::AtCmdConfig;
use heapless::Vec;
use static_cell::StaticCell;

static CHANNEL: Channel<ThreadModeRawMutex, [u8; 8], 1> = Channel::new();

static EXECUTOR: StaticCell<Executor> = StaticCell::new();

#[entry]
fn main() -> ! {
    esp_println::println!("Init!");
    let peripherals = Peripherals::take();
    let mut system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
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
        esp32s3_hal::systimer::SystemTimer::new(peripherals.SYSTIMER),
    );

    #[cfg(feature = "embassy-time-timg0")]
    embassy::init(&clocks, timer_group0.timer0);

    let mut uart0 = Uart::new(peripherals.UART0, &mut system.peripheral_clock_control);
    let (mut tx, rx) = uart0.split();

    interrupt::enable(Interrupt::UART0, interrupt::Priority::Priority1).unwrap();

    let executor = EXECUTOR.init(Executor::new());
    executor.run(|spawner| {
        spawner.spawn(reader(rx)).ok();
    });

    // Message must be in SRAM
    {
        let mut buf = [0; 23];
        buf.copy_from_slice(b"Type 8 chars to echo!\r\n");

        embedded_io_async::Write::write(&mut tx, &buf)
            .await
            .unwrap();
        embedded_io_async::Write::flush(&mut tx).await.unwrap();
        esp_println::println!("wrote hello in uart!");
    }

    // Continue reading in this main task and write
    // back out the buffer we receive from the read
    // task.
    loop {
        let buf = CHANNEL.receive().await;
        esp_println::println!("writing...");
        embedded_io_async::Write::write(&mut tx, &buf)
            .await
            .unwrap();
        embedded_io_async::Write::flush(&mut tx).await.unwrap();
    }
}

#[embassy_executor::task]
async fn reader(mut rx: UartRx<'static, UART0>) {
    let mut buf = [0; 8];
    loop {
        esp_println::println!("reading...");
        embedded_io_async::Read::read(&mut rx, &mut buf)
            .await
            .unwrap();
        CHANNEL.send(buf).await;
    }
}
