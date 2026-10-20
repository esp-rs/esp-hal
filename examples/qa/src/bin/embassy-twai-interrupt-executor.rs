//! TWAI: `transmit_async` from a task on a higher priority executor
//!
//! The transmitting task runs on an `InterruptExecutor` at priority 2, above
//! the TWAI interrupt (esp-hal's default priority 1), so a task the interrupt
//! handler wakes runs before the handler returns. The handler must not undo
//! what that task does meanwhile: when it did, the transmit interrupt the next
//! frame had just enabled was disabled again, and every frame after the first
//! waited for some other wake-up (esp-rs/esp-hal#6421).
//!
//! Self-test mode with TX and RX on the same pin, so nothing needs to be
//! connected: the controller sends without an acknowledgement. The test sends
//! 100 frames back to back and prints PASS when they all go out within
//! 100 ms (about 10 ms of bus time at 1 Mbit/s), FAIL otherwise.
//!
//! The following pins are used:
//! - TX/RX => GPIO2

//% CHIP_FILTER: twai_driver_supported

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Timer, with_timeout};
use esp_backtrace as _;
use esp_hal::{
    Blocking,
    interrupt::Priority,
    timer::timg::TimerGroup,
    twai::{BaudRate, EspTwaiFrame, StandardId, TwaiConfiguration, TwaiMode},
};
use esp_println::println;
use esp_rtos::embassy::InterruptExecutor;
use static_cell::StaticCell;

esp_bootloader_esp_idf::esp_app_desc!();

const FRAMES: usize = 100;
const DEADLINE: Duration = Duration::from_millis(100);

static EXECUTOR: StaticCell<InterruptExecutor<2>> = StaticCell::new();

#[embassy_executor::task]
async fn transmit(config: TwaiConfiguration<'static, Blocking>) {
    let mut twai = config.into_async().start();
    let frame = EspTwaiFrame::new(StandardId::new(0x123).unwrap(), &[1, 2, 3, 4]).unwrap();

    let start = Instant::now();
    let mut sent = 0;
    while sent < FRAMES {
        // A frame whose completion wakes nobody is still picked up when the
        // timeout's own timer wakes the task: a stall shows up as ~50 ms a
        // frame, not as a hang
        match with_timeout(Duration::from_millis(50), twai.transmit_async(&frame)).await {
            Ok(Ok(())) => sent += 1,
            Ok(Err(e)) => {
                println!("FAIL: frame {} not sent: {:?}", sent, e);
                return;
            }
            Err(_) => {
                println!("FAIL: frame {} not sent within 50 ms", sent);
                return;
            }
        }
    }
    let took = start.elapsed();

    if took <= DEADLINE {
        println!("PASS: {} frames in {} ms", sent, took.as_millis());
    } else {
        println!(
            "FAIL: {} frames took {} ms, expected at most {} ms",
            sent,
            took.as_millis(),
            DEADLINE.as_millis()
        );
    }
}

#[esp_hal::main]
async fn main(_spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    let (rx, tx) = unsafe { peripherals.GPIO2.split() };
    let config = TwaiConfiguration::new(
        peripherals.TWAI0,
        rx,
        tx,
        BaudRate::B1000K,
        TwaiMode::SelfTest,
    );

    let executor = EXECUTOR.init(InterruptExecutor::new(peripherals.FROM_CPU_INTR2));
    let spawner = executor.start(Priority::Priority2);
    spawner.spawn(transmit(config).unwrap());

    loop {
        Timer::after(Duration::from_secs(1)).await;
    }
}
