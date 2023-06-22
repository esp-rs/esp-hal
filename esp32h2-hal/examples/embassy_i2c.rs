//! Embassy I2C
//!
//! Folowing pins are used:
//! SDA    GPIO1
//! SCL    GPIO2
//!
//! Depending on your target and the board you are using you have to change the
//! pins.
//!
//! This is an example of running the embassy executor with IC2. It uses an
//! LIS3DH to get accelerometer data.

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_executor::Executor;
use embassy_time::{Duration, Timer};
use esp32h2_hal::{
    clock::ClockControl,
    embassy,
    i2c::I2C,
    interrupt,
    peripherals::{Interrupt, Peripherals, I2C0},
    prelude::*,
    timer::TimerGroup,
    Rtc,
    IO,
};
use esp_backtrace as _;
use lis3dh_async::{Lis3dh, Range, SlaveAddr};
use static_cell::StaticCell;

#[embassy_executor::task]
async fn run(i2c: I2C<'static, I2C0>) {
    let mut lis3dh = Lis3dh::new_i2c(i2c, SlaveAddr::Alternate).await.unwrap();
    lis3dh.set_range(Range::G8).await.unwrap();

    loop {
        let norm = lis3dh.accel_norm().await.unwrap();
        esp_println::println!("X: {:+.5}  Y: {:+.5}  Z: {:+.5}", norm.x, norm.y, norm.z);

        Timer::after(Duration::from_millis(100)).await;
    }
}

static EXECUTOR: StaticCell<Executor> = StaticCell::new();

#[entry]
fn main() -> ! {
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

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let i2c0 = I2C::new(
        peripherals.I2C0,
        io.pins.gpio1,
        io.pins.gpio2,
        400u32.kHz(),
        &mut system.peripheral_clock_control,
        &clocks,
    );

    interrupt::enable(Interrupt::I2C_EXT0, interrupt::Priority::Priority1).unwrap();

    let executor = EXECUTOR.init(Executor::new());
    executor.run(|spawner| {
        spawner.spawn(run(i2c0)).ok();
    });
}
