//! Embassy SPI
//!
//! Folowing pins are used:
//! SCLK    GPIO36
//! MISO    GPIO37
//! MOSI    GPIO35
//! CS      GPIO34
//!
//! Depending on your target and the board you are using you have to change the
//! pins.
//!
//! Connect MISO and MOSI pins to see the outgoing data is read as incoming
//! data.
//!
//! This is an example of running the embassy executor with SPI.

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_executor::Executor;
use embassy_time::{Duration, Timer};
use esp32s2_hal::{
    clock::ClockControl,
    dma::{DmaPriority, *},
    embassy,
    pdma::*,
    peripherals::Peripherals,
    prelude::*,
    spi::{dma::SpiDma, FullDuplexMode, Spi, SpiMode},
    timer::TimerGroup,
    Rtc,
    IO,
};
use esp_backtrace as _;
use static_cell::StaticCell;

macro_rules! singleton {
    ($val:expr) => {{
        type T = impl Sized;
        static STATIC_CELL: StaticCell<T> = StaticCell::new();
        let (x,) = STATIC_CELL.init(($val,));
        x
    }};
}

pub type SpiType<'d> = SpiDma<'d, esp32s2_hal::peripherals::SPI2, Spi2DmaChannel, FullDuplexMode>;

#[embassy_executor::task]
async fn spi_task(spi: &'static mut SpiType<'static>) {
    let send_buffer = [0, 1, 2, 3, 4, 5, 6, 7];
    loop {
        let mut buffer = [0; 8];
        esp_println::println!("Sending bytes");
        embedded_hal_async::spi::SpiBus::transfer(spi, &mut buffer, &send_buffer)
            .await
            .unwrap();
        esp_println::println!("Bytes recieved: {:?}", buffer);
        Timer::after(Duration::from_millis(5_000)).await;
    }
}

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
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    #[cfg(feature = "embassy-time-systick")]
    embassy::init(
        &clocks,
        esp32s2_hal::systimer::SystemTimer::new(peripherals.SYSTIMER),
    );

    #[cfg(feature = "embassy-time-timg0")]
    embassy::init(&clocks, timer_group0.timer0);

    esp32s2_hal::interrupt::enable(
        esp32s2_hal::peripherals::Interrupt::SPI2_DMA,
        esp32s2_hal::interrupt::Priority::Priority1,
    )
    .unwrap();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let sclk = io.pins.gpio36;
    let miso = io.pins.gpio37;
    let mosi = io.pins.gpio35;
    let cs = io.pins.gpio34;

    let dma = Dma::new(system.dma, &mut system.peripheral_clock_control);
    let dma_channel = dma.spi2channel;

    let descriptors = singleton!([0u32; 8 * 3]);
    let rx_descriptors = singleton!([0u32; 8 * 3]);

    let spi = singleton!(Spi::new(
        peripherals.SPI2,
        sclk,
        mosi,
        miso,
        cs,
        100u32.kHz(),
        SpiMode::Mode0,
        &mut system.peripheral_clock_control,
        &clocks,
    )
    .with_dma(dma_channel.configure(
        false,
        descriptors,
        rx_descriptors,
        DmaPriority::Priority0,
    )));

    let executor = EXECUTOR.init(Executor::new());
    executor.run(|spawner| {
        spawner.spawn(spi_task(spi)).ok();
    });
}
