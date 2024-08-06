//! Embassy SPI
//!
//! Depending on your target and the board you are using you have to change the
//! pins.
//!
//! Connect MISO and MOSI pins to see the outgoing data is read as incoming
//! data.
//!
//! The following wiring is assumed:
//! SCLK => GPIO0
//! MISO => GPIO2
//! MOSI => GPIO4
//! CS   => GPIO5

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: async embassy embassy-generic-timers

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    dma::*,
    dma_buffers,
    gpio::Io,
    peripherals::Peripherals,
    prelude::*,
    spi::{
        master::{dma::asynch::SpiDmaAsyncBus, prelude::*, Spi},
        SpiMode,
    },
    system::SystemControl,
    timer::{timg::TimerGroup, ErasedTimer, OneShotTimer},
};

// When you are okay with using a nightly compiler it's better to use https://docs.rs/static_cell/2.1.0/static_cell/macro.make_static.html
macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    esp_println::println!("Init!");
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let timer0: ErasedTimer = timg0.timer0.into();
    let timers = [OneShotTimer::new(timer0)];
    let timers = mk_static!([OneShotTimer<ErasedTimer>; 1], timers);
    esp_hal_embassy::init(&clocks, timers);

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    let sclk = io.pins.gpio0;
    let miso = io.pins.gpio2;
    let mosi = io.pins.gpio4;
    let cs = io.pins.gpio5;

    let dma = Dma::new(peripherals.DMA);

    #[cfg(any(feature = "esp32", feature = "esp32s2"))]
    let dma_channel = dma.spi2channel;
    #[cfg(not(any(feature = "esp32", feature = "esp32s2")))]
    let dma_channel = dma.channel0;

    let (tx_buffer, tx_descriptors, rx_buffer, rx_descriptors) = dma_buffers!(32000);
    let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();
    let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();

    let spi = Spi::new(peripherals.SPI2, 100.kHz(), SpiMode::Mode0, &clocks)
        .with_pins(Some(sclk), Some(mosi), Some(miso), Some(cs))
        .with_dma(dma_channel.configure_for_async(false, DmaPriority::Priority0));

    let mut spi_bus = SpiDmaAsyncBus::new(spi, dma_tx_buf, dma_rx_buf);

    let send_buffer = [0, 1, 2, 3, 4, 5, 6, 7];
    loop {
        let mut buffer = [0; 8];
        esp_println::println!("Sending bytes");
        embedded_hal_async::spi::SpiBus::transfer(&mut spi_bus, &mut buffer, &send_buffer)
            .await
            .unwrap();
        esp_println::println!("Bytes received: {:?}", buffer);
        Timer::after(Duration::from_millis(5_000)).await;
    }
}
