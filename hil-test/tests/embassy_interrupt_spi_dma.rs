//! Reproduction and regression test for a sneaky issue.

//% CHIPS: esp32s3
//% FEATURES: integrated-timers
//% FEATURES: generic-queue

#![no_std]
#![no_main]

use embassy_time::{Duration, Instant, Ticker};
use esp_hal::{
    dma::{Dma, DmaPriority, DmaRxBuf, DmaTxBuf},
    dma_buffers,
    interrupt::{software::SoftwareInterruptControl, Priority},
    peripherals::SPI3,
    prelude::*,
    spi::{
        master::{Spi, SpiDma},
        FullDuplexMode,
        SpiMode,
    },
    timer::{timg::TimerGroup, ErasedTimer},
    Async,
};
use esp_hal_embassy::InterruptExecutor;
use hil_test as _;

cfg_if::cfg_if! {
    if #[cfg(any(
        feature = "esp32",
        feature = "esp32s2",
    ))] {
        use esp_hal::dma::Spi3DmaChannel as DmaChannel1;
    } else {
        use esp_hal::dma::DmaChannel1;
    }
}

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

#[embassy_executor::task]
async fn interrupt_driven_task(spi: SpiDma<'static, SPI3, DmaChannel1, FullDuplexMode, Async>) {
    let mut ticker = Ticker::every(Duration::from_millis(1));

    let (tx_buffer, tx_descriptors, rx_buffer, rx_descriptors) = dma_buffers!(128);
    let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();
    let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();

    let mut spi = spi.with_buffers(dma_tx_buf, dma_rx_buf);

    loop {
        let mut buffer: [u8; 8] = [0; 8];

        spi.transfer_in_place_async(&mut buffer).await.unwrap();

        ticker.next().await;
    }
}

#[cfg(test)]
#[embedded_test::tests(executor = esp_hal_embassy::Executor::new())]
mod test {
    use super::*;

    #[test]
    #[timeout(3)]
    async fn run_interrupt_executor_test() {
        let (peripherals, clocks) = esp_hal::init(esp_hal::Config::default());

        let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks);
        esp_hal_embassy::init(
            &clocks,
            [
                ErasedTimer::from(timg0.timer0),
                ErasedTimer::from(timg0.timer1),
            ],
        );

        let dma = Dma::new(peripherals.DMA);

        cfg_if::cfg_if! {
            if #[cfg(any(feature = "esp32", feature = "esp32s2"))] {
                let dma_channel1 = dma.spi2channel;
                let dma_channel2 = dma.spi3channel;
            } else {
                let dma_channel1 = dma.channel0;
                let dma_channel2 = dma.channel1;
            }
        }

        let (tx_buffer, tx_descriptors, rx_buffer, rx_descriptors) = dma_buffers!(1024);
        let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();
        let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();

        let mut spi = Spi::new(peripherals.SPI2, 100.kHz(), SpiMode::Mode0, &clocks)
            .with_dma(dma_channel1.configure_for_async(false, DmaPriority::Priority0))
            .with_buffers(dma_tx_buf, dma_rx_buf);

        let spi2 = Spi::new(peripherals.SPI3, 100.kHz(), SpiMode::Mode0, &clocks)
            .with_dma(dma_channel2.configure_for_async(false, DmaPriority::Priority1));

        let sw_ints = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);

        let interrupt_executor = mk_static!(
            InterruptExecutor<1>,
            InterruptExecutor::new(sw_ints.software_interrupt1)
        );

        let spawner = interrupt_executor.start(Priority::Priority3);

        spawner.spawn(interrupt_driven_task(spi2)).unwrap();

        let start = Instant::now();
        let mut buffer: [u8; 1024] = [0; 1024];
        loop {
            spi.transfer_in_place_async(&mut buffer).await.unwrap();

            if start.elapsed() > Duration::from_secs(1) {
                break;
            }
        }
    }
}
