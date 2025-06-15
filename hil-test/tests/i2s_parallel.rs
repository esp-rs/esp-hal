//! I2S parallel interface tests

//% CHIPS: esp32
//% FEATURES: unstable

#![no_std]
#![no_main]

use esp_hal::{
    gpio::NoPin,
    i2s::parallel::{I2sParallel, TxSixteenBits},
    peripherals::{DMA_I2S0, I2S0},
    time::Rate,
};
use hil_test as _;

esp_bootloader_esp_idf::esp_app_desc!();

#[cfg(test)]
#[embedded_test::tests(default_timeout = 3, executor = hil_test::Executor::new())]
mod tests {
    use super::*;

    struct Context {
        dma_channel: DMA_I2S0<'static>,
        i2s: I2S0<'static>,
    }

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(
            esp_hal::Config::default().with_cpu_clock(esp_hal::clock::CpuClock::max()),
        );

        let dma_channel = peripherals.DMA_I2S0;

        Context {
            dma_channel,
            i2s: peripherals.I2S0,
        }
    }

    #[test]
    async fn driver_does_not_hang_when_async(ctx: Context) {
        let pins = TxSixteenBits::new(
            NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin,
            NoPin, NoPin, NoPin, NoPin,
        );
        let i2s = I2sParallel::new(ctx.i2s, ctx.dma_channel, Rate::from_mhz(20), pins, NoPin)
            .into_async();

        // Try sending an empty buffer, as an edge case
        let tx_buf = esp_hal::dma_tx_buffer!(4096).unwrap();
        let mut xfer = i2s
            .send(tx_buf)
            .map_err(|_| "failed to send empty buffer")
            .unwrap();
        xfer.wait_for_done().await.unwrap();

        let (i2s, mut tx_buf) = xfer.wait();

        // Now send some data
        tx_buf.fill(&[0x12; 128]);

        let mut xfer = i2s
            .send(tx_buf)
            .map_err(|_| "failed to send buffer")
            .unwrap();
        xfer.wait_for_done().await.unwrap();
    }
}
