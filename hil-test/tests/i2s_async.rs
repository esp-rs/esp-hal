//! I2S Loopback Test (Async)
//!
//! It's assumed GPIO2 is connected to GPIO3
//!
//! This test uses I2S TX to transmit known data to I2S RX (forced to slave mode
//! with loopback mode enabled). It's using circular DMA mode

//% CHIPS: esp32c3 esp32c6 esp32s3 esp32h2

#![no_std]
#![no_main]

use hil_test as _;
use esp_hal::{
    clock::ClockControl,
    dma::{Dma, DmaChannel0, DmaPriority},
    gpio::Io,
    i2s::{asynch::*, DataFormat, I2s, I2sTx, Standard},
    peripheral::Peripheral,
    peripherals::{Peripherals, I2S0},
    prelude::*,
    system::SystemControl,
    Async,
};

const BUFFER_SIZE: usize = 2000;

#[derive(Clone)]
struct SampleSource {
    i: u8,
}

impl SampleSource {
    // choose values which DON'T restart on every descriptor buffer's start
    const ADD: u8 = 5;
    const CUT_OFF: u8 = 113;

    fn new() -> Self {
        Self { i: 0 }
    }
}

impl Iterator for SampleSource {
    type Item = u8;

    fn next(&mut self) -> Option<Self::Item> {
        let i = self.i;
        self.i = (i + Self::ADD) % Self::CUT_OFF;
        Some(i)
    }
}

#[embassy_executor::task]
async fn writer(tx_buffer: &'static mut [u8], i2s_tx: I2sTx<'static, I2S0, DmaChannel0, Async>) {
    let mut samples = SampleSource::new();
    for b in tx_buffer.iter_mut() {
        *b = samples.next().unwrap();
    }

    let mut tx_transfer = i2s_tx.write_dma_circular_async(tx_buffer).unwrap();

    loop {
        tx_transfer
            .push_with(|buffer| {
                for b in buffer.iter_mut() {
                    *b = samples.next().unwrap();
                }
                buffer.len()
            })
            .await
            .unwrap();
    }
}

#[cfg(test)]
#[embedded_test::tests(executor = esp_hal_embassy::Executor::new())]
mod tests {
    use super::*;

    #[init]
    async fn init() {}

    #[test]
    async fn test_i2s_loopback() {
        let spawner = embassy_executor::Spawner::for_current_executor().await;

        let peripherals = Peripherals::take();
        let system = SystemControl::new(peripherals.SYSTEM);
        let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

        let mut io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

        let dma = Dma::new(peripherals.DMA);
        let dma_channel = dma.channel0;

        let (tx_buffer, tx_descriptors, rx_buffer, rx_descriptors) =
            esp_hal::dma_circular_buffers!(BUFFER_SIZE, BUFFER_SIZE);

        let i2s = I2s::new(
            peripherals.I2S0,
            Standard::Philips,
            DataFormat::Data16Channel16,
            16000.Hz(),
            dma_channel.configure_for_async(false, DmaPriority::Priority0),
            tx_descriptors,
            rx_descriptors,
            &clocks,
        );

        let i2s_tx = i2s
            .i2s_tx
            .with_bclk(unsafe { io.pins.gpio0.clone_unchecked() })
            .with_ws(unsafe { io.pins.gpio1.clone_unchecked() })
            .with_dout(io.pins.gpio2)
            .build();

        let i2s_rx = i2s
            .i2s_rx
            .with_bclk(io.pins.gpio0)
            .with_ws(io.pins.gpio1)
            .with_din(io.pins.gpio3)
            .build();

        // enable loopback testing
        unsafe {
            let i2s = esp_hal::peripherals::I2S0::steal();
            i2s.tx_conf().modify(|_, w| w.sig_loopback().set_bit());

            i2s.rx_conf().modify(|_, w| w.rx_slave_mod().set_bit());

            i2s.tx_conf().modify(|_, w| w.tx_update().clear_bit());
            i2s.tx_conf().modify(|_, w| w.tx_update().set_bit());

            i2s.rx_conf().modify(|_, w| w.rx_update().clear_bit());
            i2s.rx_conf().modify(|_, w| w.rx_update().set_bit());
        }

        let mut rx_transfer = i2s_rx.read_dma_circular_async(rx_buffer).unwrap();
        spawner.must_spawn(writer(tx_buffer, i2s_tx));

        let mut rcv = [0u8; BUFFER_SIZE];
        let mut sample_idx = 0;
        let mut samples = SampleSource::new();
        for _ in 0..30 {
            let len = rx_transfer.pop(&mut rcv).await.unwrap();
            for &b in &rcv[..len] {
                let expected = samples.next().unwrap();
                assert_eq!(
                    b, expected,
                    "Sample #{} does not match ({} != {})",
                    sample_idx, b, expected
                );
                sample_idx += 1;
            }
        }
    }
}
