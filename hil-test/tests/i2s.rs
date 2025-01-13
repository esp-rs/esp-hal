//! I2S Loopback Test
//!
//! This test uses I2S TX to transmit known data to I2S RX (forced to slave mode
//! with loopback mode enabled).

//% CHIPS: esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: unstable
// FIXME: re-enable on ESP32 when it no longer fails spuriously

#![no_std]
#![no_main]

use esp_hal::{
    delay::Delay,
    dma_buffers,
    gpio::{AnyPin, NoPin, Pin},
    i2s::master::{DataFormat, I2s, I2sTx, Standard},
    peripherals::I2S0,
    time::RateExtU32,
    Async,
};
use hil_test as _;

cfg_if::cfg_if! {
    if #[cfg(any(esp32, esp32s2))] {
        type DmaChannel0 = esp_hal::dma::I2s0DmaChannel;
    } else {
        type DmaChannel0 = esp_hal::dma::DmaChannel0;
    }
}

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
async fn writer(tx_buffer: &'static mut [u8], i2s_tx: I2sTx<'static, Async>) {
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

fn enable_loopback() {
    unsafe {
        let i2s = esp_hal::peripherals::I2S0::steal();
        cfg_if::cfg_if! {
            if #[cfg(any(esp32, esp32s2))] {
                i2s.conf().modify(|_, w| w.sig_loopback().set_bit());
                i2s.conf().modify(|_, w| w.rx_slave_mod().set_bit());
            } else {
                i2s.tx_conf().modify(|_, w| w.sig_loopback().set_bit());
                i2s.rx_conf().modify(|_, w| w.rx_slave_mod().set_bit());

                i2s.tx_conf().modify(|_, w| w.tx_update().set_bit());
                i2s.rx_conf().modify(|_, w| w.rx_update().set_bit());
            }
        }
    }
}

#[cfg(test)]
#[embedded_test::tests(default_timeout = 3, executor = hil_test::Executor::new())]
mod tests {
    use super::*;

    struct Context {
        dout: AnyPin,
        dma_channel: DmaChannel0,
        i2s: I2S0,
    }

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        cfg_if::cfg_if! {
            if #[cfg(pdma)] {
                let dma_channel = peripherals.DMA_I2S0;
            } else {
                let dma_channel = peripherals.DMA_CH0;
            }
        }

        let (_, dout) = hil_test::common_test_pins!(peripherals);

        Context {
            dout: dout.degrade(),
            dma_channel,
            i2s: peripherals.I2S0,
        }
    }

    #[test]
    async fn test_i2s_loopback_async(ctx: Context) {
        let spawner = embassy_executor::Spawner::for_current_executor().await;

        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) =
            esp_hal::dma_circular_buffers!(BUFFER_SIZE, BUFFER_SIZE);

        let i2s = I2s::new(
            ctx.i2s,
            Standard::Philips,
            DataFormat::Data16Channel16,
            16000.Hz(),
            ctx.dma_channel,
            rx_descriptors,
            tx_descriptors,
        )
        .into_async();

        let (din, dout) = ctx.dout.split();

        let i2s_tx = i2s
            .i2s_tx
            .with_bclk(NoPin)
            .with_ws(NoPin)
            .with_dout(dout)
            .build();

        let i2s_rx = i2s
            .i2s_rx
            .with_bclk(NoPin)
            .with_ws(NoPin)
            .with_din(din)
            .build();

        enable_loopback();

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

    #[test]
    fn test_i2s_loopback(ctx: Context) {
        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(16000, 16000);

        let i2s = I2s::new(
            ctx.i2s,
            Standard::Philips,
            DataFormat::Data16Channel16,
            16000.Hz(),
            ctx.dma_channel,
            rx_descriptors,
            tx_descriptors,
        );

        let (din, dout) = ctx.dout.split();

        let mut i2s_tx = i2s
            .i2s_tx
            .with_bclk(NoPin)
            .with_ws(NoPin)
            .with_dout(dout)
            .build();

        let mut i2s_rx = i2s
            .i2s_rx
            .with_bclk(NoPin)
            .with_ws(NoPin)
            .with_din(din)
            .build();

        enable_loopback();

        let mut samples = SampleSource::new();
        for b in tx_buffer.iter_mut() {
            *b = samples.next().unwrap();
        }

        let mut rcv = [0u8; 11000];
        let mut filler = [0x1u8; 12000];

        let mut rx_transfer = i2s_rx.read_dma_circular(rx_buffer).unwrap();
        // trying to pop data before calling `available` should just do nothing
        assert_eq!(0, rx_transfer.pop(&mut rcv[..100]).unwrap());

        // no data available yet
        assert_eq!(0, rx_transfer.available().unwrap());

        let mut tx_transfer = i2s_tx.write_dma_circular(tx_buffer).unwrap();

        let mut iteration = 0;
        let mut sample_idx = 0;
        let mut check_samples = SampleSource::new();
        loop {
            let tx_avail = tx_transfer.available().unwrap();

            // make sure there are more than one descriptor buffers ready to push
            if tx_avail > 5000 {
                for b in &mut filler[0..tx_avail].iter_mut() {
                    *b = samples.next().unwrap();
                }
                tx_transfer.push(&filler[0..tx_avail]).unwrap();
            }

            // test calling available multiple times doesn't break anything
            rx_transfer.available().unwrap();
            rx_transfer.available().unwrap();
            rx_transfer.available().unwrap();
            rx_transfer.available().unwrap();
            rx_transfer.available().unwrap();
            rx_transfer.available().unwrap();
            let rx_avail = rx_transfer.available().unwrap();

            // make sure there are more than one descriptor buffers ready to pop
            if rx_avail > 0 {
                // trying to pop less data than available is an error
                assert_eq!(
                    Err(esp_hal::dma::DmaError::BufferTooSmall),
                    rx_transfer.pop(&mut rcv[..rx_avail / 2])
                );

                rcv.fill(0xff);
                let len = rx_transfer.pop(&mut rcv).unwrap();
                assert!(len > 0);

                for &b in &rcv[..len] {
                    let expected = check_samples.next().unwrap();
                    assert_eq!(
                        b, expected,
                        "Sample #{} does not match ({} != {})",
                        sample_idx, b, expected
                    );
                    sample_idx += 1;
                }

                iteration += 1;

                if iteration == 1 {
                    // delay to make it likely `available` will need to handle more than one
                    // descriptor next time
                    Delay::new().delay_millis(160);
                }
            }

            if iteration > 30 {
                break;
            }
        }
    }

    #[test]
    fn test_i2s_push_too_late(ctx: Context) {
        let (_, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(0, 16000);

        let i2s = I2s::new(
            ctx.i2s,
            Standard::Philips,
            DataFormat::Data16Channel16,
            16000.Hz(),
            ctx.dma_channel,
            rx_descriptors,
            tx_descriptors,
        );

        let mut i2s_tx = i2s
            .i2s_tx
            .with_bclk(NoPin)
            .with_ws(NoPin)
            .with_dout(ctx.dout)
            .build();

        let mut tx_transfer = i2s_tx.write_dma_circular(tx_buffer).unwrap();

        let delay = esp_hal::delay::Delay::new();
        delay.delay_millis(300);

        assert!(matches!(tx_transfer.push(&[0; 128]), Err(_)));
    }

    #[test]
    #[timeout(1)]
    fn test_i2s_read_too_late(ctx: Context) {
        let (rx_buffer, rx_descriptors, _, tx_descriptors) = dma_buffers!(16000, 0);

        let i2s = I2s::new(
            ctx.i2s,
            Standard::Philips,
            DataFormat::Data16Channel16,
            16000.Hz(),
            ctx.dma_channel,
            rx_descriptors,
            tx_descriptors,
        );

        let mut i2s_rx = i2s
            .i2s_rx
            .with_bclk(NoPin)
            .with_ws(NoPin)
            .with_din(ctx.dout) // not a typo
            .build();

        let mut buffer = [0u8; 1024];
        let mut rx_transfer = i2s_rx.read_dma_circular(rx_buffer).unwrap();

        let delay = esp_hal::delay::Delay::new();
        delay.delay_millis(300);

        assert!(matches!(rx_transfer.pop(&mut buffer), Err(_)));
    }
}
