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
    Async,
    delay::Delay,
    dma::{DmaDescriptor, DmaTxStreamBuf},
    gpio::{AnyPin, NoPin, Pin},
    i2s::master::{Channels, Config, DataFormat, I2s, I2sTx},
    peripherals::I2S0,
    time::Rate,
};
use hil_test as _;

cfg_if::cfg_if! {
    if #[cfg(any(esp32, esp32s2))] {
        type DmaChannel0<'d> = esp_hal::peripherals::DMA_I2S0<'d>;
    } else {
        type DmaChannel0<'d> = esp_hal::peripherals::DMA_CH0<'d>;
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
async fn writer(
    tx_buffer: &'static mut [u8],
    tx_descriptors: &'static mut [DmaDescriptor],
    i2s_tx: I2sTx<'static, Async>,
) {
    let mut samples = SampleSource::new();
    for b in tx_buffer.iter_mut() {
        *b = samples.next().unwrap();
    }

    let mut tx_transfer = i2s_tx
        .write(DmaTxStreamBuf::new(tx_descriptors, tx_buffer).unwrap())
        .unwrap();

    loop {
        while tx_transfer.available_bytes() == 0 {
            tx_transfer.wait_for_available().await.unwrap();
        }
        tx_transfer.push_with(|buffer| {
            for b in buffer.iter_mut() {
                *b = samples.next().unwrap();
            }
            buffer.len()
        });
    }
}

fn enable_loopback() {
    let i2s = esp_hal::peripherals::I2S0::regs();
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

#[embedded_test::tests(default_timeout = 3, executor = hil_test::Executor::new())]
mod tests {
    use esp_hal::{dma::DmaRxStreamBuf, dma_buffers_chunk_size};

    use super::*;

    struct Context {
        dout: AnyPin<'static>,
        dma_channel: DmaChannel0<'static>,
        i2s: I2S0<'static>,
    }

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(
            esp_hal::Config::default().with_cpu_clock(esp_hal::clock::CpuClock::max()),
        );

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
        let spawner = unsafe { embassy_executor::Spawner::for_current_executor().await };

        // We need more than 3 descriptors for continuous transfer to work
        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) =
            esp_hal::dma_buffers_chunk_size!(BUFFER_SIZE, BUFFER_SIZE, BUFFER_SIZE / 3);

        let i2s = I2s::new(
            ctx.i2s,
            ctx.dma_channel,
            Config::new_tdm_philips()
                .with_sample_rate(Rate::from_hz(16000))
                .with_data_format(DataFormat::Data16Channel16)
                .with_channels(Channels::STEREO),
        )
        .unwrap()
        .into_async();

        let (din, dout) = unsafe { ctx.dout.split() };

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

        let mut rx_transfer = i2s_rx
            .read(
                DmaRxStreamBuf::new(rx_descriptors, rx_buffer).unwrap(),
                BUFFER_SIZE,
            )
            .unwrap();
        spawner.must_spawn(writer(tx_buffer, tx_descriptors, i2s_tx));

        let mut sample_idx = 0;
        let mut samples = SampleSource::new();
        for _ in 0..30 {
            while rx_transfer.available_bytes() == 0 {
                rx_transfer.wait_for_available().await.unwrap();
            }
            let data = rx_transfer.peek();
            let len = data.len();
            for &b in data {
                let expected = samples.next().unwrap();
                assert_eq!(
                    b, expected,
                    "Sample #{} does not match ({} != {})",
                    sample_idx, b, expected
                );
                sample_idx += 1;
            }
            rx_transfer.consume(len);
        }
    }

    #[test]
    fn test_i2s_loopback(ctx: Context) {
        // NOTE: 32000 bits of buffer maybe too large, but for some reason it fails with buffer
        // size 16000 as it seems DMA can be quick enough to run out of descriptors in that
        // case.
        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) =
            dma_buffers_chunk_size!(32000, 32000, 4000);

        let i2s = I2s::new(
            ctx.i2s,
            ctx.dma_channel,
            Config::new_tdm_philips()
                .with_sample_rate(Rate::from_hz(16000))
                .with_data_format(DataFormat::Data16Channel16)
                .with_channels(Channels::STEREO),
        )
        .unwrap();

        let (din, dout) = unsafe { ctx.dout.split() };

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

        let mut samples = SampleSource::new();
        for b in tx_buffer.iter_mut() {
            *b = samples.next().unwrap();
        }

        let mut rcv = [0u8; 11000];
        let mut filler = [0x1u8; 12000];

        let mut rx_transfer = i2s_rx
            .read(
                DmaRxStreamBuf::new(rx_descriptors, rx_buffer).unwrap(),
                4000,
            )
            .unwrap();
        // trying to peek data before calling `available` should just do nothing
        assert_eq!(0, rx_transfer.peek().len());

        // no data available yet
        assert_eq!(0, rx_transfer.available_bytes());

        let mut tx_transfer = i2s_tx
            .write(DmaTxStreamBuf::new(tx_descriptors, tx_buffer).unwrap())
            .unwrap();

        let mut iteration = 0;
        let mut sample_idx = 0;
        let mut check_samples = SampleSource::new();
        loop {
            let tx_avail = tx_transfer.available_bytes();

            // make sure there are more than one descriptor buffers ready to push
            if tx_avail > 5000 {
                for b in &mut filler[0..tx_avail].iter_mut() {
                    *b = samples.next().unwrap();
                }
                tx_transfer.push(&filler[0..tx_avail]);
            }

            // test calling available multiple times doesn't break anything
            rx_transfer.available_bytes();
            rx_transfer.available_bytes();
            rx_transfer.available_bytes();
            rx_transfer.available_bytes();
            rx_transfer.available_bytes();
            rx_transfer.available_bytes();
            let rx_avail = rx_transfer.available_bytes();

            // make sure there are more than one descriptor buffers ready to pop
            if rx_avail > 0 {
                rcv.fill(0xff);
                let data = rx_transfer.peek();
                let len = data.len();
                assert!(len > 0);

                for &b in data {
                    let expected = check_samples.next().unwrap();
                    assert_eq!(
                        b, expected,
                        "Sample #{} does not match ({} != {})",
                        sample_idx, b, expected
                    );
                    sample_idx += 1;
                }

                rx_transfer.consume(len);

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
        let (_, _, tx_buffer, tx_descriptors) = dma_buffers!(0, 16000);

        let i2s = I2s::new(
            ctx.i2s,
            ctx.dma_channel,
            Config::new_tdm_philips()
                .with_sample_rate(Rate::from_hz(16000))
                .with_data_format(DataFormat::Data16Channel16)
                .with_channels(Channels::STEREO),
        )
        .unwrap();

        let mut i2s_tx = i2s
            .i2s_tx
            .with_bclk(NoPin)
            .with_ws(NoPin)
            .with_dout(ctx.dout)
            .build(tx_descriptors);

        let mut tx_transfer = i2s_tx.write_dma_circular(tx_buffer).unwrap();

        let delay = esp_hal::delay::Delay::new();
        delay.delay_millis(300);

        assert!(matches!(tx_transfer.push(&[0; 128]), Err(_)));
    }

    #[test]
    #[timeout(1)]
    fn test_i2s_read_too_late(ctx: Context) {
        let (rx_buffer, rx_descriptors, _, _) = dma_buffers!(16000, 0);

        let i2s = I2s::new(
            ctx.i2s,
            ctx.dma_channel,
            Config::new_tdm_philips()
                .with_sample_rate(Rate::from_hz(16000))
                .with_data_format(DataFormat::Data16Channel16)
                .with_channels(Channels::STEREO),
        )
        .unwrap();

        let mut i2s_rx = i2s
            .i2s_rx
            .with_bclk(NoPin)
            .with_ws(NoPin)
            .with_din(ctx.dout) // not a typo
            .build(rx_descriptors);

        let mut buffer = [0u8; 1024];
        let mut rx_transfer = i2s_rx.read_dma_circular(rx_buffer).unwrap();

        let delay = esp_hal::delay::Delay::new();
        delay.delay_millis(300);

        assert!(matches!(rx_transfer.pop(&mut buffer), Err(_)));
    }

    #[test]
    #[cfg(not(esp32s2))]
    fn test_i2s_rx_half_sample_bits_regression(ctx: Context) {
        // Regression test for rx_half_sample_bits configuration bug.
        // Validates that TX and RX half_sample_bits registers are configured identically.
        let _i2s = I2s::new(
            ctx.i2s,
            ctx.dma_channel,
            Config::new_tdm_philips()
                .with_sample_rate(Rate::from_hz(16000))
                .with_data_format(DataFormat::Data16Channel16)
                .with_channels(Channels::STEREO),
        )
        .unwrap();

        // Access registers directly to read back the configured values
        let regs = esp_hal::peripherals::I2S0::regs();
        let tx_conf1 = regs.tx_conf1().read();
        let rx_conf1 = regs.rx_conf1().read();

        let tx_half_sample_bits = tx_conf1.tx_half_sample_bits().bits();
        let rx_half_sample_bits = rx_conf1.rx_half_sample_bits().bits();

        // These MUST be identical - if they differ, the rx_half_sample_bits bug is present
        assert_eq!(
            tx_half_sample_bits, rx_half_sample_bits,
            "16-bit Stereo: half_sample_bits mismatch TX={}, RX={} (should be identical for proper timing)",
            tx_half_sample_bits, rx_half_sample_bits
        );

        // For Data16Channel16 + STEREO: (16 * 2) / 2 - 1 = 15
        let expected_value = 15u8;
        assert_eq!(
            tx_half_sample_bits, expected_value,
            "16-bit Stereo: TX half_sample_bits={}, expected {}",
            tx_half_sample_bits, expected_value
        );
        assert_eq!(
            rx_half_sample_bits, expected_value,
            "16-bit Stereo: RX half_sample_bits={}, expected {}",
            rx_half_sample_bits, expected_value
        );
    }
}
